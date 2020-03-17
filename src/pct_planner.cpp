#include <pct/pct_planner.hpp>
#include <pct/ik_routine.hpp>
#include <cmath>

pct_planner::pct_planner()
{
	tol_optns.resize(3);
	tolerances.resize(4);
	constr_optns.resize(4);
	pose_status.resize(4);
	flange_T_ee = KDL::Frame::Identity();
	cart_velc.resize(1);
	cart_velc(0) = 15 / 1000;

	// Declarations for process velocity
	nxt_wp.resize(12);
	curr_w_T_ff = Eigen::Matrix4d::Identity();
	curr_inv_ff_T_ee = Eigen::Matrix4d::Identity();
	nxt_w_T_ff = Eigen::Matrix4d::Identity();
	nxt_inv_ff_T_ee = Eigen::Matrix4d::Identity();

	// Make this false to stop printing
	print_statements = true;
};

pct_planner::~pct_planner() {};

void pct_planner::init_robot(std::string& _rob_urdf, Eigen::Matrix4d& _world_T_base, std::string& _rob_base_link, std::string& _rob_tip_link)
{
	rob_urdf = _rob_urdf;
	world_T_base = KDL::Frame( KDL::Rotation(_world_T_base(0,0),_world_T_base(0,1),_world_T_base(0,2),
                                  _world_T_base(1,0),_world_T_base(1,1),_world_T_base(1,2),
                                  _world_T_base(2,0),_world_T_base(2,1),_world_T_base(2,2)  ) , KDL::Vector(_world_T_base(0,3),_world_T_base(1,3),_world_T_base(2,3)) );

	rob_base_link = _rob_base_link;
	rob_tip_link = _rob_tip_link;
	robot.initialize(rob_urdf, world_T_base,flange_T_ee,rob_base_link,rob_tip_link);
	NrJoints = robot.NrOfJoints;
};

void pct_planner::set_trajOptions(Eigen::Vector3i& _tol_axes, Eigen::VectorXd& _tolerances, Eigen::VectorXi& _constr_optns)
{
	tol_optns = _tol_axes;
	tolerances = _tolerances;
	constr_optns = _constr_optns;

	// Initialize robot velocity limits if velocity constraint needs to be enforced
	// Write a sanity checker to make sure urdf has joint velocities in it
	if (constr_optns(1)==1)
		robot.populate_robot_joint_vel_limits();	
	
	optXtolRel = 1e-4;
	gradH = 1e-8;
	optVardim = NrJoints;
	std::cout<< "NLopt dimensions: " << optVardim << std::endl;
};

void pct_planner::process_velocity(int index,Eigen::MatrixXd& waypoints,Eigen::MatrixXd& _tcp_tf,Eigen::VectorXi& _tcp_id, Eigen::VectorXd& inst_vel)
{
	curr_wp = waypoints.row(index).transpose();
	if (index==waypoints.rows()-1) // If it is the last point
		nxt_wp = curr_wp;
	else
		nxt_wp = waypoints.row(index+1).transpose();

	curr_ff_T_ee = _tcp_tf.block(_tcp_id(index)*4,0,4,4);	// _tcp_id(i)*4 is the starting row number
	invert_matrix(curr_ff_T_ee,curr_inv_ff_T_ee);

	if (index==waypoints.rows()-1) // If it is the last point
		nxt_inv_ff_T_ee = curr_inv_ff_T_ee;
	else
	{
		nxt_ff_T_ee = _tcp_tf.block(_tcp_id(index+1)*4,0,4,4);	// _tcp_id(i)*4 is the starting row number
		invert_matrix(nxt_ff_T_ee,nxt_inv_ff_T_ee);		
	}
	
	// Instantaenous Velocity Vector
	// Get flange wrt world
	tf_wp(curr_wp,curr_inv_ff_T_ee,curr_w_T_ff);
	tf_wp(nxt_wp,nxt_inv_ff_T_ee,nxt_w_T_ff);
	double velc;
	if (cart_velc.rows()==1)
		velc = cart_velc(0);
	else
		velc = cart_velc(index);

	inst_vel.segment(0,3) = nxt_w_T_ff.block(0,3,3,1) - curr_w_T_ff.block(0,3,3,1); // Converting translation
	if (tol_optns(0)==1 && tol_optns(1)==1 && tol_optns(2)==1) // Exact
		inst_vel.segment(3,3) = rot2eul(nxt_w_T_ff.block(0,0,3,3)) - rot2eul(curr_w_T_ff.block(0,0,3,3));

	inst_vel.segment(3,3) << 0,0,0;
	seg_time = inst_vel.segment(0,3).norm() / velc;
	if (index==waypoints.rows()-1)
		inst_vel<< 0,0,0,0,0,0;
	else
	{	
		inst_vel /= seg_time;	// Get the instantaneous velocity vector
		seg_times(index+1) = seg_time*1.9;	// Relaxation factor 1.5
	}
};

void pct_planner::solve_pct(std::vector<double> iterator, Eigen::MatrixXd& _waypoints, Eigen::Matrix4d& _wrld_T_wps, Eigen::MatrixXd& _tcp_tf, Eigen::VectorXi& _tcp_id)
{
	trajectory.resize(_waypoints.rows(),optVardim);

	wrld_T_wps = _wrld_T_wps;
	waypoints = tf_waypoints(_waypoints,wrld_T_wps);

	Eigen::MatrixXd temp = Eigen::MatrixXd::Constant(waypoints.rows(),1,5000);
	seg_times = temp;
	report.setZero(waypoints.rows(),5);
	failures = 0;
	sol_status = true;

	// Initialize numIK object
	numIK ikRoutine( &robot,optXtolRel,gradH,tol_optns,tolerances,constr_optns );
	// Process err func and constr flags. Add segment time and other processors
	// Add Group Indices later
	Eigen::Matrix4d ff_T_ee;
	Eigen::Matrix4d inv_ff_T_ee = Eigen::MatrixXd::Identity(4,4);
	KDL::Frame FF_T_EE;
	Eigen::VectorXd curr_wp;
	std::vector<double> prev_config;
	prev_config.resize(optVardim);
	Eigen::VectorXd inst_vel;
	inst_vel.resize(6); // Cartesian Instantaneous Velocity
	bool frst_pt = true;

	// Loop through waypoints and solve IK for each waypoint
	for (int i=0; i<waypoints.rows();i++)
	{
		if (i>0)
			frst_pt = false;
		// Convert the TCP to a KDL Frame
		ff_T_ee = _tcp_tf.block(_tcp_id(i)*4,0,4,4);	// _tcp_id(i)*4 is the starting row number
		FF_T_EE = KDL::Frame( KDL::Rotation(ff_T_ee(0,0),ff_T_ee(0,1),ff_T_ee(0,2),
                                  ff_T_ee(1,0),ff_T_ee(1,1),ff_T_ee(1,2),
                                  ff_T_ee(2,0),ff_T_ee(2,1),ff_T_ee(2,2)  ) , KDL::Vector(ff_T_ee(0,3),ff_T_ee(1,3),ff_T_ee(2,3)) );

		curr_wp = waypoints.row(i).transpose();

		// Successive Refinement
		// Solve without velocity constraint
		inst_vel<< 0,0,0,0,0,0;
		ikRoutine.solve(iterator, curr_wp, FF_T_EE,frst_pt,prev_config,inst_vel,seg_times(i));

		// Process velocity constraint
		if (constr_optns(1)==1)
		{
			process_velocity(i,waypoints,_tcp_tf,_tcp_id,inst_vel);
			ikRoutine.solve(iterator, curr_wp, FF_T_EE,frst_pt,prev_config,inst_vel,seg_times(i));
		}

		// Store the Solution
		for (int jnt=0; jnt<optVardim; jnt++)
			trajectory(i,jnt) = iterator[jnt];
		// std::cout<< trajectory.row(i) << std::endl;
		// std::cout<< std::endl;
		// std::cout<< ikRoutine.status << std::endl;
		// std::cout<< ikRoutine.f_val << std::endl;
		// Verify IK solutions for each waypoint.
		validate_pose( i, waypoints.row(i).transpose(), iterator );
		if (report(i,0)==1)
			failures += 1;
		if (failures>2)
			sol_status = false;

	}
	// if (!sol_status && print_statements)
		// std::cout<< "SOLUTION FAILED." << std::endl;
	// Each solve instance receives the tool tcp transformation being used, waypoint, 
	// Apply constr successively using flags
};



// Validation Functions
void pct_planner::validate_pose( int index, const Eigen::VectorXd& waypoint, std::vector<double>& iterator )
{
	KDL::Frame FK_tcp;
	KDL::JntArray sol_config;
	sol_config = KDL::JntArray(optVardim);
	for (int jnt=0; jnt<optVardim; jnt++)
    	sol_config(jnt) = iterator[jnt];

	robot.FK_KDL(sol_config, FK_tcp);
	Eigen::VectorXd err_pos(3);
	err_pos << waypoint(0)-FK_tcp(0,3),  waypoint(1)-FK_tcp(1,3),  waypoint(2)-FK_tcp(2,3);
	if (err_pos.norm() > tolerances(0))
	{
		report(index,0) = 1;
		report(index,1) = err_pos.norm() - tolerances(0);
	}

	if (acos( waypoint(3)*FK_tcp(0,0) + waypoint(4)*FK_tcp(1,0) + waypoint(5)*FK_tcp(2,0) ) > tolerances(1) + 1e-4  && tol_optns(0)==1)
	{
		report(index,0) = 1;
		report(index,2) = acos( waypoint(3)*FK_tcp(0,0) + waypoint(4)*FK_tcp(1,0) + waypoint(5)*FK_tcp(2,0) ) - tolerances(1) + 1e-4;
	}
	if (acos( waypoint(6)*FK_tcp(0,1) + waypoint(7)*FK_tcp(1,1) + waypoint(8)*FK_tcp(2,1) ) > tolerances(2) + 1e-4  && tol_optns(1)==1)
	{
		report(index,0) = 1;
		report(index,3) = acos( waypoint(6)*FK_tcp(0,1) + waypoint(7)*FK_tcp(1,1) + waypoint(8)*FK_tcp(2,1) ) - tolerances(2) + 1e-4;
	}
	if (acos( waypoint(9)*FK_tcp(0,2) + waypoint(10)*FK_tcp(1,2) + waypoint(11)*FK_tcp(2,2) ) > tolerances(3) + 1e-4  && tol_optns(2)==1)
	{
		report(index,0) = 1;
		report(index,4) = acos( waypoint(9)*FK_tcp(0,2) + waypoint(10)*FK_tcp(1,2) + waypoint(11)*FK_tcp(2,2) ) - tolerances(3) + 1e-4;
	}
};

void pct_planner::status_report(int type)
{
	int viols = 0;
	double m_pos, m_bx, m_by, m_bz;
	m_pos = 0; m_bx = 0; m_by = 0; m_bz = 0;

	std::vector < std::vector<double> > errors;
	for (int i=0; i<report.rows();i++)
	{
		if (report(i,0)==1)
		{
			viols += 1;
			errors.push_back({(double)i+1,report(i,1),report(i,2) * (180.0/3.1415926),report(i,3) * (180.0/3.1415926),report(i,4) * (180.0/3.1415926)});
			m_pos += report(i,1);
			m_bx += report(i,2) * (180.0/3.1415926);
			m_by += report(i,3) * (180.0/3.1415926);
			m_bz += report(i,4) * (180.0/3.1415926);
		}
	}
	
	if (sol_status && type==2)
		std::cout<< "IK successful" << std::endl;
	else
	{
		if (type==2)
		{
			std::cout<< "Number of violations: " << viols << std::endl;
			std::cout << "Point and it's violations in position (meters), bx, by, bz (degree angles)" << std::endl;
			for (int i=0; i<errors.size();i++)
			{
				std::cout<< "Waypoint ";
				for (int j=0; j<errors[i].size();j++)
				{
					if (j==errors[i].size()-1)
						std::cout<< errors[i][j];
					else
						std::cout<< errors[i][j] << "  ,  ";
				}
				std::cout<< std::endl;
			}
			std::cout<< std::endl;
		}
		if (type==1 || type==2)
		std::cout<< "MEAN ERRORS: " << m_pos/errors.size() << "  ,  " << m_bx/errors.size() << "  ,  "
								<< m_by/errors.size() << "  ,  " << m_bz/errors.size() << std::endl;
	}
};







// Utilities
Eigen::MatrixXd pct_planner::tf_waypoints(Eigen::MatrixXd waypoints, Eigen::Matrix4d wrld_T_wps)
{
    for (int i=0; i<waypoints.rows(); i++)
	{
		// Transform the waypoints
		// Points
		waypoints.block(i,0,1,3).transpose() = wrld_T_wps.block(0,0,3,3) * waypoints.block(i,0,1,3).transpose() + wrld_T_wps.block(0,3,3,1);
		// bx
		waypoints.block(i,3,1,3).transpose() = wrld_T_wps.block(0,0,3,3) * waypoints.block(i,3,1,3).transpose();
		 //by
		waypoints.block(i,6,1,3).transpose() = wrld_T_wps.block(0,0,3,3) * waypoints.block(i,6,1,3).transpose();
		 //bz
		waypoints.block(i,9,1,3).transpose() = wrld_T_wps.block(0,0,3,3) * waypoints.block(i,9,1,3).transpose();
	}
	return waypoints;
};

void pct_planner::tf_wp(Eigen::VectorXd wp, Eigen::Matrix4d T, Eigen::Matrix4d& T_wp)
{
	T_wp.block(0,0,3,1) = T.block(0,0,3,3) * wp.segment(3,3); //bx
	T_wp.block(0,1,3,1) = T.block(0,0,3,3) * wp.segment(6,3); //by
	T_wp.block(0,2,3,1) = T.block(0,0,3,3) * wp.segment(9,3); //bz
	T_wp.block(0,3,3,1) = T.block(0,0,3,3) * wp.segment(0,3) + T.block(0,3,3,1); //xyz
};

void pct_planner::invert_matrix(Eigen::Matrix4d& T, Eigen::Matrix4d& inv_T)
{
	inv_T.block(0,0,3,3) =  T.block(0,0,3,3).transpose(); // Inverse TCP, rotation transpose
	inv_T.block(0,3,3,1) = -T.block(0,0,3,3).transpose() * T.block(0,3,3,1); // Inverse TCP, translation component
};

Eigen::Vector3d pct_planner::rot2eul(Eigen::Matrix3d rot_mat)
{
	Eigen::Vector3d eul_angles;
	eul_angles = rot_mat.eulerAngles(0, 1, 2);	// XYZ sequence
	return eul_angles;
};