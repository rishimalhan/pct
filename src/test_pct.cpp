#include <iostream>
#include <pct/pct_planner.hpp>
#include <ros/package.h>
#include <boost/date_time.hpp>
#include <gen_utilities/file_rw.hpp>

int main(int argc, char**argv)
{
	//////////////////////////////INITIALIZATIONS/////////////////////////////////////////
	// Set the planner parameters
	Eigen::Vector3i tol_optns(3);
	Eigen::VectorXd tolerances(4);
	Eigen::VectorXi constr_optns(4);
	std::string rob_urdf;
	std::string rob_base_link;
	std::string rob_tip_link;

	// std::vector<double> initial_guess = {0,0,0,0,0,0,0};
	// std::vector<double> initial_guess = {0.31189,0.2209,-0.1785,-1.5357,0.0176,1.3463,0}; // Iiwa7
	// std::vector<double> initial_guess = {0.00775273,0.112036,0.650733,0.028248,1.05047,-0.00839303}; // ABB
	std::vector<double> initial_guess = {0,0,0,0,0,0}; // ABB

	// tol_optns: bx, by, bz respectively.
	tol_optns << 1, 1, 1;
	// Orientation tolerance 3.14 implies one doesn't care about it's alignment
	tolerances << 0.003, 0.0524, 0.0524, 0.0524;
	// tolerances << 0.003, 3.14, 3.14, 0.0524; 
	constr_optns << 1,0,0,0;
	double velocity = 50;
	
	rob_urdf = ros::package::getPath("pct") + "/utilities/urdf/abb_irb_2600.urdf";
	// rob_urdf = ros::package::getPath("pct") + "/utilities/urdf/iiwa7.urdf";
	// rob_urdf = ros::package::getPath("pct") + "/utilities/urdf/gp8.urdf";
	// rob_urdf = ros::package::getPath("pct") + "/utilities/urdf/iiwa14.urdf";

	// ROBOT base and tip links for URDF parser
	// ABB
	rob_base_link = "base_link";
	rob_tip_link = "tool0";

	// IIWA 7 and 14
	// rob_base_link = "iiwa_link_0";
	// rob_tip_link = "iiwa_link_ee";

	// GP8
	// rob_base_link = "gp8_1_base_link";
	// rob_tip_link = "gp8_1_tool0";


	Eigen::Matrix4d world_T_base = Eigen::Matrix4d::Identity(4,4);

	// Files to be read
	std::string csv_path = ros::package::getPath("pct") + "/csv/";

	// std::string file_name = "ascent_test_points.csv";
	// std::string file_name = "sanding_points_v2.csv";
	// std::string file_name = "sanding_points_v2.csv";
	std::string file_name = "gear_points.csv";
	// std::string file_name = "draping_motion_xyz_bxbybz.csv";
	// std::string grp_file_name = "draping_motion_grp_idx.csv";
	// std::string file_name = "ascent_lockheed_points.csv";
	// std::string grp_file_name = "ascent_lockheed_grp.csv";
	// std::string tcp_id_file = "tcp_idx.csv";
	// std::string tcp_tf_file = "tcp_tf.csv";
	
	// std::string file_name = "ak_points_new.csv";	

	file_rw file_handler;
	Eigen::MatrixXd waypoints = file_handler.file_read_mat(csv_path+file_name);
	// waypoints.block(0,0,waypoints.rows(),3) = waypoints.block(0,0,waypoints.rows(),3) / 1000;

	// Eigen::MatrixXd temp1 = file_handler.file_read_mat(csv_path+grp_file_name);
	// Eigen::MatrixXd temp2 = file_handler.file_read_mat(csv_path+tcp_id_file);
	// Eigen::MatrixXd tcp_tf = file_handler.file_read_mat(csv_path+tcp_tf_file);
	// Eigen::MatrixXi grp_idx(temp1.rows(),2);
	// Eigen::VectorXi tcp_id = Eigen::VectorXi::Zero(temp2.rows());

	// for (int i=0; i<temp1.rows();i++)
	// {
	//  	grp_idx(i,0) = (int) temp1(i,0)-1; // Remove the -1 if matlab indices
	//  	grp_idx(i,1) = (int) temp1(i,1)-1;
	// }

	// for (int i=0; i<temp2.rows(); i++)
	// 	tcp_id(i) = (int) temp2(i,0) - 1;
	
	// For AK gp8
	// Eigen::MatrixXi grp_idx(1,2);
	// grp_idx << 0,waypoints.rows()-1;

	// IDENTITY MATRICES ACTIVATE
	Eigen::Matrix4d wrld_T_wp = Eigen::MatrixXd::Identity(4,4);
	Eigen::MatrixXd tcp_tf = Eigen::MatrixXd::Identity(4,4);
	Eigen::VectorXi tcp_id = Eigen::VectorXi::Zero(waypoints.rows());


	// //Ascent for ABB
	// Eigen::Quaterniond q;
	// q.x() = 0; q.y() = 0; q.z() = 1; q.w() = 0;
	// wrld_T_wp.block(0,0,3,3) = q.normalized().toRotationMatrix();
	// wrld_T_wp(0,3) = 1.2; wrld_T_wp(1,3) = 0.3; wrld_T_wp(2,3) = 0.3467;


	// // //Ascent for iiwa7
	// Eigen::Quaterniond q;
	// q.x() = 0; q.y() = 0; q.z() = -0.8096; q.w() = 0.5869;
	// wrld_T_wp.block(0,0,3,3) = q.normalized().toRotationMatrix();
	// wrld_T_wp(0,3) = 0.0700; wrld_T_wp(1,3) = 0.7700; wrld_T_wp(2,3) = 0.0467;


	// Blade for ABB
	// Eigen::Quaterniond q;
	// q.x() = -0.0714; q.y() = 0.1823; q.z() = -0.0975; q.w() = 0.9758;
	// wrld_T_wp.block(0,0,3,3) = q.normalized().toRotationMatrix();
	// wrld_T_wp(0,3) = -0.1720; wrld_T_wp(1,3) = 0.1636; wrld_T_wp(2,3) = 0.6727;

	Eigen::Quaterniond q;
	// wrld_T_wp(0,3) = 0.5779; wrld_T_wp(1,3) = -0.4655; wrld_T_wp(2,3) = 0.3; // Sanding
	q.x() = -0.0090; q.y() = -0.0242; q.z() = 0.0204; q.w() = 0.9995;
	wrld_T_wp(0,3) = 0.2922; wrld_T_wp(1,3) = -0.0031; wrld_T_wp(2,3) = 0.8352; //Gear

	// //Crown for gp8
	// Eigen::Quaterniond q;
	// q.x() = 0; q.y() = 0; q.z() = 0; q.w() = 1;
	// wrld_T_wp.block(0,0,3,3) = q.normalized().toRotationMatrix();
	// wrld_T_wp(0,3) = 0.401; wrld_T_wp(1,3) = 0.036; wrld_T_wp(2,3) = 0.307 + 0.006;


	// LM Demo
	// wrld_T_wp(0,3) = 0.3921; wrld_T_wp(1,3) = -0.2804; wrld_T_wp(2,3) = 0.1930;	

	// Roller TCP
	// tcp_tf(0,3) = -0.049; tcp_tf(2,3) = 0.133;

	// // Sander TCP
	// Eigen::Quaterniond q;
	// tcp_tf(0,3) = -0.0327; tcp_tf(1,3) = -0.0054; tcp_tf(2,3) = 0.1383;
	// q.x() = -0.0001; q.y() = -0.0050;  q.z() = 0.9998; q.w() = 0.0208;
	// tcp_tf.block(0,0,3,3) = q.normalized().toRotationMatrix();

	// // Burring
	// tcp_tf(2,3) = 0.096;











	////////////////////////////// MAIN ////////////////////////////////////////////////
	// Sanity checks
	if ((tol_optns.rows() != 3) || (tolerances.rows() != 4))
	{
		std::cout<< "Parameters not defined. Terminating" << std::endl;
		return 0;
	}

	// if ((tol_optns(0)==1&&tol_optns(1)==1&&tol_optns(2)==0) || (tol_optns(0)==0&&tol_optns(1)==1&&tol_optns(2)==1)
	// 	 || (tol_optns(0)==1&&tol_optns(1)==0&&tol_optns(2)==1))
	// {
	// 	std::cout<< "Tolerance Options Incorrect. Terminating" << std::endl;
	// 	return 0;
	// }

	if (waypoints.cols()!=12)
	{
		std::cout<< "Waypoints Incorrect. Terminating" << std::endl;
		return 0;
	}	
	std::cout<< "*******************INITIALIZING PLANNER*******************" << std::endl;
	pct_planner planner;
	std::cout<< "********************INITIALIZING ROBOT********************" << std::endl;
	planner.init_robot(rob_urdf,world_T_base,rob_base_link,rob_tip_link);
	std::cout<< "****************SETTING TRAJECTORY OPTIONS****************" << std::endl;
	planner.set_trajOptions(tol_optns,tolerances,constr_optns);
	planner.cart_velc(0) = velocity / 1000;	// Can be a vector equal to waypoint size - 1
	// plan_pct method accepts waypoints in xyz_bxbybz format. Trajectory is given as an output.
	// Other trajectory parameters are also accessible
	std::cout<< "*********************Solving for PCT*********************" << std::endl;
	double elapsed;
	boost::posix_time::ptime start_time;
	boost::posix_time::time_duration time_diff;
	start_time = boost::posix_time::microsec_clock::local_time();

	Eigen::MatrixXd trajectory(waypoints.rows(),planner.optVardim);
	planner.solve_pct(initial_guess, waypoints, wrld_T_wp, tcp_tf,tcp_id);
	if (!planner.sol_status)
		planner.status_report(1);
	trajectory = planner.trajectory;


	// Eigen::MatrixXd trajectory(waypoints.rows(),planner.optVardim);
	// for (int i=0; i<grp_idx.rows(); i++) // Group Indices
	// {
	// 	Eigen::MatrixXd wp_block = waypoints.block(grp_idx(i,0),0,grp_idx(i,1)-grp_idx(i,0)+1,12);
	// 	Eigen::VectorXi tcp_id_block = tcp_id.segment(grp_idx(i,0),grp_idx(i,1)-grp_idx(i,0)+1);
	// 	planner.solve_pct(initial_guess, wp_block, wrld_T_wp, tcp_tf,tcp_id_block);
	// 	if (!planner.sol_status)
	// 		planner.status_report(1);
	// 	trajectory.block(grp_idx(i,0),0,grp_idx(i,1)-grp_idx(i,0)+1,planner.optVardim) = planner.trajectory;
	// }
	time_diff = boost::posix_time::microsec_clock::local_time() - start_time;
	elapsed = time_diff.total_nanoseconds() / 1e9;	
	std::cout<< "compute time: " << elapsed << std::endl;	

	// std::cout<< "****************Generating Status Report****************" << std::endl;
	// planner.status_report();

	// file_handler.file_write(csv_path+"irb_traj.csv",planner.trajectory);
	file_handler.file_write(csv_path+"irb_traj.csv",trajectory);
	return 0;
}
