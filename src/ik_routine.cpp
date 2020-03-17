#include <pct/ik_routine.hpp>
#include <cmath>

// defualt nlopt fuctions begin
// Error function only minimizes the obejective. So for max use negative.
double err_func_gateway(const std::vector<double>& x, std::vector<double>& grad, void* data) 
{
    // Because we wanted a Class
    // without static members, but NLOpt library does not support
    // passing methods of Classes, we use these auxilary functions.
    numIK *gateway = (numIK *) data;
    return gateway->obj_func(x,grad);
}
void IneqConstr_gateway(unsigned constr_dim, double* constr_vec, unsigned optVarDim, const double* x, double* grad, void* data) // Multiple Constraints
{
    // Because we wanted a Class
    // without static members, but NLOpt library does not support
    // passing methods of Classes, we use these auxilary functions.
    numIK *gateway = (numIK *) data;
    gateway->IneqConstr(constr_dim, constr_vec, optVarDim, x, grad);
}
// defualt nlopt fuctions end


// Class numIK starts here
numIK::numIK(SerialLink_Manipulator::SerialLink_Manipulator* _robot, double _optXtolRel, double _gradH, 
    Eigen::Vector3i& _tol_axes, Eigen::VectorXd& _tolerances, Eigen::VectorXi& _constr_optns)
{
    robot = _robot; // Copying the robot object into the class attribute. Robot object handles FK and Jacobian

    // Set flag variables
    tol_axes = _tol_axes;
    tolerances = _tolerances;
    constr_optns = _constr_optns;
    inst_vel.resize(6);

    //choose optimizer
    // alg_type = nlopt::LN_NEWUOA;
    // alg_type = nlopt::LN_NEWUOA_BOUND;
    // alg_type = nlopt::LD_MMA;
    // alg_type = nlopt::LD_TNEWTON_PRECOND_RESTART;
    // alg_type = nlopt::LN_BOBYQA;
    // alg_type = nlopt::LN_COBYLA;
    alg_type = nlopt::LD_SLSQP;
    // alg_type = nlopt::LD_LBFGS;
    // alg_type = nlopt::GN_ISRES;

        
    OptVarDim = robot->NrOfJoints;
    optXtolRel = _optXtolRel;
    gradH = _gradH;
    // Lower and Upper Bounds for joints
    OptVarlb.resize(OptVarDim);
    OptVarub.resize(OptVarDim);
    motor_velc.resize(OptVarDim);
    for (int i=0; i < OptVarDim; i++)
    {
        OptVarlb[i] = robot->Joints_ll(i) + 0.05; OptVarub[i] = robot->Joints_ul(i) - 0.05; // 0.05 or 3 degrees away from theta bounds
        if (constr_optns(1)==1)
            motor_velc(i) = robot->Joints_vel(i);
    }
    
    // Determine dimension of all the constraints. If constraints are applied successively,
    // Set this to max dimension and keep flag in constr_func which will keep 
    // non-enforced constraints zero.
    constr_dim = 1 + OptVarDim + OptVarDim;
    std::vector<double> contr_tol;
    contr_tol.resize(constr_dim);
    for (int i=0; i<constr_dim; i++)
        contr_tol[i] = 1e-8;

    optimizer = nlopt::opt(alg_type, OptVarDim);
    optimizer.set_xtol_rel(optXtolRel);
    optimizer.set_min_objective(err_func_gateway, this);
    optimizer.add_inequality_mconstraint(IneqConstr_gateway, this, contr_tol);   // 1e-8 is optional constraint tolerance. Constraint is feasible within this.
    optimizer.set_lower_bounds(OptVarlb);
    optimizer.set_upper_bounds(OptVarub);
    optimizer.set_ftol_rel(1e-8);   // Tolerance in objective function change.
    optimizer.set_maxeval(2000);

    // Other var initializations
    joint_config = KDL::JntArray(OptVarDim);
    prev_config = KDL::JntArray(OptVarDim);
    prev_config_e.resize(OptVarDim);
    curr_config.resize(OptVarDim);

    status = true;
};

numIK::~numIK() {};

double numIK::obj_func(const std::vector<double>& x, std::vector<double>& grad)
{
    double err = err_func(x);
    if (!grad.empty()) {
        std::vector<double> xph = x;
        for (uint i=0; i < x.size(); ++i)
        {
            xph[i] += gradH;
            grad[i] = (err_func(xph)-err)/gradH;
            xph[i] -= gradH;
        }
    }    
    return err;
}

double numIK::err_func(const std::vector<double>& x)
{
    for (int i=0; i<OptVarDim; i++)
        joint_config(i) = x[i];

    robot->FK_KDL (joint_config, FK_tcp);
    return ( pow(waypoint(0)-FK_tcp(0,3),2) + pow(waypoint(1)-FK_tcp(1,3),2) + pow(waypoint(2)-FK_tcp(2,3),2) + // Translation Error
              (tol_axes(0) * exp(1 - (waypoint(3)*FK_tcp(0,0)+waypoint(4)*FK_tcp(1,0)+waypoint(5)*FK_tcp(2,0)) - acos(tolerances(1))) + //bx
               tol_axes(1) * exp(1 - (waypoint(6)*FK_tcp(0,1)+waypoint(7)*FK_tcp(1,1)+waypoint(8)*FK_tcp(2,1)) - acos(tolerances(2))) +    //by
               tol_axes(2) * exp(1 - (waypoint(9)*FK_tcp(0,2)+waypoint(10)*FK_tcp(1,2)+waypoint(11)*FK_tcp(2,2)) - acos(tolerances(3)))  )   ); //bz
};

void numIK::IneqConstr(unsigned constr_dim, double* constr_vec, unsigned optVarDim, const double* x, double* grad)
{
    IneqConstrViol(constr_dim,constr_vec,optVarDim,x);
    if (grad) // Algorithm uses gradient then below steps are executed
    {
        std::vector<double> temp_constr; temp_constr.resize(constr_dim);
        std::vector<double> temp_x; temp_x.resize(optVarDim);
        for(int i=0; i<constr_dim;i++)
            temp_constr[i] = constr_vec[i];
        for(int i=0; i<optVarDim;i++)
            temp_x[i] = x[i];
        
        for(int i=0; i<optVarDim;i++)   // Compute gradient by varying each x
        {
            temp_x[i] += gradH; // Increment x ith element by finite step
            IneqConstrViol(temp_constr,temp_x);
            temp_x[i] -= gradH; // Decrement x ith element by finite step

            for (int j=0; j<constr_dim; j++)
                grad[i*constr_dim + j] = temp_constr[j];
        }
    }    
};

void numIK::IneqConstrViol(unsigned constr_dim, double* constr_vec, unsigned optVarDim, const double* x)
{
    for (int i=0; i<OptVarDim; i++)
    {
        curr_config(i) = x[i];
        joint_config(i) = x[i];
    }
    robot->Jac_KDL (joint_config,curr_Jac);

    if (constr_optns(0)==1) // Position, orientation, and continuity
        // Continuity Constraint
        if (!frst_pt)
        {
            robot->Jac_KDL (prev_config,prev_Jac);
            constr_vec[0] = -eval_continuity(prev_Jac,curr_Jac) + 0.9;
        }
        else
            constr_vec[0] = -1;

    if (constr_optns(1)==1) // Position, orientation, and continuity + Velocity
    {
        // Instantaneous velocity constraints
        Eigen::VectorXd joint_vel_req;
        joint_vel_req.resize(OptVarDim);
        Eigen::MatrixXd jac(curr_Jac.rows(),curr_Jac.columns());
        for (int i=0;i<curr_Jac.rows();i++)
            for (int j=0;j<curr_Jac.columns();j++)
                jac(i,j) = curr_Jac(i,j);
        joint_vel_req = jac.colPivHouseholderQr().solve(inst_vel);
        joint_vel_req = joint_vel_req.cwiseAbs();
        for (int i=1;i<1+OptVarDim;i++)
            constr_vec[i] = joint_vel_req(i-1) - motor_velc(i-1);

        // Transition Constraint
        joint_vel_req = curr_config - prev_config_e;
        joint_vel_req = joint_vel_req.cwiseAbs();
        joint_vel_req = joint_vel_req.array() * motor_velc.array();
        for (int i=1+OptVarDim;i<1+2*OptVarDim;i++)
            constr_vec[i] = joint_vel_req(i-1-OptVarDim) - motor_velc(i-1-OptVarDim);
    }
    else
    {
        for (int i=1;i<1+2*OptVarDim;i++)
            constr_vec[i] = -1;
    }

    if (constr_optns(2)==1) // Position, orientation, and continuity + Velocity + Collision
    {

    }

    if (constr_optns(3)==1) // Position, orientation, and continuity + Velocity + Collision + Force
    {

    }
};

void numIK::IneqConstrViol(std::vector<double>& constr_vec, std::vector<double>& x)
{
    for (int i=0; i<OptVarDim; i++)
    {
        curr_config(i) = x[i];
        joint_config(i) = x[i];
    }
    robot->Jac_KDL (joint_config,curr_Jac);

    if (constr_optns(0)==1) // Position, orientation, and continuity
        // Continuity Constraint
        if (!frst_pt)
        {
            robot->Jac_KDL (prev_config,prev_Jac);
            constr_vec[0] = -eval_continuity(prev_Jac,curr_Jac) + 0.9;
        }
        else
            constr_vec[0] = -1;

    if (constr_optns(1)==1) // Position, orientation, and continuity + Velocity
    {
        // Instantaneous velocity constraints
        Eigen::VectorXd joint_vel_req;
        joint_vel_req.resize(OptVarDim);
        Eigen::MatrixXd jac(curr_Jac.rows(),curr_Jac.columns());
        for (int i=0;i<curr_Jac.rows();i++)
            for (int j=0;j<curr_Jac.columns();j++)
                jac(i,j) = curr_Jac(i,j);
        joint_vel_req = jac.colPivHouseholderQr().solve(inst_vel);
        joint_vel_req = joint_vel_req.cwiseAbs();
        for (int i=1;i<1+OptVarDim;i++)
            constr_vec[i] = joint_vel_req(i-1) - motor_velc(i-1);

        // Transition Constraint
        joint_vel_req = curr_config - prev_config_e;
        joint_vel_req = joint_vel_req.cwiseAbs();
        joint_vel_req = joint_vel_req.array() * motor_velc.array();
        for (int i=1+OptVarDim;i<1+2*OptVarDim;i++)
            constr_vec[i] = joint_vel_req(i-1-OptVarDim) - seg_time;
    }
    else
    {
        for (int i=1;i<1+2*OptVarDim;i++)
            constr_vec[i] = -1;
    }

    if (constr_optns(2)==1) // Position, orientation, and continuity + Velocity + Collision
    {

    }

    if (constr_optns(3)==1) // Position, orientation, and continuity + Velocity + Collision + Force
    {

    }
};

void numIK::solve(std::vector<double>& iterator, Eigen::VectorXd& _waypoint, KDL::Frame& _ff_T_ee, bool _frst_pt, 
                    std::vector<double>& _prev_config, Eigen::VectorXd& _inst_vel, double _seg_time)
{
    seg_time = _seg_time;
    inst_vel = _inst_vel;
    frst_pt = _frst_pt;

    waypoint = _waypoint;
    ff_T_ee = _ff_T_ee;
    robot -> setTCP(ff_T_ee);

    // NLopt routine
    bool optSuccess = false;
    try{
        
        nlopt::result result = optimizer.optimize(iterator, f_val);
        optSuccess = true;
    }
    catch(std::exception &e) {
        std::cout << "nlopt failed: " << e.what() << std::endl;
    }

    if (!optSuccess || f_val > 1e-4)
        status = false;

    _prev_config = iterator;
    for (int i=0; i < OptVarDim; i++)
    {
        prev_config_e(i) = _prev_config[i];
        prev_config(i) = _prev_config[i];
    }
};