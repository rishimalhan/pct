#ifndef IK_ROUTINE
#define IK_ROUTINE

#include <iostream>
#include <nlopt.hpp>
#include <Eigen/Eigen>
#include <SerialLink_Manipulator/SerialLink_Manipulator.hpp>
#include <Continuity/eval_continuity.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <Eigen/QR>

class numIK
{
	public:
		numIK(SerialLink_Manipulator::SerialLink_Manipulator*,double, double,
			Eigen::Vector3i&, Eigen::VectorXd&, Eigen::VectorXi&);
		~numIK();

		// Optimization Variables
		nlopt::opt optimizer;
		nlopt::algorithm alg_type;
		int OptVarDim;	// Decision variable dimension
        double optXtolRel;	// Relative tolerance for stopping condition
        double gradH;	// Finite difference step size
        unsigned constr_dim;
        Eigen::VectorXd waypoint;	// Current waypoint being solved for
        double f_val;
        bool frst_pt;
        bool status;
        double seg_time;

        // Robot kinodynamics variables
        SerialLink_Manipulator::SerialLink_Manipulator* robot;
        KDL::Frame ff_T_ee;
        KDL::JntArray joint_config;
        KDL::JntArray prev_config;
        KDL::Frame FK_tcp;
        KDL::Jacobian curr_Jac;
        KDL::Jacobian prev_Jac;
        std::vector<double> OptVarlb;	// Lower Bounds
        std::vector<double> OptVarub;   // Upper Bounds
        Eigen::VectorXd motor_velc; // Max Joint Velocities if constraint is applied
        Eigen::VectorXd inst_vel;
        Eigen::VectorXd curr_config;
        Eigen::VectorXd prev_config_e;

        // Flags
        Eigen::Vector3i tol_axes;
        Eigen::VectorXd tolerances;
        Eigen::VectorXi constr_optns;


        // Optimization methods    
        double obj_func(const std::vector<double>&, std::vector<double>&);
        double err_func(const std::vector<double>&);
        void IneqConstr(unsigned, double*, unsigned, const double*, double*);
		void IneqConstrViol(unsigned, double*, unsigned, const double*);
		void IneqConstrViol(std::vector<double>&, std::vector<double>&);
		void solve(std::vector<double>&, Eigen::VectorXd&, KDL::Frame& ,bool, std::vector<double>&, Eigen::VectorXd&, double);
};

#endif