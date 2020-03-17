#ifndef PCT_PLANNER
#define PCT_PLANNER


#include <iostream>
#include <Eigen/Eigen>
#include <SerialLink_Manipulator/SerialLink_Manipulator.hpp>
#include <kdl/frames.hpp>
#include <pct/ik_routine.hpp>

class pct_planner
{
	public:
		pct_planner();
		~pct_planner();

		// General trajectory variables
		Eigen::VectorXd tolerances;
		Eigen::Vector3i tol_optns;
		Eigen::VectorXi constr_optns;
		Eigen::MatrixXd trajectory;
		Eigen::MatrixXd waypoints;
		Eigen::Matrix4d wrld_T_wps;
		Eigen::VectorXd cart_velc;
		Eigen::VectorXd seg_times;

		// Robot Initialization Variables
		std::string rob_urdf;
		std::string rob_base_link;
		std::string rob_tip_link;
		SerialLink_Manipulator::SerialLink_Manipulator robot;
		KDL::Frame world_T_base;
		KDL::Frame flange_T_ee;
		int NrJoints;

		// Optimization and solution variables
		bool sol_status;
		std::vector<bool> wp_status;
		std::vector<double> wp_err;
		int optVardim;
		double optXtolRel;
		double gradH;
		Eigen::MatrixXd report;
		int failures;
		bool print_statements;

		// Validation Flags
        std::vector<bool> pose_status;

        // Process Velocity Variables
        Eigen::VectorXd curr_wp;
		Eigen::VectorXd nxt_wp;
		Eigen::Matrix4d curr_ff_T_ee;
		Eigen::Matrix4d curr_w_T_ff;
		Eigen::Matrix4d curr_inv_ff_T_ee;
		Eigen::Matrix4d nxt_ff_T_ee;
		Eigen::Matrix4d nxt_w_T_ff;
		Eigen::Matrix4d nxt_inv_ff_T_ee;
		double seg_time;

		// Methods
		void init_robot(std::string&, Eigen::Matrix4d&, std::string&, std::string&);
		void set_trajOptions(Eigen::Vector3i&, Eigen::VectorXd&, Eigen::VectorXi&);
		void process_velocity(int,Eigen::MatrixXd&,Eigen::MatrixXd&,Eigen::VectorXi&, Eigen::VectorXd&);
		void solve_pct(std::vector<double>, Eigen::MatrixXd&, Eigen::Matrix4d&, Eigen::MatrixXd&, Eigen::VectorXi&);
		void validate_pose( int, const Eigen::VectorXd&, std::vector<double>& );
		void status_report(int);
		Eigen::MatrixXd tf_waypoints(Eigen::MatrixXd, Eigen::Matrix4d);
		void tf_wp(Eigen::VectorXd,Eigen::Matrix4d, Eigen::Matrix4d&);
		void invert_matrix(Eigen::Matrix4d&, Eigen::Matrix4d&);
		Eigen::Vector3d rot2eul(Eigen::Matrix3d);
};


#endif