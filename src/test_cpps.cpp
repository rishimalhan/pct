#include <iostream>
#include <urdf/model.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include <SerialLink_Manipulator/SerialLink_Manipulator.hpp>

#include <boost/scoped_ptr.hpp>
#include <boost/date_time.hpp>
#include <boost/algorithm/string.hpp>


int main(int argc, char** argv)
{

	// Eigen::MatrixXd FK_all;
	// // Eigen::MatrixXd Jac;

	// Eigen::VectorXd joint_config(7);
	// Eigen::MatrixXd world_T_base = Eigen::MatrixXd::Identity(4,4);
	// joint_config<< 0,0,0,0,0,0,0;

	// robot_description robot;
	// FK_all = robot.get_robotFK(joint_config,world_T_base);

	// std::cout << FK_all << std::endl;


	// ros::init(argc,argv,"test_cpps");
	// std::string urdf_path;
	// urdf_path = ros::package::getPath("PCT") + "/utilities/urdf/abb_irb_2600.urdf";
	// urdf::Model robotModel;
	// if (!robotModel.initFile(urdf_path))
	// 	std::cout << "Failed to read URDF" << std::endl;
	// else
	// 	std::cout << "Successfully imported robot:  " << robotModel.getName() << std::endl;

	// KDL::Tree robotTree;
	// if (!kdl_parser::treeFromUrdfModel(robotModel,robotTree))
	// 	std::cout<< "Error Parsing tree from URDF" << std::endl;

	// std::vector<boost::shared_ptr<urdf::Link> > links;
	// robotModel.getLinks(links);


	// // for(int i=0; i<links.size(); i++)
	// // 	std::cout<< "Link Name: " << links[i]->name << std::endl;

	// // return 0;

	// boost::shared_ptr<const urdf::Link> worldLink = robotModel.getLink("world");
 //    std::vector< boost::shared_ptr< urdf::Link > >  ctempLink = worldLink->child_links;
 //    boost::shared_ptr< urdf::Link > tempLink;

 //    int i = 0;
 //    while(i<links.size()-1){
 //        tempLink = ctempLink[0];
 //        ctempLink.clear();
 //        ctempLink = tempLink->child_links;
 //        std::cout<< tempLink->name << std::endl;
 //        ++i;
 //    }

 //    kdl::Chain temp_chain;
 //    robotTree.getChain(links[0]->name, links[2]->name, temp_robot_chain);


	KDL::Frame shit;
	shit = KDL::Frame( KDL::Rotation(1,0,0,0,1,0,0,0,1), KDL::Vector(1,1,1) );

	for (int i=0; i<4; i++)
		for (int j=0; j<4; j++)
			std::cout<< shit(i,j) << std::endl;


	// boost::posix_time::ptime start_time;
	// boost::posix_time::time_duration time_diff;
	// double elapsed = 0;

	// KDL::Frame baseFrame = KDL::Frame::Identity();
	// KDL::Frame robotEE_T_toolEE = KDL::Frame::Identity();

	// std::string urdf_path;
	// urdf_path = ros::package::getPath("PCT") + "/utilities/urdf/abb_irb_2600.urdf";
	// SerialLink_Manipulator::SerialLink_Manipulator robot(urdf_path, baseFrame,robotEE_T_toolEE,"base_link","tool0");

	// KDL::JntArray joints;
	// joints.resize(6);
	// joints(0) = 0;
	// joints(1) = 0;
	// joints(2) = 0;
	// joints(3) = 0;
	// joints(4) = 0;
	// joints(5) = 0;

	// std::cout << "Joint Angles:" << std::endl;
	// robot.print_KDL_JointArray(joints);
	// std::cout<<std::endl;

	// KDL::Frame eeFrame = KDL::Frame::Identity();

	// std::cout << "Forward Kinematics (base to TCP):" << std::endl;
	// start_time = boost::posix_time::microsec_clock::local_time();
	// robot.FK_KDL (joints, eeFrame);
	// time_diff = boost::posix_time::microsec_clock::local_time() - start_time;
	// elapsed = time_diff.total_nanoseconds() / 1e9;	
	// std::cout<< "compute time: " << elapsed << std::endl;
	// robot.print_KDL_Frame(eeFrame);
	// std::cout<<std::endl;

	return 0;
}