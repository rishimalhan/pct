// author: akabir@usc.edu

#include <iostream>
#include <iomanip>
// boost includes
#include <boost/date_time.hpp>
#include <boost/algorithm/string.hpp>
// KDL includes
#include <kdl/frames.hpp>
// AK includes
#include <smPCT.hpp>

int main(int argc, char *argv[])
{
	boost::posix_time::ptime start_time;
	boost::posix_time::time_duration time_diff;
	double elapsed = 0;


	/* ######################################################################## */
	// define base frame for robot
	KDL::Frame baseFrame = KDL::Frame::Identity(); 	// modify as needed

	// define base tool frame for robot
    KDL::Rotation toolR(KDL::Vector(1,0,0),
                        KDL::Vector(0,1,0),
                        KDL::Vector(0,0,1));    
    KDL::Vector toolt(KDL::Vector(0,0,0));
    KDL::Frame robotEE_T_toolEE(toolR,toolt); 	 	// modify as needed


	/* ######################################################################## */
	// instantiate robot object
    // iiwa 7
	std::string base_link = "iiwa_link_0";
	std::string tip_link = "iiwa_link_ee";
	SerialLink_Manipulator::SerialLink_Manipulator robot("../data/iiwa7.urdf", baseFrame,robotEE_T_toolEE,base_link,tip_link); // modify as needed
	
	// UR5
	// std::string base_link = "base_link";
	// std::string tip_link = "ee_link";
	// SerialLink_Manipulator::SerialLink_Manipulator robot("../data/ur5.urdf", baseFrame,robotEE_T_toolEE,base_link,tip_link); // modify as needed
	
	// ABB IRB 2600
	// std::string base_link = "base_link";
	// std::string tip_link = "tool0";	
	// SerialLink_Manipulator::SerialLink_Manipulator robot("../data/abb_irb_2600.urdf", baseFrame,robotEE_T_toolEE,base_link,tip_link); // modify as needed


	/* ######################################################################## */
	// Initialize variables 
	// number of frames vs number of links need to be carefully selected
	// current implementation assumes NrOfLinks = NrOfJoints + 1 

	int NrOfJoints = 7;
    KDL::JntArray joints = KDL::JntArray(NrOfJoints);
    KDL::JntArray randomSeedJoints = KDL::JntArray(NrOfJoints);
    KDL::JntArray ikResultJoints = KDL::JntArray(NrOfJoints);

    KDL::Frame eeFrame = KDL::Frame::Identity(); 	
    KDL::Frame I = KDL::Frame::Identity(); 
    KDL::Jacobian jac;
    std::vector<KDL::Frame> linkFrames;
    linkFrames.clear();

    for(int i = 0; i < NrOfJoints; ++i){
		joints(i) = 0;
		randomSeedJoints(i) = rand();
		linkFrames.push_back(I);
    }	
    linkFrames.push_back(I); // iiwa has 8 links


	/* ######################################################################## */
    // test kinematics
	std::cout << "Joint Angles:" << std::endl;
	robot.print_KDL_JointArray(joints);
	std::cout<<std::endl;

	std::cout << "Forward Kinematics (base to TCP):" << std::endl;
	start_time = boost::posix_time::microsec_clock::local_time();
	robot.FK_KDL (joints, eeFrame);
	time_diff = boost::posix_time::microsec_clock::local_time() - start_time;
	elapsed = time_diff.total_nanoseconds() / 1e9;	
	std::cout<< "compute time: " << elapsed << std::endl;
	robot.print_KDL_Frame(eeFrame);
	std::cout<<std::endl;

	std::cout << "Jacobian:" << std::endl;
	start_time = boost::posix_time::microsec_clock::local_time();
	robot.Jac_KDL (joints,jac);
	time_diff = boost::posix_time::microsec_clock::local_time() - start_time;
	elapsed = time_diff.total_nanoseconds() / 1e9;	
	std::cout<< "compute time: " << elapsed << std::endl;	
	robot.print_KDL_Jacobian(jac);
	std::cout<<std::endl;

	std::cout << "Inverse Kinematics (TCP):" << std::endl;
	start_time = boost::posix_time::microsec_clock::local_time();
	robot.IK_KDL (randomSeedJoints, eeFrame, ikResultJoints);
	time_diff = boost::posix_time::microsec_clock::local_time() - start_time;
	elapsed = time_diff.total_nanoseconds() / 1e9;	
	std::cout<< "compute time: " << elapsed << std::endl;	
	robot.print_KDL_JointArray(ikResultJoints);
	std::cout<<std::endl;

	std::cout << "Forward Kinematics (base to all links):" << std::endl;
	start_time = boost::posix_time::microsec_clock::local_time();
	robot.FK_KDL_all_links (joints, linkFrames);
	time_diff = boost::posix_time::microsec_clock::local_time() - start_time;
	elapsed = time_diff.total_nanoseconds() / 1e9;	
	std::cout<< "compute time: " << elapsed << std::endl;	
	for(int i = 0; i < NrOfJoints+1; ++i){
		std::cout << "base to link: " << i << std::endl;
		robot.print_KDL_Frame(linkFrames[i]);
	}

	std::cout << "\n\n..terminating\n\n" << std::endl;
	return 0;
}

