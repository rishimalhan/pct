// author: akabir@usc.edu

#include <iostream>
#include <iomanip>
#include <vector>
#include <fstream>
#include <array>
#include <valarray>
#include <typeinfo>

#include <boost/date_time.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>

#include <ros/ros.h>
#include <urdf/model.h>
#include <limits>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

#include <SerialLink_Manipulator/SerialLink_Manipulator.hpp>
#include <gen_utilities/utilities.hpp>

namespace SerialLink_Manipulator{

SerialLink_Manipulator::SerialLink_Manipulator() {};

SerialLink_Manipulator::SerialLink_Manipulator(std::string urdf_path, KDL::Frame base_frame, KDL::Frame tcp_frame, std::string base_link, std::string tip_link) :
    URDFpath(urdf_path),
    BaseFrame(base_frame),
    TCPFrame(tcp_frame),
    BaseLink(base_link),
    TipLink(tip_link) 
{
    print_stuff = false;
    dummy_str = "burp";
    // import robot through URDF
    spawn_KDLRobotModel_from_urdf ();
    // spawn KDL tree for the robot
    spawn_KDLTree_from_KDLRobotModel ();
    // spawn KDL chain for each link of the robot
    spawn_KDLChain_from_link0_to_all_links ();
    // get number of links
    NrOfLinks = LinkNames.size(); 
    // std::cout << "\nNrOfLinks: " <<  NrOfLinks << std::endl;
    // spawn KDL chain for the robot from link0 to linkee
    spawn_KDLRobotChain_from_link0_to_linkee ();
    // get number of joints
    NrOfJoints = RobotChain.getNrOfJoints();
    // std::cout << "\nNrOfJoints: " << NrOfJoints << std::endl; 
    // get robot joint limits
    populate_robot_joint_limits ();
    // spawn KDL FK/Jac/IK solvers 
    spawn_KDL_FK_JAC_IK_solvers ();
    // std::cout << "################################"<< std::endl; 
    // std::cout << "ROBOT INITIALIZATION COMPLETE" << std::endl; 
    // std::cout << "################################"<< std::endl; 
};

void SerialLink_Manipulator::initialize(std::string urdf_path, KDL::Frame base_frame, KDL::Frame tcp_frame, std::string base_link, std::string tip_link)
{
    print_stuff = false;
    URDFpath = urdf_path;
    BaseFrame = base_frame;
    TCPFrame = tcp_frame;
    BaseLink = base_link;
    TipLink = tip_link;

    dummy_str = "burp";
    // import robot through URDF
    spawn_KDLRobotModel_from_urdf ();
    // spawn KDL tree for the robot
    spawn_KDLTree_from_KDLRobotModel ();
    // spawn KDL chain for each link of the robot
    spawn_KDLChain_from_link0_to_all_links ();
    // get number of links
    NrOfLinks = LinkNames.size(); 
    std::cout << "\nNrOfLinks: " <<  NrOfLinks << std::endl;
    // spawn KDL chain for the robot from link0 to linkee
    spawn_KDLRobotChain_from_link0_to_linkee ();
    // get number of joints
    NrOfJoints = RobotChain.getNrOfJoints();
    std::cout << "\nNrOfJoints: " << NrOfJoints << std::endl; 
    // get robot joint limits
    populate_robot_joint_limits ();
    // spawn KDL FK/Jac/IK solvers 
    spawn_KDL_FK_JAC_IK_solvers ();
    // std::cout << "################################"<< std::endl; 
    // std::cout << "ROBOT INITIALIZATION COMPLETE" << std::endl; 
    // std::cout << "################################"<< std::endl; 
};

void SerialLink_Manipulator::setTCP(KDL::Frame _ff_T_tcp)
{
    TCPFrame = _ff_T_tcp;
};

double SerialLink_Manipulator::fRand(double min, double max)
{
  double f = (double)rand() / RAND_MAX;
  return min + f * (max - min);
};

void SerialLink_Manipulator::spawn_KDLRobotModel_from_urdf (){
    if (!RobotModel.initFile(URDFpath)){
        std::cout << "Failed to parse urdf robot model" << std::endl;
    }    
    else {
        RobotName = RobotModel.getName();
        if (print_stuff)
        {
            std::cout << std::endl;
            std::cout << "################################"<< std::endl; 
            std::cout << "\nImported Robot: " << RobotName << "\n" << std::endl; 
            std::cout << "################################"<< std::endl; 
        }
    }
};

void SerialLink_Manipulator::spawn_KDLTree_from_KDLRobotModel (){
    if (!kdl_parser::treeFromUrdfModel(RobotModel, RobotModelTree)){
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<< std::endl; 
        std::cout << "Failed to construct kdl tree" << std::endl;
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<< std::endl; 
    }
};


void SerialLink_Manipulator::spawn_KDLChain_from_link0_to_all_links (){
    // this function needs to be tested on different robot URDF's. It may need to be patched based on urdf variations.
    // currently it assumes that "world" is the root link in a manipulator

    RobotModel.getLinks(links);
    if (print_stuff)
        std::cout << "\nAvailable Links: " << std::endl;
    
    // the following segment of code attempts to sort the links in sequential order. originating from robot base and ending at end-effector
    // it assumes that "world" is the root link in a manipulator
    boost::shared_ptr<const urdf::Link> worldLink = RobotModel.getLink("world");
    std::vector< boost::shared_ptr< urdf::Link > >  ctempLink = worldLink->child_links;
    int i = 0;
    while(i<links.size()){
        boost::shared_ptr< urdf::Link > tempLink = ctempLink[0];
        ctempLink.clear();
        ctempLink = tempLink->child_links;
        if( !std::count(LinkNames.begin(),LinkNames.end(),tempLink->name) ){ // avoid repeatation
            LinkNames.push_back(tempLink->name);
            links[i] = tempLink;
            if (print_stuff)
                std::cout << i << " : " << tempLink->name << std::endl;
        }
        else
            break;
        tempLink.reset();
        i++;
    }
    
    // the following segment of code assumes that the links are in sequential order. originating from robot base and ending at end-effector
    for (int link_id = 0; link_id < LinkNames.size(); ++link_id){
        boost::shared_ptr<urdf::Link> current_link = links[link_id];
        if(current_link->name != "world"){
            KDL::Chain temp_robot_chain;
            if(!RobotModelTree.getChain(links[0]->name, links[link_id]->name, temp_robot_chain) || LinkNames[link_id]=="world"){
                std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<< std::endl;  
                std::cout << "Couldn't find chain" << links[0]->name << "to" <<  links[link_id]->name << std::endl;
                std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<< std::endl; 
            }
            else{
                RobotModelTree.getChain(links[0]->name, links[link_id]->name, temp_robot_chain);
                RobotChainsAllLinks.push_back(temp_robot_chain);  

                if (print_stuff)
                {
                    std::cout << "\n############################" << std::endl;
                    std::cout << "RobotChain formed from " << links[0]->name << " to " <<  links[link_id]->name << std::endl;  
                }
                std::vector<KDL::Segment> chain_segs;
                chain_segs = RobotChainsAllLinks[link_id].segments;
                if (print_stuff)
                {
                    std::cout << "joints: " <<std::endl;
                    for(int i = 0; i < link_id; ++i)
                        std::cout << chain_segs[i].getJoint().getName() << std::endl;
                    std::cout << "############################\n" << std::endl;
                }
            }
        }
    }

};

void SerialLink_Manipulator::spawn_KDLRobotChain_from_link0_to_linkee (){
    // comment out the following line to over-ride user passed link names
    // BaseLink = LinkNames[0]; TipLink  = LinkNames[NrOfLinks-1];      // e.g. BaseLink = "iiwa_link_0"; TipLink  = "iiwa_link_ee";  
    if(!RobotModelTree.getChain(BaseLink, TipLink, RobotChain)){
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<< std::endl; 
        std::cout << "Couldn't find chain " << BaseLink << " to " <<  TipLink << std::endl;  
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<< std::endl; 
    }
    if (print_stuff)
        std::cout << "\nRobotChain formed from " << BaseLink << " to " <<  TipLink << std::endl;  
};

void SerialLink_Manipulator::populate_robot_joint_limits (){
    KDL::JntArray temp_jnt_l = KDL::JntArray(1);
    KDL::JntArray temp_jnt_u = KDL::JntArray(1);

    Joints_ll = KDL::JntArray(NrOfJoints); Joints_ul = KDL::JntArray(NrOfJoints); Joints_ntrl = KDL::JntArray(NrOfJoints);
    urdf::JointConstSharedPtr joint;
    std::vector<KDL::Segment> chain_segs = RobotChain.segments;
    // std::cout << "\nRobot Joint Limits: " <<std::endl;
    for(int i = 0; i < NrOfJoints; ++i) {
        joint = RobotModel.getJoint(chain_segs[i].getJoint().getName());
        Joints_ll(i) = joint->limits->lower;
        Joints_ul(i) = joint->limits->upper;
        Joints_ntrl(i) = ( Joints_ll(i) + Joints_ul(i) ) / 2.0;
        JointNames.push_back(chain_segs[i].getJoint().getName()); 
        if (print_stuff)      
            std::cout << JointNames[i] << ": " << Joints_ll(i) << ", " << Joints_ul(i) <<std::endl;
    }
};

void SerialLink_Manipulator::populate_robot_joint_vel_limits (){
    Joints_vel = KDL::JntArray(NrOfJoints);
    urdf::JointConstSharedPtr joint;
    std::vector<KDL::Segment> chain_segs = RobotChain.segments;
    if (print_stuff)
        std::cout << "\nRobot Joint Velocity Limits: " <<std::endl;
    for(int i = 0; i < NrOfJoints; ++i) {
        joint = RobotModel.getJoint(chain_segs[i].getJoint().getName());
        Joints_vel(i) = joint->limits->velocity;
    }
};

void SerialLink_Manipulator::getRandJointArray(KDL::JntArray & randJoints)
{
    for (int i = 0; i<NrOfJoints; ++i)
        randJoints(i) = fRand(Joints_ll(i), Joints_ul(i));
}

void SerialLink_Manipulator::spawn_KDL_FK_JAC_IK_solvers (){
    KDL_FKSolver.reset(new KDL::ChainFkSolverPos_recursive(RobotChain));
    KDL_JacSolver.reset(new KDL::ChainJntToJacSolver(RobotChain));
    KDL_VIKSolver.reset(new KDL::ChainIkSolverVel_pinv(RobotChain));
    KDL_IK_eps = 1e-5;
    KDL_IKSolver.reset(new KDL::ChainIkSolverPos_NR_JL(RobotChain,Joints_ll,Joints_ul,*KDL_FKSolver, *KDL_VIKSolver, 1, KDL_IK_eps));
    if (print_stuff){
        std::cout << "\n############################" << std::endl;
        std::cout << "Spawning FK" << std::endl;
        std::cout << "############################\n" << std::endl;
    }
    // FK solvers for all links
    KDL_FKSolver_all_links.resize(RobotChainsAllLinks.size());
    for (int i =0; i<RobotChainsAllLinks.size(); ++i){
        KDL_FKSolver_all_links[i].reset(new KDL::ChainFkSolverPos_recursive(RobotChainsAllLinks[i]));
        
        KDL::ChainFkSolverPos_recursive tempsolver(RobotChainsAllLinks[i]);
        KDL_FKSolver_all_links_obj.push_back(tempsolver);
        
        // // following two lines does not compile. can't push boost shared pointer
        // KDL_FKSolverTEMP.reset(new KDL::ChainFkSolverPos_recursive(RobotChainsAllLinks[i]));
        // KDL_FKSolver_all_links_obj.push_back(KDL_FKSolverTEMP);

        if (print_stuff)
            std::cout << "\n############################" << std::endl;
        std::vector<KDL::Segment> chain_segs;
        chain_segs = RobotChainsAllLinks[i].segments;
        if (print_stuff){
            std::cout << "joints: " <<std::endl;
            for(int j = 0; j < i; ++j)
                std::cout << chain_segs[j].getJoint().getName() << std::endl;
            std::cout << "############################\n" << std::endl;
        }
    }
};

KDL::JntArray SerialLink_Manipulator::convert_double_joints_to_KDL_joint(double* joints){
    KDL::JntArray jointAngles = KDL::JntArray(NrOfJoints);
    for (int idx = 0; idx < NrOfJoints; ++idx) {
        try { 
            jointAngles(idx) = joints[idx]; 
        } 
        catch (const std::exception& e) { 
            std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<< std::endl; 
            std::cout << "Failed to read joint angle for joint: " << idx << std::endl;
            std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<< std::endl; 
            jointAngles(idx) = -10000;
        }
    }
    return jointAngles;    
};

bool SerialLink_Manipulator::is_valid_joint_angle(KDL::JntArray jointAngles) {
    for (int i = 0; i < NrOfJoints; ++i)
        if(jointAngles(i) < Joints_ll(i) - 0.001 || jointAngles(i) > Joints_ul(i) + 0.001)
            return false;
    return true;
};

void SerialLink_Manipulator::FK_KDL_all_links (KDL::JntArray & joints, std::vector<KDL::Frame> & linkFrames){
    if (print_stuff){
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<< std::endl; 
        std::cout << "SerialLink_Manipulator::FK_KDL_all_links will be available in future release." << std::endl;
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<< std::endl; 
    }
    // exit(EXIT_FAILURE);

    KDL::Frame tempFrame = KDL::Frame::Identity();    
    if(is_valid_joint_angle(joints)){
        for (int i=0;i<NrOfJoints;++i){
            KDL::JntArray jointAngles = KDL::JntArray(i+1);
            for (int j=0;j<=i;++j){
                jointAngles(j) = joints(j);
            }
            KDL_FKSolver_all_links_obj[i+1].JntToCart(jointAngles, linkFrames[i+1]);
        }
        // linkFrames[NrOfJoints+1] = linkFrames[NrOfJoints] * TCPFrame;
    }
    else {
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<< std::endl; 
        std::cout << "Invalid Joint Angles. Can't solve FK" << std::endl;
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<< std::endl; 
        for (int i = 0; i < NrOfJoints; ++i)
            std::cout << JointNames[i] << ": " << joints(i) << std::endl;
    }
};

void SerialLink_Manipulator::FK_KDL (KDL::JntArray & joints, KDL::Frame & eeFrame){
    if(is_valid_joint_angle(joints)){
        KDL_FKSolver->JntToCart(joints, eeFrame);
        eeFrame = eeFrame * TCPFrame;
    }
    else {
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<< std::endl; 
        std::cout << "Invalid Joint Angles. Can't solve FK" << std::endl;
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<< std::endl; 
        for (int i = 0; i < NrOfJoints; ++i)
            std::cout << JointNames[i] << ": " << joints(i) << std::endl;
    }
};

void SerialLink_Manipulator::FK_Analytical (){
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<< std::endl; 
        std::cout << "Analytical FK Solver coming in future release" << std::endl;
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<< std::endl; 
};

void SerialLink_Manipulator::Jac_KDL (KDL::JntArray & joints, KDL::Jacobian & KDLJacobian){
    // KDL::JntArray jointAngles = convert_double_joints_to_KDL_joint(joints);
    KDLJacobian.resize(NrOfJoints);
    if(is_valid_joint_angle(joints))
        // std::cout << "im in" << std::endl;
        KDL_JacSolver->JntToJac(joints, KDLJacobian);
    else {
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<< std::endl; 
        std::cout << "Invalid Joint Angles. Can't solve for Jacobian" << std::endl;
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<< std::endl; 
        // for (int i = 0; i < NrOfJoints; ++i)
            // std::cout << JointNames[i] << ": " << joints(i) << std::endl;
    }
};

void SerialLink_Manipulator::IK_KDL (KDL::JntArray & seed_joints, KDL::Frame & eeFrame, KDL::JntArray & ikresult){
    // KDL::JntArray seed_jointAngles = convert_double_joints_to_KDL_joint(seed_joints);
    if(!is_valid_joint_angle(seed_joints))
        seed_joints = Joints_ntrl;
    // int rc = IKSolver.CartToJnt(q,eeFrame,ikresult); //q_seed,T_base_goal,q_out 
    if(KDL_IKSolver->CartToJnt(seed_joints,eeFrame,ikresult)>=0)
        std::cout << "IK successful" << std::endl;
    else
        std::cout << "IK failed"<< std::endl;    
};

void SerialLink_Manipulator::print_KDL_JointArray(KDL::JntArray & q){
    std::cout << std::endl;std::cout << std::endl;
    if(!is_valid_joint_angle(q)){
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<< std::endl; 
        std::cout << "Invalid Joint Angles" << std::endl;
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<< std::endl;         
    }
    for (int i = 0; i < NrOfJoints; ++i){
        std::cout << q(i) << ",\t";
    }    
    std::cout<< std::endl;
}; 

void SerialLink_Manipulator::print_KDL_Frame(KDL::Frame & kdlFrame){
    std::cout << std::endl;std::cout << std::endl;
    for (int i = 0; i < 4; ++i){
        for (int j = 0; j < 4; ++j) {
            double a = kdlFrame(i, j);
            if (a < 0.0001 && a > -0.001) {
                a = 0.0;
            }
            std::cout << std::setprecision(4) << a << "\t\t";
        }
        std::cout << std::endl;
    }    
    std::cout << std::endl;std::cout << std::endl;
}; 

void SerialLink_Manipulator::print_KDL_Jacobian(KDL::Jacobian & KDLjacobian){
    std::cout << std::endl;std::cout << std::endl;
    for (int i = 0; i < 6; ++i){
        for (int j = 0; j < NrOfJoints; ++j) {
            double a = KDLjacobian(i, j);
            if (a < 0.0001 && a > -0.0001) {
                a = 0.0;
            }
            std::cout << std::setprecision(4) << a << "\t";
        }
        std::cout << std::endl;
    } 
    std::cout << std::endl;std::cout << std::endl;
};

double SerialLink_Manipulator::get_collision_score(double * joints){
    // this will be updated
    return 0;
};


SerialLink_Manipulator::~SerialLink_Manipulator(){

};

}



