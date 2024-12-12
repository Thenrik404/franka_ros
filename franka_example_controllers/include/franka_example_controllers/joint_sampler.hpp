#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl_parser/kdl_parser.hpp>
// #include <yaml-cpp/yaml.h>
#include <base_methods/StageBase.h>
// #include <urdf/model.h>

class JointSampler 
: public cpp_utils::StageBase<
    std::vector<double>, std::vector<double>>
{
public:
    JointSampler(){
        this->ThreadHandle.reset(new std::thread(&JointSampler::ThreadFunction, this));
    };
    ~JointSampler(){
        this->Terminate();
    };

    KDL::Frame cart_frame;
    KDL::Chain chain;
    KDL::Tree tree;
    std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver;
    KDL::JntArray q_init;
    std::string urdf_path = "/home/docker/catkin_ws/src/franka_ros/franka_description/robots/fr3/fr3.urdf";
    std::string joint_limits_path = "/home/docker/catkin_ws/src/franka_ros/franka_description/robots/fr3/joint_limits.yaml";
    std::string joint_positions_path = "/home/docker/catkin_ws/src/planning_basics/config/joint_positions.yaml";

    std::string base_link = "_link0";
    std::string tcp_link = "_hand_tcp";
    std::string arm_id = "fr3";

    std::vector<double> joint_positions_home, joint_positions_scan;

    bool joint_sampler_init();

// private:
    bool ProcessFunction(
        std::vector<double> &inputs,
        std::vector<double> &outputs
    );

};