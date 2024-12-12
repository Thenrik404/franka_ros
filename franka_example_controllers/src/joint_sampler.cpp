#include <franka_example_controllers/joint_sampler.hpp>

bool JointSampler::joint_sampler_init()
{
    if (!kdl_parser::treeFromFile(this->urdf_path, this->tree)){
        throw std::runtime_error("Failed to construct kdl tree");
        return false;
    }
    if (
        !this->tree.getChain(
            this->arm_id+this->base_link,
            this->arm_id+this->tcp_link,
            this->chain)
        )
    {
        throw std::runtime_error("Failed to construct kdl chain");
        return false;
    };

    this->ik_solver.reset(
        new KDL::ChainIkSolverPos_LMA(
            this->chain
            // 1E-5, //_eps
		    // 500, //_maxiter
		    // 1E-15 //_eps_joints
        )
    );
    // TODO: fix yaml
    // // load joint limits
    // YAML::Node config = YAML::LoadFile(this->joint_limits_path);
    // for(int i = 1; i<=7; i++){
    //     std::string key = this->arm_id+"_joint" + std::to_string(i);
    //     std::cout<<key<<std::endl;
    //     std::cout<<config["home"][key].as<double>()<<std::endl;
    //     // this->joint_positions_home.push_back(config["home"][key].as<double>());        
    //     // this->joint_positions_scan.push_back(config["charuco_scan"][key].as<double>());        
    // }

    this->joint_positions_home = std::vector<double> {
        -0.010905314150772269,
        -1.3490079706152414,
        -0.03299393789114348,
        -1.8988035002999315,
        -0.004569867948255371,
        1.575852691303458,
        0.8298735693704054
    };
    this->q_init = KDL::JntArray(7);
    for (std::size_t i = 0; i<7; i++){
        this->q_init(i) = joint_positions_home.at(i);
    }

    return true;
}


bool JointSampler::ProcessFunction(
    std::vector<double> &inputs,
    std::vector<double> &outputs
){
    outputs.clear();
    KDL::Vector vector(
        inputs.at(0),
        inputs.at(1),
        inputs.at(2)
    );
    KDL::Rotation rotation = KDL::Rotation::Quaternion(
        inputs.at(3),
        inputs.at(4),
        inputs.at(5),
        inputs.at(6)
    );
    this->cart_frame = KDL::Frame(rotation, vector);
    KDL::JntArray q_out(7);
    this->ik_solver->CartToJnt(this->q_init, this->cart_frame, q_out);
    for (std::size_t i = 0; i<7; i++){
        outputs.push_back(
            static_cast<double> (q_out(i))
        );
    }
    return true;
}
