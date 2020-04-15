#include <KinDynModelFactory.hpp>

KinDynModel*  KinDynModelFactory::initialize(std::string urdf_file_path,
                                             std::string base_name,
                                             std::string ee_name,
                                             bool floating_base,
                                             bool parse_model_verbose,
                                             int kin_dyn_solver) {

    KinDynModel * kin_dyn = nullptr;

    switch (kin_dyn_solver) {
    case KinDynSolvers::RBDL:
        kin_dyn = new KinDynModelRBDL(urdf_file_path,base_name,ee_name,floating_base,parse_model_verbose);
        break;
    case KinDynSolvers::KDL:
        std::cerr<<"KDL implementation is still missing..."<<std::endl;
        //abort()
        break;
    case KinDynSolvers::IDYNTREE:
        std::cerr<<"iDynTree implementation is still missing..."<<std::endl;
        //abort()
        break;
    default:
        break;
    }

    return  kin_dyn;
}
