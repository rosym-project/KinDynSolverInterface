#ifndef KINDYNMODELFACTORY_HPP
#define KINDYNMODELFACTORY_HPP

#include <KinDynModelRBDL.hpp>
#include <iostream>


typedef enum {
    RBDL,
    KDL,
    IDYNTREE
} KinDynSolvers;


class KinDynModelFactory{
public:
    static KinDynModel* initialize(std::string urdf_file_path,
                                   std::string base_name,
                                   std::string ee_name,
                                   bool floating_base,
                                   int kin_dyn_solver = KinDynSolvers::RBDL);
};


#endif // KINDYNMODELFACTORY_HPP
