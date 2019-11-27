/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Author: Pouya Mohammadi
 *
 */


//#include <KinDynModelRBDL.hpp>
#include "KinDynModelRBDL.hpp"
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <iostream>
#include <rbdl/Kinematics.h>


KinDynModelRBDL::KinDynModelRBDL(std::string urdf_file_path,
                                 std::string base_name,
                                 std::string ee_name,
                                 bool floating_base) {

    this->zeros3 = Eigen::Vector3d::Zero(3);

    if(!RigidBodyDynamics::Addons::URDFReadFromFile(urdf_file_path.c_str(),
                                                    & model,
                                                    floating_base,
                                                    true)) {
        std::cerr << "Problem loading the model at the path: " << urdf_file_path <<std::endl;
        // abort();
        // FIXME
        // or maybe add something like model_initialized=false
    } else {
        this->model_initialized = true;
    }

    this->dof_size = model.dof_count;
    // This local matrix is used to pass to the solver. The rational is that
    // RBDL returns [Jw, JV]^T (top rows angular velocity, bottom 3 rows linear
    // velocity. We use this so that we can reoder them to [Jv, Jw]^T when
    // sending them back to user
    this->local_jacobian = Eigen::MatrixXd::Zero(6,dof_size);



    this->base_link_name = base_name;
    this->ee_link_name   = ee_name;
    this->base_link_id   = model.GetBodyId(base_name.c_str());
    this->ee_link_id     = model.GetBodyId(ee_name.c_str());

    if (base_link_id == std::numeric_limits<unsigned int>::max()) {
        std::cerr << "Base link \""<< base_link_name<<"\" could not be found" <<std::endl;
        this->model_initialized = false;
    }
    if (ee_link_id   == std::numeric_limits<unsigned int>::max()) {
        std::cerr << "EE link \""<< ee_link_name<<"\" could not be found" <<std::endl;
        this->model_initialized = false;
    }
}

bool KinDynModelRBDL::getEEJacobian(Eigen::VectorXd &conf, Eigen::MatrixXd &jacobian) const {
    //rbdl asks for a matrix that is all set to zero so the next line. But the code says otherwise...
    //local_jacobian.setZero(6,dof_size);
    RigidBodyDynamics::CalcPointJacobian6D(model,conf,ee_link_id,zeros3,local_jacobian,false);

    //reorder the returned jacobian
    jacobian.topRows<3>()=local_jacobian.bottomRows<3>();
    jacobian.bottomRows<3>()=local_jacobian.topRows<3>();
    return true;
}

KinDynModelRBDL::~KinDynModelRBDL(){

}

bool KinDynModelRBDL::getEEJacobian(Eigen::VectorXd &conf, Eigen::Vector3d &offset, Eigen::MatrixXd &jacobian) const {
    //rbdl asks for a matrix that is all set to zero so the next line. But the code says otherwise...
    //local_jacobian.setZero(6,dof_size);
    RigidBodyDynamics::CalcPointJacobian6D(model,conf,ee_link_id,offset,local_jacobian,false);

    //reorder the returned jacobian
    jacobian.topRows<3>()=local_jacobian.bottomRows<3>();
    jacobian.bottomRows<3>()=local_jacobian.topRows<3>();
    return true;
}
unsigned int KinDynModelRBDL::getDofSize() const {
    return this->dof_size;
}
