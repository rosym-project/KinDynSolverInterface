/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Author: Pouya Mohammadi
 *
 */

#ifndef KINDYNMODELRBDL_H
#define KINDYNMODELRBDL_H

#include <KinDynModel.hpp>
#include <rbdl/Model.h>

class KinDynModelRBDL : KinDynModel {
public:
    //KinDynModelRBDL();
    //virtual ~KinDynModelRBDL();
    KinDynModelRBDL(std::string urdf_file_path,
                    std::string base_name,
                    std::string ee_name,
                    bool floating_base);

    virtual bool getEEJacobian(Eigen::VectorXd &conf, Eigen::MatrixXd &jacobian) const override;    
    virtual bool getEEJacobian(Eigen::VectorXd &conf, Eigen::Vector3d &offset, Eigen::MatrixXd &jacobian) const override;

private:
    mutable RigidBodyDynamics::Model model;
    Eigen::Vector3d zeros3;

    // TODO make sure mutable is necessary here
    // add mutable to this so that it can be set to zero if needed
    mutable Eigen::MatrixXd local_jacobian;

};


#endif /* KINDYNMODELRBDL_H */
