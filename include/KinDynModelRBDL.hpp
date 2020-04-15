/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Author: Pouya Mohammadi
 *
 */

#ifndef KINDYNMODELRBDL_H
#define KINDYNMODELRBDL_H

#include <KinDynModel.hpp>
#include <rbdl/Model.h>

class KinDynModelRBDL : public KinDynModel {
public:
    //KinDynModelRBDL();
    virtual ~KinDynModelRBDL(); //override

    KinDynModelRBDL(std::string urdf_file_path,
                    std::string base_name,
                    std::string ee_name,
                    bool floating_base,
                    bool parse_model_verbose);

    virtual bool getEEJacobian(Eigen::VectorXd &conf, Eigen::MatrixXd &jacobian) const override;
    virtual bool getEEJacobian(Eigen::VectorXd &conf, Eigen::Vector3d &offset, Eigen::MatrixXd &jacobian) const override;
    virtual bool getPointJacobian() const override;
    virtual bool getEEPose(Eigen::VectorXd &conf, Eigen::Affine3d &pose) const override;
    virtual bool getPointPose() const override;
    virtual bool getCoM(Eigen::VectorXd &conf, Eigen::Vector3d &com) const override;
    virtual bool getJdotQdot() const override;
    virtual unsigned int getDofSize() const override;

private:
    Eigen::Vector3d zeros3;
    bool default_update_kinematic_model = true;
    

    // TODO make sure mutable is necessary here
    // add mutable to this so that it can be set to zero if needed
    mutable RigidBodyDynamics::Model model;
    mutable Eigen::MatrixXd local_jacobian;
    mutable Eigen::VectorXd qdot;
    mutable RigidBodyDynamics::Math::Vector3d tmp_vec3;

};


#endif /* KINDYNMODELRBDL_H */
