/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Author: Pouya Mohammadi
 *
 */

#ifndef KINDYNMODEL_HPP
#define KINDYNMODEL_HPP

#include <string>
#include <Eigen/Core>

/**
 * @brief The KinDynModel class
 * This is pure abstract class that acts as interface factory. The idea is that
 * its pure abstract methods are implemented in classes that extend it. These
 * classes will implement said functionality using different kinematic/dynamic
 * solvers, e.g., RBDL, KDL, etc.
 *
 * ##CONVENTIONS##
 * Although compiler optimizations makes the return by copy very efficient, since
 * most libraries still prefer to return the results by reference via argument,
 * I will adhere to the latter. This means, for operations it might be necessary
 * to do some internal work to make it happen. RBDL for instance, is a bit
 * inconsistent in this regards.
 */
class KinDynModel {
public:
//    KinDynModel();

    virtual ~KinDynModel() = 0;

    /**
     * @brief getEEJacobian
     * @param conf
     * @param[out] jacobian
     * @return Jacobian of the end-effector
     */
    virtual bool getEEJacobian(Eigen::VectorXd & conf,
                               Eigen::MatrixXd & jacobian) const = 0;


    /**
     * @brief getCoM
     * @param conf The configuration of the robot
     * @param[out] com
     * @return Center of the mass at conf
     */
    virtual bool getCoM(Eigen::Vector3d & conf,
                        Eigen::Vector3d & com) const = 0;

    /**
     * @brief getEEPose
     * @param conf
     * @param[out] pose
     * @return 4x4 homogenous transformation between the base and the EE
     */
    virtual bool getEEPose(Eigen::VectorXd & conf,
                           Eigen::Matrix4d & pose) const = 0;


    virtual bool getPointJacobian() const = 0;
    virtual bool getJdotQdot() const = 0;
    virtual bool getPointPose() const = 0;



protected:
    /**
     * @brief base_link_name
     */
    std::string base_link_name;

    /**
     * @brief ee_link_name End-effector link of the robot/chain
     */
    std::string ee_link_name;

    /**
     * @brief base_link_id mirrors KinDynModel::base_link_name
     * The idea is that if the back-end solver offers access by name and by id,
     * we use the latter for better performance.
     */
    int base_link_id;

    /**
     * @brief ee_link_id similar to KinDynModel::base_link_id
     */
    int ee_link_id;


};

#endif // KINDYNMODEL_HPP
