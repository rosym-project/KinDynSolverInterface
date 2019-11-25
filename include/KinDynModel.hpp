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
     * @brief getEEJacobian
     * @param conf
     * @param offset Point position on the ee link
     * @param jacobian Jacobian of the end-effector
     * @return
     *
     * This is similar to its sibling version but with an offset.
     * TODO: Make sure this make sense in RBDL strudcture
     */
    virtual bool getEEJacobian(Eigen::VectorXd & conf,
                               Eigen::Vector3d & offset,
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
    unsigned int base_link_id;

    /**
     * @brief ee_link_id similar to KinDynModel::base_link_id
     */
    unsigned int ee_link_id;

    /**
     * @brief is_floating_base If the robot is floating base or not
     * This should be handled with care by the implemnting classes as
     * many solvers do not suport floating base notation out of the
     * box. It should be either implemented by the developers, or the
     * user of the class must be prompted, should this parameter is
     * set to true.
     */
    bool is_floating_base;    

    /**
     * @brief model_initialized
     * Checks if the model and specified links were found and initialized.
     * If for instance, link_name or urdf are invalid, this remains false.
     * This variable can later be exploited by the factory insuring the user
     * gets its solver only everything is in order.
     */
    bool model_initialized = false;

};

#endif // KINDYNMODEL_HPP
