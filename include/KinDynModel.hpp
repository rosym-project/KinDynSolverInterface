/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Author: Pouya Mohammadi
 *
 */

#ifndef KINDYNMODEL_HPP
#define KINDYNMODEL_HPP

#include <string>


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
