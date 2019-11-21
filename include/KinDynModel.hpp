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


};

#endif // KINDYNMODEL_HPP
