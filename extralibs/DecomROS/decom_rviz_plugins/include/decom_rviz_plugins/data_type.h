/**
 * @file data_type.h
 * @brief Defines all data types used in this lib

 * Mostly alias from Eigen Library.
 */

#include <stdio.h>
#include <math.h>
#include <limits>
#include <vector>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

///Set red font in printf funtion
#ifndef ANSI_COLOR_RED
#define ANSI_COLOR_RED "\x1b[1;31m"
#endif
///Set green font in printf funtion
#ifndef ANSI_COLOR_GREEN
#define ANSI_COLOR_GREEN "\x1b[1;32m"
#endif
///Set yellow font in printf funtion
#ifndef ANSI_COLOR_YELLOW
#define ANSI_COLOR_YELLOW "\x1b[1;33m"
#endif
///Set blue font in printf funtion
#ifndef ANSI_COLOR_BLUE
#define ANSI_COLOR_BLUE "\x1b[1;34m"
#endif
///Set magenta font in printf funtion
#ifndef ANSI_COLOR_MAGENTA
#define ANSI_COLOR_MAGENTA "\x1b[1;35m"
#endif
///Set cyan font in printf funtion
#ifndef ANSI_COLOR_CYAN
#define ANSI_COLOR_CYAN "\x1b[1;36m"
#endif
///Reset font color in printf funtion
#ifndef ANSI_COLOR_RESET
#define ANSI_COLOR_RESET "\x1b[0m"
#endif

#ifndef DATA_TYPE_H
#define DATA_TYPE_H
/*! \brief Rename the float type used in lib

    Default is set to be double, but user can change it to float.
*/
typedef double decimal_t;

///Pre-allocated std::vector for Eigen using vec_E
template <typename T>
using vec_E = std::vector<T, Eigen::aligned_allocator<T>>;
///Eigen 1D float vector
template <int N>
using Vecf = Eigen::Matrix<decimal_t, N, 1>;
///Eigen 1D int vector
template <int N>
using Veci = Eigen::Matrix<int, N, 1>;
///MxN Eigen matrix
template <int M, int N>
using Matf = Eigen::Matrix<decimal_t, M, N>;
///MxN Eigen matrix with M unknown
template <int N>
using MatDNf = Eigen::Matrix<decimal_t, Eigen::Dynamic, N>;
///Vector of Eigen 1D float vector
template <int N>
using vec_Vecf = vec_E<Eigen::Matrix<decimal_t, N, 1>>;
///Vector of Eigen 1D int vector
template <int N>
using vec_Veci = vec_E<Veci<N>>;

#endif

#ifndef EIGEN_QUAT
#define EIGEN_QUAT
///Allias of Eigen::Quaterniond
typedef Eigen::Quaternion<decimal_t> Quatf;
#endif

#ifndef EIGEN_EPSILON
#define EIGEN_EPSILON
///Compensate for numerical error
constexpr decimal_t epsilon_ = 1e-10; // numerical calculation error
#endif
