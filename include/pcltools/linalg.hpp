//
// Created by sean on 10/07/16.
//

#ifndef PCL_OBJECT_DETECT_BATCH_MODE_LINALG_HPP
#define PCL_OBJECT_DETECT_BATCH_MODE_LINALG_HPP


#include <Eigen/Core>

#include "pcltools/common.hpp"


using namespace pcltools::literals;

namespace pcltools {

namespace linalg {

/**
 * Converts a 4x4 rigid transformation matrix to a quaternion and translation vector pair
 * @param transformation  A 4x4 matrix representing a rigid transformation.
 * @return                A quaternion and vector pair representing the passed transformation.
 */
template <typename Scalar>
auto getQuaternionAndTranslation (Eigen::Matrix <Scalar, 4, 4> const & transformation)
-> std::pair <Eigen::Quaternion <Scalar>, Eigen::Matrix <Scalar, 3, 1>> {
  auto rotation_matrix =
  Eigen::Matrix <Scalar, 3, 3> {transformation.block (0, 0, 3, 3)};
  auto translation_vector =
      static_cast <Eigen::Matrix <Scalar, 3, 1>> (transformation.block (0, 3, 3, 1));
  return std::make_pair (Eigen::Quaternion <Scalar> (rotation_matrix), translation_vector);
}


/**
 * Converts a quaternion and translation vector representing a rigid transformation to 4x4 matrix
 * @param quaternion  The quaternion representing the rigid rotation.
 * @param translation The translation represented as a vector (a 3x1 matrix).
 * @return            The 4x4 matrix that represents a rigid body transformation.
 */
template <typename Scalar>
auto getTransformationMatrix (Eigen::Quaternion <Scalar> const & quaternion,
                              Eigen::Matrix <Scalar, 3, 1> const & translation)
-> Eigen::Matrix <Scalar, 4, 4> {
  auto rotation = Eigen::Matrix <Scalar, 3, 3> {quaternion.toRotationMatrix ()};
  auto transformation = Eigen::Matrix <Scalar, 4, 4> {Eigen::Matrix <Scalar, 4, 4>::Identity ()};

  for (auto col = 0_sz; col < rotation.cols (); ++col)
    for (auto row = 0_sz; row < rotation.rows (); ++row)
      transformation (row, col) = rotation (row, col);

  for (auto index = 0_sz; index < translation.size (); ++index)
    transformation (index, transformation.cols () - 1) = translation (index);

  return transformation;
}

}
}

#endif //PCL_OBJECT_DETECT_BATCH_MODE_LINALG_HPP
