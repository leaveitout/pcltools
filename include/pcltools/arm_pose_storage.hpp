//
// Created by sean on 08/08/16.
//

#ifndef PCL_DETECT_HANDS_HAND_POSE_STORAGE_HPP
#define PCL_DETECT_HANDS_HAND_POSE_STORAGE_HPP

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/filesystem.hpp>
#include <Eigen/Dense>
#include <mutex>
#include <set>


namespace pt = boost::property_tree;
namespace fs = boost::filesystem;

namespace pcltools {

namespace fileio {

class ArmPoseStorage {

 private:
  constexpr static auto PCD_DIRECTORY_NAME = "basedir";
  constexpr static auto POSES_NAME = "poses";
  constexpr static auto FILENAME_NAME = "file";
  constexpr static auto POINT_1_NAME = "point1";
  constexpr static auto POINT_2_NAME = "point2";

 public:
  ArmPoseStorage (fs::path const & pcd_directory);

  auto save (fs::path const & destination_path) -> bool;

  auto load (fs::path const & source_path) -> bool;

  auto getPcdDirectory () const noexcept -> fs::path;

  auto addPose (fs::path const & file_path,
                Eigen::Vector3f const & point_1,
                Eigen::Vector3f const & point_2) -> void;

  auto getPoses () const
  -> std::vector <std::tuple <fs::path, Eigen::Vector3f, Eigen::Vector3f>> const &;

 private:
  fs::path pcd_directory_;

  // TODO: Eigen aligned allocator
  std::vector <std::tuple <fs::path, Eigen::Vector3f, Eigen::Vector3f>> poses_;
  mutable std::mutex poses_mutex_;
};

}
}


#endif //PCL_DETECT_HANDS_HAND_POSE_STORAGE_HPP
