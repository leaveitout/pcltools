//
// Created by sean on 06/07/16.
//

#ifndef PCL_OBJECT_DETECT_BATCH_MODE_POSE_FILE_STORAGE_HPP
#define PCL_OBJECT_DETECT_BATCH_MODE_POSE_FILE_STORAGE_HPP

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/filesystem.hpp>
#include <Eigen/Dense>
#include <pcltools/common.hpp>


using namespace pcltools::literals;

namespace pt = boost::property_tree;
namespace fs = boost::filesystem;

namespace pcltools {

namespace fileio {

/**
 * Class to input and output to a json file the detected poses of an object in pcd files.
 */
class ObjectPoseStorage {

 private:
  constexpr static auto OBJECT_PCD_PATH_NAME = "object";
  constexpr static auto LINEMOD_TEMPLATE_PATH_NAME = "linemod";
  constexpr static auto PCD_DIRECTORY_NAME = "basedir";
  constexpr static auto POSES_NAME = "poses";
  constexpr static auto FILENAME_NAME = "file";
  constexpr static auto QUATERNION_NAME = "quat";
  constexpr static auto TRANSLATION_NAME = "trans";

 public:
  ObjectPoseStorage ();

  ObjectPoseStorage (fs::path const & object_pcd_path,
                     fs::path const & linemod_template_path,
                     fs::path const & pcd_directory);

  auto save (fs::path const & destination_path) const -> bool;

  auto load (fs::path const & source_path) -> bool;

  auto getObjectPcdPath () const -> fs::path const &;

  auto setObjectPcdPath (fs::path const & object_pcd_path) -> void;

  auto getLinemodTemplatePath () const -> fs::path const &;

  auto setLinemodTemplatePath (fs::path const & linemod_template_path) -> void;

  auto getPcdDirectory () const -> fs::path const &;

  auto setPcdDirectory (fs::path const & pcd_directory) -> void;

  auto getPoses () const -> std::vector <std::tuple <fs::path,
                                                     Eigen::Quaternionf,
                                                     Eigen::Vector3f>> const &;

  auto addPose (fs::path const & file_path,
                Eigen::Quaternionf const & quaternion,
                Eigen::Vector3f const & translation) -> void;

 private:
  fs::path object_pcd_path_;
  fs::path linemod_template_path_;
  fs::path pcd_directory_;
  std::vector <std::tuple <fs::path, Eigen::Quaternionf, Eigen::Vector3f>> poses_;
};

/**
 * Get the pose at the index.
 *
 * @param storage
 * @param index
 * @return pose at the index.
 * @throw std::out_of_range if index out of range.
 */
auto getObjectPoseAtIndex (ObjectPoseStorage const & storage, unsigned index)
-> std::tuple <fs::path, Eigen::Quaternionf, Eigen::Vector3f>;

}
}

#endif //PCL_OBJECT_DETECT_BATCH_MODE_POSE_FILE_STORAGE_HPP
