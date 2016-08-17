//
// Created by sean on 06/07/16.
//

#include <pcltools/fileio.hpp>
#include "pcltools/object_pose_storage.hpp"


pcltools::fileio::ObjectPoseStorage::ObjectPoseStorage ():
    object_pcd_path_ {fs::path {""}},
    linemod_template_path_ {fs::path {""}},
    pcd_directory_ {fs::path {""}} {

}


pcltools::fileio::ObjectPoseStorage::ObjectPoseStorage (fs::path const & object_pcd_path,
                                  fs::path const & linemod_template_path,
                                  fs::path const & pcd_directory):
    object_pcd_path_ {object_pcd_path},
    linemod_template_path_ {linemod_template_path},
    pcd_directory_ {pcd_directory} {

}


auto pcltools::fileio::ObjectPoseStorage::save (fs::path const & destination_path) const
-> bool {
  auto root = pt::ptree {};
  root.put (OBJECT_PCD_PATH_NAME, object_pcd_path_.string ());
  root.put (LINEMOD_TEMPLATE_PATH_NAME, linemod_template_path_.string ());
  root.put (PCD_DIRECTORY_NAME, pcd_directory_.string ());

  auto poses_tree = pt::ptree {};
  try {
    for (auto const & pose : poses_) {
      auto pose_node = pt::ptree {};
      pose_node.put (FILENAME_NAME, std::get <fs::path> (pose).string ());

      auto const & quaternion = std::get <Eigen::Quaternionf> (pose);
      auto quaternion_node = pt::ptree {};
      addValueToList (quaternion_node, quaternion.w ());
      addValueToList (quaternion_node, quaternion.x ());
      addValueToList (quaternion_node, quaternion.y ());
      addValueToList (quaternion_node, quaternion.z ());
      pose_node.add_child (QUATERNION_NAME, quaternion_node);

      auto const & translation = std::get <Eigen::Vector3f> (pose);
      auto translation_node = pt::ptree {};
      for (auto index = 0UL; index < translation.size (); ++index)
        addValueToList (translation_node, translation (index));
      pose_node.add_child (TRANSLATION_NAME, translation_node);

      poses_tree.push_back (std::make_pair ("", pose_node));
    }

    root.add_child (POSES_NAME, poses_tree);

    pt::write_json (destination_path.c_str (), root);
    return true;
  }
  catch (pt::json_parser_error const & error) {
    std::cerr << "Unable to save the json file, the destination may not be valid or "
        "the file may be ill-formed." << std::endl;
    std::cerr << error.what () << std::endl;
    return false;
  }
}


auto pcltools::fileio::ObjectPoseStorage::load (fs::path const & source_path)
-> bool {
  auto root = pt::ptree {};

  try {
    pt::read_json (source_path.c_str (), root);
    object_pcd_path_ = fs::path {root.get <std::string> (OBJECT_PCD_PATH_NAME)};
    linemod_template_path_ = fs::path {root.get <std::string> (LINEMOD_TEMPLATE_PATH_NAME)};
    pcd_directory_ = fs::path {root.get <std::string> (PCD_DIRECTORY_NAME)};

    poses_.clear ();
    // Iterate over all poses
    for (pt::ptree::value_type & pose : root.get_child (POSES_NAME)) {
      auto filename = fs::path {pose.second.get <std::string> (FILENAME_NAME)};
      auto translation = Eigen::Vector3f {};

      auto quat_values = std::vector <float> {};
      for (pt::ptree::value_type const & quat_el : pose.second.get_child (QUATERNION_NAME)) {
        quat_values.push_back (quat_el.second.get_value <float> ());
      }

      auto quaternion = Eigen::Quaternionf {quat_values.at (0),
                                            quat_values.at (1),
                                            quat_values.at (2),
                                            quat_values.at (3)};

      auto index = 0_sz;
      for (pt::ptree::value_type const & trans_el : pose.second.get_child (TRANSLATION_NAME)) {
        translation (index) = trans_el.second.get_value <float> ();
        ++index;
      }
      poses_.emplace_back (filename, quaternion, translation);
    }
  }
  catch (pt::json_parser_error const & error) {
    std::cerr << "Unable to load the json file, the source may not be valid or "
        "the file may be ill-formed." << std::endl;
    std::cerr << error.what () << std::endl;
    return false;
  }
  catch (pt::ptree_bad_path const & error) {
    std::cerr << "Unable to parse the specified property in the json file (bad path)." << std::endl;
    std::cerr << error.what () << std::endl;
    return false;
  }
  catch (pt::ptree_bad_data const & error) {
    std::cerr << "Unable to parse the specified property in the json file (bad data)." << std::endl;
    std::cerr << error.what () << std::endl;
    return false;
  }
  catch (std::exception const & ex) {
    std::cerr << "No specified error handler for exception in json load method." << std::endl;
    std::cerr << ex.what () << std::endl;
  }
}


auto pcltools::fileio::ObjectPoseStorage::getObjectPcdPath () const
-> fs::path const & {
  return object_pcd_path_;
}


auto pcltools::fileio::ObjectPoseStorage::setObjectPcdPath (fs::path const & object_pcd_path)
-> void {
  object_pcd_path_ = object_pcd_path;
}


auto pcltools::fileio::ObjectPoseStorage::getLinemodTemplatePath () const
-> fs::path const & {
  return linemod_template_path_;
}


auto pcltools::fileio::ObjectPoseStorage::setLinemodTemplatePath (fs::path const & linemod_template_path)
-> void {
  linemod_template_path_ = linemod_template_path;
}


auto pcltools::fileio::ObjectPoseStorage::getPcdDirectory () const
-> fs::path const & {
  return pcd_directory_;
}


auto pcltools::fileio::ObjectPoseStorage::setPcdDirectory (fs::path const & pcd_directory)
-> void {
  pcd_directory_ = pcd_directory;
}


auto pcltools::fileio::ObjectPoseStorage::getPoses () const
-> std::vector <std::tuple <fs::path,
                            Eigen::Quaternionf,
                            Eigen::Vector3f>> const & {
  return poses_;
}


auto pcltools::fileio::ObjectPoseStorage::addPose (fs::path const & file_path,
                               Eigen::Quaternionf const & quaternion,
                               Eigen::Vector3f const & translation)
-> void {
  poses_.emplace_back (file_path, quaternion, translation);
}


auto pcltools::fileio::getObjectPoseAtIndex (pcltools::fileio::ObjectPoseStorage const &storage,
                                               unsigned index) -> std::tuple <boost::filesystem::path,
                                                                              Eigen::Quaternionf,
                                                                              Eigen::Vector3f> {
  return storage.getPoses ().at(index);
}
