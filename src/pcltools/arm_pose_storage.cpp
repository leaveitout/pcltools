//
// Created by sean on 08/08/16.
//

#include "pcltools/arm_pose_storage.hpp"
#include "pcltools/fileio.hpp"


pcltools::fileio::ArmPoseStorage::ArmPoseStorage (fs::path const & pcd_directory):
    pcd_directory_ {pcd_directory} {

}


auto pcltools::fileio::ArmPoseStorage::save (fs::path const & destination_path) -> bool {
  auto root = pt::ptree {};
  root.put (PCD_DIRECTORY_NAME, pcd_directory_.string ());

  std::sort (poses_.begin (), poses_.end (),
             [] (auto const & a, auto const & b) {
               return (std::get <fs::path> (a) < std::get <fs::path> (b));
             });

  auto poses_tree = pt::ptree {};
  try {
    for (auto const & pose : poses_) {
      auto pose_node = pt::ptree {};
      pose_node.put (FILENAME_NAME, std::get <fs::path> (pose).string ());

      auto const & point_1 = std::get <1> (pose);
      auto point_1_node = pt::ptree {};
      for (auto index = 0UL; index < point_1.size (); ++index)
        addValueToList (point_1_node, point_1 (index));
      pose_node.add_child (POINT_1_NAME, point_1_node);

      auto const & point_2 = std::get <2> (pose);
      auto point_2_node = pt::ptree {};
      for (auto index = 0UL; index < point_2.size (); ++index)
        addValueToList (point_2_node, point_2 (index));
      pose_node.add_child (POINT_2_NAME, point_2_node);

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


auto pcltools::fileio::ArmPoseStorage::load (fs::path const & source_path) -> bool {
  auto root = pt::ptree {};

  try {
    pt::read_json (source_path.c_str (), root);

    pcd_directory_ = fs::path {root.get <std::string> (PCD_DIRECTORY_NAME)};

    poses_.clear ();
    // Iterate over all poses
    for (pt::ptree::value_type & pose : root.get_child (POSES_NAME)) {
      auto filename = fs::path {pose.second.get <std::string> (FILENAME_NAME)};

      auto point_1 = Eigen::Vector3f {};
      auto point_2 = Eigen::Vector3f {};

      auto index = 0UL;
      for (pt::ptree::value_type const & point_1_el : pose.second.get_child (POINT_1_NAME)) {
        point_1 (index) = point_1_el.second.get_value <float> ();
        ++index;
      }

      index = 0UL;
      for (pt::ptree::value_type const & point_2_el : pose.second.get_child (POINT_2_NAME)) {
        point_2 (index) = point_2_el.second.get_value <float> ();
        ++index;
      }

      poses_.emplace_back (filename, point_1, point_2);
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


void pcltools::fileio::ArmPoseStorage::addPose (fs::path const & file_path,
                               Eigen::Vector3f const & point_1,
                               Eigen::Vector3f const & point_2) {
  std::lock_guard <std::mutex> lock {poses_mutex_};
  poses_.emplace_back (file_path, point_1, point_2);
}


auto pcltools::fileio::ArmPoseStorage::getPcdDirectory () const noexcept -> fs::path {
  return pcd_directory_;
}


auto pcltools::fileio::ArmPoseStorage::getPoses () const -> std::vector <std::tuple <fs::path,
                                                                    Eigen::Vector3f,
                                                                    Eigen::Vector3f>> const & {
  std::lock_guard <std::mutex> lock {poses_mutex_};
  return poses_;
}
