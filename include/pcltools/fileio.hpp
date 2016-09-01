//
// Created by sean on 30/06/16.
//

#ifndef PCL_OBJECT_DETECT_LINEMOD_FILEIO_HPP
#define PCL_OBJECT_DETECT_LINEMOD_FILEIO_HPP

#include <forward_list>
#include <queue>
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include <boost/make_shared.hpp>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/exceptions.hpp>


namespace fs = boost::filesystem;
namespace pt = boost::property_tree;

namespace pcltools {
namespace fileio {

/**
 * Gets all files ending in .pcd in a folder.
 * @param pcd_dir The directory in which the .pcd files are located.
 * @param sort Sort the files according to their path (default is true).
 * @return A std::forward_list container of the paths.
 */
auto getPcdFilesInPath (fs::path const & pcd_dir, bool sort = true) -> std::forward_list <fs::path>;


/**
 * Gets all files ending in .pcd in a folder.
 * @param pcd_dir The directory in which the .pcd files are located.
 * @param sort Sort the files according to their path (default is true).
 * @return A std::deque container of the paths.
 */
auto getPcdFilesInPathDeque (fs::path const & pcd_dir, bool sort = true)
-> std::deque <fs::path>;


/**
 * Gets all files ending in .pcd in a folder.
 * @param pcd_dir The directory in which the .pcd files are located.
 * @return A std::map container of the time and the path of each file.
 */
auto getPcdFilesInPathMap (fs::path const & pcd_dir)
-> std::map <double, fs::path>;


/**
 * Checks whether the filepath exists and whether it is a file.
 * @param filepath The filepath to be checked.
 * @return True if filepath both exists and is a file, false otherwise.
 */
auto checkValidFile (fs::path const & filepath) noexcept -> bool;

/**
 * Checks whether the directory path exists and whether it is a directory.
 * @param dirpath The directory  path to be checked.
 * @return True if dirpath both exists and is a directory, false otherwise.
 */
auto checkValidDir (fs::path const & dirpath) noexcept -> bool;

/**
 * Replaces tilde char in a string with the value of the $HOME environment variable.
 * @param path_string A string that may contain a tilde ('~') char.
 * @return A new string with the tilde replaced with the value of the $HOME environment variable.
 */
auto expandTilde (std::string path_string) noexcept -> fs::path;


/**
 * Associate timings from different folders of pcd recordings.
 *
 * @param target_set The timing set to target.
 * @param source_sets A vector of maps that we wish to associate with the target set.
 * @return Queue of vectors with each pcd associated with a target pcd.
 */
auto associateTimings (std::map <double, fs::path> const & target_set,
                       std::vector <std::map <double, fs::path>> const & source_sets)
-> std::queue <std::vector <fs::path>>;


/**
 * Associate timings from different folders of pcd recordings.
 *
 * @param target_set The timing set to target.
 * @param source_sets A vector of maps that we wish to associate with the target set.
 * @return Vector of vectors with each pcd associated with a target pcd.
 */
auto associateTimingsVector (std::map <double, fs::path> const & target_set,
                             std::vector <std::map <double,
                                                    fs::path>> const & source_sets)
-> std::vector <std::vector <fs::path>>;


/**
 * This method loads a pcd point cloud at the specified path.
 *
 * @param pcd_cloud_path    The path to the .pcd file.
 * @return                  A pointer to the loaded pcd cloud.
 * @throws                  std::system_error if the file does not exist or an error is encountered
 *                          loading the point cloud.
 */
template <typename PointType>
auto loadCloud (fs::path const & pcd_cloud_path)
-> typename pcl::PointCloud <PointType>::Ptr {
  // Check location is valid
  if (!pcltools::fileio::checkValidFile (pcd_cloud_path)) {
    auto ss = std::stringstream {};
    ss << "Invalid pcd filepath (" << pcd_cloud_path.string () << ") specified.";
    auto ec = std::make_error_code (std::errc::no_such_file_or_directory);
    throw std::system_error (ec, ss.str ());
  }

  // Attempt to load
  auto cloud_ptr = boost::make_shared <pcl::PointCloud <PointType>> ();
  if (pcl::io::loadPCDFile <PointType> (pcd_cloud_path.c_str (), *cloud_ptr) == -1) {
    auto ss = std::stringstream {};
    ss << "Error loading cloud (" << pcd_cloud_path.string () << ") from file specified.";
    auto error_code = std::error_code {};
    auto ec = std::make_error_code (std::errc::io_error);
    throw std::system_error (ec, ss.str ());
  }

  return cloud_ptr;
}


/**
 * Save a point cloud at the specified path as a binary compressed pcd.
 *
 * This function checks whether the cloud has positive size, whether the path is valid, before
 * attempting to save.
 *
 * @param cloud             The point cloud to be saved.
 * @param pcd_cloud_path    The path to save the cloud as a binary compressed pcd.
 * @throws                  std::system_error with error code if cloud size is empty, path is not
 *                          valid or if there is a problem attempting to save the file.
 */
template <typename PointType>
auto saveCloud (pcl::PointCloud <PointType> const & cloud, fs::path const & pcd_cloud_path)
-> void {
  // Check if cloud is empty
  if (cloud.size () == 0) {
    auto ss = std::stringstream {};
    ss << "Point cloud to save at (" << pcd_cloud_path.string () << ") size == 0.";
    auto ec = std::make_error_code (std::errc::invalid_argument);
    throw std::system_error (ec, ss.str ());
  }

  // Check location is valid
  auto canonical_path = fs::absolute (pcd_cloud_path);
  if (!pcltools::fileio::checkValidDir (canonical_path.parent_path ())) {
    auto ss = std::stringstream {};
    ss << "Invalid pcd filepath (" << pcd_cloud_path.string () << ") specified.";
    auto ec = std::make_error_code (std::errc::no_such_file_or_directory);
    throw std::system_error (ec, ss.str ());
  }

  // Attempt to save
  // TODO: Other versions than binary compressed.
  auto return_code = pcl::io::savePCDFileBinaryCompressed (canonical_path.c_str (), cloud);

  if (return_code == -1) {
    auto ss = std::stringstream {};
    ss << "Saving pcd failed for file (" << pcd_cloud_path.string () << ") specified.";
    auto ec = std::make_error_code (std::errc::io_error);
    throw std::system_error (ec, ss.str ());
  }
}


/**
 * @throws boost::property_tree::ptree_bad_data exception when the value cannot be added.
 */
template <typename ValueType>
auto addValueToList (pt::ptree & tree, ValueType const & value) {
  auto temp_node = pt::ptree {};
  try {
    temp_node.put_value (value);
  } catch (pt::ptree_bad_data const & ex) {
    throw ex;
  }
  tree.push_back (std::make_pair ("", temp_node));
}

}
}

#endif //PCL_OBJECT_DETECT_LINEMOD_FILEIO_HPP

