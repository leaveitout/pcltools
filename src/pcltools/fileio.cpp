//
// Created by sean on 30/06/16.
//

#include <forward_list>
#include <iostream>
#include "pcltools/fileio.hpp"


/**
 * Get a list of the filenames of all of the .pcd files within a directory.
 *
 * @param pcd_dir The directory that contains the .pcd files.
 * @param sort    Should the returned list be sorted lexicographically.
 * @returns       A forward_list of filenames of the .pcd files in the directory.
 * @throws        An exception of type boost::filesystem::file_system::error if the pcd_dir
 *                argument is not a valid directory.
 */
auto pcltools::fileio::getPcdFilesInPath (fs::path const & pcd_dir, bool sort)
-> std::forward_list <fs::path> {
  auto result_list = std::forward_list <fs::path> {};
  try {
    for (auto const & entry : boost::make_iterator_range (fs::directory_iterator{pcd_dir})) {
      if (fs::is_regular_file (entry.status ())) {
        if (entry.path ().extension () == ".pcd") {
          // Is not sorted so can place at front of list
          result_list.emplace_front (entry.path ().filename ());
        }
      }
    }
    if (sort)
      result_list.sort ();
    return result_list;
  } catch (boost::filesystem::filesystem_error const & ex) {
    // Just return the empty list
    std::cerr << "getPcdFilesInPath passed an invalid pcd_dir argument." << std::endl;
    return result_list;
  }
}


auto pcltools::fileio::checkValidFile (fs::path const & filepath) noexcept
-> bool {
  return fs::exists (filepath) && fs::is_regular_file (filepath);
}


auto pcltools::fileio::checkValidDir (fs::path const & dirpath) noexcept
-> bool {
  return fs::exists (dirpath) && fs::is_directory (dirpath);
}


auto pcltools::fileio::expandTilde (std::string path_string) noexcept
-> fs::path {
  try {
    if (path_string.at (0) == '~')
      path_string.replace (0, 1, getenv ("HOME"));
    return fs::path{path_string};
  } catch (std::out_of_range const & ex) {
    return fs::path{};
  }
}

