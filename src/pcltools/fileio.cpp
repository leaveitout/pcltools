//
// Created by sean on 30/06/16.
//

#include <forward_list>
#include <queue>
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


/**
 * Get a deque of the filenames of all of the .pcd files within a directory.
 *
 * @param pcd_dir The directory that contains the .pcd files.
 * @param sort    Should the returned list be sorted lexicographically.
 * @returns       A forward_list of filenames of the .pcd files in the directory.
 * @throws        An exception of type boost::filesystem::file_system::error if the pcd_dir
 *                argument is not a valid directory.
 */
auto pcltools::fileio::getPcdFilesInPathDeque (fs::path const & pcd_dir, bool sort)
-> std::deque <fs::path> {
  auto result_list = std::deque <fs::path> {};
  try {
    for (auto const & entry : boost::make_iterator_range (fs::directory_iterator{pcd_dir})) {
      if (fs::is_regular_file (entry.status ())) {
        if (entry.path ().extension () == ".pcd") {
          // Is not sorted so can place at front of list
          result_list.emplace_back (entry.path ().filename ());
        }
      }
    }
    if (sort) {
      std::sort (result_list.begin (), result_list.end ());
    }
    return result_list;
  } catch (boost::filesystem::filesystem_error const & ex) {
    // Just return the empty list
    std::cerr << "getPcdFilesInPath passed an invalid pcd_dir argument." << std::endl;
    return result_list;
  }
}


auto pcltools::fileio::getPcdFilesInPathMap (fs::path const & pcd_dir)
-> std::map <double, fs::path> {
  auto result_set = std::map <double, fs::path>{};
  for (auto const & entry : boost::make_iterator_range (fs::directory_iterator{pcd_dir})) {
    if (fs::is_regular_file (entry.status ())) {
      if (entry.path ().extension () == ".pcd") {
        auto filename = entry.path ().stem ().string ();
        auto T_pos = filename.find_last_of ("T");
        if (T_pos != std::string::npos) {
          auto time_str = filename.substr (T_pos + 1);
          try {
            auto time = boost::lexical_cast <double> (time_str);
            result_set.emplace (time, entry);
          } catch (boost::bad_lexical_cast & e) {
            // Do nothing
          }
        }
      }
    }
  }
  return result_set;
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


auto pcltools::fileio::associateTimings (std::map <double, fs::path> const & target_set,
                                         std::vector <std::map <double,
                                                                fs::path>> const & source_sets)
-> std::queue <std::vector <fs::path>> {

  auto result_associated_paths = std::queue <std::vector <fs::path>>{};

  for (auto const & target_path : target_set) {
    auto merge_paths = std::vector <fs::path>{};
    merge_paths.emplace_back (target_path.second);

    for (auto const & source_set : source_sets) {
      auto lower = source_set.lower_bound (target_path.first);

      // Edge cases
      if (lower == source_set.end ()) {
        merge_paths.emplace_back (source_set.rbegin ()->second);
      } else if (lower == source_set.begin ()) {
        merge_paths.emplace_back (lower->second);
      } else {
        auto prev = lower;
        --prev;
        if (target_path.first - prev->first < lower->first - target_path.first)
          merge_paths.emplace_back (prev->second);
        else
          merge_paths.emplace_back (lower->second);
      }
    }
    result_associated_paths.emplace (merge_paths);
  }

  return result_associated_paths;
}


auto pcltools::fileio::associateTimingsVector (std::map <double, fs::path> const & target_set,
                                               std::vector <std::map <double,
                                                                      fs::path>> const & source_sets)
-> std::vector <std::vector <fs::path>> {

  auto result_associated_paths = std::vector <std::vector <fs::path>>{};

  for (auto const & target_path : target_set) {
    auto merge_paths = std::vector <fs::path>{};
    merge_paths.emplace_back (target_path.second);

    for (auto const & source_set : source_sets) {
      auto lower = source_set.lower_bound (target_path.first);

      // Edge cases
      if (lower == source_set.end ()) {
        merge_paths.emplace_back (source_set.rbegin ()->second);
      } else if (lower == source_set.begin ()) {
        merge_paths.emplace_back (lower->second);
      } else {
        auto prev = lower;
        --prev;
        if (target_path.first - prev->first < lower->first - target_path.first)
          merge_paths.emplace_back (prev->second);
        else
          merge_paths.emplace_back (lower->second);
      }
    }
    result_associated_paths.emplace_back (merge_paths);
  }

  return result_associated_paths;
}
