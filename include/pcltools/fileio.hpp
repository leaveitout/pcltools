//
// Created by sean on 30/06/16.
//

#ifndef PCL_OBJECT_DETECT_LINEMOD_FILEIO_HPP
#define PCL_OBJECT_DETECT_LINEMOD_FILEIO_HPP

#include <deque>
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>


namespace fs = boost::filesystem;

namespace pcltools {
namespace fileio {

/**
 * Gets all files ending in .pcd in a folder.
 * @param pcd_dir The directory in which the .pcd files are located.
 * @param sort Sort the files according to their path (default is true).
 * @return A std::deque container of the paths.
 */
auto getPcdFilesInPath (fs::path const & pcd_dir, bool sort = true) -> std::deque <fs::path>;


/**
 * Checks whether the filepath exists and whether it is a file.
 * @param filepath The filepath to be checked.
 * @return True if filepath both exists and is a file, false otherwise.
 */
auto checkValidFile (fs::path const & filepath) -> bool;

/**
 * Checks whether the directory path exists and whether it is a directory.
 * @param dirpath The directory  path to be checked.
 * @return True if dirpath both exists and is a directory, false otherwise.
 */
auto checkValidDir (fs::path const & dirpath) -> bool;

/**
 * Replaces tilde char in a string with the value of the $HOME environment variable.
 * @param path_string A string that may contain a tilde ('~') char.
 * @return A new string with the tilde replaced with the value of the $HOME environment variable.
 */
auto expandTilde (std::string path_string) -> fs::path;

}
}

#endif //PCL_OBJECT_DETECT_LINEMOD_FILEIO_HPP

