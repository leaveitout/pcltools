//
// Created by sean on 30/08/16.
//

#ifndef PCL_LABEL_TOOL_LABEL_STORAGE_HPP
#define PCL_LABEL_TOOL_LABEL_STORAGE_HPP

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/filesystem.hpp>


namespace pt = boost::property_tree;
namespace fs = boost::filesystem;


namespace pcltools {

namespace fileio {

class LabelStorage {

 private:
  constexpr static auto PCD_DIRECTORY_NAME = "basedir";
  constexpr static auto LABELS_NAME = "labels";
  constexpr static auto ACTION_NAME = "action";
  constexpr static auto FILENAME_NAME = "file";

 public:
  LabelStorage ();

  LabelStorage (fs::path const & pcd_directory);

  auto save (fs::path const & destination_path) const -> bool;

  auto load (fs::path const & source_path) -> bool;

  auto getPcdDirectory () const -> fs::path const &;

  auto setPcdDirectory (fs::path const & pcd_directory) -> void;

  auto addLabel (fs::path const & file_path, unsigned label) -> void;

  /**
   * Get the label at the index.
   *
   * @param index
   * @return label at the index.
   * @throw std::out_of_range if index out of range.
   */
  auto getLabelAtIndex (unsigned index)
  -> std::tuple <fs::path, int>;

  /**
   * Set the label at the index.
   *
   * @param index
   * @throw std::out_of_range if index out of range.
   */
  auto setLabelAtIndex (unsigned index, unsigned label) -> void;

 private:
  fs::path pcd_directory_;
  std::vector <std::tuple <fs::path, unsigned>> labels_;
};


}
}
#endif //PCL_LABEL_TOOL_LABEL_STORAGE_HPP
