//
// Created by sean on 30/08/16.
//

#include <pcltools/label_storage.hpp>


pcltools::fileio::LabelStorage::LabelStorage () {

}


pcltools::fileio::LabelStorage::LabelStorage (fs::path const & pcd_directory,
                                              size_t reserve_size)
    : pcd_directory_ {pcd_directory} {
      if (reserve_size > 0ULL)
        labels_.reserve (reserve_size);
}


auto pcltools::fileio::LabelStorage::save (fs::path const & destination_path) const -> bool {
  auto root = pt::ptree {};
  root.put (PCD_DIRECTORY_NAME, pcd_directory_.string ());

  auto labels_tree = pt::ptree {};
  try {
    for (auto const & label : labels_) {
      auto label_node = pt::ptree {};
      label_node.put (FILENAME_NAME, std::get <fs::path> (label).string ());
      label_node.put (ACTION_NAME, std::get <unsigned> (label));

      labels_tree.push_back (std::make_pair ("", label_node));
    }

    root.add_child (LABELS_NAME, labels_tree);

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


auto pcltools::fileio::LabelStorage::load (fs::path const & source_path) -> bool {
  auto root = pt::ptree {};

  try {
    pt::read_json (source_path.c_str (), root);
    pcd_directory_ = fs::path {root.get <std::string> (PCD_DIRECTORY_NAME)};

    labels_.clear ();

    // Iterate over all labels
    for (pt::ptree::value_type & label : root.get_child (LABELS_NAME)) {
      auto filename = fs::path {label.second.get <std::string> (FILENAME_NAME)};
      auto action_label = label.second.get <unsigned> (ACTION_NAME);
      labels_.emplace_back (filename, action_label);
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
    return false;
  }
}


auto pcltools::fileio::LabelStorage::getPcdDirectory () const noexcept -> fs::path const & {
  return pcd_directory_;
}


auto pcltools::fileio::LabelStorage::setPcdDirectory (fs::path const & pcd_directory) noexcept -> void {
  pcd_directory_ = pcd_directory;
}


auto pcltools::fileio::LabelStorage::addLabel (fs::path const & file_path, unsigned label) -> void {
  labels_.emplace_back (file_path, label);
}


//auto pcltools::fileio::LabelStorage::getLabelAtIndex (unsigned index) const -> std::tuple <fs::path, int> {
//  return labels_.at (index);
//}


auto pcltools::fileio::LabelStorage::setLabelAtIndex (unsigned index, unsigned label) -> void {
  std::get <unsigned> (labels_.at (index)) = label;
}


auto pcltools::fileio::LabelStorage::removeLabel () -> void {
  if (!labels_.empty ())
    labels_.pop_back ();
}
