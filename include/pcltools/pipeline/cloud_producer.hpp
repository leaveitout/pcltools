//
// Created by sean on 09/08/16.
//

#ifndef PCL_TOOLS_CLOUD_PRODUCER_HPP
#define PCL_TOOLS_CLOUD_PRODUCER_HPP

#include <thread>
#include <pcl/point_cloud.h>
#include "pcltools/pipeline/producer.hpp"
#include "pcltools/fileio.hpp"


namespace pcltools {

namespace pipeline {

template <typename PointType>
class CloudProducer:
    public Producer < std::tuple <typename pcl::PointCloud <PointType>::Ptr, fs::path>> {
  //                                      pcl::PointCloud <PointType>::Ptr> {

 public:

  typedef typename pcl::PointCloud <PointType> Cloud;
  typedef  std::tuple <typename pcl::PointCloud <PointType>::Ptr, fs::path> CloudPathPair;

  static constexpr auto DEFAULT_DESCRIPTION = "Cloud Producer";


  CloudProducer (fs::path const & pcd_directory,
                 size_t id,
                 std::string const & description = DEFAULT_DESCRIPTION,
                 size_t buffer_size = 50):
      Node (id, description),
      Producer < CloudPathPair> (id, description, buffer_size),
      base_directory_ (pcd_directory) {
    pcd_files_ = pcltools::fileio::getPcdFilesInPath (pcd_directory);
  }


  virtual void start () override {
    thread_ = std::make_unique <std::thread> (&CloudProducer::run, this);
    //    thread_ = std::make_unique <std::thread> ([=] {run ();});
  }


  virtual void stop () override {
    std::lock_guard <std::mutex> lock {stop_mutex_};
    is_stopping_ = true;

    if (thread_->joinable ())
      thread_->join ();
  }


  /**
   * Produce all elements before stopping.
   */
  void processAndStop () {
    if (thread_->joinable ())
      thread_->join ();
  }


 private:

  auto run () -> void {
    for (auto const & pcd_file : pcd_files_) {
      try {
        auto full_path = base_directory_ / pcd_file;
        auto element = std::make_unique < CloudPathPair> (
            std::make_tuple (pcltools::fileio::loadCloud <PointType> (full_path), pcd_file)
        );
        this->buffer_->pushBack (std::move (element));
      }
      catch (std::exception const & ex) {
        // TODO: Need to pass the exception between threads
      }

      std::lock_guard <std::mutex> lock {stop_mutex_};
      if (is_stopping_) {
        break;
      }
    }
  }


  fs::path base_directory_;
  std::forward_list <fs::path> pcd_files_;
  bool is_stopping_ = false;
  std::unique_ptr <std::thread> thread_;
  std::mutex stop_mutex_;
};

}
}

#endif //PCL_TOOLS_CLOUD_PRODUCER_HPP

