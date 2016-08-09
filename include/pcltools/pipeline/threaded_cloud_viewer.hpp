//
// Created by sean on 05/08/16.
//

#ifndef PCL_DETECT_HANDS_THREADED_CLOUD_VIEWER_HPP
#define PCL_DETECT_HANDS_THREADED_CLOUD_VIEWER_HPP

#include <pcl/visualization/pcl_visualizer.h>
#include <mutex>
#include <boost/make_shared.hpp>
#include <thread>


namespace viz = pcl::visualization;
using namespace std::literals;

namespace pcltools {

namespace pipeline {
/**
 * This class can be used to display a buffer of point clouds at a specific refresh rate.
 */
template <typename PointType>
class ThreadedCloudViewer {

  using Cloud = pcl::PointCloud <PointType>;
  using ColorHandler = viz::PointCloudColorHandlerCustom <PointType>;

 public:
  /**
   * Constructor
   *
   * @param viewer_title    The window title.
   */
  ThreadedCloudViewer (std::string const & viewer_title) {
    viz_thread_ = std::make_shared <std::thread> (&ThreadedCloudViewer::run, this, viewer_title);
  }


  ~ThreadedCloudViewer () noexcept {
    stop ();
  }

  // Need to specify that we will


  //


  /**
   * Add a point cloud to the viewer.
   *
   * @param cloud       Point cloud to be added.
   * @param cloud_id    The id associated with the point cloud.
   * @param handler     A color handler if desired
   * @param viewport    Viewport in which cloud is to be added.
   * @return            False if the cloud exists already with id, true otherwise.
   */
  auto addPointCloud (typename Cloud::ConstPtr cloud,
                      std::string const & cloud_id = "cloud",
                      typename ColorHandler::ConstPtr handler = nullptr,
                      int viewport = 0)
  -> bool {
    if (!addUniqueCloudId (cloud_id)) {
      return false;
    } else {
      {
        std::lock_guard <std::mutex> lock {add_buffer_mutex_};
        cloud_adding_buffer_.emplace_back (cloud, handler, cloud_id, viewport);
      }
      return true;
    }
  }


  auto updatePointCloud (typename Cloud::ConstPtr cloud,
                         std::string const & cloud_id = "cloud",
                         typename ColorHandler::ConstPtr handler = nullptr)
  -> bool {
    if (!checkCloudIdExists (cloud_id)) {
      return false;
    } else {
      {
        std::lock_guard <std::mutex> lock {update_buffer_mutex_};
        cloud_update_buffer_.emplace_back (cloud, handler, cloud_id);
      }
      return true;
    }
  }


 private:
  auto run (std::string const & viewer_title) {
    cloud_viewer_ = boost::make_shared <viz::PCLVisualizer> (viewer_title);
    cloud_viewer_->initCameraParameters ();

    // Fix the camera view
    auto camera = pcl::visualization::Camera {};
    cloud_viewer_->getCameraParameters (camera);
    camera.view[1] *= -1;
    cloud_viewer_->setCameraParameters (camera);
    auto pt = pcl::PointXYZ {0, 0, 1};

    // TODO: Feature, Add keyboard and mouse callbacks

    while (!cloud_viewer_->wasStopped () && !stopping_) {
      stop_mutex_.unlock ();

      cloud_viewer_->spinOnce ();
      //      std::this_thread::sleep_for (100ms);


      while (!cloud_adding_buffer_.empty ()) {
        std::tuple <typename Cloud::ConstPtr, typename ColorHandler::ConstPtr, std::string, int>
            params_ref;
        {
          std::lock_guard <std::mutex> lock {add_buffer_mutex_};
          params_ref = cloud_adding_buffer_.front ();
          cloud_adding_buffer_.pop_front ();
        }

        auto handler = std::get <typename ColorHandler::ConstPtr> (params_ref);
        if (handler) {
          cloud_viewer_->addPointCloud <PointType> (std::get <typename Cloud::ConstPtr> (params_ref),
                                                    *handler,
                                                    std::get <std::string> (params_ref),
                                                    std::get <int> (params_ref));
        } else {
          cloud_viewer_->addPointCloud <PointType> (std::get <typename Cloud::ConstPtr> (params_ref),
                                                    std::get <std::string> (params_ref),
                                                    std::get <int> (params_ref));
        }
      }

      while (!cloud_update_buffer_.empty ()) {
        std::tuple <typename Cloud::ConstPtr, typename ColorHandler::ConstPtr, std::string>
            params_ref;
        {
          std::lock_guard <std::mutex> lock {update_buffer_mutex_};
          params_ref = cloud_update_buffer_.front ();
          cloud_update_buffer_.pop_front ();
        }

        auto handler = std::get <typename ColorHandler::ConstPtr> (params_ref);
        if (handler) {
          cloud_viewer_->updatePointCloud (std::get <typename Cloud::ConstPtr> (params_ref),
                                           *handler,
                                           std::get <std::string> (params_ref));
        } else {
          cloud_viewer_->updatePointCloud (std::get <typename Cloud::ConstPtr> (params_ref),
                                           std::get <std::string> (params_ref));
        }
      }
      stop_mutex_.lock ();
    }
    stop_mutex_.unlock ();

    cloud_viewer_->close ();
  }


  auto stop () {
    {
      std::lock_guard <std::mutex> lock {stop_mutex_};
      stopping_ = true;
    }
    if (viz_thread_->joinable ())
      viz_thread_->join ();
  }


 private:
  auto addUniqueCloudId (std::string const & cloud_id)
  -> bool {
    auto result_iter = std::find (std::begin (cloud_ids_), std::end (cloud_ids_), cloud_id);
    if (result_iter == std::end (cloud_ids_)) {
      cloud_ids_.push_back (cloud_id);
      return true;
    } else {
      return false;
    }
  }


  auto checkCloudIdExists (std::string const & cloud_id) const
  -> bool {
    auto result_iter = std::find (std::begin (cloud_ids_), std::end (cloud_ids_), cloud_id);
    if (result_iter != std::end (cloud_ids_)) {
      return true;
    } else {
      return false;
    }
  }


  viz::PCLVisualizer::Ptr cloud_viewer_;
  bool stopping_ = false;
  std::mutex stop_mutex_;
  std::mutex add_buffer_mutex_;
  std::mutex update_buffer_mutex_;
  std::shared_ptr <std::thread> viz_thread_;
  std::vector <std::string> cloud_ids_;
  std::deque <std::tuple <typename Cloud::ConstPtr,
                          typename ColorHandler::ConstPtr,
                          std::string,
                          int>>
      cloud_adding_buffer_;
  std::deque <std::tuple <typename Cloud::ConstPtr, typename ColorHandler::ConstPtr, std::string>>
      cloud_update_buffer_;
};

}

}
#endif //PCL_DETECT_HANDS_THREADED_CLOUD_VIEWER_HPP
