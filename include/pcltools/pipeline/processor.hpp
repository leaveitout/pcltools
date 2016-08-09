//
// Created by sean on 07/12/15.
//

#ifndef PCL_TOOLS_PROCESSOR_HPP
#define PCL_TOOLS_PROCESSOR_HPP


#include <memory>
#include <thread>
#include <sstream>
#include <iostream>
#include <boost/log/trivial.hpp>

#include "buffer.hpp"
#include "node.hpp"

namespace pcltools {

namespace pipeline {

// TODO: Logging
template <typename Type>
class Processor: public virtual Node {
 public:

  /**
   * @throws std::invalid_argument exception if buffer is nullptr.
   */
  Processor (size_t id,
             std::string const & description,
             std::shared_ptr <Buffer <Type>> buffer):
      Node (id, description),
      buffer_ (buffer),
      is_stopping_ (false),
      is_started_ (false) {
    if (!buffer) {
      auto ss = std::stringstream {};
      ss << "Processor " << id << " passed invalid buffer argument, nullptr.";
      throw std::invalid_argument {ss.str ()};
    }
  }


  auto virtual start ()
  -> void override final {
    is_stopping_ = false;
    is_started_ = true;
    //    std::stringstream ss;
    //    ss << "Processor " << id_ << ", " << description_ << "";
    //    timer_.reset (new Timer (ss.str ()));
    thread_ = std::make_unique <std::thread> (&Processor::run, this);
  }


  auto virtual stop ()
  -> void override final {
    this->stop (false);
  }


  /**
   * @param process_remaining Process all the elements remaining in buffer before joining thread.
   */
  auto virtual stop (bool process_remaining)
  -> void final {
    if (is_started_) {
      {
        std::lock_guard <std::mutex> lock (mutex_);
        is_stopping_ = true;
        process_remaining_ = process_remaining;
      }
      thread_->join ();
      is_started_ = false;
      std::stringstream ss;
      ss << "Processor " << id_ << ", " << description_ << " done." << std::endl;
      std::cout << ss.str ();
    }
  }


 protected:

  /**
   * If there are resources that need to be allocated on the processing thread for subclasses,
   * they should be allocated using this method and not in the constructor to ensure that they
   * exist on the same thread.
   */
  virtual void initializeThreadResources () {};


  /**
   * Subclasses need to implement this method to process the element.
   * @param element The current element of the buffer to be processed in the processing thread.
   */
  virtual void processBufferElement (Type & element) = 0;


 private:

  void run () {
    // TODO: Change to a call_once inside getElementAndProcess, allowing inspection of first element
    initializeThreadResources ();
    mutex_.lock ();
    while (!is_stopping_) {
      mutex_.unlock ();
      getElementAndProcess ();
      mutex_.lock ();
    }
    mutex_.unlock ();

    //    if (!is_detached_) {
    //      std::stringstream ss;
    //      ss << "Processing remaining " << buffer_->getSize () << " elements in the buffer..."
    //          << std::endl;
    //      //            Logger::log(Logger::INFO, ss.str());
    //
    //      while (!buffer_->isEmpty ())
    //        getElementAndProcess ();
    //    }
    if (process_remaining_)
      while (!buffer_->isEmpty ())
        getElementAndProcess ();
  }


  void getElementAndProcess () {
    auto element = buffer_->popFront ();
    if (element) {
      // TODO: Should we pass along the boost::optional
      processBufferElement (*element);
#ifndef NDEBUG
      BOOST_LOG_TRIVIAL (info) << "Processing, size " << buffer_->getSize ();
#endif
    }
  }


 private:
  std::unique_ptr <std::thread> thread_;
  std::shared_ptr <Buffer <Type>> buffer_;
  bool is_started_;
  bool is_stopping_;
  bool process_remaining_;
  std::mutex mutex_;
};

}
}

#endif //PCL_TOOLS_PROCESSOR_HPP

