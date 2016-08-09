//
// Created by sean on 17/07/16.
//

#ifndef PCL_TOOLS_BUFFER_HPP
#define PCL_TOOLS_BUFFER_HPP

#include <memory>
#include <condition_variable>
#include <boost/circular_buffer.hpp>
#include <boost/optional.hpp>


namespace pcltools {

namespace pipeline {


template <typename Type>
class Buffer {

 public:
  using BufferElement = std::unique_ptr <Type>;

  static constexpr auto TIME_OUT_MS_DEFAULT = 10UL;
  static constexpr auto BUFF_SIZE_DEFAULT = 100UL;
  static constexpr auto OVERWRITE_DEFAULT = false;


  /**
   * Constructor of a templated buffer for use in MPMC patterns.
   *
   * @param   buff_size   This is desired buffer size.
   * @param   time_out_ms This is the amount of milliseconds that the popFront method should wait
   *                      before returning if no element available.
   * @param   overwrite   Should the pushBack method replace the element at the back of the buffer.
   *
   * @throws  An allocation error if memory is exhausted, (std::bad_alloc if the standard allocator
   *          is used).
   *          Whatever Type::Type (Type const &) throws.
   */
  Buffer (size_t buff_size = BUFF_SIZE_DEFAULT,
          int time_out_ms = TIME_OUT_MS_DEFAULT,
          bool overwrite = OVERWRITE_DEFAULT):
      time_out_ms_ {time_out_ms},
      overwrite_ {overwrite} {
    std::lock_guard <std::mutex> lock (mutex_);
    buffer_.set_capacity (buff_size);
  }


  /**
   *
   * @return
   */
  auto isFull () const noexcept {
    std::lock_guard <std::mutex> lock (mutex_);
    return (buffer_.full ());
  }


  auto isEmpty () const noexcept {
    std::lock_guard <std::mutex> lock (mutex_);
    return (buffer_.empty ());
  }


  auto getSize () const noexcept {
    std::lock_guard <std::mutex> lock (mutex_);
    return (buffer_.size ());
  }


  auto getCapacity () const noexcept {
    return (buffer_.capacity ());
  }


  /**
   * Push an element in to buffer.
   *
   * @param new_item  The object to be added to the back of the buffer.
   * @return True indicates that an overwrite of a buffer element took place, otherwise false.
   */
  auto pushBack (std::unique_ptr <Type> new_item) noexcept
  -> bool {
    bool did_overwrite = false;
    if (!overwrite_) {
      std::unique_lock <std::mutex> lock (mutex_);
      if (buffer_.full ()) {
        // Wait until we get the signal the buffer is no longer full
        // Predicate to protect against spurious wake-ups
        not_full_condition_.wait (lock, [&] () { return !buffer_.full (); });
      }
      buffer_.push_back (std::move(new_item));
    } else {
      {
        std::lock_guard <std::mutex> lock (mutex_);
        if (buffer_.full ())
          did_overwrite = true;
        buffer_.push_back (std::move(new_item));
      }
    }
    not_empty_condition.notify_all ();
    return did_overwrite;
  }


  /**
   * Get the front element.
   *
   * @return Pointer to front element, may be null if buffer empty for duration of timeout.
   */
  auto popFront () noexcept
  -> std::unique_ptr <Type> {
    auto front_item = std::unique_ptr <Type> {};
    {
      std::unique_lock <std::mutex> lock (mutex_);
      if (not_empty_condition.wait_for (lock,
                                        time_out_ms_,
                                        [&] () { return !buffer_.empty (); })) {
        front_item = std::move(buffer_.front ());
        buffer_.pop_front ();
      }
    }
    not_full_condition_.notify_one ();
    return front_item;
  }


 private:
  boost::circular_buffer <std::unique_ptr<Type>> buffer_;
  mutable std::mutex mutex_;
  std::condition_variable not_empty_condition;
  std::condition_variable not_full_condition_;
  std::chrono::duration <int, std::milli> time_out_ms_;
  bool overwrite_;

  // Non-copyable
  Buffer (Buffer const &) = delete;
  Buffer & operator = (Buffer const &) = delete;
};

}
}
#endif //PCL_TOOLS_BUFFER_HPP

