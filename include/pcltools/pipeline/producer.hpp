//
// Created by sean on 06/12/15.
//

#ifndef PCL_CLOUD_REGISTRATION_PRODUCER_HPP
#define PCL_CLOUD_REGISTRATION_PRODUCER_HPP

#include "node.hpp"
#include "buffer.hpp"

namespace pcltools {

namespace pipeline {

template <typename Type>
class Producer: public virtual Node {
 public:
  Producer (size_t id,
            std::string const & description,
            size_t buffer_size = Buffer <Type>::BUFF_SIZE_DEFAULT,
            size_t time_out_millis = Buffer <Type>::TIME_OUT_MS_DEFAULT,
            bool overwrite = Buffer <Type>::OVERWRITE_DEFAULT):
      Node {id, description} {
    buffer_ = std::make_shared <Buffer <Type>> (buffer_size, time_out_millis, overwrite);
  }


  virtual ~Producer () {};


  auto virtual getBuffer () const -> std::shared_ptr <Buffer <Type>> final {
    return buffer_;
  }


 protected:
  mutable std::shared_ptr <Buffer <Type>> buffer_;
};

}
}

#endif //PCL_CLOUD_REGISTRATION_PRODUCER_HPP
