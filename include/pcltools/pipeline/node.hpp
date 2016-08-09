//
// Created by sean on 06/12/15.
//

#ifndef PCL_TOOLS_NODE_HPP
#define PCL_TOOLS_NODE_HPP

#include <string>

namespace pcltools {

namespace pipeline {

class Node {
 public:
  Node(size_t id, std::string const & description);

  virtual ~Node() noexcept {};

  virtual void start() = 0;

  virtual void stop() = 0;

  auto getDescription () const noexcept -> std::string;

  auto getId () const noexcept -> size_t;

 protected:
  size_t id_;
  std::string description_;
};

}
}

#endif //PCL_TOOLS_NODE_HPP

