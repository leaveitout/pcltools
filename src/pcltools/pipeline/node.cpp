//
// Created by sean on 06/12/15.
//

#include "pcltools/pipeline/node.hpp"


pcltools::pipeline::Node::Node (size_t id, std::string const & description):
    id_ {id},
    description_ {description} {
}


auto pcltools::pipeline::Node::getDescription () const noexcept -> std::string {
  return description_;
}


auto pcltools::pipeline::Node::getId () const noexcept -> size_t {
  return id_;
}
