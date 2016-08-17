//
// Created by sean on 15/08/16.
//

#ifndef PCLTOOLS_PALETTE_HPP
#define PCLTOOLS_PALETTE_HPP

#include <vector>
#include <array>

#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

namespace pcltools {

class RGBColor {
 public:
  RGBColor (unsigned char r, unsigned char g, unsigned char b);

  auto getAt(unsigned index) const -> unsigned char;

  auto setAt (unsigned index, unsigned char channel_value) -> void;

  auto r() const -> unsigned char;

  auto g() const -> unsigned char;

  auto b() const -> unsigned char;

 private:
  std::array<unsigned char, 3> rgb_;
};

class Palette {
 private:

 public:
  Palette(unsigned num_colors,
          unsigned char value = 200,
          unsigned char saturation = 200);

  auto getColorAt (unsigned int index) -> RGBColor;

 private:
  std::vector<RGBColor> rgb_colors_;

};

}
#endif //PCLTOOLS_PALETTE_HPP
