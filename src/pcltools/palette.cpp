//
// Created by sean on 15/08/16.
//

#include "pcltools/palette.hpp"


pcltools::RGBColor::RGBColor (unsigned char r, unsigned char g, unsigned char b) {
  rgb_.at (0) = r;
  rgb_.at (1) = g;
  rgb_.at (2) = b;
}


auto pcltools::RGBColor::getAt (unsigned index) const -> unsigned char {
  return rgb_.at (index);
}


auto pcltools::RGBColor::r () const -> unsigned char {
  return getAt (0);
}


auto pcltools::RGBColor::g () const -> unsigned char {
  return getAt (1);
}


auto pcltools::RGBColor::b () const -> unsigned char {
  return getAt (2);
}


auto pcltools::RGBColor::setAt (unsigned index, unsigned char channel_value) -> void {
  rgb_.at (index) = channel_value;
}


pcltools::Palette::Palette (unsigned num_colors, unsigned char value, unsigned char saturation) {
  cv::Scalar default_color (0, saturation, value);
  cv::Mat palette (1, num_colors, CV_8UC3, default_color);

  for (int i = 0; i < num_colors; i++) {
    uchar hue = static_cast<uchar>(round (static_cast<double>(i * 180) / num_colors));
    palette.at <cv::Vec3b> (cv::Point (i, 0))[0] = hue;
  }

  cv::cvtColor (palette, palette, CV_HSV2RGB);

  for (int i = 0; i < num_colors; i++) {
    cv::Vec3b const & pixel = palette.at <cv::Vec3b> (cv::Point (i, 0));
    auto color = RGBColor {pixel[0], pixel[1], pixel[2]};
    rgb_colors_.push_back (color);
  }

  auto size = rgb_colors_.size ();
  auto new_indices = std::vector <size_t> (size);
  auto is_set = std::vector <bool> (size);
  auto current_index = 0UL;

  for (auto & new_index : new_indices) {
    new_index = current_index;
    is_set.at (current_index) = true;
    current_index = (current_index + (size / 2)) % size;

    auto try_count = 0;
    while (is_set.at (current_index) && try_count < size) {
      if (current_index == 0)
        current_index = size - 1;
      else
        --current_index;
      ++try_count;
    }
  }

  auto rgb_colors_copy = rgb_colors_;

  auto old_index = 0UL;
  for (auto const & index : new_indices) {
    rgb_colors_.at (old_index) = rgb_colors_copy.at (index);
    ++old_index;
  }
}


auto pcltools::Palette::getColorAt (unsigned int index) -> RGBColor {
  return rgb_colors_.at (index);
}
