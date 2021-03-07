#ifndef COLOR_DETECTOR_PARAMS_HSV_H_
#define COLOR_DETECTOR_PARAMS_HSV_H_

#include <ros/node_handle.h>

#include <algorithm>
#include <string>
#include <vector>

#include "color_detector_params/HsvConfig.h"

namespace color_detector_params_hsv {
struct HSV {
    int h, s, v;
};

struct ThresholdHSV {
    HSV lower, upper;
};

inline void init(std::vector<std::string> &colors, std::vector<ThresholdHSV> &config_hsvs) {
    colors = {"green", "red", "blue", "yellow", "gold", "black", "white"};
    config_hsvs.resize(colors.size());
}

inline void update_use_colors(const std::vector<std::string> &colors, const color_detector_params::HsvConfig &config,
                       std::vector<bool> &use_colors) {
    for (size_t i = 0; i < colors.size(); i++) {
        if (colors[i] == "red") use_colors[i] = config.red_enable;
        if (colors[i] == "blue") use_colors[i] = config.blue_enable;
        if (colors[i] == "green") use_colors[i] = config.green_enable;
        if (colors[i] == "yellow") use_colors[i] = config.yellow_enable;
        if (colors[i] == "gold") use_colors[i] = config.gold_enable;
        if (colors[i] == "black") use_colors[i] = config.black_enable;
        if (colors[i] == "white") use_colors[i] = config.white_enable;
    }
}

inline void update_hsv_params(const std::vector<std::string> &colors, const color_detector_params::HsvConfig &config,
                       std::vector<ThresholdHSV> &config_hsvs) {
    for (size_t i = 0; i < colors.size(); i++) {
        if (colors[i] == "red") {
            config_hsvs[i].lower.h = config.LOWER_RED_H;
            config_hsvs[i].lower.s = config.LOWER_RED_S;
            config_hsvs[i].lower.v = config.LOWER_RED_V;
            config_hsvs[i].upper.h = config.UPPER_RED_H;
            config_hsvs[i].upper.s = config.UPPER_RED_S;
            config_hsvs[i].upper.v = config.UPPER_RED_V;
        } else if (colors[i] == "green") {
            config_hsvs[i].lower.h = config.LOWER_GREEN_H;
            config_hsvs[i].lower.s = config.LOWER_GREEN_S;
            config_hsvs[i].lower.v = config.LOWER_GREEN_V;
            config_hsvs[i].upper.h = config.UPPER_GREEN_H;
            config_hsvs[i].upper.s = config.UPPER_GREEN_S;
            config_hsvs[i].upper.v = config.UPPER_GREEN_V;
        } else if (colors[i] == "blue") {
            config_hsvs[i].lower.h = config.LOWER_BLUE_H;
            config_hsvs[i].lower.s = config.LOWER_BLUE_S;
            config_hsvs[i].lower.v = config.LOWER_BLUE_V;
            config_hsvs[i].upper.h = config.UPPER_BLUE_H;
            config_hsvs[i].upper.s = config.UPPER_BLUE_S;
            config_hsvs[i].upper.v = config.UPPER_BLUE_V;
        } else if (colors[i] == "yellow") {
            config_hsvs[i].lower.h = config.LOWER_YELLOW_H;
            config_hsvs[i].lower.s = config.LOWER_YELLOW_S;
            config_hsvs[i].lower.v = config.LOWER_YELLOW_V;
            config_hsvs[i].upper.h = config.UPPER_YELLOW_H;
            config_hsvs[i].upper.s = config.UPPER_YELLOW_S;
            config_hsvs[i].upper.v = config.UPPER_YELLOW_V;
        } else if (colors[i] == "gold") {
            config_hsvs[i].lower.h = config.LOWER_GOLD_H;
            config_hsvs[i].lower.s = config.LOWER_GOLD_S;
            config_hsvs[i].lower.v = config.LOWER_GOLD_V;
            config_hsvs[i].upper.h = config.UPPER_GOLD_H;
            config_hsvs[i].upper.s = config.UPPER_GOLD_S;
            config_hsvs[i].upper.v = config.UPPER_GOLD_V;
        } else if (colors[i] == "black") {
            config_hsvs[i].lower.h = config.LOWER_BLACK_H;
            config_hsvs[i].lower.s = config.LOWER_BLACK_S;
            config_hsvs[i].lower.v = config.LOWER_BLACK_V;
            config_hsvs[i].upper.h = config.UPPER_BLACK_H;
            config_hsvs[i].upper.s = config.UPPER_BLACK_S;
            config_hsvs[i].upper.v = config.UPPER_BLACK_V;
        } else if (colors[i] == "white") {
            config_hsvs[i].lower.h = config.LOWER_WHITE_H;
            config_hsvs[i].lower.s = config.LOWER_WHITE_S;
            config_hsvs[i].lower.v = config.LOWER_WHITE_V;
            config_hsvs[i].upper.h = config.UPPER_WHITE_H;
            config_hsvs[i].upper.s = config.UPPER_WHITE_S;
            config_hsvs[i].upper.v = config.UPPER_WHITE_V;
        }
    }
}

}  // namespace color_detector_params_hsv

#endif  // COLOR_DETECTOR_PARAMS_HSV_H_
