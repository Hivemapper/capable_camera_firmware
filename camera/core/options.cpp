/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * options.cpp - common program options helpers
 */

#include "core/options.hpp"

bool Options::Parse(int argc, char *argv[])
{
    using namespace boost::program_options;
    using namespace libcamera;
    variables_map vm;
    // Read options from the command line
    store(parse_command_line(argc, argv, options_), vm);
    notify(vm);
    // Read options from a file if specified
    std::ifstream ifs(config_file.c_str());
    if (ifs)
    {
      store(parse_config_file(ifs, options_), vm);
      notify(vm);
    }

    if (help)
    {
      std::cout << options_;
      return false;
    }

    if (version)
    {
      std::cout << "libcamera-apps build: " << "FIXME" << std::endl;
      std::cout << "libcamera build: " << libcamera::CameraManager::version() << std::endl;
      return false;
    }

    transform = Transform::Identity;
    if (hflip_)
      transform = Transform::HFlip * transform;
    if (vflip_)
      transform = Transform::VFlip * transform;
    bool ok;
    Transform rot = transformFromRotation(rotation_, &ok);
    if (!ok)
      throw std::runtime_error("illegal rotation value");
    transform = rot * transform;
    if (!!(transform & Transform::Transpose))
      throw std::runtime_error("transforms requiring transpose not supported");

    if (sscanf(roi.c_str(), "%f,%f,%f,%f", &roi_x, &roi_y, &roi_width, &roi_height) != 4)
      roi_x = roi_y = roi_width = roi_height = 0; // don't set digital zoom

    std::map<std::string, int> metering_table =
      { { "centre", libcamera::controls::MeteringCentreWeighted },
        { "spot", libcamera::controls::MeteringSpot },
        { "average", libcamera::controls::MeteringMatrix },
        { "matrix", libcamera::controls::MeteringMatrix },
        { "custom", libcamera::controls::MeteringCustom } };
    if (metering_table.count(metering) == 0)
      throw std::runtime_error("Invalid metering mode: " + metering);
    metering_index = metering_table[metering];

    std::map<std::string, int> exposure_table =
      { { "normal", libcamera::controls::ExposureNormal },
        { "sport", libcamera::controls::ExposureShort },
        { "short", libcamera::controls::ExposureShort },
        // long mode?
        { "custom", libcamera::controls::ExposureCustom } };
    if (exposure_table.count(exposure) == 0)
      throw std::runtime_error("Invalid exposure mode:" + exposure);
    exposure_index = exposure_table[exposure];

    std::map<std::string, int> awb_table =
      { { "auto", libcamera::controls::AwbAuto },
        { "normal", libcamera::controls::AwbAuto },
        { "incandescent", libcamera::controls::AwbIncandescent },
        { "tungsten", libcamera::controls::AwbTungsten },
        { "fluorescent", libcamera::controls::AwbFluorescent },
        { "indoor", libcamera::controls::AwbIndoor },
        { "daylight", libcamera::controls::AwbDaylight },
        { "cloudy", libcamera::controls::AwbCloudy },
        { "custom", libcamera::controls::AwbCustom } };
    if (awb_table.count(awb) == 0)
      throw std::runtime_error("Invalid AWB mode: " + awb);
    awb_index = awb_table[awb];

    if (sscanf(awbgains.c_str(), "%f,%f", &awb_gain_r, &awb_gain_b) != 2)
      throw std::runtime_error("Invalid AWB gains");

    brightness = std::clamp(brightness, -1.0f, 1.0f);
    contrast = std::clamp(contrast, 0.0f, 15.99f); // limits are arbitrary..
    saturation = std::clamp(saturation, 0.0f, 15.99f); // limits are arbitrary..
    sharpness = std::clamp(sharpness, 0.0f, 15.99f); // limits are arbitrary..

    // We have to pass the tuning file name through an environment variable.
    // Note that we only overwrite the variable if the option was given.
    if (tuning_file != "-")
      setenv("LIBCAMERA_RPI_TUNING_FILE", tuning_file.c_str(), 1);

    return true;
}

void Options::Print() const
{
    std::cout << "Options:" << std::endl;
    std::cout << "    verbose: " << verbose << std::endl;
    if (!config_file.empty())
      std::cout << "    config file: " << config_file << std::endl;
    std::cout << "    info_text:" << info_text << std::endl;
    std::cout << "    timeout: " << timeout << std::endl;
    std::cout << "    width: " << width << std::endl;
    std::cout << "    height: " << height << std::endl;
    std::cout << "    output: " << output << std::endl;
    std::cout << "    post_process_file: " << post_process_file << std::endl;
    std::cout << "    rawfull: " << rawfull << std::endl;
    std::cout << "    transform: " << transformToString(transform) << std::endl;
    if (roi_width == 0 || roi_height == 0)
      std::cout << "    roi: all" << std::endl;
    else
      std::cout << "    roi: " << roi_x << "," << roi_y << "," << roi_width << "," << roi_height << std::endl;
    if (shutter)
      std::cout << "    shutter: " << shutter << std::endl;
    if (gain)
      std::cout << "    gain: " << gain << std::endl;
    std::cout << "    metering: " << metering << std::endl;
    std::cout << "    exposure: " << exposure << std::endl;
    std::cout << "    ev: " << ev << std::endl;
    std::cout << "    awb: " << awb << std::endl;
    if (awb_gain_r && awb_gain_b)
      std::cout << "    awb gains: red " << awb_gain_r << " blue " << awb_gain_b << std::endl;
    std::cout << "    flush: " << (flush ? "true" : "false") << std::endl;
    std::cout << "    wrap: " << wrap << std::endl;
    std::cout << "    brightness: " << brightness << std::endl;
    std::cout << "    contrast: " << contrast << std::endl;
    std::cout << "    saturation: " << saturation << std::endl;
    std::cout << "    sharpness: " << sharpness << std::endl;
    std::cout << "    framerate: " << framerate << std::endl;
    std::cout << "    denoise: " << denoise << std::endl;
    std::cout << "    viewfinder-width: " << viewfinder_width << std::endl;
    std::cout << "    viewfinder-height: " << viewfinder_height << std::endl;
    std::cout << "    tuning-file: " << (tuning_file == "-" ? "(libcamera)" : tuning_file) << std::endl;
    std::cout << "    lores-width: " << lores_width << std::endl;
    std::cout << "    lores-height: " << lores_height << std::endl;
}