/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * options.cpp - common program options helpers
 */

#include "core/options.hpp"

void Options::json_manage_cx_cfg(nlohmann::json connection_cfg)
{
  std::string prefix_output = "";
  std::string postfix_output = "";
  bool setOutput = false;
  
  if(connection_cfg.contains("socket"))
  {
      postfix_output = connection_cfg.at("socket");
      setOutput = true;
  }
  if(connection_cfg.contains("socketType"))
  {
      prefix_output = connection_cfg.at("socketType");
      setOutput = true;
  }
  
  if(setOutput)
  {
      output = prefix_output + postfix_output;  
  }
}

void Options::json_manage_fs_cfg(nlohmann::json fileinfo_cfg)
{ 
  if(fileinfo_cfg.contains("prefix"))
  {
      prefix = fileinfo_cfg.at("prefix");
  }
  if(fileinfo_cfg.contains("writeTmp"))
  {
      writeTmp = fileinfo_cfg.at("writeTmp");
  }
  if(fileinfo_cfg.contains("output"))
  {
      output = fileinfo_cfg.at("output");
  }
  if(fileinfo_cfg.contains("output2"))
  {
      output_2nd = fileinfo_cfg.at("output2");
  }
  if(fileinfo_cfg.contains("downsampleStreamDir"))
  {
    downsampleStreamDir = fileinfo_cfg.at("downsampleStreamDir");
  }
  if(fileinfo_cfg.contains("gpsLockCheckDir"))
  {
    gpsLockCheckDir = fileinfo_cfg.at("gpsLockCheckDir");
  }
  if(fileinfo_cfg.contains("latestChkFileDir"))
  {
    latestChkFileDir = fileinfo_cfg.at("latestChkFileDir");
  }
  if(fileinfo_cfg.contains("minfreespace"))
  {
      minfreespace = fileinfo_cfg.at("minfreespace");
  }
  if(fileinfo_cfg.contains("maxusedspace"))
  {
      maxusedspace = fileinfo_cfg.at("maxusedspace");
  }
  if(fileinfo_cfg.contains("minfreespace2"))
  {
      minfreespace_2nd = fileinfo_cfg.at("minfreespace2");
  }
  if(fileinfo_cfg.contains("maxusedspace2"))
  {
      maxusedspace_2nd = fileinfo_cfg.at("maxusedspace2");
  }
  
}

void Options::json_manage_rec_cfg(nlohmann::json recording_cfg)
{
  if(recording_cfg.contains("connection"))
  {
    json_manage_cx_cfg(recording_cfg.at("connection"));
  }
  if(recording_cfg.contains("directory"))
  {
    json_manage_fs_cfg(recording_cfg.at("directory"));
  }
}

void Options::json_manage_enc_cfg(nlohmann::json encoding_cfg)
{
  if(encoding_cfg.contains("fps"))
  {
    framerate = encoding_cfg.at("fps");
  }
  if(encoding_cfg.contains("width"))
  {
    width = encoding_cfg.at("width");
  }
  if(encoding_cfg.contains("height"))
  {  
    height = encoding_cfg.at("height");
  }
  if(encoding_cfg.contains("denoise"))
  {
    denoise = encoding_cfg.at("denoise");
  }
}

void Options::json_manage_adj_cfg(nlohmann::json adjustment_cfg)
{
  if(adjustment_cfg.contains("rotation"))
  {
    rotation_ = adjustment_cfg.at("rotation");
  }
  if(adjustment_cfg.contains("hflip"))
  {
    hflip_ = adjustment_cfg.at("hflip");
  } 
  if(adjustment_cfg.contains("vflip"))
  {
    vflip_ = adjustment_cfg.at("vflip");
  }
    if(adjustment_cfg.contains("roi"))
    {
        roi = adjustment_cfg.at("roi");
    }
}

void Options::json_manage_cb_cfg(nlohmann::json color_cfg)
{
  if(color_cfg.contains("awb"))
  {
    awb = color_cfg.at("awb");
  }
  if(color_cfg.contains("awbGains"))
  {
    auto arrayFormat = color_cfg.at("awbGains");
    awb_gain_r = arrayFormat[0];
    awb_gain_b = arrayFormat[1];
  }
  if(color_cfg.contains("brightness"))
  {
    brightness = color_cfg.at("brightness");
  }
  if(color_cfg.contains("contrast"))
  {
    contrast = color_cfg.at("contrast");
  }
  if(color_cfg.contains("saturation"))
  {
    saturation = color_cfg.at("saturation");
  } 
}

void Options::json_manage_exp_cfg(nlohmann::json exposure_cfg)
{
  if(exposure_cfg.contains("exposure"))
  {
    exposure = exposure_cfg.at("exposure");
  }
  if(exposure_cfg.contains("ev"))
  {
    ev = exposure_cfg.at("ev");
  }
  if(exposure_cfg.contains("fixedGain"))
  {
    gain = exposure_cfg.at("fixedGain");
  }
  if(exposure_cfg.contains("metering"))
  {
    metering = exposure_cfg.at("metering");
  }
  if(exposure_cfg.contains("sharpness"))
  {
    sharpness = exposure_cfg.at("sharpness");
  }
  if(exposure_cfg.contains("shutter"))
  {
    shutter = exposure_cfg.at("shutter");
  }
}

void Options::json_manage_cam_cfg(nlohmann::json camera_cfg)
{
  if(camera_cfg.contains("encoding"))
  {
    json_manage_enc_cfg(camera_cfg.at("encoding"));
  }
  if(camera_cfg.contains("colorBalance"))
  {
    json_manage_cb_cfg(camera_cfg.at("colorBalance"));
  }
  if(camera_cfg.contains("exposure"))
  {
    json_manage_exp_cfg(camera_cfg.at("exposure"));
  }
  if(camera_cfg.contains("adjustment"))
  {
    json_manage_adj_cfg(camera_cfg.at("adjustment"));
  }
}

bool Options::JSON_Option_Parse(nlohmann::json new_cfg)
{
    if(new_cfg.contains("camera"))
    {
      json_manage_cam_cfg(new_cfg.at("camera"));    
    }
    if(new_cfg.contains("recording"))
    {
      json_manage_rec_cfg(new_cfg.at("recording"));
    }
    return true;
}

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
        nlohmann::json new_cfg = nlohmann::json::parse(ifs);
        JSON_Option_Parse(new_cfg);
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

    if (sscanf(roi.c_str(), "%d,%d,%d,%d", &roi_x, &roi_y, &roi_width, &roi_height) != 4)
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
        { "long",  libcamera::controls::ExposureLong },
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
    std::cout << "    prefix: " << prefix << std::endl;
    std::cout << "    writeTmp: " << writeTmp << std::endl;
    std::cout << "    min free space: " << minfreespace << std::endl;
    std::cout << "    max used space: " << maxusedspace << std::endl; 
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
