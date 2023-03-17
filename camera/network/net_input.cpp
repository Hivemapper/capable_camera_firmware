/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Raspberry Pi (Trading) Ltd.
 *
 * net_input.cpp - accept input over network
 */

#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <iostream>

#include <nlohmann/json.hpp>
#include "net_input.hpp"

#define DEFAULT_PATH "/tmp/config.sock\0"
#define DEFAULT_PATH_SIZE 17

using json = nlohmann::json;

NetInput::NetInput(VideoOptions *options)
{
    options_ = options;

	  sock_ = {};
   	sock_.sun_family = AF_UNIX;
   	strncpy(sock_.sun_path, DEFAULT_PATH, DEFAULT_PATH_SIZE);

   	fd_ = socket(AF_UNIX, SOCK_STREAM, 0);
	if (fd_ < 0) {
		throw std::runtime_error("unable to open unix socket");
	}

	if (connect(fd_, (struct sockaddr *) &sock_, sizeof(struct sockaddr_un)) == -1) {
   		throw std::runtime_error("unable to connect to unix socket");
   	}

    fd_to_check_[0].fd = fd_;
    fd_to_check_[0].events = POLLIN;
}

NetInput::~NetInput()
{
	close(fd_);
}

void NetInput::manage_cx_cfg(json connection_cfg)
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
  if(connection_cfg.contains("listen"))
  {
      options_->listen = connection_cfg.at("listen");
      setOutput = true;
  }
  
  if(setOutput)
  {
      options_->output = prefix_output + postfix_output;  
  }
}

void NetInput::manage_rec_cfg(json recording_cfg)
{
  if(recording_cfg.contains("connection"))
  {
    manage_cx_cfg(recording_cfg.at("connection"));
  }
}

void NetInput::manage_enc_cfg(json encoding_cfg)
{
  if(encoding_cfg.contains("fps"))
  {
    options_->framerate = encoding_cfg.at("fps");
  }
  if(encoding_cfg.contains("width"))
  {
    options_->width = encoding_cfg.at("width");
  }
  if(encoding_cfg.contains("height"))
  {  
    options_->height = encoding_cfg.at("height");
  }
  if(encoding_cfg.contains("codec"))
  {   
    options_->codec = encoding_cfg.at("codec");
  }
  if(encoding_cfg.contains("quality"))
  {   
    options_->framerate = encoding_cfg.at("quality");
  }    
}

void NetInput::manage_cb_cfg(json color_cfg)
{
  if(color_cfg.contains("awb"))
  {
    options_->awb = color_cfg.at("awb");
  }
  if(color_cfg.contains("awbGains"))
  {
    auto arrayFormat = color_cfg.at("awbGains");
    options_->awb_gain_r = arrayFormat[0];
    options_->awb_gain_b = arrayFormat[1];
  }
  if(color_cfg.contains("brightness"))
  {
    options_->brightness = color_cfg.at("brightness");
  }
  if(color_cfg.contains("contrast"))
  {
    options_->contrast = color_cfg.at("contrast");
  }
  if(color_cfg.contains("saturation"))
  {
    options_->saturation = color_cfg.at("saturation");
  } 
}

void NetInput::manage_exp_cfg(json exposure_cfg)
{
  if(exposure_cfg.contains("exposure"))
  {
    options_->exposure = exposure_cfg.at("exposure");
  }
  if(exposure_cfg.contains("ev"))
  {
    options_->ev = exposure_cfg.at("ev");
  }
  if(exposure_cfg.contains("fixedGain"))
  {
    options_->gain = exposure_cfg.at("fixedGain");
  }
  if(exposure_cfg.contains("metering"))
  {
    options_->metering = exposure_cfg.at("metering");
  }
  if(exposure_cfg.contains("sharpness"))
  {
    options_->sharpness = exposure_cfg.at("sharpness");
  } 
}

void NetInput::manage_cam_cfg(json camera_cfg)
{
  if(camera_cfg.contains("encoding"))
  {
    manage_enc_cfg(camera_cfg.at("encoding"));
  }
  if(camera_cfg.contains("colorBalance"))
  {
    manage_enc_cfg(camera_cfg.at("colorBalance"));
  }
  if(camera_cfg.contains("exposure"))
  {
    manage_enc_cfg(camera_cfg.at("exposure"));
  }
}



bool NetInput::update_options(uint8_t *buffer)
{
  bool force_restart = false;
  try
  {
    json new_cfg = json::parse(buffer);
    if(new_cfg.contains("recording"))
    {
      manage_rec_cfg(new_cfg.at("recording"));
      force_restart = true;
    }
    if(new_cfg.contains("camera"))
    {
      manage_cam_cfg(new_cfg.at("camera"));
      force_restart = true;      
    }
  }catch(json::parse_error& e)
  { 
     std::cout << e.what() << std::endl;
  }
  return force_restart;
}

size_t NetInput::poll_input()
{
  static uint8_t inbound_buf[2048];
  poll(fd_to_check_, 1, 0);
  if (fd_to_check_[0].revents & POLLIN)
  {
    size_t bytes_in = read(fd_to_check_[0].fd, inbound_buf, 2048);
    if(bytes_in > 0)
    {
      inbound_buf[bytes_in] = 0;
      std::cout << "Received config: " << bytes_in << " bytes" << std::endl;
      std::cout << inbound_buf << std::endl;
      if(update_options(inbound_buf))
      {
          return bytes_in;
      }
    }
  }
  return 0;
}

