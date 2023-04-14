/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * still_video.hpp - video capture program options
 */

#pragma once

#include <cstdio>

#include <string>

#include "options.hpp"

struct VideoOptions : public Options
{
	VideoOptions() : Options()
	{
		using namespace boost::program_options;
		// Generally we shall use zero or empty values to avoid over-writing the
		// codec's default behaviour.
		options_.add_options()
			("bitrate,b", value<uint32_t>(&bitrate)->default_value(0),
			 "Set the bitrate for encoding, in bits/second (h264 only)")
			("profile", value<std::string>(&profile),
			 "Set the encoding profile (h264 only)")
			("level", value<std::string>(&level),
			 "Set the encoding level (h264 only)")
			("intra,g", value<unsigned int>(&intra)->default_value(0),
			 "Set the intra frame period (h264 only)")
			("inline", value<bool>(&inline_headers)->default_value(false)->implicit_value(true),
			 "Force PPS/SPS header with every I frame (h264 only)")
			("codec", value<std::string>(&codec)->default_value("h264"),
			 "Set the codec to use, either h264, mjpeg or yuv420")
			("save-pts", value<std::string>(&save_pts),
			 "Save a timestamp file with this name")
			("quality,q", value<int>(&quality)->default_value(50),
			 "Set the MJPEG quality parameter (mjpeg only)")
			("quality,q", value<int>(&quality)->default_value(50),
			 "Set the MJPEG quality parameter (mjpeg only)")
            ("scale_quality,q", value<int>(&scale_quality)->default_value(50),
             "Set the MJPEG quality parameter (mjpeg only)")
            ("scale_width,d", value<int>(&scale_width)->default_value(50),
            "scale width")
            ("scale_height,d", value<int>(&scale_height)->default_value(50),
            "scale height")
            ("crop_width,d", value<int>(&crop_width)->default_value(50),
            "crop width")
            ("crop_height,d", value<int>(&crop_height)->default_value(50),
            "crop height")
            ("crop_offset_from_top,d", value<int>(&crop_offset_from_top)->default_value(50),
            "crop offset from top")
			("skip_4k", value<bool>(&skip_4k)->default_value(false)->implicit_value(true),
			 "skip 4k encoding")
                ("skip_2k", value<bool>(&skip_2k)->default_value(false)->implicit_value(true),
                 "skip 2k encoding")
			("listen,l", value<bool>(&listen)->default_value(false)->implicit_value(true),
			 "Listen for an incoming client network connection before sending data to the client")
			("keypress,k", value<bool>(&keypress)->default_value(false)->implicit_value(true),
			 "Pause or resume video recording when ENTER pressed")
			("signal,s", value<bool>(&signal)->default_value(false)->implicit_value(true),
			 "Pause or resume video recording when signal received")
			("initial,i", value<std::string>(&initial)->default_value("record"),
			 "Use 'pause' to pause the recording at startup, otherwise 'record' (the default)")
			("split", value<bool>(&split)->default_value(false)->implicit_value(true),
			 "Create a new output file every time recording is paused and then resumed")
			("segment", value<uint32_t>(&segment)->default_value(0),
			 "Break the recording into files of approximately this many milliseconds")
			("circular", value<bool>(&circular)->default_value(false)->implicit_value(true),
			 "Write output to a circular buffer which is saved on exit")
			;
	}

	uint32_t bitrate;
	std::string profile;
	std::string level;
	unsigned int intra;
	bool inline_headers;
	std::string codec;
	std::string save_pts;
	int quality;
	int scale_quality;
    int scale_width;
    int scale_height;
    int crop_width;
    int crop_height;
    int crop_offset_from_top;
	bool skip_4k;
	bool skip_2k;
	bool listen;
	bool keypress;
	bool signal;
	std::string initial;
	bool pause;
	bool split;
	uint32_t segment;
	bool circular;

        void json_manage_cam_cfg(nlohmann::json camera_cfg)
        {
            if(camera_cfg.contains("encoding"))
            {
                json_manage_enc_cfg(camera_cfg.at("encoding"));
            }
            
        }

        void json_manage_enc_cfg(nlohmann::json encoding_cfg)
        {
            if(encoding_cfg.contains("codec"))
            {   
                codec = encoding_cfg.at("codec");
            }
            if(encoding_cfg.contains("quality"))
            {   
                quality = encoding_cfg.at("quality");
            }
            if(encoding_cfg.contains("scale_quality"))
            {
                scale_quality = encoding_cfg.at("scale_quality");
            }
            if(encoding_cfg.contains("scale_width"))
            {
                scale_width = encoding_cfg.at("scale_width");
            }
            if(encoding_cfg.contains("scale_height"))
            {
                scale_height = encoding_cfg.at("scale_height");
            }
            if(encoding_cfg.contains("crop_width"))
            {
                crop_width = encoding_cfg.at("crop_width");
            }
            if(encoding_cfg.contains("crop_height"))
            {
                crop_height = encoding_cfg.at("crop_height");
            }
            if(encoding_cfg.contains("crop_offset_from_top"))
            {
                crop_offset_from_top = encoding_cfg.at("crop_offset_from_top");
            }
        }
        
        virtual bool JSON_Option_Parse(nlohmann::json new_cfg) override
        {
            if (Options::JSON_Option_Parse(new_cfg) == false)
            {
                return false;
            }
            if(new_cfg.contains("camera"))
            {
                json_manage_cam_cfg(new_cfg.at("camera"));    
            }
            return true;
        }
        
	virtual bool Parse(int argc, char *argv[]) override
	{
		if (Options::Parse(argc, argv) == false)
			return false;

               std::ifstream ifs(config_file.c_str());
               if (ifs)
               {
                   nlohmann::json new_cfg = nlohmann::json::parse(ifs);
                   JSON_Option_Parse(new_cfg);
               }

		if (width == 0)
			width = 640;
		if (height == 0)
			height = 480;
		if (strcasecmp(codec.c_str(), "h264") == 0)
			codec = "h264";
		else if (strcasecmp(codec.c_str(), "yuv420") == 0)
			codec = "yuv420";
		else if (strcasecmp(codec.c_str(), "mjpeg") == 0)
			codec = "mjpeg";
		else
			throw std::runtime_error("unrecognised codec " + codec);
		if (strcasecmp(initial.c_str(), "pause") == 0)
			pause = true;
		else if (strcasecmp(initial.c_str(), "record") == 0)
			pause = false;
		else
			throw std::runtime_error("incorrect initial value " + initial);
		if ((pause || split || segment || circular) && !inline_headers)
			std::cerr << "WARNING: consider inline headers with 'pause'/split/segment/circular" << std::endl;
		if ((split || segment) && output.find('%') == std::string::npos)
			std::cerr << "WARNING: expected % directive in output filename" << std::endl;

		return true;
	}
	virtual void Print() const override
	{
		Options::Print();
		std::cerr << "    bitrate: " << bitrate << std::endl;
		std::cerr << "    profile: " << profile << std::endl;
		std::cerr << "    level:  " << level << std::endl;
		std::cerr << "    intra: " << intra << std::endl;
		std::cerr << "    inline: " << inline_headers << std::endl;
		std::cerr << "    save-pts: " << save_pts << std::endl;
		std::cerr << "    codec: " << codec << std::endl;
		std::cerr << "    scale_quality (for MJPEG): " << scale_quality << std::endl;
		std::cerr << "    quality (for MJPEG): " << quality << std::endl;
        std::cerr << "    scale width: " << scale_width << std::endl;
        std::cerr << "    scale height: " << scale_height << std::endl;
        std::cerr << "    crop width: " << crop_width << std::endl;
        std::cerr << "    crop_height: " << crop_height << std::endl;
        std::cerr << "    crop_offset_from_top: " << crop_offset_from_top << std::endl;
		std::cerr << "    keypress: " << keypress << std::endl;
		std::cerr << "    signal: " << signal << std::endl;
		std::cerr << "    initial: " << initial << std::endl;
		std::cerr << "    split: " << split << std::endl;
		std::cerr << "    segment: " << segment << std::endl;
		std::cerr << "    circular: " << circular << std::endl;
	}
};
