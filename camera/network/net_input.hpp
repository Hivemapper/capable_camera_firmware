/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Raspberry Pi (Trading) Ltd.
 *
 * net_input.hpp - receive input over network.
 */

#pragma once

#include <netinet/in.h>
#include <poll.h>
#include <sys/un.h>

#include <nlohmann/json.hpp>

#include "output.hpp"

#define NUM_CAM_OPTS 

using json = nlohmann::json;

class NetInput
{
public:
    
    NetInput(VideoOptions *options);
    ~NetInput();

    size_t poll_input();
    bool   update_options(uint8_t *buffer);

private:
    
    int fd_;
    sockaddr_un sock_;
    struct pollfd fd_to_check_[1];
    
    VideoOptions *options_;

    void manage_cx_cfg(json connection_cfg);

    void manage_rec_cfg(json recording_cfg);
    
    void manage_enc_cfg(json encoding_cfg);
    void manage_cb_cfg(json color_cfg);
    void manage_exp_cfg(json exposure_cfg);
    
    void manage_cam_cfg(json camera_cfg);
    
};
