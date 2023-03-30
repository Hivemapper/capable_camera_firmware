/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright 2022 Gunnar Ryder for Hellbender, Inc.
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 * 
 * inotify.hpp - Definition for inotify file watching class
 */

#pragma once

#include <chrono>
#include <functional>
#include <string>
#include <vector>

#include <poll.h>
#include <sys/inotify.h>

#include "thread.hpp"

using namespace std::chrono;

#define MAX_EVENTS      16
#define NAME_LENGTH     32
#define EVENT_SIZE      (sizeof(inotify_event))
#define BUFFER_SIZE     (MAX_EVENTS * (EVENT_SIZE + NAME_LENGTH))

class Inotify : public Thread
{
public:
    using FileList = std::vector<std::string>;
    using CallbackFunc = std::function<void(std::string&)>;
    Inotify(
        int flags,
        std::string &dir,
        FileList &files,
        milliseconds interval,
        bool verbose = false,
        int debugLevel = 0
    );
    ~Inotify();

    inline bool IsWatching();

    void SetChangeCallback(CallbackFunc &&func);

protected:
    virtual void ThreadFunc() override;

    const int           mFlags;
    const std::string   &mPath;
    const FileList      &mSearchFiles;

private:   
    int                 mFd;
    int                 mWd;
    char                mBuffer[BUFFER_SIZE];
    pollfd              mPollFds;
    CallbackFunc        mFunc;

};
