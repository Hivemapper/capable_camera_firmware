/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright 2022 Gunnar Ryder for Hellbender, Inc.
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 * 
 * inotify.cpp - Implementation for inotify file watching class
 */

#include "inotify.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>

#include <errno.h>
#include <poll.h>
#include <sys/inotify.h>
#include <unistd.h>
using namespace std::chrono;

Inotify::Inotify(
        int flags,
        std::string &dir,
        FileList &files,
        milliseconds interval,
        bool verbose,
        int debugLevel
    ) :
    Thread(verbose, debugLevel),
    mFlags(flags),
    mPath(dir),
    mSearchFiles(std::move(files)),
    mFd(0),
    mWd(0)
{
    if (mVerbose == true)
    {
        std::cout << "Inotify constructor start..." << std::endl;
        std::cout << "Files in watch list:" << std::endl;
        for (std::string file : mSearchFiles)
        {
            std::cout << file << std::endl;
        }
    }

    // Init inotify to get file descriptor
    mFd = inotify_init();
    if (mFd != -1)
    {
        // Add directory to watch list
        mWd = inotify_add_watch(mFd, mPath.c_str(), mFlags);
        if (mWd != -1)
        {
            // Set thread interval
            SetInterval(interval);

            // Prepare poll structure
            mPollFds.fd = mFd;
            mPollFds.events = POLLIN;
        }
        else if (mVerbose == true)
        {
            std::cout << "Failed to open inotify watch descriptor" << std::endl;
        }
    }
    else if (mVerbose == true)
    {
        std::cout << "Failed to open inotify file descriptor" << std::endl;
    }

    if (mVerbose == true)
    {
        std::cout << "Inotify constructor Finished" << std::endl;
    }
}

Inotify::~Inotify()
{
    if (mVerbose == true)
    {
        std::cout << "Inotify destructor start..." << std::endl;
    }

    // If we've initialized an inotify element and started a watch
    if ((mFd > 0) && (mWd > 0))
    {
        // Remove watch and close the inotify file
        inotify_rm_watch(mFd, mWd);
        close(mFd);
        if (mVerbose == true)
        {
            std::cout << "Stopped watching directory and closed inotify." << std::endl;
        }
    }
    
    if (mVerbose == true)
    {
        std::cout << "Inotify destructor Finished" << std::endl;
    }
}

inline bool Inotify::IsWatching()
{
    return ((mFd > 0) && (mWd > 0));
}

void Inotify::SetChangeCallback(CallbackFunc &&func)
{
    mFunc = func;
}

void Inotify::ThreadFunc()
{
    // Check to see if data is ready.  Only wait 1ms for the data
    int ready = poll(&mPollFds, 1, 1);
    if (ready > 0)
    {
        // Read data from the inotify file
        int length = read(mFd, mBuffer, BUFFER_SIZE);
        inotify_event *curEvent = nullptr;
        int curPos = 0;
        if (length > 0)
        {
            // Loop through all inotify events
            do
            {
                curEvent = (inotify_event*)&mBuffer[0];
                std::string curFileName = curEvent->name;
                if (mVerbose == true)
                {
                    std::cout << "Inotify update on " << curFileName << std::endl;
                }

                // Make sure the event isn't for a directory
                if ((curEvent->mask & IN_ISDIR) == 0)
                {
                    // Check to see if the file is in our watch list
                    FileList::const_iterator findIt;
                    findIt = std::find(mSearchFiles.begin(), mSearchFiles.end(), curFileName);

                    // If the file is in our watch list and we have a callback function,
                    // pass the file name to the callback
                    if ((findIt != mSearchFiles.end()) && (mFunc))
                    {
                        if (mVerbose == true)
                        {
                            std::cout << "Found name in watch list. Calling callback..." << std::endl;
                        }

                        mFunc(curFileName);

                        if (mVerbose == true)
                        {
                            std::cout << "Callback complete!" << std::endl;
                        }
                    }
                    else if (mVerbose == true)
                    {
                        std::cout << "File not found in watch list. Skipping..." << std::endl;
                    }
                }
                else if (mVerbose == true)
                {
                    std::cout << "Event was for a directory. Skipping..." << std::endl;
                }

                // Advance the index to the next event
                curPos += (EVENT_SIZE + curEvent->len);
            } while (curPos < length);
        }
    }
    else if (ready != 0)
    {
        std::cout << "An error occurred while polling the file descriptor" 
            << std::endl;
    }
}