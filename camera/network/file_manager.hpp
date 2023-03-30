/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Chris Niessl, Hellbender Inc.
 *
 * file_output.hpp - send directly to file
 */

#pragma once

#include <netinet/in.h>
#include <sys/un.h>
#include <sys/time.h>

#include <condition_variable>
#include <mutex>
#include <thread>
#include <queue>

typedef std::pair<size_t, std::string> fileInfo; 
typedef std::pair<std::time_t, fileInfo> filePoint;

class FileManager
{
public:

    FileManager();
    FileManager(bool verbose, 
                std::string prefix,
                std::vector<size_t> minFreeSizeThresh,
                std::vector<size_t> maxUsedSizeThresh,
                std::string* directory,
                int recordLocs);
    
    ~FileManager();

    bool canWrite(int index);
    void addFile(int index, size_t size, std::string fullFileName);

    void initVars(bool verbose, 
                  std::string prefix,
                  std::vector<size_t> minFreeSizeThresh,
                  std::vector<size_t> maxUsedSizeThresh,
                  std::string* directory,
                  int recordLocs);

protected:

    void accountForExistingFiles(int index);
    void deleteThread();
    bool checkFreeSpace(int index);
    void deleteOldestFile(int index);

private:

    static const int NUM_MAX_DESTS = 3;

    bool verbose_;
    std::string prefix_;
    std::string postfix_;
    
    int recordLocs_;
    bool doCheck_[NUM_MAX_DESTS];
    bool canWrite_[NUM_MAX_DESTS];
    std::string directory_[NUM_MAX_DESTS];
    std::queue<std::string> filenameQueue_[NUM_MAX_DESTS];
    std::queue<size_t>      filesizeQueue_[NUM_MAX_DESTS];

    std::priority_queue<filePoint, std::vector<filePoint>, std::greater<filePoint>> oldFileQueue_[NUM_MAX_DESTS];
    
    size_t minFreeSizeThresh_[NUM_MAX_DESTS];
    size_t maxUsedSizeThresh_[NUM_MAX_DESTS];
    size_t currentUsedSize_[NUM_MAX_DESTS];
    
    std::mutex metric_mutex_;
    std::condition_variable free_cond_var_;
    std::thread delete_thread_;
};
