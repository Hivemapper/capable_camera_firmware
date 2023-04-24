/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Chris Niessl, Hellbender Inc.
 *
 * file_output.cpp - send directly to file.
 */
#include <iostream>
#include <iomanip>

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/un.h>
#include <unistd.h>
#include <time.h>
#include <boost/filesystem.hpp>
#include <fmt/core.h>

#include "file_output.hpp"

static const unsigned char exif_header[] = { 0xff, 0xd8, 0xff, 0xe1 };

FileOutput::FileOutput(VideoOptions const *options) : Output(options)
{

    std::vector<size_t> minFreeSizes = {0, 0, 0};
    std::vector<size_t> maxUsedSizes = {0, 0, 0};

    previewDir_   = options_->downsampleStreamDir;
    gpsReadyDir_  = options_->gpsLockCheckDir;

    directory_[0] = options_->output;
    directory_[1] = options_->output_2nd;
    directory_[2] = previewDir_;

    latestDir_    = options_->latestChkFileDir;
    minFreeSizes[0] = options_->minfreespace;
    minFreeSizes[1] = options_->minfreespace_2nd;
    minFreeSizes[2] = options_->minfreespace;

    std::cerr << "Initializing sizes.." << std::endl;

    maxUsedSizes[0] = options_->maxusedspace;
    maxUsedSizes[1] = options_->maxusedspace_2nd;
    maxUsedSizes[2] = options_->maxusedspace;

    verbose_ = options_->verbose;
    prefix_  = options_->prefix;
    writeTempFile_ = options_->writeTmp;

    //TODO - Assume jpeg format for now. Otherwise extract
    postfix_ = ".jpg";
    int numLocs = 3;
    gpsLockAcq_ = false;

    //Check if directories exist, and if not then ignore them
    if(!boost::filesystem::exists(directory_[0]))
    {
        directory_[0] = "";
    }
    if(!boost::filesystem::exists(directory_[1]))
    {
        directory_[1] = "";
    }
    if(!boost::filesystem::exists(previewDir_))
    {
        previewDir_ = "";
        directory_[2] = "";
    }

    //Use stringstream to create latest file for picture
    std::stringstream fileNameGenerator;
    fileNameGenerator << latestDir_;
    fileNameGenerator << "latest.txt";
    latestFileName_ = fileNameGenerator.str();

    std::cerr << "Initializing file handler..." << std::endl;
    fileManager_.initVars(verbose_,
                          prefix_,
                          minFreeSizes,
                          maxUsedSizes,
                          directory_,
                          numLocs);
}

FileOutput::~FileOutput()
{
}

void FileOutput::checkGPSLock()
{
    if(boost::filesystem::exists(gpsReadyDir_))
    {
        gpsLockAcq_ = true;
    }
}

void FileOutput::outputBuffer(void *mem,
                              size_t size,
                              void *prevMem,
                              size_t prevSize,
                              void *exifMem,
                              size_t exifSize,
                              int64_t timestamp_us,
                              uint32_t /*flags*/)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    static int32_t frameNumTrun = 0;

    try
    {
        tv = getAdjustedTime(timestamp_us);
    }
    catch (std::exception const &e)
    {
        std::cerr << "Time recording issues" << std::endl;
    }
    std::string primFileName = fmt::format("{}{}{:0>10d}_{:0>6d}{}", directory_[0],prefix_, tv.tv_sec,
                                           tv.tv_usec, postfix_);
    if(directory_[0] != "")
    {
        wrapAndWrite(mem, primFileName, size, exifMem, exifSize, 0);
    }
    if(directory_[1] != "")
    {
        if(gpsReadyDir_ == "" || gpsLockAcq_)
        {
            std::string secFileName = fmt::format("{}{}{:0>10d}_{:0>6d}{}", directory_[1],prefix_, tv.tv_sec,
                                                  tv.tv_usec, postfix_);
            wrapAndWrite(mem, secFileName, size, exifMem, exifSize, 1);
        }
    }
    if(previewDir_ != "")
    {
        std::string prevFileName = fmt::format("{}{}{:0>10d}_{:0>6d}{}", previewDir_,prefix_, tv.tv_sec,
                                               tv.tv_usec, postfix_);
        wrapAndWrite(prevMem, prevFileName, prevSize, exifMem, exifSize, 2);
    }

    //After files are written. Update the latest marker
    {
        int fd, ret;
        size_t latestSize = primFileName.size();
        fd = open(latestFileName_.c_str(), O_CREAT|O_WRONLY|O_TRUNC, 0644);
        if ((ret = write(fd, primFileName.c_str(), latestSize)) < 0) {
            throw std::runtime_error("failed to write latest data in file: " + latestFileName_);
        }
        close(fd);
    }

    frameNumTrun = (frameNumTrun + 1) % 1000;
    if((frameNumTrun % 100 == 0) && (gpsReadyDir_ != ""))
    {
        checkGPSLock();
    }
}

struct timeval FileOutput::getAdjustedTime(int64_t timestamp_us)
{
    static bool firstRun = false;
    struct timeval tv;
    time_t   fullSec  = timestamp_us / 1000000;
    long int microSec = timestamp_us % 1000000;

    if(!firstRun)
    {
        firstRun = true;
        gettimeofday(&baseTime_, NULL);
        if(baseTime_.tv_usec < microSec)
        {
            baseTime_.tv_usec = 1000000 + baseTime_.tv_usec - microSec;
            baseTime_.tv_sec  = baseTime_.tv_sec - fullSec - 1;
        } else
        {
            baseTime_.tv_usec = baseTime_.tv_usec - microSec;
            baseTime_.tv_sec  = baseTime_.tv_sec - fullSec;
        }
    }

    tv.tv_sec = baseTime_.tv_sec + fullSec;
    tv.tv_usec = baseTime_.tv_usec + microSec;
    if(tv.tv_usec > 1000000)
    {
        tv.tv_usec -= 1000000;
        tv.tv_sec  += 1;
    }
    return tv;
}

void FileOutput::wrapAndWrite(void *mem, std::string fullFileName, size_t size,
                              void *exifMem, size_t exifSize, int index)
{
    std::string tempFileName = fmt::format("{}.tmp", fullFileName);

    bool fileWritten = false;
    while(!fileWritten)
    {
        if(fileManager_.canWrite(index))
        {
            try
            {
                fileManager_.addFile(index, size, fullFileName);
                if(writeTempFile_)
                {
                    writeFile(tempFileName, mem, size, exifMem, exifSize);
                    boost::filesystem::rename(tempFileName, fullFileName);
                }
                else
                {
                    writeFile(fullFileName, mem, size, exifMem, exifSize);
                }
            }
            catch (std::exception const &e)
            {
                std::cerr << "Failed to write file" << std::endl;
            }
            fileWritten = true;
        }
    }
}

void FileOutput::writeFile(std::string fullFileName, void *mem, size_t size,
                           void *exifMem, size_t exifSize)
{

    //open file name and assign fd
    size_t totalWritten = 0;
    int nowWritten = 0;
    int fd = open(fullFileName.c_str(), O_CREAT|O_WRONLY|O_TRUNC|O_NONBLOCK, 0644);
    uint8_t *writerIndex = (uint8_t *)mem;
    uint8_t exifLenBuff[2];

    //if we have an exif header then shift the writerIndex by 20
    if(exifSize > 0)
    {
        writerIndex += 20;
        size -= 20;

        exifLenBuff[0] = (exifSize + 2) >> 8;
        exifLenBuff[1] = (exifSize + 2) & 0xff;
        while(totalWritten < 4)
        {
            totalWritten += write(fd, exif_header, 4);
        }
        totalWritten = 0;
        while(totalWritten < 2)
        {
            totalWritten += write(fd, exifLenBuff, 2);
        }
        totalWritten = 0;
        while(totalWritten < exifSize)
        {
            totalWritten += write(fd, exifMem, exifSize);
        }
    }

    totalWritten = 0;
    while(totalWritten < size)
    {
        nowWritten = write(fd, writerIndex, size - totalWritten);
        if(nowWritten < 0){
            throw std::runtime_error("failed to write data2");
        }else if (nowWritten == 0){
            std::cerr << "no data written..." << std::endl;
        }
        writerIndex += nowWritten;
        totalWritten += nowWritten;
    }
    close(fd);

}
