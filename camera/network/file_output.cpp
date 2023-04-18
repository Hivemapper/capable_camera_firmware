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

#include "file_output.hpp"

FileOutput::FileOutput(VideoOptions const *options) : Output(options)
{

  std::vector<size_t> minFreeSizes = {0, 0, 0};
  std::vector<size_t> maxUsedSizes = {0, 0, 0};

  previewDir_   = options_->downsampleStreamDir;
  gpsReadyDir_  = options_->gpsLockCheckDir;

  directory_[0] = options_->output;
  directory_[1] = options_->output_2nd;
  directory_[2] = options_->downsampleStreamDir;

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
  fileNameGenerator << directory_[0];
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
  
  if(directory_[0] != "")
  {
    wrapAndWrite(mem, size, &tv, 0);
  }
  if(directory_[1] != "")
  {
    if(gpsReadyDir_ == "" || gpsLockAcq_)
    {
      wrapAndWrite(mem, size, &tv, 1);
    }
  }
  if(options_->downsampleStreamDir != "")
  {
      previewWrapAndWrite(prevMem, prevSize, &tv, frameNumTrun);
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

void FileOutput::wrapAndWrite(void *mem, size_t size, struct timeval *timestamp, int index)
{

  std::stringstream fileNameGenerator;
  fileNameGenerator << directory_[index];
  fileNameGenerator << prefix_;
  fileNameGenerator << std::setw(10) << std::setfill('0') << timestamp->tv_sec;
  fileNameGenerator << "_";
  fileNameGenerator << std::setw(6) << std::setfill('0') << timestamp->tv_usec;
  fileNameGenerator << postfix_;
  std::string fullFileName = fileNameGenerator.str();
  fileNameGenerator << ".tmp";
  std::string tempFileName = fileNameGenerator.str();

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
            writeFile(tempFileName, mem, size);
            boost::filesystem::rename(tempFileName, fullFileName);
        }
        else
        {
            writeFile(fullFileName, mem, size);
        }
      }
      catch (std::exception const &e)
      {
        std::cerr << "Failed to write file" << std::endl;
      }
      fileWritten = true;
    }
  }
  //After file is written, if we are the primary, set the latest marker
  if(index == 0)
  {
    int fd, ret;
    size_t latestSize = fullFileName.size();
    fd = open(latestFileName_.c_str(), O_CREAT|O_WRONLY|O_TRUNC, 0644);
    if ((ret = write(fd, fullFileName.c_str(), latestSize)) < 0) {
      throw std::runtime_error("failed to write data");
    }
    close(fd);
  }
}

void FileOutput::previewWrapAndWrite(void *mem, size_t size, struct timeval *timestamp, int64_t frameNum)
{
  std::stringstream fileNameGenerator;
  fileNameGenerator << options_->downsampleStreamDir;
  fileNameGenerator << prefix_;
  fileNameGenerator << std::setw(10) << std::setfill('0') << timestamp->tv_sec;
  fileNameGenerator << "_";
  fileNameGenerator << std::setw(6) << std::setfill('0') << timestamp->tv_usec;//picCounter;
  fileNameGenerator << postfix_;
  std::string fullFileName = fileNameGenerator.str();
  fileNameGenerator << ".tmp";
  std::string tempFileName = fileNameGenerator.str();

  bool fileWritten = false;
  while(!fileWritten)
  {
    if(fileManager_.canWrite(2))
    {
      fileManager_.addFile(2, size, fullFileName);
      try
      {
        if(writeTempFile_)
        {
          writeFile(tempFileName, mem, size);
            boost::filesystem::rename(tempFileName, fullFileName);
        }
        else
        {
            if (options_->verbose) {
                std::cout << "calling writeFile2: " << tempFileName <<" to: " << fullFileName << std::endl;
            }
          writeFile(fullFileName, mem, size);
        }
      }
      catch (std::exception const &e)
      {
        std::cerr << "Failed to write downsampled file" << std::endl;
      }
      fileWritten = true;
    }
  }
}

void FileOutput::writeFile(std::string fullFileName, void *mem, size_t size)
{
  //open file name and assign fd
  int fd, ret;
  fd = open(fullFileName.c_str(), O_CREAT|O_WRONLY|O_TRUNC, 0644);
  if ((ret = write(fd, mem, size)) < 0) {
    throw std::runtime_error("failed to write data");
  }
  close(fd);
  fsync(fd);
}
