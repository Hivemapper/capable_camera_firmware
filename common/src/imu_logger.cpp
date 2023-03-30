/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright 2022 Gunnar Ryder for Hellbender, Inc.
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 * 
 * imu_logger.hpp - Implementation for IMU logging class
 */

#include "imu_logger.hpp"

#include "nlohmann/json.hpp"
using json = nlohmann::json;

#include "imu_data.hpp"

ImuLogger::ImuLogger(
    std::string &path,
    std::string &tempPath,
    std::string &ext,
    int maxSize,
    int fileDuration,
    bool verbose,
    int debugLevel,
    bool live
    ) :
    Logger(path, tempPath, ext, maxSize, fileDuration, verbose, debugLevel, live) {};

ImuLogger::~ImuLogger() = default;

void ImuLogger::AddData(ImuData::Data &data)
{
    json organizedData = OrganizeData(data);
    QueueData(organizedData);
}

json ImuLogger::OrganizeData(ImuData::Data &data)
{
    json dataObject = json::object();
    std::string key;

    // Check whether accelerometer data is available.  If it is add it
    // to the json object
    if ((data.status & ImuData::DataStatus::AccelAvailable) > 0)
    {
        key = "accel";
        dataObject[key] = json::object();
        dataObject[key]["x"] = data.accel[0];
        dataObject[key]["y"] = data.accel[1];
        dataObject[key]["z"] = data.accel[2];
    }
    
    // Check whether gyroscope data is available.  If it is add it
    // to the json object
    if ((data.status & ImuData::DataStatus::GyroAvailable) > 0)
    {
        key = "gyro";
        dataObject[key] = json::object();
        dataObject[key]["x"] = data.gyro[0];
        dataObject[key]["y"] = data.gyro[1];
        dataObject[key]["z"] = data.gyro[2];
    }
    
    // Check whether magnetometer data is available.  If it is add it
    // to the json object
    if ((data.status & ImuData::DataStatus::MagAvailable) > 0)
    {
        key = "mag";
        dataObject[key] = json::object();
        dataObject[key]["x"] = data.mag[0];
        dataObject[key]["y"] = data.mag[1];
        dataObject[key]["z"] = data.mag[2];
    }
    
    // Check whether temperature data is available.  If it is add it
    // to the json object
    if ((data.status & ImuData::DataStatus::TempAvailable) > 0)
    {
        key = "temp";
        dataObject[key] = data.temp;
    }

    // Add the time to the json object
    dataObject["time"] = GetDateTimeString(data.time);

    return dataObject;
}
