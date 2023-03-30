/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright 2022 Gunnar Ryder for Hellbender, Inc.
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 * 
 * gnss_logger.cpp - Data logger for gnss data
 */

#include "gnss_logger.hpp"

#include <string>
#include <cmath>

#include "nlohmann/json.hpp"
using json = nlohmann::json;

#define NUM_MODE_STRINGS    4
static const char *modeStrings[NUM_MODE_STRINGS] = {
    "n/a",
    "None",
    "2D",
    "3D"
};

GnssLogger::GnssLogger(
    std::string &path,
    std::string &tempPath,
    std::string &readyLoc,
    std::string &ext,
    int maxSize,
    int fileDuration,
    bool logSnr,
    bool verbose,
    int debugLevel
    ) :
    // don't output samples to stdout, do output the latest to <path>/latest
    Logger(path, tempPath, ext, maxSize, fileDuration, verbose, debugLevel, false, true),
    mLogSnr(logSnr),
    mReadyLoc(readyLoc){};

GnssLogger::~GnssLogger() = default;

void GnssLogger::AddData(gps_data_t &data)
{
    // Organize the data and queue it for output
    json organizedData = OrganizeData(data);
    QueueData(organizedData);
    ShareData(organizedData);
}

json GnssLogger::OrganizeData(gps_data_t &data)
{
    json dataObject = json::object();

    // add system time
    struct timespec currTime;
    timespec_get(&currTime, TIME_UTC);
    dataObject["systemtime"] = GetDateTimeString(currTime);

    // Add the fix string if the value is known
    if ((data.fix.mode >= 0) && (data.fix.mode < NUM_MODE_STRINGS))
    {
        dataObject["fix"] = modeStrings[data.fix.mode];
    }
    // If the value is not known, output the value directly
    else
    {
        dataObject["fix"] = data.fix.mode;
    }

    // Add the time stamp
    /*
    if (((data.set & TIME_SET) != 0) || 
        (data.fix.mode == mMinMode))
    */
    if ((data.set & TIME_SET) != 0)
    {
        dataObject["timestamp"] = GetDateTimeString(data.fix.time);
    }
    
    // Add lat and long data
    if (((data.set & LATLON_SET) != 0))
    {
        dataObject["latitude"] = data.fix.latitude;
        dataObject["longitude"] = data.fix.longitude;
    }

    // Add altitude
    if ((data.set & ALTITUDE_SET) != 0)
    {
        // Add altitude depending on whether we have a 2D or 3D fix
        double altitude = 
            (data.fix.mode == MODE_3D) ? 
                data.fix.altHAE : 
                data.fix.altMSL;
        dataObject["height"] = altitude;
    }

    // Add heading
    if ((data.set & TRACK_SET) != 0)
    {
        dataObject["heading"] = data.fix.track;
    }

    // Add Speed
    if ((data.set & SPEED_SET) != 0)
    {
        dataObject["speed"] = data.fix.speed;
    }

    // Add ECEF position data
    // Need to determine if data is finite, the library does not check it
    // If gpsd version 3.24 is used ECEF_SET can be used instead
    if (/*(data.set & ECEF_SET) != 0) && */
        (std::isfinite(data.fix.ecef.x) == true) &&
        (std::isfinite(data.fix.ecef.y) == true) &&
        (std::isfinite(data.fix.ecef.z) == true) &&
        (std::isfinite(data.fix.ecef.pAcc) == true))
    {
        dataObject["ecef"]["position"] = json::array({
            data.fix.ecef.x,
            data.fix.ecef.y,
            data.fix.ecef.z
        });
        dataObject["ecef"]["positionAccel"] = data.fix.ecef.pAcc;
    }
    
    // Add ECEF velocity data
    // Need to determine if data is finite, the library does not check it
    // If gpsd version 3.24 is used VECEF_SET can be used instead
    if (/*(data.set & VECEF_SET) != 0) && */
        (std::isfinite(data.fix.ecef.vx) == true) &&
        (std::isfinite(data.fix.ecef.vy) == true) &&
        (std::isfinite(data.fix.ecef.vz) == true) &&
        (std::isfinite(data.fix.ecef.vAcc) == true))
    {
        dataObject["ecef"]["velocity"] = json::array({
            data.fix.ecef.vx,
            data.fix.ecef.vy,
            data.fix.ecef.vz
        });
        dataObject["ecef"]["velocityAccel"] = data.fix.ecef.vAcc;
    }

    // Adding satellite data
    if ((data.set & SATELLITE_SET) != 0)
    {
        std::string parentKey = "satellites";
        int numVisible = data.satellites_visible;
        dataObject[parentKey]["seen"] = numVisible;
        dataObject[parentKey]["used"] = data.satellites_used;

        if (mLogSnr == true)
        {
            int sum = 0;
            int count = 0;
            dataObject[parentKey]["data"] = json::array();
            for (int index = 0; index < numVisible; index++)
            {
                json satelliteData = json::object();
                satellite_t &currSat = data.skyview[index];
                satelliteData["snr"] = currSat.ss;
                satelliteData["used"] = currSat.used;
                satelliteData["elevation"] = currSat.elevation;
                satelliteData["azimuth"] = currSat.azimuth;
                satelliteData["GNSSID"] = currSat.gnssid;
                satelliteData["SVID"] = currSat.svid;
                satelliteData["health"] = currSat.health;
                if (currSat.used == true)
                {
                    sum += currSat.ss;
                    count++;
                }
                dataObject[parentKey]["data"].push_back(satelliteData);
            }
            dataObject[parentKey]["snrAverage"] = sum / count;
        }
    }

	// This should check data.set & DOP_SET, but gpsd doesn't appear to set it
	std::string parentKey = "dop";
	dataObject[parentKey]["xdop"] = data.dop.xdop;
	dataObject[parentKey]["ydop"] = data.dop.ydop;
	dataObject[parentKey]["pdop"] = data.dop.pdop;
	dataObject[parentKey]["hdop"] = data.dop.hdop;
	dataObject[parentKey]["vdop"] = data.dop.vdop;
	dataObject[parentKey]["tdop"] = data.dop.tdop;
	dataObject[parentKey]["gdop"] = data.dop.gdop;

    return dataObject;
}

void GnssLogger::ShareData(json organizedData)
{
  static bool wroteLock = false;
  if(organizedData["fix"] == modeStrings[2] ||
     organizedData["fix"] == modeStrings[3])
  {
    if(!wroteLock)
    {
      wroteLock = true;
      std::ofstream output(mReadyLoc);
    }
  }
}
