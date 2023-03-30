#include "iim42652.hpp"

#include <iostream>
#include <climits>

#include <unistd.h>

#include "imu.hpp"

#define POWER_ON_SLEEP      250

#define READ_MASK           0x80u

#define DEV_CFG_REG         0x11u
#define TEMP_DATA1_REG      0x1Du
#define ACCEL_DATA_X1_REG   0x1Fu
#define GYRO_DATA_X1_REG    0x25u
#define PWR_MGMT0_REG       0x4Eu
#define GYRO_CONFIG0_REG    0x4Fu
#define ACCEL_CONFIG0_REG   0x50u

#define GYRO_MODE_LN        0x0Cu
#define ACCEL_MODE_LN       0x03u

#define CONFIG_SCALE_MASK   0x07u
#define CONFIG_SCALE_SHIFT  5
#define CONFIG_RATE_MASK    0x0Fu

#define TEMP_DATA_SIZE      2u
#define ACCEL_DATA_SIZE     6u
#define GYRO_DATA_SIZE      6u

const float Iim42652::mGyroScales[] = {
    [DPS_2000   ]   = 2000.0 / SHRT_MAX,
    [DPS_1000   ]   = 1000.0 / SHRT_MAX,
    [DPS_500    ]   = 500.0 / SHRT_MAX,
    [DPS_250    ]   = 250.0 / SHRT_MAX,
    [DPS_125    ]   = 125.0 / SHRT_MAX,
    [DPS_62_5   ]   = 62.5 / SHRT_MAX,
    [DPS_31_25  ]   = 31.25 / SHRT_MAX,
    [DPS_15_62  ]   = 15.62 / SHRT_MAX,
};

const float Iim42652::mAccelScales[] = {
    [G_16       ]   = 16.0 / SHRT_MAX,
    [G_8        ]   = 8.0 / SHRT_MAX,
    [G_4        ]   = 4.0 / SHRT_MAX,
    [G_2        ]   = 2.0 / SHRT_MAX,
};

const Iim42652::AccelScale Iim42652::mDefaultAccelScale = G_16;
const Iim42652::GyroScale Iim42652::mDefaultGyroScale = DPS_2000;

Iim42652::Iim42652(Interface::IfacePtr &iface, bool verbose) :
    Imu(iface, verbose)
{
    Init();
}

Iim42652::~Iim42652() = default;

void Iim42652::Init()
{
    if (mVerbose == true)
    {
        std::cout << "Initializing IIM-42652... " << std::endl;
    } 

    // Enable IMU in low noise mode
    Interface::DataArray writeEnable{GYRO_MODE_LN | ACCEL_MODE_LN};
    mIface->Write(writeEnable, PWR_MGMT0_REG);

    // Per TDK IIM-42652 Design Specification (DS-000401-IIM-42652-v1.2) no
    // register writes should be issues for 200us when devices are transitioned
    // from off to on.  Added 50us for safe measure
    usleep(POWER_ON_SLEEP);

    Interface::DataArray enableStat{READ_MASK | PWR_MGMT0_REG};
    Interface::DataArray result(1,0);
    int count = mIface->Transfer(enableStat, result);

    if (mVerbose == true)
    {
        // If we got a result size of one and the value we read is the same
        // as the value we wrote, the device is set up.
        if ((count == 1) && (result[0] == writeEnable[0]))
        {
            std::cout << "IMU devices powered on!" << std::endl;
        }
        else
        {
            std::cout << "Failed to power on IMU devices..." << std::endl;
        }
    } 

    // Update ready status
    mReady = true;

    // Set default scale values
    mAccelScale = mDefaultAccelScale;
    mGyroScale = mDefaultGyroScale;

    if (mVerbose == true)
    {
        std::cout << "IIM-42652 initialization complete." << std::endl;
    } 
}

void Iim42652::Reset()
{
    // Write the reset flag
    Interface::DataArray resetData{0x01u};
    int count = mIface->Write(resetData, DEV_CFG_REG);

    // Wait 1000us / 1ms for device to reset
    usleep(1000);
}

bool Iim42652::GetAccelData(AxisData &result)
{
    bool success = false;

    // Prepare write and result variables
    Interface::DataArray writeData{READ_MASK | ACCEL_DATA_X1_REG};
    Interface::DataArray accelData(ACCEL_DATA_SIZE, 0);

    // Do the transfer and handle the results
    int count = mIface->Transfer(writeData, accelData);
    if (count == ACCEL_DATA_SIZE)
    {
        result[0] = (accelData[0] << 8) | accelData[1];
        result[1] = (accelData[2] << 8) | accelData[3];
        result[2] = (accelData[4] << 8) | accelData[5];
        success = true;
    }

    return success;
}

bool Iim42652::GetGyroData(AxisData &result)
{
    bool success = false;

    // Prepare write and result variables
    Interface::DataArray writeData{READ_MASK | GYRO_DATA_X1_REG};
    Interface::DataArray gyroData(GYRO_DATA_SIZE, 0);
    
    // Do the transfer and handle the results
    int count = mIface->Transfer(writeData, gyroData);
    if (count == GYRO_DATA_SIZE)
    {
        result[0] = (gyroData[0] << 8) | gyroData[1];
        result[1] = (gyroData[2] << 8) | gyroData[3];
        result[2] = (gyroData[4] << 8) | gyroData[5];
        success = true;
    }

    return success;
}

bool Iim42652::GetMagData(AxisData &result)
{
    // This device doesn't have a magnetometer, so there isn't anything to get
    return false;
}

bool Iim42652::GetTempData(short &result)
{
    bool success = false;

    // Prepare write and result variables
    Interface::DataArray writeData{READ_MASK | TEMP_DATA1_REG};
    Interface::DataArray tempData(TEMP_DATA_SIZE, 0);
    
    // Do the transfer and handle the results
    int count = mIface->Transfer(writeData, tempData);
    if (count == TEMP_DATA_SIZE)
    {
        result = (tempData[0] << 8) | tempData[1];

        success = true;
    }

    return success;
}


bool Iim42652::GetAccelValues(AxisValues &results)
{
    // Get the raw data
    AxisData data;
    bool success = GetAccelData(data);

    // If we got data, scale it based on stored values
    if (success == true)
    {
        results[0] = (float)data[0] * mAccelScales[mAccelScale];
        results[1] = (float)data[1] * mAccelScales[mAccelScale];
        results[2] = (float)data[2] * mAccelScales[mAccelScale];
    }

    return success;
}

bool Iim42652::GetGyroValues(AxisValues &results)
{
    // Get the raw data
    AxisData data;
    bool success = GetGyroData(data);
    
    // If we got data, scale it based on stored values
    if (success == true)
    {
        results[0] = (float)data[0] * mGyroScales[mGyroScale];
        results[1] = (float)data[1] * mGyroScales[mGyroScale];
        results[2] = (float)data[2] * mGyroScales[mGyroScale];
    }

    return success;
}

bool Iim42652::GetMagValues(AxisValues &results)
{
    // This device doesn't have a magnetometer so there's nothing to do
    return false;
}

bool Iim42652::GetTempValue(float &result)
{
    // Get the raw data
    short value = 0;
    bool success = GetTempData(value);
    
    // If we got data, scale it based on stored values
    if (success == true)
    {
        result = ((float)value / 132.48) + 25;
    }

    return success;
}

void Iim42652::UpdateAccelConfig(Rates rate, AccelScale scale)
{
    // Validate the configuration values are valid
    if ((((rate > RESERVED_0) && (rate < RESERVED_12)) || (rate == HZ_500)) &&
        ((scale >= G_16) && (scale <= G_2)))
    {
        // Prepare and write the data
        Interface::DataArray updateConfig{
            ACCEL_CONFIG0_REG,
            FormatConfig(rate, scale)
        };
        mIface->Write(updateConfig);
        mAccelScale = scale;
    }
}

void Iim42652::UpdateGyroConfig(Rates rate, GyroScale scale)
{
    // Validate the configuration values are valid
    if ((((rate > RESERVED_0) && (rate < RESERVED_12)) || (rate == HZ_500)) &&
        ((scale >= DPS_2000) && (scale <= DPS_15_62)))
    {
        // Prepare and write the data
        Interface::DataArray updateConfig{
            GYRO_CONFIG0_REG,
            FormatConfig(rate, scale)
        };
        mIface->Write(updateConfig);
        mGyroScale = scale;
    }
}

Interface::Value Iim42652::FormatConfig(Interface::Value rate, Interface::Value scale)
{
    Interface::Value newConfig = rate & CONFIG_RATE_MASK;
    newConfig |= (scale & CONFIG_SCALE_MASK) << CONFIG_SCALE_SHIFT;
    return newConfig;
}