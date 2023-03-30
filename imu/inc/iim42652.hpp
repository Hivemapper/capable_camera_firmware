#pragma once

#include "imu.hpp"
#include "interface.hpp"

class Iim42652 : public Imu
{
public:
    Iim42652(Interface::IfacePtr &iface, bool verbose = false);
    ~Iim42652();
    
    virtual void    Init() override;
    virtual void    Reset() override;

    virtual bool    GetAccelData(AxisData &result) override;
    virtual bool    GetGyroData(AxisData &result) override;
    virtual bool    GetMagData(AxisData &result) override;
    virtual bool    GetTempData(short &result) override;

    virtual bool    GetAccelValues(AxisValues &results) override;
    virtual bool    GetGyroValues(AxisValues &results) override;
    virtual bool    GetMagValues(AxisValues &results) override;
    virtual bool    GetTempValue(float &result) override;

    enum Rates
    {
        RESERVED_0  = 0,
        KHZ_32      = 1,
        KHZ_16      = 2,
        KHZ_8       = 3,
        KHZ_4       = 4,
        KHZ_2       = 5,
        KHZ_1       = 6,
        HZ_200      = 7,
        HZ_100      = 8,
        HZ_50       = 9,
        HZ_25       = 10,
        HZ_12_5     = 11,
        RESERVED_12 = 12,
        RESERVED_13 = 13,
        RESERVED_14 = 14,
        HZ_500      = 15,
        NUM_RATES
    };

    enum AccelScale
    {
        G_16        = 0,
        G_8         = 1,
        G_4         = 2,
        G_2         = 3
    };
    void UpdateAccelConfig(Rates rate, AccelScale scale);

    enum GyroScale
    {
        DPS_2000    = 0,
        DPS_1000    = 1,
        DPS_500     = 2,
        DPS_250     = 3,
        DPS_125     = 4,
        DPS_62_5    = 5,
        DPS_31_25   = 6,
        DPS_15_62   = 7
    };
    void UpdateGyroConfig(Rates rate, GyroScale scale);

protected:
    Interface::Value FormatConfig(Interface::Value rate, Interface::Value scale);

    static const float      mAccelScales[];
    static const float      mGyroScales[];

private:
    static const AccelScale mDefaultAccelScale;
    static const GyroScale  mDefaultGyroScale;
    AccelScale              mAccelScale;
    GyroScale               mGyroScale;
};
