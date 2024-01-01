#pragma once
#include <cstdint>
namespace vermils
{
namespace wit
{
    namespace reg
    {
        /*

#define NORMAL          0x00
#define CALGYROACC      0x01
#define CALMAG          0x02
#define CALALTITUDE     0x03
#define CALANGLEZ       0x04
#define CALACCL         0x05
#define CALACCR         0x06
#define CALMAGMM        0x07
#define CALREFANGLE		0x08
#define CALMAG2STEP		0x09
//#define CALACCX       0x09
//#define ACC45PRX      0x0A
//#define ACC45NRX      0x0B
//#define CALACCY       0x0C
//#define ACC45PRY      0x0D
//#define ACC45NRY     	0x0E
//#define CALREFANGLER  0x0F
//#define CALACCINIT    0x10
//#define CALREFANGLEINIT 0x11
#define CALHEXAHEDRON		0x12

#define WIT_TIME        0x50
#define WIT_ACC         0x51
#define WIT_GYRO        0x52
#define WIT_ANGLE       0x53
#define WIT_MAGNETIC    0x54
#define WIT_DPORT       0x55
#define WIT_PRESS       0x56
#define WIT_GPS         0x57
#define WIT_VELOCITY    0x58
#define WIT_QUATER      0x59
#define WIT_GSA         0x5A
#define WIT_REGVALUE    0x5F

#define CAN_BAUD_1000000	 	0
#define CAN_BAUD_800000	  		1
#define CAN_BAUD_500000	  		2
#define CAN_BAUD_400000	  		3
#define CAN_BAUD_250000	  		4
#define CAN_BAUD_200000   		5
#define CAN_BAUD_125000	  		6
#define CAN_BAUD_100000  		7
#define CAN_BAUD_80000	  		8
#define CAN_BAUD_50000  		9
#define CAN_BAUD_40000	  		10
#define CAN_BAUD_20000  		11
#define CAN_BAUD_10000	  		12
#define CAN_BAUD_5000	  		13
#define CAN_BAUD_3000	  		14
*/
        #define WIT_REG_T inline constexpr const uint8_t
        WIT_REG_T SAVE = 0x00;  // Save/Reboot/Reset
        WIT_REG_T CALSW = 0x01; // Calibration mode, CALSW[3:0]
        WIT_REG_T RSW = 0x02;   // Data output options, [15:11], GSA, QUATERNION, VELOCITY, GPS, PRESSURE, PORT, MAG, ANGLE, GYRO, ACC, TIME
        WIT_REG_T RRATE = 0x03; // Data output rate, RRATE[3:0]
        WIT_REG_T BAUD = 0x04;  // Baud rate for serial port, BAUD[3:0]
        WIT_REG_T AXOFFSET = 0x05; // Accelerometer X-axis offset
        WIT_REG_T AYOFFSET = 0x06; // Accelerometer Y-axis offset
        WIT_REG_T AZOFFSET = 0x07; // Accelerometer Z-axis offset
        WIT_REG_T GXOFFSET = 0x08; // Gyroscope X-axis offset
        WIT_REG_T GYOFFSET = 0x09; // Gyroscope Y-axis offset
        WIT_REG_T GZOFFSET = 0x0a; // Gyroscope Z-axis offset
        WIT_REG_T HXOFFSET = 0x0b; // Magnetometer X-axis offset
        WIT_REG_T HYOFFSET = 0x0c; // Magnetometer Y-axis offset
        WIT_REG_T HZOFFSET = 0x0d; // Magnetometer Z-axis offset
        WIT_REG_T D0MODE = 0x0e;   // D0 mode, use D0 pin, D0MODE[3:0]
        WIT_REG_T D1MODE = 0x0f;   // D1 mode, use D1 pin, D1MODE[3:0]
        WIT_REG_T D2MODE = 0x10;   // D2 mode, use D2 pin, D2MODE[3:0]
        WIT_REG_T D3MODE = 0x11;   // D3 mode, use D3 pin, D3MODE[3:0]
        WIT_REG_T D0PWMH = 0x12;   // D0 PWM high byte, D0PWMH[7:0]?
        WIT_REG_T D1PWMH = 0x13;   // D1 PWM high byte, D1PWMH[7:0]?
        WIT_REG_T D2PWMH = 0x14;   // D2 PWM high byte, D2PWMH[7:0]?
        WIT_REG_T D3PWMH = 0x15;   // D3 PWM high byte, D3PWMH[7:0]?
        WIT_REG_T D0PWMT = 0x16;   // D0 PWM low byte, D0PWMT[7:0]?
        WIT_REG_T D1PWMT = 0x17;   // D1 PWM low byte, D1PWMT[7:0]?
        WIT_REG_T D2PWMT = 0x18;   // D2 PWM low byte, D2PWMT[7:0]?
        WIT_REG_T D3PWMT = 0x19;   // D3 PWM low byte, D3PWMT[7:0]?
        WIT_REG_T IICADDR = 0x1a;  // IIC address, IICADDR[6:0]
        WIT_REG_T LEDOFF = 0x1b;   // LED control, LEDOFF[0]
        WIT_REG_T MAGRANGX = 0x1c; // Magnetometer X-axis range
        WIT_REG_T MAGRANGY = 0x1d; // Magnetometer Y-axis range
        WIT_REG_T MAGRANGZ = 0x1e; // Magnetometer Z-axis range
        WIT_REG_T BANDWIDTH = 0x1f; // Bandwidth for .... idk, BANDWIDTH[3:0]
        WIT_REG_T GYRORANGE = 0x20; // Gyroscope range, GYRORANGE[3:0]
        WIT_REG_T ACCRANGE = 0x21;  // Accelerometer range, ACCRANGE[3:0]
        WIT_REG_T SLEEP = 0x22;     // Sleep mode, SLEEP[0]
        WIT_REG_T ORIENT = 0x23;    // Orientation of the device, ORIENT[0]
        WIT_REG_T AXIS6 = 0x24;     // Axis6 Algorithm, AXIS6[0]
        WIT_REG_T FILTK = 0x25;     // Filter K
        WIT_REG_T GPSBAUD = 0x26;   // Baud rate for GPS, GPSBAUD[3:0]
        WIT_REG_T READADDR = 0x27;  // Read address register, READADDR[7:0]
        WIT_REG_T BWSCALE = 0x28;   // Bandwidth scale, BWSCALE[3:0]?
        WIT_REG_T MOVETHR = 0x28;   // Move threshold, MOVETHR[7:0]?
        WIT_REG_T MOVESTA = 0x29;   // Move status, MOVESTA[0]?
        WIT_REG_T ACCFILT = 0x2A;   // Accelerometer filter
        WIT_REG_T GYROFILT = 0x2b;  // Gyroscope filter?
        WIT_REG_T MAGFILT = 0x2c;   // Magnetometer filter?
        WIT_REG_T POWONSEND = 0x2d; // Send date when power on, POWONSEND[0]
        WIT_REG_T VERSION = 0x2e;   // Version, RO
        WIT_REG_T CCBW = 0x2f;      // CCBW, idk wtf is this?
        WIT_REG_T YYMM = 0x30;      // Year and month, MM[15:8], YY[7:0]
        WIT_REG_T DDHH = 0x31;      // Day and hour, HH[15:8], DD[7:0]
        WIT_REG_T MMSS = 0x32;      // Minute and second, SS[15:8], MM[7:0]
        WIT_REG_T MS = 0x33;        // Millisecond
        WIT_REG_T AX = 0x34;        // Accelerometer X-axis, RO
        WIT_REG_T AY = 0x35;        // Accelerometer Y-axis, RO
        WIT_REG_T AZ = 0x36;        // Accelerometer Z-axis, RO
        WIT_REG_T GX = 0x37;        // Gyroscope X-axis, RO
        WIT_REG_T GY = 0x38;        // Gyroscope Y-axis, RO
        WIT_REG_T GZ = 0x39;        // Gyroscope Z-axis, RO
        WIT_REG_T HX = 0x3a;        // Magnetometer X-axis, RO
        WIT_REG_T HY = 0x3b;        // Magnetometer Y-axis, RO
        WIT_REG_T HZ = 0x3c;        // Magnetometer Z-axis, RO
        WIT_REG_T ROLL = 0x3d;      // Roll, RO
        WIT_REG_T PITCH = 0x3e;     // Pitch, RO
        WIT_REG_T YAW = 0x3f;       // Yaw, RO
        WIT_REG_T TEMP = 0x40;      // Temperature, RO
        WIT_REG_T D0STATUS = 0x41;  // D0 status, RO
        WIT_REG_T D1STATUS = 0x42;  // D1 status, RO
        WIT_REG_T D2STATUS = 0x43;  // D2 status, RO
        WIT_REG_T D3STATUS = 0x44;  // D3 status, RO
        WIT_REG_T PRESSUREL = 0x45; // Pressure low byte, RO
        WIT_REG_T PRESSUREH = 0x46; // Pressure high byte, RO
        WIT_REG_T HEIGHTL = 0x47;   // Altitude low byte, RO
        WIT_REG_T HEIGHTH = 0x48;   // Altitude high byte, RO
        WIT_REG_T LONL = 0x49;      // Longitude low byte, RO
        WIT_REG_T LONH = 0x4a;      // Longitude high byte, RO
        WIT_REG_T LATL = 0x4b;      // Latitude low byte, RO
        WIT_REG_T LATH = 0x4c;      // Latitude high byte, RO
        WIT_REG_T GPSHEIGHT = 0x4d; // GPS altitude, RO
        WIT_REG_T GPSYAW = 0x4e;    // GPS yaw, RO
        WIT_REG_T GPSVL = 0x4f;     // GPS velocity low byte, RO
        WIT_REG_T GPSVH = 0x50;     // GPS velocity high byte, RO
        WIT_REG_T Q0 = 0x51;        // Quaternion 0, RO
        WIT_REG_T Q1 = 0x52;        // Quaternion 1, RO
        WIT_REG_T Q2 = 0x53;        // Quaternion 2, RO
        WIT_REG_T Q3 = 0x54;        // Quaternion 3, RO
        WIT_REG_T SVNUM = 0x55;     // Number of satellites, RO
        WIT_REG_T PDOP = 0x56;      // Position dilution of precision, RO
        WIT_REG_T HDOP = 0x57;      // Horizontal dilution of precision, RO
        WIT_REG_T VDOP = 0x58;      // Vertical dilution of precision, RO
        WIT_REG_T DELAYT = 0x59;    // Delay time
        WIT_REG_T XMIN = 0x5a;      // X-axis alert minimum value threshold
        WIT_REG_T XMAX = 0x5b;      // X-axis alert maximum value threshold
        WIT_REG_T BATVAL = 0x5c;    // Battery voltage, RO
        WIT_REG_T ALARMPIN = 0x5d;  // Alarm pin, X-axis[15:8], Y-axis[7:0]
        WIT_REG_T YMIN = 0x5e;      // Y-axis alert minimum value threshold
        WIT_REG_T YMAX = 0x5f;      // Y-axis alert maximum value threshold
        WIT_REG_T GYROZSCALE = 0x60; // Gyroscope Z-axis scale
        WIT_REG_T GYROCALITHR = 0x61; // Gyroscope still threshold for calibration
        WIT_REG_T ALARMLEVEL = 0x62; // Alarm level, ALARMLEVEL[3:0]
        WIT_REG_T GYROCALTIME = 0x63; // Gyroscope auto calibration time
        WIT_REG_T REFROLL = 0x64;    // Reference roll?
        WIT_REG_T REFPITCH = 0x65;   // Reference pitch?
        WIT_REG_T REFYAW = 0x66;     // Reference yaw?
        WIT_REG_T GPSTYPE = 0x67;    // GPS type, GPSTYPE[3:0]?
        WIT_REG_T TRIGTIME = 0x68;   // Trigger time
        WIT_REG_T KEY = 0x69;        // Key register
        WIT_REG_T WERROR = 0x6a;     // Gyro deviation error, RO
        WIT_REG_T TIMEZONE = 0x6b;   // Time zone for GPS
        WIT_REG_T CALICNT = 0x6c;    // Calibration count?
        WIT_REG_T WZCNT = 0x6d;      // Gyroscope still count?
        WIT_REG_T WZTIME = 0x6e;     // Gyroscope still time
        WIT_REG_T WZSTATIC = 0x6f;   // Gyroscope speed integral threshold, wtf
        WIT_REG_T ACCSENSOR = 0x70;  // Accelerometer sensor type, ACCSENSOR[3:0]?
        WIT_REG_T GYROSENSOR = 0x71; // Gyroscope sensor type, GYROSENSOR[3:0]?
        WIT_REG_T MAGSENSOR = 0x72;  // Magnetometer sensor type, MAGSENSOR[3:0]?
        WIT_REG_T PRESSENSOR = 0x73; // Pressure sensor type, PRESSENSOR[3:0]?
        WIT_REG_T MODDELAY = 0x74;   // Mode delay, delay time for 485
        WIT_REG_T ANGLEAXIS = 0x75;  // Angle axis, ANGLEAXIS[3:0]?
        WIT_REG_T XRSCALE = 0x76;    // X-axis reference scale?
        WIT_REG_T YRSCALE = 0x77;    // Y-axis reference scale??
        WIT_REG_T ZRSCALE = 0x78;    // Z-axis reference scale????
        WIT_REG_T XREFROLL = 0x79;   // X-axis reference roll
        WIT_REG_T YREFPITCH = 0x7a;  // Y-axis reference pitch
        WIT_REG_T ZREFYAW = 0x7b;    // Z-axis reference yaw?
        WIT_REG_T ANGXOFFSET = 0x7c; // X-axis angle offset?
        WIT_REG_T ANGYOFFSET = 0x7d; // Y-axis angle offset?
        WIT_REG_T ANGZOFFSET = 0x7e; // Z-axis angle offset?
        WIT_REG_T NUMBERID1 = 0x7f;  // Device ID 1-2 bytes
        WIT_REG_T NUMBERID2 = 0x80;  // Device ID 3-4 bytes
        WIT_REG_T NUMBERID3 = 0x81;  // Device ID 5-6 bytes
        WIT_REG_T NUMBERID4 = 0x82;  // Device ID 7-8 bytes
        WIT_REG_T NUMBERID5 = 0x83;  // Device ID 9-10 bytes
        WIT_REG_T NUMBERID6 = 0x84;  // Device ID 11-12 bytes
        WIT_REG_T XA85PSCALE = 0x85; // X-axis 85 degree positive scale?
        WIT_REG_T XA85NSCALE = 0x86; // X-axis 85 degree negative scale?
        WIT_REG_T YA85PSCALE = 0x87; // Y-axis 85 degree positive scale?
        WIT_REG_T YA85NSCALE = 0x88; // Y-axis 85 degree negative scale?
        WIT_REG_T XA30PSCALE = 0x89; // X-axis 30 degree positive scale?
        WIT_REG_T XA30NSCALE = 0x8a; // X-axis 30 degree negative scale?
        WIT_REG_T YA30PSCALE = 0x8b; // Y-axis 30 degree positive scale?
        WIT_REG_T YA30NSCALE = 0x8c; // Y-axis 30 degree negative scale?
        WIT_REG_T CHIPIDL = 0x8D;    // Chip ID low byte, RO?
        WIT_REG_T CHIPIDH = 0x8E;    // Chip ID high byte, RO?
        WIT_REG_T REGINITFLAG = 0x8F; // Register init flag, RO?
        #undef WIT_REG_T
    }
    enum class ReportRate : uint8_t
    {
        None = 0x0d,
        Hz0_2 = 0x01,
        Hz0_5 = 0x02,
        Hz1 = 0x03,
        Hz2 = 0x04,
        Hz5 = 0x05,
        Hz10 = 0x06,
        Hz20 = 0x07,
        Hz50 = 0x08,
        Hz100 = 0x09,
        Hz125 = 0x0a, // only in WT931
        Hz200 = 0x0b,
        Once = 0x0c
    };

    enum class ReportBaud : uint8_t
    {
        Baud4800 = 0x01,
        Baud9600 = 0x02,
        Baud19200 = 0x03,
        Baud38400 = 0x04,
        Baud57600 = 0x05,
        Baud115200 = 0x06,
        Baud230400 = 0x07,
        Baud460800 = 0x08,  // only in WT931/JY931/HWT606/HWT906
        Baud921600 = 0x09  // only in WT931/JY931/HWT606/HWT906
    };

    enum class Orientation : bool
    {
        Horizontal = false,
        Vertical = true
    };

    enum OutputOpts : uint16_t
    {
        TimeMsk = 0x01,
        AccMsk = 0x02,
        GyroMsk = 0x04,
        AngleMsk = 0x08,
        MagMsk = 0x10,
        PortMsk = 0x20,
        PressMsk = 0x40,
        GpsMsk = 0x80,
        VelocityMsk = 0x100,
        QuatMsk = 0x200,
        GsaMsk = 0x400,
        AllMsk = 0xfff
    };

    enum class FilterAlgo : bool
    {
        Axis6 = true,
        Axis9 = false
    };

    enum class Bandwidth : uint8_t
    {
        Hz256 = 0x00,
        Hz184 = 0x01,
        Hz94 = 0x02,
        Hz44 = 0x03,
        Hz21 = 0x04,
        Hz10 = 0x05,
        Hz5 = 0x06
    };

    enum class DxMode
    {
        Analog = 0x00,
        DigitaiInput = 0x01,
        OutputHigh = 0x02,
        OutputLow = 0x03,
    };

    enum class CaliMode
    {
        Normal = 0x00,
        AutoAccCali = 0x01,
        ZeroAlti = 0x03,
        ZeroYaw = 0x04,
        MagCaliSphere = 0x07,
        AngleRef = 0x08,
        MagCaliDualPlane = 0x09,
    };
}
}
