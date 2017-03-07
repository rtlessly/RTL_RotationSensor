/*******************************************************************************
 RotationSensor.h
 Support for an interrupt-driven rotation sensor.
*******************************************************************************/

#ifndef _RotationSensor_h_
#define _RotationSensor_h_

#include <inttypes.h>


void RotationSensor_DebugDump();


//******************************************************************************
/// \class RotationSensor
/// \brief Support for an interrupt-driven rotation sensor.
//******************************************************************************
class RotationSensor
{
    public: static const int NO_READING = -1;

    public: typedef struct CountData_struct
    {
        uint32_t Count;
        uint32_t LastCountTime;
        uint32_t LastInterval;
        uint8_t  CountsPerRev;
        uint8_t  SensorID;

        CountData_struct() : SensorID(0), Count(0), CountsPerRev(0), LastCountTime(0), LastInterval(0) { };

        int RPM() { return (LastInterval == 0) ? 0.0 : (60000000.0 / (LastInterval * CountsPerRev)); };

        float Revs() { return ((float)Count) / CountsPerRev; };
    }
    CountData;


    //**************************************************************************
    /// Constructor
    //**************************************************************************
    public: RotationSensor(int pin, int pulsesPerRev=1);

    //**************************************************************************
    /// Reset the sensor counters to 0.
    //**************************************************************************
    public: void Reset();

    //**************************************************************************
    /// Enables/Disables the rotation sensor counter.
    ///
    /// If enabling, the sensor is reset and counting starts immediately once enabled.
    //**************************************************************************
    public: void Enable(bool enabled=true);

    public: void Disable() { Enable(false); };

    public: bool Enabled();

    //**************************************************************************
    /// Returns the resoultion of the sensor as pulses per revolution.
    //**************************************************************************
    public: int Resolution() { return _state.PulsesPerRev; };

    //**************************************************************************
    /// Returns the sensor ID (i.e.,the pin number the sensor is connected to).
    //**************************************************************************
    public: int ID() { return _state.Pin; };

    //**************************************************************************
    /// Returns the full sensor count data. 
    /// The return value is a CountData structure containing the accumulated pulse 
    /// count since the sensor was last reset, and the time interval (in microseconds) 
    /// between the last 2 sensor pulses. The struct also contains the ID (as the 
    /// pin number) and the pulses/revolution setting of the sensor.
    //**************************************************************************
    public: CountData Read();

    //**************************************************************************
    /// Returns just the current sensor count.
    //**************************************************************************
    public: uint32_t ReadCount();

    //**************************************************************************
    /// Returns the instantaneous sensor rotation rate as an RPM value. The sensor 
    /// should be enabled before this method is called, otherwise NO_READING is 
    /// returned. 
    ///
    /// A minimum of two pulses must occur before a reading can be made, since a
    /// reading is based on the time interval between pulses. So, if the sensor 
    /// is moving slowly (or not at all) then 0 will be returned until at least 
    /// 2 readings have been made.
    //**************************************************************************
    public: float ReadRPM();

    //**************************************************************************
    /// Returns the number of revolutions measured by the sensor since the last 
    /// reset. Since the return type is a float, fractional revolutions can be 
    /// measured up to the sensor resolution.
    //**************************************************************************
    public: float ReadRevs();

    /***************************************************************************
     Internal implementation
    ***************************************************************************/
    private: void Count_ISR();

    private: volatile uint32_t _count;
    private: volatile uint32_t _lastPulseTime;
    private: volatile uint32_t _prevPulseTime;
    
    private: struct
    {
        uint8_t PulsesPerRev : 8;
        uint8_t Pin          : 5;
        uint8_t IRQ          : 2;
        uint8_t Enabled      : 1;
    }
    _state;

    friend void RotationSensor_ISR_0();
    friend void RotationSensor_ISR_1();
};

#endif
