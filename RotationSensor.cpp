/*******************************************************************************
 RotationSensor.cpp
Support for an interrupt-driven rotation sensor.
*******************************************************************************/

#define DEBUG 0

#include <Arduino.h>
#include <RTL_Stdlib.h>
#include <RTL_Debug.h>
#include "RotationSensor.h"


static DebugHelper Debug("RotationSensor");


/******************************************************************************
 Interrupt variables

 These variables contain the state information that is maintained by the sensor
 interrupt service routines (ISR). Since they are referenced by an ISR they MUST
 be static and volatile.
******************************************************************************/

#if DEBUG
static const int debugStackSize = 32;
static volatile int debugIdx = 0;
static volatile uint32_t debugTime[debugStackSize];
static volatile uint32_t debugCount[debugStackSize];
#endif


/******************************************************************************
 Forward declarations for each ISR
******************************************************************************/
static volatile RotationSensor* pSensors[2];
static void RotationSensor_ISR_0();
static void RotationSensor_ISR_1();


/*******************************************************************************
 Constructor
*******************************************************************************/
RotationSensor::RotationSensor(int pin, int pulsesPerRev)
{
    _state.Pin = pin;
    _state.PulsesPerRev = max(1, pulsesPerRev);
    _state.IRQ = digitalPinToInterrupt(_state.Pin);
    _state.Enabled = false;

    if (_state.IRQ != NOT_AN_INTERRUPT)
    {
        pinMode(pin, INPUT);
    }
}


/*******************************************************************************
 Resets the rotation sensor counter values to 0.
*******************************************************************************/
void RotationSensor::Reset()
{
    // Disable interrupts while resetting sensor values
    cli();
    _count = 0;
    _lastPulseTime  = 0;
    _prevPulseTime  = 0;
    sei();

//#if DEBUG
//    cli();
//    debugIdx = 0;
//    for (int i=0; i < debugStackSize; i++)
//    {
//        debugTime[i] = 0;
//        debugCount[i] = 0;
//    }
//    sei();
//#endif
}


/*******************************************************************************
 Enables/Disables the rotation sensor counter.

 If enabling, the counter is reset and counting starts immediately once enabled.
*******************************************************************************/
void RotationSensor::Enable(bool enabled)
{
    if (_state.IRQ == NOT_AN_INTERRUPT)
    {
        Debug.Log("%s - Invalid interrupt pin %i", __func__, _state.Pin);
        return;
    }

    int irq = _state.IRQ;

    if (enabled && !_state.Enabled)
    {
        Reset();
        pSensors[irq] = this;

        if (irq == 0) attachInterrupt(irq, RotationSensor_ISR_0, RISING);
        else
        if (irq == 1) attachInterrupt(irq, RotationSensor_ISR_1, RISING);
    }
    else if (!enabled)
    {
        detachInterrupt(irq);
        pSensors[irq] = NULL;
    }

    _state.Enabled = enabled;
}


/*******************************************************************************
 Returns the enabled state of the rotation sensor counter.
*******************************************************************************/
inline bool RotationSensor::Enabled()
{
    return (_state.IRQ != NOT_AN_INTERRUPT) && _state.Enabled;
}


/*******************************************************************************
 Returns the sensor count data. The return value is a CountData structure
 containing the accumulated pulse count and time (in microseconds) since the
 sensor was last reset, as well the ID of the sensor (as the pin number) and
 the pulses/revolution setting for the sensor.
*******************************************************************************/
RotationSensor::CountData RotationSensor::Read()
{
    CountData data;

    data.SensorID = _state.Pin;
    data.CountsPerRev = _state.PulsesPerRev;

    if (Enabled())
    {
        // Disable interrupts while reading sensor values to make sure we don't get
        // inconsistent values (e.g., if a sensor interrupt occurs between the following
        // lines then the values will be mis-matched).
        cli();
        uint32_t count     = _count;
        uint32_t endTime   = _lastPulseTime;
        uint32_t prevTime  = _prevPulseTime;
        sei();

        data.Count          = count;
        data.LastCountTime  = endTime;
        data.LastInterval   = (prevTime == 0) ? 0 : endTime - prevTime;

        Debug.Log("%s[%i]: count=%l, LastCountTime=%l, LastInterval=%l, resolution=%i", __func__, data.SensorID, count, endTime, data.LastInterval, data.CountsPerRev);
    }

    return data;
}


/*******************************************************************************
 Returns the current sensor count.
*******************************************************************************/
uint32_t RotationSensor::ReadCount()
{
    uint32_t count = 0;
    
    if (Enabled())
    {
        // Disable interrupts while reading count.
        cli();
        count = _count;
        sei();
    }

    return count;
}


/*******************************************************************************
 Returns the sensor rotation rate as an RPM value. The sensor should be enabled
 before this method is called.

 Returns the RPM value, or NO_READING if the sensor is not enabled.

 A minimum of two pulses must occur before a reading can be made, since a
 reading is based on the time interval between pulses. So, if the sensor is moving
 slowly (or not at all) then 0 will be returned until 2 readings have been made.
*******************************************************************************/
float RotationSensor::ReadRPM()
{
    if (!Enabled()) return (float)NO_READING;

    CountData data = Read();

    return data.RPM();
}


/*******************************************************************************
 Returns the number of revolotions measured by the sensor since the last reset.
 Since the return type is a float, fractional revolutions can be measured up to
 the sensor resolution.
*******************************************************************************/
float RotationSensor::ReadRevs()
{
    if (!Enabled()) return (float)NO_READING;

    CountData data = Read();

    return data.Revs();
}



void RotationSensor::Count_ISR()
{
    uint32_t now = micros();

//    if ((now - _lastPulseTime) < 200) return;  // 200 microsecond debounce

    _prevPulseTime = _lastPulseTime;
    _lastPulseTime = now;
    _count++;

//#if DEBUG
//    debugTime[debugIdx] = lastPulseTime[0];
//    debugCount[debugIdx] = pulseCount[0];
//    debugIdx = (debugIdx + 1) & (debugStackSize-1);
//#endif
}


/*******************************************************************************
 Interrupt Service Routines (ISR)

 Currently, two sensors are supported on IRQ 0 (pin 2) and IRQ 1 (pin 3). Since
 there is no inherent way to distinguish which sensor is triggering the interrupt,
 each sensor must have its own interrupt. So there are 2 ISR functions with
 essentially identical code, differing only by the index used to access the
 respective variables. 

 Hopefully, the compiler will be able to optimize away the array indexing since
 the indexes are all constants, so they should just resolve at compile time to
 absolute, constant memory addresses.
*******************************************************************************/
static void RotationSensor_ISR_0()
{
    RotationSensor* pSensor = pSensors[0];

    if (pSensor != NULL) pSensor->Count_ISR();
}


static void RotationSensor_ISR_1()
{
    RotationSensor* pSensor = pSensors[1];

    if (pSensor != NULL) pSensor->Count_ISR();
}


void RotationSensor_DebugDump()
{
#if DEBUG
    uint32_t times[debugStackSize];
    uint32_t counts[debugStackSize];

    cli();
    for (int i=0; i < debugStackSize; i++)
    {
        times[i]   = debugTime[i];
        counts[i]  = debugCount[i];
    }
    sei();

    Debug.PrintLine("Index, Time,Count");

    for (int i=0; i < debugStackSize; i++)
    {
        Debug.PrintLine("%i,%l,%l", i, times[i], counts[i]);
    }
#endif
}





