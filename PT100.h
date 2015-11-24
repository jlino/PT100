/*
  PT100.h - Library for measuring temperature with a PT100.
  Created by João Lino, June 24, 2015.
  Released into the public domain.
*/
#ifndef PT100_h
#define PT100_h

#include "Arduino.h"

#define DEBUG

#define CONSTANT_ADC_STEP_COUNT 	1024.0

#define TEMPERATURE_AVERAGE_VALUE_I				50
#define TEMPERATURE_AVERAGE_VALUE_F 			50.0
//#define TEMPERATURE_AVERATE_INIT_VALUES		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
#define TEMPERATURE_AVERATE_INIT_VALUES		0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
#define TEMPERATURE_AVERATE_INIT_VALUES_I		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
#define TEMPERATURE_SETTING_MAX_VALUE 		120

class PT100
{
  public:
		// PT100(PT100_OUTPUT_PIN, PT100_INPUT_PIN, PT100_TIME_BETWEEN_READINGS, PT100_DEFAULT_ADC_VMAX, PT100_DEFAULT_VS, PT100_DEFAULT_R1_RESISTENCE, PT100_DEFAULT_LINE_RESISTENCE, PT100_DEFAULT_OPERATION_RESISTENCE);
    PT100(char *name,
    				int OutputPin_SensorPower, 
    				int OutputPin_ThirdLinePower,
					int InputPin_TemperatureReading, 
                    int InputPin_ThirdLineReading, 
					int TimeBetweenReadings = 100, 
					float ADCVmax = 1.081, 
					float Vs = 4.87,
					float R1 = 606.0,
					float R2 = 606.0,
					float m = 1.0,
					float b = 0.0);
    
		void setPower(float ADCVmax = 1.081, float Vs = 4.87);

		void measure(boolean ln);
		void measure1(boolean ln, boolean rline);
		void safeHardwarePowerOff();
		
		float getCurrentTemperature();

		float getMeasuredTemperatureDeviation();
		float setMeasuredTemperatureDeviation( float measuredTemperatureDeviation);

		float setSampleDeviation( float sampleDeviation);
  
	private:
		char		*_name;
		int 						_OutputPin_SensorPower; 
		int 						_OutputPin_ThirdLinePower;
		int 						_InputPin_TemperatureReading;
		int 						_InputPin_ThirdLineReading;
		int 						_TimeBetweenReadings;
		float 					_ADCVmax;
		float						_Vs;
		float 					_R1;
		float 					_R2;
		float __m;
		float __b;
		
		float           _temperatureAverage;
		float           _measuredTemperature;
		float			_measuredTemperatureDeviation;
		float			_sampleDeviation;
		unsigned long   _lastTemperatureRead;
		int           _VoutAnalogSample;
		int           _VoutRAnalogSample;
		float           _VoutPreviousAnalogSample;
		int             _temperatureMeasurementsMarker;
		int             _rPT100MeasurementsMarker;
		int             _rLineMeasurementsMarker;
		float           _temperatureMeasurements[TEMPERATURE_AVERAGE_VALUE_I] = {TEMPERATURE_AVERATE_INIT_VALUES};
		int           _rPT100Measurements[TEMPERATURE_AVERAGE_VALUE_I] = {TEMPERATURE_AVERATE_INIT_VALUES_I};
		int           _rLineMeasurements[TEMPERATURE_AVERAGE_VALUE_I] = {TEMPERATURE_AVERATE_INIT_VALUES_I};

		void xFilterNoise( int position );
		float GetMedian(int array[]);
		float GetMode(float array[]);
};

#endif