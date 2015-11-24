/*
  PT100.cpp - Library for measuring temperature with a PT100.
  Created by João Lino, June 24, 2015.
  Released into the public domain.
*/
#include "Arduino.h"
#include "PT100.h"

PT100::PT100(char *name,
                    int OutputPin_SensorPower, 
                    int OutputPin_ThirdLinePower, 
                    int InputPin_TemperatureReading, 
                    int InputPin_ThirdLineReading, 
					int TimeBetweenReadings, 
					float ADCVmax, 
					float Vs,
					float R1,
					float R2,
                    float m,
                    float b) {
	
    _name                              =   name;
    _OutputPin_SensorPower             =   OutputPin_SensorPower;
    _OutputPin_ThirdLinePower          =   OutputPin_ThirdLinePower;
    _InputPin_TemperatureReading       =   InputPin_TemperatureReading;
    _InputPin_ThirdLineReading         =   InputPin_ThirdLineReading;
	_TimeBetweenReadings               =   TimeBetweenReadings;
	_ADCVmax                           =   ADCVmax;
	_Vs                                =   Vs;
	_R1                                =   R1;
	_R2 	                           =   R2;
    __m = m;
    __b = b;
	
	_temperatureAverage	               =   24.0;
	_measuredTemperature               =   24.0;
	_lastTemperatureRead               =   0;
	_VoutAnalogSample                  =   -1;
	_VoutPreviousAnalogSample          =   -1.0;
	_temperatureMeasurementsMarker     =   0;
    _rPT100MeasurementsMarker          =   0;
    _rLineMeasurementsMarker           =   0;
    _measuredTemperatureDeviation      =   0.0;
    _sampleDeviation                   =   0.0;

    analogReference(INTERNAL1V1);									// EXTERNAL && INTERNAL2V56 && INTERNAL1V1
    pinMode(_OutputPin_SensorPower, OUTPUT);            // setup temperature sensor input pin
    pinMode(_OutputPin_ThirdLinePower, OUTPUT);            // setup temperature sensor input pin
    digitalWrite(_OutputPin_SensorPower, LOW);      // initialize sensor on
    digitalWrite(_OutputPin_ThirdLinePower, LOW);      // initialize sensor on
}

void PT100::setPower(float ADCVmax, float Vs) {
    _ADCVmax                           =   ADCVmax;
    _Vs                                =   Vs;
}

void PT100::safeHardwarePowerOff() {
  digitalWrite(_OutputPin_SensorPower, LOW);        // Turn temperature sensor OFF for safety
  digitalWrite(_OutputPin_ThirdLinePower, LOW);        // Turn temperature sensor OFF for safety
}

void PT100::measure(boolean ln) {
    if(millis() - _lastTemperatureRead >= _TimeBetweenReadings) {                           //time to measure temperature
    
    /** measure Vout analog sample */
    _VoutPreviousAnalogSample   =   _VoutAnalogSample;                              // Store previous
    //digitalWrite(_OutputPin_SensorPower, HIGH);                                         // Turn on sensor
    //delay(10);
    _VoutAnalogSample               = analogRead(_InputPin_TemperatureReading);     // Get a reading
    //float test = analogRead(A7);
    //digitalWrite(_OutputPin_SensorPower, LOW);                                          // Turn off sensor
    
    _lastTemperatureRead            = millis();                                             // Mark time of temperature reading

    /** Calculate temperature value */
    float Vout = (_VoutAnalogSample + _VoutPreviousAnalogSample) / 2 * _ADCVmax / CONSTANT_ADC_STEP_COUNT;
    //float Rtest = 608 / ( _Vs / (test * _ADCVmax / CONSTANT_ADC_STEP_COUNT) - 1.0);
    float Rx = _R1 / ( _Vs / Vout - 1.0);// - Rtest;
    float measuredTemperatureNow = 1.08271 * pow(10.0, -13.0) * (3.12508 * pow(10.0, 16.0) - 5.65566 * pow(10.0, 6.0) * sqrt(3.51501 * pow(10.0, 19.0) - 4.61805 * pow(10.0, 16.0) * Rx));

    float temperatureDiff = measuredTemperatureNow - _temperatureAverage;                                                                           // Calculate deviation between temperature reading and average temperature
    _temperatureMeasurementsMarker++;                                                                                                                                                   // Position reading buffer marker at the last updated position
    if(_temperatureMeasurementsMarker >= TEMPERATURE_AVERAGE_VALUE_I) _temperatureMeasurementsMarker = 0;           // Check that it has not gone out of the buffer range

    if( _temperatureMeasurements[_temperatureMeasurementsMarker] == 0.0 ) {
        // First Run
        if ( measuredTemperatureNow > 12.0 ) {
            // First Run with good data
            _temperatureMeasurements[_temperatureMeasurementsMarker] = measuredTemperatureNow;
        }
        return;
    }
    /*
    if(temperatureDiff > 5 || temperatureDiff < 5) {
        _temperatureMeasurements[_temperatureMeasurementsMarker] = measuredTemperatureNow;   // Write the average temp + one third of the measured temperature deviation
    }
    else {
        _temperatureMeasurements[_temperatureMeasurementsMarker] = _temperatureAverage + (temperatureDiff / 3.0);   // Write the average temp + one third of the measured temperature deviation
    }
    */
    //_temperatureMeasurements[_temperatureMeasurementsMarker] = measuredTemperatureNow; 
    _temperatureMeasurements[_temperatureMeasurementsMarker] = _temperatureAverage + (temperatureDiff / 3.0);   // Write the average temp + one third of the measured temperature deviation

    _temperatureAverage = 0.0;                                                                                      // Zero out the average, it is time to calculate the new one
    for(int markerCounter = 0; markerCounter < TEMPERATURE_AVERAGE_VALUE_I; markerCounter++) {                  // Iterate over the temperature values in the buffer
      _temperatureAverage += _temperatureMeasurements[markerCounter];                                           // Sum all the temperature values
    }
    _temperatureAverage = _temperatureAverage / TEMPERATURE_AVERAGE_VALUE_F;                                    // Divide the sum by the number of values summed.
    _measuredTemperature = _temperatureAverage;                                                                 // Set the measured temperature to the calculated average value


    xFilterNoise(_temperatureMeasurementsMarker);
    /*
    Serial.print("PT100 : [");
    Serial.print(_name);
    Serial.print("]\tVoutSample: [");
    Serial.print(_VoutAnalogSample);


    //    Serial.print("]\tVout[");
    //    Serial.print(Vout,6);
    //    Serial.print("]\tRx[");
    //    Serial.print(Rx,6);

    
    Serial.print("]\tTNow[");
    Serial.print(measuredTemperatureNow,6);
    Serial.print("]\tTCalc[");
    Serial.print(_measuredTemperature,6);
    Serial.println("] ");
    */


    //Serial.print("PT100 : [");
    #ifdef DEBUG
    Serial.print(_name);
    Serial.print(",");
    Serial.print(_VoutAnalogSample);


    Serial.print(",");

    /*Serial.print(test);
    Serial.print(",");
    Serial.print(Rtest);
    Serial.print(",");*/
    /*Serial.print(Vout,6);
    Serial.print(",");
    Serial.print(Rx,6);

    
    Serial.print(",");
    Serial.print(measuredTemperatureNow,6);
    Serial.print(",");
    Serial.print(_measuredTemperature,6);
    Serial.print(",");*/
    if(ln) Serial.println("");
    #endif
  }
}

void PT100::measure1(boolean ln, boolean rline) {
  if(millis() - _lastTemperatureRead >= _TimeBetweenReadings) {                           //time to measure temperature
    
    /** measure Vout analog sample */
    digitalWrite(_OutputPin_SensorPower, HIGH);      // initialize sensor on
    digitalWrite(_OutputPin_ThirdLinePower, HIGH);      // initialize sensor on
    delay(10);
    _VoutAnalogSample = analogRead(_InputPin_TemperatureReading) + _sampleDeviation;     // Get a reading
    _VoutRAnalogSample = analogRead(_InputPin_ThirdLineReading) + _sampleDeviation;     // Get a reading
    digitalWrite(_OutputPin_SensorPower, LOW);      // initialize sensor on
    digitalWrite(_OutputPin_ThirdLinePower, LOW);      // initialize sensor on

    _lastTemperatureRead            = millis();                                             // Mark time of temperature reading

    _rPT100MeasurementsMarker++;                                                                                                                                                   // Position reading buffer marker at the last updated position
    if(_rPT100MeasurementsMarker >= TEMPERATURE_AVERAGE_VALUE_I) _rPT100MeasurementsMarker = 0;           // Check that it has not gone out of the buffer range
    _rPT100Measurements[_rPT100MeasurementsMarker] = _VoutAnalogSample;
    float Vout = GetMedian(_rPT100Measurements) * _ADCVmax / CONSTANT_ADC_STEP_COUNT;
    float Rx = _R1 / ( _Vs / Vout - 1.0);

    if(rline) {
        _rLineMeasurementsMarker++;                                                                                                                                                   // Position reading buffer marker at the last updated position
        if(_rLineMeasurementsMarker >= TEMPERATURE_AVERAGE_VALUE_I) _rLineMeasurementsMarker = 0;           // Check that it has not gone out of the buffer range
        _rLineMeasurements[_rLineMeasurementsMarker] = _VoutRAnalogSample;

        /** Calculate temperature value */
        float VoutR = GetMedian(_rLineMeasurements) * _ADCVmax / CONSTANT_ADC_STEP_COUNT;
        float Rline = _R2 / ( _Vs / VoutR - 1.0);
        _measuredTemperature = 1.08271 * pow(10.0, -13.0) * (3.12508 * pow(10.0, 16.0) - 5.65566 * pow(10.0, 6.0) * sqrt(3.51501 * pow(10.0, 19.0) - 4.61805 * pow(10.0, 16.0) * (Rx - Rline)));
    }
    else {
        /** Calculate temperature value */
        _measuredTemperature = 1.08271 * pow(10.0, -13.0) * (3.12508 * pow(10.0, 16.0) - 5.65566 * pow(10.0, 6.0) * sqrt(3.51501 * pow(10.0, 19.0) - 4.61805 * pow(10.0, 16.0) * Rx));
    }
    

    //xFilterNoise(_temperatureMeasurementsMarker);
/*
    Serial.print("PT100 : [");
    Serial.print(_name);
    Serial.print("]\tVoutSample: [");
    Serial.print(_VoutAnalogSample);


//    Serial.print("]\tVout[");
//    Serial.print(Vout,6);
//    Serial.print("]\tRx[");
//    Serial.print(Rx,6);

    
    Serial.print("]\tTNow[");
    Serial.print(measuredTemperatureNow,6);
    Serial.print("]\tTCalc[");
    Serial.print(_measuredTemperature,6);
    Serial.println("] ");
*/


    //Serial.print("PT100 : [");
    #ifdef DEBUG
    Serial.print(_name);
    Serial.print(",");
    Serial.print(_VoutAnalogSample);
    Serial.print(",");
    //Serial.print(_VoutRAnalogSample);
    //Serial.print(",");

    /*Serial.print(test);
    Serial.print(",");
    Serial.print(Rtest);
    Serial.print(",");*/
    /*Serial.print(Vout,6);
    Serial.print(",");
    Serial.print(Rx,6);

    
    Serial.print(",");
    Serial.print(measuredTemperatureNow,6);
    Serial.print(",");
    Serial.print(_measuredTemperature,6);
    Serial.print(",");*/
    if(ln) Serial.println("");
    #endif
  }
}

float PT100::GetMedian(int array[]){
    int sorted[TEMPERATURE_AVERAGE_VALUE_I];
    float value = 0.0;

    for(int x = 0; x < TEMPERATURE_AVERAGE_VALUE_I; x++) {
        sorted[x] = array[x];
    }

     //ARRANGE VALUES
    for(int x = 0; x < TEMPERATURE_AVERAGE_VALUE_I; x++){
        for(int y = 0; y < TEMPERATURE_AVERAGE_VALUE_I - 1; y++){
            if(sorted[y]>sorted[y+1]){
                int temp = sorted[y+1];
                sorted[y+1] = sorted[y];
                sorted[y] = temp;
            }
        }
    }

    //CALCULATE THE MEDIAN (middle number)
    if(TEMPERATURE_AVERAGE_VALUE_I % 2 != 0){// is the # of elements odd?
        int temp = ((TEMPERATURE_AVERAGE_VALUE_I+1)/2)-1;
        value = (float) sorted[temp];
    }
    else{// then it's even! :)
        value = ((float) ( sorted[(TEMPERATURE_AVERAGE_VALUE_I/2)-1] + sorted[TEMPERATURE_AVERAGE_VALUE_I/2] )) / 2.0;
    }

    return value;
}

float PT100::GetMode(float new_array[]) {
    int ipRepetition[TEMPERATURE_AVERAGE_VALUE_I];
    for (int i = 0; i < TEMPERATURE_AVERAGE_VALUE_I; i++) {
        ipRepetition[i] = 0;//initialize each element to 0
        int j = 0;//
        while ((j < i) && (new_array[i] != new_array[j])) {
            if (new_array[i] != new_array[j]) {
                j++;
            }
        }
        (ipRepetition[j])++;
    }
    int iMaxRepeat = 0;
    for (int i = 1; i < TEMPERATURE_AVERAGE_VALUE_I; i++) {
        if (ipRepetition[i] > ipRepetition[iMaxRepeat]) {
            iMaxRepeat = i;
        }
    }

    return new_array[iMaxRepeat];
}
/*
void PT100::mean(int new_array[], int num){
 //GET TOTAL & CALCULATE MEAN
    float total;
    for(int i=0;i<num; i++){
        total += new_array[i];
    }
    cout << "The mean is " << total/num << endl;
    mode(new_array, num);
}
void PT100::median(int new_array[], int num){
    //CALCULATE THE MEDIAN (middle number)
    if(num % 2 != 0){// is the # of elements odd?
        int temp = ((num+1)/2)-1;
        cout << "The median is " << new_array[temp] << endl;
    }
    else{// then it's even! :)
        cout << "The median is "<< new_array[(num/2)-1] << " and " << new_array[num/2] << endl;
    }
    mean(new_array, num);
}*/

void PT100::xFilterNoise( int position ) {
    for( int i = 0; i < 10 ; i++ ) {
        int first = (position - 2 - i < 0)?(position - 2 - i + TEMPERATURE_AVERAGE_VALUE_I):(position - 2 - i);
        int second = (position - 1 - i < 0)?(position - 1 - i + TEMPERATURE_AVERAGE_VALUE_I):(position - 1 - i);
        int third = (position - i < 0)?(position - i + TEMPERATURE_AVERAGE_VALUE_I):(position - i);

        if( 
            ( ( _temperatureMeasurements[second] > _temperatureMeasurements[first] ) && ( _temperatureMeasurements[second] > _temperatureMeasurements[third] ) ) 
            ||
            ( ( _temperatureMeasurements[second] < _temperatureMeasurements[first] ) && ( _temperatureMeasurements[second] < _temperatureMeasurements[third] ) ) 
            )
        {
            _temperatureMeasurements[second] = ( _temperatureMeasurements[first] + _temperatureMeasurements[third] ) / 2;
        }
    }

}

float PT100::getCurrentTemperature() {
    float y = __m * _measuredTemperature + __b;
    return _measuredTemperature + y + _measuredTemperatureDeviation; // - 4.41;
}

float PT100::getMeasuredTemperatureDeviation() {
    return _measuredTemperatureDeviation; // - 4.41;
}
float PT100::setMeasuredTemperatureDeviation( float measuredTemperatureDeviation) {
    if( _measuredTemperatureDeviation != measuredTemperatureDeviation ) {
        _measuredTemperatureDeviation = measuredTemperatureDeviation;

        for( int i = 0; i < TEMPERATURE_AVERAGE_VALUE_I; i++ ) {
            _temperatureMeasurements[i] = _temperatureMeasurements[i] + ( _measuredTemperatureDeviation * -1 );
        }
    }
    
    return _measuredTemperatureDeviation; // - 4.41;
}

float PT100::setSampleDeviation( float sampleDeviation) {
    _sampleDeviation = sampleDeviation;

    return _sampleDeviation;
}