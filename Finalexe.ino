#include <Arduino.h>
#include <EEPROM.h>
#define EEPROM_ADDR 0
float yourVariable = 0;
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#ifndef ZMPT101B_h
#define ZMPT101B_h

#include <Arduino.h>

#define DEFAULT_FREQUENCY 50.0f
#define DEFAULT_SENSITIVITY 500.0f

#if defined(AVR)
	#define ADC_SCALE 1023.0f
	#define VREF 5.0f
#elif defined(ESP8266)
	#define ADC_SCALE 1023.0
	#define VREF 3.3
#elif defined(ESP32)
	#define ADC_SCALE 4095.0
	#define VREF 3.3
#endif

class ZMPT101B
{
public:
	ZMPT101B (uint8_t pin, uint16_t frequency = DEFAULT_FREQUENCY);
	void     setSensitivity(float value);
	float 	 getRmsVoltage(uint8_t loopCount = 1);

private:
	uint8_t  pin;
	uint32_t period;
	float 	 sensitivity = DEFAULT_SENSITIVITY;
	int 	 getZeroPoint();
};

#endif

/// @brief ZMPT101B constructor
/// @param pin analog pin that ZMPT101B connected to.
/// @param frequency AC system frequency
ZMPT101B::ZMPT101B(uint8_t pin, uint16_t frequency)
{
	this->pin = pin;
	period = 1000000 / frequency;
	pinMode(pin, INPUT);
}

/// @brief Set sensitivity
/// @param value Sensitivity value
void ZMPT101B::setSensitivity(float value)
{
	sensitivity = value;
}

/// @brief Calculate zero point
/// @return zero / center value
int ZMPT101B::getZeroPoint()
{
	uint32_t Vsum = 0;
	uint32_t measurements_count = 0;
	uint32_t t_start = micros();

	while (micros() - t_start < period)
	{
		Vsum += analogRead(pin);
		measurements_count++;
	}

	return Vsum / measurements_count;
}

/// @brief Calculate root mean square (RMS) of AC valtage
/// @param loopCount Loop count to calculate
/// @return root mean square (RMS) of AC valtage
float ZMPT101B::getRmsVoltage(uint8_t loopCount)
{
	double readingVoltage = 0.0f;

	for (uint8_t i = 0; i < loopCount; i++)
	{
		int zeroPoint = this->getZeroPoint();

		int32_t Vnow = 0;
		uint32_t Vsum = 0;
		uint32_t measurements_count = 0;
		uint32_t t_start = micros();

		while (micros() - t_start < period)
		{
			Vnow = analogRead(pin) - zeroPoint;
			Vsum += (Vnow * Vnow);
			measurements_count++;
		}

		readingVoltage += sqrt(Vsum / measurements_count) / ADC_SCALE * VREF * sensitivity;
	}

	return readingVoltage / loopCount;
}

#ifndef EmonLib_h
#define EmonLib_h

#if defined(ARDUINO) && ARDUINO >= 100

#include "Arduino.h"

#else

#include "WProgram.h"

#endif


#ifndef READVCC_CALIBRATION_CONST
#define READVCC_CALIBRATION_CONST 1126400L
#endif


#if defined(__arm__)
#define ADC_BITS    12
#else
#define ADC_BITS    10
#endif

#define ADC_COUNTS  (1<<ADC_BITS)


class EnergyMonitor
{
  public:

    void voltage(unsigned int _inPinV, double _VCAL, double _PHASECAL);
    void current(unsigned int _inPinI, double _ICAL);

    void voltageTX(double _VCAL, double _PHASECAL);
    void currentTX(unsigned int _channel, double _ICAL);

    void calcVI(unsigned int crossings, unsigned int timeout);
    double calcIrms(unsigned int NUMBER_OF_SAMPLES);
    void serialprint();

    long readVcc();
  
    double realPower,
      apparentPower,
      powerFactor,
      Vrms,
      Irms;

  private:

  
    unsigned int inPinV;
    unsigned int inPinI;
  
    double VCAL;
    double ICAL;
    double PHASECAL;



  
    int sampleV;                     
    int sampleI;

    double lastFilteredV,filteredV;          
    double filteredI;
    double offsetV;                        
    double offsetI;                        

    double phaseShiftedV;                             

    double sqV,sumV,sqI,sumI,instP,sumP;             

    int startV;                                      

    boolean lastVCross, checkVCross;                  


};

#endif

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif



void EnergyMonitor::voltage(unsigned int _inPinV, double _VCAL, double _PHASECAL)
{
  inPinV = _inPinV;
  VCAL = _VCAL;
  PHASECAL = _PHASECAL;
  offsetV = ADC_COUNTS>>1;
}

void EnergyMonitor::current(unsigned int _inPinI, double _ICAL)
{
  inPinI = _inPinI;
  ICAL = _ICAL;
  offsetI = ADC_COUNTS>>1;
}


void EnergyMonitor::voltageTX(double _VCAL, double _PHASECAL)
{
  inPinV = 2;
  VCAL = _VCAL;
  PHASECAL = _PHASECAL;
  offsetV = ADC_COUNTS>>1;
}

void EnergyMonitor::currentTX(unsigned int _channel, double _ICAL)
{
  if (_channel == 1) inPinI = 3;
  if (_channel == 2) inPinI = 0;
  if (_channel == 3) inPinI = 1;
  ICAL = _ICAL;
  offsetI = ADC_COUNTS>>1;
}


void EnergyMonitor::calcVI(unsigned int crossings, unsigned int timeout)
{
  #if defined emonTxV3
  int SupplyVoltage=3300;
  #else
  int SupplyVoltage = readVcc();
  #endif

  unsigned int crossCount = 0;                          
  unsigned int numberOfSamples = 0;                        

  unsigned long start = millis();   
  while(1)                                  
  {
    startV = analogRead(inPinV);                    
    if ((startV < (ADC_COUNTS*0.55)) && (startV > (ADC_COUNTS*0.45))) break;
    if ((millis()-start)>timeout) break;
  }

  start = millis();

  while ((crossCount < crossings) && ((millis()-start)<timeout))
  {
    numberOfSamples++;                    
    lastFilteredV = filteredV;             

  
    sampleV = analogRead(inPinV);                 
    sampleI = analogRead(inPinI);                 

   
    
    offsetV = offsetV + ((sampleV-offsetV)/1024);
    filteredV = sampleV - offsetV;
    offsetI = offsetI + ((sampleI-offsetI)/1024);
    filteredI = sampleI - offsetI;

    
    sqV= filteredV * filteredV;                 
    sumV += sqV;                                

    sqI = filteredI * filteredI;                
    sumI += sqI;                                


    phaseShiftedV = lastFilteredV + PHASECAL * (filteredV - lastFilteredV);


    instP = phaseShiftedV * filteredI;          
    sumP +=instP;                               

    
    lastVCross = checkVCross;
    if (sampleV > startV) checkVCross = true;
                     else checkVCross = false;
    if (numberOfSamples==1) lastVCross = checkVCross;

    if (lastVCross != checkVCross) crossCount++;
  }

 

  double V_RATIO = VCAL *((SupplyVoltage/1000.0) / (ADC_COUNTS));
  Vrms = V_RATIO * sqrt(sumV / numberOfSamples);

  double I_RATIO = ICAL *((SupplyVoltage/1000.0) / (ADC_COUNTS));
  Irms = I_RATIO * sqrt(sumI / numberOfSamples);

  realPower = V_RATIO * I_RATIO * sumP / numberOfSamples;
  apparentPower = Vrms * Irms;
  powerFactor=realPower / apparentPower;

  
  sumV = 0;
  sumI = 0;
  sumP = 0;
}

double EnergyMonitor::calcIrms(unsigned int Number_of_Samples)
{

  #if defined emonTxV3
    int SupplyVoltage=3300;
  #else
    int SupplyVoltage = readVcc();
  #endif


  for (unsigned int n = 0; n < Number_of_Samples; n++)
  {
    sampleI = analogRead(inPinI);

    
    offsetI = (offsetI + (sampleI-offsetI)/1024);
    filteredI = sampleI - offsetI; 
    sqI = filteredI * filteredI;
    sumI += sqI;
  }
  double I_RATIO = ICAL *((SupplyVoltage/1000.0) / (ADC_COUNTS));
  Irms = I_RATIO * sqrt(sumI / Number_of_Samples);
  sumI = 0;
  return Irms;
}

void EnergyMonitor::serialprint()
{
  Serial.print(realPower);
  Serial.print(' ');
  Serial.print(apparentPower);
  Serial.print(' ');
  Serial.print(Vrms);
  Serial.print(' ');
  Serial.print(Irms);
  Serial.print(' ');
  Serial.print(powerFactor);
  Serial.println(' ');
  delay(100);
}


long EnergyMonitor::readVcc() {
  long result;


  #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328__) || defined (__AVR_ATmega328P__)
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined(__AVR_ATmega644__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_AT90USB1286__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  ADCSRB &= ~_BV(MUX5);   
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);

  #endif


  #if defined(__AVR__)
  delay(2);                                        
  ADCSRA |= _BV(ADSC);                             
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = READVCC_CALIBRATION_CONST / result;  
  return result;
  #elif defined(__arm__)
  return (3300);                                  
  #else
  return (3300);                                  
  #endif
}
// WebServer server(8070);

// void handlePinOn() {
//   server.send(200, "text/plain", "Pin is now ON");
//   digitalWrite(5,HIGH);  
// }
// void handlePinOff() {
//   server.send(200, "text/plain", "Pin is now OFF");
//   digitalWrite(5, LOW); 
//   }
const char* ssid = "realme GT 2";
const char* password = "11111111";
const char* serverUrl = "http://192.168.134.192:8090/readings";
EnergyMonitor emon1,emon2,emon3;
const int sensorPin1 = 34;  // Analog pin for phase 1
const int sensorPin2 = 35;  // Analog pin for phase 2
const int sensorPin3 = 32;  // Analog pin for phase 3
bool graphStart = false;
unsigned long timer=0;
float u=0;
double voltage1,voltage2,voltage3,Irms1,Irms2,Irms3,seq,f1,f2,f3;
float readValue;
ZMPT101B voltageSensor1(34, 50.0);
ZMPT101B voltageSensor2(35, 50.0);
ZMPT101B voltageSensor3(32, 50.0);
void setup() {
  Serial.begin(115200);
  EEPROM.begin(512);
  pinMode(21,OUTPUT);
  pinMode(22,OUTPUT);
  pinMode(23,OUTPUT);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to WiFi");
  // server.on("/on", handlePinOn);
  // server.on("/off", handlePinOff);
  voltageSensor1.setSensitivity(1025.0f);
  voltageSensor2.setSensitivity(1035.0f);
  voltageSensor3.setSensitivity(1035.0f);
  emon1.current(33, 6); 
  emon2.current(36, 6);            
  emon3.current(39, 6);
}

void loop() {
  // server.handleClient();
  voltcurrpowReading();
  frequency();
  Serial.println(" ");
  Sequence();
  Serial.println(" ");
  if (voltage1>=100 && voltage2>100 && voltage3>100){
  sendData();
  }
}
void sendData() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverUrl);
    http.addHeader("Content-Type", "application/json");
    StaticJsonDocument<256> doc;
    doc["voltage1"] = voltage1;
    doc["current1"] = Irms1;
    doc["power1"] = voltage1*Irms1;
    doc["frequency1"] = f1;
    doc["voltage2"] = voltage2;
    doc["current2"] = Irms2;
    doc["power2"] = voltage2*Irms2;
    doc["frequency2"] = f2;
    doc["voltage3"] = voltage3;
    doc["current3"] = Irms3;
    doc["power3"] = voltage3*Irms3;
    doc["frequency3"] = f3;
    doc["sequence"] = seq; 
    doc["deviceId"] = 1;
    doc["powerunit"]=readValue;
    String requestBody;
    serializeJson(doc, requestBody);
    int httpResponseCode = http.POST(requestBody);
    if (httpResponseCode > 0) {
      String response = http.getString();
      Serial.println(httpResponseCode);
      Serial.println(response);
    } else {
      Serial.print("Error on sending POST: ");
      Serial.println(httpResponseCode);
    }
    http.end(); 
  } else {
    Serial.println("Error in WiFi connection");
  }
}
void voltcurrpowReading(){
  voltage1 = voltageSensor1.getRmsVoltage(50);
  voltage2 = voltageSensor2.getRmsVoltage(50);
  voltage3 = voltageSensor3.getRmsVoltage(50);
  Serial.print("Voltage 1 : ");
  Serial.println(voltage1);
  Serial.print("Voltage 2 :");
  Serial.println(voltage2);
  Serial.print("Voltage 3 : ");
  Serial.println(voltage3);
  Serial.println(" ");
  if(voltage1>240 || voltage2>240){
      digitalWrite(21,HIGH);
      digitalWrite(22,HIGH);
 }
 if(voltage3>175){
      digitalWrite(23,HIGH);
      delay(3000);
      digitalWrite(23,LOW);
 }

  Irms1 = emon1.calcIrms(1480);
  Irms2 = emon2.calcIrms(1480); 
  Irms3 = emon3.calcIrms(1480);
  Serial.print("I1 : ");
  Serial.println(Irms1);
  Serial.print("I2 : ");
  Serial.println(Irms2);
  Serial.print("I3 : ");
  Serial.println(Irms3);
  float p1=voltage1*Irms1,p2=voltage2*Irms2,p3=voltage3*Irms3;
  Serial.print("P1 : ");
  Serial.println(p1);
  Serial.print("P2 : ");
  Serial.println(p2);
  Serial.print("P3 : ");
  Serial.println(p3);
  Serial.println(" ");
  u =(((p1+p2+p3)*3)/(1000*60*60));
  yourVariable=readEEPROM()+u;
  Serial.println(yourVariable);
  writeEEPROM();
  readValue = readEEPROM();
}
void Sequence(){
  while(true){
  int volt1=analogRead(34);
  float v1 = volt1 * (3.3 / 4095.0);
  if(v1>=1.40 && v1<=1.60){
  int volt2=analogRead(35);
  float v2 = volt2 * (3.3 / 4095.0);
  int volt3=analogRead(32);
  float v3 = volt3 * (3.3 / 4095.0);
  delay(2);
  volt1=analogRead(34);
  float v11 = volt1 * (3.3 / 4095.0);
  if((v1>=1.45 && v1<=1.60)&&(v1<v11)){
    if(v1<v3 && v1>v2){
      Serial.println("Sequence Correct");
      seq=true;
      return;
    }
    else{
      Serial.println("Wrong");
      seq=false;
      return;
    }
  }
  }
  }
}
void frequency() {
  int a[]={34,35,32};
  for (int i = 0; i < 3; i++) { 
    int pin = a[i]; 
    timer=0;
    while (true) {
      float volt1 = analogRead(pin);
      float v1 = (volt1 * 3.3) / 4096;
      if (v1 >= 1.45 && v1 <= 1.65) {
        delay(3);
        volt1 = analogRead(pin);
        float v11 = (volt1 * 3.3) / 4096;
        if ((v1 < v11) && graphStart && (millis() - timer) > 5) {
          graphStart = false;

          float elapsed = millis() - timer;
          if (elapsed <= 30) {
            float b = 1000 / elapsed;
            Serial.print("Frequency ");
            Serial.print(i+1);
            Serial.print(" : ");
            Serial.print(b);
            Serial.println(" Hz");
            if(i==0){
              f1=b;
            }
            else if(i==1){
              f2=b;
            }
            else if(i==2){
              f3=b;
            }
            break;
          }
        }
        if (v1 < v11 && !graphStart) {
          graphStart = true;
          timer = millis();
        }
      }
    }
  }
}
void writeEEPROM() {
  EEPROM.put(EEPROM_ADDR, yourVariable);
  EEPROM.commit(); // Saves the data to EEPROM
}
float readEEPROM() {
  float value;
  EEPROM.get(EEPROM_ADDR, value);
  return value;
}


