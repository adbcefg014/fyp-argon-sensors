/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "d:/JSN/Desktop/repos/c53-iot/sensors/src/sensors.ino"
// Code for all one complete sensor node
#include <Wire.h>
#include <Particle.h>
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <BH1750.h>	
#include <Adafruit_BME280.h>
#include <SparkFun_SCD30_Arduino_Library.h>
#include <Adafruit_PM25AQI.h>
#include <Adafruit_VEML6070.h>


//SYSTEM_MODE(MANUAL);
//SYSTEM_THREAD (ENABLED);

void setup();
void loop();
#line 16 "d:/JSN/Desktop/repos/c53-iot/sensors/src/sensors.ino"
BH1750 bh;
#define SEALEVELPRESSURE_HPA (1013.25)
#define BME_ADDRESS 0x77
Adafruit_BME280 bme;
SCD30 airSensor;
Adafruit_PM25AQI aqi = Adafruit_PM25AQI();
Adafruit_VEML6070 uv = Adafruit_VEML6070();
#define COMMAND_GET_VALUE 0x05
#define COMMAND_NOTHING_NEW 0x99
const byte qwiicAddress = 0x30;
uint16_t ADC_VALUE = 0;
float dBnumber = 0.0;

void initializeSensors();
String getSensorReadings();
void qwiicTestForConnectivity();
void qwiicGetValue();

// setup() runs once, when the device is first turned on.
void setup() {

	// Put initialization like pinMode and begin functions here.
	Wire.begin();
	pinMode(D7,OUTPUT);
	Serial.begin(9600);

	initializeSensors();
}


// loop() runs over and over again, as quickly as it can execute.
void loop() {
	// The core of your code will likely live here.
	if (Particle.connected() == false) {
		Particle.connect();
	}

	digitalWrite(D7,HIGH);
	String sensorData = getSensorReadings();
	Particle.publish("sensor-reading", sensorData);
	digitalWrite(D7,LOW);

	SystemSleepConfiguration sleepConfig;
	sleepConfig.mode(SystemSleepMode::ULTRA_LOW_POWER).duration(1min);
	System.sleep(sleepConfig);
}


/* Main program flow above, below are helper functions */

void initializeSensors()
{
	while (!bh.begin())
	{
		delay(500);
		Serial.println("Trying to connect BH1750 Lux Sensor");
	}
	bh.set_sensor_mode(BH1750::forced_mode_low_res);

	while (!bme.begin())
	{
		delay(500);
		Serial.println("Trying to connect BME280 PTH Sensor");
	}

	while (!airSensor.begin())
	{
		delay(500);
		Serial.println("Trying to connect SCD30 CO2 Sensor");
	}
	airSensor.setMeasurementInterval(2);

	aqi.begin_I2C();	// Particulate sensor PM2.5

	Serial.println("Zio Qwiic Loudness Sensor Master Awake");
	qwiicTestForConnectivity();

	uv.begin(VEML6070_1_T);
}

String getSensorReadings()
{
	/*
	Planned JSON Structure:
	{
		"deviceID": xxxxxxx
		"DateTime": xxxxxxx
		"sensor1":
			{
				"Measurement1": Value1
				"Measurement2": Value2
			}
	}
	*/

	// Preparations for JSON string pointer
	char buf[400];
	memset(buf, 0, sizeof(buf));
	JSONBufferWriter writer(buf, sizeof(buf) - 1);
	writer.beginObject();

	// Device ID as 1st data entry
	writer.name("DeviceID").value(System.deviceID());

	// DateTime data entry
	writer.name("DateTime").value(Time.now());

	// LUX Sensor (BH1750), decimal precision to .1
	bh.make_forced_measurement();
	float lux = (int)(bh.get_light_level() * 10 + 0.5);	
		// + 0.5 for rounding off number
	lux = (float)lux / 100;
	writer.name("BH1750");
	writer.beginObject();
	writer.name("Light level(lux)").value(lux);
	writer.endObject();

	// CO2 Sensor (SCD30)
	if (airSensor.dataAvailable())
	{
		writer.name("SCD30");
		writer.beginObject();
		writer.name("CO2(ppm)").value(airSensor.getCO2());
		writer.name("Temperature(*C)").value(airSensor.getTemperature());
		writer.name("Humidity(%)").value(airSensor.getHumidity());
		writer.endObject();
	}
	
	// Particulate Sensor (PMSA003I)
	PM25_AQI_Data data;
	writer.name("PMSA003I");
	writer.beginObject();
	writer.name("Standard PM1.0").value(data.pm10_standard);
	writer.name("Standard PM2.5").value(data.pm25_standard);
	writer.name("Standard PM10").value(data.pm100_standard);
	writer.name("Environmental PM1.0").value(data.pm10_env);
	writer.name("Environmental PM2.5").value(data.pm25_env);
	writer.name("Environmental PM10").value(data.pm100_env);
	writer.endObject();

	// Peak Sound Sensor (SPARKFUN SEN-15892)
	dBnumber = 0.0;
	qwiicGetValue();
	writer.name("PMSA003I");
	writer.beginObject();
	writer.name("ADC Value").value(ADC_VALUE);
	writer.name("dB").value(dBnumber);
	writer.endObject();

	// UV Sensor (VEML 6070)
	writer.name("VEML6070");
	writer.beginObject();
	writer.name("UV light level").value(uv.readUV());
	writer.name("Pressure(mbar)").value(bme.readPressure()/100.0F);
	writer.name("Humidity(%)").value(bme.readHumidity());
	writer.name("Temperature(*C)").value(bme.readTemperature());
	writer.endObject();

	// Null terminator for end of JSON string
	writer.endObject();
	writer.buffer()[std::min(writer.bufferSize(), writer.dataSize())] = 0;
	return buf;
}


void qwiicGetValue()
{
	Wire.beginTransmission(qwiicAddress);
	Wire.write(COMMAND_GET_VALUE); // command for status
	Wire.endTransmission(); // stop transmitting //this looks like it was essential.
	Wire.requestFrom(qwiicAddress, 2); // request 1 bytes from slave device qwiicAddress

	while (Wire.available())
	{ // slave may send less than requested
		uint8_t ADC_VALUE_L = Wire.read();
		uint8_t ADC_VALUE_H = Wire.read();
		ADC_VALUE=ADC_VALUE_H;
		ADC_VALUE<<=8;
		ADC_VALUE|=ADC_VALUE_L;
		dBnumber = (ADC_VALUE+83.2073) / 11.003; //emprical formula to convert ADC value to dB
	}
}

// qwiicTestForConnectivity() checks for an ACK from an Sensor. If no ACK
// program freezes and notifies user.
void qwiicTestForConnectivity()
{
	Wire.beginTransmission(qwiicAddress);
	//check here for an ACK from the slave, if no ACK don't allow change?
	if (Wire.endTransmission() != 0)
	{
		Serial.println("Check connections. No slave attached.");
		while (1);
	}
}