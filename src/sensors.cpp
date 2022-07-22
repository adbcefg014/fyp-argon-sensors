/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "d:/JSN/Desktop/repos/c53-iot/sensors/src/sensors.ino"
/*
 * Project sensors
 * Description:
 * Author:
 * Date:
 */

// Code for all one complete sensor node
#include <Wire.h>
#include <Particle.h>
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <BH1750.h>			// https://github.com/claws/BH1750
#include <Adafruit_BME280.h>
#include <SparkFun_SCD30_Arduino_Library.h>
#include <Adafruit_PM25AQI.h>
#include <Adafruit_VEML6070.h>

//SYSTEM_MODE(MANUAL);
//SYSTEM_THREAD (ENABLED);

void setup();
void loop();
#line 22 "d:/JSN/Desktop/repos/c53-iot/sensors/src/sensors.ino"
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

void testForConnectivity();
void get_value();
void calibrate_bh();


// setup() runs once, when the device is first turned on.
void setup() {

	// Put initialization like pinMode and begin functions here.
	Wire.begin();
	pinMode(D7,OUTPUT);
	Serial.begin(9600);
	while (!bh.begin(BH1750::ONE_TIME_HIGH_RES_MODE))
	{
		delay(500);
		Serial.println("Trying to connect BH1750 Lux Sensor");
	}
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
	aqi.begin_I2C();
	Serial.println("Zio Qwiic Loudness Sensor Master Awake");
	testForConnectivity();
	uv.begin(VEML6070_1_T);

	// Adjust light sensor calibration
	calibrate_bh();
	bh.configure(BH1750::ONE_TIME_LOW_RES_MODE);
}


// loop() runs over and over again, as quickly as it can execute.
void loop() {
	// The core of your code will likely live here.
	digitalWrite(D7,HIGH);
	Serial.println("==================================================================");
	time_t time = Time.now();
	//time_t time = Time.zone(+8);
	Serial.println(Time.format(time, TIME_FORMAT_DEFAULT));

	//LUX Sensor (BH1750); continuous mode takes measurements non-stop, one time mode takes a measurement then sleep
	while (!bh.measurementReady(true)) {
		delay(20);
	}
	Serial.println(String::format("Light level: %.1f lux", bh.readLightLevel()));
	bh.configure(BH1750::ONE_TIME_LOW_RES_MODE);

	//CO2 Sensor (SCD30)
	if (airSensor.dataAvailable())
	{
		Serial.print("CO2(ppm): ");
		Serial.print(airSensor.getCO2());
		Serial.print(" Temperature(*C): ");
		Serial.print(airSensor.getTemperature(), 1);
		Serial.print(" Humidity(%): ");
		Serial.print(airSensor.getHumidity(), 1);
		Serial.println();
	}
	else
		Serial.println("No data");
	
	//Particulate Sensor (PMSA003I)
	PM25_AQI_Data data;
	if (!aqi.read(&data))
	{
		Serial.println("Could not read from AQI");
		delay(5000); // try again in a bit!
		return;
	}
	float pm10s = data.pm10_standard;
	float pm25s = data.pm25_standard;
	float pm100s = data.pm100_standard;
	//float pm10e = data.pm10_env;
	//float pm25e = data.pm25_env;
	//float pm100e = data.pm100_env;

	Serial.println(String::format("Standard PM -- PM1.0: %.2f | PM2.5: %.2f | PM10.0: %.2f", pm10s, pm25s, pm100s));
	//Serial.println(String::format("Environmental PM -- PM1.0: %.2f| PM2.5: %.2f| PM10.0: %.2f", pm10e, pm25e, pm100e));

	//Peak Sound Sensor (SPARKFUN SEN-15892)
	get_value();

	// UV Sensor (VEML 6070)
	Serial.print("UV light level: "); Serial.println(uv.readUV());

	//float pressure = bme.readPressure()/100.0F;
	//float temp = bme.readTemperature();
	//float humid = bme.readHumidity();
	//Pressure, Temperature, Humidity Sensor (BME280)
	//Serial.println(String::format("Pressure: %.2f mbar | Temperature: %.2f *C | Humidity %.2f %",pressure, temp, humid));

	Serial.println(String::format("Pressure: %.2f mbar",(bme.readPressure()/100.0F)));
	Serial.print("Humidity: ");
	Serial.print(bme.readHumidity());
	Serial.println(" %");
	Serial.println(String::format("Temperature: %.2f *C",bme.readTemperature()));
	digitalWrite(D7,LOW);

	delay(30000);
}

void get_value()
{
	Wire.beginTransmission(qwiicAddress);
	Wire.write(COMMAND_GET_VALUE); // command for status
	Wire.endTransmission(); // stop transmitting //this looks like it was essential.
	Wire.requestFrom(qwiicAddress, 2); // request 1 bytes from slave device qwiicAddress

	while (Wire.available())
	{ // slave may send less than requested
		uint8_t ADC_VALUE_L = Wire.read();
		// Serial.print("ADC_VALUE_L: ");
		// Serial.println(ADC_VALUE_L,DEC);
		uint8_t ADC_VALUE_H = Wire.read();
		// Serial.print("ADC_VALUE_H: ");
		// Serial.println(ADC_VALUE_H,DEC);
		ADC_VALUE=ADC_VALUE_H;
		ADC_VALUE<<=8;
		ADC_VALUE|=ADC_VALUE_L;
		float dB = (ADC_VALUE+83.2073) / 11.003; //emprical formula to convert ADC value to dB
		//Serial.print("ADC_VALUE: ");
		//Serial.println(ADC_VALUE,DEC);
		Serial.printlnf("ADC VALUE: %u, dB: %.2f",ADC_VALUE,dB);
	}
}

// testForConnectivity() checks for an ACK from an Sensor. If no ACK
// program freezes and notifies user.
void testForConnectivity()
{
	Wire.beginTransmission(qwiicAddress);
	//check here for an ACK from the slave, if no ACK don't allow change?
	if (Wire.endTransmission() != 0)
	{
		Serial.println("Check connections. No slave attached.");
		while (1);
	}
}

// Calibration of BH1750 sensor at boot up
void calibrate_bh() 
{
	while (true)
	{
		if (bh.measurementReady(true)) {
			float lux = bh.readLightLevel();

			if (lux < 0) {
				Serial.println(F("Error condition detected in BH1750"));
			} else {
				if (lux > 40000.0) {
					// reduce measurement time - needed in direct sun light
					if (bh.setMTreg(32)) {
						Serial.println(
								F("Setting MTReg to low value for high light environment"));
					} else {
						Serial.println(
								F("Error setting MTReg to low value for high light environment"));
					}
				} else {
					if (lux > 10.0) {
						// typical light environment
						if (bh.setMTreg(69)) {
							Serial.println(F(
									"Setting MTReg to default value for normal light environment"));
						} else {
							Serial.println(F("Error setting MTReg to default value for normal "
															"light environment"));
						}
					} else {
						if (lux <= 10.0) {
							// very low light environment
							if (bh.setMTreg(138)) {
								Serial.println(
										F("Setting MTReg to high value for low light environment"));
							} else {
								Serial.println(F("Error setting MTReg to high value for low "
																"light environment"));
							}
						}
					}
				}
			}
			Serial.println(F("--------------------------------------"));
			return;
		}
		delay(5000);
	}
	return;
}