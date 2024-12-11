#include <Adafruit_AHTX0.h>
#include <Adafruit_SGP30.h>
#include <Arduino.h>
#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>
#include <WiFiMulti.h>
#include <Wire.h>

#include "config.h"

#define SDA_PIN 21
#define SCL_PIN 22
#define I2C_FREQUENCY 100000

Adafruit_AHTX0 aht;
Adafruit_SGP30 sgp;

InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN, InfluxDbCloud2CACert);
WiFiMulti wifiMulti;

Point sensor("airquality");

const int aggregationInterval = 10;  // Aggregation interval in (approximate) seconds
const int measurementInterval = 1;   // Measurement interval in (approximate) seconds

float temperatureSum = 0;
float humiditySum = 0;
float TVOCSum = 0;
float eCO2Sum = 0;
float rawH2Sum = 0;
float rawEthanolSum = 0;

int measurementCount = 0;

/* return absolute humidity [mg/m^3] with approximation formula
 * @param temperature [°C]
 * @param humidity [%RH]
 */
uint32_t getAbsoluteHumidity(float temperature, float humidity) {
    // approximation formula from Sensirion SGP30 Driver Integration
    // chapter 3.15
    const float absoluteHumidity =
        216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) /
                  (273.15f + temperature));                                                     // [g/m^3]
    const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity);  // [mg/m^3]
    return absoluteHumidityScaled;
}

void scanI2C() {
    byte error, address;
    int nDevices;

    Serial.println("Scanning I2C bus...");

    nDevices = 0;
    for (address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
            Serial.print("I2C device found at address 0x");
            if (address < 16) Serial.print("0");
            Serial.print(address, HEX);
            Serial.println(" !");
            nDevices++;
        } else if (error == 4) {
            Serial.print("Unknown error at address 0x");
            if (address < 16) Serial.print("0");
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0) {
        Serial.println("No I2C devices found\n");
    } else {
        Serial.println("done\n");
    }
}

void setup() {
    Serial.begin(9600);
    delay(2000);

    WiFi.mode(WIFI_STA);
    wifiMulti.addAP(WIFI_SSID, WIFI_PASSWORD);

    Serial.print("Connecting to wifi ");
    Serial.print(WIFI_SSID);
    while (wifiMulti.run() != WL_CONNECTED) {
        Serial.print(".");
        delay(100);
    }
    Serial.println();

    timeSync(TZ_INFO, "pool.ntp.org", "time.nis.gov");

    client.setInsecure(true);

    // Check InfluxDB server connection
    if (client.validateConnection()) {
        Serial.print("Connected to InfluxDB: ");
        Serial.println(client.getServerUrl());
    } else {
        Serial.print("InfluxDB connection failed: ");
        Serial.println(client.getLastErrorMessage());
    }

    sensor.addTag("location", SENSOR_LOCATION);

    Wire.begin(SDA_PIN, SCL_PIN, I2C_FREQUENCY);
    scanI2C();

    Serial.println("Initializing sensors...");

    if (!aht.begin()) {
        Serial.println("AHT21 not found!");
        while (1) delay(10);
    } else {
        Serial.println("AHT21 found!");
    }

    if (!sgp.begin()) {
        Serial.println("SGP30 not found!");
        while (1);
    } else {
        Serial.println("SGP30 found!");
    }

    // prefill SGP30 baseline values
    if (PREFILL_BASE_VALUES) {
        uint16_t eCO2_base, TVOC_base;
        eCO2_base = ECO2_BASE;
        TVOC_base = TVOC_BASE;
        sgp.setIAQBaseline(eCO2_base, TVOC_base);
    }

    Serial.println("Initialization done! Waiting for sensor to warm up...");
    delay(20000);
    Serial.println("Measurement started!");

    // inital measurment (likely to be off)
    sensors_event_t humidity, temp;
    sgp.setHumidity(getAbsoluteHumidity(temp.temperature, humidity.relative_humidity));
    if (!sgp.IAQmeasure() || !sgp.IAQmeasureRaw()) {
        Serial.println("SGP30 measurement failed");
    }
    delay(1000);
}

void loop() {
    // Measure values
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);

    sgp.setHumidity(getAbsoluteHumidity(temp.temperature, humidity.relative_humidity));

    if (!sgp.IAQmeasure() || !sgp.IAQmeasureRaw()) {
        Serial.println("SGP30 measurement failed");
    }

    temperatureSum += temp.temperature;
    humiditySum += humidity.relative_humidity;
    TVOCSum += sgp.TVOC;
    eCO2Sum += sgp.eCO2;
    rawH2Sum += sgp.rawH2;
    rawEthanolSum += sgp.rawEthanol;
    measurementCount++;

    Serial.print("Measured Temperature: ");
    Serial.print(temp.temperature);
    Serial.print(" C, Humidity: ");
    Serial.print(humidity.relative_humidity);
    Serial.print(" %, TVOC: ");
    Serial.print(sgp.TVOC);
    Serial.print(" ppb, eCO2: ");
    Serial.print(sgp.eCO2);
    Serial.println(" ppm");

    // Aggregate and send data
    if (measurementCount >= aggregationInterval) {
        if (measurementCount > 0) {
            float avgTemperature = temperatureSum / measurementCount;
            float avgHumidity = humiditySum / measurementCount;
            float avgTVOC = TVOCSum / measurementCount;
            float avgCO2 = eCO2Sum / measurementCount;
            float avgRawH2 = rawH2Sum / measurementCount;
            float avgRawEthanol = rawEthanolSum / measurementCount;

            sensor.clearFields();
            sensor.addField("temperature", avgTemperature);
            sensor.addField("humidity", avgHumidity);
            sensor.addField("TVOC", avgTVOC);
            sensor.addField("eCO2", avgCO2);
            sensor.addField("rawH2", avgRawH2);
            sensor.addField("rawEthanol", avgRawEthanol);

            if (SAVE_BASE_VALUES) {
                uint16_t base_TVOC;
                uint16_t base_eCO2;

                if (sgp.getIAQBaseline(&base_eCO2, &base_TVOC)) {
                    sensor.addField("base_eCO2", base_eCO2);
                    sensor.addField("base_TVOC", base_TVOC);
                }
            }

            // Print what we are exactly writing
            Serial.print("Writing: ");
            Serial.println(sensor.toLineProtocol());

            // Check WiFi connection and reconnect if needed
            if (wifiMulti.run() != WL_CONNECTED) {
                Serial.println("Wifi connection lost");
            }

            // Write point
            if (!client.writePoint(sensor)) {
                Serial.print("InfluxDB write failed: ");
                Serial.println(client.getLastErrorMessage());
            }

            // Reset the sums and measurment count
            temperatureSum = 0;
            humiditySum = 0;
            TVOCSum = 0;
            eCO2Sum = 0;
            rawH2Sum = 0;
            rawEthanolSum = 0;
            measurementCount = 0;

            delay(50);
        }
    } else {
        delay(1000);
    }
}