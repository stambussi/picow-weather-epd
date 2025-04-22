/* Main program for picow-weather-epd.
 * Copyright (C) 2022-2024  Luke Marzen
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <Arduino.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <time.h>
#include <WiFi.h>
#include <Wire.h>

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "_locale.h"
#include "api_response.h"
#include "client_utils.h"
#include "config.h"
#include "display_utils.h"
#include "icons/icons_48x48.h"
#include "icons/icons_196x196.h"
#include "renderer.h"
#if defined(USE_HTTPS_WITH_CERT_VERIF) || defined(USE_HTTPS_WITH_CERT_VERIF)
#include <WiFiClientSecure.h>
#endif
#ifdef USE_HTTPS_WITH_CERT_VERIF
#include "cert.h"
#endif

// too large to allocate locally on stack
static owm_resp_onecall_t owm_onecall;
static owm_resp_air_pollution_t owm_air_pollution;

/**
 * Poor man's low power sleep function
 * Hopefully a temporary solution until a proper low-power setting function can be developed
 */
void lowPowerSleep(uint32_t sleepMs)
{
#if DEBUG_LEVEL >= 1
    printHeapUsage();
#endif

    Serial.println("Setting low-power state: cpu_freq: 125MHz --> 20MHz, voltage: 1.1V --> 0.95V");
    set_sys_clock_khz(20000, false); // set system clock to 20 mhz
    sleep_ms(2);
    vreg_set_voltage(VREG_VOLTAGE_0_95); // 0.85V did not work, 0.95V seems pretty stable

    Serial.print("Sleeping for ");
    Serial.print(sleepMs * 1000);
    Serial.println(" s");
    sleep_ms(sleepMs);

    Serial.println("Waking up from sleep and exiting low-power state");
    vreg_set_voltage(VREG_VOLTAGE_DEFAULT); // corresponds to 1.10V
    sleep_ms(2);
    set_sys_clock_khz(125000, true); // set system clock back to 125 mhz (og 133, but slightly lower is fine)
}

void controlLoop()
{
    uint64_t sleepDurationMs = SLEEP_DURATION * 60 * 1000;
    String errMsg1 = {};
    String errMsg2 = {};
    const uint8_t *err_bitmap_ptr = NULL;
    bool encounteredError = false;
    String statusStr = {};
    tm timeInfo = {};
    int wifiRSSI = 0; // “Received Signal Strength Indicator"
    int retries = 0;
    wl_status_t wifiStatus = (wl_status_t)0;
    bool timeConfigured = false;
    // Initialize both of these structs to 0 on each iteration
    owm_onecall = {};
    owm_air_pollution = {};
#ifdef USE_HTTP
    WiFiClient client;
#elif defined(USE_HTTPS_NO_CERT_VERIF)
    WiFiClientSecure client;
    client.setInsecure();
#elif defined(USE_HTTPS_WITH_CERT_VERIF)
    WiFiClientSecure client;
    client.setCACert(cert_Sectigo_RSA_Domain_Validation_Secure_Server_CA);
#endif
    int httpRxStatus = 0;
    float inTemp = NAN;
    float inHumidity = NAN;
#if DEBUG_LEVEL >= 1
    printHeapUsage();
#endif

    disableBuiltinLED();

    // START WIFI
    // Try to connect twice
    while (wifiStatus != WL_CONNECTED && retries < 2)
    {
        wifiStatus = startWiFi(&wifiRSSI);
        retries++;
    }
    if (wifiStatus != WL_CONNECTED)
    { // WiFi Connection Failed
        encounteredError = true;
        err_bitmap_ptr = wifi_x_48x48;
        statusStr = "Error Occurred: See Alert Message for Details";
        if (wifiStatus == WL_NO_SSID_AVAIL)
        {
            Serial.println(TXT_NETWORK_NOT_AVAILABLE);
            errMsg1 = String(TXT_NETWORK_NOT_AVAILABLE);
        }
        else
        {
            Serial.println(TXT_WIFI_CONNECTION_FAILED);
            errMsg1 = String(TXT_WIFI_CONNECTION_FAILED);
        }
        goto screen_draw;
    }

    // TIME SYNCHRONIZATION
    configTzTime(TIMEZONE, NTP_SERVER_1, NTP_SERVER_2);
    // Try to sync time twice
    retries = 0;
    while (!timeConfigured && retries < 2)
    {
        timeConfigured = waitForNTPSync(&timeInfo);
        retries++;
    }
    if (!timeConfigured)
    {
        encounteredError = true;
        err_bitmap_ptr = wi_time_4_48x48;
        statusStr = "Error Occurred: See Alert Message for Details";
        Serial.println(TXT_TIME_SYNCHRONIZATION_FAILED);
        errMsg1 = String(TXT_TIME_SYNCHRONIZATION_FAILED);
        goto screen_draw;
    }

    // MAKE API REQUESTS
    // Try to call API twice
    retries = 0;
    while (httpRxStatus != HTTP_CODE_OK && retries < 2)
    {
        httpRxStatus = getOWMonecall(client, owm_onecall);
        retries++;
    }
    if (httpRxStatus != HTTP_CODE_OK)
    {
        encounteredError = true;
        err_bitmap_ptr = wi_cloud_down_48x48;
        statusStr = "Error Occurred: See Alert Message for Details";
        errMsg1 = "One Call " + OWM_ONECALL_VERSION + " API";
        errMsg2 = String(httpRxStatus, DEC) + ": " + getHttpResponsePhrase(httpRxStatus);
        Serial.println(errMsg2);
        goto screen_draw;
    }

    retries = 0;
    httpRxStatus = 0;
    while (httpRxStatus != HTTP_CODE_OK && retries < 2)
    {
        httpRxStatus = getOWMairpollution(client, owm_air_pollution);
        retries++;
    }
    if (httpRxStatus != HTTP_CODE_OK)
    {
        encounteredError = true;
        err_bitmap_ptr = wi_cloud_down_48x48;
        statusStr = "Error Occurred: See Alert Message for Details";
        errMsg1 = "Air Pollution API";
        errMsg2 = String(httpRxStatus, DEC) + ": " + getHttpResponsePhrase(httpRxStatus);
        goto screen_draw;
    }

    // GET INDOOR TEMPERATURE AND HUMIDITY, start BME280 (if present)
#if BME280_PRESENT
    pinMode(PIN_BME_PWR, OUTPUT);
    digitalWrite(PIN_BME_PWR, HIGH);
    Serial.print(String(TXT_READING_FROM) + " BME280... ");
    TwoWire I2C_bme = TwoWire(0);
    Adafruit_BME280 bme;

    I2C_bme.begin(PIN_BME_SDA, PIN_BME_SCL, 100000); // 100kHz
    if (bme.begin(BME_ADDRESS, &I2C_bme))
    {
        inTemp = bme.readTemperature();  // Celsius
        inHumidity = bme.readHumidity(); // %

        // check if BME readings are valid
        // note: readings are checked again before drawing to screen. If a reading
        //       is not a number (NAN) then an error occurred, a dash '-' will be
        //       displayed.
        if (std::isnan(inTemp) || std::isnan(inHumidity))
        {
            statusStr = "BME " + String(TXT_READ_FAILED);
            Serial.println(statusStr);
        }
        else
        {
            Serial.println(TXT_SUCCESS);
        }
    }
    else
    {
        statusStr = "BME " + String(TXT_NOT_FOUND); // check wiring
        Serial.println(statusStr);
    }
    digitalWrite(PIN_BME_PWR, LOW);
#endif

screen_draw:
    String refreshTimeStr;
    getRefreshTimeStr(refreshTimeStr, timeConfigured, &timeInfo);
    String dateStr;
    getDateStr(dateStr, &timeInfo);

    killWiFi(); // WiFi no longer needed
    // RENDER FULL REFRESH
    initDisplay();
    do
    {
        drawCurrentConditions(owm_onecall.current, owm_onecall.daily[0],
                              owm_air_pollution, inTemp, inHumidity);
        drawForecast(owm_onecall.daily, timeInfo);
        drawLocationDate(CITY_STRING, dateStr);
        drawOutlookGraph(owm_onecall.hourly, timeInfo);
        if (encounteredError == true)
        {
            drawPartialError(err_bitmap_ptr, dateStr, errMsg1, errMsg2);
        }
        else
        {
#if DISPLAY_ALERTS
        drawAlerts(owm_onecall.alerts, CITY_STRING, dateStr);
#endif
        }
        drawStatusBar(statusStr, refreshTimeStr, wifiRSSI);
    } while (display.nextPage());
    powerOffDisplay();

    // Low Power Sleep
    lowPowerSleep(sleepDurationMs);
}

// Program entry point.
void setup()
{

    Serial.begin(115200);
    delay(500);
    // SPI needs to be initialized here
    SPIClassRP2040 SPIn(spi1, PIN_EPD_MISO, PIN_EPD_CS, PIN_EPD_SCK, PIN_EPD_MOSI);
    display.epd2.selectSPI(SPIn, SPISettings(4000000, MSBFIRST, SPI_MODE0));

    for (;;)
    {
        controlLoop();
    }
} // end setup

// Will supposedly never get here
void loop()
{
} // end loop
