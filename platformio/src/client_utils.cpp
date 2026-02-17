/* Client side utilities for esp32-weather-epd.
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

// built-in C++ libraries
#include <cstring>
#include <vector>

// arduino/esp32 libraries
#include <Arduino.h>
#include <HTTPClient.h>
#include <SPI.h>
#include <time.h>
#include <WiFi.h>
#include <WiFiManager.h>

// additional libraries
#include <Adafruit_BusIO_Register.h>
#include <ArduinoJson.h>

// header files
#include "_locale.h"
#include "api_response.h"
#include "aqi.h"
#include "client_utils.h"
#include "config.h"
#include "display_utils.h"
#include "renderer.h"
#ifndef USE_HTTP
  #include <WiFiClientSecure.h>
#endif

#ifdef USE_HTTP
  static const uint16_t OWM_PORT = 80;
#else
  static const uint16_t OWM_PORT = 443;
#endif

/* Power-on and connect WiFi.
 * Takes int parameter to store WiFi RSSI, or “Received Signal Strength
 * Indicator"
 *
 * Returns WiFi status.
 */
wl_status_t startWiFi(WiFiManager *wm, int *wifiRSSI)
{
    bool success;
    wl_status_t connection_status;

    success = wm->autoConnect();

    if (success)
    {
        *wifiRSSI = WiFi.RSSI();
        Serial.println("IP: " + WiFi.localIP().toString());
        connection_status = WL_CONNECTED;
    }
    else
    {
        connection_status = WL_CONNECT_FAILED;
    }
    return connection_status;
} // startWiFi

/* Disconnect and power-off WiFi.
 */
void killWiFi()
{
  Serial.println("Turning off WiFi module");
  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_OFF);
} // killWiFi

/**
 *  Implementation of configTzTime for rpipicow (taken from esp32-hal-time.c)
 */
void configTzTime(const char *tz, const char *s1, const char *s2)
{
  NTP.begin(s1, s2); // start NTP client

  setenv("TZ", tz, 1); // Set timezone env variable
  tzset(); // set timezone from tz env variable
}

/**
 *  Implementation of getLocalTime for rpipicow (taken from esp32-hal-time.c)
 */
bool getLocalTime(tm *info)
{
  time_t now;
  time(&now);
  localtime_r(&now, info);
  if(info->tm_year > (2016 - 1900))
  {
    return true;
  }
  delay(10);
  return false;
}

/* Prints the local time to serial monitor.
 *
 * Returns true if getting local time was a success, otherwise false.
 */
bool printLocalTime(tm *timeInfo)
{
  int attempts = 0;
  while (!getLocalTime(timeInfo) && attempts++ < 3)
  {
    Serial.println(TXT_FAILED_TO_GET_TIME);
    return false;
  }
  //Serial.println(timeInfo, "%A, %B %d, %Y %H:%M:%S");
  Serial.println(asctime(timeInfo));
  return true;
} // printLocalTime

/* Waits for NTP server time sync, adjusted for the time zone specified in
 * config.cpp.
 *
 * Returns true if time was set successfully, otherwise false.
 *
 * Note: Must be connected to WiFi to get time from NTP server.
 */
bool waitForNTPSync(tm *timeInfo)
{
  // Wait for NTP synchronization to complete
  unsigned long timeout = millis() + NTP_TIMEOUT;

  Serial.print(TXT_WAITING_FOR_SNTP);
  // This below seemed wrong, it was incorrectly returning false even when the time server was synced
  // if(!NTP.waitSet([]() { Serial.print("."); }, timeout)) // prints 10 times/sec
  // {
  //   return false;
  // }
  NTP.waitSet([]() { Serial.print("."); }, timeout); // prints 10 times/sec
  Serial.println();

  return printLocalTime(timeInfo);
} // waitForNTPSync

/* Perform an HTTP GET request to OpenWeatherMap's "One Call" API
 * If data is received, it will be parsed and stored in the global variable
 * owm_onecall.
 *
 * Returns the HTTP Status Code.
 */
#ifdef USE_HTTP
  int getOWMonecall(WiFiClient &client, owm_resp_onecall_t &r)
#else
  int getOWMonecall(WiFiClientSecure &client, owm_resp_onecall_t &r)
#endif
{
  int attempts = 0;
  bool rxSuccess = false;
  DeserializationError jsonErr = {};
  String uri = "/data/" + OWM_ONECALL_VERSION
               + "/onecall?lat=" + LAT + "&lon=" + LON + "&lang=" + OWM_LANG
               + "&units=standard&exclude=minutely";
#if !DISPLAY_ALERTS
  // exclude alerts
  uri += ",alerts";
#endif

  // This string is printed to terminal to help with debugging. The API key is
  // censored to reduce the risk of users exposing their key.
  String sanitizedUri = OWM_ENDPOINT + uri + "&appid={API key}";

  uri += "&appid=" + OWM_APIKEY;

  Serial.print(TXT_ATTEMPTING_HTTP_REQ);
  Serial.println(": " + sanitizedUri);
  int httpResponse = 0;
  while (!rxSuccess && attempts < 3)
  {
    wl_status_t connection_status = (wl_status_t)WiFi.status();
    if (connection_status != WL_CONNECTED)
    {
      // -512 offset distinguishes these errors from httpClient errors
      return -512 - static_cast<int>(connection_status);
    }

    HTTPClient http;
    http.setTimeout(HTTP_CLIENT_TCP_TIMEOUT); // default 5000ms
    http.begin(client, OWM_ENDPOINT, OWM_PORT, uri);
    httpResponse = http.GET();
    if (httpResponse == HTTP_CODE_OK)
    {
      jsonErr = deserializeOneCall(http.getStream(), r);
      if (jsonErr)
      {
        // -256 offset distinguishes these errors from httpClient errors
        httpResponse = -256 - static_cast<int>(jsonErr.code());
      }
      rxSuccess = !jsonErr;
    }
    client.stop();
    http.end();
    Serial.println("  " + String(httpResponse, DEC) + " "
                   + getHttpResponsePhrase(httpResponse));
    ++attempts;
  }

  return httpResponse;
} // getOWMonecall

/* Perform an HTTP GET request to OpenWeatherMap's "Air Pollution" API
 * If data is received, it will be parsed and stored in the global variable
 * owm_air_pollution.
 *
 * Returns the HTTP Status Code.
 */
#ifdef USE_HTTP
  int getOWMairpollution(WiFiClient &client, owm_resp_air_pollution_t &r)
#else
  int getOWMairpollution(WiFiClientSecure &client, owm_resp_air_pollution_t &r)
#endif
{
  int attempts = 0;
  bool rxSuccess = false;
  DeserializationError jsonErr = {};

  // set start and end to appropriate values so that the last 24 hours of air
  // pollution history is returned. Unix, UTC.
  time_t now;
  int64_t end = time(&now);
  // minus 1 is important here, otherwise we could get an extra hour of history
  int64_t start = end - ((3600 * OWM_NUM_AIR_POLLUTION) - 1);
  char endStr[22];
  char startStr[22];
  sprintf(endStr, "%lld", end);
  sprintf(startStr, "%lld", start);
  String uri = "/data/2.5/air_pollution/history?lat=" + LAT + "&lon=" + LON
               + "&start=" + startStr + "&end=" + endStr
               + "&appid=" + OWM_APIKEY;
  // This string is printed to terminal to help with debugging. The API key is
  // censored to reduce the risk of users exposing their key.
  String sanitizedUri = OWM_ENDPOINT +
               "/data/2.5/air_pollution/history?lat=" + LAT + "&lon=" + LON
               + "&start=" + startStr + "&end=" + endStr
               + "&appid={API key}";

  Serial.print(TXT_ATTEMPTING_HTTP_REQ);
  Serial.println(": " + sanitizedUri);
  int httpResponse = 0;
  while (!rxSuccess && attempts < 3)
  {
    wl_status_t connection_status = (wl_status_t)WiFi.status();
    if (connection_status != WL_CONNECTED)
    {
      // -512 offset distinguishes these errors from httpClient errors
      return -512 - static_cast<int>(connection_status);
    }

    HTTPClient http;
    http.setTimeout(HTTP_CLIENT_TCP_TIMEOUT); // default 5000ms
    http.begin(client, OWM_ENDPOINT, OWM_PORT, uri);
    httpResponse = http.GET();
    if (httpResponse == HTTP_CODE_OK)
    {
      jsonErr = deserializeAirQuality(http.getStream(), r);
      if (jsonErr)
      {
        // -256 offset to distinguishes these errors from httpClient errors
        httpResponse = -256 - static_cast<int>(jsonErr.code());
      }
      rxSuccess = !jsonErr;
    }
    client.stop();
    http.end();
    Serial.println("  " + String(httpResponse, DEC) + " "
                   + getHttpResponsePhrase(httpResponse));
    ++attempts;
  }

  return httpResponse;
} // getOWMairpollution
