
/**
 * Created by K. Suwatchai (Mobizt)
 *
 * Email: k_suwatchai@hotmail.com
 *
 * Github: https://github.com/mobizt/Firebase-ESP-Client
 *
 * Copyright (c) 2023 mobizt
 *
 */

/** This example will show how to authenticate using
 * the legacy token or database secret with the new APIs (using config and auth data).
 */
#include <Arduino.h>
#include <SoftwareSerial.h>
//Pinos de comunicacao serial com a ST Núcleo
#define Pin_ST_NUCLEO_RX    5  //Pino D1 da placa Node MCU
#define Pin_ST_NUCLEO_TX    4  //Pino D2 da placa Node MCU
SoftwareSerial SSerial(Pin_ST_NUCLEO_RX, Pin_ST_NUCLEO_TX);

#if defined(ESP32) || defined(ARDUINO_RASPBERRY_PI_PICO_W)
#include <WiFi.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#endif

#include <Firebase_ESP_Client.h>

// Provide the RTDB payload printing info and other helper functions.
#include <addons/RTDBHelper.h>

/* 1. Define the WiFi credentials */
#define WIFI_SSID "iPhone de Pedro (2)"
#define WIFI_PASSWORD "aloaloalo"

/* 2. If work with RTDB, define the RTDB URL and database secret */
#define DATABASE_URL "https://dogcare-7b294-default-rtdb.firebaseio.com/" //<databaseName>.firebaseio.com or <databaseName>.<region>.firebasedatabase.app
#define DATABASE_SECRET "aI0aibwbhkdba0szELstqtxDu4Wzzx6TD2zO6L4L"

/* 3. Define the Firebase Data object */
FirebaseData fbdo;

/* 4, Define the FirebaseAuth data for authentication data */
FirebaseAuth auth;

/* Define the FirebaseConfig data for config data */
FirebaseConfig config;

unsigned long dataMillis = 0;
int count = 0;

#if defined(ARDUINO_RASPBERRY_PI_PICO_W)
WiFiMulti multi;
#endif

void setup()
{
    // Open serial communications and wait for port to open:
    Serial.begin(115200);

    SSerial.begin(115200);

#if defined(ARDUINO_RASPBERRY_PI_PICO_W)
    multi.addAP(WIFI_SSID, WIFI_PASSWORD);
    multi.run();
#else
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
#endif

    Serial.print("Connecting to Wi-Fi");
    unsigned long ms = millis();
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(300);
#if defined(ARDUINO_RASPBERRY_PI_PICO_W)
        if (millis() - ms > 10000)
            break;
#endif
    }
    Serial.println();
    Serial.print("Connected with IP: ");
    Serial.println(WiFi.localIP());
    Serial.println();

    Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);

    /* Assign the certificate file (optional) */
    // config.cert.file = "/cert.cer";
    // config.cert.file_storage = StorageType::FLASH;

    /* Assign the database URL and database secret(required) */
    config.database_url = DATABASE_URL;
    config.signer.tokens.legacy_token = DATABASE_SECRET;

    // The WiFi credentials are required for Pico W
    // due to it does not have reconnect feature.
#if defined(ARDUINO_RASPBERRY_PI_PICO_W)
    config.wifi.clearAP();
    config.wifi.addAP(WIFI_SSID, WIFI_PASSWORD);
#endif

    Firebase.reconnectWiFi(true);

    /* Initialize the library with the Firebase authen and config */
    Firebase.begin(&config, &auth);

    // Or use legacy authenticate method
    // Firebase.begin(DATABASE_URL, DATABASE_SECRET);
}

void loop()
{
    if (millis() - dataMillis > 3000) {
        dataMillis = millis();
        /*
        1 = Quiet Mode
        2 = 
        3 = Red and Buzzer
        */

        int iVal = 0;
        int ref = 0;

        Serial.printf("Red led:  %s\n\r", Firebase.RTDB.getInt(&fbdo, "/prod/rLed", &iVal) ? String(iVal).c_str() : fbdo.errorReason().c_str());
        Serial.printf("Yellow led: %s\n\r", Firebase.RTDB.getInt(&fbdo, "/prod/yLed", &iVal) ? String(iVal).c_str() : fbdo.errorReason().c_str());
        Serial.printf("Green led: %s\n\r", Firebase.RTDB.getInt(&fbdo, "/prod/gLed", &iVal) ? String(iVal).c_str() : fbdo.errorReason().c_str());
        Serial.printf("Buzzer: %s\n\r", Firebase.RTDB.getInt(&fbdo, "/prod/buz", &iVal) ? String(iVal).c_str() : fbdo.errorReason().c_str());
        Serial.printf("Mic: %s\n\r", Firebase.RTDB.getString(&fbdo, "/prod/mic") ? fbdo.to<const char *>() : fbdo.errorReason().c_str());

        int c = SSerial.read();

        if (c > 8 || c == 0) {
            Serial.printf("Setting mic value... %s\n\r", Firebase.RTDB.setString(&fbdo, "/prod/mic", String(c * 16)) ? "ok" : fbdo.errorReason().c_str());
        
        } else {
            if (c == 1) {
                Serial.printf("Setting G %s, ", Firebase.RTDB.setInt(&fbdo, "/prod/gLed", 1) ? "ok" : fbdo.errorReason().c_str());
                Serial.printf("Y %s, ", Firebase.RTDB.setInt(&fbdo, "/prod/yLed", 0) ? "ok" : fbdo.errorReason().c_str());
                Serial.printf("R %s, and ", Firebase.RTDB.setInt(&fbdo, "/prod/rLed", 0) ? "ok" : fbdo.errorReason().c_str());
                Serial.printf("B %s values... \n\r", Firebase.RTDB.setInt(&fbdo, "/prod/buz", 0) ? "ok" : fbdo.errorReason().c_str());

            } else if (c == 2) {
                Serial.printf("Setting G %s, ", Firebase.RTDB.setInt(&fbdo, "/prod/gLed", 0) ? "ok" : fbdo.errorReason().c_str());
                Serial.printf("Y %s,  ", Firebase.RTDB.setInt(&fbdo, "/prod/yLed", 1) ? "ok" : fbdo.errorReason().c_str());
                Serial.printf("R %s, and  ", Firebase.RTDB.setInt(&fbdo, "/prod/rLed", 0) ? "ok" : fbdo.errorReason().c_str());
                Serial.printf("B %s values...\n\r", Firebase.RTDB.setInt(&fbdo, "/prod/buz", 0) ? "ok" : fbdo.errorReason().c_str());

            } else if (c == 3) {
                Serial.printf("Setting G %s, ", Firebase.RTDB.setInt(&fbdo, "/prod/gLed", 0) ? "ok" : fbdo.errorReason().c_str());
                Serial.printf("Y %s,  ", Firebase.RTDB.setInt(&fbdo, "/prod/yLed", 0) ? "ok" : fbdo.errorReason().c_str());
                Serial.printf("R %s, and  ", Firebase.RTDB.setInt(&fbdo, "/prod/rLed", 1) ? "ok" : fbdo.errorReason().c_str());
                Serial.printf("B %s values...\n\r", Firebase.RTDB.setInt(&fbdo, "/prod/buz", 1) ? "ok" : fbdo.errorReason().c_str());

            } else if (c == 4) {
                Serial.printf("", Firebase.RTDB.getInt(&fbdo, "/prod/gLed", &iVal) ? String(iVal).c_str() : fbdo.errorReason().c_str());
                if (iVal == 0) {
                    ref = 1;
                }
                Serial.printf("Setting green led... %s\n\r", Firebase.RTDB.setInt(&fbdo, "/prod/gLed", ref) ? "ok" : fbdo.errorReason().c_str());

            } else if (c == 5) {
                Serial.printf("", Firebase.RTDB.getInt(&fbdo, "/prod/yLed", &iVal) ? String(iVal).c_str() : fbdo.errorReason().c_str());
                if (iVal == 0) {
                    ref = 1;
                }
                
                Serial.printf("Setting yellow led... %s\n\r", Firebase.RTDB.setInt(&fbdo, "/prod/yLed", ref) ? "ok" : fbdo.errorReason().c_str());

            } else if (c == 6) {
                Serial.printf("", Firebase.RTDB.getInt(&fbdo, "/prod/rLed", &iVal) ? String(iVal).c_str() : fbdo.errorReason().c_str());
                if (iVal == 0) {
                    ref = 1;
                }
                Serial.printf("Setting red led... %s\n\r", Firebase.RTDB.setInt(&fbdo, "/prod/rLed", ref) ? "ok" : fbdo.errorReason().c_str());

            } else if (c == 7) {
                Serial.printf("", Firebase.RTDB.getInt(&fbdo, "/prod/buz", &iVal) ? String(iVal).c_str() : fbdo.errorReason().c_str());
                if (iVal == 0) {
                    ref = 1;
                }
                Serial.printf("Setting buzzer... %s\n\r", Firebase.RTDB.setInt(&fbdo, "/prod/buz", ref) ? "ok" : fbdo.errorReason().c_str());

            } else if (c == 8) {
                Serial.printf("Setting mic... %s\n\r", Firebase.RTDB.setInt(&fbdo, "/prod/mic", 1) ? "ok" : fbdo.errorReason().c_str());

            }

        }
    }
}