#include <Adafruit_ADS1X15.h>  // Library for ADS1115 ADC module
#include <Wire.h>              // I2C communication library
#include <Adafruit_MPU6050.h>  // Library for MPU6050 sensor
#include <WiFi.h>              // WiFi library for ESP32
#include <FirebaseESP32.h>     // Firebase library for ESP32
#include "time.h"              // Time library for NTP
#include <NTPClient.h>         // NTP Client for time synchronization
#include <WiFiUdp.h>           // Library for UDP protocol
#include <addons/TokenHelper.h> // Firebase token helper
#include <addons/RTDBHelper.h>  // Firebase real-time database helper

// WiFi and Firebase configuration
#define WIFI_SSID "OPPO F19 Pro"
#define WIFI_PASSWORD "1234567891011"
#define API_KEY "AIzaSyB3wf81dNB1g8yaZmXcmNjoqMyEwoHXH1c"
#define DATABASE_URL "https://poochpaw-8b913-default-rtdb.firebaseio.com/"
#define USER_EMAIL "edirisinghapkss@gmail.com"
#define USER_PASSWORD "Dj&shaapp98"

#define LED_PIN 2  // Define pin for onboard LED

FirebaseData fbdo;         // Firebase data object
FirebaseAuth auth;         // Firebase authentication object
FirebaseConfig config;     // Firebase configuration object
Adafruit_ADS1115 ads;      // ADS1115 ADC object
Adafruit_MPU6050 mpu;      // MPU6050 sensor object
WiFiUDP ntpUDP;            // UDP instance for NTP communication
NTPClient timeClient(ntpUDP, "pool.ntp.org");  // NTP client object for time synchronization

// Sensor offset values for calibration
float offset_ax = 0.62;
float offset_ay = -0.12;
float offset_az = 2.68;
float offset_gx = 0.18;
float offset_gy = 0.04;
float offset_gz = 0.02;
unsigned long motionTime;  // Variable to store motion timestamp

// Variables for heartbeat measurement
unsigned long pre_time;  // Previous time in milliseconds
unsigned long now_time;  // Current time in milliseconds
unsigned int pre_val;    // Previous ADC value
unsigned int now_val;    // Current ADC value
unsigned long bpm;       // Beats per minute (BPM)
bool inc;                // Increment flag
bool dec;                // Decrement flag
bool en_cal;             // Enable calculation flag

// Task handles for multitasking
TaskHandle_t Task1;
TaskHandle_t Task2;

void setup() {
  pinMode(LED_PIN, OUTPUT);  // Set LED pin as output
  Serial.begin(115200);      // Initialize serial communication
  delay(1000);               // Wait for setup

  ads.begin();  // Initialize ADS1115 ADC

  // Initialize MPU6050 sensor
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);  // Retry delay if sensor not found
    }
  } else {
    Serial.println("Found the MPU6050 chip");
  }

  // Set sensor ranges and bandwidth
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);

  // Connect to Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);  // Blink LED while connecting
    digitalWrite(LED_PIN, LOW);
    delay(200);
    Serial.print(".");
  }
  digitalWrite(LED_PIN, LOW);
  Serial.println("");
  Serial.println("Connected to Wi-Fi!");

  // Configure Firebase connection
  config.api_key = API_KEY;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  config.database_url = DATABASE_URL;
  config.token_status_callback = tokenStatusCallback;  // Firebase token callback
  Firebase.reconnectNetwork(true);  // Reconnect Firebase on network failure
  fbdo.setBSSLBufferSize(4096 /* Rx buffer size */, 1024 /* Tx buffer size */);  // Set buffer size for Firebase
  Firebase.begin(&config, &auth);  // Begin Firebase connection
  Firebase.setDoubleDigits(5);  // Set precision for double values
  Serial.print("Connecting to Firebase");
  while (!Firebase.ready()) {
    delay(500);  // Wait until Firebase is ready
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("Connected to Firebase!");

  // Initialize NTP client with time offset for Sri Lanka (UTC+5:30)
  timeClient.begin();
  timeClient.setTimeOffset(19800);  // Set offset to UTC+5:30
  timeClient.forceUpdate();  // Force initial update
  timeClient.update();       // Update time

  // Create Task1 for BPM measurement
  xTaskCreatePinnedToCore(
    Task1code,
    "Task1",
    8192,  // Stack size for Task1
    NULL,
    1,     // Priority of Task1
    &Task1,
    0);    // Pin Task1 to core 0

  // Create Task2 for Wi-Fi and Firebase communication
  xTaskCreatePinnedToCore(
    Task2code,
    "Task2",
    8192,  // Stack size for Task2
    NULL,
    1,     // Priority of Task2
    &Task2,
    1);    // Pin Task2 to core 1
}

// Task1: Measuring BPM from ADS1115 ADC
void Task1code(void* pvParameters) {
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());  // Print core ID for Task1

  for (;;) {
    now_val = ads.readADC_SingleEnded(0);  // Read value from ADS1115 ADC on channel 0

    // Determine if the signal is increasing or decreasing
    if (pre_val < now_val) {
      inc = true;
      dec = false;
    } else if (pre_val > now_val) {
      inc = false;
      dec = true;
    }

    // Calculate BPM if the signal is increasing and exceeds threshold
    if (inc == true && now_val > 16000 && en_cal == false) {
      now_time = millis();  // Get current time in milliseconds
      bpm = 60000 / (now_time - pre_time);  // Calculate BPM
      en_cal = true;  // Enable calculation
    } else if (dec == true && now_val < 16000 && en_cal == true) {
      en_cal = false;  // Reset calculation flag when signal decreases
    }

    pre_val = now_val;  // Update previous value
    pre_time = now_time;  // Update previous time
    delay(15);  // Small delay between readings
  }
}

// Task2: Handle Wi-Fi, Firebase data, and MPU6050 sensor data
void Task2code(void* pvParameters) {
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());  // Print core ID for Task2

  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      // Blink LED if Wi-Fi is disconnected
      digitalWrite(LED_PIN, HIGH);
      delay(200);
      digitalWrite(LED_PIN, LOW);
      delay(200);
    } else {
      digitalWrite(LED_PIN, LOW);  // Turn off LED if Wi-Fi is connected

      FirebaseJsonArray array2;  // Create a JSON array for storing sensor data
      for (int x = 0; x < 10; x++) {
        float sum_ax = 0, sum_ay = 0, sum_az = 0;  // Sum of accelerometer data
        float sum_gx = 0, sum_gy = 0, sum_gz = 0;  // Sum of gyroscope data

        motionTime = timeClient.getEpochTime();  // Get the current epoch time
        for (int i = 0; i < 100; i++) {
          sensors_event_t a, g, temp;  // Create sensor event objects
          mpu.getEvent(&a, &g, &temp);  // Get sensor events
          sum_ax += (a.acceleration.x - offset_ax);  // Apply offset to accelerometer x-axis
          sum_ay += (a.acceleration.y - offset_ay);  // Apply offset to accelerometer y-axis
          sum_az += (a.acceleration.z - offset_az);  // Apply offset to accelerometer z-axis
          sum_gx += (g.gyro.x - offset_gx);          // Apply offset to gyroscope x-axis
          sum_gy += (g.gyro.y - offset_gy);          // Apply offset to gyroscope y-axis
          sum_gz += (g.gyro.z - offset_gz);          // Apply offset to gyroscope z-axis
          delay(1);  // Small delay for sensor stability
        }

        // Prepare Firebase data in JSON format
        FirebaseJsonArray data;
        data.add(motionTime);      // Add motion time
        data.add(bpm);             // Add BPM
        data.add(sum_ax / 100);    // Average accelerometer x-axis data
        data.add(sum_ay / 100);    // Average accelerometer y-axis data
        data.add(sum_az / 100);    // Average accelerometer z-axis data
        data.add(sum_gx / 100);    // Average gyroscope x-axis data
        data.add(sum_gy / 100);    // Average gyroscope y-axis data
        data.add(sum_gz / 100);    // Average gyroscope z-axis data

        array2.add(data);  // Add sensor data to the array
        digitalWrite(LED_PIN, HIGH);  // Blink LED to indicate data collection
        delay(50);
        digitalWrite(LED_PIN, LOW);
      }

      // Update the NTP client time
      timeClient.update();
      time_t rawtime = motionTime;  // Store motion time as raw time
      struct tm* timeinfo;  // Struct for storing time information
      char buffer[80];  // Buffer for formatted time

      // Convert epoch time to local time (Sri Lankan time)
      timeinfo = localtime(&rawtime);
      strftime(buffer, 80, "%Y-%m-%d %H-%M-%S", timeinfo);  // Format time as string
      String path = String("1718248723/collar_data/") + buffer;  // Create path for Firebase data

      // Send the data array to Firebase
      if (Firebase.setArray(fbdo, path.c_str(), array2)) {
        Serial.print("Data sent to path: ");
        Serial.println(path);  // Print path where data is sent
        digitalWrite(LED_PIN, HIGH);  // Blink LED to indicate data sent
        delay(1000);
        digitalWrite(LED_PIN, LOW);
      } else {
        // Handle error if data is not sent
        Serial.print("Error sending data: ");
        Serial.println(fbdo.errorReason());  // Print error reason
        for (int i = 0; i < 10; i++) {
          digitalWrite(LED_PIN, HIGH);
          delay(50);  // Blink LED quickly to indicate an error
          digitalWrite(LED_PIN, LOW);
          delay(50);
        }
      }

      // Send battery level data to Firebase
      Firebase.setString(fbdo, "/1718248723/collar_data/battery_level", "56%");
      Serial.println("(core 1) Sent data to Firebase");  // Indicate data sent successfully
    }
  }
}

void loop() {
  // Empty loop as tasks run in parallel
}
