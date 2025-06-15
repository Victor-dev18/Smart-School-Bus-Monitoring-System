#include <WiFi.h>
#include <TinyGPSPlus.h>
#include <Firebase_ESP_Client.h>
#include <Wire.h>
#include <MPU6050.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

// WiFi credentials
#define WIFI_SSID "xxxxxx"        //Your Wi-Fi SSID
#define WIFI_PASSWORD "xxxxxx"    //Your Wi-Fi Password

// Firebase project details
#define DATABASE_URL "https://xxxxxxxxxxx.firebaseio.com/"     //FireBase URL
#define DATABASE_SECRET "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"     //FireBase Secret Code

// Firebase objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// GPS setup
TinyGPSPlus gps;
HardwareSerial gpsSerial(2); // UART2
#define GPS_RX 16
#define GPS_TX 17

// MPU6050 setup
MPU6050 mpu;
float previousLat = 0, previousLng = 0;
unsigned long previousTime = 0;
float currentSpeed = 0;

// Status tracking
unsigned long lastHeartbeat = 0;
const unsigned long HEARTBEAT_INTERVAL = 5000; // 5 seconds

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  
  // Initialize I2C for MPU6050
  Wire.begin();
  
  // Initialize MPU6050
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
  }

  // Wi-Fi setup
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nConnected!");

  // Firebase setup
  config.database_url = DATABASE_URL;
  config.signer.tokens.legacy_token = DATABASE_SECRET;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  
  // Set initial online status
  if (Firebase.ready()) {
    Firebase.RTDB.setBool(&fbdo, "/bus/status/online", true);
    Firebase.RTDB.setString(&fbdo, "/bus/status/lastSeen", "connected");
  }
}

float calculateSpeedFromGPS(float lat1, float lng1, float lat2, float lng2, unsigned long timeDiff) {
  // Haversine formula to calculate distance between two GPS points
  float R = 6371000; // Earth's radius in meters
  float dLat = (lat2 - lat1) * PI / 180.0;
  float dLng = (lng2 - lng1) * PI / 180.0;
  float a = sin(dLat/2) * sin(dLat/2) + cos(lat1 * PI / 180.0) * cos(lat2 * PI / 180.0) * sin(dLng/2) * sin(dLng/2);
  float c = 2 * atan2(sqrt(a), sqrt(1-a));
  float distance = R * c; // Distance in meters
  
  float speed = (distance / (timeDiff / 1000.0)) * 3.6; // Convert m/s to km/h
  return speed;
}

float calculateSpeedFromAccelerometer() {
  // Read accelerometer data
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  
  // Convert to m/s²
  float accelX = ax / 16384.0 * 9.81;
  float accelY = ay / 16384.0 * 9.81;
  float accelZ = az / 16384.0 * 9.81;
  
  // Calculate magnitude of acceleration (removing gravity)
  float totalAccel = sqrt(accelX*accelX + accelY*accelY + (accelZ-9.81)*(accelZ-9.81));
  
  // Simple speed estimation (this is approximate)
  static float estimatedSpeed = 0;
  static unsigned long lastAccelTime = 0;
  
  if (lastAccelTime > 0) {
    float deltaTime = (millis() - lastAccelTime) / 1000.0;
    estimatedSpeed += totalAccel * deltaTime * 3.6; // Convert to km/h
    
    // Apply some damping to prevent unrealistic speeds
    estimatedSpeed *= 0.95;
    if (estimatedSpeed < 0) estimatedSpeed = 0;
    if (estimatedSpeed > 120) estimatedSpeed = 120; // Max reasonable bus speed
  }
  
  lastAccelTime = millis();
  return estimatedSpeed;
}

void sendHeartbeat() {
  if (Firebase.ready() && (millis() - lastHeartbeat) > HEARTBEAT_INTERVAL) {
    Firebase.RTDB.setBool(&fbdo, "/bus/status/online", true);
    Firebase.RTDB.setString(&fbdo, "/bus/status/lastSeen", "active");
    lastHeartbeat = millis();
  }
}

void loop() {
  // Send heartbeat to maintain online status
  sendHeartbeat();
  
  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected, attempting to reconnect...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    return;
  }
  
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isUpdated()) {
    float lat = gps.location.lat();
    float lng = gps.location.lng();
    unsigned long currentTime = millis();

    Serial.print("Latitude: ");
    Serial.println(lat, 6);
    Serial.print("Longitude: ");
    Serial.println(lng, 6);

    // Calculate speed from GPS if we have previous coordinates
    float gpsSpeed = 0;
    if (previousLat != 0 && previousLng != 0 && previousTime != 0) {
      unsigned long timeDiff = currentTime - previousTime;
      if (timeDiff > 1000) { // Only calculate if more than 1 second has passed
        gpsSpeed = calculateSpeedFromGPS(previousLat, previousLng, lat, lng, timeDiff);
      }
    }
    
    // Get speed from accelerometer
    float accelSpeed = calculateSpeedFromAccelerometer();
    
    // Use GPS speed if available and reasonable, otherwise use accelerometer
    if (gpsSpeed > 0 && gpsSpeed < 120) {
      currentSpeed = gpsSpeed;
    } else if (gps.speed.isValid() && gps.speed.kmph() < 120) {
      currentSpeed = gps.speed.kmph(); // Use GPS module's built-in speed calculation
    } else {
      currentSpeed = accelSpeed;
    }
    
    Serial.print("Speed: ");
    Serial.print(currentSpeed);
    Serial.println(" km/h");

    if (Firebase.ready()) {
      // Send location data
      Firebase.RTDB.setFloat(&fbdo, "/bus/location/latitude", lat);
      Firebase.RTDB.setFloat(&fbdo, "/bus/location/longitude", lng);
      
      // Send speed data
      Firebase.RTDB.setFloat(&fbdo, "/bus/speed", currentSpeed);
      
      // Send timestamp
      Firebase.RTDB.setString(&fbdo, "/bus/lastUpdate", "updated");
      
      // Update online status
      Firebase.RTDB.setBool(&fbdo, "/bus/status/online", true);
      Firebase.RTDB.setString(&fbdo, "/bus/status/lastSeen", "active");
    }

    // Store current values for next calculation
    previousLat = lat;
    previousLng = lng;
    previousTime = currentTime;

    delay(2000); // Reduced delay for more frequent updates
  }
  
  delay(100); // Small delay to prevent overwhelming the loop
}