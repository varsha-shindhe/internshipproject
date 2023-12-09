#include <SoftwareSerial.h>

SoftwareSerial sim900a(14, 12); // RX, TX for SIM900a module
SoftwareSerial neo6m(4, 5);   // RX, TX for NEO-6M GPS module

const int buzzerPin = 16;      // Buzzer pin
const int buttonPin = 15;
const int buttonPin2 = 13;// Push button pin

float geofenceLat = 12.971598; // Latitude of geofence
float geofenceLon = 77.594562; // Longitude of geofence
float geofenceRadius = 0.0004;    // Geofence radius in kilometers

bool geofenceAlertSent = false;

float latitude;
float longitude;

float lat;
float lon;

void setup() {

  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);
  pinMode(buttonPin, INPUT);
  pinMode(buttonPin2, INPUT);
  
  Serial.begin(9600);
  neo6m.begin(9600);
  sim900a.begin(9600);
}

void loop() {
  if (digitalRead(buttonPin) == HIGH) {
    
    geofenceAlertSent = false; // Reset geofence alert flag when the button is pressed
  }

  if (digitalRead(buttonPin2) == HIGH)
  {
    geofenceLat = latitude;
    geofenceLon = longitude; 
  }

  if (neo6m.available() > 0) {
    if (neo6m.find("$GPGGA")) {
      // Parse GPS data
      latitude = getGPSData(2);
      longitude = getGPSData(4);

      // Check if the vehicle has crossed the geofence
      if (isInsideGeofence(latitude, longitude) && !geofenceAlertSent) {

        // Send SMS with location details
        sendSMSWithLocation(latitude, longitude);

        // Trigger the buzzer
        triggerBuzzer();
        

        geofenceAlertSent = true; // Set the flag to avoid continuous alerts
        delay(60000);             // Delay for 1 minute to avoid spamming SMS
      }
    }
  }
}

float getGPSData(int index) {
  neo6m.find(",");
  for (int i = 0; i < index - 1; i++) {
    neo6m.find(",");
  }
  return neo6m.parseFloat();
}

bool isInsideGeofence(float lat, float lon) {
  // Simple distance-based geofence check
  float distance = calculateDistance(lat, lon, geofenceLat, geofenceLon);
  return distance <= geofenceRadius;
}

float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
  // Haversine formula to calculate distance between two points on the Earth
  float dlat = radians(lat2 - lat1);
  float dlon = radians(lon2 - lon1);
  float a = sin(dlat / 2) * sin(dlat / 2) + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon / 2) * sin(dlon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  float distance = 6371.0 * c; // Earth radius in kilometers
  return distance;
}

void sendSMSWithLocation(float lat, float lon) {
  // Format the SMS with Google Maps link
  String message = "Vehicle has crossed the geofence! Location: https://maps.google.com/?q=" + String(lat, 6) + "," + String(lon, 6);
 
}

void triggerBuzzer() {
  digitalWrite(buzzerPin, HIGH);
  delay(1000);
  digitalWrite(buzzerPin,LOW);
}
