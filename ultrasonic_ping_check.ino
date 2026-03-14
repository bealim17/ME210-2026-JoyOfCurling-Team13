/*
  Ping))) Sensor

  This sketch reads a PING))) ultrasonic rangefinder and returns the distance
  to the closest object in range. To do this, it sends a pulse to the sensor to
  initiate a reading, then listens for a pulse to return. The length of the
  returning pulse is proportional to the distance of the object from the sensor.

  The circuit:
	- +V connection of the PING))) attached to +5V
	- GND connection of the PING))) attached to ground
	- SIG connection of the PING))) attached to digital pin 7

  created 3 Nov 2008
  by David A. Mellis
  modified 30 Aug 2011
  by Tom Igoe

  This example code is in the public domain.

  https://docs.arduino.cc/built-in-examples/sensors/Ping/
*/

// this constant won't change. It's the pin number of the sensor's output:
const uint8_t pingPin  = 30, echoPin  = 31;
const uint8_t pingPin2 = 10, echoPin2 = 11;
const uint8_t pingPin3 = 26, echoPin3 = 24;
const uint8_t pingPin4 = 46, echoPin4 = 47;
//const int ECHO_PINS[NUM_SENSORS] = {6, 7};
//const int TRIG_PINS[NUM_SENSORS] = {8, 9};
const int MAX_DISTANCE = 1000; // cm
const int SETTLE_MS = 10; //ms

void setupUltrasonic() {
  pinMode(pingPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(pingPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(pingPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  pinMode(pingPin4, OUTPUT);
  pinMode(echoPin4, INPUT);
}

void pollUltrasonic(float &cm, float &cm2, float &cm3, float &cm4) {
  // establish variables for duration of the ping, and the distance result
  // in inches and centimeters:
  long duration, inches;
  long duration2, inches2;
  long duration3, inches3;
  long duration4, inches4;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  //pinMode(pingPin, OUTPUT);
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pingPin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH pulse
  // whose duration is the time (in microseconds) from the sending of the ping
  // to the reception of its echo off of an object.
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);

  delay(SETTLE_MS);

  pinMode(pingPin2, OUTPUT);
  digitalWrite(pingPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(pingPin2, LOW);
  pinMode(echoPin2, INPUT);
  duration2 = pulseIn(echoPin2, HIGH);

  delay(SETTLE_MS);

  pinMode(pingPin3, OUTPUT);
  digitalWrite(pingPin3, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin3, HIGH);
  delayMicroseconds(10);
  digitalWrite(pingPin3, LOW);
  pinMode(echoPin3, INPUT);
  duration3 = pulseIn(echoPin3, HIGH);

  delay(SETTLE_MS);

  pinMode(pingPin4, OUTPUT);
  digitalWrite(pingPin4, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin4, HIGH);
  delayMicroseconds(10);
  digitalWrite(pingPin4, LOW);
  pinMode(echoPin4, INPUT);
  duration4 = pulseIn(echoPin4, HIGH);


  // convert the time into a distance
  cm = microsecondsToCentimeters(duration);
  cm2 = microsecondsToCentimeters(duration2);
  cm3 = microsecondsToCentimeters(duration3);
  cm4 = microsecondsToCentimeters(duration4);

  // // Check for invalid readings
  // if (cm == 0 || cm > MAX_DISTANCE)
  // {
  //   Serial.println("TOO NEAR");
  // }
  // else {
  //   Serial.print(cm);
  //   Serial.print("cm / ");
  //   Serial.print(cm2);
  //   Serial.print("cm / ");
  //   Serial.print(cm3);
  //   Serial.println();
  // }
}

long microsecondsToInches(long microseconds) {
  // According to Parallax's datasheet for the PING))), there are 73.746
  // microseconds per inch (i.e. sound travels at 1130 feet per second).
  // This gives the distance travelled by the ping, outbound and return,
  // so we divide by 2 to get the distance of the obstacle.
  // See: https://www.parallax.com/package/ping-ultrasonic-distance-sensor-downloads/
  return microseconds / 74 / 2;
}

float microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the object we
  // take half of the distance travelled.
  return (float)microseconds / 29 / 2;
}
