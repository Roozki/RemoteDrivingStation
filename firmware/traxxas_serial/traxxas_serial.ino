#include <Arduino.h>

#define STEERING_PIN A7
#define THROTTLE_PIN A6

const int leftSignalPin = 13; 
const int rightSignalPin = 12;
const int headLightPin = 14;
const int brakeLightPin = 27;
const int rSonarPin = 26;
const int lSonarPin = 25;
const int steeringPot = 34;
const int steeringF = 33;
const int steeringR = 32;
const int drivingF = 19;
const int drivingR = 21;

const int signalFlashDelay = 250; // Delay between flashes (ms)
unsigned long previousMillis1 = 0;
unsigned long previousMillis2 = 0;

int leftSignalOn = 0;
int rightSignalOn = 0;
int headLightOn = 0;
int brakeLightOn = 0;
int hazardSignalOn = 0;
float steeringAngle = 0;
float accel = 0;

int rightMotion = LOW; 
int leftMotion = LOW;
int rval = 0; 
int lval = 0;

int rightSensor = 0;
int leftSensor = 0;

//double Kp=2, Ki=5, Kd=1;
//double target, input, output;
//PID myPID(&input, &output, &target, Kp, Ki, Kd, DIRECT);
//const int potHigh = 1023;

void setup() {
  Serial.begin(9600);
  pinMode(leftSignalPin, OUTPUT);
  pinMode(rightSignalPin, OUTPUT);
  pinMode(headLightPin, OUTPUT);
  pinMode(brakeLightPin, OUTPUT);

  pinMode(rSonarPin, INPUT);
  pinMode(lSonarPin, INPUT);
  pinMode(steeringPot, INPUT);

  pinMode(steeringR, OUTPUT);
  pinMode(steeringF, OUTPUT);
  pinMode(drivingR, OUTPUT);
  pinMode(drivingF, OUTPUT);

  //myPID.SetMode(AUTOMATIC);
}
// Custom mapping function
float customMap(float value, float fromLow, float fromHigh, float toLow, float toHigh) {
    // Calculate the ratio of the value relative to the input range
    float ratio = (value - fromLow) / (fromHigh - fromLow);
    
    // Map the ratio to the output range
    float mappedValue = ratio * (toHigh - toLow) + toLow;
    
    // Ensure the mapped value is within the output range
    if (mappedValue < toLow) {
        return toLow;
    }
    if (mappedValue > toHigh) {
        return toHigh;
    }
    
    return mappedValue;
}

void leftBlinkerController(bool state) {
  if (state) {
    if (millis() - previousMillis1 >= signalFlashDelay) {
    // If it's time, toggle the hazard lights
      if (leftSignalOn) {
        digitalWrite(leftSignalPin, HIGH); // Turn on hazard lights
      } else {
        digitalWrite(leftSignalPin, LOW); // Turn off hazard lights
      }
      leftSignalOn = !leftSignalOn;
      previousMillis1 = millis(); // Save the last time the hazard lights were toggled
    }
  } else {
    digitalWrite(leftSignalPin, LOW);
  }
}

void rightBlinkerController(bool state) {
  if (state) {
    if (millis() - previousMillis2 >= signalFlashDelay) {
    // If it's time, toggle the hazard lights
      if (rightSignalOn) {
        digitalWrite(rightSignalPin, HIGH); // Turn on hazard lights
      } else {
        digitalWrite(rightSignalPin, LOW); // Turn off hazard lights
      }
      rightSignalOn = !rightSignalOn;
      previousMillis2 = millis(); // Save the last time the hazard lights were toggled
    }
  } else {
    digitalWrite(rightSignalPin, LOW);
  }
}

void brakeLightController(bool state) {
  if (state) {
    digitalWrite(brakeLightPin, HIGH);
  } else {
    digitalWrite(brakeLightPin, LOW);
  }
}

void headLightController(bool state) {
  if (state) {
    digitalWrite(headLightPin, HIGH);
  } else {
    digitalWrite(headLightPin, LOW);
  }
}

void hazardLightController(bool state) {
  unsigned long currentMillis = millis();
  if (state) {
   if (currentMillis - previousMillis1 >= signalFlashDelay) {
    // If it's time, toggle the hazard lights
    if (leftSignalOn) {
      digitalWrite(leftSignalPin, HIGH); // Turn on hazard lights
      digitalWrite(rightSignalPin, HIGH);
    } else {
      digitalWrite(leftSignalPin, LOW); // Turn off hazard lights
      digitalWrite(rightSignalPin, LOW);
    }
    leftSignalOn != leftSignalOn;
    previousMillis1 = currentMillis; // Save the last time the hazard lights were toggled
    }
  }
}
void steeringMotorControl(float steeringmotor) {
  int angle_pwm = map(steeringmotor, -1, 1, 0, 255);
//  if (steeringmotor> 0 ) {
//    float forwardSpeed = customMap(steeringmotor, 0, 1, 0, 255);
//    Serial.println(forwardSpeed);
//    analogWrite(steeringF, forwardSpeed);
//  } else if (steeringmotor < 0) {
//    float flipSpeed = -steeringmotor;
//    int reverseSpeed = customMap(steeringmotor, 0, 1, 0, 255);
//    analogWrite(steeringR, reverseSpeed);
//  } else { //speed is 0
//    analogWrite(steeringF, 0);
//    analogWrite(steeringR, 0);
//  }
}

//void steeringControl(float steeringAngle) {
//  float remapSteeringAngle = customMap(steeringAngle, -1, 1, 0, potHigh);
//  input = analogRead(steeringPot);
//  target = remapSteeringAngle;
//  myPID.Compute();
//  steeringMotorControl(output);
//}

void motorControl(float speed) {
  speed = 0;
  if (speed > 0 ) {
    Serial.println(speed);
    float forwardSpeed = customMap(speed, 0, 1, 127, 255);
    //Serial.println(forwardSpeed);
    analogWrite(THROTTLE_PIN, forwardSpeed);
  } else if (speed < 0) {
    float flipSpeed = -speed;
    int reverseSpeed = customMap(flipSpeed, 0, 1, 0, 127);
    analogWrite(THROTTLE_PIN, reverseSpeed);
  } else { //speed is 0
    analogWrite(THROTTLE_PIN, 127);
  }
}

void rightSonar() {
  rval = digitalRead(rSonarPin);   // read sensor value
  if (rval == HIGH) {           // check if the sensor is HIGH  // turn LED ON
    if (rightMotion == LOW) {
      //Serial.println("Motion detected!"); 
      rightMotion = HIGH;       // update variable state to HIGH
    }
  } 
  else {
      if (rightMotion == HIGH){
        //Serial.println("Motion stopped!");
        rightMotion = LOW;    // update variable state to LOW
    }
  }
}

void leftSonar() {
  lval = digitalRead(rSonarPin);   // read sensor value
  if (lval == HIGH) {           // check if the sensor is HIGH  // turn LED ON
    if (leftMotion == LOW) {
      //Serial.println("Motion detected!"); 
      leftMotion = HIGH;
    }
  } 
  else {
      if (leftMotion == HIGH){
        //Serial.println("Motion stopped!");
        leftMotion = LOW;    // update variable state to LOW
      }
  }
}

void masterControl(float MCSteeringAngle, float MCAccel, int MCLeftSignalOn, int MCRightSignalOn, int MCHazardLightOn, int MCHeadLightOn, int MCBrakeLightOn) {
  steeringAngle = MCSteeringAngle;
  accel = MCAccel;
  steeringMotorControl(steeringAngle);
  //steeringControl(steeringAngle);
  motorControl(accel);
  //leftBlinkerController(MCLeftSignalOn);
  //rightBlinkerController(MCRightSignalOn);
  //hazardLightController(MCHazardLightOn);
  //headLightController(MCHeadLightOn);
  //brakeLightController(MCBrakeLightOn);
}

void parseSerialMessage() {
  if (Serial.available() > 0) {
    String message = Serial.readStringUntil('\n'); // Read the incoming message until newline character

    // Check if the message starts with "$C("
    if (message.startsWith("$C(") && message.endsWith(")")) {
      // Remove the "$C(" and ")" parts from the message
      message.remove(0, 3);
      message.remove(message.length() - 1);
      // Split the message by commas
      String tokens[7]; // Array to store the parsed values
      int tokenIndex = 0;
      int startIndex = 0;
      for (int i = 0; i < message.length(); i++) {
        if (message.charAt(i) == ',') {
          tokens[tokenIndex++] = message.substring(startIndex, i);
          startIndex = i + 1;
        }
      }
      // Extract the last token (no comma after it)
      tokens[tokenIndex++] = message.substring(startIndex);
      masterControl(tokens[0].toFloat(), tokens[1].toFloat(), tokens[2].toInt(), tokens[3].toInt(), tokens[4].toInt(), tokens[5].toInt(), tokens[6].toInt());
        /**Serial.print("Steering: ");
        Serial.println(steeringAngle);
        Serial.print("Motor: ");
        Serial.println(accel);
        Serial.print("Left Blinker: ");
        Serial.println(leftSignalOn);
        Serial.print("Right Blinker: ");
        Serial.println(rightSignalOn);
        Serial.print("Hazard Lights: ");
        Serial.println(hazardSignalOn);
        Serial.print("Headlights: ");
        Serial.println(headLightOn);
        Serial.print("Brake Lights: ");
        Serial.println(brakeLightOn);**/
    }
  }
}

void encodeSerialMessage() {
  rightSonar();
  leftSonar();
  String command = "$R(" + String(rightSensor) + "," + String(leftSensor) + ")\n";
  Serial.println(command);
}

void loop() {
  parseSerialMessage();
  //encodeSerialMessage();
}
