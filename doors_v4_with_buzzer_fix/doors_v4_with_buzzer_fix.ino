#include <Servo.h>

#define TRIG_s1 3    // Left sensor
#define ECHO_s1 4
#define TRIG_s2 A1   // Right sensor
#define ECHO_s2 A2

#define TRIG_s3 A0 //right sensor inside the bin
#define ECHO_s3 A3
#define TRIG_s4 12 //left sensor inside the bin
#define ECHO_s4 13

#define doorState 2  // Controls door lock

#define ULTRASONIC_TRIG 5  // Ultrasonic sensor TRIG pin
#define ULTRASONIC_ECHO 6  // Ultrasonic sensor ECHO pin
#define BUZZER 7
#define BUZZER_DELAY 300 // to make an alarm like sound

Servo servo1;  // Left door
Servo servo2;  // Right door

unsigned long doorOpenDuration = 5000;  // Doors stay open for 5 seconds
unsigned long lastOpenTime = 0;
unsigned long objectDetectedTime = 0;
bool door1Open = false;
bool door2Open = false;
bool doorLock = true;  // Default: door is locked
bool buzzerActive = false;
unsigned long buzzerStartTime = 0;
int buzzerState = 0;

void setup() {
  Serial.begin(9600);

  // servos
  servo1.attach(9);
  servo2.attach(10);

  // Set sensor pins
  pinMode(TRIG_s1, OUTPUT); //ultrasonice for doors
  pinMode(ECHO_s1, INPUT);
  pinMode(TRIG_s2, OUTPUT);
  pinMode(ECHO_s2, INPUT);

  pinMode(TRIG_s3,OUTPUT); //ultrasonice inside bin
  pinMode(ECHO_s3,INPUT);
  pinMode(TRIG_s4,OUTPUT);
  pinMode(ECHO_s4,INPUT);

  pinMode(doorState, OUTPUT);
  pinMode(BUZZER,OUTPUT); //to controll the buzzer

  pinMode(ULTRASONIC_TRIG, OUTPUT); //ultrasonic for stoping 
  pinMode(ULTRASONIC_ECHO, INPUT);

  digitalWrite(doorState, LOW);  // to keep the doors locked
}

void loop() {
  // Check if ultrasonic sensor detects something
  float distance = getDistance(ULTRASONIC_TRIG, ULTRASONIC_ECHO);
  
  if (distance < 6) {  //detected
    Serial.println("Object detected, unlocking door for 10s...");
    doorLock = false;
    digitalWrite(doorState, HIGH);
    objectDetectedTime = millis();  // Start 10s timer
  }

  // Check if 10s pass after detecting 
  if (!doorLock && millis() - objectDetectedTime >= 10000) {
    Serial.println("Time up, locking door.");
    doorLock = true;
    digitalWrite(doorState, LOW);
  }

  // If the doors are unlocked this will make the doors open and close
  if (!doorLock) {
    float distanceLeft = getDistance(TRIG_s1, ECHO_s1); //TRIG_s1, ECHO_s1 this is what was there
    float distanceRight = getDistance(TRIG_s2, ECHO_s2);//TRIG_s2, ECHO_s2 

    Serial.print("Distance Left: "); Serial.print(distanceLeft); Serial.print(" cm  || ");
    Serial.print("Distance Right: "); Serial.print(distanceRight); Serial.println(" cm");

    checkBinStorage();

    if (!door1Open && !door2Open) {
      if (distanceLeft <= 10) {
        servo1.write(0);  // Open left door
        Serial.println("Opening left door");
        lastOpenTime = millis();
        door1Open = true;
      } else if (distanceRight < 10) {
        servo2.write(100);  // Open right door
        Serial.println("Opening right door");
        lastOpenTime = millis();
        door2Open = true;
      }
    }

    // Close doors after timeout if nothing is detected
    if (door1Open && distanceLeft > 10 && millis() - lastOpenTime >= doorOpenDuration) {
      servo1.write(100);  // Close left door
      Serial.println("Closing left door after timeout");
      door1Open = false;
    }

    if (door2Open && distanceRight > 10 && millis() - lastOpenTime >= doorOpenDuration) {
      servo2.write(5);  // Close right door
      Serial.println("Closing right door after timeout");
      door2Open = false;
    }
  } 
  else {  // If the door is locked  make sure the door stay close
    Serial.println("Door locked.");
    servo2.write(0);
    servo1.write(100);
  }

  delay(100);  
}

// function to get distance form ultrasonic
float getDistance(int TRIG, int ECHO) {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  long duration = pulseIn(ECHO, HIGH);
  float distance = (duration * 0.0343) / 2;  

  return distance;
}

//to sound an alarm if the bin is full or close to being full
void checkBinStorage() {
    float leftStorage = getDistance(TRIG_s3, ECHO_s3);
    float rightStorage = getDistance(TRIG_s4, ECHO_s4);

    Serial.print("Storage Left: "); Serial.print(leftStorage); Serial.print(" cm  || ");
    Serial.print("Storage Right: "); Serial.print(rightStorage); Serial.println(" cm");

    if (leftStorage <= 6 || rightStorage <= 6) {
        if (!buzzerActive) {  // activate only when the buzzer is not already buzzing
            buzzerActive = true; 
            buzzerStartTime = millis();
            buzzerState = 0;
            Serial.println("Bin is full! Activating buzzer...");
        }

        // Non-blocking buzzer control
        if (millis() - buzzerStartTime >= BUZZER_DELAY) {
            buzzerStartTime = millis();
            if (buzzerState == 0) {
                digitalWrite(BUZZER, HIGH);
                buzzerState = 1;
            } else {
                digitalWrite(BUZZER, LOW);
                buzzerState = 0;
            }
        }
    } else {
        if (buzzerActive) {
            Serial.println("Bin is no longer full. Stopping buzzer.");
            buzzerActive = false;
            digitalWrite(BUZZER, LOW); // make sure the buzzer is off when bin is not full
        }
    }
}