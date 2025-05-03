// Define HC-SR04 pins for sensor 1
#define TRIGGER_PIN1 A0 // Trigger pin for sensor 1
#define ECHO_PIN1 A1    // Echo pin for sensor 1

// Define HC-SR04 pins for sensor 2
#define TRIGGER_PIN2 A2 // Trigger pin for sensor 2
#define ECHO_PIN2 A3    // Echo pin for sensor 2

// Define HC-SR04 pins for sensor 3
#define TRIGGER_PIN3 A4 // Trigger pin for sensor 3
#define ECHO_PIN3 A5    // Echo pin for sensor 3

#define THRESHOLD 10  // this will deside where to turn if all three sensors detect somthing 

#define ENA 10 //left motor speed
#define ENB 11 //right motor speed
#define doorState 12 

// Define IR sensors
  #define IR_SENSOR_RIGHT 8
  #define IR_SENSOR_LEFT 9
  #define MOTOR_SPEED 150 //was 100 

  #define MOTOR_SPEED_OBJECT_AVOID 200

  bool blutoothModeOn = false;
  unsigned long obstacleDetectedTime = 0; // Time when obstacle was first detected
  bool obstacleDetected = false; // Flag to indicate if an obstacle is detected

  unsigned long stopTime =0; //used for the pir
  bool isStopped = false;


  
void setup() {
  // Motor pins setup
  pinMode(2, OUTPUT); // Left motor forward
  pinMode(3, OUTPUT); // Left motor backward
  pinMode(4, OUTPUT); // Right motor forward
  pinMode(5, OUTPUT); // Right motor backward

  // Ultrasonic sensor setup
  Serial.begin(9600);          // Start serial communication at 9600 baud
  
  pinMode(TRIGGER_PIN1, OUTPUT); // Set trigger pin for sensor 1 as output
  pinMode(ECHO_PIN1, INPUT);    // Set echo pin for sensor 1 as input
  pinMode(TRIGGER_PIN2, OUTPUT); // Set trigger pin for sensor 2 as output
  pinMode(ECHO_PIN2, INPUT);    // Set echo pin for sensor 2 as input
  pinMode(TRIGGER_PIN3, OUTPUT); // Set trigger pin for sensor 3 as output
  pinMode(ECHO_PIN3, INPUT);    // Set echo pin for sensor 3 as input

  pinMode(6,INPUT); // Enable/disable lineFollowing mode 

  pinMode(doorState,INPUT); //stop and allow the bot to move as the doors get locked or unlocked



  pinMode(ENA,OUTPUT);//ENA = 10
  pinMode(ENB,OUTPUT);//ENB = 11

  pinMode(IR_SENSOR_LEFT,INPUT); // =9
  pinMode(IR_SENSOR_RIGHT,INPUT); //= 8

}

void loop() {
  String blutoothCom ;

  // Read distance from all three sensors
  int distance1 = getDistance(TRIGGER_PIN1, ECHO_PIN1);
  int distance2 = getDistance(TRIGGER_PIN2, ECHO_PIN2);
  int distance3 = getDistance(TRIGGER_PIN3, ECHO_PIN3);

  // Print distances to Serial Monitor 
  // THIS IS FOR DEBUG DO NOT REMOVE
  
  /*
  Serial.print("Sensor 1 Distance: ");
  Serial.print(distance1);
  Serial.print(" cm  ");

  Serial.print("Sensor 2 Distance: ");
  Serial.print(distance2);
  Serial.print(" cm  ");

  Serial.print("Sensor 3 Distance: ");
  Serial.print(distance3);
  Serial.println(" cm"); 
  */
  
  if(Serial.available() || blutoothModeOn){
      blutoothCom = Serial.readStringUntil('\n'); 
      blutoothCom.trim();


      if(blutoothCom =="left"){
        digitalWrite(doorState,LOW); //lock door

        digitalWrite(2, HIGH);  // Left motor backward
        digitalWrite(3, LOW); // Left motor forward
        digitalWrite(4, LOW); // Right motor forward
        digitalWrite(5, HIGH);  // Right motor backward

        analogWrite(ENA,MOTOR_SPEED);
        analogWrite(ENB,MOTOR_SPEED);

        Serial.println("resived :"+blutoothCom);
      }
      else if(blutoothCom =="right"){
        digitalWrite(doorState,LOW); //lock door

        digitalWrite(2, LOW);  // Left motor backward
        digitalWrite(3, HIGH); // Left motor forward
        digitalWrite(4, HIGH); // Right motor forward
        digitalWrite(5, LOW);  // Right motor backward

        analogWrite(ENA,MOTOR_SPEED);
        analogWrite(ENB,MOTOR_SPEED);

        Serial.println("resived :"+blutoothCom);
      }
      else if(blutoothCom == "forward"){
        //digitalWrite(doorState,LOW); //make the bot move

        digitalWrite(2, LOW);  // Left motor backward
        digitalWrite(3, HIGH); // Left motor forward
        digitalWrite(4, LOW); // Right motor forward
        digitalWrite(5, HIGH);  // Right motor backward

        analogWrite(ENA,MOTOR_SPEED);
        analogWrite(ENB,MOTOR_SPEED);

        Serial.println("resived :"+blutoothCom);
      }
      else if(blutoothCom == "back"){
        digitalWrite(doorState,LOW); //lock door

        digitalWrite(2, HIGH);  // Left motor backward
        digitalWrite(3, LOW); // Left motor forward
        digitalWrite(4, HIGH); // Right motor forward
        digitalWrite(5, LOW);  // Right motor backward

        analogWrite(ENA,MOTOR_SPEED);
        analogWrite(ENB,MOTOR_SPEED);

        Serial.println("resived :"+blutoothCom);
      }
      else if(blutoothCom == "stop"){

        //digitalWrite(doorState,HIGH); //stop the bot

        digitalWrite(2, LOW);  // Left motor backward
        digitalWrite(3, LOW); // Left motor forward
        digitalWrite(4, LOW); // Right motor forward
        digitalWrite(5, LOW);  // Right motor backward

        analogWrite(ENA,0);
        analogWrite(ENB,0);

        Serial.println("resived :"+blutoothCom);
      }
      else if(blutoothCom =="On"){
        blutoothModeOn = true;
        Serial.println("Blutooth Controll On");
      }
      else if(blutoothCom =="off"){
        blutoothModeOn = false;
        Serial.println("Blutooth Controll Off");
      }
      else if(blutoothCom =="lineFollow"){
        lineFollow();
      }
      else if(blutoothCom =="objectVoid"){
          isSomethingThere(distance1,distance2,distance3);
      }
  }
else if(blutoothModeOn == false){
  //change the isStopped bool depending on the doorState pin state
  if(digitalRead(doorState) == HIGH){isStopped = true;}
  else{isStopped = false;}

  if(isStopped == false){
  if(digitalRead(6)==LOW){
      
      Serial.print("Mode : ");
      Serial.println("Obstacalavoid");

      isSomeOneThere();
      
      isSomethingThere(distance1,distance2,distance3);
  }
  else if(digitalRead(6)==HIGH){

      Serial.print("Mode : ");
      Serial.println("Linefollow");

      lineFollow();
  }
  }
  else{
    Serial.println("bot is stoping : movment detected in the top sensor");
    stop();
  }
}

    
  delay(50);
}
void turnLeft(int leftSpeed,int rightSpeed){
    Serial.println("turning left - escape");

    analogWrite(ENA,leftSpeed); 
    analogWrite(ENB,rightSpeed); 

    digitalWrite(2, HIGH);
    digitalWrite(3, LOW);
    digitalWrite(4, LOW); 
    digitalWrite(5, HIGH); 
}
void turnRight(int leftSpeed,int rightSpeed){
    Serial.println("turning right - escape");

    analogWrite(ENA,leftSpeed); 
    analogWrite(ENB,rightSpeed);  

    digitalWrite(2, LOW); 
    digitalWrite(3, HIGH); 
    digitalWrite(4, HIGH);
    digitalWrite(5, LOW);
}
void goForward(int leftSpeed,int rightSpeed){
    Serial.println("forward - escape");

    analogWrite(ENA,leftSpeed);
    analogWrite(ENB,rightSpeed);

    //all motors foward
    digitalWrite(2, LOW);
    digitalWrite(3, HIGH);
    digitalWrite(4, LOW);
    digitalWrite(5, HIGH);
}
void stop(){
    analogWrite(ENA,0);
    analogWrite(ENB,0);

    //all motors foward
    digitalWrite(2, LOW);
    digitalWrite(3, LOW);
    digitalWrite(4, LOW);
    digitalWrite(5, LOW);
}

// measure distance 
int getDistance(int triggerPin, int echoPin) {
  // 10-microsecond pulse to the trigger pin
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.034 / 2;

  // Return the distance
  return distance;
}

void checkLeft(int distanceLeft) {
  if (distanceLeft <= 15) {
    // Obstacle on the left - steer to the right
    digitalWrite(2, LOW); 
    digitalWrite(3, HIGH); 
    digitalWrite(4, HIGH); 
    digitalWrite(5, LOW);  

    analogWrite(ENA,MOTOR_SPEED_OBJECT_AVOID);
    analogWrite(ENB,MOTOR_SPEED_OBJECT_AVOID);

    Serial.println("Obstacle on the left. Turning right.");
  }
}

void checkRight(int distanceRight) {
  if (distanceRight <= 15) {
    // Obstacle on the right - steer to the left
    digitalWrite(2, HIGH); 
    digitalWrite(3, LOW);  
    digitalWrite(4, LOW);  
    digitalWrite(5, HIGH); 

    analogWrite(ENA,MOTOR_SPEED_OBJECT_AVOID);
    analogWrite(ENB,MOTOR_SPEED_OBJECT_AVOID);

    Serial.println("Obstacle on the right. Turning left.");
  }
}

void checkMiddle(int distanceMiddle, int distanceLeft, int distanceRight) {
  if (distanceMiddle <= 15) {
    Serial.println("Obstacle in the middle. Checking sides...");
    
    if (distanceLeft > 15 && distanceRight > 15) {
      //both sides are clear, default to turning right
      digitalWrite(2, LOW); 
      digitalWrite(3, HIGH); 
      digitalWrite(4, HIGH); 
      digitalWrite(5, LOW); 

      analogWrite(ENA,MOTOR_SPEED_OBJECT_AVOID);
      analogWrite(ENB,MOTOR_SPEED_OBJECT_AVOID);

      Serial.println("Both sides are clear. Turning right.");
    } else if (distanceLeft <= 15 && distanceRight > 15) {
      // Obstacle on the left, turn right
      digitalWrite(2, LOW); 
      digitalWrite(3, HIGH); 
      digitalWrite(4, HIGH); 
      digitalWrite(5, LOW);  

      analogWrite(ENA,MOTOR_SPEED_OBJECT_AVOID);
      analogWrite(ENB,MOTOR_SPEED_OBJECT_AVOID);

      Serial.println("Left blocked. Turning right.");
    } else if (distanceRight <= 15 && distanceLeft > 15) {
      // Obstacle on the right, turn left
      digitalWrite(2, HIGH); 
      digitalWrite(3, LOW); 
      digitalWrite(4, LOW);  
      digitalWrite(5, HIGH); 

      analogWrite(ENA,MOTOR_SPEED_OBJECT_AVOID);
      analogWrite(ENB,MOTOR_SPEED_OBJECT_AVOID);

      Serial.println("Right blocked. Turning left.");
    } else {
      // Both sides blocked, stop
      analyzeEscapeRoute(distanceLeft,distanceMiddle,distanceRight);
    }
  }
}

//logic of dectecting and avoding obsticals
void isSomethingThere(int distanceLeft, int distanceMiddle, int distanceRight) {
  if (distanceMiddle <= 15) {
    checkMiddle(distanceMiddle, distanceLeft, distanceRight);
  } else if (distanceLeft <= 15) {
    checkLeft(distanceLeft);
  } else if (distanceRight <= 15) {
    checkRight(distanceRight);
  } else {
    // No obstacles - move forward
    digitalWrite(2, LOW); 
    digitalWrite(3, HIGH);  
    digitalWrite(4, LOW); 
    digitalWrite(5, HIGH);  

    analogWrite(ENA,MOTOR_SPEED_OBJECT_AVOID);
    analogWrite(ENB,MOTOR_SPEED_OBJECT_AVOID);

    Serial.println("No obstacles. Moving forward.");
  }
}

String analyzeEscapeRoute(int distanceLeft, int distanceMiddle, int distanceRight) {
    Serial.println("Analyzing escape route...");
    if (distanceLeft > THRESHOLD || distanceMiddle > THRESHOLD || distanceRight > THRESHOLD) {
        if (distanceLeft >= distanceMiddle && distanceLeft >= distanceRight) {
            Serial.println("Turning LEFT as it has the most space.");
            return "LEFT";
        }
        if (distanceRight >= distanceMiddle && distanceRight >= distanceLeft) {
            Serial.println("Turning RIGHT as it has the most space.");
            return "RIGHT";
        }
        //this condition need to be fix as this will make the bot go forward when all sensors are bolcked and front has most space
       // Serial.println("Moving STRAIGHT as the middle has the most space.");
       // return "STRAIGHT";
    }
    Serial.println("No escape route possible.");
    executeEmergency(); //do emergency action as there is no way to go
    return "NONE";
}


void executeEmergency() {
    Serial.println("Executing emergency escape: Performing 180-degree turn...");

    // Rotate about 180 degrees
    digitalWrite(2, LOW);
    digitalWrite(3, HIGH);
    digitalWrite(4, HIGH);
    digitalWrite(5, LOW);

    analogWrite(ENA,MOTOR_SPEED_OBJECT_AVOID);
    analogWrite(ENB,MOTOR_SPEED_OBJECT_AVOID);

    delay(3000); //change the time to do exactly 180 trun

    Serial.println("Emergency escape complete. Stopping motors.");
    // Stop motors
    digitalWrite(2, LOW);
    digitalWrite(3, LOW);
    digitalWrite(4, LOW);
    digitalWrite(5, LOW);
}

//stop the bot if pir detect movement
void isSomeOneThere() {
    int stat = digitalRead(7);

    if (stat == 0) {
        Serial.println("No movement detected");
        isStopped = false;  // Reset the flag when no movement is detected
    } 
    else if (stat == 1) {
        if (!isStopped) { // Trigger once per movement detection
            Serial.println("Movement detected: stopping for 5 seconds");
            digitalWrite(2, LOW);
            digitalWrite(3, LOW);
            digitalWrite(4, LOW);
            digitalWrite(5, LOW);

            stopTime = millis();  // Start timer
            isStopped = true;      // Set the flag to indicate stopping
        }
    }

    // After 5 seconds, allow execution to continue
    if (isStopped && millis() - stopTime >= 5000) {
        Serial.println("Resuming normal operation");
        isStopped = false;
        // If you need to turn something back ON after stopping, do it here
    }

    //
    if (isStopped && millis() - stopTime > 3000) { // Wait 3 seconds before moving again
        isStopped = false;
    }
}



//needs fixing 
void adaptivePathFollowing(){
  int leftDistance = getDistance(TRIGGER_PIN1, ECHO_PIN1);
  int rightDistance = getDistance(TRIGGER_PIN3,ECHO_PIN3);

   // Decide which side to move based on distance
    if (leftDistance > rightDistance) {// left has more space
    Serial.println("left turning adaptive following");
        turnLeft(200,200); //first left turn
        delay(600);
        goForward(100,100); 
        delay(1000);
        turnRight(150,150); //second turn right to go parallal
        delay(600);
        goForward(150,150);
        delay(2000);
        turnRight(120, 120);//turn towards the line 
        delay(750);
       // goForward(100, 100); //go toward the line
       // delay(100);
        // **Move forward until LEFT IR detects black**

        Serial.println("Returning to line...");
        while (digitalRead(IR_SENSOR_LEFT) == HIGH) { // Move forward until LEFT IR detects black
            goForward(100, 100);
        }

        // **Now, turn left to align with the line**
        Serial.println("Left IR detected! Turning left to align...");
        turnLeft(100, 100);

    } else {
      Serial.println("right turning adaptive following");
        turnRight(200, 200); // First right turn
        delay(600);
        goForward(100, 100); 
        delay(1000);
        turnLeft(150,150); // Second turn left to go parallel
        delay(600);
        goForward(150, 150);
        delay(2000);
        turnLeft(120, 120); // Turn towards the line
        delay(600);
        //goForward(100,100); // Move towards the line
        //delay(100);

        // **Move forward until RIGHT IR detects black**
        Serial.println("Returning to line...");
        while (digitalRead(IR_SENSOR_RIGHT) == HIGH) { // Move forward until RIGHT IR detects black
            goForward(100, 100);
        }

        // **Now, turn right to align with the line**
        Serial.println("Right IR detected! Turning right to align...");
        turnRight(100, 100);
        delay(300); // Adjust this delay to fine-tune alignment
    }

}

void lineFollow() {
  int leftSensorValue = digitalRead(IR_SENSOR_LEFT);
  int rightSensorValue = digitalRead(IR_SENSOR_RIGHT);

  Serial.print("Left IR Sensor: ");
  Serial.print(leftSensorValue);
  Serial.print(" | Right IR Sensor: ");
  Serial.println(rightSensorValue);



  int frontDistance = getDistance(TRIGGER_PIN2, ECHO_PIN2);
  if(frontDistance <=10){stop();}


 // TCCR0B = TCCR0B & B11111000 | B00000010;

  if (frontDistance <= 15) {  // Obstacle detected in the middle
    if (!obstacleDetected) {
      obstacleDetected = true; // Mark that an obstacle is detected 
      obstacleDetectedTime = millis(); // Start the timer
    }

    // Stop the bot immediately if obstacle detected
    digitalWrite(2, LOW);
    digitalWrite(3, LOW);
    digitalWrite(4, LOW);
    digitalWrite(5, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);

    // Check if the obstacle is still detected after 5 seconds
    if (millis() - obstacleDetectedTime >= 10000) {
      Serial.println("Obstacle still detected after 10 seconds, calling adaptivePathFollowing");
      adaptivePathFollowing(); // Call the adaptive path following function after 5 seconds
    }
  } else {
    obstacleDetected = false; // No obstacle detected, reset the flag

    // Line following logic
  if (leftSensorValue == LOW && rightSensorValue == LOW) {
    Serial.println("Line in middle: forward");

    //delay(10);

    analogWrite(ENA,MOTOR_SPEED);
    analogWrite(ENB,MOTOR_SPEED);

    //all motors foward
    digitalWrite(2, LOW);
    digitalWrite(3, HIGH);
    digitalWrite(4, LOW);
    digitalWrite(5, HIGH);
    
  } 
  else if(leftSensorValue == HIGH && rightSensorValue == LOW){
    Serial.println("turning left");

    analogWrite(ENA,100); //was 50
    analogWrite(ENB,150); //was 100

    //all motors foward
    digitalWrite(2, HIGH);
    digitalWrite(3, LOW);
    digitalWrite(4, LOW); 
    digitalWrite(5, HIGH);  

  }
  else if(rightSensorValue == HIGH && leftSensorValue == LOW){
    Serial.println("turning right");

    analogWrite(ENA,150); //was 100
    analogWrite(ENB,100);  //was 50

    //all motors foward
    digitalWrite(2, LOW); 
    digitalWrite(3, HIGH); 
    digitalWrite(4, HIGH);
    digitalWrite(5, LOW);
  }
  else {
    Serial.println("Both detect the line");

    analogWrite(ENA,0);
    analogWrite(ENB,0);

    //all motors stop
    digitalWrite(2, LOW);
    digitalWrite(3, LOW);
    digitalWrite(4, LOW);
    digitalWrite(5, LOW);

   }
  }
}














  


