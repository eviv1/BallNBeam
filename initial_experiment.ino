#include <PID_v1.h>
#include <HCSR04.h>
#include <SimpleKalmanFilter.h> // to improve on the error of the noise
#include <Servo.h>

// the pins
const double ServoPin = 9; // PWM pin for the motor
const double trigPin = 10; // PWM pin for the trigger pin
const double echoPin = 11; // PWM pin for the echo pin

float distance, ball_position, pos, angle;
double Input, Output, setPoint;

// the parameters, can be changed
const double Setpoint = 0;
const double K_values[1][3] = 
{{4,2,15}, // conservative
{5,1,15}}; // aggressive

const double Kp = 4;
const double Ki = 2;
const double Kd = 15;

// distances on the beam
const double minDistance = 2; // mininmum distance to detect (cm)
const double maxDistance = 50; // max distance of beam (cm)
const double equiPosition = maxDistance/2; // point where the ball balances at the beam
const double stableAngle = 47;

// sensor processing
const double Error = 2;
const double Variance = 0.5;

//initialization
Servo myservo;  // create servo object to control a servo
HCSR04 hc(trigPin, echoPin); //initialisation class HCSR04 (trig pin , echo pin)
PID myPID(&Input, &Output, &setPoint, Kp, Ki, Kd, DIRECT); // pid
SimpleKalmanFilter simpleKalmanFilter(Error, Error, Variance);

void setup() {

  // attaches the servo on pin 9 to the servo object 
  myservo.attach(ServoPin);  
	Serial.begin(9600); 

  //turn the PID on
  myPID = PID(&Input, &Output, &setPoint, Kp, Ki, Kd, DIRECT);
  myPID.SetOutputLimits(20,110);  // Constrain output to servo-safe range
  myPID.SetMode(AUTOMATIC);
  myservo.write(stableAngle);

}

void read(){
  distance = hc.dist();
  Serial.print("Distance: ");  
	Serial.print(distance); 
  if (distance > minDistance && distance < maxDistance)
  {
    ball_position = simpleKalmanFilter.updateEstimate(distance);
  }
  
	delay(20);
}

void move(){
  angle = Output;  // Use the PID output directly
  myservo.write(angle);  // Convert to 0-180 range if needed

  // debug
  Serial.print(" | Angle: ");  
  Serial.print(angle);
}

void loop() {
  read();
    Input = equiPosition - ball_position;  // Simplified calculation
    myPID.Compute();

/** if (distance < 5.5){
      Output = 20;
      move();
    }
  else if (distance > 19 && distance < 25){
      Output = 45;
      move();
  }
  else if (distance > 50){
      Output = 110;
      move();
    }
  else{
    move();
  }**/


    // debug
   // Serial.print("Ball_pos: "); Serial.print(ball_position); 
    Serial.print(" | Error (h): "); Serial.println(equiPosition - ball_position);

  }
