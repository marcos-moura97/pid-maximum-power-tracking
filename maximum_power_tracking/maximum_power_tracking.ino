#include <SimpleKalmanFilter.h>
#include <PID_v1.h>
#include <Servo.h>

#define SERVO 6
#define LDR1 A0
#define LDR2 A1

/////// Filter
SimpleKalmanFilter kf = SimpleKalmanFilter(2, 2, 0.01);
                                        //(e_mea,e_est,q)
                                        

Servo s;      // Servo Variable
int pos = 90; // Servo Initial Position


double Setpoint ; // will be the desired value
double Input;     // photo sensor
double Output ;   // LED
//PID parameters
double Kp=0.2, Ki=10, Kd=0; 

int a,b; // calibration variables

// LDR Values
int difference;
int ldrValue1 = 0; 
int ldrValue2 = 0; 

 
//create PID instance 
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);

void sendData2Supervisory(float Input_); // this function sends the data to be plotted on Processing program
void updatePos(); // this function updates the position of the servo

 
void setup()
{
  
  s.attach(SERVO);  // attaches the servo on pin 9 to the servo object 
  s.write(90);

  Serial.begin(9600);   
  //Hardcode the brigdness value
  Setpoint = 10;
  //Turn the PID on
  myPID.SetMode(AUTOMATIC);
  //Adjust PID values
  myPID.SetTunings(Kp, Ki, Kd);

  delay(500);

  // Calibration

  ldrValue1 = analogRead(LDR1);
  ldrValue2 = analogRead(LDR2);
  
  if(ldrValue1 > ldrValue2)
  {
    a = ldrValue1 - ldrValue2;
  }

  if(ldrValue2 > ldrValue1)
  {
    b = ldrValue2 - ldrValue1;
  }
}
 
void loop()
{

  ldrValue1 = analogRead(LDR1) + b;
  ldrValue2 = analogRead(LDR2) + a;

  //Read the value from the light sensor. Analog input : 0 to 1024. We map is to a value from 0 to 255 as it's used for our PWM function.
  //Input = map(abs(ldrValue1 - ldrValue2) , 0, 1024, 0, 255);  // photo senor is set on analog pin 5
  Input = abs(ldrValue1 - ldrValue2);
  //Input = Input + random(-100,100)/100.0;
  float Input_ = kf.updateEstimate(Input);
  //PID calculation
  myPID.Compute();
  //Write the output as calculated by the PID function
  analogWrite(3,Output); //LED is set to digital 3 this is a pwm pin. 

  
  sendData2Supervisory(Input_);  
  updatePos();
}

void sendData2Supervisory(float Input_){
  //Send data by serial for plotting 
  Serial.print(Input);
  Serial.print(":");
  Serial.print(Output);
  Serial.print(":");
  Serial.print(Setpoint);
  Serial.print(":");
  Serial.print(Input);
  Serial.print(":");
  Serial.print(Input_);
  Serial.print(":");
  Serial.println(pos);  
}

void updatePos(){
  if(Input > Setpoint)
  {
    if(ldrValue1 > (ldrValue2))
    {
      if(pos > 30)
      {
        pos--;
        s.write(pos);
      }
   }
  
   if(ldrValue2 > (ldrValue1))
   {
      if(pos < 150)
      {
        pos++;
        s.write(pos);
      }
   }
  }  
}
