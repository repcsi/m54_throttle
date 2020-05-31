#include <Arduino.h>
#include <PID_v1.h>
//#include <EnableInterrupt.h>

//pins: 7: enable 
//      8: inA
//      9: inB
//     11: pwm

#define ENABLE_PIN 7
#define INA_PIN 8
#define INB_PIN 9
#define PWM_PIN 11


#define TBV_PIN1 A0
#define TBV_PIN2 A1
#define PED_PIN1 A2
#define PED_PIN2 A3


///////In search for 20KHz
// TCCR1A
// Bit                7      6      5      4      3      2        1       0
// Bit Name         COM1A1 COM1A0 COM1B1 COM1B0   -----  -----    WGM11   WGM10
// Initial Value      0      0      0      0      0      0        0       0
// changed to         1      1      1      1      0      0        1       0
//TCCR1A = B11110010;

// TCCR1B
// Bit              7      6      5       4        3       2       1        0
// Bit Name         ICNC1  ICES1  -----   WGM13    WGM12   CS12    CS11     CS10
// Initial Value    0      0      0       0        0       0       0        0
// changed to       0      0      0       1        1       0       0        1  
// CS12,CS11,CS10 = (0,0,1) System clock, no division
//TCCR1B = B00011001;

//TCCR1B = 25;// 00011001
//ICR1 = 1023 ; // 10 bit resolution
//OCR1A = 511; // vary this value between 0 and 1024 for 10-bit precision
//OCR1B = 511; // vary this value between 0 and 1024 for 10-bit precision      
///////In search for 20KHz

/////second option:
      // Timer 1 configuration
      // prescaler: clockI/O / 1
      // outputs enabled
      // phase-correct PWM
      // top of 400
      //
      // PWM frequency calculation
      // 16MHz / 1 (prescaler) / 2 (phase-correct) / 400 (top) = 20kHz
      //TCCR1A = 0b10100000;
      //TCCR1B = 0b00010001;
      //ICR1 = 400;
/////second option:


//Define the PID controller
double Input, Output, Setpoint;

int acceleratorRead;

//PID Control variables
//double kP = 2.3;
//double kP = 2;
//double kP = 12;
double kP = 20;
//double kI = 1;
double kI = .8;
//double kD = .9;
//double kD = 0.01;
double kD = 0.02;

PID repPid(&Input, &Output, &Setpoint, kP, kI, kD, DIRECT);


//Setting tps and accelerator limits.
int tpsMin = 150;
int tpsMax = 970;
int accMin = 0;
int accMax = 1023;

//vals:
int pedMin=165;
int pedMax=800;

int pedSum;

//Zeroing Error
double error = 0;
  
//Zeroing helper for mapping accelerator to TPS range
int accelerationRequest = 0;

void setupTimers(){
  // Timer 1 configuration
  // prescaler: clockI/O / 1
  // outputs enabled
  // phase-correct PWM
  // top of 400
  //
  // PWM frequency calculation
  // 16MHz / 1 (prescaler) / 2 (phase-correct) / 400 (top) = 20kHz
  TCCR1A = 0b10100000;
  TCCR1B = 0b00010001;
  ICR1 = 400;
  //PWM1 = 9;
  //OCR1A = 400 // max pwm on 20kHz if we need to fallback speed must be multiplied by 51/80, because 400*51/80=255
}

void moveValve();

void forward(int pwm, int duration){
  digitalWrite(ENABLE_PIN,HIGH);
  digitalWrite(INA_PIN,LOW);
  digitalWrite(INB_PIN,HIGH);
  analogWrite(PWM_PIN,pwm);
  delay(duration);
  analogWrite(PWM_PIN,0);
  digitalWrite(ENABLE_PIN,LOW);
}

void backward(int pwm, int duration){
  digitalWrite(ENABLE_PIN,HIGH);
  digitalWrite(INA_PIN,HIGH);
  digitalWrite(INB_PIN,LOW);
  analogWrite(PWM_PIN,pwm);
  delay(duration);
  analogWrite(PWM_PIN,0);
  digitalWrite(ENABLE_PIN,LOW);
}

void setup() {
  //potmeter
  pinMode(A0, INPUT);
  
  //tps 0.5 - 4.5v
  pinMode(A1,INPUT);
  
  //tps 4.5 - 0.5v
  pinMode(A2,INPUT);
  
  //enable pin
  //pinMode(7,OUTPUT);
  //pwm pin
  pinMode(PWM_PIN,OUTPUT);
  
//  TCCR1A = 0b10100000;
//  TCCR1B = 0b00010001;
//  ICR1 = 400;  
//  OCR2A = 180; 180 / 255 = 70.6% 
  
//
//  throttlePID.SetMode(AUTOMATIC);
//  throttlePID.SetSampleTime(1);

  Serial.begin(115200);
  Serial.println("Start, waiting 2 secs...");
  delay(2000);
  
  //Input = (double)analogRead(A1);
  
  repPid.SetMode(AUTOMATIC);
  
  digitalWrite(ENABLE_PIN,HIGH);
  digitalWrite(INA_PIN,LOW);
  digitalWrite(INB_PIN,HIGH);
  
  analogWrite(PWM_PIN,100);
  delay(1000);
  analogWrite(PWM_PIN,0);
}

void loop() {
  
  int pedRead = analogRead(PED_PIN1); //  x
  int pedRead2 = analogRead(PED_PIN2); // 2 times x
  pedSum = pedRead + pedRead;
  Serial.println("pedread 1:");
  Serial.println(pedRead);
  Serial.println("pedread 2:");
  Serial.println(pedRead2);
  //pedal - this works
  int acceleratorRead=map(pedSum,pedMin,pedMax, 0,100);

  //valve this doesnt
  int tpsSensorRead = analogRead(TBV_PIN1);   // 4.5-0.5v
  int tpsSensorRead2 = analogRead(TBV_PIN2);  // 0.5-4.5v

  Serial.println("tps 1:");
  Serial.println(tpsSensorRead);
  Serial.println("tps 2:");
  Serial.println(tpsSensorRead2);

  //delay(2000);
  
  int valveRead = map(tpsSensorRead2, tpsMin, tpsMax, 0, 100);

  Serial.println("Pedal %: ");
  //Serial.println(tpsValue);
  Serial.println(acceleratorRead);

  Serial.println("Valve %: ");
  //Serial.println(tpsValue);
  Serial.println(valveRead);
  

  Input=(double)analogRead(A0);
  
  Setpoint=(double)acceleratorRead;
  Setpoint=(double)valveRead;
  Input=(double)tpsSensorRead;
  Setpoint = acceleratorRead;
  Input = valveRead;
  Serial.println("Input: ");
  Serial.println(Input);
  Serial.println("Setpoint: ");
  Serial.println(Setpoint);
  repPid.SetSampleTime(1);
  repPid.SetMode(AUTOMATIC);
  repPid.Compute();
//  if (Output > 254){
//    Output = 255;
//  }
  
  analogWrite(PWM_PIN,Output);
  Serial.println("Output: ");
  Serial.println(Output);
  //delay(1000);
}
