#include <Arduino.h>
#include <PID_v1.h>

//pins: 7: enable 
//      8: inA
//      9: inB
//     11: pwm
//     A0: valve pot1
//     A1: valve pot2
//     A2: pedal pot1
//     A3: pedal pot1

#define ENABLE_PIN 4
#define INA_PIN 7
#define INB_PIN 8
#define PWM_PIN 9 //do not change this!


#define TBV_PIN1 A0
#define TBV_PIN2 A1
#define PED_PIN1 A2
#define PED_PIN2 A3

void analogWrite20k_Init( void )
{
  // Stop the timer while we muck with it
  TCCR1B = (0 << ICNC1) | (0 << ICES1) | (0 << WGM13) | (0 << WGM12) | (0 << CS12) | (0 << CS11) | (0 << CS10);
 
  // Set the timer to mode 14...
  //
  // Mode  WGM13  WGM12  WGM11  WGM10  Timer/Counter Mode of Operation  TOP   Update of OCR1x at TOV1  Flag Set on
  //              CTC1   PWM11  PWM10
  // ----  -----  -----  -----  -----  -------------------------------  ----  -----------------------  -----------
  // 14    1      1      1      0      Fast PWM                         ICR1  BOTTOM                   TOP
 
  // Set output on Channel A to...
  //
  // COM1A1  COM1A0  Description
  // ------  ------  -----------------------------------------------------------
  // 1       0       Clear OC1A/OC1B on Compare Match (Set output to low level).
  TCCR1A =
      (1 << COM1A1) | (0 << COM1A0) |   // COM1A1, COM1A0 = 1, 0
      (0 << COM1B1) | (0 << COM1B0) |
      (1 << WGM11) | (0 << WGM10);      // WGM11, WGM10 = 1, 0
 
  // Set TOP to...
  //
  // fclk_I/O = 16000000
  // N        = 1
  // TOP      = 799
  //
  // fOCnxPWM = fclk_I/O / (N * (1 + TOP))
  // fOCnxPWM = 16000000 / (1 * (1 + 799))
  // fOCnxPWM = 16000000 / 800
  // fOCnxPWM = 20000
  ICR1 = 799;
 
  // Ensure the first slope is complete
  TCNT1 = 0;
 
  // Ensure Channel B is irrelevant
  OCR1B = 0;
 
  // Ensure Channel A starts at zero / off
  OCR1A = 0;
 
  // We don't need no stinkin interrupts
  TIMSK1 = (0 << ICIE1) | (0 << OCIE1B) | (0 << OCIE1A) | (0 << TOIE1);

  // Ensure the Channel A pin is configured for output
  DDRB |= (1 << DDB1);

  // Start the timer...
  //
  // CS12  CS11  CS10  Description
  // ----  ----  ----  ------------------------
  // 0     0     1     clkI/O/1 (No prescaling)
  TCCR1B =
      (0 << ICNC1) | (0 << ICES1) |
      (1 << WGM13) | (1 << WGM12) |              // WGM13, WGM12 = 1, 1
      (0 << CS12) | (0 << CS11) | (1 << CS10);
} 

void analogWrite20k( uint16_t value )   {
    if ( (value >= 0) && (value < 800) )
  {
    OCR1A = value;
  }
}

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

// void moveValve();

// void forward(int pwm, int duration){
//   digitalWrite(ENABLE_PIN,HIGH);
//   digitalWrite(INA_PIN,LOW);
//   digitalWrite(INB_PIN,HIGH);
//   analogWrite(PWM_PIN,pwm);
//   delay(duration);
//   analogWrite(PWM_PIN,0);
//   digitalWrite(ENABLE_PIN,LOW);
// }

// void backward(int pwm, int duration){
//   digitalWrite(ENABLE_PIN,HIGH);
//   digitalWrite(INA_PIN,HIGH);
//   digitalWrite(INB_PIN,LOW);
//   analogWrite(PWM_PIN,pwm);
//   delay(duration);
//   analogWrite(PWM_PIN,0);
//   digitalWrite(ENABLE_PIN,LOW);
// }

void setup() {
  //potmeter
  pinMode(A0, INPUT);
  
  //tps 0.5 - 4.5v
  pinMode(A1,INPUT);
  
  //tps 4.5 - 0.5v
  pinMode(A2,INPUT);
  
  //enable pin
  pinMode(ENABLE_PIN,OUTPUT);
  //pwm pin
  pinMode(PWM_PIN,OUTPUT);

  Serial.begin(115200);
  Serial.println("Start, waiting 2 secs...");
  delay(2000);
  
  repPid.SetMode(AUTOMATIC);
  
  //enable motor controller and tell it to go forward
  digitalWrite(ENABLE_PIN,HIGH);
  digitalWrite(INA_PIN,LOW);
  digitalWrite(INB_PIN,HIGH);
  
  //initialize 20kHz mode for pwm
  analogWrite20k_Init();

  //movetest
  // analogWrite20k(400);
  // delay(1000);
  // analogWrite20k(400);
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
  Serial.println(acceleratorRead);

  Serial.println("Valve %: ");
  Serial.println(valveRead);
  

  Input=(double)analogRead(A0);
  
  // Setpoint=(double)acceleratorRead;
  // Setpoint=(double)valveRead;
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
  
  analogWrite20k(Output);
  Serial.println("Output: ");
  Serial.println(Output);
  //delay(1000);
}
