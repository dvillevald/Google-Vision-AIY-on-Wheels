
//============================UNIROI========================================
// Comprehensive Experiment
//=============================================================================
#include <IRremote.h>
//Infrared Remote Control

//==============================
int RECV_PIN = 2; // Set Infrared Remote port
IRrecv irrecv(RECV_PIN);
decode_results results; // Store infrared remote decode data
unsigned long last = millis();
#define run_car     '1'//key go
#define back_car    '2'//key back
#define left_car    '3'//key left
#define right_car   '4'//key right
#define stop_car    '0'//key stop
//===============================================
/*Car running status enumeration*/
//============================================
enum {
  enSTOP = 0,
  enRUN,
  enBACK,
  enLEFT,
  enRIGHT,
  enTLEFT,
  enTRIGHT
} enCarState;

//==============================
//==============================
//Speed control
//==============================
int control = 150;//PWM control speed
#define level1  0x08//Speed level flag
#define level2  0x09
#define level3  0x0A
#define level4  0x0B
#define level5  0x0C
#define level6  0x0D
#define level7  0x0E
#define level8  0x0F

//==============================
//==============================
int Left_motor_back=9;       
int Left_motor_go=8;         

int Right_motor_go=6;       
int Right_motor_back=7;   

int Right_motor_en=5;      
int Left_motor_en=10;   
//==============================
//Bluetooth protocol related
//==============================
int incomingByte = 0;       // Store Received Data(byte)
String inputString = "";         // Store Received Data(String)
boolean newLineReceived = false; // Previous data end flag
boolean startBit  = false;  // Protocol start flag
String returntemp = ""; // Store return data

/*Set BUZZER port*/
int BUZZER = 12;             

/*Set Button port*/
int KEY = 13;
/*Set right &left LED port*/
int right_led=4;
int left_led=3;

/*Set Ultrasonic Sensor*/
int Echo = A1;  // Set Echo port
int Trig = A0; // Set Trig port
int Distance = 0;
/*Line Walking*/
const int SensorRight = A2;   	// Set Right Line Walking Infrared sensor port
const int SensorLeft = A3;     	// Set Left Line Walking Infrared sensor port
int SL;    // State of Left Line Walking Infrared sensor
int SR;    // State of Right Line Walking Infrared sensor
/*follow*/
const int SensorRight_2 = A4;     // Right Tracking Infrared sensor
const int SensorLeft_2 = A5;     // Left Tracking Infrared sensor
int SL_2;    // State of Left Tracking Infrared sensor
int SR_2;    // State of Right Tracking Infrared sensor



//State
int g_carstate = enSTOP; //  1:front 2:back 3:left 4:right 0:stop // State of vehicle running state
int g_modeSelect = 0;  // 0:remote control mode(default); 1:line walking mode ; 2: obsracle avoidance mode; 3: tracking
int g_modeComunication = 0; // 0:Infrared remote control 1:Bluetooth remote control
int g_AllState = 0;  // 0: Busying; 1:Mode selection 
//int g_IRRealse = 0; //Remote control buttons loosen detection


/*Format string initialization*/
int serial_putc( char c, struct __file * )
{
  Serial.write( c );
  return c;
}
void printf_begin(void)
{
  fdevopen( &serial_putc, 0 );
}

void setup()
{
   //Initialize motor drive for output mode
  pinMode(Left_motor_go, OUTPUT);
  pinMode(Left_motor_back, OUTPUT); 
  pinMode(Right_motor_go, OUTPUT); 
  pinMode(Right_motor_back, OUTPUT); 
  pinMode(Right_motor_en, OUTPUT); 
  pinMode(Left_motor_en, OUTPUT); 
  pinMode(BUZZER, OUTPUT);// Set buzzer as output
  pinMode(left_led, OUTPUT);//Set led as output
  pinMode(right_led, OUTPUT);//Set led as output
  pinMode(KEY, INPUT_PULLUP);// Set button as input and internal pull-up mode

  pinMode(Echo, INPUT);    // Set Ultrasonic echo port as input
  pinMode(Trig, OUTPUT);   // Set Ultrasonic trig port as input

  pinMode(SensorRight, INPUT); // Set Right Line Walking Infrared sensor as input
  pinMode(SensorLeft, INPUT); // Set left Line Walking Infrared sensor as input

 
 
  Serial.begin(9600);	// Set Bluetooth baud rate 9600


  digitalWrite(BUZZER, LOW);   //Set beep mute
  digitalWrite(Left_motor_en, HIGH); // set left motor enble
  digitalWrite(Right_motor_en, HIGH); // set right motor enble

 //Initialize state
  g_carstate = enSTOP; // stop
  g_modeComunication = 0; // Infrared Remote Control
  g_modeSelect = 0;    // remote mode
  
  //Initialize Infrared decode
  irrecv.enableIRIn();
  pinMode(RECV_PIN, INPUT_PULLUP);

  printf_begin();
}

void Distance_test()   // Measuring front distance
{
  digitalWrite(Trig, LOW);   // set trig port low level for 2μs
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);  // set trig port high level for 10μs(at least 10μs)
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);    // set trig port low level
  float Fdistance = pulseIn(Echo, HIGH);  // Read echo port high level time(unit:μs)
  Fdistance = Fdistance / 58;    // Distance(m) =(time(s) * 344(m/s)) / 2     /****** The speed of sound is 344m/s.*******/
                                 //  ==> 2*Distance(cm) = time(μs) * 0.0344(cm/μs)
                                 // ==> Distance(cm) = time(μs) * 0.0172 = time(μs) / 58
  Distance = Fdistance;
}

void run1()     // Advance
{
  digitalWrite(Left_motor_en,HIGH);  // Left motor enable
  analogWrite(Left_motor_en,157);
  digitalWrite(Right_motor_en,HIGH);  // Right motor enable
  digitalWrite(Right_motor_go,HIGH);  // right motor go ahead
  digitalWrite(Right_motor_back,LOW);   
  analogWrite(Right_motor_go,210);//PWM--Pulse Width Modulation(0~255). right motor go speed is 255.
  analogWrite(Right_motor_back,0); 
  digitalWrite(Left_motor_go,HIGH);  // set left motor go ahead
  digitalWrite(Left_motor_back,LOW);
  analogWrite(Left_motor_go,130);//PWM--Pulse Width Modulation(0~255).left motor go speed is 135.
  analogWrite(Left_motor_back,0);
  digitalWrite(left_led, LOW);
  digitalWrite(right_led, LOW);
  //delay(time * 100);   //Running time can be adjusted 
}

void brake()         //Stop
{
  digitalWrite(Left_motor_back, LOW);
  digitalWrite(Left_motor_go, LOW);
  digitalWrite(Right_motor_go, LOW);
  digitalWrite(Right_motor_back, LOW);
   digitalWrite(left_led, HIGH);
  digitalWrite(right_led, HIGH);
  
}

void left()         //Turn left
{
  digitalWrite(Right_motor_go,HIGH);	// right motor go ahead
  digitalWrite(Right_motor_back,LOW);
  analogWrite(Right_motor_go,250); // PWM--Pulse Width Modulation(0~255) control speed，right motor go speed is 255.
  analogWrite(Right_motor_back,0);
  digitalWrite(Left_motor_go,LOW);   // left motor stop
  digitalWrite(Left_motor_back,LOW); 
  analogWrite(Left_motor_go,0); 
  analogWrite(Left_motor_back,0); 
  digitalWrite(left_led, HIGH);
  digitalWrite(right_led, LOW);
}

void spin_left()         //Left rotation
{
  digitalWrite(Right_motor_go,HIGH);// right motor go ahead
  digitalWrite(Right_motor_back,LOW);
  analogWrite(Right_motor_go,200);// PWM--Pulse Width Modulation(0~255) control speed ,right motor go speed is 200.
  analogWrite(Right_motor_back,0);
  digitalWrite(Left_motor_go,LOW);   // left motor back off
  digitalWrite(Left_motor_back,HIGH);
  analogWrite(Left_motor_go,0); 
  analogWrite(Left_motor_back,200);// PWM--Pulse Width Modulation(0~255) control speed,left motor back speed is 200.
   digitalWrite(left_led, HIGH);
  digitalWrite(right_led, LOW);
}

void right()        //turn right
{
  digitalWrite(Right_motor_go,LOW);   // right motor stop
  digitalWrite(Right_motor_back,LOW);
  analogWrite(Right_motor_go,0); 
  analogWrite(Right_motor_back,0);
  digitalWrite(Left_motor_go,HIGH);// left motor go ahead
  digitalWrite(Left_motor_back,LOW);
  analogWrite(Left_motor_go,230);// PWM--Pulse Width Modulation(0~255) control speed ,left motor go speed is 255.
  analogWrite(Left_motor_back,0);
   digitalWrite(left_led, LOW);
  digitalWrite(right_led, HIGH);
}

void spin_right()        //Right rotation
{
    digitalWrite(Right_motor_go,LOW);  // right motor back off
  digitalWrite(Right_motor_back,HIGH);
  analogWrite(Right_motor_go,0); 
  analogWrite(Right_motor_back,200);// PWM--Pulse Width Modulation(0~255) control speed
  digitalWrite(Left_motor_go,HIGH);// left motor go ahead
  digitalWrite(Left_motor_back,LOW);
  analogWrite(Left_motor_go,200);// PWM--Pulse Width Modulation(0~255) control speed 
  analogWrite(Left_motor_back,0);
   digitalWrite(left_led, LOW);
  digitalWrite(right_led, HIGH);
}

void back()          //back off
{
   digitalWrite(Right_motor_go,LOW); //right motor back off
  digitalWrite(Right_motor_back,HIGH);
  analogWrite(Right_motor_go,0);
  analogWrite(Right_motor_back,150);// PWM--Pulse Width Modulation(0~255) control speed
  analogWrite(Right_motor_en,165);
  digitalWrite(Left_motor_go,LOW);  //left motor back off
  digitalWrite(Left_motor_back,HIGH);
  analogWrite(Left_motor_go,0);
  analogWrite(Left_motor_back,140);// PWM--Pulse Width Modulation(0~255) control speed
  digitalWrite(left_led, LOW);
  digitalWrite(right_led, HIGH);
  //led();
}
void whistle()   //beep sounds
{
  /* int i;
  for (i = 0; i < 15; i++) //Output one frequency sound
  {
    digitalWrite(BUZZER, LOW); //sounds
    delay(10);// delay 10ms
    digitalWrite(BUZZER, HIGH); //mute
    delay(1);// delay 1ms
  }*/
    digitalWrite(BUZZER, HIGH); //sounds
    delay(2000);//delay 2s
    digitalWrite(BUZZER, LOW); //mute
   // delay(2);//delay 2s
  //}
}


/*Bluetooth receive*/
void Bluetooth(void)
{
  if (newLineReceived)
  {
    //Determine if mode selection or not
    if (inputString[1] == 'M' && inputString[2] == 'O' && inputString[3] == 'D' && inputString[4] == 'E')
    {
      if (inputString[6] == '0') //stop
      {
        g_carstate = enSTOP;
        g_modeSelect = 0;
        g_AllState = 0;
        BeepOnOffMode();
      }
      else
      {
        switch (inputString[5])//the fifth bit in the string (0:remote control mode(default); 1:line walking mode ; 2: obsracle avoidance mode; 3: tracking)
        {
          case '0': g_modeSelect = 0; break;
          case '1': g_modeSelect = 1; break;
          case '2': g_modeSelect = 2; break;
          case '3': g_modeSelect = 3; break;
          default: g_modeSelect = 0; break;
        }
        g_AllState = 0;
         BeepOnOffMode();
      }
    }
    else if (g_modeSelect == 0 && g_AllState == 0) //remote control mode(default)
    {
      switch (inputString[1]) //Direction
      {
        case run_car:   g_carstate = enRUN;  break;
        case back_car:  g_carstate = enBACK;  break;
        case left_car:  g_carstate = enLEFT;  break;
        case right_car: g_carstate = enRIGHT; break;
        case stop_car:  g_carstate = enSTOP;  break;
        default: g_carstate = enSTOP; break;
      }
      if (inputString[3] == '1') //Left rotation
      {
        spin_left();
        delay(2000);//delay 2s
        brake();
      }
      else if (inputString[3] == '2') //Right rotation
      {
        spin_right();
        delay(2000);
        brake();
      }
      if (inputString[5] == '1') //beep sounds
      {
        whistle();
        
      }
      if (inputString[7] == '1') //speed up
      {
        control += 50;
        if (control > 255)
        {
          control = 255;
        }
           }
      if (inputString[9] == '1') //speed reduction
      {
        control -= 50;
        if (control < 50)
        {
          control = 100;
        }
         }
    
     
    }
    inputString = "";   // clear the string
    newLineReceived = false;

  }
}

void Key_Scan(void)
{
  int val;
  while (!digitalRead(KEY)) // When the button is pressed
  {
    delay(10);
    val = digitalRead(KEY);// Reads the button ,the level value assigns to val
    if (val == LOW) //Double check the button is pressed
    {
      if (g_modeComunication == 0)// In Infrared remote control
      {
        g_modeComunication = 1; // Switch to Bluetooth remote control
        //digitalWrite(LED, HIGH); //led lights up
      }
      else//In  Bluetooth remote control
      {
        g_modeComunication = 0; //Switch to Infrared remote control
        //digitalWrite(LED, LOW); led lights off
      }
      digitalWrite(BUZZER, HIGH);		//beep sounds
      delay(100);//100ms
      digitalWrite(BUZZER, LOW);		//beep mute
      while (!digitalRead(KEY));	//Determine if the button is released or not
    }
    else
      digitalWrite(BUZZER, LOW); //beep mute
  }
}

//Car running control
void CarControl()
{
  if (g_modeSelect != 2 )
  {
    switch (g_carstate)
    {
      case enSTOP: brake(); break;
      case enRUN: run1(); break;
      case enLEFT: left(); break;
      case enRIGHT: right(); break;
      case enBACK: back(); break;
      case enTLEFT: spin_left(); break;
      case enTRIGHT: spin_right(); break;
      default: brake(); break;
    }
   
  }
}

void IR_Deal()
{
  if (irrecv.decode(&results)) //Infrared received result
  {
    //Serial.println(results.value, HEX);

    //if (((results.value >> 16) & 0x0000ffff) == 0x00ff)
    //{
    //printf("$AR,HSX,%08lX#\n", results.value);
    //According to the different values to perform different operations
    //  00FD10EF   7  mode off
    //  00FDA05F   5  NC
    //  00FD609F   6  NC
    //  00FD50AF   9  mode play
    //  00FD906F   8   NC
    //  00FDB04F   0  beep
    //  00FD8877   up  Advance
    //  00FD28D7   <   Turn left
    //  00FDA857   ok   Stop
    //  00FD6897   >   Turn right
    //  00FD9867   DW  Back
    //  00FD30CF   *  Left rotation
    //  00FD708F   #  right rotation

    //  00FD00FF   1    remote control mode  1
    //  00FD807F   2    obstacle avoidance mode  2
    //  00FD40BF   3    line walking mode  3
    //  00FD20DF   4    tacking mode  4
    switch (results.value)
    {
      case 0x00FD10EF: g_carstate = enSTOP; g_AllState = 0; g_modeSelect = 0; BeepOnOffMode() ;break; // 7 mode off
      case 0x00FD50AF: g_carstate = enSTOP; g_AllState = 0; BeepOnOffMode() ; break; //  9 mode play 
      //case 0x00FFE01F: g_AllState = 1; g_modeSelect--; if (g_modeSelect == -1) g_modeSelect = 3; break;
      //case 0x00FF906F: g_AllState = 1; g_modeSelect++; if (g_modeSelect == 4) g_modeSelect = 0;  break;
      case 0x00FD00FF: g_AllState = 1; g_modeSelect = 0;  ModeBEEP(g_modeSelect); break; // 1  remote control mode  1
      case 0x00FD807F: g_AllState = 1; g_modeSelect = 2;  ModeBEEP(g_modeSelect); break; // 2  obstacle avoidance mode  2
      case 0x00FD40BF: g_AllState = 1; g_modeSelect = 1;  ModeBEEP(g_modeSelect); break; //3   line walking mode  3
      case 0x00FD20DF: g_AllState = 1; g_modeSelect = 3;  ModeBEEP(g_modeSelect); break; //4   tacking mode  4

      default: break;
    }
    if (g_modeSelect == 0 && g_AllState == 0)
    {
      switch (results.value)
      {
        
        //case 0x00FF02FD: control += 50; if (control > 255) control = 255; break;
        //case 0x00FF9867: control -= 50; if (control < 50) control = 100; break;

        case 0x00FDB04F: whistle(); break;  //   0  beep  OFF/ON
        case 0x00FD8877:  g_carstate = enRUN; break;    // up  Advance
        case 0x00FD28D7:  g_carstate = enLEFT; break;   // <   Turn left
        case 0x00FDA857:  g_carstate = enSTOP; break;   // ok   Stop
        case 0x00FD6897:  g_carstate = enRIGHT; break;  // >   Turn right
        case 0x00FD9867:  g_carstate = enBACK; break;   //  DW  Back
        case 0x00FD30CF:  g_carstate = enTLEFT; break;  // *  Left rotation
        case 0x00FD708F:  g_carstate = enTRIGHT; break; // #  right rotation
        default: break; //Stay the same

      }
     
    }

    //}
    last = millis();
    irrecv.resume(); // received next Infrared decode
  }
  else if (millis() - last > 120)
  {
    g_carstate = enSTOP;
    last = millis();
  }

}
//mode display
void ModeBEEP(int mode)
{
  for (int i = 0; i < mode + 1; i++)
  {
    digitalWrite(BUZZER, HIGH); //beep sounding
    delay(100);
    digitalWrite(BUZZER, LOW); //mute
    delay(100);
  }
  delay(100);
  digitalWrite(BUZZER, LOW); //mute
}
//buzzer sounds for 1s to wait mode selection
void BeepOnOffMode()
{
  digitalWrite(BUZZER, HIGH); //beep sounding
  delay(1000);
  digitalWrite(BUZZER, LOW); //mute
}

//line walking mode
void track()
{
  /**************************************************************************************
  Infrared signal back means white undersurface ,returns low level and led lights up.
  Infrared signal gone means black undersurface ,returns high level and led lights off. 
  **************************************************************************************/
  SR = digitalRead(SensorRight);//Right Line Walking Infrared sensor against white undersurface,then LED[L2] light illuminates and while against black undersurface,LED[L2] goes off
  SL = digitalRead(SensorLeft);//Left Line Walking Infrared sensor against white undersurface,then LED[L3] light illuminates and while against black undersurface,LED[L3] goes off

  if (SL == LOW && SR == LOW) // Black lines were not detected at the same time
    g_carstate = enRUN;   // go ahead
  else if (SL == LOW & SR == HIGH)// Left sensor against white undersurface and right against black undersurface , the car left off track and need to adjust to the right.
    g_carstate = enRIGHT;
  else if (SR == LOW & SL ==  HIGH) // Rihgt sensor against white undersurface and left against black undersurface , the car right off track and need to adjust to the left.
    g_carstate = enLEFT;
  else // Black lines were detected at the same time , the car stop.
    g_carstate = enSTOP;
}
// ultrason_obstacle_avoid
void ultrason_obstacle_avoiding()
{
  Distance_test();// Measuring front distance

  if (Distance < 38) ////The value is the distance that meets the obstacle, and can be set according to the actual situation
  {
    delay(10);
    Distance_test();////Measuring front distance
    while (Distance < 38)  //Determine whether there is an obstruction again.If there is obstacle , turn the direction and determine again
    {

      spin_right();//Right rotation for 400ms
      delay(400);
      brake();//stop
      Distance_test();////Measuring front distance
    }
  }
  else
    run1();//There is nothing obstacled. Go  Advance
}
// infrared follow mode
void Infrared_follow()
{
 /**************************************************************************************
  Infrared signal back means there is something obstacled ,returns low level and led lights up.
  Infrared signal gone means there is nothing obstacled ,returns high level and led lights off.
  **************************************************************************************/
  SR_2 = digitalRead(SensorRight_2);//Right infrared sensor detects the obstacle,then LED[L5] light illuminates and otherwise it goes off.
  SL_2 = digitalRead(SensorLeft_2);//Left infrared sensor detects the obstacle,then LED[L4] light illuminates and otherwise it goes off.
  if (SL_2 == LOW && SR_2 == LOW)//There is something obstacled ,goes and follow it.
    g_carstate = enRUN;   
  else if (SL_2 == HIGH & SR_2 == LOW)// There is something obstacled on the right then LED[L4] light illuminates,turns right and follow.
    g_carstate = enRIGHT;
  else if (SR_2 == HIGH & SL_2 == LOW)// There is something obstacled on the left then LED[L4] light illuminates,turns left and follow.
    g_carstate = enLEFT;
  else // There is nothing obstacled , stop.
    g_carstate = enSTOP;
}


/*main loop*/
void loop()
{
  /*Check button press*/
  Key_Scan();

  if (g_modeComunication == 0) //Infrared Remote Control mode
  {
    IR_Deal();
  }
  else//Bluetooth Remote Control mode
  {
    Bluetooth();
  }

  // Switch different mode
  if (g_AllState == 0)
  {
    switch (g_modeSelect)
    {
      case 1: track(); break; // Line walking mode
      case 2: ultrason_obstacle_avoiding(); break;// Obstacle avoidance mode
      case 3: Infrared_follow(); break;// Tracking mode
    }
  }
  CarControl();
}

//Serial read data
void serialEvent()
{
  while (Serial.available())
  {
    incomingByte = Serial.read();   //One byte by one byte reads 
    if (incomingByte == '$')  // '$' means the start of packet
    {
      startBit = true;
    }
    if (startBit == true)
    {
      inputString += (char) incomingByte;     // The received data constitutes a completed packet.
    }
    if (incomingByte == '#')    // '#' means the end of packet
    {
      newLineReceived = true;
      startBit = false;
    }
  }
}
