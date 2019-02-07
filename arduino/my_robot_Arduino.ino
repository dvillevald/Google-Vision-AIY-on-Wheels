#include <PinChangeInterrupt.h>
#include <IRremote.h>

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

/*Set Button port*/
int KEY = 13;
/*Set right & left LED port*/
int right_led=4;
int left_led=3;

/*Set Ultrasonic Sensor*/
int Echo = A1;  // Set Echo port
int Trig = A0; // Set Trig port
int Distance = 0;
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

/*
 * Define pins used to provide RC PWM signal to Arduino
 * Pins 8, 9 and 10 are used since they work on both ATMega328 and 
 * ATMega32u4 board. So this code will work on Uno/Mini/Nano/Micro/Leonardo
 * See PinChangeInterrupt documentation for usable pins on other boards
 */

// Mapping of Gooogle AIY kit pins to Ardunion pins (connected via logic level converter):
//------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Google AIY pin  |  Arduino pin  | gpiozero function |                                 Purpose                                                                   |
//------------------------------------------------------------------------------------------------------------------------------------------------------------------
//    pin_A        |      11       |       PWMLED      | Sends "angle between robot pose and direction to face" from Google AIY to Arduino                         |
//    pin_B        |      12       |       PWMLED      | Sends "distance to face" in inches from Google AIY to Arduino                                             |
//    pin_C        |      A3       |       PWMLED      | Sends clock_frame from AIY to Arduino (frame counter used to check if face detector is running)           |
//    pin_D        |      A2       |       Button      | Set HIGH when Arduino is powered - used to automatically shutdown Google AIY when Arduino is powered down |
//------------------------------------------------------------------------------------------------------------------------------------------------------------------

const byte channel_pin[] = {11, 12, A3};             /* Arduino pins assigned to (PWMLED) channels PIN_A, PIN_B and PIN_C of Google Vision AIY kit) */ 
volatile unsigned long rising_start[] = {0, 0, 0};   
volatile long channel_length[] = {0, 0, 0};          /* Time spans in microsecond between LOW-to-HIGH and HIGH-to-LOW for channels PIN_A, PIN_B and PIN_C of Google Vision AIY kit) */
const int x_center = 820;                            /* location of the horizontal center of teh camera in pixels (horizontal resolution 1640 /2 */
const int focal_length  = 1320;                      /* focal length of PiCamera in pixels (estimated by calibration) */
float min_angle = - atan2(x_center,focal_length);    /* min angle between direction the robot faces and direction to the detected face */
float max_angle = atan2(x_center,focal_length);      /* max angle between direction the robot faces and direction to the detected face */
const int min_distance = 20;                         /* min distance to face in inches */ 
const int max_distance = 200;                        /* max distance to face in inches */
float a = (0.9 - 0.2)/(max_distance - min_distance); /* coefficient for coding/decoding of distance (PIN_B) */
float b = 0.9 - a * max_distance;                    /* coefficient for coding/decoding of distance (PIN_B) */
const int gpiozero_pwmled_freq_in_hz = 100;          /* default base frequency of gpiozero library's PWMLED function (refer to documentation) */
String print_output = "short";                       /* How to print output. Options: "long" = detailed output, "short" = one-line summary, "none" (or any other) = no output */

float angle = 0.;
float angle_raw = 0.;
float distance_to_face = 0.;
float clock_frame = 0.;
float clock_frame_prev = 0.;
int face_detected_on_prev_frame = 0;
int dir_of_last_detected_face = 1; /* determines the direction of robot's rotation to search for faces 1 = To the RIGHT, 0 = to the LEFT */      

void setup() {
  Serial.begin(9600);

  //Initialize motor drive for output mode
  pinMode(Left_motor_go, OUTPUT);
  pinMode(Left_motor_back, OUTPUT); 
  pinMode(Right_motor_go, OUTPUT); 
  pinMode(Right_motor_back, OUTPUT); 
  pinMode(Right_motor_en, OUTPUT); 
  pinMode(Left_motor_en, OUTPUT); 
  pinMode(left_led, OUTPUT);//Set led as output
  pinMode(right_led, OUTPUT);//Set led as output
  pinMode(KEY, INPUT_PULLUP);// Set button as input and internal pull-up mode

  pinMode(Echo, INPUT);    // Set Ultrasonic echo port as input
  pinMode(Trig, OUTPUT);   // Set Ultrasonic trig port as input

  pinMode(SensorRight_2, INPUT); //Set Right  Infrared sensor as input
  pinMode(SensorLeft_2, INPUT); //Set left Infrared sensor as input

  digitalWrite(Left_motor_en, HIGH); // set left motor enble
  digitalWrite(Right_motor_en, HIGH); // set right motor enble

 //Initialize state
  g_carstate = enSTOP;    // stop
  g_modeComunication = 0; // Infrared Remote Control
  g_modeSelect = 0;       // remote mode
  
  //Initialize Infrared decode
  irrecv.enableIRIn();
  pinMode(RECV_PIN, INPUT_PULLUP);

  pinMode(channel_pin[0], INPUT);
  pinMode(channel_pin[1], INPUT);
  pinMode(channel_pin[2], INPUT);
  pinMode(channel_pin[3], INPUT);
  pinMode(A2, OUTPUT);
  digitalWrite(A2, HIGH);
  
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(channel_pin[0]), onRising0, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(channel_pin[1]), onRising1, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(channel_pin[2]), onRising2, CHANGE);
}

void processPin(byte pin) {
  uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(channel_pin[pin]));

  if(trigger == RISING) {
    rising_start[pin] = micros();
  } else if(trigger == FALLING) {
    channel_length[pin] = micros() - rising_start[pin];
  }
}

void onRising0(void) {
  processPin(0);
}

void onRising1(void) {
  processPin(1);
}

void onRising2(void) {
  processPin(2);
}

// Ultrasonic sensor
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

void run2()     // Advance
{
  digitalWrite(Left_motor_en,HIGH);  // Left motor enable
  analogWrite(Left_motor_en,157);
  digitalWrite(Right_motor_en,HIGH);  // Right motor enable
  digitalWrite(Right_motor_go,HIGH);  // right motor go ahead
  digitalWrite(Right_motor_back,LOW);   
  analogWrite(Right_motor_go,210);//PWM--Pulse Width Modulation(0~255). right motor go speed is 255. 210
  analogWrite(Right_motor_back,0); 
  digitalWrite(Left_motor_go,HIGH);  // set left motor go ahead
  digitalWrite(Left_motor_back,LOW);
  analogWrite(Left_motor_go,210);//PWM--Pulse Width Modulation(0~255).left motor go speed is 135. 130
  analogWrite(Left_motor_back,0);
  digitalWrite(left_led, LOW);
  digitalWrite(right_led, LOW);
  //delay(time * 100);   //Running time can be adjusted 
}

void run1()     // Advance (test)
{
  digitalWrite(Left_motor_en,HIGH);  // Left motor enable
  analogWrite(Left_motor_en,157);
  digitalWrite(Right_motor_en,HIGH);  // Right motor enable
  digitalWrite(Right_motor_go,HIGH);  // right motor go ahead
  digitalWrite(Right_motor_back,LOW);   
  analogWrite(Right_motor_go,170);//PWM--Pulse Width Modulation(0~255). right motor go speed is 255. 210
  analogWrite(Right_motor_back,0); 
  digitalWrite(Left_motor_go,HIGH);  // set left motor go ahead
  digitalWrite(Left_motor_back,LOW);
  analogWrite(Left_motor_go,140);//PWM--Pulse Width Modulation(0~255).left motor go speed is 135. 130
  analogWrite(Left_motor_back,0);
  digitalWrite(left_led, LOW);
  digitalWrite(right_led, LOW);
  //delay(1000);   //Running time can be adjusted 
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
  digitalWrite(Right_motor_go,HIGH);  // right motor go ahead
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
  delay(100);
}

void right()        //turn right
{
  digitalWrite(Right_motor_go,LOW);   // right motor stop
  digitalWrite(Right_motor_back,LOW);
  analogWrite(Right_motor_go,0); 
  analogWrite(Right_motor_back,0);
  digitalWrite(Left_motor_go,HIGH);// left motor go ahead
  digitalWrite(Left_motor_back,LOW);
  analogWrite(Left_motor_go,250);// PWM--Pulse Width Modulation(0~255) control speed ,left motor go speed is 255.
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
  delay(100);
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
  digitalWrite(left_led, HIGH);
  digitalWrite(right_led, HIGH);
  //led();
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
        g_modeComunication = 1; // Switch to search_and_approach
      }
      else    //In  search_and_approach
      {
        g_modeComunication = 0; //Switch to Infrared remote control
      }
      while (!digitalRead(KEY));  //Determine if the button is released or not
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
      case 0x00FD10EF: g_carstate = enSTOP; g_AllState = 0; g_modeSelect = 0; break; // 7 mode off
      case 0x00FD50AF: g_carstate = enSTOP; g_AllState = 0; break;                   // 9 mode play 
      case 0x00FD00FF: g_AllState = 1; g_modeSelect = 0;  break;                     // 1 remote control mode
      case 0x00FD807F: g_AllState = 1; g_modeSelect = 2;  break;                     // 2 obstacle avoidance mode
      case 0x00FD40BF: g_AllState = 1; g_modeSelect = 1;  break;                     // 3 line walking mode
      case 0x00FD20DF: g_AllState = 1; g_modeSelect = 3;  break;                     // 4 tacking mode

      default: break;
    }
    if (g_modeSelect == 0 && g_AllState == 0)
    {
      switch (results.value)
      {
        
        //case 0x00FF02FD: control += 50; if (control > 255) control = 255; break;
        //case 0x00FF9867: control -= 50; if (control < 50) control = 100; break;

        case 0x00FDB04F: ; break;  //   0  beep  OFF/ON
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

// Ultrasonic sensor (avoid obstacles)
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
    delay(700);
    brake();//stop
    delay(500);
}

// Infrared follow-object mode
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


// Printing state of the robot 
void print_robot_state(String is_face_detected,String robot_action)
{
  String direction_to_rotate = "Right";
  if (dir_of_last_detected_face == 0)
  {
    direction_to_rotate = "Left";
  }

  if (print_output == "long")
  { 
    Serial.print("Face detected = ");
    Serial.println(is_face_detected);
    Serial.print("Action = ");
    Serial.println(robot_action);
    Serial.print("PIN_A (Angle to goal in grads) = ");
    Serial.println(angle);
    Serial.print("PIN_B (Disctance to face in inches) = ");
    Serial.println(distance_to_face);
    Serial.print("PIN_C (Frame ID) = ");
    Serial.println(clock_frame);
    Serial.print("Previous Frame ID = ");
    Serial.println(clock_frame_prev);
    Serial.print("Direction to rotate next time = ");
    Serial.println(direction_to_rotate);
    Serial.print("Distance to obstacle in inches (ultrasonic sensor) = ");
    Serial.println(Distance);
    Serial.println("---------------------------------------------------------------------");
  }
  else if (print_output == "short")
  {
    Serial.print("Face = ");
    Serial.print(is_face_detected);
    Serial.print(" | Action: ");
    Serial.print(robot_action);
    Serial.print(" | Angle =  ");
    Serial.print(angle);
    Serial.print(" | Distance = ");
    Serial.print(distance_to_face);
    Serial.print(" | Frame = ");
    Serial.print(clock_frame);
    Serial.print(" | Prev Frame = ");
    Serial.print(clock_frame_prev);
    Serial.print(" | Next Rotation  = ");
    Serial.print(direction_to_rotate);
    Serial.println("");
  }
}


// Autonomous driving more - robot is searching for and approaching human faces ot their images
// Warning: Don't put your face too close to the robot and don't let little children play with it without supervision!
void search_and_approach()
{
  Distance_test(); /* engage ultrasonic sensor (work-in-progress) */

  // Read anc decode angle between direction the robot faces and the direction to the detected face (from PIN_A of Google Vision AIY kit)   
  angle_raw = (channel_length[0]*gpiozero_pwmled_freq_in_hz/1000000. - 0.1)/0.8;
  angle = angle_raw * (max_angle - min_angle) + min_angle;

  // Read anc decode a distance from robot to the detected face in inches (from PIN_B of Google Vision AIY kit)
  // If this value is less than 20 then no face is detected.  
  distance_to_face = (channel_length[1]*gpiozero_pwmled_freq_in_hz/1000000. - b)/a;

  // Read anc decode the number of frame used to check if the arrived data is new (from PIN_C of Google Vision AIY kit)
  clock_frame = 100. * channel_length[2]*gpiozero_pwmled_freq_in_hz/1000000.;
  delay(100);

  if (abs(clock_frame - clock_frame_prev)> 1) /* check if the data from Google Vision AIY kit are new (updated) */
  {
    if (distance_to_face < 15) /* no face detected => search for face */
    {
      if (face_detected_on_prev_frame == 1) /* step to increase robustnes - if robot missed the face by accident it gets the second chance */
      {
         brake();
         delay(1500);
         face_detected_on_prev_frame = 0;
      }
      else
      {
        if (dir_of_last_detected_face == 1) /* searching for faces by rotating in the direction of the most recently detected face */
        { 
          spin_right();
          delay(50);
          print_robot_state("No","Spin right");

        }
        else
        {
          spin_left();
          delay(50);
          print_robot_state("No","Spin left");
          
        }
        brake();
        delay(750);
      }
    }
    else if (angle > min_angle && angle < max_angle) /* robot's actions when face is detected */
    {
      face_detected_on_prev_frame = 1;
      if (angle > 0.3)  /* adjust robot pose by spinning to the right */
      {
        spin_right();
        delay(2);
        brake();
        delay(500);
        dir_of_last_detected_face = 1;
        print_robot_state("Yes","Spin right");
      }
      else if(angle < -0.3)  /* adjust robot pose by spinning to the left */
      {
        spin_left();
        delay(2);
        brake();
        delay(500);
        dir_of_last_detected_face = 0;
        print_robot_state("Yes","Spin left");
      }
      else /* approach the target */
      {
        run1();
        delay(1000);
        brake();
        delay(500);
        /* update/correct the "direction to the last detected face" if the angle is different from zero */ 
        if (angle > 0.03)
        {
          dir_of_last_detected_face = 1;      
        }
        else if (angle < -0.03)
        {
          dir_of_last_detected_face = 0;      
        }
        print_robot_state("Yes","Go forward");
      }
    }
  }
  clock_frame_prev = clock_frame; /* update the clock */
}


void loop()
{
  /*Check button press*/
  Key_Scan();

  if (g_modeComunication == 0) //Infrared Remote Control mode
  {
    IR_Deal();
  }
  else //search and approach mode
  {
    search_and_approach();
  }

  // Switch different mode
  if (g_AllState == 0)
  {
    switch (g_modeSelect)
    {
      case 1: ultrason_obstacle_avoiding(); break;// Obstacle avoidance mode
      case 2: Infrared_follow(); break;// Tracking mode
    }
  }
  CarControl();
}
