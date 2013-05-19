
/* 
пример команд:
пшикать сколько (мили)секунд
перестать пшикать
ответ на проверку связи
вкл-выкл телеметрию
изменить уроветь яркости якоря
*/

#include <Servo.h> 

/* gyro */
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
MPU6050 mpu;
#define OUTPUT_READABLE_EULER

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

/* gyro */

//####### global constants 

// int inByte = 0;         // incoming serial byte
const int c_serial_bps       = 9600;
const int first_servo_pin    =    9; // vrode pin 9 ne budet rabotat kak pwm esli podkluchena <Servo.h> 
const int second_servo_pin    =  10;
const int first_servo_neutral_pos  =  140; // bylo 50 so starym servo
const int first_servo_armed_pos    =   50; // bylo 180 so starym servo
const int second_servo_neutral_pos =  180; 
const int second_servo_armed_pos   =    0; 
const int full_speed         =  255;

const int      sensorIR      =    0;

// constants for case statement
const int c_pshyk            = 1;
const int c_stop_pshyk       = 3;
const int c_toggle_options   = 4;
const int c_go               = 5;
const int c_check_connection = 7;
// options, 5 bit
const int opt_pshyk      = 1;
const int opt_led_inc    = 2;
const int opt_led_dec    = 3;

///////////// MASHINKA
const int motors_count = 4;

const int LEFT     = 10;
const int RIGHT    = 20;
const int FORWARD  = 30;
const int BACKWARD = 40;
const int STOP     = 50;

const int speed_motor_pins[]    ={ 3, 5, 6, 11 }; // зеленые провода
  
const int direction_motor_pins[]={ 2, 4, 7, 8 }; // бело-зеленые провода
  
const int left_speed_pins[]     ={ 6, 11 }; 
const int right_speed_pins[]    ={ 3, 5}; 
  
const int right_dir_pins[]     ={ 2, 4 }; 
const int left_dir_pins[]      ={ 7, 8 }; 

const int led_pin = 10; 

/////////////

//####### global variables
Servo first_servo;  // create servo object to control a servo 
Servo second_servo;

unsigned long when_release_servo;
unsigned long when_read_mpu;
unsigned long when_stop; 

boolean  send_ir_data  =    false; // !!!
boolean  use_gyro      =    false; 

// options
boolean opt_pshyk_flag      = false;  
boolean opt_use_marker      = false;  

float left_motors_speed  = 1;
float right_motors_speed = 1;

byte led_br = 0;

void setup()
{
  // start serial port
  Serial.begin( c_serial_bps );
  
  first_servo.attach(  first_servo_pin  ); 
  second_servo.attach( second_servo_pin )
  
  when_release_servo = millis(); // переменная для отключение серво-привода пшикалки
  
  ///////////// MASHINKA
  for (int i = 0; i < motors_count; i++) {
    pinMode(speed_motor_pins[i],  OUTPUT);
    pinMode(direction_motor_pins[i],OUTPUT);
     }  

  /* gyro */
  if (use_gyro){
  mpu_setup();
  }

  pinMode(led_pin, OUTPUT); 
}

byte serial_read()
{
  byte incoming_byte = 0;

  if ( Serial.available() ){
     incoming_byte = Serial.read();

   // Serial.println( incoming_byte );
  }
	
  return incoming_byte;
}

void pshyk( int duration ) // duration ot 1 do 32, 1 eq. 333 miliseconds, 32 eq. 10656 miliseconds
{
  //Serial.println("pshyk"); Serial.println(duration);
 
 first_servo.write(  first_servo_armed_pos  );
 second_servo.write( second_servo_armed_pos );
 when_release_servo = millis() + int(duration * 1000/3.0);

}

void stop_pshyk() 
{
  //Serial.println("stop_pshyk");
 
 first_servo.write( first_servo_neutral_pos );
 second_servo.write( second_servo_neutral_pos );
 when_release_servo = millis();

}

void mashinka( byte recieved_number ) 
{
  int duration;

  switch ( recieved_number ) {   
  case FORWARD/10:      
    go(FORWARD, 400);
    break;
  case BACKWARD/10:
    go(BACKWARD, 400);
    break;
  case STOP/10:
    when_stop  = millis();
    break;
  case LEFT/10:
    turn(LEFT, 100);
    break;  
  case RIGHT/10:
    turn(RIGHT, 100);
    break;    
    }
}

void check_connection( byte recieved_number ) 
{
  delay(100);
  serial_send( recieved_number ); // надо вернуть число + 196
  delay(100);
  pshyk(5);
}


byte command_number( byte incoming_byte )
{
  //byte mask = B11100000;
  //return (incoming_byte & mask) >> 5;
  return incoming_byte  >> 5;
}

byte command_argument( byte incoming_byte )
{
  byte mask = B00011111;
  return incoming_byte & mask;
}

void serial_send( byte byte_to_send )
{
  // пока пишу отправку одного байта
  Serial.write( byte_to_send );
  // именно write !
}

float calculate_ir_range ()
{
  const float m1 = 10650.08;
  const float m2 = -0.935;
  const float m3 = 10;
  
  float sensorValue;    //Must be of type float for pow()
  
  sensorValue = analogRead(sensorIR);
  //delay(5);
  
  return m1 * pow(sensorValue,m2) - m3;
}
/*
byte get_ir_range() 
{
  return byte( calculate_ir_range() );
}
*/


byte get_ir_range() // get the data from ir range sensor МОЖЕТ ДОБАВИТЬ ВОЗВРАТ ОТРИЦ. ЗНАЧЕНИЯ КОГДА ЗА ПРЕДЕЛАМИ 15-150 см?
{
//  Noah Stahl
//  5/25/2011
//  http://arduinomega.blogspot.com
//  Arduino Mega 2560
//This sketch is used to test the Sharp Long Range Infrared Sensor.
//The sensor output is attached to analog pin 15. Once the distance
//is calculated, it is printed out to the serial monitor.

  float  cm, sum, maximum, minimum, result;
  
  cm      = calculate_ir_range();
  maximum = cm;
  minimum = cm;
  sum     = cm;
  
  //Serial.println("--- "); 
  //Serial.print(cm); 

// пусть измеряет 4 (5?) раз и крайние значения выбрасывает, возвращает среднее из 2(3?)
  for (int i = 0; i < 3; i++) {
    cm      = calculate_ir_range();
    sum += cm;
    
    if ( maximum < cm )
    { maximum = cm; }
    
    if ( minimum > cm)
    { minimum = cm; }    
    
    //Serial.print(cm); 
  }    
  
  result = ( sum - maximum - minimum )*0.5;
  
  //Serial.println("---"); 
  //Serial.println(result); 
  //delay(100);
  //  Serial.print("Inches: ");
  
  if ( result < 20 || result > 150 )
  {
    return 0;
  } else
  {
    return byte( result );
  }  
}



void release_servo_if_needed(boolean anyway) // вместо использования делея: в этой функции собрать все действия после делей, например отключение пшыкалки
{
  if( when_release_servo <= millis() || anyway )
  { first_servo.write( first_servo_neutral_pos ); }
}

void stop_if_needed(boolean anyway) // вместо использования делея: в этой функции собрать все действия после делей, например отключение пшыкалки
{
  if( when_stop <= millis() || anyway )
    { 
      stop(); 

      if(opt_pshyk_flag){
        stop_pshyk();
      }
    }
}

void mpu_setup()
{
  Wire.begin();
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
}

void mpu_read()
{
  if (!dmpReady) return;

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);


      }
}


void loop()
{ 
  //Serial.println("start");
  //Serial.println(millis());
  byte incoming_byte = 0;
  incoming_byte = serial_read(); 
  byte c_argument = command_argument( incoming_byte );
  //incoming_byte = B00100011;
  
  // Serial.println(command_number(incoming_byte));
  
  switch ( command_number(incoming_byte) ) {   
    case c_pshyk:      
      pshyk( command_argument( incoming_byte ) );
      break;
    case c_stop_pshyk:
      stop_pshyk();
      break;
    case c_check_connection:
      check_connection( command_argument( incoming_byte ) ); 
      break;
    case c_toggle_options:
      toggle_options( c_argument ); 
      break;
    case c_go:
      mashinka( command_argument( incoming_byte ) );  
      break;       
  }
    
  release_servo_if_needed( false );
  stop_if_needed( false );
  
  //Serial.println("1");   // send an initial string
  //  delay(40);
  //delay(220);
  //Serial.println(millis());
  
  if ( send_ir_data )
  {
    serial_send( get_ir_range() );
  }

  if ( use_gyro && mpuInterrupt )
  {
    mpu_read();
  }
}

  ///////////// MASHINKA
  // думаю эти ф-и должны принимать ссылку на массив, а по умолчанию изменять значения для всех моторов
  void change_direction(int dir, const int direction_pins[], int count)
  {
    //  int count = sizeof(direction_pins)/sizeof(int);
    for (int i = 0; i < count; i++) {
      digitalWrite(direction_pins[i],dir);
    }
  }
  
  void stop()
   {
     change_speed(0, speed_motor_pins, motors_count);  
   }
  
  void change_speed(int spd,const int speed_pins[], int count)
   {
   //    int count = sizeof(speed_pins)/sizeof(int);
    for (int i = 0; i < count; i++) {
        analogWrite(speed_pins[i], spd);
      }
    }
    
  void weak_motor_compensation()
   {
    // analogWrite(6, 250);
    // analogWrite(9, 250);
   }
   
  void go(int direct, int duration)
   {
     stop();
     int state;
   
     if (direct == FORWARD) {      
      state = HIGH;
      }
     if (direct == BACKWARD) {
     state = LOW;
      }
   
     change_direction(state, direction_motor_pins, motors_count); // turn all motors forward/backward
     //change_speed(full_speed, speed_motor_pins, motors_count); // provide power to all motors

     change_speed((int)(full_speed*left_motors_speed),   left_speed_pins, 2);
     change_speed((int)(full_speed*right_motors_speed), right_speed_pins, 2);

     weak_motor_compensation();
     //delay(duration); // лучше бы определить угол     
    
 //    stop();
     if(opt_pshyk_flag){
      pshyk(32);
      when_stop  = millis() + duration*10;
     } else
     {
      when_stop  = millis() + duration;
     }
 }
 
 void turn(int direct, int duration)
 {
     int l_state, r_state;
     stop();
     
     if (direct == RIGHT) {
       r_state = HIGH;
       l_state = LOW;
     }
     if (direct == LEFT) {
       r_state = LOW;
       l_state = HIGH;
     }
     
     change_direction(r_state, right_dir_pins, 2);
     change_direction(l_state, left_dir_pins, 2);
     change_speed(full_speed, speed_motor_pins, motors_count);
     
     when_stop  = millis() + duration;
     //weak_motor_compensation();
     //delay(duration); // лучше бы определить угол
     //stop();  
}

void  toggle_options(byte recieved_number)
{  
  switch ( recieved_number ) {   
    case opt_pshyk:      
      if(opt_pshyk_flag){
        opt_pshyk_flag = false;                
      } else
      {
        opt_pshyk_flag = true;
      }

      break;
    case opt_led_inc:        
      led_br += 25;
      analogWrite(led_pin, led_br);
      break;
  }
}

/*
void  macros_xyz()
{  
  stop();

}
*/


