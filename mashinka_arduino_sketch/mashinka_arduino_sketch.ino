
/* 
пример команд:
пшикать сколько (мили)секунд
перестать пшикать
ответ на проверку связи
вкл-выкл телеметрию
изменить уроветь яркости якоря


еще программа должна передавать расстояние до стены
и подтверждать выполнение команд


ПЛОХО РАБОТАЕТ
видимо из-за того что постоянно шлет данные о расстоянии теряются отправленные байты?

при увеличении тайминга проблемы не исчезают
попробовать отключить пересылку ир_данных и слать команды пшикать
без пересылки данных все ок работает

надо пробовать посылать байты изредко с ардуино
будет ли в таком случает стабильно работать прием?

может надо все же неким асинхронным методом читать байты и в буфер их записывать
типа в треде

еще посмотреть частоту аналог_реад, может надо читать не чаще некторого значения 

может реализовать задачу длительности пшика
*/

#include <Servo.h> 

//####### global constants 

// int inByte = 0;         // incoming serial byte
const int c_serial_bps       = 9600;
const int first_servo_pin    =    3; // first servo pin
const int  servo_neutral_pos =    140; // bylo 50 so starym servo
const int  servo_armed_pos   =    50; // bylo 180 so starym servo

const int      sensorIR      =    0;

// constants for case statement
const int c_pshyk            = 1;
const int c_stop_pshyk       = 3;
const int c_check_connection = 7;



//####### global variables
Servo first_servo;  // create servo object to control a servo 
unsigned long when_release_servo;
//boolean  send_ir_data  =    true; // !!!
boolean  send_ir_data  =    false; // !!!


void setup()
{
  // start serial port
  Serial.begin( c_serial_bps );
  
  first_servo.attach(first_servo_pin);  // attaches the servo on pin 10 to the servo object 
  
  when_release_servo = millis(); // переменная для отключение серво-привода пшикалки
  
  Serial.print(42);
}

byte serial_read()
{
  byte incoming_byte = 0;

  if ( Serial.available() ){
     incoming_byte = Serial.read();

   // Serial.println( incoming_byte );
    //delay(700);  
  }
	
  return incoming_byte;
}

void pshyk( int duration ) // duration ot 1 do 32, 1 eq. 333 miliseconds, 32 eq. 10656 miliseconds
{
  //Serial.println("pshyk"); Serial.println(duration);
 
 first_servo.write( servo_armed_pos );
 when_release_servo = millis() + int(duration * 1000/3.0);

}

void stop_pshyk() 
{
  //Serial.println("stop_pshyk");
 
 first_servo.write( servo_neutral_pos );
 when_release_servo = millis();

}

void check_connection( byte recieved_number ) 
{
  delay(100);
  serial_send( recieved_number ); // надо вернуть число + 196?
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
  { first_servo.write( servo_neutral_pos ); }

}


void loop()
{
  //Serial.println("start");
  //Serial.println(millis());
  byte incoming_byte = 0;
  incoming_byte = serial_read(); 
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
  }
    
  release_servo_if_needed( false );
  
  
  //Serial.println("1");   // send an initial string
//  delay(40);
  //delay(220);
  //Serial.println(millis());
  
  if ( send_ir_data )
  {
  serial_send( get_ir_range() );
  }
}

/*

 switch (inChar) {   
    case 'u':
      // move mouse up
      Mouse.move(0, -40);
      break; 
    case 'd':
      // move mouse down
      Mouse.move(0, 40);
      break;
    case 'l':
      // move mouse left
      Mouse.move(-40, 0);
      break;
    case 'r':
      // move mouse right
      Mouse.move(40, 0);
      break;
    case 'm':
      // move mouse right
      Mouse.click(MOUSE_LEFT);
      break;
    }
	
	
	
*/


