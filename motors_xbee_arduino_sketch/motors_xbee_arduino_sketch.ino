const int motors_count = 4;

int speed_motor_pins[]    ={ 3, 5, 6, 9 }; // зеленые провода

int direction_motor_pins[]={ 2, 4, 7, 8 }; // бело-зеленые провода


void setup()
{
  Serial.begin(9600);
  
  for (int i = 0; i < motors_count; i++) {
    pinMode(speed_motor_pins[i],    OUTPUT);
    pinMode(direction_motor_pins[i],OUTPUT);
  }      
  
}

void loop()
{

  delay(2000);
  change_direction(LOW); // turn all motors forward
 
  change_speed(120); // provide power to all motors
}

// думаю эти ф-и должны принимать ссылку на массив, а по умолчанию изменять значения для всех моторов
void change_direction(int dir)
{
  for (int i = 0; i < motors_count; i++) {
    digitalWrite(direction_motor_pins[i],dir);
  }
}

void change_speed(int spd)
{
    for (int i = 0; i < motors_count; i++) {
    analogWrite(speed_motor_pins[i], spd);
  }
}

