#include <ESP8266WiFi.h>
#include <SPI.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include <LOLIN_HP303B.h>

#include <SimpleKalmanFilter.h>

/*Debugging*/
#define DEBUG false 
#define DEBUG_SERIAL if(DEBUG)Serial

#define TIMER_LIMIT 5000
#define SERVO_OPEN_POS 180
#define SERVO_CLOSED_POS 0
#define ACCEL_START_THRESHOLD 2
#define ACCEL_LANDED_THRESHOLD 0.7
#define APOGE_THRESHOLD 0.2

#define OVERSAMPLING 4
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C
#define PIN_SPI_SS    (17)
#define SERVO_PIN 0
#define CHIPSELECT D4


Adafruit_SSD1306 display(OLED_RESET);
LOLIN_HP303B PressureSensor;
Adafruit_MPU6050 AccelerationSensor;

File last_apogee_file, flight_history_file, index_file;

SimpleKalmanFilter pressureKalmanFilter(1, 1, 0.01);
SimpleKalmanFilter accelerationtKalmanFilter(1, 1, 0.03);
Servo release_door_servo;

enum { STATE_DISPLAY = 0, STATE_IDLE = 1, STATE_LAUNCHED = 2, STATE_APOGE = 3, STATE_DESCENT = 4, STATE_LANDED = 5};
long temperature;
float altitude_ref = 0;
float apoge=0.;
float corrected_altitude, corrected_elevation, current_acceleration, current_elevation ;
float acceleration_ref=0;
int32_t pressure;
int16_t ret;
int state, tmp, servo_timer=0;
String line, flight_index;
sensors_event_t a, g, temp;


float get_asl_altitude()
{
  ret = PressureSensor.measurePressureOnce(pressure, OVERSAMPLING);
  if (ret != 0)
  {
    return -999;
  }
  else
  { 
    //return 44330.0*(1.0-pow(pressure/101325.0,1.0/5.255));
    return pressureKalmanFilter.updateEstimate(44330.0*(1.0-pow(pressure/101325.0,1.0/5.255)));
  }
}

void display_state(int state){
  display.clearDisplay();
  display.setTextSize(2); 
  display.setCursor(0, 0);
  display.println(state);
  display.display();
}

void display_altitude(float elevation) {
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setCursor(0, 0);     // Start at top-left corner
  display.println("Apogee:");
  display.setTextSize(2);
  display.print(elevation);
  display.println("m");
  display.display();
}

void init_servo(int pos=SERVO_CLOSED_POS) {
  release_door_servo.write(pos);  // set servo in lock position
  release_door_servo.attach(SERVO_PIN);
  delay(100); // wait servo going to position
}


void setup()
{
  
  WiFi.mode( WIFI_OFF );
  WiFi.forceSleepBegin();

  #ifdef DEBUG
     Serial.begin(115200);
  #endif
  
  // Screen initialization
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.fillScreen(BLACK);
  display.clearDisplay();
  display.display();

  // SD card initialization
  DEBUG_SERIAL.println("");
  DEBUG_SERIAL.print("SD card initialization.... ");
  if (!SD.begin(CHIPSELECT)) {
    DEBUG_SERIAL.println("Failed !");
    return;
  }
  DEBUG_SERIAL.println("OK");

  // Servo motor initialization
  DEBUG_SERIAL.print("Servo initialization.... ");
  init_servo();
  DEBUG_SERIAL.println("OK");

  // HP303B pressure sensor initialization
  PressureSensor.begin();

  // MPU initialization
  DEBUG_SERIAL.print("MPU initialization.... ");
  if (!AccelerationSensor.begin()) {
    DEBUG_SERIAL.print("Failed !");
    return;
  }
  DEBUG_SERIAL.println("OK");
  
  AccelerationSensor.setAccelerometerRange(MPU6050_RANGE_8_G);
  AccelerationSensor.setGyroRange(MPU6050_RANGE_500_DEG);
  AccelerationSensor.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Acceleration calibration
  DEBUG_SERIAL.print("Accel calibration.... ");
  AccelerationSensor.getEvent(&a, &g, &temp);
  for (int i = 0; i <= 50; i += 1) {
    current_acceleration = sqrt(sq(a.acceleration.x) + sq(a.acceleration.y) + sq(a.acceleration.z));
    acceleration_ref = acceleration_ref + current_acceleration;
    delay(15);
  }
  acceleration_ref = acceleration_ref/51.;

  for (int i = 0; i <= 50; i += 1) {
    current_acceleration = sqrt(sq(a.acceleration.x) + sq(a.acceleration.y) + sq(a.acceleration.z))-acceleration_ref;
    current_acceleration = accelerationtKalmanFilter.updateEstimate(current_acceleration);
  }
  DEBUG_SERIAL.println("OK");

  // Reading file index
  DEBUG_SERIAL.print("File Index reading.... ");
  index_file = SD.open("index.txt");
    if (index_file) {
      while (index_file.available()) {
        flight_index = index_file.readStringUntil('\n');
      }
      index_file.close();
      DEBUG_SERIAL.println("OK");
    } else {
      DEBUG_SERIAL.println("Failed !");
    }
  
  // State machine initialization
  state = STATE_DISPLAY;

}


void loop()
{ 
  // Open history file the first time
  if (!flight_history_file) {
    flight_history_file = SD.open("hist_" + String(flight_index.toInt()) + ".csv", FILE_WRITE);
    flight_history_file.println("time;state;acc_kal;alt_kal");
    DEBUG_SERIAL.println("print header");  
    if (!flight_history_file) {
      DEBUG_SERIAL.println("erro reading flight history file");  
    }
  }

  AccelerationSensor.getEvent(&a, &g, &temp);
  
  current_acceleration = abs(sqrt(sq(a.acceleration.x) + sq(a.acceleration.y) + sq(a.acceleration.z))-acceleration_ref);
  current_acceleration = accelerationtKalmanFilter.updateEstimate(current_acceleration);
  
  corrected_elevation =  get_asl_altitude() - altitude_ref;

  if (state > STATE_IDLE) {
    flight_history_file.println(String(millis()-servo_timer)+";"+String(state)+";"+String(current_acceleration)+";"+String(corrected_elevation));
  }


  switch (state) {

    case STATE_DISPLAY:
      DEBUG_SERIAL.println("state DISPLAY");
      last_apogee_file = SD.open("apogee.txt");
      if (last_apogee_file) {
        while (last_apogee_file.available()) {
          line = last_apogee_file.readStringUntil('\n');
        }
        display_altitude(line.toFloat());
        last_apogee_file.close();
      } else {
        DEBUG_SERIAL.println("error opening apogee.txt");
      }
      delay(3000);
      display.ssd1306_command(SSD1306_DISPLAYOFF);
      state = STATE_IDLE;
      break;

    
    case STATE_IDLE:
      DEBUG_SERIAL.println("state IDLE");
      DEBUG_SERIAL.print("linear accel: ");
      DEBUG_SERIAL.println(current_acceleration);
    
      if (current_acceleration < ACCEL_START_THRESHOLD) {
        // keep current pressure as reference, avoid elevation driffting
        altitude_ref = get_asl_altitude();
      } else {
        servo_timer = millis();
        state = STATE_LAUNCHED;
      }
      break;


    case STATE_LAUNCHED:
      DEBUG_SERIAL.println("state LAUNCHED");
      if (corrected_elevation >apoge) {
          apoge=corrected_elevation;
          //DEBUG_SERIAL.print("elevation: ");
          //DEBUG_SERIAL.println(current_acceleration);
      } else if (corrected_elevation<apoge-APOGE_THRESHOLD) {
        DEBUG_SERIAL.println("apoge reach, going to state apoge");
        state = STATE_APOGE;
      } 
      
      if (millis()-servo_timer>TIMER_LIMIT) {
        DEBUG_SERIAL.println("timer reach, going to state apoge");
         state = STATE_APOGE;
      }
      break;


    case STATE_APOGE:
      DEBUG_SERIAL.println("state APOGE");
      release_door_servo.write(SERVO_OPEN_POS);

      
      last_apogee_file = SD.open("apogee.txt", FILE_WRITE);
      if (last_apogee_file) {
        last_apogee_file.println(apoge);
        last_apogee_file.close();
      } else {
        DEBUG_SERIAL.println("error opening apogee.txt");
      }
      state = STATE_DESCENT;
      break;
    

    case STATE_DESCENT:
      if ((millis()-servo_timer>300*1000) | ((current_acceleration < 0.7))  ) {
        DEBUG_SERIAL.println("go to landed");
        state = STATE_LANDED;
      }
      break;


    case STATE_LANDED: 
      flight_history_file.close();  
      SD.remove("index.txt"); 
      index_file = SD.open("index.txt", FILE_WRITE);
      index_file.println(String(flight_index.toInt()+1));
      index_file.close();
      
      release_door_servo.write(SERVO_CLOSED_POS);

      delay(1000);
      ESP.deepSleep(0);

      break;

  }

}
