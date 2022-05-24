#include <Arduino.h>
#include <Wire.h>
#include "SPI.h"
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "RH_RF95.h"

// ======================================================== Defines ========================================================
// Status Indicator Pins
#define STATUS_LED_PIN        (5)
#define LED1_PIN              (15)
#define LED2_PIN              (16)
#define LED3_PIN              (17)
#define LED4_PIN              (6)

// Interface Pins
#define TE_CS_PIN             (10)
#define TE_INT_PIN            (3)

// Ematch Pins
#define EMATCH                (4)

// Vbatt Pins
#define VBATT                 (14)
// LoRa Defines
#define LORA_PACKET_SIZE      (33)        // bytes
#define LORA_FREQ             (868.0)     // MHz
#define LORA_SF               (7)

// Flight Parameters
#define EVENT_1_TIMER         (4000)     // seconds
#define EVENT_1_ALT           (100)         // meters

// Thresholds and Delay Defines
#define LIFTOFF_THRESHOLD     (30)         // m/s2
#define LIFTOFF_DELAY         (100)       // milliseconds
#define GROUND_ALT_THRESHOLD  (50)        // meters
#define MOTOR_BURNOUT         (1000)      // milliseconds
#define TOUCHDOWN_DELAY       (10000)     // milliseconds
#define APOGEE_THRESHOLD      (50)        //5 meters below apogee altitude (altitude multiplied by 10 to avoid float)
#define NUM_READINGS          (50)        //Number of readings in the moving average

// ======================================================== Hardware Interfaces ========================================================




// ======================================================== State Machine ========================================================
#define IDLE                  (1)
#define ARMED                 (2)
#define P_ASCENT              (3)
#define B_ASCENT              (4)
#define APOGEE                (5)
#define TOUCHDOWN             (6)

// ======================================================== Structures ========================================================
struct ACCEL
{
  float x;
  float y;
  float z;
};


// ======================================================== Global Variables ========================================================

// LoRa Variables
RH_RF95 rf95(TE_CS_PIN, TE_INT_PIN);
uint8_t lora_packet[LORA_PACKET_SIZE];
int16_t lora_rssi = 0;

// MPU Variables
Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;

// BMP Variables
Adafruit_BMP280 bmp;

//State Machine Variables
uint8_t current_state = IDLE;

//Battery Levels
float v_batt;


// Timers
uint16_t timer_burnout_init = 0;
uint16_t timer_re_event_init = 0;

// Misc. Variables
uint16_t liftoff_delay = false;
uint16_t liftoff_timer = 0;
uint8_t liftoff_status = 0;
uint8_t apogee_status = 0;
uint16_t touchdown_delay = 0;
uint16_t touchdown_timer = 0;
uint8_t touchdown_status = 0;
uint8_t first_execute = true;
float ground_altitude = 0;
float ground_pressure = 0;
float mean_alti = 0; //current altitude through filter
float max_alti = 0;
float total = 0; //for the filter
float readings [NUM_READINGS]; //table of readings
uint8_t avgIndex  = 0; //Index
uint32_t timestamp = 0;
// Sensor Variables
ACCEL accel_data;
ACCEL accel_calib;
float pressure;
float est_altitude;
float acc_x;
float acc_y;
float acc_z;

// ======================================================== Function Prototypes ========================================================
void heartbeat(void);
void MPU_init(void);
void MPU_read(void);
void BMP_init(void);
void BMP_read(void);
void lora_packet_build(void);
void lora_tx_handle(void);
void lora_rx_handle(void);
void lora_parse(uint8_t *buffer);
void state_update(void);
uint8_t liftoff_detect(void);
float alti_filter(void);
uint8_t apogee_detect(void);
uint8_t touchdown_detect(void);
void battery_level_read(void);
void ematch_trigger(uint8_t ematch);
void logData(void);

// ======================================================== Setup ========================================================
void setup() {

  // Pin Init
  pinMode(STATUS_LED_PIN,OUTPUT);
  pinMode(LED1_PIN,OUTPUT);
  pinMode(LED2_PIN,OUTPUT);
  pinMode(LED3_PIN,OUTPUT);
  pinMode(LED4_PIN,OUTPUT);
  pinMode(TE_CS_PIN, OUTPUT);
  pinMode(VBATT, INPUT);
  pinMode(EMATCH, OUTPUT);

  Serial.begin(9600);
  delay(1000);

  // LoRa Init
  if (!rf95.init())
    Serial.println("init failed");
  rf95.setTxPower(12, false);
  rf95.setFrequency(LORA_FREQ);
  rf95.setSpreadingFactor(LORA_SF);
  
  // Sensor init
  MPU_init();
  BMP_init();
 
  digitalWrite(STATUS_LED_PIN, HIGH);
  digitalWrite(EMATCH, LOW);

  accel_calib.x = 0;
  accel_calib.y = 0;
  accel_calib.z = 0;

  for (int i = 0; i < 20; i++) {
    mpu.getEvent(&a, &g, &temp);
    accel_calib.x += a.acceleration.x/20;
    accel_calib.y += (a.acceleration.y-10)/20;
    accel_calib.z += a.acceleration.z/20;
    delay(100);
  }
  delay(2000);
  //Serial.println("Initialization Complete.");
  //Serial.println("SanPellegrionique Ready for flight.");
}

// ======================================================== Loop ========================================================
void loop() {
  timestamp = millis();
  // Data Acquisition Handle
  MPU_read();
  BMP_read();
  battery_level_read();

  // State Machine Handle
  state_update();

  // LoRa Handle
  //lora_packet_build();
  //lora_tx_handle();
  //lora_rx_handle();

}

// ======================================================== Functions ========================================================

void MPU_init(void) {
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void MPU_read(void) {
  mpu.getEvent(&a, &g, &temp);

  accel_data.x = a.acceleration.x - accel_calib.x;
  accel_data.y = a.acceleration.y - accel_calib.y;
  accel_data.z = a.acceleration.z - accel_calib.z;

  /*Serial.print("Acceleration X: ");
  Serial.print(accel_data.x);
  Serial.print(", Y: ");
  Serial.print(accel_data.y);
  Serial.print(", Z: ");
  Serial.print(accel_data.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(gyro_data.x);
  Serial.print(", Y: ");
  Serial.print(gyro_data.y);
  Serial.print(", Z: ");
  Serial.print(gyro_data.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp_data.mpu);
  Serial.println(" degC");

  Serial.println("");*/
}

void BMP_init(void) {
  bmp.begin(0x76);
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X8,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */
}

void BMP_read(void) {

  pressure = bmp.readPressure();
  //est_altitude = bmp.readAltitude(ground_pressure) - ground_altitude;
  est_altitude = bmp.readAltitude(ground_pressure/100) - ground_altitude;

  /*Serial.print(F("Temperature = "));
  Serial.print(bmp.readTemperature());
  Serial.println(" °C");
  Serial.print(F("Pressure = "));
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");
  Serial.print(F("Approx altitude = "));
  Serial.print(bmp.readAltitude(ground_pressure) - ground_altitude);
  Serial.println(" m");
  Serial.println("");*/
}

void lora_packet_build(void) {

  uint8_t temp_buffer[sizeof(float)];

  // Timestamp
  memcpy(temp_buffer, &(timestamp), sizeof(float));
  lora_packet[0] = temp_buffer[0];
  lora_packet[1] = temp_buffer[1];
  lora_packet[2] = temp_buffer[2];
  lora_packet[3] = temp_buffer[3];

  // IMU Data
  memcpy(temp_buffer, &(accel_data.x), sizeof(float));
  lora_packet[4] = temp_buffer[0];
  lora_packet[5] = temp_buffer[1];
  lora_packet[6] = temp_buffer[2];
  lora_packet[7] = temp_buffer[3];
  memcpy(temp_buffer, &(accel_data.y), sizeof(float));
  lora_packet[8] = temp_buffer[0];
  lora_packet[9] = temp_buffer[1];
  lora_packet[10] = temp_buffer[2];
  lora_packet[11] = temp_buffer[3];
  memcpy(temp_buffer, &(accel_data.z), sizeof(float));
  lora_packet[12] = temp_buffer[0];
  lora_packet[13] = temp_buffer[1];
  lora_packet[14] = temp_buffer[2];
  lora_packet[15] = temp_buffer[3];

  memcpy(temp_buffer, &pressure, sizeof(float));
  lora_packet[16] = temp_buffer[0];
  lora_packet[17] = temp_buffer[1];
  lora_packet[18] = temp_buffer[2];
  lora_packet[19] = temp_buffer[3];
  memcpy(temp_buffer, &est_altitude, sizeof(float));
  lora_packet[20] = temp_buffer[0];
  lora_packet[21] = temp_buffer[1];
  lora_packet[22] = temp_buffer[2];
  lora_packet[23] = temp_buffer[3];

  // Battery Levels
  memcpy(temp_buffer, &v_batt, sizeof(float));
  lora_packet[24] = temp_buffer[0];
  lora_packet[25] = temp_buffer[1];
  lora_packet[26] = temp_buffer[2];
  lora_packet[27] = temp_buffer[3];

  // Status
  lora_packet[28] = current_state;

  memcpy(temp_buffer, &(lora_rssi), sizeof(float));
  lora_packet[29] = temp_buffer[0];
  lora_packet[30] = temp_buffer[1];
  lora_packet[31] = temp_buffer[2];
  lora_packet[32] = temp_buffer[3];
}

void lora_tx_handle(){
  //Serial.println("Sending to Ground Station");
  // Send a message to rf95_server
  digitalWrite(LED2_PIN, HIGH);
  rf95.send(lora_packet, sizeof(lora_packet));
  //rf95.waitPacketSent();
  digitalWrite(LED2_PIN, LOW);
  lora_rssi = rf95.lastRssi();
  delay(100);
}

// void lora_rx_handle(){
//   if (rf95.available()){
//     // Should be a message for us now   
//     uint8_t buf[LORA_PACKET_SIZE];
//     uint8_t len = sizeof(buf);
//     if (rf95.recv(buf, &len))
//     {
//       digitalWrite(LED1_PIN, HIGH);
// //      RH_RF95::printBuffer("request: ", buf, len);
//       Serial.print("got request: ");
//       Serial.println((char*)buf);
//       Serial.print("RSSI: ");
//       Serial.println(rf95.lastRssi(), DEC);
      
//       lora_parse(buf);

//       // Send a reply
//       uint8_t data[] = "ACK";
//       rf95.send(data, sizeof(data));
//       rf95.waitPacketSent();
//       Serial.println("Sent ACK");
//       digitalWrite(LED1_PIN, LOW);
//     }
//     else
//     {
//       Serial.println("recv failed");
//     }
//   }
// }

// void lora_parse(uint8_t *buffer){
//   switch(buffer[0]){
//     case LORA_ARM:
//       current_state = ARMED;
//       break;
    
//     case LORA_RESET:
//       break;

//     default:
//       break;
//   }
// }

void state_update(void){
  //Serial.print("Current State: ");
  Serial.println(current_state);
  switch(current_state){
    case IDLE:
      if(first_execute){
        float sum_press = 0;
        float sum_alti = 0;
        for(int j = 0; j<10; j++){ //Average the 10 first measurements
          ground_pressure = bmp.readPressure();
          //Serial.println(ground_pressure);
          ground_altitude = bmp.readAltitude(ground_pressure/100);
          sum_press += ground_pressure;
          sum_alti += ground_altitude;
        }
        ground_pressure = sum_press/10;
        ground_altitude = sum_alti/10;
        
        first_execute = false;
      }
      // if Telemetry command or vertical for a little while -> current_state = ARMED;
      // Liftoff Detection
      if(liftoff_detect()){
        current_state = P_ASCENT;
        first_execute = true;
        timer_re_event_init = millis();
      }
      break;
    case P_ASCENT:
      if(first_execute){
          timer_burnout_init = millis();
          first_execute = false;
          digitalWrite(LED3_PIN, HIGH);
      }
      if((millis() - timer_burnout_init) > MOTOR_BURNOUT){
        current_state = B_ASCENT;
        first_execute = true;
      }
      //if apogee detection or burn timer -> current_state = B_ASCENT
      break;
    case B_ASCENT:
      if(first_execute){
          first_execute = false;
          digitalWrite(LED3_PIN, LOW);
          digitalWrite(LED4_PIN, HIGH);
      }
      //apogee detection or safety timer -> current_state = EVENT_1
      if(apogee_detect()){
        current_state = APOGEE;
        first_execute = true;
      }
      else if((millis() - timer_re_event_init) > EVENT_1_TIMER){
        current_state = APOGEE;
        first_execute = true;
      }
      break;
    case APOGEE:
      if(first_execute){
        //ematch_trigger(EMATCH);
        first_execute = false;
      }
      //no movement for a bit -> current_state = TOUCHDOWN
      if(touchdown_detect()){
        current_state = TOUCHDOWN;
        first_execute = true;
      }
      break;
    case TOUCHDOWN:
      if(first_execute){
        digitalWrite(LED4_PIN, LOW);
        digitalWrite(EMATCH, LOW);
        first_execute = false;
      }
      break;
  }
}

uint8_t liftoff_detect(void){

  if((abs(accel_data.y) > LIFTOFF_THRESHOLD) && liftoff_delay == false){
  liftoff_timer = millis();
  liftoff_delay = true;
  }
  if((millis() - liftoff_timer > LIFTOFF_DELAY) && (abs(accel_data.y) > LIFTOFF_THRESHOLD) && (liftoff_delay = true)){
    liftoff_status = true;
  }
  else{
    liftoff_status = false;
  }
  return liftoff_status;
}

float alti_filter(void){
  float mean;
  BMP_read();
  total -= readings[avgIndex]; //Substract value numReadings times ago (here 20)
  readings[avgIndex] = est_altitude; //Multiply by 10 to remove float
  total += readings[avgIndex]; //Add new value;
  avgIndex++;
  if(avgIndex>=NUM_READINGS){
    avgIndex = 0;
  }
  mean = total/NUM_READINGS;
  return mean;
}

uint8_t apogee_detect(void){
  //Either certain threshold from max altitude or slope of altitude decreases. Doing threshold for now, slope if doesn't work
  mean_alti = alti_filter();
  //Threshold:
  if(mean_alti > max_alti){ //Update max alti
    max_alti = mean_alti;
    apogee_status = false;
  }
  else if(max_alti - mean_alti > APOGEE_THRESHOLD){
    apogee_status = true;
    //Serial.println("Threshold passed");
  }
  /*Serial.print("Estimated altitude: ");
  Serial.println(est_altitude);
  Serial.print("Averaged altitude: ");
  Serial.println(mean_alti);
  Serial.print("Max alti: ");
  Serial.print(max_alti);
  Serial.println("\n");*/
  return apogee_status;
}

uint8_t touchdown_detect(void){
  if((abs(est_altitude) < GROUND_ALT_THRESHOLD) && (touchdown_delay == false)){
  touchdown_timer = millis();
  touchdown_delay = true;
  }
  if((millis() - touchdown_timer > TOUCHDOWN_DELAY) && (abs(est_altitude) < GROUND_ALT_THRESHOLD) && (touchdown_delay = true)){
    touchdown_status = true;
  }
  else{
    touchdown_status = false;
  }
  return touchdown_status;
}

void ematch_trigger(uint8_t ematch){
  digitalWrite(ematch, HIGH);
}

void battery_level_read(void){
  v_batt = analogRead(VBATT)*88.0/68.0*3.3/1024;
  //Serial.print("Vbatt: ");
  //Serial.println(v_batt);
}
