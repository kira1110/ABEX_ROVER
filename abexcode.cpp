#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int8.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_BME280.h>
#include <Arduino.h>

// NPK Sensor Pins 
#define RXD2 16
#define TXD2 17
#define RS485_EN 4   // DE + RE pin for RS485

// Modbus request for 7 parameters
unsigned char byteRequest[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x07, 0x04, 0x08};
unsigned char byteResponse[19] = {};

// LCD I2C
#define LCD_I2C_ADDRESS 0x27
#define LCD_COLS 16
#define LCD_ROWS 2
LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, LCD_COLS, LCD_ROWS);

// led
#define led_pin 26
int count=0;

// Stepper Motor Pins 
#define STEP_PIN 18
#define DIR_PIN  19
#define STEPS_FOR_5_DEG 20   // calculated for 1.8° motor @ 1/16 microstep

// Sensors 
#define MQ135_PIN 34
uint8_t sensor_status = 0;   // bit0: MQ135, bit1: TCS34725, bit2: BME280

// Adafruit_TCS34725 tcs = Adafruit_TCS34725(
//   TCS34725_INTEGRATIONTIME_50MS,
//   TCS34725_GAIN_4X
// );

Adafruit_BME280 bme;
bool tcs_ok = false;
bool bme_ok = false;

// micro-ROS
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_timer_t timer;
rclc_executor_t executor;

std_msgs__msg__Float32MultiArray sensor_msg;
std_msgs__msg__Int8 stepper_msg;

// 14 values: MQ135, RGB, BME temp/hum/pres, NPK moisture/temp/cond/pH/N/P/K
float data_buffer[14];

// Stepper Motor Function 
void stepMotor(int steps, bool direction)
{
  digitalWrite(DIR_PIN, direction);

  for (int i = 0; i < steps; i++)
  {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(600);   // speed control
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(600);
  }
}

// Stepper Subscription Callback 
void subscription_callback(const void *msgin)
{
  const std_msgs__msg__Int8 *cmd = (const std_msgs__msg__Int8 *)msgin;

  if (cmd->data == 1)
  {
    stepMotor(STEPS_FOR_5_DEG, HIGH);   // +5°
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Stepper: +5 deg");
    delay(10);
  }
  else if (cmd->data == -1)
  {
    stepMotor(STEPS_FOR_5_DEG, LOW);    // -5°
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Stepper: -5 deg");
    delay(10);
  }
  else if (cmd->data == 10)
  {
   count++;
  }
}

// Timer Callback (Sensor Reading) 
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void) last_call_time;
  if (timer == NULL) return;

  // MQ135 
  int mq135_value = analogRead(MQ135_PIN);

  // TCS34725 
  // float r = 0, g = 0, b = 0;
  // if (tcs_ok) {
  //   tcs.getRGB(&r, &g, &b);
  // }

  // BME280 
  float temperature = bme_ok ? bme.readTemperature() : 0.0;
  float humidity = bme_ok ? bme.readHumidity() : 0.0;
  float pressure = bme_ok ? bme.readPressure() / 100.0F : 0.0;

  /* NPK Sensor Values */
  float soilHumidity = 0.0;
  float soilTemperature = 0.0;
  float soilConductivity = 0.0;
  float soilPH = 0.0;
  float nitrogen = 0.0;
  float phosphorus = 0.0;
  float potassium = 0.0;
  
  bool npk_sensor_responding = false;

  // SEND MODBUS REQUEST 
  digitalWrite(RS485_EN, HIGH);   // TX mode
  delay(2);
  Serial2.write(byteRequest, sizeof(byteRequest));
  Serial2.flush();
  digitalWrite(RS485_EN, LOW);    // RX mode
  delay(300);                     // Sensor response time

  // READ RESPONSE 
  if (Serial2.available() >= sizeof(byteResponse)) {
    Serial2.readBytes(byteResponse, sizeof(byteResponse));

    // Check if response is valid
    if (byteResponse[0] == 0x01) {  // Valid response from device
      soilHumidity = ((byteResponse[3] << 8) | byteResponse[4]) / 10.0;
      soilTemperature = ((byteResponse[5] << 8) | byteResponse[6]) / 10.0;
      soilConductivity = (byteResponse[7] << 8) | byteResponse[8];
      soilPH = ((byteResponse[9] << 8) | byteResponse[10]) / 10.0;
      nitrogen = (byteResponse[11] << 8) | byteResponse[12];
      phosphorus = (byteResponse[13] << 8) | byteResponse[14];
      potassium = (byteResponse[15] << 8) | byteResponse[16];
      
      npk_sensor_responding = true;
    }
  }

  // Fill message buffer - 14 values (all NPK data published) 
  data_buffer[0] = mq135_value;
  // data_buffer[1] = r;
  // data_buffer[2] = g;
  // data_buffer[3] = b;
  data_buffer[1] = temperature;
  data_buffer[2] = humidity;
  data_buffer[3] = pressure;
  data_buffer[4] = soilHumidity;
  data_buffer[5] = soilTemperature;
  data_buffer[6] = soilConductivity;
  data_buffer[7] = soilPH;
  data_buffer[8] = nitrogen;
  data_buffer[9] = phosphorus;
  data_buffer[10] = potassium;

  sensor_msg.data.data = data_buffer;
  sensor_msg.data.size = 11;
  sensor_msg.data.capacity = 11;

  rcl_publish(&publisher, &sensor_msg, NULL);

 // LCD DISPLAY 
  lcd.clear();
  
  // First line: Sensors status
  lcd.setCursor(0, 0);
  lcd.print("MQ BME LBRT");
  
  // Second line: Status indicators (0 or 1) including NPK status
  lcd.setCursor(0, 1);
  // MQ135 status (bit 0)
  lcd.print((mq135_value > 50) ? "1" : "0");
  // lcd.setCursor(3, 1);
  // // TCS status (bit 1)
  // lcd.print(tcs_ok ? "1" : "0");
  lcd.setCursor(3, 1);
  // BME status (bit 2)
  lcd.print(bme_ok ? "1" : "0");
  lcd.setCursor(7, 1);
  // NPK/Labrat status indicator - just shows if sensor is responding
  lcd.print(npk_sensor_responding ? "1" : "0");
}

// SETUP 
void setup()
{
  Serial.begin(115200);
   pinMode(led_pin,OUTPUT);
  digitalWrite(led_pin,LOW);
  // Initialize NPK sensor serial
  pinMode(RS485_EN, OUTPUT);
  digitalWrite(RS485_EN, LOW); // Receive mode
  Serial2.begin(4800, SERIAL_8N1, RXD2, TXD2);

  // Initialize Stepper Motor pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  Wire.begin(21, 22);

  // Initialize I2C LCD
  lcd.init();
  lcd.backlight();  // Turn on backlight
  lcd.print("Starting...");

  /* Sensor Init */
  // tcs_ok = tcs.begin();
  bme_ok = bme.begin(0x77);

  lcd.clear();
  lcd.print("Sensors init...");

  /* micro-ROS */
  set_microros_transports();
  
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

  // Create a single node for both publisher and subscriber
  rclc_node_init_default(
    &node,
    "esp32_combined_node",
    "",
    &support
  );

  // Initialize publisher for sensor data
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "sensor_data_live"
  );

  // Initialize subscriber for stepper commands
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
    "stepper_cmd"
  );

  // Initialize timer for sensor readings
  rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(1000), // 2 second interval for NPK sensor
    timer_callback
  );

  // Initialize executor with both timer and subscription
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_timer(&executor, &timer);
  rclc_executor_add_subscription(
    &executor,
    &subscriber,
    &stepper_msg,
    &subscription_callback,
    ON_NEW_DATA
  );

  lcd.clear();
  lcd.print("micro-ROS OK");
  delay(500);
}

// LOOP 
void loop()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
if(count%2 == 0)
{
  digitalWrite(led_pin,LOW);
}
else if(count%2 != 0)
{
  digitalWrite(led_pin,HIGH);
}
}