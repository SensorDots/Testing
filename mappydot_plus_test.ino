/**
   Basic MappyDot Plus Test Application

   Copyright (C) 2018  Blair Wyatt

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

/* Basics */
#define READ_DISTANCE                               (0x72)
#define PERFORM_SINGLE_RANGE                        (0x53)
#define READ_ACCURACY                               (0x52)
#define READ_ERROR_CODE                             (0x45)
#define RANGING_MEASUREMENT_MODE                    (0x6d)
#define MEASUREMENT_BUDGET                          (0x42)
#define SET_CONTINUOUS_RANGING_MODE                 (0x63)
#define SET_SINGLE_RANGING_MODE                     (0x73)
#define CHECK_INTERRUPT                             (0x49)
#define SIGMA_LIMIT_CHECK_VALUE                     (0x4C)
#define SIGNAL_LIMIT_CHECK_VALUE                    (0x47)

/* Configuration */
#define FILTERING_ENABLE                            (0x46)
#define FILTERING_DISABLE                           (0x66)
#define AVERAGING_ENABLE                            (0x56)
#define AVERAGING_DISABLE                           (0x76)
#define AVERAGING_SAMPLES                           (0x69)
#define SET_LED_MODE                                (0x6c)
#define SET_LED_THRESHOLD_DISTANCE_IN_MM            (0x65)
#define SET_GPIO_MODE                               (0x67)
#define SET_GPIO_THRESHOLD_DISTANCE_IN_MM           (0x6f)
#define CALIBRATE_DISTANCE_OFFSET                   (0x61)
#define CALIBRATE_CROSSTALK                         (0x78)
#define CALIBRATE_SPAD                              (0x75)
#define TEMPERATURE_CALIBRATION                     (0x55)
#define INTERSENSOR_CROSSTALK_REDUCTION_ENABLE      (0x54)
#define INTERSENSOR_CROSSTALK_REDUCTION_DISABLE     (0x74)
#define INTERSENSOR_CROSSTALK_TIMEOUT               (0x71)
#define INTERSENSOR_CROSSTALK_MEASUREMENT_DELAY     (0x51)
#define REGION_OF_INTEREST                          (0x70)

/* Settings */
#define FIRMWARE_VERSION                            (0x4e)
#define NAME_DEVICE                                 (0x6e)
#define DEVICE_NAME                                 (0x64)
#define READ_CURRENT_SETTINGS                       (0x62)
#define RESTORE_FACTORY_DEFAULTS                    (0x7a)
#define WRITE_CURRENT_SETTINGS_AS_START_UP_DEFAULT  (0x77)

/* Advanced */
#define RESET_VL53L1X_RANGING                       (0x58)
#define VL53L1X_NOT_SHUTDOWN                        (0x48)
#define VL53L1X_SHUTDOWN                            (0x68)
#define READ_NONFILTERED_VALUE                      (0x6a)
#define AMBIENT_RATE_RETURN                         (0x41)
#define SIGNAL_RATE_RETURN                          (0x4A)

/* Super Advanced */
#define ENTER_FACTORY_MODE                          (0x23) //"#"//"!#!#!#"

/* Ranging Modes */
#define SHORT_RANGE                                 (0x73)
#define MED_RANGE                                   (0x6d)
#define LONG_RANGE                                  (0x6c)

/* LED Modes */
#define LED_ON                                      (0x6f)
#define LED_OFF                                     (0x66)
#define LED_THRESHOLD_ENABLED                       (0x74)
#define LED_PWM_ENABLED                             (0x70)
#define LED_MEASUREMENT_OUTPUT                      (0x6d)

/* GPIO Modes */
#define GPIO_HIGH                                   (0x6f)
#define GPIO_LOW                                    (0x66)
#define GPIO_THRESHOLD_ENABLED                      (0x74)
#define GPIO_PWM_ENABLED                            (0x70)
#define GPIO_MEASUREMENT_INTERRUPT                  (0x6d)

#include <i2c_t3.h> //Teensy 3 I2C library.

#define BUFFER_SIZE 30

uint8_t data_buffer[BUFFER_SIZE];
uint8_t address = 0;
const int ledPin = 13;
uint16_t distance;
uint8_t buff;
uint8_t toggle = 0;

void device_name_write() {
  /* Test set device name write */
  Wire.beginTransmission(address);
  Wire.write(NAME_DEVICE);
  Wire.write("A");
  Wire.write("B");
  Wire.write("C");
  Wire.write("D");
  Wire.write("E");
  Wire.write("F");
  Wire.write("G");
  Wire.write("H");
  Wire.write("I");
  Wire.write("J");
  Wire.write("K");
  Wire.write("L");
  Wire.write("M");
  Wire.write("N");
  Wire.write("O");
  Wire.write("P");
  Wire.endTransmission();
}

/*
  #define CALIBRATE_DISTANCE_OFFSET                   (0x61)
  #define CALIBRATE_CROSSTALK                         (0x78)
  #define CALIBRATE_SPAD                              (0x75)
  #define TEMPERATURE_CALIBRATION                     (0x55)
  #define INTERSENSOR_CROSSTALK_TIMEOUT               (0x71)
  #define INTERSENSOR_CROSSTALK_MEASUREMENT_DELAY     (0x51)
*/

void measurement_mode_short()
{
  Wire.beginTransmission(address);
  Wire.write(RANGING_MEASUREMENT_MODE);
  Wire.write(SHORT_RANGE);
  Wire.endTransmission();
}

void measurement_mode_med()
{
  Wire.beginTransmission(address);
  Wire.write(RANGING_MEASUREMENT_MODE);
  Wire.write(MED_RANGE);
  Wire.endTransmission();
}


void measurement_mode_long()
{
  Wire.beginTransmission(address);
  Wire.write(RANGING_MEASUREMENT_MODE);
  Wire.write(LONG_RANGE);
  Wire.endTransmission();
}

void addr_in_test()
{
  Wire.beginTransmission(address);
  Wire.write(0x2e);
  Wire.endTransmission();
}

void intersensor_crosstalk_enable()
{
  Wire.beginTransmission(address);
  Wire.write(INTERSENSOR_CROSSTALK_REDUCTION_ENABLE);
  Wire.endTransmission();
}

void intersensor_crosstalk_disable()
{
  Wire.beginTransmission(address);
  Wire.write(INTERSENSOR_CROSSTALK_REDUCTION_DISABLE);
  Wire.endTransmission();

}

void reset_VL53L1X()
{
  Wire.beginTransmission(address);
  Wire.write(RESET_VL53L1X_RANGING);
  Wire.endTransmission();
}

void shutdown_VL53L1X()
{
  Wire.beginTransmission(address);
  Wire.write(VL53L1X_SHUTDOWN);
  Wire.endTransmission();
}

void startup_VL53L1X()
{
  Wire.beginTransmission(address);
  Wire.write(VL53L1X_NOT_SHUTDOWN);
  Wire.endTransmission();
}

void averaging_samples_4()
{
  Wire.beginTransmission(address);
  Wire.write(AVERAGING_SAMPLES);
  Wire.write(4);
  Wire.endTransmission();
}

void averaging_samples_10()
{
  Wire.beginTransmission(address);
  Wire.write(AVERAGING_SAMPLES);
  Wire.write(10);
  Wire.endTransmission();
}
void filtering_enable()
{
  Wire.beginTransmission(address);
  Wire.write(FILTERING_ENABLE);
  Wire.endTransmission();
}

void filtering_disable()
{
  Wire.beginTransmission(address);
  Wire.write(FILTERING_DISABLE);
  Wire.endTransmission();
}

void averaging_enable()
{
  Wire.beginTransmission(address);
  Wire.write(AVERAGING_ENABLE);
  Wire.endTransmission();
}

void averaging_disable()
{
  Wire.beginTransmission(address);
  Wire.write(AVERAGING_DISABLE);
  Wire.endTransmission();
}

void perform_single_range()
{
  Wire.beginTransmission(address);
  Wire.write(PERFORM_SINGLE_RANGE);
  Wire.endTransmission();
}

void set_continuous()
{
  Wire.beginTransmission(address);
  Wire.write(SET_CONTINUOUS_RANGING_MODE);
  Wire.endTransmission();
}

void set_single()
{
  Wire.beginTransmission(address);
  Wire.write(SET_SINGLE_RANGING_MODE);
  Wire.endTransmission();
}

void read_factory_defaults()
{
  Wire.beginTransmission(address);
  Wire.write(RESTORE_FACTORY_DEFAULTS);
  Wire.endTransmission();
}

void write_as_startup_settings()
{

  Wire.beginTransmission(address);
  Wire.write(WRITE_CURRENT_SETTINGS_AS_START_UP_DEFAULT);
  Wire.endTransmission();
}


void mm_to_bytes(uint8_t *bytes, uint16_t mm)
{
  bytes[0] = (mm >> 8 & 0xFF);
  bytes[1] = (mm & 0xFF);
}

void enter_factory()
{
#define ENTER_FACTORY_MODE                          (0x23) //"#"//"!#!#!#"
  Wire.beginTransmission(address);
  Wire.write(ENTER_FACTORY_MODE);
  Wire.write('!');
  Wire.write('#');
  Wire.write('!');
  Wire.write('#');
  Wire.write('!');
  Wire.write('#');
  Wire.endTransmission();
}

void set_gpio_threshold_100()
{
  uint8_t bytes[2];
  mm_to_bytes(bytes, 100);
  Wire.beginTransmission(address);
  Wire.write(SET_GPIO_THRESHOLD_DISTANCE_IN_MM);
  Wire.write(bytes[0]);
  Wire.write(bytes[1]);
  Wire.endTransmission();
}
void set_gpio_threshold_300()
{
  uint8_t bytes[2];
  mm_to_bytes(bytes, 300);
  Wire.beginTransmission(address);
  Wire.write(SET_GPIO_THRESHOLD_DISTANCE_IN_MM);
  Wire.write(bytes[0]);
  Wire.write(bytes[1]);
  Wire.endTransmission();
}
void set_led_threshold_100()
{
  uint8_t bytes[2];
  mm_to_bytes(bytes, 100);
  Wire.beginTransmission(address);
  Wire.write(SET_LED_THRESHOLD_DISTANCE_IN_MM);
  Wire.write(bytes[0]);
  Wire.write(bytes[1]);
  Wire.endTransmission();
}
void set_led_threshold_300()
{
  uint8_t bytes[2];
  mm_to_bytes(bytes, 300);
  Wire.beginTransmission(address);
  Wire.write(SET_LED_THRESHOLD_DISTANCE_IN_MM);
  Wire.write(bytes[0]);
  Wire.write(bytes[1]);
  Wire.endTransmission();
}
void set_led_mode_threshold()
{

  Wire.beginTransmission(address);
  Wire.write(SET_LED_MODE);
  Wire.write(LED_THRESHOLD_ENABLED);
  Wire.endTransmission();
}

void set_led_mode_on()
{

  Wire.beginTransmission(address);
  Wire.write(SET_LED_MODE);
  Wire.write(LED_ON);
  Wire.endTransmission();
}
void set_led_mode_off()
{

  Wire.beginTransmission(address);
  Wire.write(SET_LED_MODE);
  Wire.write(LED_OFF);
  Wire.endTransmission();
}
void set_led_mode_pwm()
{

  Wire.beginTransmission(address);
  Wire.write(SET_LED_MODE);
  Wire.write(LED_PWM_ENABLED);
  Wire.endTransmission();
}

void set_led_mode_measurement()
{

  Wire.beginTransmission(address);
  Wire.write(SET_LED_MODE);
  Wire.write(LED_MEASUREMENT_OUTPUT);
  Wire.endTransmission();
}

void set_gpio_mode_threshold()
{

  Wire.beginTransmission(address);
  Wire.write(SET_GPIO_MODE);
  Wire.write(GPIO_THRESHOLD_ENABLED);
  Wire.endTransmission();
}

void set_gpio_mode_on()
{

  Wire.beginTransmission(address);
  Wire.write(SET_GPIO_MODE);
  Wire.write(GPIO_HIGH);
  Wire.endTransmission();
}
void set_gpio_mode_off()
{

  Wire.beginTransmission(address);
  Wire.write(SET_GPIO_MODE);
  Wire.write(GPIO_LOW);
  Wire.endTransmission();
}
void set_gpio_mode_pwm()
{

  Wire.beginTransmission(address);
  Wire.write(SET_GPIO_MODE);
  Wire.write(GPIO_PWM_ENABLED);
  Wire.endTransmission();
}

void set_fov_27()
{

  Wire.beginTransmission(address);
  Wire.write(REGION_OF_INTEREST);
  Wire.write((byte)0);
  Wire.write((byte)15);
  Wire.write((byte)15);
  Wire.write((byte)0);
  Wire.endTransmission();
}

void set_fov_16()
{

  Wire.beginTransmission(address);
  Wire.write(REGION_OF_INTEREST);
  Wire.write((byte)5);
  Wire.write((byte)10);
  Wire.write((byte)10);
  Wire.write((byte)5);
  Wire.endTransmission();
}

void set_gpio_mode_measurement()
{

  Wire.beginTransmission(address);
  Wire.write(SET_GPIO_MODE);
  Wire.write(GPIO_MEASUREMENT_INTERRUPT);
  Wire.endTransmission();
}

void simple_read() {
  /* Test simple read (after previous write too) */
  Wire.requestFrom(address, 2);
  distance = Wire.read() << 8; distance |= Wire.read();
  Serial.print("Simple: ");
  Serial.println(distance, DEC);
}

void read() {
  /* Test Read Distance */
  Wire.beginTransmission(address);
  Wire.write(READ_DISTANCE);
  Wire.endTransmission(false); //repeated start
  Wire.requestFrom(address, 2, true);
  distance = Wire.read() << 8; distance |= Wire.read();
  Serial.print("Read: ");
  Serial.println(distance, DEC);
}

void ambient_rate_return() {
  /* Test Read Distance */
  Wire.beginTransmission(address);
  Wire.write(AMBIENT_RATE_RETURN);
  Wire.endTransmission(false); //repeated start
  Wire.requestFrom(address, 2, true);
  uint16_t rate;
  rate = Wire.read() << 8; rate |= Wire.read();
  Serial.println(rate, DEC);
}

void signal_rate_return() {
  /* Test Read Distance */
  Wire.beginTransmission(address);
  Wire.write(SIGNAL_RATE_RETURN);
  Wire.endTransmission(false); //repeated start
  Wire.requestFrom(address, 2, true);
  uint16_t rate;
  rate = Wire.read() << 8; rate |= Wire.read();
  Serial.println(rate, DEC);
}

void interrupt_read() {

  Wire.beginTransmission(address);
  Wire.write(CHECK_INTERRUPT);
  Wire.endTransmission(false); //repeated start
  Wire.requestFrom(address, 1, true);
  Serial.println(Wire.read(), DEC);
}
void test_no_restart_read() {
  Wire.beginTransmission(address);
  Wire.write(READ_DISTANCE);
  Wire.endTransmission(); //repeated start
  Wire.requestFrom(address, 2);
  distance = Wire.read() << 8; distance |= Wire.read();
  Serial.print("Read(NRS): ");
  Serial.println(distance, DEC);
}
void test_non_filtered_distance() {
  /* Test Non Filtered Distance */
  Wire.beginTransmission(address);
  Wire.write(READ_NONFILTERED_VALUE);
  Wire.endTransmission(false); //repeated start
  Wire.requestFrom(address, 3, true);
  distance = Wire.read() << 8; distance |= Wire.read();
  Wire.read();
  Serial.print("NonFilt: ");
  Serial.println(distance, DEC);
}
void too_many_bytes() {
  /* Test Too Many Bytes */
  Wire.beginTransmission(address);
  Wire.write(READ_DISTANCE);
  Wire.endTransmission(false); //repeated start
  Wire.requestFrom(address, 3, true);
  Serial.print("TooManyBytes: ");
  Serial.print(Wire.read(), HEX);
  Serial.print(",");
  Serial.print(Wire.read(), HEX);
  Serial.print(",");
  Serial.println(Wire.read(), HEX);
}
void test_error_code() {
  /* Test Error Code */
  Wire.beginTransmission(address);
  Wire.write(READ_ERROR_CODE);
  Wire.endTransmission(false); //repeated start
  Wire.requestFrom(address, 1, true);
  buff = Wire.read();
  Serial.print("ErrorCode: ");
  Serial.println(buff, DEC);
}
void test_distance_accuracy() {
  /* Test Distance Accuracy */
  Wire.beginTransmission(address);
  Wire.write(READ_ACCURACY);
  Wire.endTransmission(false); //repeated start
  Wire.requestFrom(address, 2, true);
  distance = Wire.read() << 8 | Wire.read();
  Serial.print("Accuracy: ");
  Serial.println(distance, DEC);
}
void test_firmware_version() {
  /* Test Firmware Version */
  Wire.beginTransmission(address);
  Wire.write(FIRMWARE_VERSION);
  Wire.endTransmission(false); //repeated start
  Wire.requestFrom(address, 10, true);
  Serial.print("Version: ");
  Serial.write(Wire.read());
  Serial.write(Wire.read());
  Serial.write(Wire.read());
  Serial.write(Wire.read());
  Serial.write(Wire.read());
  Serial.write(Wire.read());
  Serial.write(Wire.read());
  Serial.write(Wire.read());
  Serial.write(Wire.read());
  Serial.write(Wire.read());
  Serial.println("");
}
void test_device_name() {
  /* Test Device Name */
  Wire.beginTransmission(address);
  Wire.write(DEVICE_NAME);
  Wire.endTransmission(false); //repeated start
  Wire.requestFrom(address, 16, true);
  Serial.print("Name: ");
  int i = 0;
  while (i < 16) {
    Serial.write(Wire.read());
    i++;
  }

  Serial.println("");
}
void test_settings_read() {
  /* Test Settings Read */
  Wire.beginTransmission(address);
  Wire.write(READ_CURRENT_SETTINGS);
  Wire.endTransmission(false); //repeated start
  Wire.requestFrom(address, 23, true);
  Serial.print("Settings: ");
  int i = 0;
  while (i < 22) {
    Serial.print(Wire.read(), HEX);
    Serial.print(",");
    i++;
  }

  Serial.println(Wire.read(), HEX);
}

void print_menu()
{
  Serial.println("n - device_name_write");
  Serial.println("s - simple_read");
  Serial.println("r - read");
  Serial.println("N - no_restart_read");
  Serial.println("f - test_non_filtered_distance");
  Serial.println("B - too_many_bytes");
  Serial.println("e - error_code");
  Serial.println("a - distance_accuracy");
  Serial.println("v - firmware_version");
  Serial.println("d - device_name");
  Serial.println("S - settings_read");
  Serial.println("h - print_menu");
  Serial.println("1 - set_led_mode_threshold");
  Serial.println("2 - set_led_mode_on");
  Serial.println("3 - set_led_mode_off");
  Serial.println("4 - set_led_mode_pwm");
  Serial.println("5 - set_led_mode_measurement");
  Serial.println("6 - set_gpio_mode_threshold");
  Serial.println("7 - set_gpio_mode_on");
  Serial.println("8 - set_gpio_mode_off");
  Serial.println("9 - set_gpio_mode_pwm");
  Serial.println("0 - set_gpio_mode_measurement");
  Serial.println("! - set_led_threshold_300");
  Serial.println("@ - set_led_threshold_100");
  Serial.println("# - set_gpio_threshold_300");
  Serial.println("$ - set_gpio_threshold_100");
  Serial.println("R - read_factory_defaults");
  Serial.println("W - write_as_startup_settings");
  Serial.println("F - enter_factory");
  Serial.println("C - set_continuous");
  Serial.println("q - set_single");
  Serial.println("p - perform_single_range");
  Serial.println("D - measurement_mode_short");
  Serial.println("H - measurement_mode_med");
  Serial.println("L - measurement_mode_long");
  Serial.println("A - measurement_budget");
  Serial.println("I - intersensor_crosstalk_enable");
  Serial.println("i - intersensor_crosstalk_disable");
  Serial.println("V - reset_VL53L1X");
  Serial.println("g - shutdown_VL53L1X");
  Serial.println("G - startup_VL53L1X");
  Serial.println("M - averaging_samples_4");
  Serial.println("X - averaging_samples_10");
  Serial.println("O - filtering_enable");
  Serial.println("o - filtering_disable");
  Serial.println("Y - averaging_enable");
  Serial.println("y - averaging_disable");
  Serial.println("- - change_address");
  Serial.println("? - calibration_routine");
  Serial.println(". - addr_in_test");
  Serial.println("[ - set_fov_27");
  Serial.println("] - set_fov_16");
  Serial.println("t - interrupt_read");
  Serial.println("< - signal_limit");
  Serial.println("> - sigma_limit");
  Serial.println("{ - signal_rate_return");
  Serial.println("} - ambient_rate_return");

  Serial.print("Current address is: ");
  Serial.println(address, DEC);

  Serial.println("");

}

void calibration_routine()
{
  set_led_mode_off();

  Wire.beginTransmission(address);
  Wire.write(CALIBRATE_SPAD);
  Wire.endTransmission();

  delay(2000);

  uint8_t calib_dist = 103;
  uint8_t dist_bytes[2];
  mm_to_bytes(dist_bytes, calib_dist);
  Wire.beginTransmission(address);
  Wire.write(CALIBRATE_DISTANCE_OFFSET);
  Wire.write(dist_bytes[0]);
  Wire.write(dist_bytes[1]);
  Wire.endTransmission();
  delay(12000);
  set_led_mode_pwm();
  Serial.println("Calibration Complete");
}
void setup() {

  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_100);
  Serial.begin(115200);

  //Stop bus locking up when I2C glitches occur.
  Wire.setDefaultTimeout(10000);

  // initialize the LED pin as an output.
  pinMode(ledPin, OUTPUT);

  address = 0x08;
}

/*Wire.beginTransmission(address);
  Wire.write(READ_DISTANCE);
  Wire.endTransmission();
      delay(2);*/

void change_address()
{
  char char_buffer[4] = {0};
  uint8_t i = 0;
  uint8_t was_set = 0;

  Serial.print("Enter I2C address >= 8 and =< 112): ");
  while (i < 4) {
    while (Serial.available() == 0) {}
    char character = Serial.read();
    Serial.print(character);
    char_buffer[i] = character;
    i++;
    if (character == '\r' || character == '\n')
    {
      if (atoi(char_buffer) >= 8 && atoi(char_buffer) <= 112)
      {
        address = atoi(char_buffer);
        was_set = 1;
      }
      i = 4;
    }
  }

  Serial.println("");

  if (was_set)
  {
    Serial.print("Address set to: ");
    Serial.println(address, DEC);
  } else {
    Serial.println("Address not set");
  }

}

void measurement_budget()
{
  char char_buffer[5] = {0};
  uint8_t i = 0;
  uint8_t was_set = 0;
  uint16_t budget = 0;

  Serial.print("Enter measurement budget (ms) >= 20 and <= 1000): ");
  while (i < 5) {
    while (Serial.available() == 0) {}
    char character = Serial.read();
    Serial.print(character);
    char_buffer[i] = character;
    i++;
    if (character == '\r' || character == '\n')
    {
      if (atol(char_buffer) >= 10 && atol(char_buffer) <= 1000)
      {
        budget = atoi(char_buffer);
        was_set = 1;
      }
      i = 5;
    }
  }

  Serial.println("");

  if (was_set)
  {
    Serial.print("Budget set to: ");
    Serial.println(budget, DEC);
    uint8_t bytes[2];
    mm_to_bytes(bytes, budget); //Uses the same byte format as this function
    Wire.beginTransmission(address);
    Wire.write(MEASUREMENT_BUDGET);
    Wire.write(bytes[0]);
    Wire.write(bytes[1]);
    Wire.endTransmission();
  } else {
    Serial.println("Budget not set");
  }

}

void sigma_limit()
{
  char char_buffer[4] = {0};
  uint8_t i = 0;
  uint8_t was_set = 0;
  uint16_t budget = 0;

  Serial.print("Enter sigma limit (mm) >= 1 and <= 254): ");
  while (i < 4) {
    while (Serial.available() == 0) {}
    char character = Serial.read();
    Serial.print(character);
    char_buffer[i] = character;
    i++;
    if (character == '\r' || character == '\n')
    {
      if (atol(char_buffer) >= 1 && atol(char_buffer) <= 254)
      {
        budget = atoi(char_buffer);
        was_set = 1;
      }
      i = 4;
    }
  }

  Serial.println("");

  if (was_set)
  {
    Serial.print("Sigma set to: ");
    Serial.println(budget, DEC);
    uint8_t bytes[2];
    mm_to_bytes(bytes, budget); //Uses the same byte format as this function
    Wire.beginTransmission(address);
    Wire.write(SIGMA_LIMIT_CHECK_VALUE);
    Wire.write(bytes[0]);
    Wire.endTransmission();
  } else {
    Serial.println("Sigma not set");
  }

}


void signal_limit()
{
  char char_buffer[5] = {0};
  uint8_t i = 0;
  uint8_t was_set = 0;
  uint16_t budget = 0;

  Serial.print("Enter signal limit kcps >= 1 and <= 1000): ");
  while (i < 5) {
    while (Serial.available() == 0) {}
    char character = Serial.read();
    Serial.print(character);
    char_buffer[i] = character;
    i++;
    if (character == '\r' || character == '\n')
    {
      if (atol(char_buffer) >= 1 && atol(char_buffer) <= 1000)
      {
        budget = atoi(char_buffer);
        was_set = 1;
      }
      i = 5;
    }
  }

  Serial.println("");

  if (was_set)
  {
    Serial.print("Signal set to: ");
    Serial.println(budget, DEC);
    uint8_t bytes[2];
    mm_to_bytes(bytes, budget); //Uses the same byte format as this function
    Wire.beginTransmission(address);
    Wire.write(SIGNAL_LIMIT_CHECK_VALUE);
    Wire.write(bytes[0]);
    Wire.write(bytes[1]);
    Wire.endTransmission();
  } else {
    Serial.println("Signal not set");
  }

}

void loop() {

  while (!Serial) {
    // Wait for serial port
  }
  //digitalWrite(ledPin, HIGH);   // set the LED on
  while (Serial.available() == 0) {}
  switch (Serial.read())
  {
    case 'n': device_name_write(); break;
    case 's': simple_read(); break;
    case 'r': read(); break;
    case 'N': test_no_restart_read(); break;
    case 'f': test_non_filtered_distance(); break;
    case 'B': too_many_bytes(); break;
    case 'e': test_error_code(); break;
    case 'a': test_distance_accuracy(); break;
    case 'v': test_firmware_version(); break;
    case 'd': test_device_name(); break;
    case 'S': test_settings_read(); break;
    case 'h': print_menu(); break;
    case '1': set_led_mode_threshold(); break;
    case '2': set_led_mode_on(); break;
    case '3': set_led_mode_off(); break;
    case '4': set_led_mode_pwm(); break;
    case '5': set_led_mode_measurement(); break;
    case '6': set_gpio_mode_threshold(); break;
    case '7': set_gpio_mode_on(); break;
    case '8': set_gpio_mode_off(); break;
    case '9': set_gpio_mode_pwm(); break;
    case '0': set_gpio_mode_measurement(); break;
    case '!': set_led_threshold_300(); break;
    case '@': set_led_threshold_100(); break;
    case '#': set_gpio_threshold_300(); break;
    case '$': set_gpio_threshold_100(); break;
    case 'R': read_factory_defaults(); break;
    case 'W': write_as_startup_settings(); break;
    case 'F': enter_factory(); break;
    case 'C': set_continuous(); break;
    case 'q': set_single(); break;
    case 'p': perform_single_range(); break;
    case 'D': measurement_mode_short(); break;
    case 'H': measurement_mode_med(); break;
    case 'L': measurement_mode_long(); break;
    case 'A': measurement_budget(); break;
    case 'I': intersensor_crosstalk_enable(); break;
    case 'i': intersensor_crosstalk_disable(); break;
    case 'V': reset_VL53L1X(); break;
    case 'g': shutdown_VL53L1X(); break;
    case 'G': startup_VL53L1X(); break;
    case 'M': averaging_samples_4(); break;
    case 'X': averaging_samples_10(); break;
    case 'O': filtering_enable(); break;
    case 'o': filtering_disable(); break;
    case 'Y': averaging_enable(); break;
    case 'y': averaging_disable(); break;
    case '-': change_address(); break;
    case '?': calibration_routine(); break;
    case '.': addr_in_test(); break;
    case '[': set_fov_27(); break;
    case ']': set_fov_16(); break;
    case 't': interrupt_read(); break;
    case '<': signal_limit(); break;
    case '>': sigma_limit(); break;
    case '{': signal_rate_return(); break;
    case '}': ambient_rate_return(); break;



    default: print_menu(); break;
  }


  delay(20);
  //digitalWrite(ledPin, LOW); // set the LED off

}


