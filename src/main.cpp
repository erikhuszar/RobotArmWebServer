#include <Arduino.h>
#include <esp_task_wdt.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <vector>
#include <math.h>










#define WDT_TIMEOUT 60
#define blink_interval 5000



// Server stuff:
//-------------------------------------------------------------------------------------------------
IPAddress ip_address(192, 168, 4, 123);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet_mask(255, 255, 255, 0);

struct JSON_Data
{
  char access_point_name[32];
  char access_point_password[32];
  uint8_t access_point_channel;
};

const char *json_data_filename = "/data.json";
JSON_Data json_data;



AsyncWebServer server(80);







void initSPIFFS()
{
  if (!SPIFFS.begin(true))
  {
    Serial.println("An error has occurred while mounting SPIFFS");
  }

  Serial.println("SPIFFS mounted successfully");
}



void loadJSONData(const char *filename, JSON_Data &data)
{
  File file = SPIFFS.open(filename);

  StaticJsonDocument<512> json_document;

  DeserializationError error = deserializeJson(json_document, file);

  if (error)
  {
    Serial.println(F("Failed to read file, using default data"));
  }

  json_data.access_point_channel = json_document["access_point_channel"] | 0;

  strlcpy
  (
    json_data.access_point_name, // destination

    json_document["access_point_name"] | "C1", // source

    sizeof(json_data.access_point_name) // destination's capacity
  );

  strlcpy
  (
    json_data.access_point_password, // destination

    json_document["access_point_password"] | "987654321", // source

    sizeof(json_data.access_point_password) // destination's capacity
  );

  file.close();
}

void saveJSONData(const char *filename, const JSON_Data & data)
{
  SPIFFS.remove(filename);

  File file = SPIFFS.open(filename, FILE_WRITE);

  if (!file)
  {
    Serial.println(F("Failed to create file"));
    return;
  }

  StaticJsonDocument<256> json_document;

  json_document["access_point_name"] = json_data.access_point_name;
  json_document["access_point_password"] = json_data.access_point_password;
  json_document["access_point_channel"] = json_data.access_point_channel;

  if (serializeJson(json_document, file) == 0)
  {
    Serial.println(F("Failed to write to file"));
  }

  file.close();
}

void printJSONData(const char *filename)
{
  File file = SPIFFS.open(filename);
  if (!file) 
  {
    Serial.println(F("Failed to read file"));
    return;
  }

  while (file.available())
  {
    Serial.print((char)file.read());
  }

  Serial.println();

  file.close();
}



void notFound(AsyncWebServerRequest *request)
{
  request -> send(404, "text/plain", "Not found");
}
//-------------------------------------------------------------------------------------------------



// Pins:
//-------------------------------------------------------------------------------------------------
// Stepper outputs:
/*
const uint8_t stepper_1_IN1 = 26;
const uint8_t stepper_1_IN2 = 25;
const uint8_t stepper_1_IN3 = 33;
const uint8_t stepper_1_IN4 = 32;
*/

// Actual connections:
#define stepper_1_IN1 32
#define stepper_1_IN2 33
#define stepper_1_IN3 25
#define stepper_1_IN4 26

#define stepper_2_IN1 17
#define stepper_2_IN2 23
#define stepper_2_IN3 18
#define stepper_2_IN4 19





// Buttons which are pressed when the robotic arm reaches its maximum and minimum turning angles:
#define stepper_1_min_angle_button 4     // green input wire
#define stepper_2_min_angle_button 16    // white input wire

// Encoder pins:
#define encoder_1_green 13
#define encoder_1_white 14

#define encoder_2_green 5
#define encoder_2_white 27
//-------------------------------------------------------------------------------------------------



// Task handles:
//-------------------------------------------------------------------------------------------------
// Task handles for stepper tasks:
TaskHandle_t stepper_1_task_handle;
TaskHandle_t stepper_2_task_handle;

// Task handle for calibration tasks, which only run once:
TaskHandle_t stepper_calibration_task_handle;

// Task handle for calculating the coordinates and angles of end effector:
TaskHandle_t calculate_current_position_and_angles_task_handle;

// Task hadles for PID loops:
TaskHandle_t PID_1_task_handle;
TaskHandle_t PID_2_task_handle;
//-------------------------------------------------------------------------------------------------





// Variables that store encoder readings:
volatile int encoder_1_counter;
volatile int encoder_2_counter;



// Constants for PID control of position:
//-------------------------------------------------------------------------------------------------
/*
float kp_pos = 1.0;
float ki_pos = 1.0;
float kd_pos = 1.0;
*/

// Constants for PID control of delays (velocity):
float kp_vel = 3.0f; /*30000.0f*/
float ki_vel = 0.5f;
float kd_vel = 50.0f;

/*
// Integral contributions for both motors:
float integ_1 = 0.0f, integ_2 = 0.0f;

// Variables that need to be stored:
float last_error_1 = 0.0f, last_error_2 = 0.0f;
unsigned long last_time = 0;
*/

volatile float setpoint_1 = 90.0f;
volatile float setpoint_2 = 90.0f;

// Variables stored for PID:
float last_error_1 = 0.0f;
unsigned long last_time_1 = 0;

float last_error_2 = 0.0f;
unsigned long last_time_2 = 0;



float prop_1 = 0.0f;
float integ_1 = 0.0f;
float deriv_1 = 0.0f;

float prop_2 = 0.0f;
float integ_2 = 0.0f;
float deriv_2 = 0.0f;


float dt_1 = 0.0f;
float dt_2 = 0.0f;

float error_1 = 0.0f;
float error_2 = 0.0f;
//-------------------------------------------------------------------------------------------------



// Robot constants:
//-------------------------------------------------------------------------------------------------
// l is the length of each arm:
//const float l = 150.0f;
#define l 150.0

// Amount of radians for each encoder pulse:
//const double beta_rad = PI / 200.0;
#define beta_rad (PI / 200.0)

// Amount of degrees for each encoder pulse:
//const double beta_deg = 0.9;
#define beta_deg 0.9

// Number of stepper steps per degree:
//const double alpha_deg = 2037.8864 / 360.0;
#define alpha_deg (2037.8864 / 360.0)
//const double alpha_deg = 1024.0 / 360.0;

// Number of radians for each stepper step:
//const double mu_rad = (2.0 * PI) / 2037.8864;
//-------------------------------------------------------------------------------------------------



// Variables that store position and angles data, as well as stepper delays:
//-------------------------------------------------------------------------------------------------
// Current coordinates of end effector:
volatile float x_p, y_p;
volatile float theta_1_p, theta_2_p;

// Coordinates that the end effector will move to:
float x_d = 0.0f, y_d = 300.0f;
float theta_1_d = 90.0f, theta_2_d = 90.0f;

// Difference between desired and actual angles:
//volatile float delta_theta_1 = 0.0f, delta_theta_2 = 0.0f;

// Delays for steppers:
volatile unsigned int delta_t_1, delta_t_2;

// Length of line segments for interpolation:
#define delta_l 20.0f 

// Variable that stores positions while moving.
// After the values were sent, reset and empty the vectors:
std::vector<float> x_real;
std::vector<float> y_real;
//-------------------------------------------------------------------------------------------------


// Variables for storing button states:
volatile uint8_t stepper_1_stop_button = 1;
volatile uint8_t stepper_2_stop_button = 1;



// Array that stores intermediate points for line segment calculation:
std::vector<float> x_values;
std::vector<float> y_values;

// Array that stores intermediate angles for line segment calculation:
std::vector<float> theta_1_values;
std::vector<float> theta_2_values;

// Array that stores intermediate steps for line segment calculation:
std::vector<int> steps_1;
std::vector<int> steps_2;



// Array that stores intermediate directions for line segment calculation:
// -1 for negative angle difference
// 1 for positive angle difference
/*
std::vector<int16_t> direction_1;
std::vector<int16_t> direction_2;
*/





// If both tasks are at the same i, move.
// If one task is ahead of the other, wait.
// If one task is behind the other one, move.
volatile uint16_t task_1_at = 0;
volatile uint16_t task_2_at = 0;



// Variable that stores whether the arm is moving:
bool moving = false;








// Encoder 1 functions:
// Counter increases in trig direction and decreases in orar direction.
/*
void acti_green_1()
{
  if (digitalRead(encoder_1_white) == LOW)
  {
    encoder_1_counter++;
  }
  else
  {
    encoder_1_counter--;
  }

  //Serial.print("Encoder 1: ");
  //Serial.println(encoder_1_counter);
}

void acti_white_1()
{
  if (digitalRead(encoder_1_green) == LOW)
  {
    encoder_1_counter--;
  }
  else
  {
    encoder_1_counter++;
  }

  //Serial.print("Encoder 1: ");
  //Serial.println(encoder_1_counter);
}



// Encoder 2 functions:
// Counter increases in trig direction and decreases in orar direction.
void acti_green_2()
{
  if (digitalRead(encoder_2_white) == LOW)
  {
    encoder_2_counter++;
  }
  else
  {
    encoder_2_counter--;
  }

  //Serial.print("Encoder 2: ");
  //Serial.println(encoder_2_counter);
}

void acti_white_2()
{
  if (digitalRead(encoder_2_green) == LOW)
  {
    encoder_2_counter--;
  }
  else
  {
    encoder_2_counter++;
  }
  
  //Serial.print("Encoder 2: ");
  //Serial.println(encoder_2_counter);
}
*/



// Alternative functions, which use only 2 interrupts, activated on green wire rising signal:
void count_1()
{
  if (digitalRead(encoder_1_white) == LOW)
  {
    encoder_1_counter++;
  }
  else
  {
    encoder_1_counter--;
  }

  /*
  Serial.print("enc_1: ");
  Serial.println(encoder_1_counter);
  */
}

void count_2()
{
  if (digitalRead(encoder_2_white) == LOW)
  {
    encoder_2_counter++;
  }
  else
  {
    encoder_2_counter--;
  }

  /*
  Serial.print("enc_2: ");
  Serial.println(encoder_2_counter);
  */
}






// Class that defines the stepper objects:
class Stepper
{
  // Variables used for selecting which way the stepper turns (input from user):
  public:

  //int steps_taken = 0;

  uint8_t IN1, IN2, IN3, IN4;
  uint8_t stop_buton = 1;

  bool calibrating = false;

  uint32_t step_delay = 50000;



  /*
  // Function that takes the value of the pushbutton from outside the class:
  void Change_stop_button(uint8_t state)
  {
    stop_buton = state;
  }
  


  // Function that is used to change the delay:
  void Change_delay(int delay)
  {
    step_delay = delay;
  }
  */



  // Function used for selecting the control pins and the 2 physical buttons for each stepper:
  void attach_pins(int A, int B, int C, int D)
  {
    IN1 = A;
    IN2 = B;
    IN3 = C;
    IN4 = D;
  }



  // Function used for writing the respective on/off values to the stepper coils:
  void write(int a, int b, int c, int d)
  {
    digitalWrite(IN1, a);
    digitalWrite(IN2, b);
    digitalWrite(IN3, c);
    digitalWrite(IN4, d);
  }



  // Half-step mode, half the speed, double the angular resolution:
  // Sequence of steps for anti-clockwise rotation of motor:
  /*const int steps_trig[8][4] =
  {
    {1, 0, 0, 0},
    {1, 1, 0, 0},
    {0, 1 ,0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 0},
    {0, 0, 1, 1},
    {0, 0, 0, 1},
    {1, 0, 0, 1}
  };

  // Sequence of steps for clockwise rotation of motor:
  const int steps_orar[8][4] =
  {
    {1, 0, 0, 1},
    {0, 0, 0, 1},
    {0, 0, 1, 1},
    {0, 0, 1, 0},
    {0, 1, 1, 0},
    {0, 1 ,0, 0},
    {1, 1, 0, 0},
    {1, 0, 0, 0}
  };*/



  // Full-step mode, double the speed, half the angular resolution:
  // Only need one array, since it is symmetric.

  // Sequence of steps for anti-clockwise rotation of motor:
  const int steps_fast[4][4] =
  {
    {1, 0, 0, 1},
    {0, 0, 1, 1},
    {0, 1, 1, 0},
    {1, 1, 0, 0}
  };



  // Used for controlling delay:
  unsigned long previous_micros = 0;

  // Function that moves stepper:
  // Positive for trig diection.
  // Negative for orar direction.
  void step(int steps = 1)
  {
    uint16_t j = 0;

    while (j <= abs(steps))
    {
      uint8_t i = 0;

      while (i < 4)
      {
        unsigned long current_micros = micros();

        if ((stop_buton == 0) && (calibrating == false))
        {
          goto end;
        }
        else if (current_micros - previous_micros > step_delay)
        {
          j++;

          if (j == steps)
          {
            goto end;
          }



          if (steps > 0)
          {
            write(steps_fast[i][0], steps_fast[i][1], steps_fast[i][2], steps_fast[i][3]);
          }
          else if (steps < 0)
          {
            write(steps_fast[i][3], steps_fast[i][2], steps_fast[i][1], steps_fast[i][0]);
          }



          i++;

          previous_micros = current_micros;
        }
      }
    }

    end:{}
  }



  // Function that rotates the motor in the clockwise direction:
  /*
  void orar(int delta_t, int steps)
  {
    uint16_t j = 0;

    while (j <= steps)
    {
      uint8_t i = 0;

      while (i < 4)
      {
        unsigned long current_micros = micros();

        if ((stop_buton == 0) && (calibrating == false))
        {
          goto end;
        }
        else if (current_micros - previous_micros > delta_t)
        {
          j++;

          if (j == steps)
          {
            goto end;
          }

          Write(steps_fast[i][3], steps_fast[i][2], steps_fast[i][1], steps_fast[i][0]);

          i++;

          previous_micros = current_micros;
        }
      }
    }

    end:{}
  }
  */



  // Delay for calibration:
  const unsigned long interval_micros = 3000;

  // Stepper 1 calibration function:
  void calibrate_stepper_1()
  {
    while (stop_buton == 1)
    {
      //orar(interval_micros, 1024);

      step(-1024);

      //Serial.println(encoder_1_counter);
    }



    if (stop_buton == 0)
    {
      encoder_1_counter = 0;
      calibrating = true;

      //steps_taken = 0;

      unsigned long previous_time = millis();

      // Waits 1 second for the button state to stabilise:
      while (millis() - previous_time < 1000){}

      // Moves 30 steps to avoid getting stuck:
      for (int i = 0; i < /*30*/ 15; i++)
      {
        //trig(interval_micros, 16);

        step(16);
      } 
    }



    // Moves in the trig direction:
    while (encoder_1_counter < /*250*/ 125)
    {
      //trig(interval_micros, 16);

      step(16);

      //Serial.println(encoder_1_counter);
    }

    //steps_taken = 0;



    encoder_1_counter = /*200*/ 100;



    /*
    while (encoder_1_counter < 400)
    {
      trig(2 * interval_micros, 8);

      //Serial.println(encoder_1_counter);
    }
    */



    delay(500);

    calibrating = false;

    write(0, 0, 0, 0);
  }



  // Stepper 2 calibration function:
  void calibrate_stepper_2()
  {
    while (stop_buton == 1)
    {
      //orar(interval_micros, 1024);

      step(-1024);

      //Serial.println(encoder_2_counter);
    }



    if (stop_buton == 0)
    {
      encoder_2_counter = 0;
      calibrating = true;

      // Waits 1 second for the button state to stabilise:
      unsigned long previous_time = millis();

      while (millis() - previous_time < 1000){}

      // Moves 30 steps to avoid getting stuck:
      for (int i = 0; i < /*30*/ 15; i++)
      {
        //trig(interval_micros, 16);

        step(8);
      } 
    }



    // Moves in the trig direction:
    while (encoder_2_counter < /*270*/ 135)
    {
      //trig(interval_micros, 16);

      step(8);

      //Serial.println(encoder_2_counter);
    }

    //steps_taken = 0;



    encoder_2_counter = /*200*/ 100;



    delay(500);

    calibrating = false;

    write(0, 0, 0, 0);
  }
};



// Creating stepper objects:
Stepper stepper_1;
Stepper stepper_2;





// Functions:
//-------------------------------------------------------------------------------------------------
// Allocate space for at least 50 elements in each vector:
void allocate_space()
{
  x_values.reserve(50);
  y_values.reserve(50);

  theta_1_values.reserve(50);
  theta_2_values.reserve(50);

  steps_1.reserve(50);
  steps_2.reserve(50);

  x_real.reserve(50);
  y_real.reserve(50);
}



// Function for checking sign of number:
int check_sign(float a)
{
  if (a < 0.0f)
  {
    return -1;
  }
  else
  {
    return 1;
  }
}



// Function called to calculate current x and y values:
void calculate_position()
{
  /*
  int enc_1 = encoder_1_counter;
  int enc_2 = encoder_2_counter;
  
  x_p = l * (cos(beta_rad * enc_1) + sin(beta_rad * (enc_1 + enc_2)));
  y_p = l * (sin(beta_rad * enc_1) - cos(beta_rad * (enc_1 + enc_2)));
  */

  x_p = l * (cos(beta_rad * encoder_1_counter) + sin(beta_rad * (encoder_1_counter + encoder_2_counter)));
  y_p = l * (sin(beta_rad * encoder_1_counter) - cos(beta_rad * (encoder_1_counter + encoder_2_counter)));
}



// Function called to calculate current theta_1 and theta_2 values:
void calculate_angles()
{
  theta_1_p = beta_deg * encoder_1_counter;
  theta_2_p = beta_deg * encoder_2_counter;
}



// Function that moves between points:
/*
void move_to_points()
{
  for (int i = 0; i < theta_1_values.size(); i++)
  {
    // First joint movement:
    if (stepper_1_stop_button == 1 && encoder_1_counter < 420 && encoder_1_counter > -20)
    {
      if (direction_1[i] == 1)
      {
        while (abs(theta_1_values[i] - theta_1_p) > 5.0)
        {
          stepper_1.trig(20000, 4);
        }
      }

      if (direction_1[i] == -1)
      {
        while (abs(theta_1_values[i] - theta_1_p) > 5.0)
        {
          stepper_1.orar(20000, 4);
        }
      }
    }

    //vTaskDelay(1);

    // Second joint movement:
    if (stepper_2_stop_button == 1 && encoder_2_counter < 420 && encoder_2_counter > -20)
    {
      if (direction_2[i] == 1)
      {
        while (abs(theta_2_values[i] - theta_2_p) > 5.0)
        {
          stepper_2.trig(20000, 4);
        }
      }

      if (direction_2[i] == -1)
      {
        while (abs(theta_2_values[i] - theta_2_p) > 5.0)
        {
          stepper_2.orar(20000, 4);
        }
      }
    }

    vTaskDelay(500);

    Serial.print("theta1: ");
    Serial.print(theta_1_values[i]);
    Serial.print("    theta1_p: ");
    Serial.println(theta_1_p);

    Serial.print("theta2: ");
    Serial.print(theta_2_values[i]);
    Serial.print("    theta2_p: ");
    Serial.println(theta_2_p);

    Serial.println();
  }

  stepper_1.Write(0, 0, 0, 0);
  stepper_2.Write(0, 0, 0, 0);
}
*/


// Function that shows results of calculations:
void print_results()
{
  // Print size of vectors:
  Serial.print("x_values size: ");
  Serial.print(x_values.size());
  Serial.print("    y_values size: ");
  Serial.println(y_values.size());

  Serial.print("theta_1_values size: ");
  Serial.print(theta_1_values.size());
  Serial.print("    theta_2_values size: ");
  Serial.println(theta_2_values.size());

  Serial.print("steps_1 size: ");
  Serial.print(steps_1.size());
  Serial.print("    steps_2 size: ");
  Serial.println(steps_2.size());

  /*
  Serial.print("dir_1 size: ");
  Serial.print(direction_1.size());
  Serial.print("    dir_2 size: ");
  Serial.println(direction_2.size());
  */

  Serial.println();



  // Print coordinates:
  calculate_position();

  Serial.print("x_p: ");
  Serial.print(x_p);
  Serial.print("    y_p: ");
  Serial.println(y_p);
  Serial.println();

  if (x_values.size() == y_values.size())
  {
    for (int i = 0; i < x_values.size(); i++)
    {
      Serial.print("x");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(x_values[i]);
      Serial.print("    y");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(y_values[i]);
    }
  }
  else
  {
    Serial.println("x_values.size() != y_values.size()");
  }

  Serial.println();



  // Print angles:
  Serial.print("theta1_p: ");
  Serial.print(theta_1_p);
  Serial.print("    theta2_p: ");
  Serial.println(theta_2_p);
  Serial.println();

  if (theta_1_values.size() == theta_2_values.size())
  {
    for (int i = 0; i < theta_1_values.size(); i++)
    {
      Serial.print("theta1_");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(theta_1_values[i]);
      Serial.print("    theta2_");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(theta_2_values[i]);
    }

    Serial.println();

    /*
    for (int i = 0; i < theta_1_values.size(); i++)
    {
      float theta1 = theta_1_values[i] * DEG_TO_RAD;
      float theta2 = theta_2_values[i] * DEG_TO_RAD;

      float x = l * (cos(theta1) + sin(theta1 + theta2));
      float y = l * (sin(theta1) - cos(theta1 + theta2));
      
      Serial.print("x_");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(x);
      Serial.print("    y_");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(y);
    }
    */
  }
  else
  {
    Serial.println("theta_1_values.size() != theta_2_values.size()");
  }

  Serial.println();



  // Print steps:
  if (steps_1.size() == steps_2.size())
  {
    for (int i = 0; i < steps_1.size(); i++)
    {
      Serial.print("steps1_");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(steps_1[i]);
      Serial.print("    steps2_");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(steps_2[i]);
    }

    Serial.println();

    for (int i = 0; i < steps_1.size(); i++)
    {
      float angle_1 = steps_1[i] / alpha_deg;
      float angle_2 = steps_2[i] / alpha_deg;

      Serial.print("theta_1_");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(angle_1);
      Serial.print("    theta_2_");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(angle_2);
    }
  }
  else
  {
    Serial.println("steps_1.size() != steps_2.size()");
  }

  Serial.println();



  // Print directions:
  /*
  if (direction_1.size() == direction_2.size())
  {
    for (int i = 0; i < direction_1.size(); i++)
    {
      Serial.print("dir1_");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(direction_1[i]);
      Serial.print("    dir2_");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(direction_2[i]);
    }
  }
  else
  {
    Serial.println("direction_1.size() != direction_2.size()");
  }
  */
}



// Function that calculates direction for each point:
// 1 is for trig direction
//-1 is for orar direction
/*
void calculate_intermediate_direction()
{
  // Clears previous values stored in vector:
  direction_1.clear();
  direction_2.clear();



  for (int i = 0; i < theta_1_values.size(); i++)
  {
    // The first value is between current point and first stored point:
    if (i == 0)
    {
      if (theta_1_values[i] < theta_1_p)
      {
        direction_1.push_back(1);
      }
      else
      {
        direction_1.push_back(-1);
      }

      if (theta_2_values[i] < theta_2_p)
      {
        direction_2.push_back(1);
      }
      else
      {
        direction_2.push_back(-1);
      }
    }
    else
    {
      if (theta_1_values[i] < theta_1_values[i - 1])
      {
        direction_1.push_back(-1);
      }
      else
      {
        direction_1.push_back(1);
      }



      if (theta_2_values[i] < theta_2_values[i - 1])
      {
        direction_2.push_back(-1);
      }
      else
      {
        direction_2.push_back(1);
      }
    }
  }



  print_results();

  move_1 = true;
  move_2 = true;

  //vTaskResume(stepper_1_task_handle);
  //vTaskResume(stepper_2_task_handle);

  //move_to_points();
}
*/



// Function that calculates number of steps between points:
void calculate_intermediate_steps()
{
  //Serial.println("entered calculate_intermediate_steps");


  // Clears previous values stored in vector:
  steps_1.clear();
  steps_2.clear();

  //Serial.println("cleared values");



  // Update current angles:
  calculate_angles();

  // First steps from current point to first stored point:
  int16_t first_step_1 = (theta_1_values[0] - theta_1_p) * alpha_deg;
  int16_t first_step_2 = (theta_2_values[0] - theta_2_p) * alpha_deg;

  //Serial.println("calculated first steps");

  steps_1.push_back(first_step_1);
  steps_2.push_back(first_step_2);



  Serial.print("steps_1: ");
  Serial.print(first_step_1);
  Serial.print("    steps_2: ");
  Serial.println(first_step_2);




  for (int i = 0; i < theta_1_values.size() - 1; i++)
  {
    //Serial.println("initialised variables");

    int16_t step_1 = (theta_1_values[i + 1] - theta_1_values[i]) * alpha_deg;
    int16_t step_2 = (theta_2_values[i + 1] - theta_2_values[i]) * alpha_deg;

    steps_1.push_back(step_1);
    steps_2.push_back(step_2);



    Serial.print("steps_1: ");
    Serial.print(steps_1[i]);
    Serial.print("    steps_2: ");
    Serial.println(steps_2[i]);
  }



  // Last steps from last point to final desired point (not needed):
  /*
  if (theta_1_values.back() != theta_1_d)
  {
    int16_t last_step_1 = (theta_1_d - theta_1_values.back()) * alpha_deg;

    steps_1.push_back(last_step_1);
  }
  else
  {
    steps_1.push_back(0);
  }

  if (theta_2_values.back() != theta_2_d)
  {
    int16_t last_step_2 = (theta_2_p - theta_2_values.back()) * alpha_deg;

    steps_2.push_back(last_step_2);
  }
  else
  {
    steps_2.push_back(0);
  }

  Serial.print("steps_1: ");
  Serial.print(steps_1.back());
  Serial.print("    steps_2: ");
  Serial.println(steps_2.back());*/
  Serial.println();



  // Remove unused memory in arrays:
  steps_1.shrink_to_fit();
  steps_2.shrink_to_fit();



  //print_results();



  vTaskResume(PID_1_task_handle);
  vTaskResume(PID_2_task_handle);

  last_time_1 = millis();
  last_time_2 = millis();

  vTaskResume(stepper_1_task_handle);
  vTaskResume(stepper_2_task_handle);
}



// Function that calculates intermediate angles:
void calculate_intermediate_angles()
{
  //Serial.println("entered calculate_intermediate_angles");

  // Clears previous values stored in vector:
  theta_1_values.clear();
  theta_2_values.clear();

  //Serial.println("cleared values");



  for (int i = 0; i < x_values.size(); i++)
  {
    float x = x_values[i];
    float y = y_values[i];

    //Serial.println("initialised variables");



    // Desired angles:
    float theta_1 = (atan2(y, x) + acos(sqrt(x * x + y * y) / (2.0f * l))) * RAD_TO_DEG;
    float theta_2 = acos(sqrt((x * x + y * y) * (1.0f - (x * x + y * y) / (4.0f * l * l))) / l) * RAD_TO_DEG;



    //Serial.println("calculated angles");



    // Store angles in arrays if they are within limits:
    if ((theta_1 <= 180.0f) && (theta_1 >= 0.0f) && (theta_2 <= 180.0f) && (theta_2 >= 0.0f))
    {
      theta_1_values.push_back(theta_1);
      theta_2_values.push_back(theta_2);
    }
    // If the first pair of angles aren't within limits, calculate second pair:
    else /*if ((theta_1 > 180.0f) || (theta_1 < 0.0f) || (theta_2 > 180.0f) || (theta_2 < 0.0f))*/
    {
      float theta_1_prime = theta_1 + theta_2 - 0.5f * PI * RAD_TO_DEG;
      float theta_2_prime = PI * RAD_TO_DEG - theta_2;

      // If these values are suitable, append them:
      if ((theta_1_prime <= 180.0f) && (theta_1_prime >= 0.0f) && (theta_2_prime <= 180.0f) && (theta_2_prime >= 0.0f))
      {
        theta_1_values.push_back(theta_1_prime);
        theta_2_values.push_back(theta_2_prime);
      }
    }



    

    //Serial.println("stored angles in arrays");



    
    Serial.print("theta_1: ");
    Serial.print(theta_1_values[i]);
    Serial.print("    theta_2: ");
    Serial.println(theta_2_values[i]);
    
  }

  Serial.println();



  // Remove unused memory in arrays:
  theta_1_values.shrink_to_fit();
  theta_2_values.shrink_to_fit();



  // Calculate steps:
  if (theta_1_values.size() != 0)
  {
    calculate_intermediate_steps();
  }

  //calculate_intermediate_direction();
}



// Function for calculating intermediate points:
void calculate_intermediate_points(float x, float y)
{
  // Clears previous values stored in vector:
  x_values.clear();
  y_values.clear();



  //Serial.println("entered calculate_intermediate_points");

  /*
  Serial.print("x_p: ");
  Serial.print(x_p, 4);
  Serial.print("    y_p: ");
  Serial.println(y_p, 4);

  Serial.print("x_d: ");
  Serial.print(x, 4);
  Serial.print("    y_d: ");
  Serial.println(y, 4);
  */


  calculate_position();

  // Previous points:
  float prev_x = x_p;
  float prev_y = y_p;



  // Varible that stores if the points are valid:
  bool can_continue = false;



  // Length of line segment connecting desired point to current point:
  float L = sqrt((x - x_p) * (x - x_p) + (y - y_p) * (y - y_p));

  
  //Serial.print("L: ");
  //Serial.println(L);
  



  // Used for checking if line intersects workspace:
  // delta > 0, intersection
  // delta = 0, tangent
  // delta < 0, no intersection
  double delta = l * l * ((x - x_p) * (x - x_p) + (y - y_p) * (y - y_p)) - (x_p * y - x * y_p) * (x_p * y - x * y_p);

  /*
  Serial.print("delta: ");
  Serial.println(delta);
  */



  if (delta >= 0.0)
  {
    //Serial.println("delta >= 0");

    float dx = x - x_p;
    float dy = y - y_p;
    float dr = sqrt(dx * dx + dy * dy);

    float D = x_p * y - x * y_p;



    // Coordinates of intersection points:
    float x1 = (D * dy + check_sign(dy) * dx * sqrt(l * l * dr * dr - D * D)) / (dr * dr);
    float y1 = (-D * dx + abs(dy) * sqrt(l * l * dr * dr - D * D)) / (dr * dr);

    float x2 = (D * dy - check_sign(dy) * dx * sqrt(l * l * dr * dr - D * D)) / (dr * dr);
    float y2 = (-D * dx - abs(dy) * sqrt(l * l * dr * dr - D * D)) / (dr * dr);

    /*
    Serial.print("x_1: ");
    Serial.print(x1);
    Serial.print("    y_1: ");
    Serial.println(y1);

    Serial.print("x_2: ");
    Serial.print(x2);
    Serial.print("    y_2: ");
    Serial.println(y2);
    */



    // If legth_1 or length_2 is longer than L, points are outside line segment and intersection point is false:
    float length_1 = sqrt((x1 - x_p) * (x1 - x_p) + (y1 - y_p) * (y1 - y_p));
    float length_2 = sqrt((x2 - x_p) * (x2 - x_p) + (y2 - y_p) * (y2 - y_p));

    /*
    Serial.print("length1: ");
    Serial.println(length_1);
    Serial.print("length2: ");
    Serial.println(length_2);
    Serial.println();
    */



    if (((length_1 > L) || (length_2 > L)) && (L >= delta_l))
    {
      can_continue = true;
    }
    else
    {
      can_continue = false;
    }
  }
  else if (L >= delta_l)
  {
    can_continue = true;
  }
  else
  {
    can_continue = false;
  }





  if (can_continue == true)
  {
    //Serial.println("can_continue==true");

    // Number that divides line segment into 30mm pieces:
    int n = int(L / delta_l);

    /*
    Serial.print("n: ");
    Serial.println(n);
    */



    // Split line into n segments:
    float dL = L / float(n);

    /*
    Serial.print("dL: ");
    Serial.println(dL, 4);
    */

    // Angle that determines slope:
    float angle = atan2(y - y_p, x - x_p);

    /*
    Serial.print("angle: ");
    Serial.println(angle);
    */



    // x and y increments:
    float x_increment = dL * cos(angle);
    float y_increment = dL * sin(angle);

    /*
    Serial.print("x_inc: ");
    Serial.print(x_increment);
    Serial.print("    y_inc: ");
    Serial.println(y_increment);
    */





    // Calculate all x and y coordinates of intermediate points and store them in array:
    for (int i = 0; i < n; i++)
    {
      //Serial.println("entered for loop");

      // Calculate values:
      float current_x = prev_x + x_increment;
      float current_y = prev_y + y_increment;

      
      Serial.print("x: ");
      Serial.print(current_x);
      Serial.print("    y: ");
      Serial.println(current_y);
      
      




      // Append values to vector:
      if ((abs(current_x) <= 300.0f) && (abs(current_y) <= 300.0f))
      {
        x_values.push_back(current_x);
        y_values.push_back(current_y);
      }



      // Store point for next iteration:
      prev_x = current_x;
      prev_y = current_y;
    }

    Serial.println();



    // Remove unused memory in arrays:
    x_values.shrink_to_fit();
    y_values.shrink_to_fit();



    // Calculate angles:
    if (x_values.size() != 0)
    {
      calculate_intermediate_angles();
    }
  }
}



// Function that calculates desired angles when new coordinates are sent through web server:
void calculate_desired_angles(float x, float y)
{
  // Desired angles:
  theta_1_d = (atan2(y, x) + acos(sqrt(x * x + y * y) / (2.0f * l))) * RAD_TO_DEG;
  theta_2_d = acos(sqrt((x * x + y * y) * (1.0f - (x * x + y * y) / (4.0f * l * l))) / l) * RAD_TO_DEG;

  // Calculates points along line:
  calculate_intermediate_points(x, y);
}
//-------------------------------------------------------------------------------------------------





// Task functions:
//-------------------------------------------------------------------------------------------------
// Calibration task for stepper 1, only runs once:
void stepper_calibration(void * parameter)
{
  while (true)
  {
    // Suspend itself:
    vTaskSuspend(stepper_calibration_task_handle);

    // Stops stepper tasks, if they are running:
    vTaskSuspend(stepper_1_task_handle);
    vTaskSuspend(stepper_2_task_handle);

    // Stops position and angle calculation, if running:
    //vTaskSuspend(calculate_current_position_and_angles_task_handle);

    // Stops PID calculation, if running:
    vTaskSuspend(PID_1_task_handle);
    vTaskSuspend(PID_2_task_handle);

    // Wait for one second:
    vTaskDelay(1000);



    // Set delays for steppers during calibration:
    //stepper_1.Change_delay(3000);
    //stepper_2.Change_delay(3000);

    stepper_1.step_delay = 5000;
    stepper_2.step_delay = 3000;



    moving = true;

    // Calibrates stepper 2:
    stepper_2.calibrate_stepper_2();

    moving = false;

    //Serial.println("stepper 2 calibrated");

    vTaskDelay(500);


    moving = true;

    // Calibrates stepper 1:
    stepper_1.calibrate_stepper_1();

    moving = false;



    // Reset delays for steppers during calibration:
    //stepper_1.Change_delay(50000);
    //stepper_2.Change_delay(50000);

    stepper_1.step_delay = 50000;
    stepper_2.step_delay = 50000;

    //Serial.println("stepper 1 calibrated");

    vTaskDelay(500);



    // Starts the tasks that calculate the position and angles.
    // This task can only run after calibration, otherwise the formulas may have division by 0 without proper encoder readings:
    //vTaskResume(calculate_current_position_and_angles_task_handle);

    vTaskDelay(500);

    // Initialises angle calculation:
    calculate_desired_angles(x_d, y_d);

    vTaskDelay(500);

    // Initialises PID loop:
    vTaskResume(PID_1_task_handle);
    vTaskResume(PID_2_task_handle);

    vTaskDelay(100);

    // Starts the tasks that control the steppers:
    //vTaskResume(stepper_1_task_handle);
    //vTaskResume(stepper_2_task_handle);
  }

  vTaskDelete(stepper_calibration_task_handle);
}



// Delay for task:
const TickType_t task_delay = 50;



// Task that monitors the state of button 1:
void button_1_task(void * parameter)
{
  uint8_t last_state = 1, current_state, counter = 0;

  TickType_t LastWakeTime;
  LastWakeTime = xTaskGetTickCount();



  while (true)
  {
    current_state = digitalRead(stepper_1_min_angle_button);

    if (current_state == last_state)
    {
      counter++;

      if (counter > 3)
      {
        stepper_1_stop_button = current_state;

        //stepper_1.Change_stop_button(current_state);

        stepper_1.stop_buton = current_state;

        counter = 0;

        /*
        Serial.print("button 1 = ");
        Serial.println(current_state);
        */
      }
    }
    else
    {
      counter = 0;
    }

    last_state = current_state;

    vTaskDelayUntil(&LastWakeTime, TickType_t(200));
  }

  vTaskDelete(NULL);
}

// Task that monitors the state of button 2:
void button_2_task(void * parameter)
{
  uint8_t last_state = 1, current_state, counter = 0;

  TickType_t LastWakeTime;
  LastWakeTime = xTaskGetTickCount();



  while (true)
  {
    current_state = digitalRead(stepper_2_min_angle_button);

    if (current_state == last_state)
    {
      counter++;

      if (counter > 3)
      {
        stepper_2_stop_button = current_state;

        //stepper_2.Change_stop_button(current_state);

        stepper_2.stop_buton = current_state;

        counter = 0;

        /*
        Serial.print("button 2 = ");
        Serial.println(current_state);
        */
      }
    }
    else
    {
      counter = 0;
    }

    last_state = current_state;

    vTaskDelayUntil(&LastWakeTime, TickType_t(200));
  }

  vTaskDelete(NULL);
}





// Task that calculates end effector x and y positions based on encoder counters:
/*
void Calculate_current_position_and_angles(void * parameter)
{
  vTaskSuspend(calculate_current_position_and_angles_task_handle);

  TickType_t LastWakeTime;
  LastWakeTime = xTaskGetTickCount();



  while (true)
  {
    // Calculate current position:
    x_p = l * (cos(beta_rad * encoder_1_counter) + sin(beta_rad * (encoder_2_counter + encoder_1_counter)));
    y_p = l * (sin(beta_rad * encoder_1_counter) - cos(beta_rad * (encoder_2_counter + encoder_1_counter)));

    // Calculate current angles:
    theta_1_p = beta_deg * encoder_1_counter;
    theta_2_p = beta_deg * encoder_2_counter;


    
    //Serial.print("x_p:");
    //Serial.println(x_p);
    //Serial.print("y_p:");
    //Serial.println(y_p);

    //Serial.print("theta_1_p:");
    //Serial.println(theta_1_p);
    //Serial.print("theta_2_p:");
    //Serial.println(theta_2_p);
    

    vTaskDelayUntil(&LastWakeTime, task_delay);
  }

  vTaskDelete(calculate_current_position_and_angles_task_handle);
}
*/



// PID task for stepper 1:
void PID_1(void * parameter)
{
  /*
  // PID local variables:
  float last_error = 0.0f;
  unsigned long last_time = 0;
  //uint16_t task_1_at_last = 0;

  //float dt = 0.0f;

  float prop = 0.0f;
  float integ = 0.0f;
  float deriv = 0.0f;
  */



  // Variables for precise delay:
  TickType_t LastWakeTime;
  LastWakeTime = xTaskGetTickCount();
  
  
  
  // Suspend task until triggered by other functions:
  vTaskSuspend(PID_1_task_handle);



  while (true)
  {
    /*
    // If step is reached, reset variables for next step:
    if (task_1_at != task_1_at_last)
    {
      last_error = 0.0f;
      last_time = 0;

      dt = 0.0f;

      prop = 0.0f;
      integ = 0.0f;
      deriv = 0.0f;
    }
    */



    // Variable for storing current time:
    unsigned long current_time = millis();

    // Variable that measures elapsed time:
    //float dt = float(current_time - last_time) / 1000.0f;

    dt_1 = float(current_time - last_time_1) / 1000.0f;



    // Update current angles:
    calculate_angles();

    // Error:
    //float error = setpoint_1 - theta_1_p;

    error_1 = setpoint_1 - theta_1_p;

    //Serial.print("error_1: ");
    //Serial.println(error);



    // Calculate only if error is different from 0 to avoid division by 0:
    if (/*error*/ error_1 != 0.0f)
    {
      //Proportional contribution:
      //prop = kp_vel * error;

      prop_1 = kp_vel * error_1;



      //Serial.print("prop1: ");
      //Serial.println(prop);



      // Integral contribution:
      //integ += ki_vel * error * dt;

      integ_1 += ki_vel * error_1 * dt_1;

      // Derivative contribution:
      if (/*(last_error != error) &&*/ (/*dt*/ dt_1 != 0.0f))
      {
        //deriv = kd_vel * (error - last_error) / dt;

        deriv_1 = kd_vel * (error_1 - last_error_1) / dt_1;
      }
      

      
      // Delay for stepper 1:
      //delta_t_1 = int(1000000.0f / abs(prop + integ) + abs(deriv));

      delta_t_1 = int(1000000.0f / abs(prop_1 + integ_1) + abs(deriv_1));

      //Serial.print("delta_t_1: ");
      //Serial.println(delta_t_1);

      // Normalisation of delay value:
      delta_t_1 = constrain(delta_t_1, 2500, 50000);



      // Apply delay to stepper:
      //stepper_1.Change_delay(delta_t_1);

      stepper_1.step_delay = delta_t_1;
    }
    // If the error is 0, delay is 50000:
    else
    {
      delta_t_1 = 50000;

      // Apply delay to stepper:
      //stepper_1.Change_delay(delta_t_1);

      stepper_1.step_delay = delta_t_1;
    }



    // Store last calculated values for next calculation:
    //last_error = error;
    //last_time = current_time;

    last_error_1 = error_1;
    last_time_1 = current_time;

    //task_1_at_last = task_1_at;



    /*
    // If steps are done, reset variables:
    last_error = 0.0f;
    last_time = 0;

    integ = 0.0f;

    dt = 0.0f;
    prop = 0.0f;
    deriv = 0.0f;
    */

    //Serial.print("delta_t_1: ");
    //Serial.println(delta_t_1);

    // Delay task for set number of ticks:
    vTaskDelayUntil(&LastWakeTime, task_delay);



    /*
    // Variable that measures elapsed time:
    dt = float(millis() - last_time) / 1000.0f;



    // Doing calculation for first stepper if error is greater than 0:
    if (delta_theta_1 != 0.0f)
    {
      //Proportional contribution:
      prop_1 = kp_vel / delta_theta_1;

      // Integral contribution:
      integ_1 += (ki_vel / delta_theta_1) * dt;

      // Derivative contribution:
      if (last_error_1 != delta_theta_1)
      {
        deriv_1 = kd_vel / ((delta_theta_1 - last_error_1) * dt);
      }



      // Delay for stepper 1:
      delta_t_1 = abs(prop_1 + integ_1 + deriv_1);
      
      // Normalisation of delay value:
      delta_t_1 = constrain(delta_t_1, 2500, 50000);
    }
    else
    {
      delta_t_1 = 50000;
    }



    // Doing calculation for second stepper if error is greater than 0:
    if (delta_theta_2 != 0.0f)
    {
      //Proportional contribution:
      prop_2 = kp_vel / delta_theta_2;
      
      // Integral contribution:
      integ_2 += (ki_vel / delta_theta_2) * dt;
      
      // Derivative contribution:
      if (last_error_2 != delta_theta_2)
      {
        deriv_2 = kd_vel / ((delta_theta_2 - last_error_2) * dt);
      }



      // Delay for stepper 2:
      delta_t_2 = abs(prop_2 + integ_2 + deriv_2);

      // Normalisation of delay value:
      delta_t_2 = constrain(delta_t_2, 2500, 50000);
    }
    else
    {
      delta_t_2 = 50000;
    }



    // Storing the last error for the derivative calculation:
    last_error_1 = delta_theta_1;
    last_error_2 = delta_theta_2;



    //Serial.print("dt: ");
    //Serial.print(dt, 4);
    //Serial.print("prop_1: ");
    //Serial.print(prop_1);
    //Serial.print("    delta_t_1: ");
    //Serial.println(delta_t_1);
    //Serial.print("    prop_2: ");
    //Serial.println(prop_2);
    //Serial.print("    dt_1: ");
    //Serial.print(delta_t_1);

    //Serial.print("    prop_2: ");
    //Serial.print(kp_vel / abs(delta_theta_2));
    //Serial.print("  dt_2: ");
    //Serial.println(delta_t_2);



    // Storing last time for calculating dt:
    last_time = millis();

    // Delaying the task:
    vTaskDelayUntil(&LastWakeTime, task_delay);
    */
  }

  vTaskDelete(PID_1_task_handle);
}



// PID task for stepper 2:
void PID_2(void * parameter)
{
  /*
  // PID local variables:
  float last_error = 0.0f;
  unsigned long last_time = 0;
  //uint16_t task_2_at_last = 0;

  //float dt = 0.0f;

  float prop = 0.0f;
  float integ = 0.0f;
  float deriv = 0.0f;
  */



  // Variables for precise delay:
  TickType_t LastWakeTime;
  LastWakeTime = xTaskGetTickCount();



  // Suspend task until triggered by other functions:
  vTaskSuspend(PID_2_task_handle);



  while (true)
  {
    /*
    // If step is reached, reset variables for next step:
    if (task_2_at != task_2_at_last)
    {
      last_error = 0.0f;
      last_time = 0;

      dt = 0.0f;

      prop = 0.0f;
      integ = 0.0f;
      deriv = 0.0f;
    }
    */



    // Variable for storing current time:
    unsigned long current_time = millis();

    // Variable that measures elapsed time:
    //float dt = float(current_time - last_time) / 1000.0f;

    dt_2 = float(current_time - last_time_2) / 1000.0f;



    // Update current angles:
    calculate_angles();

    // Error:
    //float error = setpoint_2 - theta_2_p;

    error_2 = setpoint_2 - theta_2_p;

    //Serial.print("error_2: ");
    //Serial.println(error);



    // Calculate only if error is different from 0 to avoid division by 0:
    if (/*error*/ error_2 != 0.0f)
    {
      //Proportional contribution:
      //prop = kp_vel * error;

      prop_2 = kp_vel * error_2;



      //Serial.print("prop2: ");
      //Serial.println(prop);



      // Integral contribution:
      //integ += ki_vel * error * dt;

      integ_2 += ki_vel * error_2 * dt_2;

      // Derivative contribution:
      if (/*(last_error != error) &&*/ (/*dt*/ dt_2 != 0.0f))
      {
        //deriv = kd_vel * (error - last_error) / dt;

        deriv_2 = kd_vel * (error_2 - last_error_2) / dt_2;
      }
      
      
      
      // Delay for stepper 2:
      //delta_t_2 = int(1000000.0f / abs(prop + integ) + abs(deriv));

      delta_t_2 = int(1000000.0f / abs(prop_2 + integ_2) + abs(deriv_2));

      //Serial.print("delta_t_2: ");
      //Serial.println(delta_t_2);
      
      // Normalisation of delay value:
      delta_t_2 = constrain(delta_t_2, 2500, 50000);



      // Apply delay to stepper:
      //stepper_2.Change_delay(delta_t_2);

      stepper_2.step_delay = delta_t_2;
    }
    // If the error is 0, delay is 50000:
    else
    {
      delta_t_2 = 50000;

      // Apply delay to stepper:
      //stepper_2.Change_delay(delta_t_2);

      stepper_2.step_delay = delta_t_2;
    }



    // Store last calculated values for next calculation:
    //last_error = error;
    //last_time = current_time;

    last_error_2 = error_2;
    last_time_2 = current_time;

    //task_2_at_last = task_2_at;

    //Serial.print("delta_t_2: ");
    //Serial.println(delta_t_2);

    // Delay task for set number of ticks:
    vTaskDelayUntil(&LastWakeTime, task_delay);
    


    /*
    // If steps are done, reset variables:
    last_error = 0.0f;
    last_time = 0;

    integ = 0.0f;

    dt = 0.0f;
    prop = 0.0f;
    deriv = 0.0f;

    task_2_at_last = 0;
    */
  }

  vTaskDelete(PID_2_task_handle);
}





// Task responsible for moving stepper 1:
void stepper_1_task(void * parameter)
{
  while (true)
  {
    /*
    if ((stepper_1_stop_button == 1) && (encoder_1_counter < 420) && (encoder_1_counter > -20) && (move_1 == true))
    {
      for (int i = 0; i < steps_1.size(); i++)
      {
        Serial.print("theta_1: ");
        Serial.println(theta_1_values[i]);



        if (direction_1[i] == 1)
        {
          stepper_1.trig(20000, steps_1[i]);
        }

        if (direction_1[i] == -1)
        {
          stepper_1.orar(20000, steps_1[i]);
        }
        


        // If the stepper reached it's desired position, turn off the coils:
   



        vTaskDelay(1);
      }

      move_1 = false;

      stepper_1.Write(0, 0, 0, 0);
      
      //vTaskDelay(100);
    }
    
    vTaskDelay(100);
    */

    moving = false;
    
    vTaskSuspend(stepper_1_task_handle);

    moving = true;



    for (int i = 0; i < steps_1.size(); i++)
    {
      /*
      Serial.print("i: ");
      Serial.println(i);
      Serial.print("steps_1: ");
      Serial.print(steps_1[i]);
      Serial.print("    steps_2: ");
      Serial.println(steps_2[i]);
      Serial.print("theta_1: ");
      Serial.print(steps_1[i] / mu_deg);
      Serial.print("    theta_2: ");
      Serial.println(steps_2[i] / mu_deg);
      */

      /*
      while (Serial.read() == -1)
      {
        vTaskDelay(10);
      }
      */

      //Serial.print("task 1 at:");
      //Serial.println(i);



      setpoint_1 = theta_1_values[i];



      // Move stepper 1:
      if ((stepper_1_stop_button == 1) && (encoder_1_counter < /*420*/ 210) && (encoder_1_counter > /*-20*/ -10))
      {
        /*
        while (move_to_next_point == false)
        {
          vTaskDelay(10);
        }

        if (move_to_next_point == true)
        {
          stepper_1_done = false;
        }
        */

        // Next step is about to commence:
        //stepper_1_done = false;



        task_1_at = i;

        // If Task 1 is ahead of Task 2, wait:
        while (task_1_at > task_2_at)
        {
          vTaskDelay(50);
        }

        // If Task 1 is behind or at the same step as Task 2, move:
        if ((task_1_at == task_2_at) || (task_1_at < task_2_at))
        {
          // Stepping:
          stepper_1.step(steps_1[i]);



          // At each step in the list, calculate actual positions and store them in a variable:
          x_real.push_back(l * (cos(beta_rad * encoder_1_counter) + sin(beta_rad * (encoder_1_counter + encoder_2_counter))));
          y_real.push_back(l * (sin(beta_rad * encoder_1_counter) - cos(beta_rad * (encoder_1_counter + encoder_2_counter))));
        }



        /*
        // Stepping:
        stepper_1.step(20000, steps_1[i]);

        // Step is done:
        stepper_1_done = true;



        // Waiting for other task to finish:
        // If the other task is done, while loop is skipped:
        while (stepper_2_done == false)
        {
          vTaskDelay(10);

          taskYIELD();
        }
        */
      }

      //vTaskDelay(1);
      taskYIELD();



      /*
      // If both steppers are finished, move to next point:
      if ((stepper_1_done == true) && (stepper_2_done == true))
      {
        move_to_next_point == true;
      }
      */
    }



    // Setpoint for PID is final desired angle:
    setpoint_1 = theta_1_d;

    // Reset counter:
    task_1_at = 0;

    // Reset vector:
    steps_1.clear();
    theta_1_values.clear();

    // Deenergise coils:
    stepper_1.write(0, 0, 0, 0);



    // Make the vectors more compact:
    x_real.shrink_to_fit();
    y_real.shrink_to_fit();



    // Stop PID task:
    vTaskSuspend(PID_1_task_handle);

    error_1 = 0.0f;
    prop_1 = 0.0f;
    integ_1 = 0.0f;
    deriv_1 = 0.0f;
    dt_1 = 0.0f;
    last_error_1 = 0.0f;
    //last_time_1 = 0;



    /*
    if ((stepper_1_stop_button == 1) && (encoder_1_counter < 420) && (encoder_1_counter > -20) && (move_1 == true))
    {
      for (int i = 0; i < steps_1.size(); i++)
      {
        Serial.print("theta_1: ");
        Serial.println(theta_1_values[i]);



        if (direction_1[i] == 1)
        {
          stepper_1.trig(20000, steps_1[i]);
        }

        if (direction_1[i] == -1)
        {
          stepper_1.orar(20000, steps_1[i]);
        }



        vTaskDelay(1);
      }

      move_1 = false;

      stepper_1.Write(0, 0, 0, 0);
      
      //vTaskDelay(100);
    }
    */
  }

  vTaskDelete(stepper_1_task_handle);
}



// Task responsible for moving stepper 2:
void stepper_2_task(void * parameter)
{
  while (true)
  {
    moving = false;

    vTaskSuspend(stepper_2_task_handle);

    moving = true;



    for (int i = 0; i < steps_2.size(); i++)
    {
      //Serial.print("task 2 at:");
      //Serial.println(i);



      setpoint_2 = theta_2_values[i];



      if ((stepper_2_stop_button == 1) && (encoder_2_counter < /*420*/ 210) && (encoder_2_counter > /*-20*/ -10))
      {
        /*
        while (move_to_next_point == false)
        {
          vTaskDelay(10);
        }

        if (move_to_next_point == true)
        {
          stepper_2_done = false;
        }
        */

        // Next step is about to commence:
        //stepper_2_done = false;



        task_2_at = i;

        // If Task 2 is ahead of Task 1, wait:
        while (task_2_at > task_1_at)
        {
          vTaskDelay(50);
        }

        // If Task 2 is behind or at the same step as Task 1, move:
        if ((task_2_at == task_1_at) || (task_2_at < task_1_at))
        {
          // Stepping:
          stepper_2.step(steps_2[i]);
        }



        /*
        // Stepping:
        stepper_2.step(20000, steps_2[i]);

        // Step is done:
        stepper_2_done = true;



        // Waiting for other task to finish:
        // If the other task is done, while loop is skipped:
        while (stepper_1_done == false)
        {
          vTaskDelay(10);

          taskYIELD();
        }
        */
      }

      //vTaskDelay(1);
      taskYIELD();

      /*
      // If both steppers are finished, move to next point:
      if ((stepper_1_done == true) && (stepper_2_done == true))
      {
        move_to_next_point == true;
      }
      */
    }



    // Setpoint for PID is final desired angle:
    setpoint_2 = theta_2_d;

    // Reset counter:
    task_2_at = 0;

    // Reset vector:
    steps_2.clear();
    theta_2_values.clear();

    stepper_2.write(0, 0, 0, 0);



    // Stop PID task:
    vTaskSuspend(PID_2_task_handle);

    error_2 = 0.0f;
    prop_2 = 0.0f;
    integ_2 = 0.0f;
    deriv_2 = 0.0f;
    dt_2 = 0.0f;
    last_error_2 = 0.0f;
    //last_time_2 = 0;
  }
 
  vTaskDelete(stepper_2_task_handle);
}
//-------------------------------------------------------------------------------------------------





void setup()
{
  // Initialising serial connection:
  Serial.begin(115200);


  
  // Initialising and setting up server:
  initSPIFFS();

  loadJSONData(json_data_filename, json_data);

  Serial.println("Initializing Access Point");
  WiFi.softAPConfig(ip_address, gateway, subnet_mask);

  WiFi.softAP(json_data.access_point_name, json_data.access_point_password, json_data.access_point_channel, false);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("Access Point IP Address: ");
  Serial.println(IP); 

  Serial.print("Access point name: ");
  Serial.println(String(json_data.access_point_name));

  Serial.print("Access point password: ");
  Serial.println(String(json_data.access_point_password));

  Serial.print("Access point channel: ");
  Serial.println(String( json_data.access_point_channel));

  Serial.println();





  server.on
  ("/", HTTP_GET, [] (AsyncWebServerRequest *request)
    {
      request -> send(SPIFFS, "/index.html", "text/html");
    }
  );

  server.on
  ("/style.css", HTTP_GET, [] (AsyncWebServerRequest *request)
    {
      request -> send(SPIFFS, "/style.css", "text/css");
    }
  );

  server.on
  ("/script.js", HTTP_GET, [] (AsyncWebServerRequest *request)
    {
      request -> send(SPIFFS, "/script.js", "text/javascript");
    }
  );

  server.on
  ("/favicon.png", HTTP_GET, [] (AsyncWebServerRequest *request)
    {
      request -> send(SPIFFS, "/favicon.png", "image/png");
    }
  );


  /*
  server.on
  ( "/one.png", HTTP_GET, [] ( AsyncWebServerRequest *request )
    {
      request->send( SPIFFS, "/one.png", "image/png" );
    }
  );

  server.on
  ( "/two.png", HTTP_GET, [] ( AsyncWebServerRequest *request )
    {
      request->send( SPIFFS, "/two.png", "image/png" );
    }
  );

  server.on
  ( "/three.png", HTTP_GET, [] ( AsyncWebServerRequest *request )
    {
      request->send( SPIFFS, "/three.png", "image/png" );
    }
  );

  server.on
  ( "/four.png", HTTP_GET, [] ( AsyncWebServerRequest *request )
    {
      request->send( SPIFFS, "/four.png", "image/png" );
    }
  );

  server.on
  ( "/five.png", HTTP_GET, [] ( AsyncWebServerRequest *request )
    {
      request->send( SPIFFS, "/five.png", "image/png" );
    }
  );

  server.on
  ( "/six.png", HTTP_GET, [] ( AsyncWebServerRequest *request )
    {
      request->send( SPIFFS, "/six.png", "image/png" );
    }
  );
  */


  /*
  server.on
  ( "/0", HTTP_GET, [] ( AsyncWebServerRequest *request )
    {
      //buttonAction( 0 );
      request->send( 200, "text/plain", "---" );

      stepper_1_button = 0;
      stepper_2_button = 0;

      vTaskSuspend(stepper_1_task_handle);
      vTaskSuspend(stepper_2_task_handle);

      stepper_1.Write(0, 0, 0, 0);
      stepper_2.Write(0, 0, 0, 0);
    }
  );

  server.on
  ( "/1", HTTP_GET, [] ( AsyncWebServerRequest *request )
    {
      //buttonAction( 1 );
      request->send( 200, "text/plain", "ONE" );

      stepper_1_button = 1;

      vTaskResume(stepper_1_task_handle);
    }
  );

  server.on
  ( "/2", HTTP_GET, [] ( AsyncWebServerRequest *request )
    {
      //buttonAction( 2 );
      request->send( 200, "text/plain", "TWO" );

      stepper_1_button = 2;

      vTaskResume(stepper_1_task_handle);
    }
  );

  server.on
  ( "/3", HTTP_GET, [] ( AsyncWebServerRequest *request )
    {
      //buttonAction( 3 );
      request->send( 200, "text/plain", "THREE" );

      stepper_2_button = 1;

      vTaskResume(stepper_2_task_handle);
    }
  );

  server.on
  ( "/4", HTTP_GET, [] ( AsyncWebServerRequest *request )
    {
      //buttonAction( 4 );
      request->send( 200, "text/plain", "FOUR" );

      stepper_2_button = 2;

      vTaskResume(stepper_2_task_handle);
    }
  );

  server.on
  ( "/5", HTTP_GET, [] ( AsyncWebServerRequest *request )
    {
      //buttonAction( 5 );
      request->send( 200, "text/plain", "FIVE" );

      vTaskResume(stepper_calibration_task_handle);
    }
  );

  server.on
  ( "/6", HTTP_GET, [] ( AsyncWebServerRequest *request )
    {
      //buttonAction( 6 );
      request->send( 200, "text/plain", "SIX" );
    }
  );
  */



  // Show coordinates and angles:
  server.on
  ("/7", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      // Update coordinates and angles:
      calculate_position();
      calculate_angles();

      // Send values:
      request -> send(200, "text/plain", "{\"type\":1,\"x\":" + String(x_p) + ",\"y\":" + String(y_p) + ",\"theta_1\":" + String(theta_1_p) + ",\"theta_2\":" + String(theta_2_p) + "}");

      //{"type":1,"x":111,"y":222,"theta_1":333,"theta_2":444}
    }
  );

  // Show delays:
  /*
  server.on
  ("/8", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request -> send(200, "text/plain", "{\"type\":2,\"delta_t_1\":" + String(delta_t_1) + ",\"delta_t_2\":" + String(delta_t_2) + "}");

      //{"type":2,"delta_t_1":111,"delta_t_2"}
    }
  );
  */ 
  
  // Show desired angles:
  server.on
  ("/9", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request -> send(200, "text/plain", "{\"type\":3,\"theta_1_desired\":" + String(theta_1_d) + ",\"theta_2_desired\":" + String(theta_2_d) + "}");

      //{"type":3,"theta_1_desired":111,"theta_2_desired":222}
    }
  );

  // Send stored real coordinates:
  server.on
  ("/10", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      // Temporary strings to store the values of the vectors in string format in order to be sent via the 
      String x_numbers, y_numbers;



      // Create strings from the vectors:
      for (int i = 0; i < x_real.size() - 1; i++)
      {
        x_numbers += String(x_real[i]) + ',';

        y_numbers += String(y_real[i]) + ',';
      }

      // Add last elements without the comma:
      x_numbers += String(x_real.back());
      y_numbers += String(y_real.back());



      // Sending the values:
      request -> send(200, x_numbers + ';' + y_numbers);



      // Resetting the vectors for the next cycle:
      x_real.clear();
      y_real.clear();
    }
  );

  // Send whether the arm is moving or not:
  server.on
  ("/11", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      request -> send(200, "text/plain", "{\"type\":4,\"moving\":" + String(moving));

      //{"type":4,"moving":0}
    }
  );



  // PID values input for position control:
  /*
  server.on
  ("/PID_pos", HTTP_GET, [] (AsyncWebServerRequest * request)
    {
      String value_1, value_2, value_3;

      if (request -> hasParam("proportional_pos"))
      {
        value_1 = request -> getParam("proportional_pos") -> value();

        request -> send(200);


        if (value_1.toFloat() != NAN)
        {
          kp_pos = value_1.toFloat();

          Serial.println("Proportional constant position:");
          Serial.println(kp_pos);
        }
      }

      if (request->hasParam("integrator_pos"))
      {
        value_2 = request -> getParam("integrator_pos") -> value();

        request -> send(200);


        if (value_2.toFloat() != NAN)
        {
          ki_pos = value_2.toFloat();

          Serial.println("Integrator constant position:");
          Serial.println(ki_pos);
        }
      }
      
      if (request -> hasParam("derivative_pos"))
      {
        value_3 = request -> getParam("derivative_pos") -> value();

        request -> send(200);


        if (value_3.toFloat() != NAN)
        {
          kd_pos = value_3.toFloat();

          Serial.println("Derivative constant position:");
          Serial.println(kd_pos);
          Serial.println();
        }
      }
    }
  );
  */

  // PID values input for velocity control:
  server.on
  ("/PID_vel", HTTP_GET, [] (AsyncWebServerRequest * request)
    {
      String value_1, value_2, value_3;

      if (request -> hasParam("proportional_vel"))
      {
        value_1 = request -> getParam("proportional_vel") -> value();

        request -> send(200);



        // Convert value to float:
        // If value is not a number, function returns 0.
        kp_vel = value_1.toFloat();

        Serial.print("Proportional constant velocity: ");
        Serial.println(kp_vel);
      }

      if (request->hasParam("integrator_vel"))
      {
        value_2 = request -> getParam("integrator_vel") -> value();

        request -> send(200);



        // Convert value to float:
        // If value is not a number, function returns 0.
        ki_vel = value_2.toFloat();

        Serial.print("Integrator constant velocity: ");
        Serial.println(ki_vel);
      }
      
      if (request -> hasParam("derivative_vel"))
      {
        value_3 = request -> getParam("derivative_vel") -> value();

        request -> send(200);



        // Convert value to float:
        // If value is not a number, function returns 0.
        kd_vel = value_3.toFloat();

        Serial.print("Derivative constant velocity: ");
        Serial.println(kd_vel);
        
      }
    }
  );



  // Coordinates input:
  server.on
  ("/coord", HTTP_GET, [] (AsyncWebServerRequest * request)
    {
      String x_value, y_value;

      if (request -> hasParam("x_input"))
      {
        x_value = request -> getParam("x_input") -> value();

        request -> send(200);


        //Serial.println("Received x");

        x_d = x_value.toFloat();

        //Serial.println();
        //Serial.println("Desired x:");
        //Serial.println(x_d);
      }

      if (request -> hasParam("input_y"))
      {
        y_value = request -> getParam("input_y") -> value();

        request -> send(200);


        //Serial.println("Received y");

        y_d = y_value.toFloat();


        //Serial.println();
        //Serial.println("Desired y:");
        //Serial.println(y_d);
        //Serial.println();
      }

      // Checking if the given values are within the space that the robot can move to:
      if ((pow(abs(x_d), 2) + pow(abs(y_d), 2) < pow(l, 2)) || (pow(abs(x_d), 2) + pow(abs(y_d), 2) > pow(2 * l, 2)) || (y_d < 0.0))
      {
        x_d = 0;
        y_d = 300;

        Serial.println("Coordinates are outside the workspace. Lower bound equation: x^2 + y^2 = (150)^2. Upper bound equation: x^2 + y^2 = (300)^2.");
      }

      calculate_desired_angles(x_d, y_d);
    }
  );



  // Calibration:
  server.on
  ("/calibrate_button_pressed", HTTP_GET, [](AsyncWebServerRequest *request)
    {
      x_d = 0.0;
      y_d = 300.0;

      vTaskResume(stepper_calibration_task_handle);
    }
  );




  server.onNotFound(notFound);
  server.begin();



  //esp_task_wdt_deinit();

  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);

  //esp_task_wdt_add(stepper_1_task_handle);



  // Allocate space for vectors:
  allocate_space();



  // Initialising stepper pins:
  pinMode(stepper_1_IN1, OUTPUT);
  pinMode(stepper_1_IN2, OUTPUT);
  pinMode(stepper_1_IN3, OUTPUT);
  pinMode(stepper_1_IN4, OUTPUT);

  pinMode(stepper_2_IN1, OUTPUT);
  pinMode(stepper_2_IN2, OUTPUT);
  pinMode(stepper_2_IN3, OUTPUT);
  pinMode(stepper_2_IN4, OUTPUT);



  // Initialising buttons as input pullup:
  pinMode(stepper_1_min_angle_button, INPUT_PULLUP);
  pinMode(stepper_2_min_angle_button, INPUT_PULLUP);

  // Reading values of pins only once:
  stepper_1_stop_button = digitalRead(stepper_1_min_angle_button);
  stepper_2_stop_button = digitalRead(stepper_2_min_angle_button);



  // Initialising encoder pins:
  pinMode(encoder_1_white, INPUT_PULLUP);
  pinMode(encoder_1_green, INPUT_PULLUP);

  pinMode(encoder_2_white, INPUT_PULLUP);
  pinMode(encoder_2_green, INPUT_PULLUP);





  // Encoder interrupts:
  /*
  attachInterrupt(digitalPinToInterrupt(encoder_1_green), acti_green_1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder_1_white), acti_white_1, RISING);

  attachInterrupt(digitalPinToInterrupt(encoder_2_green), acti_green_2, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder_2_white), acti_white_2, RISING);
  */



  attachInterrupt(digitalPinToInterrupt(encoder_1_green), count_1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder_2_green), count_2, RISING);
  

  //Serial.println(xPortGetCoreID()); // setup runs on core 1


  // Calling the functions which associate stepper coils with output pins:
  stepper_1.attach_pins(stepper_1_IN1, stepper_1_IN2, stepper_1_IN3, stepper_1_IN4);
  stepper_2.attach_pins(stepper_2_IN1, stepper_2_IN2, stepper_2_IN3, stepper_2_IN4);





  // Tasks:
  // Stepper calibration task:
  xTaskCreatePinnedToCore(stepper_calibration, "Stepper 1 calibration", 1000, NULL, 2, &stepper_calibration_task_handle, 0);

  // Tasks responsible for moving the arm:
  xTaskCreatePinnedToCore(stepper_1_task, "Stepper 1 Task", 10000, NULL, 3, &stepper_1_task_handle, 0);
  xTaskCreatePinnedToCore(stepper_2_task, "Stepper 2 Task", 10000, NULL, 3, &stepper_2_task_handle, 0);

  // Tasks that monitor button state:
  xTaskCreatePinnedToCore(button_1_task, "Button 1 Task", 1000, NULL, 4, NULL, 0);
  xTaskCreatePinnedToCore(button_2_task, "Button 2 Task", 1000, NULL, 4, NULL, 0);

  // Task that calculates current position and errors in position:
  //xTaskCreatePinnedToCore(Calculate_current_position_and_angles, "Calculate coordinates", 5000, NULL, 6, &calculate_current_position_and_angles_task_handle, 0);

  // PID tasks:
  xTaskCreatePinnedToCore(PID_1, "PID_1", 5000, NULL, 5, &PID_1_task_handle, 0);
  xTaskCreatePinnedToCore(PID_2, "PID_2", 5000, NULL, 5, &PID_2_task_handle, 0);
}





void loop()
{
  //vTaskDelete(NULL);

  TickType_t LastWakeTime;
  LastWakeTime = xTaskGetTickCount();



  while (true)
  {
    esp_task_wdt_reset();

    vTaskDelayUntil(&LastWakeTime, blink_interval);
  }

  /*
  uint32_t current_milliseconds = millis();
  uint32_t elapsed_milliseconds_from_last_blink = ((current_milliseconds > last_blink) ? (current_milliseconds - last_blink) : 0);


  if (elapsed_milliseconds_from_last_blink >= blink_interval)
  {
    last_blink = current_milliseconds;

    esp_task_wdt_reset();
  }
  */
}