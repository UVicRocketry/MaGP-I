// PROGRAM : ParafoilGPS
// PROJECT : MaGP-I
// NAME 1 : River Leuba V00816018
// DESC : This program is the control scheme for the parafoil drone project
// UNITS : Length: meters

#include <TinyGPS++.h>
#include <Encoder_Buffer.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h>


//Control Global Variables and Constants Setup*************************************************************************

//Target Latitude and Longitude of the Drone:
const double target_lat   =  48.462853; 
const double target_long  = -123.311187;

//Desired Path Paremeters:
const unsigned char num_cntrl_circles     = 4;                              //the number of control circles the drone will fly
unsigned char current_cntrl_circle        = 0;                              //keeps track of the current control circle
const double cntrl_radii[]                = {100, 100, 100, 100};           //the radius data for each control circle
const double cntrl_alt[]                  = {10000, 4000, 1000, 0};         //the altitude data for each control circle

//PID control values:
const double max_turn_rate = 8.0;
const double k_u = max_turn_rate/180.0;    //Max Proportional gain. When the error is max (180 degrees), we want it to return the maximum turn per second (assumed 8 degrees per second)
const double k_p = 0.6*k_u;                //Proportional gain.
const double k_d = 0.1;                    //Derivative gain
const double k_i = 0.00015;                //Integral gain
double integral_term = 0;
double derivative_term = 0;
double proportional_term = 0;

double k_pd    = max_turn_rate/cntrl_radii[0];   //This is additional proportional gain for when we are within the circle.
double error_d = 0;                              //This the control radius minus the distance to the target.

double integral_sum = 0.0;                 //Stores the integral value.
double error        = 0;                   //This is the difference between the current heading and the desired heading.
double error_prev   = 0;                   //The error in the previous control cycle. Used for the derivative portion of control.

//Current Drone Status:
double current_lat      = 0.0;           //Latitude.
double current_long     = 0.0;           //Longitude.
double current_y        = 0.0;           //Stores the y position from the target [m].
double current_x        = 0.0;           //Stores the x distance from the target [m].
double current_heading  = 0.0;           //Heading. N is 0°, E is 90°, S is 180°, W is 270°.
double current_alt      = 0.0;           //Altitude [m].
double current_distance = 0.0;           //Distance from current position to target position.
bool is_valid           = false;         //This store whether the data was updated between control cycles.

//Control Iterator:
const int delta_t = 1000;   //the control timestep, in ms.
double delta_h;             //This is the command (ie. by how much the heading changes per iteration, ie. the angular velocity).
double delta_h_prev  = 0;   //Stores the previous command.

//Physical System Global Variables and Constants Setup*************************************************************************

//Pin Declarations:
const int motor_pin1   = 10;
const int motor_pin2   = 6;
const int home_pin     = 7;
const int encoder_CS  = 8;
static const int RXPin = 5, TXPin = 3;
const int teleGPS_pin  = 8;

//GPS and Software Serial Setup:
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;                          // The TinyGPS++ object
SoftwareSerial ss(RXPin, TXPin);          // The serial connection to the GPS device

//Encoder Object Setup
Encoder_Buffer Encoder1(encoder_CS);

//SD Card Setup
File myFile;

//Motor Setup:
long encoder_count              = 0;       //Stores how many encoder counts we are from center. +tive for turning left. -tive for turning right. 
long motor_dir                  = 0;       //Stored the motor direction. +1 for CW/turn left. -1 for CCW/turn right. 0 for standing still.
long encoder_target             = 0;       //Where we want to be. Updated by generateCommand()

long duty_cycle                  = 255;    //The duty cycle for speed control of the motor

const int encoder_count_per_rev = 6600;    //Encoder resolution. Encoder has 11 pulses per motor rev, 150:1 gear ratio, and encoder counter counts 4 times per cycle. 
const int CW                    = 1;
const int CCW                   = -1;

//Physical Control Parameters
const double pulley_radius = .082;        //The radius of the pulley used to control the shroud lines, in [m].
const double line_to_turn_rate = 0.05;    //The conversion factor from deg/second of turning, to meters of line. If we pull the line by .01[m], and turn at 3[deg/sec], this 
                                           //      would be .01[m]/3[deg/s] = 0.0033333[m*s/deg]

//Function Declarations********************************************************************************************************
//Initialiser Functions
void SDInit();

//Control Loop Functions
void readPos();
void generateCommand();
void executeCommand();
void smartDelay(unsigned long ms);

//There are two types of write functions: "Pos" and "Data". "Pos" writes to the serial port,
  //and can be read by the arduino serial moniter, or matlab. "Data" writes to the SD card.  
  //"Pos" functions should be commented out during actual flight.  
void writePos();
void writeInvalidPos();

void writeData();
void writeInvalidData();

//Motor Functions
void homeMotor();
void moveMotor();
void stopMotor();

void setup(){
  Serial.begin(115200);        //Initilize the serial out connection.
  ss.begin(GPSBaud);           //Intialize the GPS serial connection.
  stopMotor();                 //Make sure the motor isn't moving
  
  //Assign Pin Modes:
  pinMode(motor_pin1, OUTPUT);
  pinMode(motor_pin2, OUTPUT);
  pinMode(home_pin, INPUT);
  
  SPI.begin();                    //Initilise SPI connection, used for the encoder.
  
  Encoder1.initEncoder();         //Initilize the encoder

  SDInit();                       //Initialize the SD Card
  
  homeMotor();                    //Home the motor
  smartDelay(9000);               //Wait for the parfoil to fully inflate
}//end setup

void loop(){
  //read the current position 
  readPos();
  
  //If the GPS values are valid
  if(is_valid){  
    //write the new position to the serial moniter
    writePos();
    writeData();
                                       
    //generate the new commands
    generateCommand();

    //execute the new command
    executeCommand();
    
  }//end if
  else{
    //if the position was not valid, we should output something to the serial moniter. 
    writeInvalidPos();
    writeInvalidData();
  }//end else
  
  //once we've passed the altitude threshold, iterate to the next control annulus
  if(current_alt < cntrl_alt[current_cntrl_circle]){
    //integral_sum = 0;                                                  //Reset the integral gain
    current_cntrl_circle++;
    k_pd = max_turn_rate/cntrl_radii[current_cntrl_circle];             //Update the distance proportional gain. 
  }//end if
  if(current_cntrl_circle >= num_cntrl_circles) current_cntrl_circle = num_cntrl_circles - 1;         //prevent overflow of current_cntrl_circle
}//end loop

//Intalises the SD card, and writes the data table heading to the data file.
void SDInit(){
  SD.begin(4);                                  //Intialise the SD Card using pin 4 as the chip select
  myFile = SD.open("data.txt", FILE_WRITE);     //Open the Data file.
  if (myFile) {                                 //If myFile isn't NULL
    myFile.println("Pos Y\tPos X\tLongitude\tLatitude\tAlt\tHeading\tDelta_h\terror\terror_d\tencder_count\tencdr_target\tCurrent_Distance\tP\tI\tD");         //Print the data table heading
    myFile.close();                              // close the file:
  }//end if
}//end SD Init

//Reads the Current position, course, and altitude data from the gps. If any one of those 
  //three are not valid, it will set is_valid to false, indicating the data is no good. 
void readPos(){
  int count = 0;                            //Used to make sure all the data is good. 
  smartDelay(delta_t);                      //Delays for the control step, giving time for the GPS to update
  if(gps.location.isValid()){
    current_lat  = gps.location.lat();       //Read the lat and long into these global variables. 
    current_long = gps.location.lng();
    
    current_y = TinyGPSPlus::distanceBetween(current_lat,current_long,  target_lat,current_long);       //Use the distanceBetween function to transform the latitude and logitude to
    current_x = TinyGPSPlus::distanceBetween(current_lat,current_long,  current_lat,target_long);         //an x and y value that we can use. 
    if(current_lat < target_lat) current_y *= -1.0;                                                     //distanceBetween returns only positive values. We need to know if we're in a negative
    if(current_long < target_long) current_x *= -1.0;                                                     //quadrant.
    count++;
    
  }//end if
  if(gps.course.isValid()){
    current_heading = gps.course.deg();                                       //Read the course data
    count++;
  }//end if
  if(gps.altitude.isValid()){
    current_alt = gps.altitude.meters();                                      //Read the Altitude data.
    current_alt = 12000;                                                    //*******************************PLACEHOLDER FOR TESTING RWMOVE BEFORE FLIGHT*******************************
    count++;
  }//end if
  
  if(count == 3)  is_valid = true;              //Lets us know if all the data we just read is valid
  else            is_valid = false;               //and safe to generate a command from.
}//end readPos

//Uses the position and heading information to determine the desired position for the motor (encoder_target).
  //Uses a PID loop to minimise the error between the current heading and the desired control circle. 
void generateCommand(){
  current_distance =  TinyGPSPlus::distanceBetween(current_lat,current_long,  target_lat,target_long); //This is the distance between the parafoil and the target
  if(current_distance == 0)  return 0;                                                   //This would result in invalid numbers. 
  if(current_y > current_distance) current_y = current_distance;                //Due to measurement errors, this sometimes happens and would result in NaN.
  
  double dir  = 1.0;                                                             //Stores whether we need to go cw (1) or ccw (-1). 
 
  double g = degrees(acos((current_y/current_distance)));                     //This is the angle the current position makes with the y axis. 
  if(current_long < target_long) g = 360.0 - g;
  if(g >= 360.0) g -= 360.0;
  
  double current_heading_rot = current_heading - g;                       //This rotates the entire coordinate system by g degrees. This effectivly rotates the 
  if (current_heading_rot < 0.0)current_heading_rot += 360.0;               //current position onto the positive y axis. This makes finding the angles much much easier. 

  if(current_distance >= cntrl_radii[current_cntrl_circle]){        //If the current position is outside the control circle
    //we need to turn until our path intersects the control circle.
 
    double alpha = degrees(asin(cntrl_radii[current_cntrl_circle]/current_distance));       //This is the angle of the tangent to the control circle. 
        
    if ((current_heading_rot >= 0.0 && current_heading_rot < 180.0 - alpha) || (current_heading_rot > 180.0 && current_heading_rot < 180.0 + alpha)) dir = 1.0;         //In these cases, we need to turn CW
    else dir = -1.0;                                                                                                                                                      //Otherwise, CCW
    
    if(current_heading_rot < 180.0){
      error = fabs(180.0 - alpha - current_heading_rot);
    }//end if
    else{
      error = fabs(current_heading_rot - 180.0 - alpha);
    }//end else
    
    integral_sum += error*delta_t;
    proportional_term = k_p*error;
    integral_term = k_i*integral_sum;
    derivative_term = k_d*((error-error_prev)/delta_t);
    delta_h = dir*(proportional_term + integral_term*0 + derivative_term*0);
    
    //delta_h = dir*(k_p*error + k_i*integral_sum + k_d*((error-error_prev)/delta_t));
    error_prev = error;
  }//end if
  
  else {  //we are inside the control radius
    if (current_heading_rot >= 0.0 && current_heading_rot < 90.0 || current_heading_rot > 180.0 && current_heading_rot < 270.0) dir = 1.0;
    else dir = -1.0;
    
    if(current_heading_rot < 180.0) error = fabs(90.0 - current_heading_rot);
    else error = fabs(270.0 - current_heading_rot);
    
    error_d = cntrl_radii[current_cntrl_circle] - current_distance;
    
    integral_sum += error*delta_t; 
    delta_h = dir*(k_pd*error_d + k_p*error + k_i*integral_sum*0.0 + k_d*((error-error_prev)/delta_t)*0.0);

  }//end else if

     if(delta_h > max_turn_rate) delta_h = max_turn_rate;
   else if (delta_h < -1.0*max_turn_rate) delta_h = -1.0*max_turn_rate;

  encoder_target = (long) encoder_count_per_rev/(2.0*3.14159*pulley_radius)*line_to_turn_rate*delta_h;
  
}//end generateCommand

//Used to do more. Now it just calls the moveMotor function to go to the desired position. 
void executeCommand(){
  moveMotor();
}//end executeCommand

//Uses a PI loop to turn the motor to the position specified by encoder_target. 
void moveMotor(){
  encoder_count = Encoder1.readEncoder();           //Read the current encoder position
  if(encoder_count == encoder_target) return 0;     //If we are already in position, we don't need to do anything
  
  unsigned long prev_time = micros();               //PI control initial timestep
  double motor_integral_sum = 0;                    //Stores the integral sum for PI control
  long encoder_count_prev = 0;                      //Stores the previous encoder position
  int stability_criteria = 0;                       //Starts off as 0. Incremented when the position isn't changing
                                                      //much between timsteps, and the posistion is near the target.
  double encoder_error = 100;                       //Difference between where we are and where we want to be. Initialized as arbitrary non-zero number.
  double dt = 0.000227;                             //Control timestep used to calculate integral gain. In this case it is the fastest period
                                                      //experienced by the encoder (40RPM * 1min/60sec * 6600pulse/rev) = 4400Hz = 227 microseconds.
                                                      //Could be larger as the motor doesn't actually rotate @ 40RPM
  double K_i = 0.1;                                 //Integral Gain. 
  while (stability_criteria < 100){                 // We need to stop at some point and stop blocking the program. 
                                                      //We want stability_criteria to be incrermented 100 times before we're sure we are at the correct position. 
                                                      //If this number is too small, we could overshoot and stop. It it's too big, the
                                                      //system will be slow.

    
    if  ((micros() - prev_time ) >= 227){           //This makes sure we send a command and update the integral gain once
      prev_time = micros();                           //per timestep.

      encoder_count = Encoder1.readEncoder();                       //Read the encoder
      encoder_error = (double) encoder_target - encoder_count;      //Calculate the error
      
      if(encoder_error < 0) {                       //If the error is negative, we need to go CCW
        motor_dir = CCW;
        encoder_error *= -1;
      }
      else motor_dir = CW;                          //Otherwise, we need to go CW
      
      motor_integral_sum += encoder_error*dt;                         //Update the integral gain
      duty_cycle =  encoder_error  + K_i * motor_integral_sum;        //Calculate the new command. Here, K_p is one.
      
      if(duty_cycle > 255) duty_cycle = 255;            //We don't want the duty cycle to be above 255.
      if(duty_cycle < 150) duty_cycle = 150;            //Or below this number, as the motor would stall
      //printEncoder();

      if(motor_dir == CW){                              //Turn the motor with the new duty_cycle
        analogWrite(motor_pin1,duty_cycle);
        digitalWrite(motor_pin2,LOW);
      }//end if
      else{
        digitalWrite(motor_pin1,LOW);
        analogWrite(motor_pin2,duty_cycle);
      }//end else
      
      if(encoder_error < 10 && encoder_count == encoder_count_prev) stability_criteria++;           //If the error is small, and we havn't chaged position
                                                                                                      //in the last timestep, it's a good indication we are at 
                                                                                                      //the desired position, at steady state. 
      encoder_count_prev = encoder_count;                             //Update encoder_count_prev
      
    }//end if
    if (ss.available()){
      gps.encode(ss.read());                       //Continue to update the gps, while not moving the motor. 
      //readPos();                                 //If there's a new command, do that instead. Maybe. This is experimental.                              
      //generateCommand();                           
      //moveMotor();
      //return 0;
    }
    
  }//end while
  stopMotor();
}//end moveMotor

//Stop The motor from turning. 
void stopMotor(){
  digitalWrite(motor_pin1,LOW);
  digitalWrite(motor_pin2,LOW);
  motor_dir = 0;
}//end stopMotor

//Homing routine for the pulley. A magnet on the pully triggers a hall effect sensor, pulling its output low.  
void homeMotor(){
  digitalWrite(motor_pin1,HIGH);                  //Turn the motor CW
  digitalWrite(motor_pin2,LOW);
  
  if(!digitalRead(home_pin)) delay(200);          //If the magnet starts on the sensor, we want to wait 200ms for the magnent to move off the sensor.
  
  while(digitalRead(home_pin));                   //Wait until the sensor triggers and stops reading HIGH
  stopMotor();                                    //Stop the motor

  Encoder1.clearEncoderCount();                   //Move CCW by -1510 encoder counts to put us back to Top Dead Center. (-1510 experimentally determined)
  encoder_target = -1510;
  moveMotor();
  Encoder1.clearEncoderCount();
  encoder_target = 0;
}//end homeMotor

//Writes data to the serial moniter all in one line, and seperated by tabs. In this form, it is
  //easy for Matlab to read. 
void writePos(){
  Serial.print(current_y);
  Serial.print("\t");
  Serial.print(current_x);
  Serial.print("\t");
  Serial.print(current_alt);
  Serial.print("\t");
  Serial.print(current_heading);
  Serial.print("\t");
  Serial.print(delta_h,6);
  Serial.print("\t");
  Serial.print(error,6);
  Serial.print("\t");
  Serial.print(error_d,6);
  Serial.print("\t");
  Serial.print(encoder_count);
  Serial.print("\t");
  Serial.print(encoder_target);
  Serial.print("\t");
  Serial.print(current_distance);
  Serial.print("\t");
  Serial.print(proportional_term);
  Serial.print("\t");
  Serial.print(integral_term);
  Serial.print("\t");
  Serial.println(derivative_term);
}//end writePos

//Writes data to the text file all in one line, and seperated by tabs.
void writeData(){
  myFile = SD.open("data.txt", FILE_WRITE);     //Open the Data file.
  if (myFile) {                                 //If myFile isn't NULL
    myFile.print(current_y);                    //Write ALL THE DATA
    myFile.print("\t");
    myFile.print(current_x);
    myFile.print("\t");
    myFile.print(current_lat);                   
    myFile.print("\t");
    myFile.print(current_long);
    myFile.print("\t");
    myFile.print(current_alt);
    myFile.print("\t");
    myFile.print(current_heading);
    myFile.print("\t");
    myFile.print(delta_h,6);
    myFile.print("\t");
    myFile.print(error,6);
    myFile.print("\t");
    myFile.print(error_d,6);
    myFile.print("\t");
    myFile.print(encoder_count);
    myFile.print("\t");
    myFile.print(encoder_target);
    myFile.print("\t");
    myFile.print(current_distance);
    myFile.print("\t");
    myFile.print(proportional_term);
    myFile.print("\t");
    myFile.print(integral_term);
    myFile.print("\t");
    myFile.println(derivative_term);
    myFile.close();                              // close the file:
  }//end if
}//end writePos

void writeInvalidPos(){
  Serial.println(-1.0);
}//end writeInvalidPos

void writeInvalidData(){
  myFile = SD.open("data.txt", FILE_WRITE);     //Open the Data file.
  if (myFile) {                                 //If myFile isn't NULL
    myFile.println("Invalid Data");             //Write to txt file
    myFile.close();                             //Close the file:
  }//end if
}//end writeInvalidData

//This alternative to delay() ensures the GPS is read while waiting around
void smartDelay(unsigned long ms){
  unsigned long start = millis();
  do {
    while (ss.available())
      gps.encode(ss.read());
  }// end do
  while (millis() - start < ms);
}//end smartDelay
