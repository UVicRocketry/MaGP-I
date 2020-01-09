// PROGRAM : ParafoilPIDTest
// PROJECT : Autonomous Parafoil
// NAME 1 : River Leuba V00816018
// DESC : This program is meant to simulate the control scheme for the parafoil drone project
// UNITS : Length: meters

#include <TinyGPS++.h>
#include <SoftwareSerial.h>

//Control Global Variables and Constants Setup*************************************************************************

int test_var = 0;

//Target Latitude and Logitude of the Drone:
const double target_lat   = 0.0; 
const double target_long  = 0.0;

//Desired Path Paremeters:
const unsigned char num_cntrl_circles     = 3;                              //the number of control circles the drone will fly
unsigned char current_cntrl_circle        = 0;                              //the current control circle
const double cntrl_radii[]                = {800, 400, 175};          //the radius data for each control circle
const double cntrl_alt[]                  = {4000, 1000, 0};         //the altitude data for each control circle

//PID control values:
const double max_turn_rate = 8.0;
const double max_accel_rate = 1.60;
const double k_u = max_turn_rate/180.0;    //Max Proportional gain. When the error is max (180 degrees), we want it to return the maximum turn per second (assumed 8 degrees per second)
const double k_p = 0.6*k_u;                //Proportional gain.
const double k_d = 0.1;                    //Derivative gain
const double k_i = 0.00015;                //Integral gain

double k_pd    = max_turn_rate/cntrl_radii[num_cntrl_circles];   //This is additional proportional gain for when we are within the circle.
double error_d = 0;                                              //This the control radius minus the distance to the target.

double integral_sum = 0.0;                 //Stores the integral value
double error        = 0;                   //This is the difference between the current heading and the desired heading
double error_prev   = 0;                   //For the derivative gain


//Current Drone Status:
double current_velo     = 10.124;            //Horizontal Velocity
double current_velo_z   = 3.23;              //Vertical Velocity
double current_lat      = -1500.0086868768;       
double current_long     = -2000.00000001;
double current_heading  = 235.0;
double current_alt      = 8000.0;           //Alttitude

//Contol Iterator:
  const double delta_v_range   = 3.4;
  const double delta_v_z_range = 0.98;
  
  const double v_avg   = 9.45;
  const double v_z_avg = 4.36;
  
  const double delta_t = 1.0; //the control timestep
  double delta_h;             //This is the command (ie. by how much the heading changes per iteration, ie. the angular velocity)
  double delta_h_prev  = 0;   //Stores the previous command

//Physical System Global Variables and Constants Setup*************************************************************************

//Pin Declarations:
const int motor_pin   = 1;
const int encoder_pin = 2;
//etc

//Function Declarations********************************************************************************************************
void iterateTimeStep();
void writePos();
void generateCommand();
double getDist(double lat_a, double long_a);
void printFinish(int num);

void setup(){
  Serial.begin(1000000);
  pinMode(motor_pin, OUTPUT);
  //etc
}//end setup

void loop(){
  //read the current position (automatically done by using global variable, lol)
  
  //generate the new commands;
  generateCommand();
  
  //write the new position to the serial moniter
  writePos();
  
  //update the time step
  iterateTimeStep();
  
  //once we've passed the altitude threshold, itearte to the next control annulus
  if(current_alt < cntrl_alt[current_cntrl_circle]){
    //integral_sum = 0;                                                  //Reset the integral gain
    current_cntrl_circle++;
    k_pd = max_turn_rate/cntrl_radii[current_cntrl_circle];            //Update the distance proportional gain. 
  }//end if
  if(current_cntrl_circle >= num_cntrl_circles) current_cntrl_circle = num_cntrl_circles - 1;
 
}//end loop

void iterateTimeStep(){
  /*
    Things to be updated:
      velocity
      z velocity
      position
      altitude
      heading
  */
  double delta_v, delta_v_z;
  
  //Update the velocity
  delta_v = random(0,delta_v_range) - delta_v_range; //this is not correct
  current_velo = v_avg + delta_v;
  
  //Update the z velocity
  delta_v_z = random(0,delta_v_z_range) - delta_v_z_range; //or this....
  current_velo_z = v_z_avg + delta_v_z;
  
  //Update the heading
  current_heading += delta_h;
  if (current_heading >= 360.0)    current_heading -= 360.0;
  else if (current_heading < 0.0)  current_heading += 360.0;
  
  //Update the position
  current_lat  += current_velo*delta_t*cos(radians(current_heading));
  current_long += current_velo*delta_t*sin(radians(current_heading));
  
  //Update the altitude
  current_alt -= current_velo_z * delta_t;  
}//end iterateTimeStep


void generateCommand(){
  double current_distance = getDist(current_lat, current_long);
  double dir  = 1.0;                                                             //Stores whether we need to go cw (1) or ccw (-1). 
 
  double g = degrees(acos((current_lat - target_lat)/current_distance));
  if(current_long < target_long) g = 360.0 - g;
  if(g >= 360.0) g -= 360.0;
  
  double current_heading_rot = current_heading - g;
  if (current_heading_rot < 0.0)current_heading_rot += 360.0;

  if(current_distance >= cntrl_radii[current_cntrl_circle]){        //we are outside the control circle
    //we need to turn until our path intersects the control circle.

    error_d = 0.0; 
    double alpha = degrees(asin(cntrl_radii[current_cntrl_circle]/current_distance));
        
    if ((current_heading_rot >= 0.0 && current_heading_rot < 180.0 - alpha) || (current_heading_rot > 180.0 && current_heading_rot < 180.0 + alpha)) dir = 1.0;
    else dir = -1.0;
    
    if(current_heading_rot < 180.0){
      error = fabs(180.0 - alpha - current_heading_rot);
    }//end if
    else{
      error = fabs(current_heading_rot - 180.0 - alpha);
    }//end else
    
    integral_sum += error*delta_t;
    
    delta_h = dir*(k_p*error + k_i*integral_sum + k_d*((error-error_prev)/delta_t));
    error_prev = error;

   if(delta_h > max_turn_rate) delta_h = max_turn_rate;
   else if (delta_h < -1.0*max_turn_rate) delta_h = -1.0*max_turn_rate;
  
  
  }//end if
  else {  //we are inside the control radius
    if (current_heading_rot >= 0.0 && current_heading_rot < 90.0 || current_heading_rot > 180.0 && current_heading_rot < 270.0) dir = 1.0;
    else dir = -1.0;
    
    if(current_heading_rot < 180.0) error = fabs(90.0 - current_heading_rot);
    else error = fabs(270.0 - current_heading_rot);
    
    error_d = cntrl_radii[current_cntrl_circle] - current_distance;
    
    integral_sum += error*delta_t; 
    delta_h = dir*(k_pd*error_d + k_p*error + k_i*integral_sum*0.0 + k_d*((error-error_prev)/delta_t)*0.0);
    if(delta_h > max_turn_rate) delta_h = max_turn_rate;

  }//end else if
  
  if(delta_h - delta_h_prev > max_accel_rate) delta_h = delta_h_prev + max_accel_rate;
  else if(delta_h_prev - delta_h > max_accel_rate) delta_h = delta_h_prev - max_accel_rate;
  delta_h_prev = delta_h;
  
}//end generateCommand

void writePos(){

    //if we've reached the ground, stop the loop
  if(current_alt < 0.0){ 
   printFinish(6);
   while(1);
  }//end if

  Serial.print(current_lat,6);
  Serial.print("\t");
  Serial.print(current_long,6);
  Serial.print("\t");
  Serial.print(current_alt,6);
  Serial.print("\t");
  Serial.print(integral_sum,6);
  Serial.print("\t");
  Serial.print(error_prev,6);
  Serial.print("\t");
  Serial.println(error,6);

/*
  Serial.print(k_p*error);
  Serial.print("\t");
  
  Serial.print(k_i*integral_sum);
  Serial.print("\t");
  
  Serial.print(k_d*((error-error_prev)/delta_t));
  Serial.print("\t");
  
  if(error_d != 0.0) Serial.print(error_d);
  Serial.print("\t");
*/
}//end writePos

//This function prints the final numbers to let matlab know we're done.
void printFinish(int num){
  int i;
  for (i=1;i<num;i++) {
   Serial.print(-1.0,6); //print data
   Serial.print("\t");
  }//end for
  Serial.println(-1.0,6); //print data
}//end print Finish

double getDist(double lat_a, double long_a){
  lat_a -= target_lat;
  long_a -= target_long;
  
  return sqrt(lat_a*lat_a+long_a*long_a);
}//end getDist
