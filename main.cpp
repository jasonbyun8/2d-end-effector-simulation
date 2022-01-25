/* 
# 2D inverse kinematics of a 2-link serial manipulator in 2D-space
# 
# Inputs
# [L1, L2]: length of the robot links
# [X0, Y0]: initial position of the end-effector
# [X1, Y1]: desired final position of the end-effector
# 
# Ouputs
# Initial, intermediate and final joint angles
# Initial, intermediate and final end-effector position
#
# Assumptions
# 1. Zero mass, manipulator joints perfectly track the command angles
# 2. Positive angle configuration for angles (since the joint angles configuration for each poision is not unique)
# 3. Cartesian coordinate; x-dir is horizontal direction of the ground, y-dir is vertical direction
# 4. The first link is attached to the origin
# 5. The motion of the arm is limited between ||L1 - L2|| and ||L1 + L2|| 
# 6. Singluar point will not be included in the trajector. e.g. L1 = L2, the origin (x=0, y=0) is singular
# 7. The manipulator moves along a straight-line
# 
*/

#include <iostream>
#include <math.h>
#include <stdio.h>
#include <sstream>

using namespace std;

/////////////////////////////////////////
// User defined structur and functions
// Defined a structure for 2D coordinates
struct coord {double x; double y;};

double norm (double a, double b){
  return sqrt(a*a+b*b);
} 
////////////////////////////////////////

// Converts positions to joint angles
coord from_pos_to_angle(coord pos, coord L){
    
    double theta2 = acos((pos.x*pos.x + pos.y*pos.y - L.x*L.x - L.y*L.y)/(2 * L.x * L.y));
    double theta1 = atan2(pos.y , pos.x) - atan2((L.y * sin(theta2)) , (L.x + L.y * cos(theta2)));
    coord angles = {theta1, theta2};

  return angles;
}

coord assign_pos_init(){
  cout << "Type the x coordinate of the initial position of the end-effector:" << endl;
  double pos_init_x;
  cin >> pos_init_x;

  cout << "Type the y coordinate of the initial position of the end-effector:" << endl;
  double pos_init_y;
  cin >> pos_init_y;
  coord init = {pos_init_x, pos_init_y};
  return init;
}

coord assign_pos_des(){
  cout << "Type the x coordinate of the desired position of the end-effector:" << endl;
  double pos_des_x;
  cin >> pos_des_x;

  cout << "Type the y coordinate of the desired position of the end-effector:" << endl;
  double pos_des_y;
  cin >> pos_des_y;
  coord des = {pos_des_x, pos_des_y};
  return des;
}

coord assign_link_length(){
  cout << "Type the length of the first link:" << endl;
  double L_x;
  cin >> L_x;

  cout << "Type the length of the second link:" << endl;
  double L_y;
  cin >> L_y;
  coord length = {L_x, L_y};
  return length;
}

// Checks feasibility of the straight line trajectory
bool is_given_inputs_feasible(coord pos_init, coord pos_des, coord L){
  double x0 = pos_init.x;
  double x1 = pos_des.x;
  double y0 = pos_init.y;
  double y1 = pos_des.y;
  double a = y0 - y1;
  double b = -x0 + x1;
  double c = y0 * (x0 - x1) - (y0 - y1) * x0;
  double d = abs(c)/sqrt(a*a + b*b); // Distance of the trajectory line from the orign

  if (norm(x0,y0) > L.x + L.y || norm(x0,y0) < abs(L.x - L.y) 
  ||norm(x1,y1) > L.x + L.y || norm(x1,y1) < abs(L.x - L.y)) {
    cout << "Position(s) is(are) not in the operable range. Terminating ..." << endl;
    exit(0);
    return false;
  } else if (d < abs(L.x-L.y)) {
    cout << "Straight-line trajectory is not possible. Terminating ..." << endl;
    exit(0);
    return false;
  } else if (L.x == L.y && d == 0){
    cout << "Straight-line trajectory includes a singular point. Terminating ..." << endl;
    exit(0);
    return false;
  } 
  return true;
}

/////////////////////////////////////////////////////////////////////////////////
// A complimentory functions for an alined table
string prd(const double x, const int decDigits, const int width) {
    stringstream ss;
    ss << fixed << right;
    ss.fill(' ');        
    ss.width(width);     
    ss.precision(decDigits); 
    ss << x;
    return ss.str();
}

string center(const string s, const int w) {
    stringstream ss, spaces;
    int padding = w - s.size();                 
    for(int i=0; i<padding/2; ++i)
        spaces << " ";
    ss << spaces.str() << s << spaces.str();    
    if(padding>0 && padding%2!=0)               
        ss << " ";
    return ss.str();
}
///////////////////////////////////////////////////////////////////////////////

// Main simulation
int main(void){

  // Define constants

  // Initial & Final position 2D cartesian coordinates
  coord pos_init = assign_pos_init();
  coord pos_des = assign_pos_des();
  coord L = assign_link_length();
  bool feasibility = is_given_inputs_feasible(pos_init, pos_des, L);  

  // Convert positions to joint angles
  coord angle_init = from_pos_to_angle(pos_init, L);
  coord angle_des = from_pos_to_angle(pos_des, L);

  // Print format
  cout << center("Angle 1 [rad]",13)       << " | "
  << center("Angle 2 [rad]",13)     << " | "
  << center("x, end-effector",15)     << " | "
  << center("y, end-effector",15) << "\n";

  cout << string(15*2 + 15*2 + 2*4, '-') << "\n";

  // Print inital data
  cout << prd(angle_init.x,3,13)       << " | "
  << prd(angle_init.y,3,13)     << " | "
  << prd(pos_init.x,3,15)     << " | "
  << prd(pos_init.y,3,15) << " (initial)"<<"\n";

  double N = 50.0; // Number of segments for the straight line

  // coord pos_des_local = {pos_init.x + (pos_des.x-pos_init.x) * (1.0) / N,
  // pos_init.y + (pos_des.y-pos_init.y) * (1.0) / N};
  // angle_des = from_pos_to_angle(pos_des_local, L);
  coord pos_des_local;
  
  for(double i=0; i < N; i++){
    // Update joint angles & end-effector positions
    pos_des_local.x = pos_init.x + (pos_des.x-pos_init.x) * (i + 1.0) / N;
    pos_des_local.y = pos_init.y + (pos_des.y-pos_init.y) * (i + 1.0) / N;
    angle_des = from_pos_to_angle(pos_des_local, L);

    // Print joint angles & end-effector positions
    if (i == N-1){
      cout << prd(angle_des.x,3,13)       << " | "
      << prd(angle_des.y,3,13)     << " | "
      << prd(pos_des_local.x,3,15)     << " | "
      << prd(pos_des_local.y,3,15) << " (final)"<< "\n";
    } else {
      cout << prd(angle_des.x,3,13)       << " | "
      << prd(angle_des.y,3,13)     << " | "
      << prd(pos_des_local.x,3,15)     << " | "
      << prd(pos_des_local.y,3,15) << "\n";
    }   
  }
}

