#ifndef CMOUSE_CTRL_H
#define CMOUSE_CTRL_H

#include "kinematics.h"
#include <thread>

// ROS includes
#include "ros/ros.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"

class CSpinePos
{
public:
    CSpinePos() { spine=tail=0; }
    CSpinePos(double s, double t) { spine=s; tail=t; }
    double spine;
    double tail;
};

class CSpine
{
public:
    CSpine(){}

    virtual ~CSpine() {}

    double stretch();
    double crouch();
    CSpinePos moveLeft(int length);
    CSpinePos moveRight(int length);
    CSpinePos moveStepLeft(int length);
    CSpinePos moveStepRight(int length);
    CSpinePos centre();

private:
    const int RangeLeft = 50;
    const int RangeRight = 130;
    const int posStreched = 110;
    const int posCrouched = 180;
    const int cOffsetSpine = -15;
    const int cOffsetTail = 0;
    const int cOffsetFlex = 0;
    const int posCentre = 90;
    const int spineStep = 20;
    const int posFarLeft = posCentre + cOffsetSpine - RangeLeft;
    const int posFarRight = posCentre + cOffsetSpine + RangeRight;
    const int posTailFarLeft = posCentre + cOffsetTail - RangeLeft;
    const int posTailFarRight = posCentre + cOffsetTail + RangeRight;


    bool leftStart = true;
    bool rightStart = true;
    double leftStepsize, rightStepsize;
    int curSP = posCentre + cOffsetSpine;
    int curTL = posCentre + cOffsetTail;

};

// class for the 4 legs
class CMouseLeg : public CKinematics
{
public:
  CMouseLeg(char _leg, char _side, int _pawLift) { vx=vy=0; leg=_leg; side=_side; pawLift=_pawLift;}
  virtual ~CMouseLeg() {}

  typedef enum Phase{ Swing, Stance} typPhase;
  //FUNCTIONS
  void StartLeg(double x, double y, int length, typPhase phase);
  CLegPos GetNext();
  void MoveTo   (double x, double y);
  void Dump();

private:
  //FUNCTIONS
  void StepStart(double x, double y);
  bool StepNext ();
  CLegPos NextWayPoint();
  CLegPos SetPosition(CLegPos ang);//double deg1, double deg2);
  float Distance(float x1, float y1, float x2, float y2);

  //OBJECTS
  CKoord  ptLeg;   // x/y destination pos in move, current pos !in move
  CLegPos dgNext;  // Next Point on walking line
  CLegPos output;  // Output Value
  typPhase currPhase;
  CKoord docu[100]; //docu array

  //VARIABLES
  double vx, vy;   // vector from current to destination x/y-point, divided by step stepcount
  int  stepcount,  // number of kinematic-steps
       step,       // current step number
       pawLift;
  char leg;    // Motor id of hip motor (knee++)
  char side;
};

class CMouseCtrl
{
public:
    CMouseCtrl() {clearArr();}
    virtual ~CMouseCtrl() {}
    //FUNCTIONS
    //control
    void Ctrl();
    void startThread();
    //walking
    void clearArr();
    void Trot(int motionlength);
    void Init(int length = 1);
    void TrotBkw(int motionlength);

    //VARIABLES
    int messages; // message to UI thread

    static const int ArrayBuffer = 100;
    static const int Motors = 13;
    double TrottArray[ArrayBuffer][Motors+1];

    typedef enum Direction{ Fwd, Bkwd, left, right, stop, stance} typDir;
    typDir dir = stop;


    void SitUp(int length);
private:
    //FUNCTIONS
    void TrotRight();
    void moveLeg();    
    void Print(int length = 1);
    //OBJECTS
    CMouseLeg LForeLeft = CMouseLeg('f','l', Lift); //fwd=0 bkwd=180 up=0 down=180
    CMouseLeg LForeRight = CMouseLeg('f','r', Lift);//fwd=180 bkwd=0 up=180 down=0
    CMouseLeg LHindLeft = CMouseLeg('h','l', Lift); //fwd=0 bkwd=180 up=0 down=180
    CMouseLeg LHindRight = CMouseLeg('h','r', Lift); //fwd=180 bkwd=0 up=180 down=0
    CSpine Spine = CSpine();

    //Motion Parameters
    static const int Lift = 15; //height of the foot lift
    const int uFrontLegStart  = -10;  // x start pos. of pace
    const int uHindLegStart   = -20;
    const int uStepLengthF    = 70;  // length of one leg move on the ground
    const int uStepLengthH    = 70;  // length of one leg move on the ground
    const int uWalkLevel      = 5;//10;  // y walking level of init and walking
    const int uSitting_x      = -30;  // X position for foot in sitting
    const int uSitting_y      = 30;  // Y position for foot in sitting


    double MotionArray[ArrayBuffer][Motors+1];

};

class CMouseRos : public CMouseCtrl
{
public:
    CMouseRos();
    virtual ~CMouseRos() {}
    //FUNCTIONS
    void RosCtrl(); 
    void ROSstartThread();
private:
    //FUNCTIONS
    void Publish(int length = 1);
    //OBJECTS
    std_msgs::Float64MultiArray msgarr;
    ros::NodeHandle n;
    ros::Publisher pub;

};

class CMouseUI
{
public:
    CMouseUI(int& msg) : _msg(msg) { }
    virtual ~CMouseUI() {}

    void process();
private:
    int getch();
    int& _msg;
};

#endif // CMOUSE_CTRL_H
