#include "cmouse_ctrl.h"
#include <cmath>
#include <iostream>
#include <termios.h>
#include <unistd.h>

// ROS Includes


/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//defines for Motors
#define TIMESTAMP 0
#define FORELEFT_HIP 1
#define FORELEFT_KNEE 2
#define FORERIGHT_HIP 3
#define FORERIGHT_KNEE 4
#define HINDLEFT_HIP 5
#define HINDLEFT_KNEE 6
#define HINDRIGHT_HIP 7
#define HINDRIGHT_KNEE 8
#define SPINE 9
#define TAIL 10
#define SPINE_FLEX 11
#define HEAD_PAN 12
#define HEAD_TILT 13

// motion is created via motionarray
// speed is done via amount of points to be published (old setup: 100 values at 500hz?!)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ROS CLASS

CMouseRos::CMouseRos()
{
    msgarr.data.resize(14); //need to declare the size, else it wont work
    pub = n.advertise<std_msgs::Float64MultiArray>("nrpmouse_servotopic", 512);
    //ros::Rate loop_rate(10); //run at 10Hz, not sleep time
}

//starting the loop thread
void CMouseRos::ROSstartThread() {
    std::cout << "starting UART and MOUSE thread"<<std::endl;
    std::thread t1 ([=] { RosCtrl(); });
    t1.detach();
    std::cout << "Walker thread detached"<<std::endl;
}

void CMouseRos::RosCtrl()
{
    int motionlength = 50;
    int dir = '0';
    char state = '0';
    clearArr(); //set everything to 90 deg, to avoid damage.

    //while(ros.OK)
    while (ros::ok())
    {
        //dir = getch();
        dir = messages;
        if (dir != -1){
            state = dir;
        }
        switch (state) {
        case 'i':
            Init();
            std::cout << "init" << std::endl;
            Publish();
            break;
        case 'w':
            Trot(motionlength);
            std::cout << "Straight ahead" << std::endl;
            Publish(motionlength);
            break;
        case 'a':
            std::cout<<"Left Turn"<<std::endl;
            break;
        case 's':
            std::cout<<"Backwards"<<std::endl;
            break;
        case 'd':
            std::cout<<"Right Turn"<<std::endl;
            break;
        case 'q':
            std::cout<<"Quitting"<<std::endl;
            return;
        case 'e':
            std::cout<<"DUMP"<<std::endl;

            break;
        }
    }
}

void CMouseRos::Publish(int length)
{
    ros::Rate loop_rate(40); //run at 40Hz, not sleep time
    for(int i=0;i<length;i++){

        msgarr.data[TIMESTAMP] = TrottArray[i][TIMESTAMP];
        msgarr.data[FORELEFT_HIP]=(TrottArray[i][FORELEFT_HIP]);
        msgarr.data[FORELEFT_KNEE]=(TrottArray[i][FORELEFT_KNEE]);
        msgarr.data[FORERIGHT_HIP]=(TrottArray[i][FORERIGHT_HIP]);
        msgarr.data[FORERIGHT_KNEE]=(TrottArray[i][FORERIGHT_KNEE]);
        msgarr.data[HINDLEFT_HIP]=(TrottArray[i][HINDLEFT_HIP]);
        msgarr.data[HINDLEFT_KNEE]=(TrottArray[i][HINDLEFT_KNEE]);
        msgarr.data[HINDRIGHT_HIP]=(TrottArray[i][HINDRIGHT_HIP]);
        msgarr.data[HINDRIGHT_KNEE]=(TrottArray[i][HINDRIGHT_KNEE]);
        msgarr.data[SPINE]=(TrottArray[i][SPINE]);
        msgarr.data[TAIL]=(TrottArray[i][TAIL]);
        msgarr.data[SPINE_FLEX]=(TrottArray[i][SPINE_FLEX]);
        msgarr.data[HEAD_PAN]=(TrottArray[i][HEAD_PAN]);
        msgarr.data[HEAD_TILT]=(TrottArray[i][HEAD_TILT]);

        pub.publish(msgarr);
        ros::spinOnce();
        loop_rate.sleep();

    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// UI methods

void CMouseUI::process()
{

    do {
    _msg = getch();
    usleep(100);
    }while (_msg != '.');
}

int CMouseUI::getch()
{
    //a non locking getchar() which will always wait for input
    static struct termios oldt, newt;
    tcgetattr( STDIN_FILENO, &oldt);           // save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON);                 // disable buffering
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings
    int dir = getchar();  // read character (non-blocking)
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
    return dir;

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// CMouseCtrl Methods

void CMouseCtrl::Init(int length)
{
    int i;
    CLegPos tmpLeg;
    CSpinePos tmpSpine;

    //Spine positions
    tmpSpine = Spine.centre();


    //initalize Leg motion with Right leg forward
    LHindLeft.StartLeg(0, 0, length, CMouseLeg::Stance);
    LHindRight.StartLeg(0, 0, length, CMouseLeg::Stance);
    LForeLeft.StartLeg(0, 0, length, CMouseLeg::Stance);
    LForeRight.StartLeg(0, 0, length, CMouseLeg::Stance);

    //calculate Servo Values and write the points to TrottArray
    for (i=0; i<length; i++)
    {
        TrottArray[i][TIMESTAMP] = i;
        tmpLeg = LHindLeft.GetNext();
        TrottArray[i][HINDLEFT_HIP] = tmpLeg.leg;
        TrottArray[i][HINDLEFT_KNEE] = tmpLeg.coil;
        tmpLeg = LHindRight.GetNext();
        TrottArray[i][HINDRIGHT_HIP] = tmpLeg.leg;
        TrottArray[i][HINDRIGHT_KNEE] = tmpLeg.coil;
        tmpLeg = LForeLeft.GetNext();
        TrottArray[i][FORELEFT_HIP] = tmpLeg.leg;
        TrottArray[i][FORELEFT_KNEE] = tmpLeg.coil;
        tmpLeg = LForeRight.GetNext();
        TrottArray[i][FORERIGHT_HIP] = tmpLeg.leg;
        TrottArray[i][FORERIGHT_KNEE] = tmpLeg.coil;
        TrottArray[i][SPINE] = tmpSpine.spine;
        TrottArray[i][TAIL] = tmpSpine.tail;

    }

}

void CMouseCtrl::Trot(int motionlength)
{
    //Variables
    int halfMotion = motionlength/2;
    int i;
    CLegPos tmp;
    CSpinePos tmpSpine;

    //Spine positions
    tmpSpine = Spine.centre();

    //Setting Goals starting with Right leg forward
    LHindLeft.StartLeg(uHindLegStart, uWalkLevel, halfMotion, CMouseLeg::Swing);
    LHindRight.StartLeg(uStepLengthH+uHindLegStart, uWalkLevel, halfMotion, CMouseLeg::Stance);
    LForeLeft.StartLeg(uStepLengthF+uFrontLegStart, uWalkLevel, halfMotion, CMouseLeg::Stance);
    LForeRight.StartLeg(uFrontLegStart, uWalkLevel, halfMotion, CMouseLeg::Swing);

    //calculate Servo Values and write the points to TrottArray
    for (i=0; i<halfMotion; i++)
    {
        TrottArray[i][TIMESTAMP] = i;
        tmp = LHindLeft.GetNext();
        TrottArray[i][HINDLEFT_HIP] = tmp.leg;
        TrottArray[i][HINDLEFT_KNEE] = tmp.coil;
        tmp = LHindRight.GetNext();
        TrottArray[i][HINDRIGHT_HIP] = tmp.leg;
        TrottArray[i][HINDRIGHT_KNEE] = tmp.coil;
        tmp = LForeLeft.GetNext();
        TrottArray[i][FORELEFT_HIP] = tmp.leg;
        TrottArray[i][FORELEFT_KNEE] = tmp.coil;
        tmp = LForeRight.GetNext();
        TrottArray[i][FORERIGHT_HIP] = tmp.leg;
        TrottArray[i][FORERIGHT_KNEE] = tmp.coil;
        TrottArray[i][SPINE] = tmpSpine.spine;
        TrottArray[i][TAIL] = tmpSpine.tail;
        TrottArray[i][SPINE_FLEX] = Spine.stretch();
    }

    // Setting Leg Goals starting with Left leg forward
    LHindLeft.StartLeg(uStepLengthH+uHindLegStart, uWalkLevel, halfMotion, CMouseLeg::Stance);
    LHindRight.StartLeg(uHindLegStart, uWalkLevel, halfMotion, CMouseLeg::Swing);
    LForeLeft.StartLeg(uFrontLegStart, uWalkLevel, halfMotion, CMouseLeg::Swing);
    LForeRight.StartLeg(uStepLengthF+uFrontLegStart, uWalkLevel, halfMotion, CMouseLeg::Stance);

    //calculate Servo Values and write the points to TrottArray
    for (i=halfMotion; i<motionlength; i++)
    {
        TrottArray[i][TIMESTAMP] = i;
        tmp = LHindLeft.GetNext();
        TrottArray[i][HINDLEFT_HIP] = tmp.leg;
        TrottArray[i][HINDLEFT_KNEE] = tmp.coil;
        tmp = LHindRight.GetNext();
        TrottArray[i][HINDRIGHT_HIP] = tmp.leg;
        TrottArray[i][HINDRIGHT_KNEE] = tmp.coil;
        tmp = LForeLeft.GetNext();
        TrottArray[i][FORELEFT_HIP] = tmp.leg;
        TrottArray[i][FORELEFT_KNEE] = tmp.coil;
        tmp = LForeRight.GetNext();
        TrottArray[i][FORERIGHT_HIP] = tmp.leg;
        TrottArray[i][FORERIGHT_KNEE] = tmp.coil;
        TrottArray[i][SPINE] = tmpSpine.spine;
        TrottArray[i][TAIL] = tmpSpine.tail;
        TrottArray[i][SPINE_FLEX] = Spine.stretch();
    }

}

void CMouseCtrl::Ctrl()
{
    int motionlength = 50;
    char dir = '0';
    int state = '0';
    bool OK = true;

    clearArr(); //set everything to 90 deg, to avoid damage.

    //while(ros.OK)
    while (OK)
    {
        dir = messages;
        if (dir != -1){
            state = dir;
        }
        switch (state) {
        case 'i':
            Init();
            std::cout << "init" << std::endl;
            Print();
            break;
        case 'w':
            Trot(motionlength);
            std::cout << "Straight ahead" << std::endl;
            Print(motionlength);
            break;
        case 'a':
            std::cout<<"Left Turn"<<std::endl;
            break;
        case 's':
            std::cout<<"Backwards"<<std::endl;
            break;
        case 'd':
            std::cout<<"Right Turn"<<std::endl;
            break;
        case 'q':
            std::cout<<"Quitting"<<std::endl;
            return;
        case 'e':
            std::cout<<"DUMP"<<std::endl;
            LForeLeft.Dump();
            LForeRight.Dump();
            LHindLeft.Dump();
            LHindRight.Dump();
            break;
        }

    }
}

/*
int CMouseCtrl::getch()
{
    int c = std::cin.peek();

    if (c == EOF){
        return -1;
    }else{
    //a non blocking getchar()
    static struct termios oldt, newt;
    tcgetattr( STDIN_FILENO, &oldt);           // save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON);                 // disable buffering
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings
    int dir = getchar();  // read character (non-blocking)
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
    return dir;
    }
}
*/

void CMouseCtrl::Print(int length)
{
    std::cout << "TIMESTAMP; "
              << "FORELEFT_HIP; "
              << "FORELEFT_KNEE; "
              << "FORERIGHT_HIP; "
              << "FORERIGHT_KNEE; "
              << "HINDLEFT_HIP; "
              << "HINDLEFT_KNEE; "
              << "HINDRIGHT_HIP; "
              << "HINDRIGHT_KNEE; "
              << "SPINE; "
              << "TAIL; "
              << "SPINE_FLEX; "
              << "HEAD_PAN; "
              << "HEAD_TILT \n ";

    for(int i=0;i<length;i++)
    {
        std::cout << (int)TrottArray[i][TIMESTAMP] << "; "
                  << (int)TrottArray[i][FORELEFT_HIP] << "; "
                  << (int)TrottArray[i][FORELEFT_KNEE] << "; "
                  << (int)TrottArray[i][FORERIGHT_HIP] << "; "
                  << (int)TrottArray[i][FORERIGHT_KNEE] << "; "
                  << (int)TrottArray[i][HINDLEFT_HIP] << "; "
                  << (int)TrottArray[i][HINDLEFT_KNEE] << "; "
                  << (int)TrottArray[i][HINDRIGHT_HIP] << "; "
                  << (int)TrottArray[i][HINDRIGHT_KNEE] << "; "
                  << (int)TrottArray[i][SPINE] << "; "
                  << (int)TrottArray[i][TAIL] << "; "
                  << (int)TrottArray[i][SPINE_FLEX] << "; "
                  << (int)TrottArray[i][HEAD_PAN] << "; "
                  << (int)TrottArray[i][HEAD_TILT] << "\n ";
    }
}

void CMouseCtrl::clearArr(){
    for(int i=0;i<ArrayBuffer;i++)
    {
        TrottArray[i][TIMESTAMP] = 0 ;
        TrottArray[i][FORELEFT_HIP]= 90 ;
        TrottArray[i][FORELEFT_KNEE] = 90 ;
        TrottArray[i][FORERIGHT_HIP] = 90 ;
        TrottArray[i][FORERIGHT_KNEE] = 90 ;
        TrottArray[i][HINDLEFT_HIP] = 90 ;
        TrottArray[i][HINDLEFT_KNEE] = 90 ;
        TrottArray[i][HINDRIGHT_HIP] = 90 ;
        TrottArray[i][HINDRIGHT_KNEE] = 90 ;
        TrottArray[i][SPINE] = 90 ;
        TrottArray[i][TAIL] = 90 ;
        TrottArray[i][SPINE_FLEX] = 90 ;
        TrottArray[i][HEAD_PAN] = 90 ;
        TrottArray[i][HEAD_TILT] = 90 ;
    }
}

//starting the loop thread
void CMouseCtrl::startThread() {
    std::cout << "starting UART and MOUSE thread"<<std::endl;
    std::thread t1 ([=] { Ctrl(); });
    //t1 = std::thread { [] { Ctrl {} (); } };
    t1.detach();
    std::cout << "Walker thread detached"<<std::endl;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// leg machine methods


void CMouseLeg::StartLeg(double x, double y, int length, typPhase phase)
{
    //length is the length of the array to be filled = number of steps
    stepcount = length;
    currPhase = phase;
    StepStart(x,y);
}

void CMouseLeg::Dump()
{
    std::cout << leg << "; " << side << "; " << pawLift << "\n";
    for (int i=0;i<stepcount;i++) {
        std::cout << docu[i].x << "; "
                  << docu[i].y << "\n";
    }
}

CLegPos CMouseLeg::GetNext()
{
    if (StepNext()){
        return output;
    }else {
        return output; //currently always same output. when done, then always endposition
    }
}

void CMouseLeg::MoveTo(double x, double y)      // direkt single step to x/y, no trajectory,
{
    vx = vy = 0;
    step = stepcount = 0;   // zurücksetzen der Werte
    ptLeg = CKoord(x,y);    //aktuelle Positioen auf aaktuelles Ziel setzen
    output = SetPosition(NextWayPoint()); //Next waypoint berechnet Kinematik und in Output Speichern.
}

// Punkte werden im Voraus gerechnet (dgNext), w�hrend der Motor l�uft, besser
void CMouseLeg::StepStart(double x, double y)   // step mode by trajectory
{
    step = 0;                              // cast: Abschneiden gewollt s. jobTime
    if (stepcount <= 1)
        MoveTo(x, y);                  // if Dist < Resolution then Singlestep.
    else {
        vx = (x-ptLeg.x)/stepcount;          // compute step vector from known position (= at start!)
        vy = (y-ptLeg.y)/stepcount;
        dgNext = NextWayPoint();             // first step of kinematik move
    }
}

bool CMouseLeg::StepNext()
{
    if (++step > stepcount) return false;  // fertig
    output = SetPosition(dgNext);                   // vorausberechneten Punkt ausgeben
    if (step < stepcount)                  // wenn nicht letzter Punkt:
        dgNext = NextWayPoint();             //   neuen Punkt berechnen, solange Motor l�uft
    return true;                           // weiter gehts
}

CLegPos CMouseLeg::NextWayPoint()
{
    double X = ptLeg.x+vx, Y = ptLeg.y+vy;        // N�chsten Punkt ab current ptLeg errechnen
    if (currPhase == Swing && step > 0 && step<stepcount-1) Y += pawLift;  // Rueckweg; 1 cm anheben, letzter Step wieder runter
    return (leg == 'f') ? ikforeleg(X, Y, side)
                        : ikhindleg(X, Y, side);
}

CLegPos CMouseLeg::SetPosition(CLegPos ang)
{
    docu[step] = ptLeg;
    ptLeg.x += vx;  ptLeg.y += vy;      // Vektor auf letzten Punkt addieren
    return ang; //output zurückgeben
}

float CMouseLeg::Distance(float x1, float y1, float x2, float y2)
{
    return std::hypot((x2-x1),(y2-y1));
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// spine machine methods

double CSpine::stretch()
{
    return posStreched;
}

double CSpine::crouch()
{
    return posCrouched;
}

CSpinePos CSpine::moveLeft(int length)
{
    if(leftStart){
        leftStepsize = length/posFarLeft;
        leftStart = false;
    }
    curSP += leftStepsize;
    curTL += leftStepsize;
    if (curSP > posFarLeft) {curSP = posFarLeft; leftStart = true;}
    if (curTL > posFarLeft) {curTL = posFarLeft; leftStart = true;}

    return CSpinePos(curSP, curTL);
}

CSpinePos CSpine::moveRight(int length)
{
    if(rightStart){
        rightStepsize = length/posFarRight;
        rightStart = false;
    }
    curSP += rightStepsize;
    curTL += rightStepsize;
    if (curSP > posFarRight) {curSP = posFarRight; rightStart=true;}
    if (curTL > posFarRight) {curTL = posFarRight; rightStart=true;}

    return CSpinePos(curSP, curTL);
}

CSpinePos CSpine::centre()
{
    return CSpinePos((posCentre+cOffsetSpine), (posCentre+cOffsetTail));
}






