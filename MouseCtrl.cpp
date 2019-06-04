﻿#include "MouseCtrl.h"
#include <cmath> //pi and others
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <thread> // multithreading
#include <fstream> //file storage
#include <chrono> //timestamp


/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//defines for Motors in Array
#define A_TIMESTAMP 0
#define A_FORELEFT_HIP 1
#define A_FORELEFT_KNEE 2
#define A_FORERIGHT_HIP 3
#define A_FORERIGHT_KNEE 4
#define A_HINDLEFT_HIP 5
#define A_HINDLEFT_KNEE 6
#define A_HINDRIGHT_HIP 7
#define A_HINDRIGHT_KNEE 8
#define A_SPINE 9
#define A_TAIL 10
#define A_SPINE_FLEX 11
#define A_HEAD_PAN 12
#define A_HEAD_TILT 13

// motion is created via motionarray
// speed is done via amount of points to be published (old setup: 100 values at 500hz?!)

#define DEBUG false


/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// UI methods

void CMouseUI::process()
{
    std::cout << "waiting for input\n";
    do {
        _msg = getch();
        usleep(100);
    }while (_msg != 'q');
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
CMouseCtrl::CMouseCtrl()
{
    clearArr();
    startUART();
    if (DEBUG) {std::cout<<"Mouse Ctrl in Debug Mode\n";}
}

void CMouseCtrl::startCtrlThread() { //starting the loop thread
    std::cout << "starting Ctrl thread"<<std::endl;
    std::thread t1 ([=] { Ctrl(); });
    //std::thread t1 ([=] { RosCtrl(); });
    t1.detach();
    std::cout << "Mouse thread detached"<<std::endl;
}


void CMouseCtrl::Ctrl() //control setup - deprecated is only used in stand alone c++
{
    int motionlength = 50;
    //char dir = '0';

    int cmd;
    bool OK = true;

    StartTime = GetCurTime().count();

    clearArr(); //set everything to 90 deg, to avoid damage.

    //while(ros.OK)
    while (OK)
    {
        cmd = messages; //just one access to messages per run - not yet atomic!!
        //cmd = 'a';
        if (cmd != state && cmd != 0){
            //newArray = true;
            state = cmd;
        }
        //n.param("length", motionlength, 50);
        switch (state) {
        case 'i':   //initalize pose
            dir = stop;
            std::cout << "init" << std::endl;           
            Init(1);
            Publish(1);
            messages = 0;
            state = 'h';
            break;
        case 'w': //walk forward
            dir = Fwd;
            std::cout << "Straight ahead" << std::endl;
            Trot(motionlength);
            messages = 0;
            state = 'm';
            break;
        case 's': //walk backward
            dir = Bkwd;
            std::cout<<"Stop Motors"<<std::endl;
            StopAllMotors();
            messages = 0;
            state = 'h';
            break;
        case 'a': //walk left
            dir = left;
            std::cout<<"Left Turn"<<std::endl;
            Trot(motionlength);
            messages = 0;
            state = 'm';
            break;
        case 'd':   //walk right
            dir = right;
            std::cout<<"Right Turn"<<std::endl;
            Trot(motionlength);
            messages = 0;
            state = 'm';
            break;
        case 'y':   //quit programm
            std::cout<<"Sitting"<<std::endl;
            SitUp(80);
            Publish(80);
            messages = 0;
            state = 'h';
            break;
        case 'q':   //quit programm
            std::cout<<"Quitting"<<std::endl;
            StopAllMotors();
            return;
        case 'm':   //publish motions to uart
            Publish(motionlength);
            break;
        case 'h':   //idle
            usleep(90);

            break;
        case 'p':   //save motion data
            StoreFile();
            messages = 0;
            state = 'h';
            break;
        }
    }
}

int CMouseCtrl::Remap(double in) //print the array values for calculated lengthss
{
    double maxPos = 4095;
    //int minPos = 0;
    double maxDeg = 360;
    //int minDeg = 0;
    double map = (maxPos*in)/maxDeg;

    return (int)std::abs(map);
}

void CMouseCtrl::Publish(int length) //print the array values for calculated lengthss
{
    int i=0;

    for(i=0;i<length;i++)
    {
        if (DEBUG){
            Print(i);
        }else {
            SendMotorMsgs(i);

            usleep(30000); //send delay between points
        }
        //Storage
        if ((si+length) > storageBuffer){storeData = false;}
        if (storeData){
            Store(i);
        }
    }
    if (storeData){si = si + i;}

}

void CMouseCtrl::SendMotorMsgs(int i){
    ProcessSpine(SetMotorPos, ID_FORELEFT_HIP, Remap(TrottArray[i][A_FORELEFT_HIP]), 1);
    ProcessSpine(SetMotorPos, ID_FORELEFT_KNEE, Remap(TrottArray[i][A_FORELEFT_KNEE]), 1);
    ProcessSpine(SetMotorPos, ID_FORERIGHT_HIP, Remap(TrottArray[i][A_FORERIGHT_HIP]), 1);
    ProcessSpine(SetMotorPos, ID_FORERIGHT_KNEE, Remap(TrottArray[i][A_FORERIGHT_KNEE]), 1);
    ProcessSpine(SetMotorPos, ID_HINDLEFT_HIP, Remap(TrottArray[i][A_HINDLEFT_HIP]), 1);
    ProcessSpine(SetMotorPos, ID_HINDLEFT_KNEE, Remap(TrottArray[i][A_HINDLEFT_KNEE]), 1);
    ProcessSpine(SetMotorPos, ID_HINDRIGHT_HIP, Remap(TrottArray[i][A_HINDRIGHT_HIP]), 1);
    ProcessSpine(SetMotorPos, ID_HINDRIGHT_KNEE, Remap(TrottArray[i][A_HINDRIGHT_KNEE]), 1);
    ProcessSpine(SetMotorPos, ID_SPINE, Remap(TrottArray[i][A_SPINE]), 1);
    // ProcessSpine(SetMotorPos, ID_TAIL, Remap(TrottArray[i][A_TAIL]), 1);
    ProcessSpine(SetMotorPos, ID_SPINE_FLEX, Remap(TrottArray[i][A_SPINE_FLEX]), 1);
    //ProcessSpine(SetMotorPos, ID_HEAD_PAN, Remap(TrottArray[i][A_HEAD_PAN]), 1);
    //ProcessSpine(SetMotorPos, ID_HEAD_TILT, Remap(TrottArray[i][A_HEAD_TILT]), 1);
}

void CMouseCtrl::Print(int i) //print the array values for calculated lengthss
{
        std::cout << (int)TrottArray[i][A_TIMESTAMP] << "; FH:"
                  << (int)TrottArray[i][A_FORELEFT_HIP] << "; FK:"
                  << (int)TrottArray[i][A_FORELEFT_KNEE] << "; FH:"
                  << (int)TrottArray[i][A_FORERIGHT_HIP] << "; FK:"
                  << (int)TrottArray[i][A_FORERIGHT_KNEE] << "; HH:"
                  << (int)TrottArray[i][A_HINDLEFT_HIP] << "; HK:"
                  << (int)TrottArray[i][A_HINDLEFT_KNEE] << "; HH:"
                  << (int)TrottArray[i][A_HINDRIGHT_HIP] << "; HK:"
                  << (int)TrottArray[i][A_HINDRIGHT_KNEE] << "; SP:"
                  << (int)TrottArray[i][A_SPINE] << "; T:"
                  << (int)TrottArray[i][A_TAIL] << "; SF:"
                  << (int)TrottArray[i][A_SPINE_FLEX] << "; HP:"
                  << (int)TrottArray[i][A_HEAD_PAN] << "; HT: "
                  << (int)TrottArray[i][A_HEAD_TILT] << "\n ";
}

void CMouseCtrl::Store(int i){
    TimeStamp[si+i] = GetCurTime().count();
    StoreArray[si+i][A_TIMESTAMP] = si+i;
    StoreArray[si+i][A_FORELEFT_HIP] = TrottArray[i][A_FORELEFT_HIP];
    StoreArray[si+i][A_FORELEFT_KNEE] = TrottArray[i][A_FORELEFT_KNEE];
    StoreArray[si+i][A_FORERIGHT_HIP] = TrottArray[i][A_FORERIGHT_HIP];
    StoreArray[si+i][A_FORERIGHT_KNEE] = TrottArray[i][A_FORERIGHT_KNEE];
    StoreArray[si+i][A_HINDLEFT_HIP] = TrottArray[i][A_HINDLEFT_HIP];
    StoreArray[si+i][A_HINDLEFT_KNEE] = TrottArray[i][A_HINDLEFT_KNEE];
    StoreArray[si+i][A_HINDRIGHT_HIP] = TrottArray[i][A_HINDRIGHT_HIP];
    StoreArray[si+i][A_HINDRIGHT_KNEE] = TrottArray[i][A_HINDRIGHT_KNEE];
    StoreArray[si+i][A_SPINE] = TrottArray[i][A_SPINE];
    StoreArray[si+i][A_TAIL] = TrottArray[i][A_TAIL];
    StoreArray[si+i][A_SPINE_FLEX] = TrottArray[i][A_SPINE_FLEX];
    StoreArray[si+i][A_HEAD_PAN] = TrottArray[i][A_HEAD_PAN];
    StoreArray[si+i][A_HEAD_PAN] = TrottArray[i][A_HEAD_PAN];
    StoreArray[si+i][14] = state;
}

void CMouseCtrl::StoreFile() //print the array values for calculated lengthss
{
    std::ofstream myfile;
    int l;
    char buffer [18];

    //wirte new Filename
    l = sprintf (buffer, "MotionStorage_%d.txt", fileNr);
    fileNr++;
    //open File
    myfile.open (&buffer[0]);

    if (myfile.is_open())
      {
        for(int i=0;i<storageBuffer;i++)
        {
            myfile << (TimeStamp[i]-StartTime) << ","
                   << StoreArray[i][A_TIMESTAMP] << ","
                   << StoreArray[i][A_FORELEFT_HIP] << ","
                   << StoreArray[i][A_FORELEFT_KNEE] << ","
                   << StoreArray[i][A_FORERIGHT_HIP] << ","
                   << StoreArray[i][A_FORERIGHT_KNEE] << ","
                   << StoreArray[i][A_HINDLEFT_HIP] << ","
                   << StoreArray[i][A_HINDLEFT_KNEE] << ","
                   << StoreArray[i][A_HINDRIGHT_HIP] << ","
                   << StoreArray[i][A_HINDRIGHT_KNEE] << ","
                   << StoreArray[i][A_SPINE] << ","
                   << StoreArray[i][A_TAIL] << ","
                   << StoreArray[i][A_SPINE_FLEX] << ","
                   << StoreArray[i][A_HEAD_PAN] << ","
                   << StoreArray[i][A_HEAD_TILT] << ","
                   << StoreArray[i][14] << "\n ";
        }

        myfile.close();
        std::cout << "Storage Completed \n ";
      }
      else
      {
        std::cout << "Error opening file";
      }
}

std::chrono::milliseconds CMouseCtrl::GetCurTime(){
    std::chrono::milliseconds ms = std::chrono::duration_cast< std::chrono::milliseconds >(
        std::chrono::system_clock::now().time_since_epoch()
    );
    return ms;

}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// CMouseCtrl Motions

void CMouseCtrl::StopAllMotors(){
    for (int i=0;i<13;i++) {
        ProcessSpine(SetMotorOff, MotorID[i], 0, 0);
        usleep(10000);
    }
}

void CMouseCtrl::Init(int length) //initalizes all legs to zero position
{
    int i;
    CLegPos tmpLeg;
    CSpinePos tmpSpine;

    clearArr();

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
        TrottArray[i][A_TIMESTAMP] = i;
        tmpLeg = LHindLeft.GetNext();
        TrottArray[i][A_HINDLEFT_HIP] = tmpLeg.leg;
        TrottArray[i][A_HINDLEFT_KNEE] = tmpLeg.coil;
        tmpLeg = LHindRight.GetNext();
        TrottArray[i][A_HINDRIGHT_HIP] = tmpLeg.leg;
        TrottArray[i][A_HINDRIGHT_KNEE] = tmpLeg.coil;
        tmpLeg = LForeLeft.GetNext();
        TrottArray[i][A_FORELEFT_HIP] = tmpLeg.leg;
        TrottArray[i][A_FORELEFT_KNEE] = tmpLeg.coil;
        tmpLeg = LForeRight.GetNext();
        TrottArray[i][A_FORERIGHT_HIP] = tmpLeg.leg;
        TrottArray[i][A_FORERIGHT_KNEE] = tmpLeg.coil;

        TrottArray[i][A_SPINE] = tmpSpine.spine;
        TrottArray[i][A_TAIL] = tmpSpine.tail;
        TrottArray[i][A_SPINE_FLEX] = 180;
    }

}

void CMouseCtrl::SitUp(int length) //initalizes all legs to zero position
{
    int i, leng_init, leng_up, leng_to_3,leng_to_4;
    CLegPos tmpHL, tmpHR, tmpFL;
    CSpinePos tmpSpine;
    double ll = 180; //leg most forward
    double lc = 180; //coil most released

    //caculate array segmentation
    // Motion is as follows:
    // 1)Move Forlegs into backward position one by one
    // 2)Move Hindlegs forward and up while Forelegs push back
    // 3)Move Forlegs backward to rise body
    // 4)lift spine up
    // left leg to start - right leg to start - Spine to fix + Sit up
    leng_init = (int)round(length * 0.25);
    leng_to_3 = leng_init+leng_init;
    leng_to_4 = leng_to_3 + leng_init;
    //leng_up = length - l2;

    clearArr();

    //Spine positions
    tmpSpine = Spine.centre();

    //Forelg init pos:
    LForeLeft.StartLeg(0, 0, 1, CMouseLeg::Stance);
    tmpFL = LForeLeft.GetNext();
    double foreLeftInitL = tmpFL.leg;
    double foreLeftInitC = tmpFL.coil;
    //LForeRight.StartLeg(uFrontLegStart+uStepLengthF, 0, 1, CMouseLeg::Stance);
    //tmpFL = LForeRight.GetNext();
    //double foreRightInitL = tmpFL.leg;
    //double foreRightInitC = tmpFL.coil;

    //1)Move Forlegs into backward position one by one--------------------------------------------------------

    //Forward Goal backward pos
    LForeLeft.StartLeg(uFrontLegStart+uStepLengthF, 0, leng_init/2, CMouseLeg::Swing);
    LForeRight.StartLeg(uFrontLegStart+uStepLengthF, 0, leng_init/2, CMouseLeg::Swing);
    //Hindleg Goal init pose
    LHindLeft.StartLeg(uHindLegStart, 0, leng_init, CMouseLeg::Stance);
    LHindRight.StartLeg(uHindLegStart, 0, leng_init, CMouseLeg::Stance);

    //calculate Servo Values and write the points to TrottArray
    for (i=0; i<leng_init; i++)
    {
        TrottArray[i][A_TIMESTAMP] = i;
        //move Forelegs to forward pose
        tmpFL = LForeRight.GetNext();
        TrottArray[i][A_FORERIGHT_HIP] = tmpFL.leg;
        TrottArray[i][A_FORERIGHT_KNEE] = tmpFL.coil;
        if (i>(leng_init/2)){
            tmpFL = LForeLeft.GetNext();
            TrottArray[i][A_FORELEFT_HIP] = tmpFL.leg;
            TrottArray[i][A_FORELEFT_KNEE] = tmpFL.coil;
        }else {
            TrottArray[i][A_FORELEFT_HIP] = foreLeftInitL;
            TrottArray[i][A_FORELEFT_KNEE] = foreLeftInitC;
        }
        tmpHL = LHindLeft.GetNext();
        TrottArray[i][A_HINDLEFT_HIP] = 180;
        TrottArray[i][A_HINDLEFT_KNEE] = 180;
        tmpHR = LHindRight.GetNext();
        TrottArray[i][A_HINDRIGHT_HIP] = 180;
        TrottArray[i][A_HINDRIGHT_KNEE] = 180;
    }

    // 2)Move Hindlegs forward and up while Forelegs push back----------------------------------

    //Forleg Goal forward Pos
    LForeLeft.StartLeg(uFrontLegStart, 0, leng_init, CMouseLeg::Stance);
    LForeRight.StartLeg(uFrontLegStart, 0, leng_init, CMouseLeg::Stance);
    //Hindleg Goal most forward pose
    LHindLeft.StartLeg(uSitting_x, uSitting_y, leng_init, CMouseLeg::Stance);
    LHindRight.StartLeg(uSitting_x, uSitting_y, leng_init, CMouseLeg::Stance);

    for (i=leng_init; i<leng_to_3; i++)
    {
        TrottArray[i][A_TIMESTAMP] = i;
        //Forelegs push back
        tmpFL = LForeRight.GetNext();
        TrottArray[i][A_FORERIGHT_HIP] = tmpFL.leg;
        TrottArray[i][A_FORERIGHT_KNEE] = tmpFL.coil;
        tmpFL = LForeLeft.GetNext();
        TrottArray[i][A_FORELEFT_HIP] = tmpFL.leg;
        TrottArray[i][A_FORELEFT_KNEE] = tmpFL.coil;
        //Move Hindlegs forward
        tmpHL = LHindLeft.GetNext();
        TrottArray[i][A_HINDLEFT_HIP] = tmpHL.leg;
        TrottArray[i][A_HINDLEFT_KNEE] = tmpHL.coil;
        tmpHR = LHindRight.GetNext();
        TrottArray[i][A_HINDRIGHT_HIP] = tmpHR.leg;
        TrottArray[i][A_HINDRIGHT_KNEE] = tmpHR.coil;
    }

    // 3)Move Forlegs backward to rise body-------------------------------------------------------

    //Forward Goal backward pos
    LForeLeft.StartLeg(uFrontLegStart+uStepLengthF, 0, leng_init/2, CMouseLeg::Swing);
    LForeRight.StartLeg(uFrontLegStart+uStepLengthF, 0, leng_init/2, CMouseLeg::Swing);

    for (i=leng_to_3; i<leng_to_4; i++)
    {
        TrottArray[i][A_TIMESTAMP] = i;
        //Forelegs push back
        tmpFL = LForeRight.GetNext();
        TrottArray[i][A_FORERIGHT_HIP] = tmpFL.leg;
        TrottArray[i][A_FORERIGHT_KNEE] = tmpFL.coil;
        tmpFL = LForeLeft.GetNext();
        TrottArray[i][A_FORELEFT_HIP] = tmpFL.leg;
        TrottArray[i][A_FORELEFT_KNEE] = tmpFL.coil;
        //Move Hindlegs forward
        TrottArray[i][A_HINDLEFT_HIP] = TrottArray[i-1][A_HINDLEFT_HIP];
        TrottArray[i][A_HINDLEFT_KNEE] = TrottArray[i-1][A_HINDLEFT_KNEE];
        TrottArray[i][A_HINDRIGHT_HIP] = TrottArray[i-1][A_HINDRIGHT_HIP];
        TrottArray[i][A_HINDRIGHT_KNEE] = TrottArray[i-1][A_HINDRIGHT_KNEE];
    }

    //4)-Lift Spine Up-----------------------------------------------------------------------

    LHindLeft.StartLeg(30, 0, leng_init, CMouseLeg::Stance);
    LHindRight.StartLeg(30, 0, leng_init, CMouseLeg::Stance);

    double posSpineSit = 150;
    double posSpineStart = 180;
    double spineCurr = posSpineStart;
    double spineStep = (posSpineStart - posSpineSit)/(double)(length-leng_to_4); //steplength for rising spine


    for (i=leng_to_4; i<length; i++)
    {
        TrottArray[i][A_TIMESTAMP] = i;
        //move both hindlegs simultaniously to sit up body
        tmpHL = LHindLeft.GetNext();
        TrottArray[i][A_HINDLEFT_HIP] = tmpHL.leg;
        TrottArray[i][A_HINDLEFT_KNEE] = TrottArray[i-1][A_HINDLEFT_KNEE];
        tmpHR = LHindRight.GetNext();
        TrottArray[i][A_HINDRIGHT_HIP] = tmpHR.leg;
        TrottArray[i][A_HINDRIGHT_KNEE] = TrottArray[i-1][A_HINDRIGHT_KNEE];
        //keep forelegs in position
        TrottArray[i][A_FORELEFT_HIP] = TrottArray[i-1][A_FORELEFT_HIP];
        TrottArray[i][A_FORELEFT_KNEE] = TrottArray[i-1][A_FORELEFT_KNEE];
        TrottArray[i][A_FORERIGHT_HIP] = TrottArray[i-1][A_FORERIGHT_HIP];
        TrottArray[i][A_FORERIGHT_KNEE] = TrottArray[i-1][A_FORERIGHT_KNEE];

        //iterate spine to stretch to move COG backward when sitting
        spineCurr = spineCurr - spineStep;
        TrottArray[i][A_SPINE_FLEX] = spineCurr;
    }

}

void CMouseCtrl::Trot(int motionlength) //calculates trott gait
{
    //Variables
    int halfMotion = (int)round(motionlength/2);
    int i;
    bool tail = true;
    CLegPos tmp;
    CSpinePos tmpSpine;

    tmpSpine = Spine.centre();
    //Spine positions left/right/centre
    switch (dir){
    case Bkwd:
    case stop:
    case stance:
    case Fwd:
        tmpSpine = Spine.centre();
        break;
    case left:
        tmpSpine = Spine.moveStepLeft(motionlength);
        break;
    case right:
        tmpSpine = Spine.moveStepRight(motionlength);
        break;
    }

    //Setting Goals starting with Right leg forward
    LHindLeft.StartLeg(uHindLegStart, uHWalkLevel, halfMotion, CMouseLeg::Swing);
    LHindRight.StartLeg(uStepLengthH+uHindLegStart, uHWalkLevel, halfMotion, CMouseLeg::Stance);
    LForeLeft.StartLeg(uStepLengthF+uFrontLegStart, uFWalkLevel, halfMotion, CMouseLeg::Stance);
    LForeRight.StartLeg(uFrontLegStart, uFWalkLevel, halfMotion, CMouseLeg::Swing);

    //calculate Servo Values and write the points to TrottArray
    for (i=0; i<halfMotion; i++)
    {
        TrottArray[i][A_TIMESTAMP] = i;
        tmp = LHindLeft.GetNext();
        TrottArray[i][A_HINDLEFT_HIP] = tmp.leg;
        TrottArray[i][A_HINDLEFT_KNEE] = tmp.coil;
        tmp = LHindRight.GetNext();
        TrottArray[i][A_HINDRIGHT_HIP] = tmp.leg;
        TrottArray[i][A_HINDRIGHT_KNEE] = tmp.coil;
        tmp = LForeLeft.GetNext();
        TrottArray[i][A_FORELEFT_HIP] = tmp.leg;
        TrottArray[i][A_FORELEFT_KNEE] = tmp.coil;
        tmp = LForeRight.GetNext();
        TrottArray[i][A_FORERIGHT_HIP] = tmp.leg;
        TrottArray[i][A_FORERIGHT_KNEE] = tmp.coil;
        TrottArray[i][A_SPINE] = tmpSpine.spine;
        if (tail) {TrottArray[i][A_TAIL] = Spine.moveTailLeft(halfMotion);}
        else {TrottArray[i][A_TAIL] = tmpSpine.tail;}
        TrottArray[i][A_SPINE_FLEX] = Spine.stretch();
    }

    // Setting Leg Goals starting with Left leg forward
    LHindLeft.StartLeg(uStepLengthH+uHindLegStart, uHWalkLevel, halfMotion, CMouseLeg::Stance);
    LHindRight.StartLeg(uHindLegStart, uHWalkLevel, halfMotion, CMouseLeg::Swing);
    LForeLeft.StartLeg(uFrontLegStart, uFWalkLevel, halfMotion, CMouseLeg::Swing);
    LForeRight.StartLeg(uStepLengthF+uFrontLegStart, uFWalkLevel, halfMotion, CMouseLeg::Stance);

    //calculate Servo Values and write the points to TrottArray
    for (i=halfMotion; i<motionlength; i++)
    {
        TrottArray[i][A_TIMESTAMP] = i;
        tmp = LHindLeft.GetNext();
        TrottArray[i][A_HINDLEFT_HIP] = tmp.leg;
        TrottArray[i][A_HINDLEFT_KNEE] = tmp.coil;
        tmp = LHindRight.GetNext();
        TrottArray[i][A_HINDRIGHT_HIP] = tmp.leg;
        TrottArray[i][A_HINDRIGHT_KNEE] = tmp.coil;
        tmp = LForeLeft.GetNext();
        TrottArray[i][A_FORELEFT_HIP] = tmp.leg;
        TrottArray[i][A_FORELEFT_KNEE] = tmp.coil;
        tmp = LForeRight.GetNext();
        TrottArray[i][A_FORERIGHT_HIP] = tmp.leg;
        TrottArray[i][A_FORERIGHT_KNEE] = tmp.coil;
        TrottArray[i][A_SPINE] = tmpSpine.spine;
        if (tail) {TrottArray[i][A_TAIL] = Spine.moveTailRight(motionlength-halfMotion);}
        else {TrottArray[i][A_TAIL] = tmpSpine.tail;}
        TrottArray[i][A_SPINE_FLEX] = Spine.stretch();
    }

}

void CMouseCtrl::TrotBkw(int motionlength) //calculates trott gait moving backwards
{
    //Variables
    int halfMotion = motionlength/2;
    int i;
    CLegPos tmp;
    CSpinePos tmpSpine;

    tmpSpine = Spine.centre();
    //Spine positions
    switch (dir){
    case Fwd:
    case stop:
    case stance:
    case Bkwd:
        tmpSpine = Spine.centre();
        break;
    case left:
        tmpSpine = Spine.moveStepRight(motionlength);
        break;
    case right:
        tmpSpine = Spine.moveStepLeft(motionlength);
        break;
    }


    //Setting Goals starting with Right leg forward
    LHindLeft.StartLeg(uStepLengthH+uHindLegStart, uHWalkLevel, halfMotion, CMouseLeg::Swing);
    LHindRight.StartLeg(uHindLegStart, uHWalkLevel, halfMotion, CMouseLeg::Stance);
    LForeLeft.StartLeg(uFrontLegStart, uFWalkLevel, halfMotion, CMouseLeg::Stance);
    LForeRight.StartLeg(uStepLengthF+uFrontLegStart, uFWalkLevel, halfMotion, CMouseLeg::Swing);

    //calculate Servo Values and write the points to TrottArray
    for (i=0; i<halfMotion; i++)
    {
        TrottArray[i][A_TIMESTAMP] = i;
        tmp = LHindLeft.GetNext();
        TrottArray[i][A_HINDLEFT_HIP] = tmp.leg;
        TrottArray[i][A_HINDLEFT_KNEE] = tmp.coil;
        tmp = LHindRight.GetNext();
        TrottArray[i][A_HINDRIGHT_HIP] = tmp.leg;
        TrottArray[i][A_HINDRIGHT_KNEE] = tmp.coil;
        tmp = LForeLeft.GetNext();
        TrottArray[i][A_FORELEFT_HIP] = tmp.leg;
        TrottArray[i][A_FORELEFT_KNEE] = tmp.coil;
        tmp = LForeRight.GetNext();
        TrottArray[i][A_FORERIGHT_HIP] = tmp.leg;
        TrottArray[i][A_FORERIGHT_KNEE] = tmp.coil;
        TrottArray[i][A_SPINE] = tmpSpine.spine;
        TrottArray[i][A_TAIL] = tmpSpine.tail;
        TrottArray[i][A_SPINE_FLEX] = Spine.stretch();
    }

    // Setting Leg Goals starting with Left leg forward
    LHindLeft.StartLeg(uHindLegStart, uHWalkLevel, halfMotion, CMouseLeg::Stance);
    LHindRight.StartLeg(uStepLengthH+uHindLegStart, uHWalkLevel, halfMotion, CMouseLeg::Swing);
    LForeLeft.StartLeg(uStepLengthF+uFrontLegStart, uFWalkLevel, halfMotion, CMouseLeg::Swing);
    LForeRight.StartLeg(uFrontLegStart, uFWalkLevel, halfMotion, CMouseLeg::Stance);

    //calculate Servo Values and write the points to TrottArray
    for (i=halfMotion; i<motionlength; i++)
    {
        TrottArray[i][A_TIMESTAMP] = i;
        tmp = LHindLeft.GetNext();
        TrottArray[i][A_HINDLEFT_HIP] = tmp.leg;
        TrottArray[i][A_HINDLEFT_KNEE] = tmp.coil;
        tmp = LHindRight.GetNext();
        TrottArray[i][A_HINDRIGHT_HIP] = tmp.leg;
        TrottArray[i][A_HINDRIGHT_KNEE] = tmp.coil;
        tmp = LForeLeft.GetNext();
        TrottArray[i][A_FORELEFT_HIP] = tmp.leg;
        TrottArray[i][A_FORELEFT_KNEE] = tmp.coil;
        tmp = LForeRight.GetNext();
        TrottArray[i][A_FORERIGHT_HIP] = tmp.leg;
        TrottArray[i][A_FORERIGHT_KNEE] = tmp.coil;
        TrottArray[i][A_SPINE] = tmpSpine.spine;
        TrottArray[i][A_TAIL] = tmpSpine.tail;
        TrottArray[i][A_SPINE_FLEX] = Spine.stretch();
    }

}


void CMouseCtrl::clearArr(){    //clear the TrottArray

    CLegPos tmpLeg;
    int centrePos = 180;

    //initalize Leg motion with Right leg forward
    LHindLeft.StartLeg(0, 0, 1, CMouseLeg::Stance);
    LHindRight.StartLeg(0, 0, 1, CMouseLeg::Stance);
    LForeLeft.StartLeg(0, 0, 1, CMouseLeg::Stance);
    LForeRight.StartLeg(0, 0, 1, CMouseLeg::Stance);

    for(int i=0;i<ArrayBuffer;i++)
    {
        TrottArray[i][A_TIMESTAMP] = 0 ;
        tmpLeg = LHindLeft.GetNext();
        TrottArray[i][A_HINDLEFT_HIP] = tmpLeg.leg;
        TrottArray[i][A_HINDLEFT_KNEE] = tmpLeg.coil;
        tmpLeg = LHindRight.GetNext();
        TrottArray[i][A_HINDRIGHT_HIP] = tmpLeg.leg;
        TrottArray[i][A_HINDRIGHT_KNEE] = tmpLeg.coil;
        tmpLeg = LForeLeft.GetNext();
        TrottArray[i][A_FORELEFT_HIP] = tmpLeg.leg;
        TrottArray[i][A_FORELEFT_KNEE] = tmpLeg.coil;
        tmpLeg = LForeRight.GetNext();
        TrottArray[i][A_FORERIGHT_HIP] = tmpLeg.leg;
        TrottArray[i][A_FORERIGHT_KNEE] = tmpLeg.coil;
        TrottArray[i][A_SPINE] = centrePos ;
        TrottArray[i][A_TAIL] = centrePos ;
        TrottArray[i][A_SPINE_FLEX] = centrePos ;
        TrottArray[i][A_HEAD_PAN] = centrePos ;
        TrottArray[i][A_HEAD_TILT] = centrePos ;
    }
}





/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// leg machine methods

//initalizes trajectory
void CMouseLeg::StartLeg(double x, double y, int length, typPhase phase)
{
    //length is the length of the array to be filled = number of steps
    stepcount = length;
    currPhase = phase;
    if (phase == Swing){
        risetime = (int)round((double)length/5);
        riseStep = pawLift/risetime;
    }
    StepStart(x,y);
}

//dumps debugging info
void CMouseLeg::Dump()
{
    std::cout << leg << "; " << side << "; " << pawLift << "\n";
    for (int i=0;i<stepcount;i++) {
        std::cout << docu[i].x << "; "
                  << docu[i].y << "\n";
    }
}

//returns the next waypoint in the given trajectory and the endpoint if invoked after end of Trajectory
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

//invokes NextWayPoint until trajectory is finished
bool CMouseLeg::StepNext()
{
    if (++step > stepcount) return false;  // fertig
    output = SetPosition(dgNext);                   // vorausberechneten Punkt ausgeben
    if (step < stepcount)                  // wenn nicht letzter Punkt:
        dgNext = NextWayPoint();             //   neuen Punkt berechnen, solange Motor l�uft
    return true;                           // weiter gehts
}

//calculates and returns next point of trajectory - when given the swing phase it lifts the leg up
CLegPos CMouseLeg::NextWayPoint()
{
    double X = ptLeg.x+vx, Y = ptLeg.y+vy;        // N�chsten Punkt ab current ptLeg errechnen
    if ((currPhase == Swing) && (step > 0) && (step<(stepcount-1)) && leg == 'h'){
        if (step < risetime){                       //leg rises
            Y += riseStep*step;                     //rise stepwise with time
        }else if (step > (stepcount-risetime)) {    //leg is set down
            Y += riseStep*(stepcount - step);       //set down with time
        }else {
            Y += pawLift;  // else keep height
        }
    }
    //    if ((currPhase == Swing) && (step > 0) && (step<stepcount-1)){
    //            Y += pawLift;  // else keep height
    //    }
    return (leg == 'f') ? ikforeleg(X, Y, side)
                        : ikhindleg(X, Y, side);
}

// returns the servo values for the leg and sets current internal positios
CLegPos CMouseLeg::SetPosition(CLegPos ang)
{
    docu[step] = ptLeg; //documentation for debugging
    if ((currPhase == Swing) && (step > 0) && (step<(stepcount-1)) && leg == 'f'){
        (side == 'l') ? ang.coil = ang.coil-(pawLift*2)
                : ang.coil = ang.coil+(pawLift*2);
    }
    ptLeg.x += vx;  ptLeg.y += vy;      // Vektor auf letzten Punkt addieren
    return ang; //output zurückgeben
}

//calculates distance between two points
float CMouseLeg::Distance(float x1, float y1, float x2, float y2)
{
    return std::hypot((x2-x1),(y2-y1));
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// spine machine methods

//returns a stretched position value
double CSpine::stretch()
{
    return posStreched;
}

//returns a crouched position value
double CSpine::crouch()
{
    return posCrouched;
}


//move right centre to 50 and back
double CSpine::moveTailLeft(int length)
{
    //catch first function call to initialize
    if(leftTailStart){
        //set stepsize to got there and back again
        TailStepsize = (((double)RangeLeft)/((double)length/2));
        leftTailStart = false;
        dir = true;
        curTL = posTailCentre;
        return curTL;
    }
    //set new positions of the spine
    if (dir && (curTL > posTailFarLeft)){
        curTL = curTL - TailStepsize;  //go left
    }else if(!dir && (curTL < posTailCentre)) {
        curTL = curTL + TailStepsize; //go centre
    }else {
        //check wether motion completed
        if (curTL >= posTailCentre) {
            curTL = posTailCentre;
            leftTailStart = true;
            return curTL;
        }
        dir = !dir; //change direction
    }
    return curTL;
}

//move right centre to 130 and back
double CSpine::moveTailRight(int length)
{
    //catch first function call to initialize
    if(rightTailStart){
        //set stepsize to got there and back again
        TailStepsize = (((double)RangeRight)/((double)length/2));
        rightTailStart = false;
        dir = true;
        curTL = posTailCentre;
        return curTL;
    }
    //set new positions of the spine
    if (dir && (curTL < posTailFarRight)){
        curTL = curTL + TailStepsize;  //go right
    }else if(!dir && (curTL > posTailCentre)) {
        curTL = curTL - TailStepsize; //go centre
    }else {
        //check wether motion completed
        if (curTL <= posTailCentre) {
            curTL = posTailCentre;
            rightTailStart = true;
            return curTL;
        }
        dir = !dir; //change direction
    }
    return curTL;
}

//moving the Spine and Tail smoothly during walking
//every call to this funtion gives the next value for a given amount of iterations (length)
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

//moving the Spine and Tail stepwise for direction control
//every call to this funtion gives the next hihger bending iteration until the maximum bending is reached.
CSpinePos CSpine::moveStepLeft(int length)
{
    if ((curSP -= spineStep) < posFarLeft){
        curSP = posFarLeft;
    }
    return CSpinePos(curSP, curTL);
}

CSpinePos CSpine::moveStepRight(int length)
{
    curSP = curSP + spineStep;
    if (curSP > posFarRight){
        curSP = posFarRight;
    }
    return CSpinePos(curSP, curTL);
}

//centering the Spine
CSpinePos CSpine::centre()
{
    return CSpinePos((posCentre+cOffsetSpine), (posCentre+cOffsetTail));
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ROS CLASS
#if defined ROS
CMouseRos::CMouseRos()
{
    msgarr.data.resize(14); //need to declare the size, else it wont work
    pub = n.advertise<std_msgs::Float64MultiArray>("nrpmouse_servotopic", 512);
    n.setParam("length", 50);
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
    int cmd = '0';
    int state = '0';
    //bool newArray = true;
    clearArr(); //set everything to 90 deg, to avoid damage.

    while (ros::ok())
    {
        cmd = messages; //just one access to messages per run - not yet atomic!!
        if (cmd != state && cmd != 0){
            //newArray = true;
            state = cmd;
        }
        n.param("length", motionlength, 50);
        switch (state) {
        case 'i':   //initalize pose
            dir = stop;
            std::cout << "init" << std::endl;
            messages = 0;
            Init(3);
            Publish(3);
            state = 'h';
            break;
        case 'w': //walk forward
            dir = Fwd;
            std::cout << "Straight ahead" << std::endl;
            Trot(motionlength);
            messages = 0;
            state = 'm';
            break;
        case 's': //walk backward
            dir = Bkwd;
            std::cout<<"Backwards"<<std::endl;
            TrotBkw(motionlength);
            messages = 0;
            state = 'm';
            break;
        case 'a': //walk left
            dir = left;
            std::cout<<"Left Turn"<<std::endl;
            Trot(motionlength);
            messages = 0;
            state = 'm';
            break;
        case 'd':   //walk right
            dir = right;
            std::cout<<"Right Turn"<<std::endl;
            Trot(motionlength);
            messages = 0;
            state = 'm';
            break;
        case 'y':   //quit programm
            std::cout<<"Sitting"<<std::endl;
            SitUp(80);
            Publish(80);
            messages = 0;
            state = 'h';
            break;
        case 'q':   //quit programm
            std::cout<<"Quitting"<<std::endl;
            clearArr();
            Publish();
            return;
        case 'm':   //publish motions to ros
            Publish(motionlength);
            break;
        case 'h':   //idle
            usleep(90);

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

        //std::cout << (TrottArray[i][SPINE_FLEX]) << "\n";

        pub.publish(msgarr);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

#endif



