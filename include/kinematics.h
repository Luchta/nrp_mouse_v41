#ifndef KINEMATICS_H
#define KINEMATICS_H


class CLegPos
{
public:
    CLegPos() { leg=coil=0; }
    CLegPos(double l, double c) { leg=l; coil=c; }
    double leg;
    double coil;
};

class CKoord
{
public:
    CKoord() { x=y=0; }
    CKoord(double px, double py) { x=px; y=py; }
    double x;
    double y;
};


class CKinematics
{
public:
    CKinematics() {
      rp1=6.75;        //radius of the pulley
      
      //Foreleg
      fl1=31.0;        //l_humerus length upper arm
      fl2=26.0;        //l_ulna length forearm
      fl3=13.0;        //l_hand length hand
      ft1i=54.24;      //180° - init angle shoulder
      ft2i=87.65;      //180° - init angle elbow
      ft3i=155.0;      //init angle hand
      
      //Foreleg_Servo_Positions
      flsi=48.332; 	//flsi=41.198+(fahip*pi/180*frhip); //initial calculated length
      fltibia=13.914;  //string attachment point on tibia
      frhip=3.5;       //effective radius of the pulley
      fahip=116.79;    //initial angle of string routed on the hip
      fksainit=80;
      
      //Hindleg
      hl1=35.0;        //l_femur 	(thigh Oberschenkel)
      hl2=40.0;        //l_tibia 	(lower leg Unterschenkel)
      hl3=19.0;        //l_toe		(toe Zeh)
      ht1i=7.49;       //a_femur	(init angle femur)
      ht2i=108.93;     //a_knee	(180 - init angle knee)
      ht3i=110.0;      //a_toe		(init angle toe)
      
      //Hindleg_Servo_Positions
      hlsi=47.477;     //hlsi=37.687+(hahip*pi/180*hrhip); //initial total length
      hltibia=16.037;  //string attachment point on tibia
      hrhip=3.5;       //effective radius of the pulley
      hahip=160.26;    //initial angle of string routed on the hip
      hksainit=40;
    }

    void createWaypoints(int Ax, int Ay, int Bx, int By, int *waypoints, int lengths);
    CLegPos ikhindleg(double hx, double hy, char side);
    CLegPos ikforeleg(double fx, double fy, char side);

    void getHindLegDims(double& h1, double& h2, double& h3, double& h3i) { h1=hl1; h2=hl2; h3=hl3; h3i=ht3i; }
    void getForeLegDims(double& h1, double& h2, double& h3, double& h3i) { h1=fl1; h2=fl2; h3=fl3; h3i=ft3i; }
private:

    //Variables

    //foreleg and hindleg position matrix for trot and walk trajectory
    //double fpos_trot[5][2]={{0,0},{25,0},{50,0},{40,25},{12,20}}; 	//refined values
    //double hpos_trot[5][2]={{0,5},{25,5},{50,0},{45,35},{20,30}};	//refined values
    //double fpos_trotinit[1][2]={28,55.15};	//refined values
    //double hpos_trotinit[1][2]={20,59.15};	//refined values


    double rp1;//=6.75;           //radius of the pulley
               
    //Foreleg 
    double fl1;//=31.0;        //l_humerus length upper arm
    double fl2;//=26.0;        //l_ulna length forearm
    double fl3;//=13.0;        //l_hand length hand
    double ft1i;//=54.24;      //180° - init angle shoulder
    double ft2i;//=87.65;      //180° - init angle elbow
    double ft3i;//=155.0;      //init angle hand

    //Foreleg_Servo_Positions
    double flsi;//=48.332; 	//flsi=41.198+(fahip*pi/180*frhip); //initial calculated length
    double fltibia;//=13.914;  //string attachment point on tibia
    double frhip;//=3.5;       //effective radius of the pulley
    double fahip;//=116.79;    //initial angle of string routed on the hip
    double fksainit;//=80;

    //Hindleg
    double hl1;//=35.0;        //l_femur 	(thigh Oberschenkel)
    double hl2;//=40.0;        //l_tibia 	(lower leg Unterschenkel)
    double hl3;//=19.0;        //l_toe		(toe Zeh)
    double ht1i;//=7.49;       //a_femur	(init angle femur)
    double ht2i;//=108.93;     //a_knee	(180 - init angle knee)
    double ht3i;//=110.0;      //a_toe		(init angle toe)

    //Hindleg_Servo_Positions
    double hlsi;//=47.477;     //hlsi=37.687+(hahip*pi/180*hrhip); //initial total length
    double hltibia;//=16.037;  //string attachment point on tibia
    double hrhip;//=3.5;       //effective radius of the pulley
    double hahip;//=160.26;    //initial angle of string routed on the hip
    double hksainit;//=40;

    //Functions

    //Hindleg
    double hindKneeServoAngle(double ht1,double ht2, char side);
    double hindHipServoAngle(double ht1, char side);

    //Foreleg
    double foreKneeServoAngle(double ht1,double ht2, char side);
    double foreHipServoAngle(double ht1, char side);

    //
    double kneeservoangle(double l);
};

#endif // KINEMATICS_H
