#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "Legs.h"

using namespace std;
#define n_legs 4

Legs::Legs(int T_legs, int DOF){
    Total_legs = T_legs;
    DOF_leg = DOF;
    Total_motors = Total_legs*DOF_leg;
}
void Legs::set_curve_parameters(int xa, int xb, int xc, int xd, int xe, int xf, int xg){
    Xa = xa;
    Xb = xb;
    Xc = xc;
    Xd = xd;
    Xe = xe;
    Xf = xf;
    Xg = xg;
}

#define unitstep(a) ((a)>=0) 
#define ramp(a) (a)*((a)>=0)

void Legs::walk(int pata) {
    if(phase[pata]>=100)
        phase[pata] = 0;
    phase[pata] += 100.0/ciclo;
    float n = phase[pata];
    float AmpAlpha = ampAlpha;
    float AmpBeta = ampBeta;
    float AmpGamma = ampGamma;
    int xb = Xb;
    int xc = Xc;
    int xd = Xd;
    //alpha = ramp(n) * (AmpAlpha / Xb)* unitstep(-n+Xb) + (AmpAlpha + (Xb -  ramp(n)) * (AmpAlpha / (Xd-Xb)) )* unitstep(n-Xb);
    alpha = ramp(n) * (AmpAlpha / Xe) - ramp(n-Xe) * (AmpAlpha / Xe) - ((ramp(n)-Xe) * (AmpAlpha / (Xg-Xe)) )* unitstep(n-Xe);
    if(pata==3||pata==0){
        alpha+= (ramp(n-Xe+25) * (AmpAlpha/2 / (Xf-Xe) ) - ramp(n-Xf+25) * (AmpAlpha/2 / (Xf-Xe) ) - ramp(n - Xf+25) * (AmpAlpha/2 / (Xf-Xe)) )* unitstep(n-Xe+25)* unitstep(-n+Xg-25);
        alpha+= -(ramp(n-Xe+75) * (AmpAlpha/2 / (Xf-Xe) ) - ramp(n-Xf+75) * (AmpAlpha/2 / (Xf-Xe) ) - ramp(n - Xf+75) * (AmpAlpha/2 / (Xf-Xe)) )* unitstep(n-Xe+75)* unitstep(-n+Xg-75);    
    }
    else{
        alpha+= -(ramp(n-Xe+25) * (AmpAlpha/2 / (Xf-Xe) ) - ramp(n-Xf+25) * (AmpAlpha/2 / (Xf-Xe) ) - ramp(n - Xf+25) * (AmpAlpha/2 / (Xf-Xe)) )* unitstep(n-Xe+25)* unitstep(-n+Xg-25);
        alpha+= (ramp(n-Xe+75) * (AmpAlpha/2 / (Xf-Xe) ) - ramp(n-Xf+75) * (AmpAlpha/2 / (Xf-Xe) ) - ramp(n - Xf+75) * (AmpAlpha/2 / (Xf-Xe)) )* unitstep(n-Xe+75)* unitstep(-n+Xg-75);        
    }    //alpha+= -(ramp(n-Xe+25) * (AmpAlpha/2 / (Xf-Xe) ) - ramp(n-Xf+25) * (AmpAlpha/2 / (Xf-Xe) ) - ramp(n - Xf+25) * (AmpAlpha/2 / (Xf-Xe)) )* unitstep(n-Xe+25)* unitstep(-n+Xg-25);
    
    //-ramp(n-Xb+25)* (AmpAlpha / (Xc-Xb)) + 2*ramp(n-Xc+25)* (AmpAlpha / (Xc-Xb)))*unitstep(-n+Xc); 
    beta  = (ramp(n-Xe) * (AmpBeta / (Xf-Xe) ) - ramp(n-Xf) * (AmpBeta / (Xf-Xe) ) - ramp(n - Xf) * (AmpBeta / (Xf-Xe)) )* unitstep(n-Xe)* unitstep(-n+Xg);
    //beta += (ramp(n-xb) * (AmpGamma / (xc-xb) ) - ramp(n-xc) * (AmpGamma / (xc-xb) ) - ramp(n - xc) * (AmpGamma / (xc-xb)) )* unitstep(n-xb)* unitstep(-n+xd);// + (ramp(n-Xd) * (AmpBeta / (Xc-Xb)) );
    
}

void Legs::set_phase(float Phase[n_legs]){
    for(int i=0;i<n_legs;i++)
        phase[i] = Phase[i];
}

