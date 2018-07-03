#include "Arduino.h"
#include "Legs.h"

using namespace std;
//#define n_legs 4

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
    float AmpA_cor = 40;
    float AmpBeta = ampBeta;
    float AmpB_cor = 40;
    float AmpGamma = ampBeta;
    float AmpG_cor = ampGamma;
    int xb = Xb;
    int xc = Xc;
    int xd = Xd;
    //alpha = ramp(n) * (AmpAlpha / Xb)* unitstep(-n+Xb) + (AmpAlpha + (Xb -  ramp(n)) * (AmpAlpha / (Xd-Xb)) )* unitstep(n-Xb);
    alpha = ramp(n) * (AmpAlpha / Xe) - ramp(n-Xe) * (AmpAlpha / Xe) - ((ramp(n)-Xe) * (AmpAlpha / (Xg-Xe)) )* unitstep(n-Xe);
    beta  = (ramp(n-Xe) * (AmpBeta / (Xf-Xe) ) - ramp(n-Xf) * (AmpBeta / (Xf-Xe) ) - ramp(n - Xf) * (AmpBeta / (Xf-Xe)) )* unitstep(n-Xe)* unitstep(-n+Xg);
    gamma = 0;
    if(GC_correct){
        /******************************Gravity center correction***********************************/
        if(pata==3||pata==0){
            alpha+= (ramp(n-Xe+25) * (AmpA_cor / (Xf-Xe) ) - ramp(n-Xf+25) * (AmpA_cor / (Xf-Xe) ) - ramp(n - Xf+25) * (AmpA_cor / (Xf-Xe)) )* unitstep(n-Xe+25)* unitstep(-n+Xg-25);
            alpha+= -(ramp(n-Xe+75) * (AmpA_cor / (Xf-Xe) ) - ramp(n-Xf+75) * (AmpA_cor / (Xf-Xe) ) - ramp(n - Xf+75) * (AmpA_cor / (Xf-Xe)) )* unitstep(n-Xe+75)* unitstep(-n+Xg-75);    
        }
        else{
            alpha+= -(ramp(n-Xe+25) * (AmpA_cor / (Xf-Xe) ) - ramp(n-Xf+25) * (AmpA_cor / (Xf-Xe) ) - ramp(n - Xf+25) * (AmpA_cor / (Xf-Xe)) )* unitstep(n-Xe+25)* unitstep(-n+Xg-25);
            alpha+= (ramp(n-Xe+75) * (AmpA_cor / (Xf-Xe) ) - ramp(n-Xf+75) * (AmpA_cor / (Xf-Xe) ) - ramp(n - Xf+75) * (AmpA_cor / (Xf-Xe)) )* unitstep(n-Xe+75)* unitstep(-n+Xg-75);        
        }
        beta -= (ramp(n-xb) * (AmpB_cor / (xc-xb) ) - ramp(n-xc) * (AmpB_cor / (xc-xb) ) - ramp(n - xc) * (AmpB_cor / (xc-xb)) )* unitstep(n-xb)* unitstep(-n+xd);// + (ramp(n-Xd) * (AmpBeta / (Xc-Xb)) );
        gamma+= (ramp(n-xb) * (AmpG_cor / (xc-xb) ) - ramp(n-xc) * (AmpG_cor / (xc-xb) ) - ramp(n - xc) * (AmpG_cor / (xc-xb)) )* unitstep(n-xb)* unitstep(-n+xd);// + (ramp(n-Xd) * (AmpBeta / (Xc-Xb)) );

    }
    /*******************************************************************************************/
    
    //-ramp(n-Xb+25)* (AmpAlpha / (Xc-Xb)) + 2*ramp(n-Xc+25)* (AmpAlpha / (Xc-Xb)))*unitstep(-n+Xc); 
    
}

void Legs::set_phase(float Phase[]){
    for(int i=0;i<n_legs;i++)
        phase[i] = Phase[i];
}

