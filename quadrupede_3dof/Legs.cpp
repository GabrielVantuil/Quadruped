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
void Legs::set_curve_parameters(int xa, int xb, int xc, int xd){
    Xa = xa;
    Xb = xb;
    Xc = xc;
    Xd = xd;
}

void Legs::walk(float dirD, float dirE, int pata) {
    //float phase2;
    //for (int i = 0; i < Total_legs; i++) {
        phase[pata] = (int)(phase[pata] + 1) % ciclo;
        float amostra = (phase[pata] * 100) / ciclo;
        if (amostra <= Xb)
            beta =  0;
        else if (amostra <= Xc)
            beta =  map(amostra, Xb, Xc, 0, amplitudeBeta);

        else if (amostra > Xc)
            beta =  map(amostra, Xc, Xd, amplitudeBeta, 0);

        if (amostra >= Xb)
            alpha =  map(amostra, Xb, Xd, amplitudeAlpha , 0);
        else if (amostra < Xb)
            alpha =  map(amostra, Xa, Xb, 0, amplitudeAlpha);

        if (pata < Total_legs/2)  //direito
            alpha = map(alpha, 0, amplitudeAlpha, amplitudeAlpha, 0);

        alpha -= amplitudeAlpha / 2;
        if (pata < Total_legs/2)              //direito
            alpha = dirD * alpha;
        else                 //esquerdo
            alpha = dirE * alpha;

        if (alpha > amplitudeAlpha / 2)           //filtro
            alpha = amplitudeAlpha / 2;
        else if (alpha < -amplitudeAlpha / 2)     //filtro
            alpha = -amplitudeAlpha / 2;

        
/*        setMotorAngle(i, alpha);
        setMotorAngle(i+4, beta);
    }
    delay(velocidade); 
    */
}

void Legs::set_phase(float Phase[n_legs]){
    for(int i=0;i<n_legs;i++){
        phase[i] = Phase[i];
    }
}

