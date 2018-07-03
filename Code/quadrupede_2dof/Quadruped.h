#include "Arduino.h"
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

class Quadruped{
    private:
        int speed = 20;
        int DOFs_leg;
        Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

        int ciclo;
        int ampA;
        int ampB; 
        int ampG;
        float phase[];
        int Xa, Xb, Xc, Xd, Xe, Xf, Xg;
        int alpha;
        int beta, gamma;
        bool GC_correct=0;
        
    public:
        Quadruped(int DOF);
        int altura[4] = {0,0,0,0};
        void real_pos(char pos[]);
        void walk_pos(char pos[]);
        void plot_curves(int l, int baud);
        void walk(float dirD, float dirE);
        void setMotorAngle(uint8_t num, uint16_t degree);
        void mover(char pos[],float dirE, float dirD);
        void begin();
//////////////////////////////////////////////////////
        void walk_leg(int pata);
        void set_curve_parameters(int xa, int xb, int xc, int xd, int xe, int xf, int xg);
        void set_phase(float Phase[]);
        void set_amp_alpha(int Alpha){ampA =Alpha;}
        void set_amp_beta(int Beta){ampB = Beta;}
        void set_amp_gamma(int Gamma){ampG = Gamma;}
        void set_speed(int s){speed=s;}
        void set_ciclo(int c){ciclo=c;}
/*        int get_alpha(){return alpha;}
        int get_beta(){return beta;}
        int get_gamma(){return gamma;}
  */      

};