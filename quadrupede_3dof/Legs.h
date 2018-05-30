#include "Arduino.h"
#define n_legs 4
class Legs{
    private:
        int Total_legs;
        int DOF_leg;
        int Total_motors;

        int ciclo;
        int velocidade;
        int amplitudeAlpha;
        int amplitudeBeta;
        float phase[n_legs];
        int Xa;
        int Xb; 
        int Xc; 
        int Xd;
        int alpha, beta, gamma;


    public:

        Legs(int T_legs, int DOF);
        void walk(float dirD, float dirE, int pata);
        void set_curve_parameters(int xa, int xb, int xc, int xd);
        void set_phase(float Phase[n_legs]);

        void set_amp_alpha(int Alpha){amplitudeAlpha=Alpha;}
        void set_amp_beta(int Beta){amplitudeBeta=Beta;}
        void set_speed(int speed){velocidade=speed;}
        void set_ciclo(int c){ciclo=c;}
        int get_alpha(){return alpha;}
        int get_beta(){return beta;}
        
        //void set_phase(float phase){phase=this->phase;}
};