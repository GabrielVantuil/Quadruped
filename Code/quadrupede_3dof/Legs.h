#include "Arduino.h"
#define n_legs 4
class Legs{
    private:
        int Total_legs;
        int DOF_leg;
        int Total_motors;

        int ciclo;
        int velocidade;
        int ampAlpha;
        int ampBeta, ampGamma;
        float phase[n_legs];
        int Xa, Xb, Xc, Xd, Xe, Xf, Xg;
        int alpha;
        int beta, gamma;


    public:

        Legs(int T_legs, int DOF);
        void walk(int pata);
        void set_curve_parameters(int xa, int xb, int xc, int xd, int xe, int xf, int xg);
        void set_phase(float Phase[n_legs]);

        void set_amp_alpha(int Alpha){ampAlpha=Alpha;}
        void set_amp_beta(int Beta){ampBeta=Beta;}
        void set_amp_gamma(int Gamma){ampGamma=Gamma;}
        void set_speed(int speed){velocidade=speed;}
        void set_ciclo(int c){ciclo=c;}
        int get_alpha(){return alpha;}
        int get_beta(){return beta;}
        int get_gamma(){return gamma;}
        
        //void set_phase(float phase){phase=this->phase;}
};