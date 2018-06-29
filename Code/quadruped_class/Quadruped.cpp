#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include "Arduino.h"
#include "Quadruped.h"

#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)
#define B_REV  (-beta_rev*2+1)

#define INV_PAR (2*(i%2)-1) // "(2*(i%2)-1)" inverte o sinal caso (i) seja par (lado esquerdo)
#define Total_motors DOFs_leg*4

Quadruped::Quadruped(int DOF){
  DOFs_leg=DOF;
  teste=0;
//  pwm.begin();//pode exigir begin -----------------------------------------------------------------------
  //pwm.setPWMFreq(60);//
}
void Quadruped::begin(){
  Serial.begin(115200);
  speed = 1000;
  pwm.begin();//pode exigir begin -----------------------------------------------------------------------
  pwm.setPWMFreq(60);//
}


void Quadruped::real_pos(char pos[]){
  int quadril[4], joelho[4], pe[4];
  for (int i = 0; i < Total_motors; i++) {
    if (i < 4)
      quadril[i] = (int)pos[i];
    else if(i<8)
      joelho[i - 4] = (int)pos[i];
    else
      pe[i - 8] = (int)pos[i];
  }
  for (int i = 0; i < 4; i++) {
    setMotorAngle(i,  quadril[i]);
    setMotorAngle(i+4,joelho[i]);
    setMotorAngle(i+8,pe[i]);
  }
}

void Quadruped::walk_pos(char pos[]){
  int quadril[4], joelho[4], pe[4];
  for (int i = 0; i < Total_motors; i++) {
    if (i < 4)
      quadril[i] = (int)pos[i];
    else if(i<8)
      joelho[i - 4] = (int)pos[i];
    else
      pe[i - 8] = (int)pos[i];
  }
  for (int i = 0; i < 4; i++) {
    /*
    if(i==0)
      setMotorAngle(i, 90 + quadril[i]);
    if(i==1)
      setMotorAngle(i, 90 + -quadril[i]);
    if(i==2)
      setMotorAngle(i, 90 + map(quadril[i],0,90,90,0));
    if(i==3)
      setMotorAngle(i, 90 + -map(quadril[i],0,90,90,0));*/
    float dir;
    int S;
    if(i==0){
      S=1;    dir = 1;}
    if(i==1){
      S=-1;   dir = 1;}
    if(i==2){
      S=1;    dir = -1;}
    if(i==3){
      S=-1;   dir = -1;}
    int quad_temp= S*(map(pos[i], 0, ampA, ampA/2 *(-dir), ampA/2 *(dir))+45);
    setMotorAngle(i, 90 + quad_temp);
    setMotorAngle(i+4, 90 + (altura[i] + B_REV*joelho[i] ) * (2*((i - 1)*(i - 3) == 0) - 1));  //"(2 * ((i-1) * (i - 3) == 0) - 1)" inverte o sinal caso (i-4) seja 1 ou 2
    if(DOFs_leg>=3)
        setMotorAngle(i+8, 90 + (altura[i] + B_REV*pe[i]     ) * (2*((i - 1)*(i - 3) == 0) - 1));      //"(2 * ((i-1) * (i - 3) == 0) - 1)" inverte o sinal caso (i-8) seja 1 ou 2
  }
}
void Quadruped::plot_curves(int l){
    walk_leg(l);
    Serial.print(alpha);
    Serial.print(',');
    Serial.print(beta);
    if(DOFs_leg>=3){
      Serial.print(',');
      Serial.print(teste);
    }
    Serial.println();
}


void Quadruped::walk(float right, float left){
  int quad_temp;
  for(int i =0; i<4; i++){
    walk_leg(i);
    /*
    if(i==0)
      quad_temp=  map(alpha, 0, ampA, ampA/2 *(-dirE + 1), ampA/2 *( dirE + 1))+45;
    if(i==1)
      quad_temp= -(map(alpha, 0, ampA, ampA/2 *(-dirD + 1), ampA/2 *( dirD + 1))+45);
    if(i==2)
      quad_temp=  map(alpha, 0, ampA, ampA/2 *( dirD + 1), ampA/2 *(-dirD + 1))+45;
    if(i==3)
      quad_temp= -(map(alpha, 0, ampA, ampA/2 *( dirE + 1), ampA/2 *(-dirE + 1))+45);
      */
    float dir;
    int S;
    if(i==0){
      S=1;    dir = left;}
    if(i==1){
      S=-1;   dir = right;}
    if(i==2){
      S=1;    dir = -right;}
    if(i==3){
      S=-1;   dir = -left;}
    quad_temp= S*(map(alpha, 0, ampA, -dir*ampA/2, dir* ampA/2)+45);
//    int quad_temp= S*(map(pos[i], 0, ampA, ampA/2 *(-dir), ampA/2 *(dir))+45);
    
    setMotorAngle(i,  90 + quad_temp);
    setMotorAngle(i+4, 90 + (altura[i] + B_REV*beta) * (2 * ((i-1) * (i-3) == 0) - 1));                  //"(2 * ((i-1) * (i - 3) == 0) - 1)" inverte o sinal caso (i-4) seja 1 ou 2
    if(DOFs_leg>=3)
      setMotorAngle(i+8, 90 + (altura[i] + B_REV*gamma) * (2 * ((i-1) * (i-3) == 0) - 1));                //"(2 * ((i-1) * (i - 3) == 0) - 1)" inverte o sinal caso (i-8) seja 1 ou 2
  }  
  delayMicroseconds(speed*100);
}


void Quadruped::setMotorAngle(uint8_t num, uint16_t degree) {
  uint16_t pulselen = map(degree, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(num, 0, pulselen);
}

void Quadruped::mover(char pos[],float dirE, float dirD) {
  int quadril[4], joelho[4], pe[4];
  for (int i = 0; i < Total_motors; i++) {
    if (i < 4)
      quadril[i] = (int)pos[i];
    else if(i<8)
      joelho[i - 4] = (int)pos[i];
    else
      pe[i - 8] = (int)pos[i];
  }
  bool Erro = 0;
  /*--------- V -------- bloquear valores absurdos ---------v ---------
  const int Max[4] = { 10,  90,  90,  90};    //{quadril[0,1],quadril[2,3],joelho[0:3],pe[0:3]   patas para frente e para baixo
  const int Min[4] = {-90, -10, -50, -50};    //                          patas para tras   e para cima

  digitalWrite(13,0);
  for (int i = 0; i < 4; i++) {
    if (quadril[i] < Min[ (i==0||i==3) ] || quadril[i] > Max[ (i==0||i==3) ]){ //i==0||i==3  Max[4] = { 10,  90,  90,  90}; 
      Erro = 1;
      Serial.print("___ERRO, quadril!__");
      Serial.println(i);
    }
    if(joelho[i] < Min[2] || joelho[i] > Max[2]){
      Erro = 1;
      Serial.print("___ERRO, joelho!__");
      Serial.println(i);
    }
    digitalWrite(13,Erro);
  }*/
  /*---------------------------------------------*/
  if (!Erro) {
  int quad_temp;
    for (int i = 0; i < 4; i++) { //portas do "quadril" do hub de servos
      /*setMotorAngle(i, 90 - quadril[i] * (2 * (i % 2) - 1)); // "(2*(i%2)-1)" inverte o sinal caso (i) seja impar (lado direito)
      // quando o valor de quadril[i] aumenta, a pata [i] vai para frente
      */
      //if(!de_lado){
        if(i==0)
          quad_temp= dirE * quadril[0];
        if(i==1)
          quad_temp= dirD * quadril[1];
        if(i==2)
          quad_temp= dirD * -quadril[2];
        if(i==3)
          quad_temp= dirE * -quadril[3];
        setMotorAngle(i, 90 + quad_temp);
      //}
      setMotorAngle(i+4, 90 + (altura[i]+joelho[i]) * (2 * ((i-1) * (i - 3) == 0) - 1));  //"(2 * ((i-1) * (i - 3) == 0) - 1)" inverte o sinal caso (i-4) seja 1 ou 2
      if(DOFs_leg>=3)
          setMotorAngle(i+8, 90 + (altura[i]+pe[i]) * (2 * ((i-1) * (i - 3) == 0) - 1));      //"(2 * ((i-1) * (i - 3) == 0) - 1)" inverte o sinal caso (i-8) seja 1 ou 2
   
    }
  }
}




void Quadruped::set_curve_parameters(int xa, int xb, int xc, int xd, int xe, int xf, int xg){
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

void Quadruped::walk_leg(int pata) {
    if(phase[pata]>=100)
        phase[pata] = 0;
    phase[pata] += 100.0/resolution;
    float n = phase[pata];
    float AmpAlpha = ampA;
    float AmpA_cor = 40;
    float AmpBeta = ampB;
    float AmpB_cor = 40;
    float AmpGamma = ampB;
    float AmpG_cor = ampG;
    int xb = Xb;
    int xc = Xc;
    int xd = Xd;
    //alpha = ramp(n) * (AmpAlpha / Xb)* unitstep(-n+Xb) + (AmpAlpha + (Xb -  ramp(n)) * (AmpAlpha / (Xd-Xb)) )* unitstep(n-Xb);
    //alpha = ramp(n) * (AmpAlpha / Xe) - ramp(n-Xe) * (AmpAlpha / Xe) - ((ramp(n)-Xe) * (AmpAlpha / (Xg-Xe)) )* unitstep(n-Xe);
    alpha = AmpAlpha*( ramp(n)/Xe  - ramp(n-Xe)/Xe - ramp(n-Xe)/(Xg-Xe));

    beta  = (ramp(n-Xe) * (AmpBeta / (Xf-Xe) ) - ramp(n-Xf) * (AmpBeta / (Xf-Xe) ) - ramp(n - Xf) * (AmpBeta / (Xg-Xf)) )* unitstep(n-Xe)* unitstep(-n+Xg);
    gamma = beta;
  /******************************Gravity center correction***********************************/
    if(GC_correct){
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

void Quadruped::set_phase(float Phase[4]){
    for(int i=0;i<4;i++)
        phase[i] = Phase[i];
}
