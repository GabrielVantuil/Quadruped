#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>
#include "Legs.h"

#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)

#define Total_legs  4
#define DOF_leg 3
#define Total_motors Total_legs*DOF_leg

#define ciclo 100
#define velocidade 20
#define amplitudeAlpha 60
#define amplitudeBeta 50
#define amplitudeGamma 20

SoftwareSerial bluetooth(10, 11); // RX, TX
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Legs legs(Total_legs,DOF_leg);


#define INV_PAR (2*(i%2)-1) // "(2*(i%2)-1)" inverte o sinal caso (i) seja par (lado esquerdo)
/*     Frente
  As portas 0,1,2,3 são os "quadril"
    90 graus (do servo) é a posição paralela aos "olhos" do robô (ou seja, perpendicular ao corpo)
    90º do servo é equivalente à 0 na função "mover"
  As portas 4,5,6,7 são os "joelho"
    0º da função é a posição em que as patas ficam na altura da base do robô
  As portas 8,9,10,11 são os "pe"
    0º da função é a posição em que as patas ficam na altura da base do robô
                _________            
   ____________|         |_____________  
  |__8______4__|0       1|______5___9__| 
               |         |            
   ____________|         |_____________  
  |_11______7__|3       2|______6___10__| 
               |_________|           
  pe^ joelho^   ^quadril^ 
*/
const int Max[4] = { 10,  90,  90,  90};    //{quadril[0,1],quadril[2,3],joelho[0:3],pe[0:3]   patas para frente e para baixo
const int Min[4] = {-90, -10, -50, -50};    //                          patas para tras   e para cima
const int pos_lado[12]={-70,-70,70,70,  0,0,0,0,  0,0,0,0};
int altura[4] = {0,0,0,0};
bool de_lado = 0;
bool modo_omni=0;
float phase[4] = {0, 25, 50, 75};
//float phase[4] = {50, 0, 0, 50};
char S;
char p40[12]={40,-40,-40,40, 40,40,40,40, 40,40,40,40};
char teste[12]={40,40,40,40, 40,40,40,40, 40,40,40,40};
//float phase[4] = {50, 0, 50, 0};
  
void setup(){
  legs.set_curve_parameters(0,30,40,50, 80, 90, 100);
  legs.set_phase(phase);
  legs.set_amp_alpha(amplitudeAlpha);
  legs.set_amp_beta(amplitudeBeta);
  legs.set_amp_gamma(amplitudeGamma);
  legs.set_speed(velocidade);
  legs.set_ciclo(ciclo);


  pwm.begin();
  pwm.setPWMFreq(60);
  Serial.begin(9600);
  /*for(int i=0;i<Total_motors;i++){
    setMotorAngle(i,90);
  }
  delay(2000);
  */
  S='1';
  bluetooth.begin(9600);
}
void real_pos(char pos[Total_motors]){
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
    setMotorAngle(i,  90 + quadril[i]);
    setMotorAngle(i+4,90 + joelho[i]);
    setMotorAngle(i+8,90 + pe[i]);

  }


}
void walk_pos(char pos[Total_motors]){
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
    if(i==0)
      setMotorAngle(i, 90 + quadril[i]);
    if(i==1)
      setMotorAngle(i, 90 + -quadril[i]);
    if(i==2)
      setMotorAngle(i, 90 + map(quadril[i],0,90,90,0));
    if(i==3)
      setMotorAngle(i, 90 + -map(quadril[i],0,90,90,0));
      
    setMotorAngle(i+4, 90 + (altura[i] + joelho[i] ) * (2*((i - 1)*(i - 3) == 0) - 1));  //"(2 * ((i-1) * (i - 3) == 0) - 1)" inverte o sinal caso (i-4) seja 1 ou 2
    setMotorAngle(i+8, 90 + (altura[i] + pe[i]     ) * (2*((i - 1)*(i - 3) == 0) - 1));      //"(2 * ((i-1) * (i - 3) == 0) - 1)" inverte o sinal caso (i-8) seja 1 ou 2
  }
}
void plot_curves(){
  //for(int i =0; i<Total_legs; i++){
    legs.walk(0);
    Serial.print(legs.get_alpha());
    Serial.print(',');
    Serial.print(legs.get_beta());
    Serial.println();
  //}
}

void walk(float dirD, float dirE){
  int quad_temp;
  for(int i =0; i<Total_legs; i++){
    legs.walk(i);
    if(!de_lado){
      if(i==0)
        quad_temp= dirE * legs.get_alpha();
      if(i==1)
        quad_temp= dirD * -legs.get_alpha();
      if(i==2)
        quad_temp= dirD * map(legs.get_alpha(),0,amplitudeAlpha,amplitudeAlpha,0);
      if(i==3)
        quad_temp= dirE * -map(legs.get_alpha(),0,amplitudeAlpha,amplitudeAlpha,0);
      setMotorAngle(i, 90 + quad_temp);
    }
    Serial.print(quad_temp);
    setMotorAngle(i+4, 90 + (altura[i]+ -legs.get_beta()) * (2 * ((i-1) * (i - 3) == 0) - 1));  //"(2 * ((i-1) * (i - 3) == 0) - 1)" inverte o sinal caso (i-4) seja 1 ou 2
    setMotorAngle(i+8, 90 + (altura[i]+ -legs.get_beta()) * (2 * ((i-1) * (i - 3) == 0) - 1));      //"(2 * ((i-1) * (i - 3) == 0) - 1)" inverte o sinal caso (i-8) seja 1 ou 2
    Serial.print(',');
  }
  Serial.println();
  
  delay(velocidade);
}

void setMotorAngle(uint8_t num, uint16_t degree) {
  uint16_t pulselen = map(degree, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(num, 0, pulselen);
}

void mover(char pos[Total_motors],float dirE, float dirD) {
  int quadril[4], joelho[4], pe[4];
  for (int i = 0; i < Total_motors; i++) {
    if (i < 4)
      quadril[i] = (int)pos[i];
    else if(i<8)
      joelho[i - 4] = (int)pos[i];
    else
      pe[i - 8] = (int)pos[i];
  }
  //-----v------bloquear valores absurdos-----v-------
  digitalWrite(13,0);
  bool Erro = 0;
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

    if(pe[i] < Min[3]  ||  pe[i] > Max[3]){
      Erro = 1;
      Serial.print("___ERRO, pe!__");
      Serial.println(i);
    }
    digitalWrite(13,Erro);
  }
  //---------------------------------------------------
  if (!Erro) {
  int quad_temp;
    for (int i = 0; i < 4; i++) { //portas do "quadril" do hub de servos
      /*setMotorAngle(i, 90 - quadril[i] * (2 * (i % 2) - 1)); // "(2*(i%2)-1)" inverte o sinal caso (i) seja impar (lado direito)
      // quando o valor de quadril[i] aumenta, a pata [i] vai para frente
      */
      if(!de_lado){
        if(i==0)
          quad_temp= dirE * quadril[0];
        if(i==1)
          quad_temp= dirD * quadril[1];
        if(i==2)
          quad_temp= dirD * -quadril[2];
        if(i==3)
          quad_temp= dirE * -quadril[3];
        setMotorAngle(i, 90 + quad_temp);
      }
      setMotorAngle(i+4, 90 + (altura[i]+joelho[i]) * (2 * ((i-1) * (i - 3) == 0) - 1));  //"(2 * ((i-1) * (i - 3) == 0) - 1)" inverte o sinal caso (i-4) seja 1 ou 2
      setMotorAngle(i+8, 90 + (altura[i]+pe[i])     * (2 * ((i-1) * (i - 3) == 0) - 1));      //"(2 * ((i-1) * (i - 3) == 0) - 1)" inverte o sinal caso (i-8) seja 1 ou 2
   
    }/*
    for (int i = 4; i < 8; i++) { //portas do hub de servos
      setMotorAngle(i, 90 + (altura[i-4]+joelho[i - 4]) * (2 * ((i - 5) * (i - 7) == 0) - 1)); //"(2*((i-4)*(i-7)==0)-1)" inverte o sinal caso (i-4) seja 1 ou 2
      // quando o valor de joelho[i] aumenta, a pata [i] vai para baixo
    }
    for (int i = 8; i < Total_motors; i++) { //portas do hub de servos
      setMotorAngle(i, 90 + (altura[i-8]+pe[i - 8]) * (2 * ((i - 9) * (i - 11) == 0) - 1)); //"(2*((i-4)*(i-7)==0)-1)" inverte o sinal caso (i-8) seja 1 ou 2
      // quando o valor de joelho[i] aumenta, a pata [i] vai para baixo
    }*/
  }
}

const int qnt_pos = 65;
char pos[qnt_pos][Total_motors] = {  // anda para frente
  {-14,13,13,-14,27,0,0,27,0,0,0,0},
  {-11,10,10,-11,30,0,0,30,0,0,0,0},
  {-8,7,7,-8,32,0,0,32,0,0,0,0},
  {-5,4,4,-5,35,0,0,35,0,0,0,0},
  {-2,1,1,-2,36,0,0,36,0,0,0,0},
  {1,-2,-2,1,38,0,0,38,0,0,0,0},
  {4,-5,-5,4,39,0,0,39,0,0,0,0},
  {6,-7,-7,6,39,0,0,39,0,0,0,0},
  {9,-10,-10,9,39,0,0,39,0,0,0,0},
  {12,-13,-13,12,39,0,0,39,0,0,0,0},
  {15,-16,-16,15,39,0,0,39,0,0,0,0},
  {17,-18,-18,17,38,0,0,38,0,0,0,0},
  {20,-21,-21,20,36,0,0,36,0,0,0,0},
  {22,-23,-23,22,34,0,0,34,0,0,0,0},
  {24,-25,-25,24,32,0,0,32,0,0,0,0},
  {25,-26,-26,25,30,0,0,30,0,0,0,0},
  {27,-28,-28,27,27,0,0,27,0,0,0,0},
  {28,-29,-29,28,24,0,0,24,0,0,0,0},
  {29,-30,-30,29,21,3,3,21,0,0,0,0},
  {29,-30,-30,29,17,7,7,17,0,0,0,0},
  {29,-30,-30,29,13,11,11,13,0,0,0,0},
  {29,-30,-30,29,10,15,15,10,0,0,0,0},
  {29,-30,-30,29,6,18,18,6,0,0,0,0},
  {28,-29,-29,28,2,22,22,2,0,0,0,0},
  {27,-28,-28,27,0,25,25,0,0,0,0,0},
  {26,-27,-27,26,0,28,28,0,0,0,0,0},
  {25,-26,-26,25,0,30,30,0,0,0,0,0},
  {23,-24,-24,23,0,33,33,0,0,0,0,0},
  {21,-22,-22,21,0,35,35,0,0,0,0,0},
  {19,-20,-20,19,0,36,36,0,0,0,0,0},
  {17,-18,-18,17,0,38,38,0,0,0,0,0},
  {14,-15,-15,14,0,39,39,0,0,0,0,0},
  {11,-12,-12,11,0,39,39,0,0,0,0,0},
  {9,-10,-10,9,0,39,39,0,0,0,0,0},
  {6,-7,-7,6,0,39,39,0,0,0,0,0},
  {3,-4,-4,3,0,39,39,0,0,0,0,0},
  {0,-1,-1,0,0,38,38,0,0,0,0,0},
  {-3,2,2,-3,0,36,36,0,0,0,0,0},
  {-6,5,5,-6,0,35,35,0,0,0,0,0},
  {-9,8,8,-9,0,32,32,0,0,0,0,0},
  {-12,11,11,-12,0,30,30,0,0,0,0,0},
  {-15,14,14,-15,0,27,27,0,0,0,0,0},
  {-17,16,16,-17,0,24,24,0,0,0,0,0},
  {-20,19,19,-20,0,21,21,0,0,0,0,0},
  {-22,21,21,-22,0,18,18,0,0,0,0,0},
  {-24,23,23,-24,0,14,14,0,0,0,0,0},
  {-26,25,25,-26,0,10,10,0,0,0,0,0},
  {-27,26,26,-27,0,7,7,0,0,0,0,0},
  {-28,27,27,-28,0,3,3,0,0,0,0,0},
  {-29,28,28,-29,0,0,0,0,0,0,0,0},
  {-30,29,29,-30,0,0,0,0,0,0,0,0},
  {-30,29,29,-30,0,0,0,0,0,0,0,0},
  {-30,29,29,-30,0,0,0,0,0,0,0,0},
  {-30,29,29,-30,0,0,0,0,0,0,0,0},
  {-30,29,29,-30,0,0,0,0,0,0,0,0},
  {-29,28,28,-29,1,0,0,1,0,0,0,0},
  {-28,27,27,-28,5,0,0,5,0,0,0,0},
  {-26,25,25,-26,9,0,0,9,0,0,0,0},
  {-25,24,24,-25,13,0,0,13,0,0,0,0},
  {-23,22,22,-23,16,0,0,16,0,0,0,0},
  {-21,20,20,-21,20,0,0,20,0,0,0,0},
  {-19,18,18,-19,23,0,0,23,0,0,0,0},
  {-16,15,15,-16,26,0,0,26,0,0,0,0},
  {-13,12,12,-13,29,0,0,29,0,0,0,0},
  {-11,10,10,-11,32,0,0,32,0,0,0,0}
};
void loop() {
  //plot_curves();
  
  if(Serial.available()){
    S=Serial.read();
  }
  if(S=='0')
    mover(p40,1,1);
  
  if(S=='g')
    real_pos(teste);
  if(S=='h')
    walk_pos(teste);
  
  if(S=='w' || S=='s' || S=='d' || S=='a' || S=='e' || S=='q'){
    if(S=='w')
        walk(1,1);
        //mover(pos[t],1,1);
      for (int t = qnt_pos; t>0; t--) {   
        if(S=='s')
          mover(pos[t],-1,-1);
        if(!modo_omni){
          if(S=='d')
            mover(pos[t],1,-1);
          if(S=='a')
            mover(pos[t],-1,1);
          if(S=='e')
            mover(pos[t],1,0.5);
          if(S=='q')
            mover(pos[t],0.5,1);
        }
        else{
          de_lado=1;
          if(S=='d')
            mover(pos[t],1,1);
          if(S=='a')
            mover(pos[t],-1,-1);
          de_lado=0;
        }
    }
  }
  if(S=='L'){
    de_lado=1;
    modo_omni=0;
    S='1';
  }
  if(S=='N'){
    de_lado=0;
    modo_omni=0;
    S='1';
  }
  if(S=='O'){
    de_lado=0;
    modo_omni=1;
    S='1';
  }
  if(S=='S'){
    for(int i=0; i<4;i++)
      altura[i] +=10;
    S='1';
  }
  if(S=='D'){
    for(int i=0; i<4;i++)
      altura[i]-=10;
    S='1';
  }
  if(S=='P'){
    for(int i=0; i<4;i++)
      altura[i]=0;
    S='1';
  }

  if(S=='2'){
    for(int i=0; i<4;i++)
      altura[i] = 50;
    S='1';
  }
  if(S=='3'){
    for(int i=0; i<4;i++)
      altura[i] =-50;
    S='1';
  }
  if(S=='5'){
    for(int i=0; i<2;i++)
      altura[i] +=10;
    for(int i=2; i<4;i++)
      altura[i] -=10;
    S='1';
  }
  if(S=='6'){
    for(int i=0; i<2;i++)
      altura[i] -=10;
    for(int i=2; i<4;i++)
      altura[i] +=10;
    S='1';
  }
  if(S=='7'){
    for (int t = qnt_pos; t>0; t--) {   
      char pos3[Total_motors] = {0,0,0,0, 20,20,20,20, 0,0,0,0};
      for(int i=0;i<4;i++)
          pos3[i] = 1.5*(int)pos[t][((i==1)||(i==3))];
      mover((int)pos3,1,1);
    }
  }
  /*if(S=='9'){
    for(int i=0;i<3;i++){
      mover(pos2[i],1,1);
      delay(1000);
    }
    S='l';
  }*/
  if(S=='1'){
    char a[Total_motors] = {0,0,0,0, 0,0,0,0, 0,0,0,0};
    mover(a,1,1);
  }
  if(S=='z'){
    Serial.println("----------test mode---------");
    Serial.println("press 'e' for exit");
    while(S!='e'){
      S = Serial.read();
      if(S=='0'){
        char a[Total_motors] = {10,10,-10,-10, 0,0,0,0, 0,0,0,0};
        mover(a,1,1);
      }
      if(S=='1'){
        char a[Total_motors] = {-90,-90,90,90, 0,0,0,0, 0,0,0,0};
        mover(a,1,1);
      }
      if(S=='2'){
        char a[Total_motors] = {0,0,0,0, 0,0,0,0, 90,90,90,90};
        mover(a,1,1);
      }
      if(S=='3'){
        char a[Total_motors] = {0,0,0,0, 0,0,0,0, -50,-50,-50,-50};
        mover(a,1,1);
      }
      if(S=='4'){
        char a[Total_motors] = {0,0,0,0, 90,90,90,90, 0,0,0,0};
        mover(a,1,1);
      }
      if(S=='5'){
        char a[Total_motors] = {0,0,0,0, -50,-50,-50,-50, 0,0,0,0};
        mover(a,1,1);
      }
    }
    Serial.println("----------Normal mode---------");
    S='0';
  }
    
}




