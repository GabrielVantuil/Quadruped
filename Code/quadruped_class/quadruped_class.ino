//#include <Wire.h>
#include <EEPROM.h>
//#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>
//#include "Legs.h"
#include "Quadruped.h"

#define Total_legs  4
#define DOF_leg 3
#define Total_motors Total_legs*DOF_leg

SoftwareSerial bluetooth(10, 11); // RX, TX


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
    ____________|         |______________
   |_11______7__|3       2|______6___10__| 
                |_________|           
   pe^ joelho^   ^quadril^ 
*/
const int pos_lado[Total_motors]={-70,-70,70,70,  0,0,0,0};
//int altura[4] = {0,0,0,0};
//bool de_lado = 0;
//bool modo_omni=0;
//float phase[4] = {0, 0, 0, 0};
//float phase[4] = {0, 25, 50, 75};
float phase[4] = {0, 50, 0, 50};

char S;
char p40[Total_motors]    = {40,-40,-40,40, 40,40,40,40};
char teste[Total_motors]  = {40,40,40,40, 40,40,40,40};

Quadruped quadruped(3);

void setup(){
  //float phase[4] = {0, 0, 0, 0};
  //float phase[4] = {0, 25, 50, 75};
  float phase[4] = {0, 50, 0, 50};

//  legs.set_curve_parameters(0,30,40,50, 80, 90, 100);
  //Serial.begin(115200);
  quadruped.set_curve_parameters(0,24,37.5,50, 60, 80, 100);
  quadruped.set_phase(phase);
  quadruped.set_amp_alpha(30);
  quadruped.set_amp_beta(40);
  quadruped.set_speed(100);
  quadruped.set_resolution(30);
  quadruped.begin();
  S='1';
  //bluetooth.begin(9600);
}

void loop() {
  /*quadruped.plot_curves(0);
  if(quadruped.get_alpha() == 0){
    int t=0;
      t=millis();
    while((millis()-t)>4000){ 
      Serial.println(0);
    } 
    delay(5000);
  }
}
*/
  if(Serial.available())
    S=Serial.read();
  
  if(S=='0')
    quadruped.mover(p40,1,1);
  
  if(S=='g')
    quadruped.real_pos(teste);
  if(S=='t'){
    char a[Total_motors] = {0,0,0,0, 0,0,0,0};
    quadruped.walk_pos(a);
  }
  if(S=='h')
    quadruped.walk_pos(teste);
  
  if(S=='u'){
    char a[Total_motors] = {-60,-30,0,30, 0,0,0,0};
    quadruped.walk_pos(a);
  }
  if(S=='w' || S=='s' || S=='d' || S=='a' || S=='e' || S=='q'){
    if(S=='w')
      quadruped.walk(1,1);
    if(S=='s')
      quadruped.walk(-1,-1);
    if(S=='d')
      quadruped.walk(1,-1);
    if(S=='a')
      quadruped.walk(-1,1);
  }
  if(S=='S'){
    for(int i=0; i<4;i++)
      quadruped.altura[i] +=10;
    S='1';
  }
  if(S=='D'){
    for(int i=0; i<4;i++)
      quadruped.altura[i]-=10;
    S='1';
  }
  if(S=='P'){
    for(int i=0; i<4;i++)
      quadruped.altura[i]=0;
    S='1';
  }

  if(S=='2'){
    for(int i=0; i<4;i++)
      quadruped.altura[i] = 50;
    S='1';
  }
  if(S=='3'){
    for(int i=0; i<4;i++)
      quadruped.altura[i] =-50;
    S='1';
  }
  if(S=='5'){
    for(int i=0; i<2;i++)
      quadruped.altura[i] +=10;
    for(int i=2; i<4;i++)
      quadruped.altura[i] -=10;
    S='1';
  }
  if(S=='6'){
    for(int i=0; i<2;i++)
      quadruped.altura[i] -=10;
    for(int i=2; i<4;i++)
      quadruped.altura[i] +=10;
    S='1';
  }
  if(S=='1'){
    char a[Total_motors] = {0,0,0,0, 0,0,0,0};
    quadruped.mover(a,1,1);
  }
  
  if(S=='v'){
    Serial.print("new speed: ");
    while(!Serial.available());
      
    S = Serial.read();

    if(S=='0')
      quadruped.set_speed(0);
    if(S=='1')
      quadruped.set_speed(1);
    if(S=='2')
      quadruped.set_speed(10);
    if(S=='3')
      quadruped.set_speed(100);
    if(S=='4')
      quadruped.set_speed(1000);
    if(S=='5')
      quadruped.set_speed(5000);
    if(S=='6')
      quadruped.set_speed(10000);
    if(S=='7')
      quadruped.set_speed(50000);
    if(S=='8')
      quadruped.set_speed(100000);
    if(S=='9')
      quadruped.set_speed(500000);
    Serial.println(S);
    S='1';
  }

  if(S=='z'){
    Serial.println("----------test mode---------");
    Serial.println("press 'e' for exit");
    while(S!='e'){
      S = Serial.read();
      if(S=='0'){
        char a[Total_motors] = {10,10,-10,-10, 0,0,0,0};
        quadruped.mover(a,1,1);
      }
      if(S=='1'){
        char a[Total_motors] = {-90,-90,90,90, 0,0,0,0};
        quadruped.mover(a,1,1);
      }
      if(S=='2'){
        char a[Total_motors] = {0,0,0,0, -50,-50,-50,-50};
        quadruped.mover(a,1,1);
      }
      if(S=='3'){
        char a[Total_motors] = {0,0,0,0, 90,90,90,90};
        quadruped.mover(a,1,1);
      }
    }
    Serial.println("----------Normal mode---------");
    S='0';
  }
    
}




/**/
