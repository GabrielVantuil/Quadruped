#include <Servo.h>
Servo quadril, joelho, pe;
void setup() {
  quadril.attach(9);
  joelho.attach(10);
  pe.attach(11);
  Serial.begin(9600);
}
char c;
int pos2 = 130;
int pos = 30;
void loop() {
  if(Serial.available()>0){
    c = Serial.read();
  }
  if(c =='1')
    for (pos = 30,pos2=130; pos <= 130; pos += 1, pos2--) {
      joelho.write(pos);            
      pe.write(pos2);
      delay(10);             
    }
  if(c =='9'){
    for (pos = 30,pos2=40; pos <= 130; pos += 1, pos2++) {
      joelho.write(pos);            
      pe.write(pos2);
      delay(20);             
    }
    for (pos = 130,pos2=120; pos >= 30; pos -= 1, pos2--) {
      joelho.write(pos);            
      pe.write(pos2);
      delay(20);             
    }}
  if(c == '2'){
    quadril.write(90);
    joelho.write(90);
    pe.write(90);
    
  } 
  if(c == '3'){
    quadril.write(30);
    joelho.write(30);
    pe.write(30);
    
  } 
  if(c == '4'){
    quadril.write(120);
    joelho.write(120);
    pe.write(120);
    
  } 
  if(c == ' ')
    delay(400);
  c = 'm';
}
