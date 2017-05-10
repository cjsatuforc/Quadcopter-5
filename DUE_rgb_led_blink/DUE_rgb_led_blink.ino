
void setup(){
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
}
void loop(){
  for(int i=2; i<=7; i++)
  {
    analogWrite(i, 90);
    delay(500);
    analogWrite(i, 0);
  }
  
}
