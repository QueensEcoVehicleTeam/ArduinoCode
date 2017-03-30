
const int hallpin = A1;

void setup (){
  Serial.begin(9600);
}

void loop(){
  Serial.print(analogRead(A1));
  delay(100);
}
