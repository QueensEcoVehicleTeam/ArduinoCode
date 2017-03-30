#define period 200

void setup() {
  pinMode(5, OUTPUT);
  pinMode(A0, INPUT);
  Serial.begin(9600);
}

void loop()
{
  digitalWrite(5, LOW);
  delay(period/50);
  digitalWrite(5, HIGH);
  Serial.println(analogRead(A0));
  delay(period);
}
