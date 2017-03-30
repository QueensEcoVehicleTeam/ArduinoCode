#define tachPin 3
#define tachRes 1 // resolution of tachometer readings
#define tachCoeff 3600 // converts ms/rev to km/h


void setup() 
{
  Serial.begin(9600);
  pinMode(tachPin, INPUT);
}

void loop() 
{
  //Serial.println(digitalRead(tachPin));
  //delay(1);
  String stringone = "Km/h ";
  String stringthree = tachometer() + stringone;
  Serial.println(stringthree);
  delay(500);
}


float tachometer()
{
  int msCounter = 0;

  //Serial.println("First Loop");
  while(digitalRead(tachPin)==1)
  {
    delay(tachRes);
  }
  //Serial.println("Second Loop");
  while(digitalRead(tachPin)==0)
  {
    msCounter++;
    //Serial.println(msCounter);
    delay(tachRes);
  }
  //Serial.println("Third Loop");
  while(digitalRead(tachPin)==1)
  {
    msCounter++;
    //Serial.println(msCounter);
    delay(tachRes);
  }
  //Serial.println("Complete");
  
  return((1.6/(float)msCounter)*tachCoeff);
}

