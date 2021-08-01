
int uu=0;
void setup() {
  // put your setup code here, to run once:
Serial1.begin(19200);
Serial.begin(115200);
Serial.print ("Using PFOD raw DATA you need to see counter");
}

void loop() {
  // put your main code here, to run repeatedly:
  uu=uu+1;
  Serial1.println(uu);
  delay(50);

}
