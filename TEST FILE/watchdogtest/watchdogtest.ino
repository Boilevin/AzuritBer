
void watchdogSetup(void) {}
int ii;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Setup.................");
  watchdogEnable(1000);// Watchdog triggerss after 1sec if not reseted.
  ii = 0;
 
}

void loop() {
  // put your main code here, to run repeatedly:
  while (ii <= 20) {
    ii = ii + 1;
    delay(500);
    Serial.println(ii);
    watchdogReset();
  }
  Serial.println("reset on the 1010 pause");
  delay(1010);
}
