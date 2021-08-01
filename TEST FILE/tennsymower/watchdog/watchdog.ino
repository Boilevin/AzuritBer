// This sample program shows how to use the watch dog timer (WDT).

// When program starts, it turns LED off for 1 seconds.
// It then turns LED on, and pauses for 5 seconds letting you know it is ready to start.
// WDT is initialized, and loop starts. It will flash led and reset wdt for the first 10 secs.
// After that, it stops resetting wdt. This causes the WDT to reboot and the cycle starts over.

const int          led                = 13;

bool                ledState            = false;
unsigned long      timer;

void setup() {

    // initialize the digital pin as an output.
    pinMode(led, OUTPUT);

    // Indicate we are starting over by hold led off for 1s
    ledState = false;
    digitalWrite(led, ledState);
    delay(1000UL);

    // Indicate we are in setup by hold LED on
    ledState = true;
    digitalWrite(led, ledState);
    delay(5000UL);

    // Setup WDT
    noInterrupts();                                        // don't allow interrupts while setting up WDOG
    WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;                        // unlock access to WDOG registers
    WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
    delayMicroseconds(1);                                  // Need to wait a bit..

    // for this demo, we will use 1 second WDT timeout (e.g. you must reset it in < 1 sec or a boot occurs)
    WDOG_TOVALH = 0x006d;
    WDOG_TOVALL = 0xdd00;

    // This sets prescale clock so that the watchdog timer ticks at 7.2MHz
    WDOG_PRESC  = 0x400;

    // Set options to enable WDT. You must always do this as a SINGLE write to WDOG_CTRLH
    WDOG_STCTRLH |= WDOG_STCTRLH_ALLOWUPDATE |
        WDOG_STCTRLH_WDOGEN | WDOG_STCTRLH_WAITEN |
        WDOG_STCTRLH_STOPEN | WDOG_STCTRLH_CLKSRC;
    interrupts();

}

void loop() {

timer = millis() + 10000UL;                                // length of time we will reset WDT

while (true) {
    ledState = !ledState;
    digitalWrite(led, ledState);
    delay(100UL);
    if (millis() < timer) {                                // Have we timed out yet?
        noInterrupts();                                    //  No - reset WDT
        WDOG_REFRESH = 0xA602;
        WDOG_REFRESH = 0xB480;
        interrupts();
        }
    } // while
}
