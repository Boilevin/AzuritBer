

struct {
  boolean on;
  uint8_t counter;
  const ledSequence_t* currSequence;
} ledStatus;


void setLed(boolean on) {
  ledStatus.on = on;
  digitalWrite(2, on ? HIGH : LOW);
}

void setLedSequence(const struct ledSequence_t& ledSeq) {
  ledStatus.currSequence = &ledSeq;
  ledStatus.counter = 0;
  setLed(true);
}

void onLedTicker(void) {
  ledStatus.counter++;
  if (ledStatus.on) {
    if (ledStatus.counter >= ledStatus.currSequence->onTicks) {
      if (ledStatus.currSequence->offTicks) {
        setLed(false);
      } else {
        // Stay ON
      }
      ledStatus.counter = 0;
    }
  } else {
    if (ledStatus.counter >= ledStatus.currSequence->offTicks) {
      setLed(true);
      ledStatus.counter = 0;
    }
  }
}
