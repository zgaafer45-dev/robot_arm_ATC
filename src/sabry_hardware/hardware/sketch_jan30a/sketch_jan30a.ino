#define RELAY_EXTEND 5
#define RELAY_RETRACT 18

void lock_tool() {
  digitalWrite(RELAY_RETRACT, LOW);
  digitalWrite(RELAY_EXTEND, HIGH);
}

void unlock_tool() {
  digitalWrite(RELAY_EXTEND, LOW);
  digitalWrite(RELAY_RETRACT, HIGH);
}

void stop_all() {
  digitalWrite(RELAY_EXTEND, LOW);
  digitalWrite(RELAY_RETRACT, LOW);
}

void setup() {
  // put your setup code here, to run once:
  pinMode(RELAY_RETRACT, OUTPUT);
  pinMode(RELAY_EXTEND, OUTPUT);

  digitalWrite(RELAY_EXTEND, HIGH);
  digitalWrite(RELAY_RETRACT, HIGH);

  Serial.begin(115200);
  Serial.setTimeout(1);
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 'L') {
      lock_tool();
    }
    else if (cmd == 'U') {
      unlock_tool();
    }
    else if (cmd == 'S') {
      stop_all();
    }
  }
}