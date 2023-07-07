bool atLeft = false;      // atLeft is whether the sensor, limitS1, is active as this indicates whether the gantry is in the left position i.e. retracted
bool atRight = false;
bool mLeft = false;
bool mRight = false;
bool calibrated = false;
bool calibrating = false;
int stepperDelay = 4;
unsigned long lastStep;
unsigned long currTime;
const int stepsTotal = 6793;

int stpPos = 0;
int stpTarg = 0; // can say stpTarg = 100, then compares stpPos with stpTarg; negative value would be a step called with false

#define limitS1 2
#define limitS2 3
#define relayC 4      // relay to open and close the circuit for the compressor
#define relay2 5     // relay for solenoid
#define relay3 6      // relay for solenoid
#define transducer A0
#define STEPPER_PIN_1 8
#define STEPPER_PIN_2 9
#define STEPPER_PIN_3 10
#define STEPPER_PIN_4 11
int step_number = 0;

int pressureZero = 102.4;
int pressure100 = 921.6;
float PSI = 0;
int psiTarg = 0;
bool compressorRunning = false;

bool solenoidActive = false;
unsigned long stopSolenoid;
const int solenoidDelay = 500;  // 500ms to go from closed to open

bool led = false;

void setup() {
  pinMode(STEPPER_PIN_1, OUTPUT);
  pinMode(STEPPER_PIN_2, OUTPUT);
  pinMode(STEPPER_PIN_3, OUTPUT);
  pinMode(STEPPER_PIN_4, OUTPUT);     
  pinMode(limitS1, INPUT); 
  pinMode(limitS2, INPUT);
  Serial.begin(9600);
}

int Calibrate() {
  calibrating = true;
  stpTarg = -(stepsTotal+50);
}

int goRight() {
  if (!atRight && !mRight) {
    if (mLeft) {
      mLeft = false;
    }
    mRight = true;
    stpTarg = stepsTotal;
    float ETA = (stpTarg-stpPos)*stepperDelay/1000;
    String start = "Gantry,ETA,";
    String sendD = start + ETA;
    Serial.println(sendD);
  }
}

int goLeft() {
  if (!atLeft && !mLeft) {
    if (mRight) {
      mRight = false;
    }
    mLeft = true;
    stpTarg = 0;
    float ETA = (stpPos-stpTarg)*stepperDelay/1000;
    String start = "Gantry,ETA";
    String sendD = start + ETA;
    Serial.println(sendD);
  }
}

int stepperMotion() {
  atLeft = digitalRead(limitS1);
  atRight = digitalRead(limitS2);
  if (!calibrated) {
    if (calibrating) {
      if (atLeft) {
        calibrated = true;
        calibrating = false;
        int steps = stepsTotal + (stpTarg-stpPos);
        String completionStr = "Steps completed to calibrate: ";
        String completion = completionStr + steps + " steps";
        Serial.println(completion);
        stpPos=0;
        stpTarg = stpPos;
      } else {
        aStep();
      }
    } else {
      Calibrate();
    }
  }
  if (mLeft) {
    if (atLeft) {
      mLeft = false;
      stpPos=0;
      stpTarg=0;
    } else {
      aStep();
      if (stpTarg==stpPos) {
        stpTarg--;
      }   
    }
  } else if (mRight) {
    if (atRight) {
      mRight = false;
      stpTarg=stpPos;
    } else {
      aStep();
      if (stpTarg==stpPos) {
        stpTarg++;
      }
      //float secs = (stpTarg-stpPos)*stepperDelay/1000;
      //String completionStr = "Completion: ";
      //String completion = completionStr + secs + "s";
      //Serial.println(completion);
    }
  }
}

void loop(){
  stepperMotion();
  checkValve();
  checkCompressor();  
  // check serial for any commands from raspberry pi
  if (Serial.available() > 0){
    String command = Serial.readStringUntil('\n');
    Serial.println(command);
    Serial.println(calibrated);
    if (command == "extendG") {
       if (calibrated) {
          goLeft();
       }
    } else if (command == "retractG") {
      if (calibrated) {
         goRight();
      }
    } else if (command == "stop") {
       mLeft = false;
       mRight = false;
       Serial.println("Gantry,Stopped");
     }
  }
}

void aStep() {
  currTime = millis();
  Serial.print("Current pos: ");Serial.println(stpPos);
  Serial.print("Target pos: ");Serial.println(stpTarg);
  if (currTime-lastStep >= stepperDelay) {
    lastStep = currTime;
    if (stpPos < stpTarg) {
      stpPos++;
      OneStep(true);      
    } else if (stpPos > stpTarg) {
      stpPos--;
      OneStep(false);
    }
  }
}

void OneStep(bool dir){
  if(dir){
    switch(step_number){
      case 0:
        digitalWrite(STEPPER_PIN_1, HIGH);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 1:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, HIGH);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 2:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, HIGH);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 3:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, HIGH);
        break;
    } 
  } else {
    switch(step_number){
      case 0:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, HIGH);
        break;
      case 1:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, HIGH);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 2:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, HIGH);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 3:
        digitalWrite(STEPPER_PIN_1, HIGH);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, LOW);
    } 
  }
  step_number++;
  if (step_number > 3) {
    step_number = 0;
  }
}
