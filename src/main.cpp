#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <HardwareTimer.h>

HardwareTimer sampleTimer(TIM1);

//Constants
  const uint32_t interval = 100; //Display update interval
  

//Pin definitions
  //Row select and enable
  const int RA0_PIN = D3;
  const int RA1_PIN = D6;
  const int RA2_PIN = D12;
  const int REN_PIN = A5;

  //Matrix input and output
  const int C0_PIN = A2;
  const int C1_PIN = D9;
  const int C2_PIN = A6;
  const int C3_PIN = D1;
  const int OUT_PIN = D11;

  //Audio analogue out
  const int OUTL_PIN = A4;
  const int OUTR_PIN = A3;

  //Joystick analogue in
  const int JOYY_PIN = A0;
  const int JOYX_PIN = A1;

  //Output multiplexer bits
  const int KNOB_MODE = 2;
  const int DEN_BIT = 3;
  const int DRST_BIT = 4;
  const int HKOW_BIT = 5;
  const int HKOE_BIT = 6;

volatile uint32_t currentStepSize = 0;
volatile int currentNoteIndex = -1;

const char* noteNames[12] = {
  "C","C#","D","D#","E","F",
  "F#","G","G#","A","A#","B"
};

const uint32_t stepSizes[12] = {
  51076002,  // C
  54098238,  // C#
  57307049,  // D
  60715768,  // D#
  64338225,  // E
  68188717,  // F
  72282135,  // F#
  76633902,  // G
  81260097,  // G#
  86177490,  // A (440 Hz)
  91403586,  // A#
  96956624   // B
};


//Display driver object
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

//Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
  digitalWrite(REN_PIN, LOW);

  digitalWrite(RA0_PIN, bitIdx & 0x01);
  digitalWrite(RA1_PIN, (bitIdx >> 1) & 0x01);
  digitalWrite(RA2_PIN, (bitIdx >> 2) & 0x01);

  digitalWrite(OUT_PIN, value);

  digitalWrite(REN_PIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(REN_PIN, LOW);
}


std::bitset<4> readCols() {


    std::bitset<4> result;
    result[0] = digitalRead(C0_PIN);
    result[1] = digitalRead(C1_PIN);
    result[2] = digitalRead(C2_PIN);
    result[3] = digitalRead(C3_PIN);
    return result;
}

void setRow(uint8_t rowIdx) {
  digitalWrite(REN_PIN, LOW);                 // disable to avoid glitches
  digitalWrite(RA0_PIN, rowIdx & 0x01);
  digitalWrite(RA1_PIN, (rowIdx >> 1) & 0x01);
  digitalWrite(RA2_PIN, (rowIdx >> 2) & 0x01);
  digitalWrite(REN_PIN, HIGH);                // enable selected row
}

void sampleISR() {
  static uint32_t phaseAcc = 0;

  phaseAcc += currentStepSize;

  // Convert to 8-bit sawtooth centered at 0
  int32_t vout = (int32_t)(phaseAcc >> 24) - 128;

  // Add DC offset for 0–255 PWM output
  analogWrite(OUTL_PIN, (uint8_t)(vout + 128));
}

void setup() {
  // put your setup code here, to run once:

  //Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT_PULLUP);
  pinMode(C1_PIN, INPUT_PULLUP);
  pinMode(C2_PIN, INPUT_PULLUP);
  pinMode(C3_PIN, INPUT_PULLUP);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply
  setOutMuxBit(KNOB_MODE, HIGH);  //Read knobs through key matrix

  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");

  analogWriteResolution(8);

  sampleTimer.setOverflow(22000, HERTZ_FORMAT);
  sampleTimer.attachInterrupt(sampleISR);
  sampleTimer.resume();
};

void loop(){
  static uint32_t next = millis();
  while (millis() < next) {}
  next += interval;

  // Scan rows 0..2 (12 keys)
  std::bitset<32> inputs;

  for (uint8_t row = 0; row < 3; row++) {
    setRow(row);
    delayMicroseconds(3);

    std::bitset<4> cols = readCols();
    for (uint8_t col = 0; col < 4; col++) {
      inputs[row * 4 + col] = cols[col];
    }
  }

  // Choose step size + note (last pressed wins)
  uint32_t localCurrentStepSize = 0;
  int newNote = -1;

  for (int i = 0; i < 12; i++) {
    if (inputs[i] == 0) {                 // active LOW
      localCurrentStepSize = stepSizes[i];
      newNote = i;
    }
  }

  // Atomic store (single, explicit write)
  __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);

  currentNoteIndex = newNote;

  // Show 12-bit key state (3 hex digits)
  uint16_t v12 = (uint16_t)(inputs.to_ulong() & 0x0FFF);
  char hexbuf[4];
  snprintf(hexbuf, sizeof(hexbuf), "%03X", v12);



  // Update display
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);

  u8g2.drawStr(0, 10, "Hello World!");

  // Hex key state (row 2)
  u8g2.setCursor(0, 20);
  u8g2.print(hexbuf);

  // Note name BELOW it (row 3)
  u8g2.setCursor(0, 30);
  if (currentNoteIndex >= 0) 
      u8g2.print(noteNames[currentNoteIndex]);
  else 
      u8g2.print("--");

  u8g2.sendBuffer();

  digitalToggle(LED_BUILTIN);
}