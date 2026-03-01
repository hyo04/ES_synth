/*
  Lab 2 — Full code with:
  - Mutex-protected sysState
  - Knob 3 quadrature decode -> volume (0..8)
  - CAN send/receive using ES_CAN library (RX ISR -> queue -> decode task)
  - CAN TX using queue + counting semaphore (mailboxes) + TX ISR
  - Execution-time measurement mode for scanKeysOnce()

  ===== HOW TO USE =====
  1) Normal operation:
       set TEST_SCANK_KEYS to 0
       -> FreeRTOS runs tasks, CAN works, display updates, audio plays from CAN RX.

  2) Timing test (measure scanKeysOnce worst-case):
       set TEST_SCANK_KEYS to 1
       -> No scheduler, no CAN start, no ISRs.
       -> Runs scanKeysOnce() 32 times and prints total + average microseconds on Serial.

  Notes:
  - This code uses ES_CAN.h API names that match your screenshot:
      setCANFilter(...)
      CAN_RegisterRX_ISR(...)
      CAN_RegisterTX_ISR(...)
      CAN_Start()
    and a CAN_RX signature that takes (uint32_t &id, uint8_t data[8]).
  - The CAN init function name differs across versions. Try CAN_Init(true) first.
    If your compiler says it doesn't exist, search ES_CAN.h for the init function name
    and change it here.
*/

#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <HardwareTimer.h>
#include <STM32FreeRTOS.h>
#include "semphr.h"
#include "ES_CAN.h"   // your library name

// ===================== BUILD SWITCHES =====================
#define TEST_SCANK_KEYS   1   // 1 = timing test only (prints Serial), 0 = normal RTOS build
#define PLAY_FROM_LOCAL_KEYS 0 // 1 = play immediately on local key edges too, 0 = play only from CAN RX

// ===================== CAN PROTOCOL =====================
static const uint32_t CAN_ID_KEYS = 0x123;

// 8-byte message:
// [0] 'P'(0x50) pressed OR 'R'(0x52) released
// [1] octave 0..8
// [2] note 0..11
// [3..7] unused
static const uint8_t MSG_PRESSED  = 0x50; // 'P'
static const uint8_t MSG_RELEASED = 0x52; // 'R'

// Choose the octave that THIS module transmits
static const uint8_t MODULE_OCTAVE = 4;

// ===================== HARDWARE =====================
HardwareTimer sampleTimer(TIM1);

// Row select and enable
const int RA0_PIN = D3;
const int RA1_PIN = D6;
const int RA2_PIN = D12;
const int REN_PIN = A5;

// Matrix columns
const int C0_PIN = A2;
const int C1_PIN = D9;
const int C2_PIN = A6;
const int C3_PIN = D1;

// Output mux
const int OUT_PIN = D11;

// Audio outputs
const int OUTL_PIN = A4;
const int OUTR_PIN = A3;

// Joystick (unused)
const int JOYY_PIN = A0;
const int JOYX_PIN = A1;

// Out mux bits
const int KNOB_MODE = 2;
const int DEN_BIT   = 3;
const int DRST_BIT  = 4;
const int HKOW_BIT  = 5;
const int HKOE_BIT  = 6;

// ===================== NOTES =====================
const char* noteNames[12] = {
  "C","C#","D","D#","E","F",
  "F#","G","G#","A","A#","B"
};

// Base step sizes (octave 4)
const uint32_t stepSizes[12] = {
  51076002, 54098238, 57307049, 60715768,
  64338225, 68188717, 72282135, 76633902,
  81260097, 86177490, 91403586, 96956624
};

static inline uint32_t stepSizeForOctave(uint32_t baseStep, uint8_t octave) {
  if (octave >= 4) return baseStep << (octave - 4);
  return baseStep >> (4 - octave);
}

// ===================== DISPLAY =====================
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

// ===================== SHARED STATE =====================
typedef struct {
  std::bitset<32> inputs;
  SemaphoreHandle_t mutex;
} sysState_t;

sysState_t sysState;

// Used by ISR -> atomic between tasks and ISR
volatile uint32_t currentStepSize = 0;
volatile int knobRotation = 8;  // 0..8 volume (start loud)

// Used by tasks (protect with mutex)
volatile int currentNoteIndex = -1;
uint8_t lastRxMsg[8] = {0};

// ===================== RTOS IPC =====================
QueueHandle_t rxQ = NULL;     // item size 8 bytes
QueueHandle_t txQ = NULL;     // item size 8 bytes
SemaphoreHandle_t canTxSem = NULL; // counting sem for 3 mailboxes

// ===================== KEY MATRIX HELPERS =====================
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
  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, rowIdx & 0x01);
  digitalWrite(RA1_PIN, (rowIdx >> 1) & 0x01);
  digitalWrite(RA2_PIN, (rowIdx >> 2) & 0x01);
  digitalWrite(REN_PIN, HIGH);
}

// ===================== AUDIO ISR =====================
void sampleISR() {
  static uint32_t phaseAcc = 0;

  uint32_t step = __atomic_load_n(&currentStepSize, __ATOMIC_RELAXED);
  phaseAcc += step;

  int32_t vout = (int32_t)(phaseAcc >> 24) - 128;

  int vol = __atomic_load_n(&knobRotation, __ATOMIC_RELAXED);
  if (vol < 0) vol = 0;
  if (vol > 8) vol = 8;
  vout = vout >> (8 - vol);

  analogWrite(OUTL_PIN, (uint8_t)(vout + 128));
}

// ===================== CAN ISRs =====================
void CAN_RX_ISR(void) {
  uint32_t id = 0;
  uint8_t data[8] = {0};

  // ES_CAN.h expects ID by reference (NOT &id)
  CAN_RX(id, data);

  BaseType_t woken = pdFALSE;
  if (rxQ) xQueueSendFromISR(rxQ, data, &woken);
  portYIELD_FROM_ISR(woken);
}

void CAN_TX_ISR(void) {
  BaseType_t woken = pdFALSE;
  if (canTxSem) xSemaphoreGiveFromISR(canTxSem, &woken);
  portYIELD_FROM_ISR(woken);
}

// ===================== scanKeysOnce (ONE ITERATION) =====================
// This is what we time in TEST_SCANK_KEYS mode.
// In normal mode, scanKeysTask() calls this once per 50ms.
static void scanKeysOnce() {
  static std::bitset<32> prevInputs; // for edge detect
  static uint8_t prevAB = 0;         // for knob decoder
  static bool firstRun = true;

  std::bitset<32> inputsLocal;

  // Scan rows 0..3 (row 3 contains knob A/B)
  for (uint8_t row = 0; row < 4; row++) {
    setRow(row);
    delayMicroseconds(3);
    std::bitset<4> cols = readCols();
    for (uint8_t col = 0; col < 4; col++) {
      inputsLocal[row * 4 + col] = cols[col];
    }
  }

  // ---- Knob 3 decode: row 3 col0=A, col1=B ----
  bool A = inputsLocal[3 * 4 + 0];
  bool B = inputsLocal[3 * 4 + 1];
  A = !A; B = !B; // active-low invert

  uint8_t currAB = (uint8_t)((B << 1) | A);
  int delta = 0;

  if (!firstRun) {
    uint8_t t = (uint8_t)((prevAB << 2) | currAB);
    switch (t) {
      // +1
      case 0b0001:
      case 0b0111:
      case 0b1110:
      case 0b1000: delta = +1; break;

      // -1
      case 0b0010:
      case 0b1011:
      case 0b1101:
      case 0b0100: delta = -1; break;

      default: delta = 0; break;
    }
  }
  prevAB = currAB;
  firstRun = false;

  if (delta != 0) {
    int v = __atomic_load_n(&knobRotation, __ATOMIC_RELAXED);
    v += delta;
    if (v < 0) v = 0;
    if (v > 8) v = 8;
    __atomic_store_n(&knobRotation, v, __ATOMIC_RELAXED);
  }

  // ---- CAN message generation ----
#if TEST_SCANK_KEYS
  // Worst-case: generate 12 messages every iteration (do NOT block)
  for (int k = 0; k < 12; k++) {
    uint8_t msg[8] = {0};
    msg[0] = MSG_PRESSED;
    msg[1] = MODULE_OCTAVE;
    msg[2] = (uint8_t)k;
    if (txQ) (void)xQueueSend(txQ, msg, 0); // 0 timeout to avoid blocking
  }
#else
  // Normal: send only on press/release edges for keys 0..11
  for (int k = 0; k < 12; k++) {
    bool prevPressed = (prevInputs[k] == 0);
    bool nowPressed  = (inputsLocal[k] == 0);

    if (prevPressed != nowPressed) {
      uint8_t msg[8] = {0};
      msg[0] = nowPressed ? MSG_PRESSED : MSG_RELEASED;
      msg[1] = MODULE_OCTAVE;
      msg[2] = (uint8_t)k;

      if (txQ) xQueueSend(txQ, msg, portMAX_DELAY);

#if PLAY_FROM_LOCAL_KEYS
      if (nowPressed) {
        uint32_t step = stepSizeForOctave(stepSizes[k], MODULE_OCTAVE);
        __atomic_store_n(&currentStepSize, step, __ATOMIC_RELAXED);
      } else {
        __atomic_store_n(&currentStepSize, 0u, __ATOMIC_RELAXED);
      }
#endif
    }
  }
#endif

  prevInputs = inputsLocal;

  // Publish inputs for display (mutex)
  if (sysState.mutex) {
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    sysState.inputs = inputsLocal;
    xSemaphoreGive(sysState.mutex);
  }
}

// ===================== TASKS =====================
void scanKeysTask(void *pvParameters) {
  (void)pvParameters;

  const TickType_t xFrequency = pdMS_TO_TICKS(50);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    scanKeysOnce();
  }
}

void canTxTask(void *pvParameters) {
  (void)pvParameters;
  uint8_t msg[8];

  while (1) {
    xQueueReceive(txQ, msg, portMAX_DELAY);
    xSemaphoreTake(canTxSem, portMAX_DELAY);
    CAN_TX(CAN_ID_KEYS, msg); // ES_CAN typically takes (ID, data[8])
  }
}

void decodeRxTask(void *pvParameters) {
  (void)pvParameters;
  uint8_t msg[8];

  while (1) {
    xQueueReceive(rxQ, msg, portMAX_DELAY);

    // Copy for display
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    for (int i = 0; i < 8; i++) lastRxMsg[i] = msg[i];
    xSemaphoreGive(sysState.mutex);

    uint8_t type   = msg[0];
    uint8_t octave = msg[1];
    uint8_t note   = msg[2];

    if (note < 12) {
      if (type == MSG_PRESSED) {
        uint32_t step = stepSizeForOctave(stepSizes[note], octave);
        __atomic_store_n(&currentStepSize, step, __ATOMIC_RELAXED);

        xSemaphoreTake(sysState.mutex, portMAX_DELAY);
        currentNoteIndex = (int)note;
        xSemaphoreGive(sysState.mutex);

      } else if (type == MSG_RELEASED) {
        __atomic_store_n(&currentStepSize, 0u, __ATOMIC_RELAXED);

        xSemaphoreTake(sysState.mutex, portMAX_DELAY);
        currentNoteIndex = -1;
        xSemaphoreGive(sysState.mutex);
      }
    }
  }
}

void displayTask(void *pvParameters) {
  (void)pvParameters;

  const TickType_t xFrequency = pdMS_TO_TICKS(100);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    std::bitset<32> inputsCopy;
    int noteCopy;
    uint8_t rxCopy[8];

    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    inputsCopy = sysState.inputs;
    noteCopy = currentNoteIndex;
    for (int i = 0; i < 8; i++) rxCopy[i] = lastRxMsg[i];
    xSemaphoreGive(sysState.mutex);

    int volCopy = __atomic_load_n(&knobRotation, __ATOMIC_RELAXED);

    uint16_t v12 = (uint16_t)(inputsCopy.to_ulong() & 0x0FFF);
    char hexbuf[5];
    snprintf(hexbuf, sizeof(hexbuf), "%03X", v12);

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);

    // Line 1: local key bits + volume
    u8g2.setCursor(0, 10);
    u8g2.print("K:");
    u8g2.print(hexbuf);

    u8g2.setCursor(70, 10);
    u8g2.print("V:");
    u8g2.print(volCopy);

    // Line 2: note name (from CAN decode)
    u8g2.setCursor(0, 22);
    u8g2.print("Note:");
    if (noteCopy >= 0) u8g2.print(noteNames[noteCopy]);
    else u8g2.print("--");

    // Line 3: last RX CAN message (P/R, octave, note)
    u8g2.setCursor(0, 32);
    u8g2.print((char)rxCopy[0]);
    u8g2.print((int)rxCopy[1]);
    u8g2.print((int)rxCopy[2]);

    u8g2.sendBuffer();
    digitalToggle(LED_BUILTIN);
  }
}

// ===================== SETUP =====================
void setup() {
  // GPIO init
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

  analogWriteResolution(8);

  // Display init
  setOutMuxBit(DRST_BIT, LOW);
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);
  setOutMuxBit(KNOB_MODE, HIGH);

  // Mutex + queues (created in both normal and test mode)
  sysState.mutex = xSemaphoreCreateMutex();
  configASSERT(sysState.mutex);

  txQ = xQueueCreate(36, 8);
  rxQ = xQueueCreate(36, 8);
  configASSERT(txQ);
  configASSERT(rxQ);

  canTxSem = xSemaphoreCreateCounting(3, 3);
  configASSERT(canTxSem);

#if TEST_SCANK_KEYS
  // ========= TIMING TEST MODE =========
  Serial.begin(9600);
  delay(200);
  Serial.println("Timing test: scanKeysOnce() x32");

  // Warm-up
  scanKeysOnce();

  uint32_t startTime = micros();
  for (int i = 0; i < 32; i++) {
    scanKeysOnce();
  }
  uint32_t dt = micros() - startTime;

  Serial.print("Total (32 iters) = ");
  Serial.print(dt);
  Serial.println(" us");

  Serial.print("Avg per iter = ");
  Serial.print(dt / 32.0f);
  Serial.println(" us");

  while (1) { /* stop */ }

#else
  // ========= NORMAL RTOS MODE =========

  // CAN init (loopback true for single-board test, false for two-board)
  // If your ES_CAN.h uses a different init name, change this line accordingly.
  CAN_Init(true);

  setCANFilter(CAN_ID_KEYS, 0x7FF);  // accept only ID 0x123

  CAN_RegisterRX_ISR(CAN_RX_ISR);
  CAN_RegisterTX_ISR(CAN_TX_ISR);
  CAN_Start();

  // Audio timer
  sampleTimer.setOverflow(22000, HERTZ_FORMAT);
  sampleTimer.attachInterrupt(sampleISR);
  sampleTimer.resume();

  // Tasks
  xTaskCreate(scanKeysTask, "scanKeys", 256, NULL, 3, NULL);
  xTaskCreate(canTxTask,    "canTx",    256, NULL, 2, NULL);
  xTaskCreate(decodeRxTask, "decodeRx", 256, NULL, 2, NULL);
  xTaskCreate(displayTask,  "display",  256, NULL, 1, NULL);

  vTaskStartScheduler();
#endif
}

void loop() {
  // unused with FreeRTOS
}