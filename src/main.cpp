/*
  Lab 2 (CAN + queue + mutex + knob volume)

  What this sketch does:
  1) Scans the 4x4 key matrix (rows 0..3, cols 0..3) every 50 ms.
     - Detects PRESS/RELEASE edges for keys 0..11 (the 12 note keys).
     - For each edge, it PACKS an 8-byte CAN message and pushes it into a TX queue.

  2) CAN Transmit path (no polling):
     - A TX thread blocks on the TX queue.
     - Before calling CAN_TX(), it takes a counting semaphore (3 mailboxes).
     - The CAN TX ISR “gives” the semaphore whenever a mailbox frees up.

  3) CAN Receive path (no polling):
     - A CAN RX ISR reads the frame and pushes the 8 data bytes into an RX queue.
     - A decode thread blocks on the RX queue, and when it gets a message:
       - Updates the “last received message” (protected by sysState.mutex) for display
       - If message is Press: computes step size for that octave+note and updates audio
       - If Release: sets step size to 0 (silence)

  4) Knob 3 quadrature decode (row 3 col 0/1):
     - Maintains volume level knobRotation 0..8
     - Audio ISR applies: vout = vout >> (8 - knobRotation)

  5) Display thread (100 ms):
     - Shows: local 12-bit keys hex, local note, volume, and LAST RX CAN message.

  IMPORTANT:
  - Mutex used only in tasks (never in ISR).
  - ISR-to-task uses queues + FromISR API.
  - Shared variables used in ISR are accessed atomically.

  To test quickly:
  - CAN_INIT(true) enables loopback (board receives its own messages).
  - Press/release a key -> you should see RX updates on screen.
*/

#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <HardwareTimer.h>
#include <STM32FreeRTOS.h>
#include <semphr.h>
#include <ES_CAN.h>  // provided CAN helper library in the lab starter code

// ---------------- Role selection ----------------
// If you want this board to PLAY ONLY what it RECEIVES over CAN, set this to 1.
// If you want local key presses to play immediately too, set to 0.
#define PLAY_FROM_LOCAL_KEYS 0

// ---------------- CAN protocol ----------------
static const uint16_t CAN_ID_KEYS = 0x123;

// Message bytes (8 bytes total)
// [0] 'P'(0x50) pressed OR 'R'(0x52) released
// [1] Octave number 0..8
// [2] Note number 0..11
// [3..7] unused
static const uint8_t MSG_PRESSED  = 0x50; // 'P'
static const uint8_t MSG_RELEASED = 0x52; // 'R'

// Choose an octave for this module’s outgoing messages (0..8).
// Octave 4 matches your stepSizes[] base.
static const uint8_t MODULE_OCTAVE = 4;

// ---------------- Shared state ----------------
typedef struct {
  std::bitset<32> inputs;
  SemaphoreHandle_t mutex;
} sysState_t;

sysState_t sysState;

// Variables shared with ISR (atomic access)
volatile uint32_t currentStepSize = 0;  // phase increment
volatile int knobRotation = 8;          // 0..8 volume

// Variables shared between tasks (protect with mutex)
volatile int currentNoteIndex = -1;

// Last received CAN message (for display)
uint8_t lastRxMsg[8] = {0};

// ---------------- FreeRTOS IPC objects ----------------
QueueHandle_t rxQ = NULL;    // items are 8 bytes (CAN data)
QueueHandle_t txQ = NULL;    // items are 8 bytes (CAN data)
SemaphoreHandle_t canTxSem = NULL; // counting sem: number of free TX mailboxes (3)

// ---------------- Hardware ----------------
HardwareTimer sampleTimer(TIM1);

// Pin definitions
const int RA0_PIN = D3;
const int RA1_PIN = D6;
const int RA2_PIN = D12;
const int REN_PIN = A5;

const int C0_PIN = A2;
const int C1_PIN = D9;
const int C2_PIN = A6;
const int C3_PIN = D1;
const int OUT_PIN = D11;

const int OUTL_PIN = A4;
const int OUTR_PIN = A3;

const int JOYY_PIN = A0;
const int JOYX_PIN = A1;

const int KNOB_MODE = 2;
const int DEN_BIT   = 3;
const int DRST_BIT  = 4;
const int HKOW_BIT  = 5;
const int HKOE_BIT  = 6;

// Notes
const char* noteNames[12] = {
  "C","C#","D","D#","E","F",
  "F#","G","G#","A","A#","B"
};

// Base (octave 4) step sizes
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

// Display
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

// ---------------- Helpers ----------------
static inline uint32_t stepSizeForOctave(uint32_t baseStep, uint8_t octave) {
  // baseStep corresponds to octave 4.
  // octave above 4: multiply by 2^(octave-4) => left shift
  // octave below 4: divide by 2^(4-octave)   => right shift
  if (octave >= 4) return baseStep << (octave - 4);
  return baseStep >> (4 - octave);
}

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

// ---------------- Audio ISR ----------------
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

// ---------------- CAN ISRs ----------------
void CAN_RX_ISR(void) {
  uint32_t id;
  uint8_t data[8] = {0};

  CAN_RX(id, data);

  BaseType_t woken = pdFALSE;
  if (rxQ) {
    xQueueSendFromISR(rxQ, data, &woken);
  }
  portYIELD_FROM_ISR(woken);
}

void CAN_TX_ISR(void) {
  BaseType_t woken = pdFALSE;
  if (canTxSem) {
    xSemaphoreGiveFromISR(canTxSem, &woken);
  }
  portYIELD_FROM_ISR(woken);
}

// ---------------- Tasks ----------------
void canTxTask(void *pvParameters) {
  (void)pvParameters;
  uint8_t msg[8];

  while (1) {
    // Wait for a message to send
    xQueueReceive(txQ, msg, portMAX_DELAY);

    // Wait for a free mailbox
    xSemaphoreTake(canTxSem, portMAX_DELAY);

    // Send (this queues into hardware mailbox)
    CAN_TX(CAN_ID_KEYS, msg);
  }
}

void decodeRxTask(void *pvParameters) {
  (void)pvParameters;
  uint8_t msg[8];

  while (1) {
    // Block until a CAN frame arrives
    xQueueReceive(rxQ, msg, portMAX_DELAY);

    // Update "last received message" for display (short lock)
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    for (int i = 0; i < 8; i++) lastRxMsg[i] = msg[i];
    xSemaphoreGive(sysState.mutex);

    // Decode press/release to play note
    const uint8_t type   = msg[0];
    const uint8_t octave = msg[1];
    const uint8_t note   = msg[2];

    if (note < 12) {
      if (type == MSG_PRESSED) {
        uint32_t step = stepSizeForOctave(stepSizes[note], octave);
        __atomic_store_n(&currentStepSize, step, __ATOMIC_RELAXED);

        // (optional) show note name as "currentNoteIndex"
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

void scanKeysTask(void * pvParameters) {
  (void) pvParameters;

  const TickType_t xFrequency = pdMS_TO_TICKS(50);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // Previous states for edge detection (notes 0..11)
  std::bitset<32> prevInputs;

  // knob decoder local state
  uint8_t prevAB = 0;
  bool firstRun = true;

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    std::bitset<32> inputsLocal;

    // Scan rows 0..3
    for (uint8_t row = 0; row < 4; row++) {
      setRow(row);
      delayMicroseconds(3);

      std::bitset<4> cols = readCols();
      for (uint8_t col = 0; col < 4; col++) {
        inputsLocal[row * 4 + col] = cols[col];
      }
    }

    // ---- Knob 3 decode (row 3 col0/1) ----
    bool A = inputsLocal[3 * 4 + 0];
    bool B = inputsLocal[3 * 4 + 1];
    A = !A; B = !B; // active-low -> invert

    uint8_t currAB = (uint8_t)((B << 1) | A);
    int delta = 0;

    if (!firstRun) {
      uint8_t transition = (uint8_t)((prevAB << 2) | currAB);
      switch (transition) {
        // +1 (CW)
        case 0b0001:
        case 0b0111:
        case 0b1110:
        case 0b1000:
          delta = +1; break;

        // -1 (CCW)
        case 0b0010:
        case 0b1011:
        case 0b1101:
        case 0b0100:
          delta = -1; break;

        default:
          delta = 0; break;
      }
    }
    prevAB = currAB;
    firstRun = false;

    // Update knob volume (atomic store so ISR can read safely)
    if (delta != 0) {
      int v = __atomic_load_n(&knobRotation, __ATOMIC_RELAXED);
      v += delta;
      if (v < 0) v = 0;
      if (v > 8) v = 8;
      __atomic_store_n(&knobRotation, v, __ATOMIC_RELAXED);
    }

    // ---- Edge detect note keys 0..11 and enqueue CAN TX messages ----
    // Active-low: 0 means pressed.
    for (int k = 0; k < 12; k++) {
      bool prevPressed = (prevInputs[k] == 0);
      bool nowPressed  = (inputsLocal[k] == 0);

      if (prevPressed != nowPressed) {
        uint8_t msg[8] = {0};
        msg[0] = nowPressed ? MSG_PRESSED : MSG_RELEASED;
        msg[1] = MODULE_OCTAVE;
        msg[2] = (uint8_t)k;

        // Push into TX queue (thread-safe)
        xQueueSend(txQ, msg, portMAX_DELAY);

#if PLAY_FROM_LOCAL_KEYS
        // Optional local play immediately (otherwise only play from received messages)
        if (nowPressed) {
          uint32_t step = stepSizeForOctave(stepSizes[k], MODULE_OCTAVE);
          __atomic_store_n(&currentStepSize, step, __ATOMIC_RELAXED);
        } else {
          __atomic_store_n(&currentStepSize, 0u, __ATOMIC_RELAXED);
        }
#endif
      }
    }

    prevInputs = inputsLocal;

    // Publish inputs for display (mutex)
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    sysState.inputs = inputsLocal;
    xSemaphoreGive(sysState.mutex);
  }
}

void displayTask(void * pvParameters) {
  (void) pvParameters;

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

    // Line 1
    u8g2.setCursor(0, 10);
    u8g2.print("K:");
    u8g2.print(hexbuf);

    u8g2.setCursor(70, 10);
    u8g2.print("V:");
    u8g2.print(volCopy);

    // Line 2: local note (from RX decode unless PLAY_FROM_LOCAL_KEYS)
    u8g2.setCursor(0, 22);
    u8g2.print("Note:");
    if (noteCopy >= 0) u8g2.print(noteNames[noteCopy]);
    else u8g2.print("--");

    // Line 3: last RX CAN message
    u8g2.setCursor(0, 32);
    u8g2.print((char)rxCopy[0]);  // 'P' or 'R'
    u8g2.print((int)rxCopy[1]);   // octave
    u8g2.print((int)rxCopy[2]);   // note number

    u8g2.sendBuffer();

    digitalToggle(LED_BUILTIN);
  }
}

// ---------------- Setup ----------------
void setup() {
  // Pins
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

  // Display init
  setOutMuxBit(DRST_BIT, LOW);
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);

  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);
  setOutMuxBit(KNOB_MODE, HIGH);

  Serial.begin(9600);
  Serial.println("Lab2 CAN + queues + knob volume");

  analogWriteResolution(8);

  // Mutex
  sysState.mutex = xSemaphoreCreateMutex();
  configASSERT(sysState.mutex);

  // Queues
  rxQ = xQueueCreate(36, 8); // 36 messages, each 8 bytes
  txQ = xQueueCreate(36, 8);
  configASSERT(rxQ);
  configASSERT(txQ);

  // TX mailbox semaphore (3 mailboxes)
  canTxSem = xSemaphoreCreateCounting(3, 3);
  configASSERT(canTxSem);

  static const uint32_t CAN_ID_KEYS = 0x123;

  // CAN init (loopback ON for quick test)
  CAN_Init(true);
  setCANFilter(CAN_ID_KEYS, 0x7FF);
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
}

void loop() {
  // unused with FreeRTOS
}