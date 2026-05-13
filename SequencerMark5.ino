/*
  SEQ5_noPIN_Itsy
  TX sequencer for 222 MHz 10W transverter
  Arduino ItsyBitsy 32U4 3V

  Sequence: RX disable -> TR relay -> PA power -> TX enable
  No RF sense input; AMP+ powers XVTR TX and bias to 10W module.

  v1.0  1/1/2020  pcw  (based on SEQ4 for 432 MHz XVTR, 3/23/2019)
*/

// #define DEBUG   // Uncomment for slowed-down visual debugging

#include <SPI.h>

// ── Pin assignments ───────────────────────────────────────────────
#define ADF4351_LE    23
#define TR_Relay_Pin   5   // 5V output drives NFET
#define TXEnable_Pin  12   // AMP_L on PCB
#define PA_Power_Pin   9   // TX V+
#define RXDisable_Pin 11

#define PTT_L_Pin    A1    // Active LOW
#define RFSense_L_Pin A2   // Active LOW
#define Voltage_Pin  A0    // Analog: monitors 12V supply
#define Amp_RDY_Pin  A3    // Active LOW when amp is ready

// Bicolor voltage LED (I2C pins repurposed)
#define Vgreen_Pin    2    // SCL
#define Vred_Pin      3    // SDA
#define Vyellow_Pin  A4    // LockDet pad

// ── Timing constants ─────────────────────────────────────────────
#ifdef DEBUG
  const unsigned long DEBOUNCEDELAY = 500;
  const unsigned long Relay_delay   = 1000;
  const unsigned long Amp_delay     = 500;
  const unsigned long TX_delay      = 500;
#else
  const unsigned long DEBOUNCEDELAY = 25;
  const unsigned long Relay_delay   = 150;
  const unsigned long Amp_delay     = 25;
  const unsigned long TX_delay      = 200;
#endif

// ── State machine ────────────────────────────────────────────────
enum SeqState {
  STATE_RX,          // 0 – receiving, TX path fully open
  STATE_RX_DISABLED, // 1 – RX muted, waiting before keying relay
  STATE_RELAY,       // 2 – TR relay active, waiting before powering PA
  STATE_PA_ON,       // 3 – PA powered, waiting before enabling TX drive
  STATE_TX           // 4 – transmitting
};

SeqState State = STATE_RX;
bool PTTL, Amp_RDY_L, Vgood, RF_Sense;

// ── Helpers ──────────────────────────────────────────────────────

// Set the bicolor status LED: true = TX (red), false = RX (green)
void setStatusLED(bool transmitting) {
  digitalWrite(Vgreen_Pin, transmitting ? LOW : HIGH);
  digitalWrite(Vred_Pin,   transmitting ? HIGH : LOW);
}

// Debounce an active-LOW digital input; returns true when pin reads LOW.
boolean debounceL(int pin) {
  boolean current, previous;
  previous = digitalRead(pin);
  for (int i = 0; i < (int)DEBOUNCEDELAY; i++) {
    delay(1);
    current = digitalRead(pin);
    if (current != previous) {
      i = 0;
      previous = current;
    }
  }
  return (current == LOW);
}

// Monitor supply voltage via resistor divider; returns false below ~10 V.
// Thresholds (INTERNAL 1.1 V Aref, 150 K / 10 K divider):
//   582 counts ≈ 10.0 V
//   692 counts ≈ 11.9 V
//   739 counts ≈ 12.7 V
boolean Voltage_Mon(int pin) {
  int v = analogRead(pin);
  if (v < 582) {          // Below 10 V – flash red and signal failure
    digitalWrite(Vred_Pin, LOW);  delay(500);
    digitalWrite(Vred_Pin, HIGH); delay(500);
    return false;
  }
  return true;
}

// Write one 32-bit word to the ADF4351 over SPI.
void WriteRegister32(uint32_t value) {
  digitalWrite(ADF4351_LE, LOW);
  for (int i = 3; i >= 0; i--)
    SPI.transfer((value >> (8 * i)) & 0xFF);
  digitalWrite(ADF4351_LE, HIGH);
  digitalWrite(ADF4351_LE, LOW);
}

// Program the ADF4351 for 2808 MHz output.
void SetADF4351() {
  // Register values for 2808 MHz
  const uint32_t registers[6] = {
    0x380040, 0x80080C9, 0x18004E42, 0x4B3, 0x8C80FC, 0x580005
  };
  for (int i = 5; i >= 0; i--)
    WriteRegister32(registers[i]);
}

// ── Setup ────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);
  Serial.println("SEQ5_noPIN_Itsy – 222 MHz 10W XVTR sequencer  1/1/2020 pcw");

  analogReference(INTERNAL);

  // Outputs
  pinMode(RXDisable_Pin, OUTPUT);
  pinMode(TXEnable_Pin,  OUTPUT);
  pinMode(PA_Power_Pin,  OUTPUT);
  pinMode(TR_Relay_Pin,  OUTPUT);
  pinMode(Vgreen_Pin,    OUTPUT);
  pinMode(Vred_Pin,      OUTPUT);
  pinMode(Vyellow_Pin,   OUTPUT);

  // Inputs
  pinMode(PTT_L_Pin,     INPUT);
  pinMode(RFSense_L_Pin, INPUT);
  pinMode(Amp_RDY_Pin,   INPUT);
  digitalWrite(Amp_RDY_Pin, HIGH);  // enable internal pull-up

  // Safe initial state: RX active, TX path disabled
  digitalWrite(TR_Relay_Pin,  LOW);
  digitalWrite(PA_Power_Pin,  LOW);
  digitalWrite(RXDisable_Pin, LOW);
  digitalWrite(TXEnable_Pin,  LOW);
  setStatusLED(false);              // green = receiving
  digitalWrite(Vyellow_Pin,   LOW);

  // ADF4351 SPI init
  pinMode(ADF4351_LE, OUTPUT);
  digitalWrite(ADF4351_LE, HIGH);
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
}

// ── Main loop ────────────────────────────────────────────────────
void loop() {
  RF_Sense  = !digitalRead(RFSense_L_Pin);
  PTTL      = debounceL(PTT_L_Pin);
  Vgood     = true;                         // Voltage_Mon result unused at top level
  Amp_RDY_L = debounceL(Amp_RDY_Pin);

  if (Amp_RDY_L)
    digitalWrite(Vyellow_Pin, HIGH);

  switch (State) {

    case STATE_RX:
      if ((PTTL && Vgood) || RF_Sense) {
        digitalWrite(RXDisable_Pin, HIGH);
        setStatusLED(false);              // green – RX disabled, waiting for relay
        State = STATE_RX_DISABLED;
      } else {
        digitalWrite(RXDisable_Pin, LOW);
        setStatusLED(false);              // green – receiving
      }
      break;

    case STATE_RX_DISABLED:
      if (PTTL && Vgood) {
        digitalWrite(TR_Relay_Pin, HIGH);
        setStatusLED(true);               // red – TR relay energising
        State = STATE_RELAY;
      } else {
        State = STATE_RX;
      }
      break;

    case STATE_RELAY:
      if (PTTL && Vgood) {
        delay(Relay_delay);
        digitalWrite(PA_Power_Pin, HIGH);
        setStatusLED(true);               // red – PA powering up
        State = STATE_PA_ON;
      } else {
        digitalWrite(TR_Relay_Pin, LOW);
        setStatusLED(false);              // green – TX aborted, relay opening
        delay(Relay_delay);
        State = STATE_RX;
      }
      break;

    case STATE_PA_ON:
      setStatusLED(true);               // red – PA on, waiting for amp ready
      if (PTTL && Vgood) {
        delay(Amp_delay);
        digitalWrite(TXEnable_Pin, HIGH);
        State = STATE_TX;
      } else {
        digitalWrite(PA_Power_Pin, LOW);
        delay(Amp_delay);
        State = STATE_RELAY;
      }
      break;

    case STATE_TX:
      setStatusLED(true);               // red – transmitting
      if (!(PTTL && Vgood)) {
        delay(TX_delay);                      // confirm PTT is really released
        if (!(PTTL && Vgood)) {
          digitalWrite(TXEnable_Pin, LOW);
          delay(TX_delay);
          State = STATE_PA_ON;
        }
      }
      break;

    default:
      State = STATE_RX;
  }

  SetADF4351();
}
