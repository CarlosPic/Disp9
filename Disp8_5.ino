#include <EEPROM.h>
#include <avr/wdt.h>
#include <avr/io.h>
#include <stdio.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

/* Version 8.5
   - Protocolo RX: $,ID,Orden,IDx,Valor,%
   - Protocolo TX: %,ID,Orden,IDx,Valor,$
   - Telemetría cada 1 s por cada bomba activa
   - START = toggle por botón, también controlable por protocolo
   - JOG = momentáneo real por botón, también controlable por protocolo
   - Refresco LCD por escalones de $100
   - Al terminar: 5 s mostrando acumulado arriba y bill/%/cant abajo
   - Luego vuelve al cartel normal
   - Anti-flicker con buffers
   - Lectura segura de contadores actualizados por ISR
   - bill final redondeado a la centena para cierre, acumulado y trama end
   - setlrn: aprende pulsos reales al siguiente cierre y actualiza fill en EEPROM
   - chgID: cambia MACHINE_ID (1..10 chars alfanuméricos) y lo guarda en EEPROM
   - setRst: reset por software usando watchdog
   - setNameB: cambia nombre visible de bomba y lo guarda en EEPROM
*/

/* 
"C:\\Users\\THINKPAD\\AppData\\Local\\Arduino15\\packages\\arduino\\tools\\avr-gcc\\7.3.0-atmel3.6.1-arduino7/bin/avr-size" -A "C:\\Users\\THINKPAD\\AppData\\Local\\arduino\\sketches\\C335023E492C78C5937CF7FF63BC1518/Disp8_2.ino.elf"
Sketch uses 18322 bytes (7%) of program storage space. Maximum is 253952 bytes.
Global variables use 1680 bytes (20%) of dynamic memory, leaving 6512 bytes for local variables. Maximum is 8192 bytes.
 */

// ========== VERSION ==========
#define FW_MAJOR 8
#define FW_MINOR 5
const char FW_VERSION_STR[] = "8.5";

// ========== TIPOS ==========
enum IdTipo { ID_INVALIDO = 0, ID_NORMAL, ID_SUPER };

// ========== MCUSR ==========
uint8_t mcusr_mirror __attribute__ ((section (".noinit")));
const char* lastResetCause = "UNKNOWN";

void getMCUSR(void) __attribute__((naked)) __attribute__((section(".init3")));
void getMCUSR(void) {
  mcusr_mirror = MCUSR;
  MCUSR = 0;
}

// ========== PINES ==========
const int Bomba1_PIN = 7;
const int Bomba2_PIN = 6;
const int Bomba3_PIN = 5;
const int Bomba4_PIN = 4;

const int Opto1_PIN = 25;
const int Opto2_PIN = 23;
const int Opto3_PIN = 24;
const int Opto4_PIN = 22;

const int Buzzer_PIN = 53;

const int BtnStart1_PIN = 41;
const int BtnJog1_PIN   = 43;
const int BtnStart2_PIN = 40;
const int BtnJog2_PIN   = 42;
const int BtnStart3_PIN = 45;
const int BtnJog3_PIN   = 47;
const int BtnStart4_PIN = 44;
const int BtnJog4_PIN   = 46;

const int BtKey_PIN = 9;

// Pines flujo
const byte flujoPin1 = A8;
const byte flujoPin2 = A9;
const byte flujoPin3 = A10;
const byte flujoPin4 = A11;

// ========== LCD ==========
#define LCD12_ADDR 0x26
#define LCD34_ADDR 0x25
LiquidCrystal_I2C lcd12(LCD12_ADDR, 16, 2);
LiquidCrystal_I2C lcd34(LCD34_ADDR, 16, 2);

// ========== CONFIG ==========
#define SERIAL_BAUD 115200
#define BUF_LEN 96

const unsigned long DEFAULT_FILL_PULSOS = 3000;
const unsigned long DEFAULT_TIME_MS     = 30000;
const unsigned long RESTART_DELAY_MS    = 20000;
const unsigned long HOLD_FIN_MS         = 5000UL;
const unsigned long DEBOUNCE_MS         = 80UL;
const unsigned long BILL_STEP_LCD       = 100UL;
const unsigned long STATUS_TX_MS        = 1000UL;

const uint8_t MACHINE_ID_MAX = 10;
const char DEFAULT_MACHINE_ID[MACHINE_ID_MAX + 1] = "AB12C";
char MACHINE_ID[MACHINE_ID_MAX + 1] = "AB12C";
const char SUPER_ID[] = "avt99";
char BT_NAME[16] = "AvtBt001";

// ========== PROTOCOLO ==========
#define PROTO_INI_RX '$'
#define PROTO_FIN_RX '%'
#define PROTO_INI_TX '%'
#define PROTO_FIN_TX '$'

// ========== EEPROM ==========
const int EE_ADDR_FLAG      = 0;
const int EE_ADDR_MACHINEID = 1;
const int EE_ADDR_FILL      = EE_ADDR_MACHINEID + (MACHINE_ID_MAX + 1);
const int EE_ADDR_TIME      = EE_ADDR_FILL + 4*4;
const int EE_ADDR_BILL      = EE_ADDR_TIME + 4*4;
const int EE_ADDR_FILLS     = EE_ADDR_BILL + 4*4;
const int EE_ADDR_BTNAME    = EE_ADDR_FILLS + 4*4;
const byte EE_FLAG_INIT     = 0xA5;
const uint8_t BT_NAME_MAX   = 15;
const int EE_ADDR_CANTML    = EE_ADDR_BTNAME + BT_NAME_MAX;
const int EE_ADDR_ACUMBILL  = EE_ADDR_CANTML + 4*4;
const int EE_ADDR_BNAME0    = EE_ADDR_ACUMBILL + 4*4;
const int EE_ADDR_BNAME1    = EE_ADDR_BNAME0 + 16;
const int EE_ADDR_BNAME2    = EE_ADDR_BNAME1 + 16;
const int EE_ADDR_BNAME3    = EE_ADDR_BNAME2 + 16;

// ========== ESTADO ==========
volatile unsigned long flujo_counts[4] = {0,0,0,0};
uint8_t lastPortK;

unsigned long surtidor_fill[4]   = {DEFAULT_FILL_PULSOS, DEFAULT_FILL_PULSOS, DEFAULT_FILL_PULSOS, DEFAULT_FILL_PULSOS};
unsigned long surtidor_bill[4]   = {0,0,0,0};
unsigned long surtidor_time[4]   = {DEFAULT_TIME_MS, DEFAULT_TIME_MS, DEFAULT_TIME_MS, DEFAULT_TIME_MS};
unsigned long surtidor_fills[4]  = {0,0,0,0};
unsigned long surtidor_cantML[4] = {0,0,0,0};
unsigned long surtidor_acumBill[4] = {0,0,0,0};

char bombaName[4][17] = { "Bomba 1", "Bomba 2", "Bomba 3", "Bomba 4" };

bool BombaActiva[4] = {false,false,false,false};
bool marchaToggle[4] = {false,false,false,false};
bool modoJogActivo[4] = {false,false,false,false};
bool modoLearn[4] = {false,false,false,false};

unsigned long BombaStartTime[4] = {0,0,0,0};
unsigned long BombaReadyTime[4] = {0,0,0,0};

// cierre visual
bool cierreActivo[4] = {false,false,false,false};
unsigned long cierreHasta[4] = {0,0,0,0};
unsigned long cierreBill[4] = {0,0,0,0};
unsigned long cierreAcum[4] = {0,0,0,0};
unsigned long cierreVol[4] = {0,0,0,0};
uint8_t cierrePercent[4] = {0,0,0,0};

// jog vivo
unsigned long jogBillActual[4] = {0,0,0,0};
unsigned long jogVolActual[4]  = {0,0,0,0};

// refresco por pasos
unsigned long lastBillStepShown[4] = {999999UL,999999UL,999999UL,999999UL};

// telemetría
unsigned long lastStatusTx[4] = {0,0,0,0};

// serial
char rxBufUSB[BUF_LEN];
uint8_t rxIndexUSB = 0;
bool frameStartedUSB = false;

char rxBufBT[BUF_LEN];
uint8_t rxIndexBT = 0;
bool frameStartedBT = false;

// debounce
int debouncedStart[4] = {HIGH,HIGH,HIGH,HIGH};
int debouncedJog[4]   = {HIGH,HIGH,HIGH,HIGH};
unsigned long lastChangeStart[4] = {0,0,0,0};
unsigned long lastChangeJog[4]   = {0,0,0,0};
bool prevStartPressed[4] = {false,false,false,false};
bool prevJogPressed[4]   = {false,false,false,false};

// anti-flicker LCD
char lastTopText[4][17] = {
  "                ",
  "                ",
  "                ",
  "                "
};
char lastBottomText[4][17] = {
  "                ",
  "                ",
  "                ",
  "                "
};

// ========== HELPERS ==========
void Beep_On() {
  digitalWrite(Buzzer_PIN, HIGH);
  delay(25);
  digitalWrite(Buzzer_PIN, LOW);
}

void Beep_Short() {
  digitalWrite(Buzzer_PIN, HIGH);
  delay(20);
  digitalWrite(Buzzer_PIN, LOW);
}

void eepromWriteULong(int addr, unsigned long value) { EEPROM.put(addr, value); }
unsigned long eepromReadULong(int addr) { unsigned long v; EEPROM.get(addr, v); return v; }

bool usaLCDSuperior(uint8_t idx) {
  return (idx == 0 || idx == 2);
}

LiquidCrystal_I2C* lcdForBomba(uint8_t idx) {
  return (idx < 2) ? &lcd12 : &lcd34;
}

bool isAlphaNumStr(const char* s, uint8_t minLen, uint8_t maxLen) {
  if (!s) return false;
  uint8_t len = strlen(s);
  if (len < minLen || len > maxLen) return false;

  for (uint8_t i = 0; i < len; i++) {
    char c = s[i];
    bool ok = ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z'));
    if (!ok) return false;
  }
  return true;
}

bool isPrintableName(const char* s, uint8_t minLen, uint8_t maxLen) {
  if (!s) return false;
  uint8_t len = strlen(s);
  if (len < minLen || len > maxLen) return false;

  for (uint8_t i = 0; i < len; i++) {
    char c = s[i];
    if (c < 32 || c > 126 || c == ',' || c == '$' || c == '%') return false;
  }
  return true;
}

unsigned long getPulsosParaLCDSafe(uint8_t idx) {
  unsigned long v;
  noInterrupts();
  if (idx == 0) v = flujo_counts[1];
  else if (idx == 2) v = flujo_counts[3];
  else v = flujo_counts[idx];
  interrupts();
  return v;
}

uint8_t calcPercent(uint8_t idx, unsigned long pulsos) {
  if (surtidor_fill[idx] == 0) return 0;
  if (pulsos >= surtidor_fill[idx]) return 100;
  return (uint8_t)((pulsos * 100UL) / surtidor_fill[idx]);
}

unsigned long calcVolumenML(uint8_t idx, unsigned long pulsos) {
  if (surtidor_fill[idx] == 0 || surtidor_cantML[idx] == 0) return 0;
  unsigned long vol = (pulsos * surtidor_cantML[idx]) / surtidor_fill[idx];
  if (vol > surtidor_cantML[idx]) vol = surtidor_cantML[idx];
  return vol;
}

unsigned long calcBillProgresivo(uint8_t idx, unsigned long pulsos) {
  if (surtidor_fill[idx] == 0 || surtidor_bill[idx] == 0) return 0;
  unsigned long val = (pulsos * surtidor_bill[idx]) / surtidor_fill[idx];
  if (val > surtidor_bill[idx]) val = surtidor_bill[idx];
  return val;
}

unsigned long floorBillStep(unsigned long bill) {
  return (bill / BILL_STEP_LCD) * BILL_STEP_LCD;
}

unsigned long roundBillTo100(unsigned long valor) {
  return ((valor + 50UL) / 100UL) * 100UL;
}

void formatear16(const char* src, char* dst) {
  uint8_t len = strlen(src);
  for (uint8_t i = 0; i < 16; i++) dst[i] = ' ';
  dst[16] = '\0';
  for (uint8_t i = 0; i < len && i < 16; i++) dst[i] = src[i];
}

void centrar16(const char* src, char* dst) {
  uint8_t len = strlen(src);
  for (uint8_t i = 0; i < 16; i++) dst[i] = ' ';
  dst[16] = '\0';
  if (len >= 16) {
    strncpy(dst, src, 16);
    dst[16] = '\0';
  } else {
    uint8_t start = (16 - len) / 2;
    for (uint8_t i = 0; i < len; i++) dst[start + i] = src[i];
  }
}

void lcdWriteTopIfChanged(uint8_t idx, const char* txt) {
  if (!usaLCDSuperior(idx)) return;
  char buf[17];
  centrar16(txt, buf);

  if (strncmp(buf, lastTopText[idx], 16) != 0) {
    strncpy(lastTopText[idx], buf, 17);
    LiquidCrystal_I2C* lcd = lcdForBomba(idx);
    lcd->setCursor(0, 0);
    lcd->print(buf);
  }
}

void lcdWriteBottomIfChanged(uint8_t idx, const char* txt) {
  char buf[17];
  formatear16(txt, buf);

  if (strncmp(buf, lastBottomText[idx], 16) != 0) {
    strncpy(lastBottomText[idx], buf, 17);
    LiquidCrystal_I2C* lcd = lcdForBomba(idx);
    lcd->setCursor(0, 1);
    lcd->print(buf);
  }
}

void lcdForceRefreshNormal(uint8_t idx) {
  lastTopText[idx][0] = '\0';
  lastBottomText[idx][0] = '\0';
}

void guardarFillEEPROM(uint8_t idx)     { eepromWriteULong(EE_ADDR_FILL + idx*4, surtidor_fill[idx]); }
void guardarTimeEEPROM(uint8_t idx)     { eepromWriteULong(EE_ADDR_TIME + idx*4, surtidor_time[idx]); }
void guardarBillEEPROM(uint8_t idx)     { eepromWriteULong(EE_ADDR_BILL + idx*4, surtidor_bill[idx]); }
void guardarFillsEEPROM(uint8_t idx)    { eepromWriteULong(EE_ADDR_FILLS + idx*4, surtidor_fills[idx]); }
void guardarCantEEPROM(uint8_t idx)     { eepromWriteULong(EE_ADDR_CANTML + idx*4, surtidor_cantML[idx]); }
void guardarAcumBillEEPROM(uint8_t idx) { eepromWriteULong(EE_ADDR_ACUMBILL + idx*4, surtidor_acumBill[idx]); }

void guardarMachineIDEEPROM() {
  for (uint8_t i = 0; i < MACHINE_ID_MAX + 1; i++) {
    EEPROM.write(EE_ADDR_MACHINEID + i, 0);
  }
  for (uint8_t i = 0; i < strlen(MACHINE_ID) && i < MACHINE_ID_MAX; i++) {
    EEPROM.write(EE_ADDR_MACHINEID + i, MACHINE_ID[i]);
  }
}

void guardarBombaNameEEPROM(uint8_t idx) {
  int baseAddr = (idx == 0) ? EE_ADDR_BNAME0 : (idx == 1) ? EE_ADDR_BNAME1 : (idx == 2) ? EE_ADDR_BNAME2 : EE_ADDR_BNAME3;
  for (uint8_t j = 0; j < 16; j++) EEPROM.write(baseAddr + j, bombaName[idx][j]);
}

void softwareReset() {
  delay(50);
  Serial.flush();
  Serial1.flush();
  wdt_enable(WDTO_15MS);
  while (1) { }
}

void cargarDesdeEEPROM() {
  byte flag = EEPROM.read(EE_ADDR_FLAG);

  if (flag != EE_FLAG_INIT) {
    for (uint8_t i = 0; i < MACHINE_ID_MAX + 1; i++) EEPROM.write(EE_ADDR_MACHINEID + i, 0);
    for (uint8_t i = 0; i < strlen(DEFAULT_MACHINE_ID); i++) EEPROM.write(EE_ADDR_MACHINEID + i, DEFAULT_MACHINE_ID[i]);

    for (int i = 0; i < 4; i++) {
      surtidor_fill[i] = DEFAULT_FILL_PULSOS;
      surtidor_time[i] = DEFAULT_TIME_MS;
      surtidor_bill[i] = 0;
      surtidor_fills[i] = 0;
      surtidor_cantML[i] = 0;
      surtidor_acumBill[i] = 0;

      guardarFillEEPROM(i);
      guardarTimeEEPROM(i);
      guardarBillEEPROM(i);
      guardarFillsEEPROM(i);
      guardarCantEEPROM(i);
      guardarAcumBillEEPROM(i);
    }

    strncpy(BT_NAME, "AvtBt001", BT_NAME_MAX);
    BT_NAME[BT_NAME_MAX] = '\0';
    for (uint8_t i = 0; i < BT_NAME_MAX; i++) EEPROM.write(EE_ADDR_BTNAME + i, BT_NAME[i]);

    const char* defaults[4] = {"Bomba 1","Bomba 2","Bomba 3","Bomba 4"};
    for (uint8_t i = 0; i < 4; i++) {
      memset(bombaName[i], 0, 17);
      strncpy(bombaName[i], defaults[i], 16);
      guardarBombaNameEEPROM(i);
    }

    EEPROM.write(EE_ADDR_FLAG, EE_FLAG_INIT);
  }

  for (uint8_t i = 0; i < MACHINE_ID_MAX; i++) {
    MACHINE_ID[i] = EEPROM.read(EE_ADDR_MACHINEID + i);
  }
  MACHINE_ID[MACHINE_ID_MAX] = '\0';

  if (!isAlphaNumStr(MACHINE_ID, 1, MACHINE_ID_MAX)) {
    memset(MACHINE_ID, 0, sizeof(MACHINE_ID));
    strncpy(MACHINE_ID, DEFAULT_MACHINE_ID, MACHINE_ID_MAX);
    MACHINE_ID[MACHINE_ID_MAX] = '\0';
    guardarMachineIDEEPROM();
  }

  for (int i = 0; i < 4; i++) {
    surtidor_fill[i] = eepromReadULong(EE_ADDR_FILL + i*4);
    surtidor_time[i] = eepromReadULong(EE_ADDR_TIME + i*4);
    surtidor_bill[i] = eepromReadULong(EE_ADDR_BILL + i*4);
    surtidor_fills[i] = eepromReadULong(EE_ADDR_FILLS + i*4);
    surtidor_cantML[i] = eepromReadULong(EE_ADDR_CANTML + i*4);
    surtidor_acumBill[i] = eepromReadULong(EE_ADDR_ACUMBILL + i*4);
  }

  for (uint8_t i = 0; i < BT_NAME_MAX; i++) BT_NAME[i] = EEPROM.read(EE_ADDR_BTNAME + i);
  BT_NAME[BT_NAME_MAX] = '\0';

  int baseAddr[4] = {EE_ADDR_BNAME0, EE_ADDR_BNAME1, EE_ADDR_BNAME2, EE_ADDR_BNAME3};
  for (uint8_t i = 0; i < 4; i++) {
    for (uint8_t j = 0; j < 16; j++) bombaName[i][j] = EEPROM.read(baseAddr[i] + j);
    bombaName[i][16] = '\0';
    if (!isPrintableName(bombaName[i], 1, 16)) {
      memset(bombaName[i], 0, 17);
      snprintf(bombaName[i], 17, "Bomba %u", i + 1);
      guardarBombaNameEEPROM(i);
    }
  }
}

IdTipo validarId(char* id) {
  if (!isAlphaNumStr(id, 1, MACHINE_ID_MAX)) return ID_INVALIDO;
  if (strcmp(id, SUPER_ID) == 0) return ID_SUPER;
  if (strcmp(id, MACHINE_ID) == 0) return ID_NORMAL;
  return ID_INVALIDO;
}

// ========== ISR ==========
ISR(PCINT2_vect) {
  uint8_t currentPortK = PINK;
  uint8_t changed = currentPortK ^ lastPortK;
  lastPortK = currentPortK;

  if ((changed & _BV(PK0)) && (currentPortK & _BV(PK0))) flujo_counts[0]++;
  if ((changed & _BV(PK1)) && (currentPortK & _BV(PK1))) flujo_counts[1]++;
  if ((changed & _BV(PK7)) && (currentPortK & _BV(PK7))) flujo_counts[2]++;
  if ((changed & _BV(PK3)) && (currentPortK & _BV(PK3))) flujo_counts[3]++;
}

// ========== LCD ==========
void activarCierreVisual(uint8_t idx, unsigned long billFinal, unsigned long now) {
  unsigned long pulsos = getPulsosParaLCDSafe(idx);
  cierreBill[idx] = billFinal;
  cierreAcum[idx] = surtidor_acumBill[idx];
  cierreVol[idx] = calcVolumenML(idx, pulsos);
  cierrePercent[idx] = calcPercent(idx, pulsos);
  cierreHasta[idx] = now + HOLD_FIN_MS;
  cierreActivo[idx] = true;

  lastTopText[idx][0] = '\0';
  lastBottomText[idx][0] = '\0';
}

void sendFrameOutTo(Stream &port, const char* id, const char* orden, uint8_t idx, unsigned long valor);
void sendFrameOutBoth(const char* id, const char* orden, uint8_t idx, unsigned long valor);

void finCicloBomba(uint8_t idx, unsigned long now, unsigned long valorFinal) {
  unsigned long valorFinalRedondeado = roundBillTo100(valorFinal);
  unsigned long pulsosFinales = getPulsosParaLCDSafe(idx);

  surtidor_fills[idx]++;
  guardarFillsEEPROM(idx);

  surtidor_acumBill[idx] += valorFinalRedondeado;
  guardarAcumBillEEPROM(idx);

  if (modoLearn[idx] && pulsosFinales > 0) {
    surtidor_fill[idx] = pulsosFinales;
    guardarFillEEPROM(idx);
    sendFrameOutBoth(MACHINE_ID, "setlrn", idx + 1, surtidor_fill[idx]);
    modoLearn[idx] = false;
  }

  activarCierreVisual(idx, valorFinalRedondeado, now);

  digitalWrite((idx==0)?Bomba1_PIN:(idx==1)?Bomba2_PIN:(idx==2)?Bomba3_PIN:Bomba4_PIN, LOW);
  BombaActiva[idx] = false;
  marchaToggle[idx] = false;
  modoJogActivo[idx] = false;
  BombaReadyTime[idx] = now + RESTART_DELAY_MS;
  lastBillStepShown[idx] = 999999UL;

  Beep_On();
}

void renderPantallaNormal(uint8_t idx) {
  char line2[17];

  if (usaLCDSuperior(idx)) {
    lcdWriteTopIfChanged(idx, bombaName[idx]);
  }

  unsigned long billShow = surtidor_bill[idx];
  uint8_t percentShow = 0;
  unsigned long volShow = surtidor_cantML[idx];

  if (BombaActiva[idx]) {
    unsigned long pulsos = getPulsosParaLCDSafe(idx);

    if (modoJogActivo[idx]) {
      billShow = jogBillActual[idx];
      volShow = jogVolActual[idx];
    } else {
      billShow = calcBillProgresivo(idx, pulsos);
      volShow = calcVolumenML(idx, pulsos);
    }

    percentShow = calcPercent(idx, pulsos);
  }

  snprintf(line2, sizeof(line2), "$%lu %3u%% %lumL", billShow, percentShow, volShow);
  lcdWriteBottomIfChanged(idx, line2);
}

void renderPantallaCierre(uint8_t idx) {
  char top[17];
  char bottom[17];

  snprintf(top, sizeof(top), "$%lu", cierreAcum[idx]);
  snprintf(bottom, sizeof(bottom), "$%lu %3u%% %lumL", cierreBill[idx], cierrePercent[idx], cierreVol[idx]);

  if (usaLCDSuperior(idx)) {
    lcdWriteTopIfChanged(idx, top);
  }

  lcdWriteBottomIfChanged(idx, bottom);
}

void refrescarLCD(uint8_t idx, unsigned long now) {
  if (cierreActivo[idx]) {
    if (now < cierreHasta[idx]) {
      renderPantallaCierre(idx);
      return;
    } else {
      cierreActivo[idx] = false;
      lcdForceRefreshNormal(idx);
      renderPantallaNormal(idx);
      return;
    }
  }

  renderPantallaNormal(idx);
}

// ========== PROTOCOLO ==========
void sendFrameOutTo(Stream &port, const char* id, const char* orden, uint8_t idx, unsigned long valor) {
  port.print(PROTO_INI_TX);
  port.print(id);
  port.print(',');
  port.print(orden);
  port.print(',');
  port.print(idx);
  port.print(',');
  port.print(valor);
  port.print(',');
  port.print(PROTO_FIN_TX);
  port.print("\r\n");
}

void sendFrameOutBoth(const char* id, const char* orden, uint8_t idx, unsigned long valor) {
  sendFrameOutTo(Serial, id, orden, idx, valor);
  sendFrameOutTo(Serial1, id, orden, idx, valor);
}

bool readFrame(Stream &port, char* buf, uint8_t &idx, bool &started, char startMarker, char endMarker) {
  while (port.available() > 0) {
    char c = port.read();

    if (!started) {
      if (c == startMarker) {
        started = true;
        idx = 0;
      }
    } else {
      if (c == endMarker) {
        buf[idx] = '\0';
        started = false;
        idx = 0;
        return true;
      } else {
        if (idx < BUF_LEN - 1) {
          buf[idx++] = c;
        } else {
          buf[BUF_LEN - 1] = '\0';
          started = false;
          idx = 0;
        }
      }
    }
  }
  return false;
}

void procesarTramaProtocolo(char* trama) {
  const uint8_t MAX_FIELDS = 8;
  char* campos[MAX_FIELDS];
  uint8_t nFields = 0;

  char* p = trama;
  campos[nFields++] = p;

  while (*p != '\0' && nFields < MAX_FIELDS) {
    if (*p == ',') {
      *p = '\0';
      if (*(p + 1) != '\0') campos[nFields++] = p + 1;
    }
    p++;
  }

  if (nFields < 4) {
    sendFrameOutBoth(MACHINE_ID, "ERR", 0, 1);
    return;
  }

  char* pId    = campos[0];
  char* pOrden = campos[1];
  char* pIdx   = campos[2];
  char* pValor = campos[3];

  if (!pId || !pOrden || !pIdx || !pValor) {
    sendFrameOutBoth(MACHINE_ID, "ERR", 0, 2);
    return;
  }

  IdTipo tipoId = validarId(pId);
  if (tipoId == ID_INVALIDO) {
    sendFrameOutBoth(MACHINE_ID, "ERR", 0, 3);
    return;
  }

  int idx = atoi(pIdx);
  if (idx < 0 || idx > 4) {
    sendFrameOutBoth(MACHINE_ID, "ERR", 0, 4);
    return;
  }

  if (strcmp(pOrden, "chgID") == 0) {
    if (idx != 0) {
      sendFrameOutBoth(MACHINE_ID, "ERR", 0, 10);
      return;
    }

    if (!isAlphaNumStr(pValor, 1, MACHINE_ID_MAX)) {
      sendFrameOutBoth(MACHINE_ID, "ERR", 0, 11);
      return;
    }

    memset(MACHINE_ID, 0, sizeof(MACHINE_ID));
    strncpy(MACHINE_ID, pValor, MACHINE_ID_MAX);
    MACHINE_ID[MACHINE_ID_MAX] = '\0';
    guardarMachineIDEEPROM();

    sendFrameOutBoth(MACHINE_ID, "chgID", 0, 1);
    return;
  }

  if (strcmp(pOrden, "setRst") == 0) {
    if (idx != 0) {
      sendFrameOutBoth(MACHINE_ID, "ERR", 0, 12);
      return;
    }

    unsigned long valorRst = strtoul(pValor, NULL, 10);
    if (valorRst != 1) {
      sendFrameOutBoth(MACHINE_ID, "ERR", 0, 13);
      return;
    }

    sendFrameOutBoth(MACHINE_ID, "setRst", 0, 1);
    softwareReset();
    return;
  }

  if (strcmp(pOrden, "setNameB") == 0) {
    if (idx < 1 || idx > 4) {
      sendFrameOutBoth(MACHINE_ID, "ERR", 0, 14);
      return;
    }

    if (!isPrintableName(pValor, 1, 16)) {
      sendFrameOutBoth(MACHINE_ID, "ERR", 0, 15);
      return;
    }

    uint8_t posName = idx - 1;
    memset(bombaName[posName], 0, sizeof(bombaName[posName]));
    strncpy(bombaName[posName], pValor, 16);
    bombaName[posName][16] = '\0';

    guardarBombaNameEEPROM(posName);
    lcdForceRefreshNormal(posName);
    refrescarLCD(posName, millis());

    sendFrameOutBoth(MACHINE_ID, "setNameB", idx, 1);
    return;
  }

  if (idx < 1 || idx > 4) {
    sendFrameOutBoth(MACHINE_ID, "ERR", 0, 4);
    return;
  }

  unsigned long valor = strtoul(pValor, NULL, 10);
  uint8_t pos = idx - 1;

  if (strcmp(pOrden, "fill") == 0) {
    surtidor_fill[pos] = valor;
    guardarFillEEPROM(pos);
    sendFrameOutBoth(MACHINE_ID, "fill", idx, surtidor_fill[pos]);
    return;
  }

  if (strcmp(pOrden, "bill") == 0) {
    surtidor_bill[pos] = valor;
    guardarBillEEPROM(pos);
    lcdForceRefreshNormal(pos);
    sendFrameOutBoth(MACHINE_ID, "bill", idx, surtidor_bill[pos]);
    return;
  }

  if (strcmp(pOrden, "time") == 0) {
    surtidor_time[pos] = valor;
    guardarTimeEEPROM(pos);
    sendFrameOutBoth(MACHINE_ID, "time", idx, surtidor_time[pos]);
    return;
  }

  if (strcmp(pOrden, "setCant") == 0) {
    surtidor_cantML[pos] = valor;
    guardarCantEEPROM(pos);
    lcdForceRefreshNormal(pos);
    sendFrameOutBoth(MACHINE_ID, "setCant", idx, surtidor_cantML[pos]);
    return;
  }

  if (strcmp(pOrden, "acumBill") == 0) {
    surtidor_acumBill[pos] = valor;
    guardarAcumBillEEPROM(pos);
    sendFrameOutBoth(MACHINE_ID, "acumBill", idx, surtidor_acumBill[pos]);
    return;
  }

  if (strcmp(pOrden, "clrAcumBill") == 0) {
    surtidor_acumBill[pos] = 0;
    guardarAcumBillEEPROM(pos);
    sendFrameOutBoth(MACHINE_ID, "clrAcumBill", idx, 0);
    return;
  }

  if (strcmp(pOrden, "setlrn") == 0) {
    modoLearn[pos] = (valor != 0);
    sendFrameOutBoth(MACHINE_ID, "setlrn", idx, modoLearn[pos] ? 1 : 0);
    return;
  }

  if (strcmp(pOrden, "start") == 0) {
    if (valor != 0 && !modoJogActivo[pos]) {
      marchaToggle[pos] = true;
    }
    sendFrameOutBoth(MACHINE_ID, "start", idx, marchaToggle[pos] ? 1 : 0);
    return;
  }

  if (strcmp(pOrden, "stop") == 0) {
    marchaToggle[pos] = false;
    modoJogActivo[pos] = false;
    modoLearn[pos] = false;
    digitalWrite((pos==0)?Bomba1_PIN:(pos==1)?Bomba2_PIN:(pos==2)?Bomba3_PIN:Bomba4_PIN, LOW);
    BombaActiva[pos] = false;
    BombaReadyTime[pos] = millis() + RESTART_DELAY_MS;
    lastBillStepShown[pos] = 999999UL;
    lcdForceRefreshNormal(pos);
    sendFrameOutBoth(MACHINE_ID, "stop", idx, 0);
    return;
  }

  if (strcmp(pOrden, "jog") == 0) {
    if (valor != 0) {
      marchaToggle[pos] = false;
      if (!BombaActiva[pos] && millis() >= BombaReadyTime[pos]) {
        BombaActiva[pos] = true;
        modoJogActivo[pos] = true;
        BombaStartTime[pos] = millis();

        noInterrupts();
        flujo_counts[pos] = 0;
        interrupts();

        jogBillActual[pos] = 0;
        jogVolActual[pos] = 0;
        lastBillStepShown[pos] = 999999UL;
        digitalWrite((pos==0)?Bomba1_PIN:(pos==1)?Bomba2_PIN:(pos==2)?Bomba3_PIN:Bomba4_PIN, HIGH);
        Beep_On();
        lcdForceRefreshNormal(pos);
      }
      sendFrameOutBoth(MACHINE_ID, "jog", idx, 1);
    } else {
      if (modoJogActivo[pos]) {
        finCicloBomba(pos, millis(), jogBillActual[pos]);
      }
      sendFrameOutBoth(MACHINE_ID, "jog", idx, 0);
    }
    return;
  }

  if (strcmp(pOrden, "getBill") == 0) {
    sendFrameOutBoth(MACHINE_ID, "getBill", idx, surtidor_bill[pos]);
    return;
  }

  if (strcmp(pOrden, "getAcumBill") == 0) {
    sendFrameOutBoth(MACHINE_ID, "getAcumBill", idx, surtidor_acumBill[pos]);
    return;
  }

  if (strcmp(pOrden, "getFill") == 0) {
    sendFrameOutBoth(MACHINE_ID, "getFill", idx, surtidor_fill[pos]);
    return;
  }

  if (strcmp(pOrden, "getCant") == 0) {
    sendFrameOutBoth(MACHINE_ID, "getCant", idx, surtidor_cantML[pos]);
    return;
  }

  if (strcmp(pOrden, "getTime") == 0) {
    sendFrameOutBoth(MACHINE_ID, "getTime", idx, surtidor_time[pos]);
    return;
  }

  if (strcmp(pOrden, "ping") == 0) {
    sendFrameOutBoth(MACHINE_ID, "pong", idx, valor);
    return;
  }

  sendFrameOutBoth(MACHINE_ID, "ERR", idx, 9);
}

void atenderProtocolosSerial() {
  if (readFrame(Serial, rxBufUSB, rxIndexUSB, frameStartedUSB, PROTO_INI_RX, PROTO_FIN_RX)) {
    procesarTramaProtocolo(rxBufUSB);
  }

  if (readFrame(Serial1, rxBufBT, rxIndexBT, frameStartedBT, PROTO_INI_RX, PROTO_FIN_RX)) {
    procesarTramaProtocolo(rxBufBT);
  }
}

void enviarEstadoBombaActiva(uint8_t i, unsigned long now) {
  if (!BombaActiva[i]) return;
  if (now - lastStatusTx[i] < STATUS_TX_MS) return;

  lastStatusTx[i] = now;

  unsigned long pulsos = getPulsosParaLCDSafe(i);
  unsigned long valorActual = modoJogActivo[i] ? jogBillActual[i] : calcBillProgresivo(i, pulsos);
  unsigned long volActual = modoJogActivo[i] ? jogVolActual[i] : calcVolumenML(i, pulsos);
  uint8_t pctActual = calcPercent(i, pulsos);

  sendFrameOutBoth(MACHINE_ID, "statBill", i + 1, valorActual);
  sendFrameOutBoth(MACHINE_ID, "statVol",  i + 1, volActual);
  sendFrameOutBoth(MACHINE_ID, "statPct",  i + 1, pctActual);
  sendFrameOutBoth(MACHINE_ID, "statRun",  i + 1, 1);
}

void enviarEstadosActivos(unsigned long now) {
  for (uint8_t i = 0; i < 4; i++) {
    enviarEstadoBombaActiva(i, now);
  }
}

void lcdInitAll() {
  lcd12.init(); lcd12.backlight(); lcd12.clear();
  lcd34.init(); lcd34.backlight(); lcd34.clear();

  lcdForceRefreshNormal(0);
  lcdForceRefreshNormal(1);
  lcdForceRefreshNormal(2);
  lcdForceRefreshNormal(3);

  renderPantallaNormal(0);
  renderPantallaNormal(1);
  renderPantallaNormal(2);
  renderPantallaNormal(3);
}

// ========== SETUP ==========
void setup() {
  pinMode(Bomba1_PIN, OUTPUT); pinMode(Bomba2_PIN, OUTPUT); pinMode(Bomba3_PIN, OUTPUT); pinMode(Bomba4_PIN, OUTPUT);
  pinMode(Buzzer_PIN, OUTPUT);

  pinMode(Opto1_PIN, INPUT_PULLUP); pinMode(Opto2_PIN, INPUT_PULLUP); pinMode(Opto3_PIN, INPUT_PULLUP); pinMode(Opto4_PIN, INPUT_PULLUP);

  pinMode(BtnStart1_PIN, INPUT_PULLUP); pinMode(BtnJog1_PIN, INPUT_PULLUP);
  pinMode(BtnStart2_PIN, INPUT_PULLUP); pinMode(BtnJog2_PIN, INPUT_PULLUP);
  pinMode(BtnStart3_PIN, INPUT_PULLUP); pinMode(BtnJog3_PIN, INPUT_PULLUP);
  pinMode(BtnStart4_PIN, INPUT_PULLUP); pinMode(BtnJog4_PIN, INPUT_PULLUP);

  pinMode(BtKey_PIN, OUTPUT);
  digitalWrite(BtKey_PIN, LOW);

  pinMode(flujoPin1, INPUT);
  pinMode(flujoPin2, INPUT);
  pinMode(flujoPin3, INPUT);
  pinMode(flujoPin4, INPUT);

  Serial.begin(SERIAL_BAUD);
  Serial1.begin(9600);

  Wire.begin();
  cargarDesdeEEPROM();
  lcdInitAll();

  DDRK = 0x00;
  PORTK = 0xFF;
  PCICR |= (1 << PCIE2);
  PCMSK2 |= 0xFF;
  lastPortK = PINK;

  if (mcusr_mirror & _BV(WDRF)) lastResetCause = "WDT";
  else if (mcusr_mirror & _BV(PORF)) lastResetCause = "POR";
  else if (mcusr_mirror & _BV(BORF)) lastResetCause = "BOR";
  else if (mcusr_mirror & _BV(EXTRF)) lastResetCause = "EXT";
  else lastResetCause = "UNKNOWN";

  sendFrameOutBoth(MACHINE_ID, "BOOT", 0, 85);
  Beep_On();
}

// ========== LOOP ==========
void loop() {
  unsigned long now = millis();

  atenderProtocolosSerial();

  int rawStart[4] = {
    digitalRead(BtnStart1_PIN), digitalRead(BtnStart2_PIN),
    digitalRead(BtnStart3_PIN), digitalRead(BtnStart4_PIN)
  };
  int rawJog[4] = {
    digitalRead(BtnJog1_PIN), digitalRead(BtnJog2_PIN),
    digitalRead(BtnJog3_PIN), digitalRead(BtnJog4_PIN)
  };

  for (uint8_t i = 0; i < 4; i++) {
    if (rawStart[i] != debouncedStart[i] && now - lastChangeStart[i] >= DEBOUNCE_MS) {
      debouncedStart[i] = rawStart[i];
      lastChangeStart[i] = now;
    }
    if (rawJog[i] != debouncedJog[i] && now - lastChangeJog[i] >= DEBOUNCE_MS) {
      debouncedJog[i] = rawJog[i];
      lastChangeJog[i] = now;
    }
  }

  bool startPressed[4];
  bool jogPressed[4];
  bool startEdge[4];
  bool jogEdgeOff[4];
  bool envaseOk[4];

  for (uint8_t i = 0; i < 4; i++) {
    startPressed[i] = (debouncedStart[i] == LOW);
    jogPressed[i]   = (debouncedJog[i] == LOW);
    startEdge[i]    = startPressed[i] && !prevStartPressed[i];
    jogEdgeOff[i]   = (!jogPressed[i] && prevJogPressed[i]);
    prevStartPressed[i] = startPressed[i];
    prevJogPressed[i]   = jogPressed[i];
  }

  envaseOk[0] = (digitalRead(Opto1_PIN) == 0);
  envaseOk[1] = (digitalRead(Opto2_PIN) == 0);
  envaseOk[2] = (digitalRead(Opto3_PIN) == 0);
  envaseOk[3] = (digitalRead(Opto4_PIN) == 0);

  for (uint8_t i = 0; i < 4; i++) {
    if (startEdge[i] && envaseOk[i] && now >= BombaReadyTime[i] && !modoJogActivo[i]) {
      marchaToggle[i] = !marchaToggle[i];
      Beep_Short();
    }

    bool jogPuro = jogPressed[i] && !marchaToggle[i];

    if (jogPuro && !BombaActiva[i] && envaseOk[i] && now >= BombaReadyTime[i]) {
      BombaActiva[i] = true;
      modoJogActivo[i] = true;
      BombaStartTime[i] = now;

      noInterrupts();
      flujo_counts[i] = 0;
      interrupts();

      jogBillActual[i] = 0;
      jogVolActual[i] = 0;
      lastBillStepShown[i] = 999999UL;
      digitalWrite((i==0)?Bomba1_PIN:(i==1)?Bomba2_PIN:(i==2)?Bomba3_PIN:Bomba4_PIN, HIGH);
      Beep_On();
      lcdForceRefreshNormal(i);
      refrescarLCD(i, now);
    }

    if (marchaToggle[i] && !BombaActiva[i] && envaseOk[i] && now >= BombaReadyTime[i]) {
      BombaActiva[i] = true;
      modoJogActivo[i] = false;
      BombaStartTime[i] = now;

      noInterrupts();
      flujo_counts[i] = 0;
      interrupts();

      lastBillStepShown[i] = 999999UL;
      digitalWrite((i==0)?Bomba1_PIN:(i==1)?Bomba2_PIN:(i==2)?Bomba3_PIN:Bomba4_PIN, HIGH);
      Beep_On();
      lcdForceRefreshNormal(i);
      refrescarLCD(i, now);
    }

    if (BombaActiva[i]) {
      unsigned long pulsos = getPulsosParaLCDSafe(i);
      bool sinEnvase = !envaseOk[i];

      if (modoJogActivo[i]) {
        jogBillActual[i] = calcBillProgresivo(i, pulsos);
        jogVolActual[i]  = calcVolumenML(i, pulsos);

        unsigned long step = floorBillStep(jogBillActual[i]);
        if (step != lastBillStepShown[i]) {
          lastBillStepShown[i] = step;
          refrescarLCD(i, now);
        }

        if (jogEdgeOff[i] || sinEnvase) {
          unsigned long billFinalRedondeado = roundBillTo100(jogBillActual[i]);
          finCicloBomba(i, now, billFinalRedondeado);
          refrescarLCD(i, now);
          sendFrameOutBoth(MACHINE_ID, "end", i + 1, billFinalRedondeado);
          continue;
        }
      } else {
        unsigned long billActual = calcBillProgresivo(i, pulsos);
        unsigned long step = floorBillStep(billActual);

        if (step != lastBillStepShown[i]) {
          lastBillStepShown[i] = step;
          refrescarLCD(i, now);
        }

        bool finPulsos = (surtidor_fill[i] > 0) && (pulsos >= surtidor_fill[i]);
        bool finTime   = (surtidor_time[i] > 0) && ((now - BombaStartTime[i]) >= surtidor_time[i]);

        if (finPulsos || finTime || sinEnvase) {
          unsigned long billFinalProceso = calcBillProgresivo(i, pulsos);
          unsigned long billFinalRedondeado = roundBillTo100(billFinalProceso);
          finCicloBomba(i, now, billFinalRedondeado);
          refrescarLCD(i, now);
          sendFrameOutBoth(MACHINE_ID, "end", i + 1, billFinalRedondeado);
          continue;
        }

        if (!marchaToggle[i]) {
          digitalWrite((i==0)?Bomba1_PIN:(i==1)?Bomba2_PIN:(i==2)?Bomba3_PIN:Bomba4_PIN, LOW);
          BombaActiva[i] = false;
          BombaReadyTime[i] = now + RESTART_DELAY_MS;
          lastBillStepShown[i] = 999999UL;
          lcdForceRefreshNormal(i);
          refrescarLCD(i, now);
          sendFrameOutBoth(MACHINE_ID, "stop", i + 1, 0);
        }
      }
    } else {
      refrescarLCD(i, now);
    }
  }

  enviarEstadosActivos(now);
}