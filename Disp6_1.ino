#include <EEPROM.h>
#include <avr/wdt.h>
#include <avr/io.h>
#include <stdio.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>


/* "C:\\Users\\THINKPAD\\AppData\\Local\\Arduino15\\packages\\arduino\\tools\\avr-gcc\\7.3.0-atmel3.6.1-arduino7/bin/avr-size" -A "C:\\Users\\THINKPAD\\AppData\\Local\\arduino\\sketches\\9FD7C89546DA468973117595F22ACEC9/Disp6_0.ino.elf"
Sketch uses 17806 bytes (7%) of program storage space. Maximum is 253952 bytes.
Global variables use 2609 bytes (31%) of dynamic memory, leaving 5583 bytes for local variables. Maximum is 8192 bytes.
Verion 6 con operatibilidad LCD corregida, porcentaje, 2 bombas 1 y 3 contando la Cero 
*/

// ========== VERSION FIRMWARE ==========
#define FW_MAJOR 6
#define FW_MINOR 0
const char FW_VERSION_STR[] = "6.0";

// ========== TIPOS ==========
enum IdTipo {
  ID_INVALIDO = 0,
  ID_NORMAL,
  ID_SUPER
};

// ========== PROTOTIPOS ==========
IdTipo validarId(char* id);
void cargarDesdeEEPROM();
void guardarMachineIdEEPROM();
void guardarFillEEPROM(uint8_t idx);
void guardarTimeEEPROM(uint8_t idx);
void guardarBillEEPROM(uint8_t idx);
void guardarFillsEEPROM(uint8_t idx);
void guardarCantEEPROM(uint8_t idx);
void guardarBtNameEEPROM();
void guardarBombaNameEEPROM(uint8_t idx);
void procesarTrama(char* trama);
void finCicloBomba(uint8_t idx, unsigned long now);
void programarHC05ConBTName();
void Beep_On();
void Beep_Short();
void enviarInfo();

// ========== MCUSR / CAUSA RESET ==========
uint8_t mcusr_mirror __attribute__ ((section (".noinit")));
const char* lastResetCause = "UNKNOWN";

void getMCUSR(void) __attribute__((naked)) __attribute__((section(".init3")));
void getMCUSR(void) {
  mcusr_mirror = MCUSR;
  MCUSR = 0;
}

// ========== HARDWARE ==========
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
const int BtnStop1_PIN  = 43;
const int BtnStart2_PIN = 40;
const int BtnStop2_PIN  = 42;
const int BtnStart3_PIN = 45;
const int BtnStop3_PIN  = 46;
const int BtnStart4_PIN = 44;
const int BtnStop4_PIN  = 47;

const int BtKey_PIN = 9;

// Simulación de botones y learn
bool btnSimStart[4] = {false, false, false, false};
bool btnSimStop[4]  = {false, false, false, false};
bool cicloPorBtn[4] = {false, false, false, false};
bool learnMode[4]   = {false, false, false, false};

// Estados bomba
unsigned long BombaStartTime[4] = {0,0,0,0};
bool BombaActiva[4]             = {false,false,false,false};
unsigned long BombaReadyTime[4] = {0,0,0,0};
// Tiempo y porcentaje al detenerse
unsigned long BombaStopTime[4]  = {0,0,0,0};
uint8_t BombaLastPercent[4]     = {0,0,0,0};

const unsigned long DEFAULT_FILL_PULSOS = 3000;
const unsigned long DEFAULT_TIME_MS     = 30000;
const unsigned long RESTART_DELAY_MS    = 20000;
const char DEFAULT_MACHINE_ID[6]        = "AB12C";
char BT_NAME[16]                        = "AvtBt001";

// Sensores flujo (PORTK PCINT2)
// flujo_counts[0] -> sensor 1
// flujo_counts[1] -> sensor 2  -> Bomba 1 (LCD12)
// flujo_counts[2] -> sensor 3
// flujo_counts[3] -> sensor 4  -> Bomba 3 (LCD34)
volatile unsigned long flujo_counts[4] = {0, 0, 0, 0};
uint8_t lastPortK;

// Pines analógicos
const byte flujoPin1 = A8;
const byte flujoPin2 = A9;
const byte flujoPin3 = A10;
const byte flujoPin4 = A11;

// ========== PROTOCOLO / CONFIG ==========
#define SERIAL_BAUD 115200
#define BUF_LEN     64

char rxBufUSB[BUF_LEN];
uint8_t rxIndexUSB = 0;
bool frameStartedUSB = false;

char rxBufBT[BUF_LEN];
uint8_t rxIndexBT = 0;
bool frameStartedBT = false;

char MACHINE_ID[6] = "AB12C";
const char SUPER_ID[] = "avt99";

unsigned long surtidor_fill[4]  = {
  DEFAULT_FILL_PULSOS, DEFAULT_FILL_PULSOS,
  DEFAULT_FILL_PULSOS, DEFAULT_FILL_PULSOS
};
unsigned long surtidor_bill[4]  = {0, 0, 0, 0};
unsigned long surtidor_time[4]  = {
  DEFAULT_TIME_MS, DEFAULT_TIME_MS,
  DEFAULT_TIME_MS, DEFAULT_TIME_MS
};
unsigned long surtidor_fills[4] = {0, 0, 0, 0};
// mL asignados a cada bomba (cantidad objetivo)
unsigned long surtidor_cantML[4] = {0,0,0,0};

// Nombres por bomba
char bombaName[4][16] = {
  "Bomba 1", "Bomba 2", "Bomba 3", "Bomba 4"
};

// ========== EEPROM MAP ==========
const int EE_ADDR_FLAG      = 0;
const int EE_ADDR_MACHINEID = 1;
const int EE_ADDR_FILL      = 6;
const int EE_ADDR_TIME      = 22;
const int EE_ADDR_BILL      = 38;
const int EE_ADDR_FILLS     = 54;
const int EE_ADDR_BTNAME    = 70;
const byte EE_FLAG_INIT     = 0xA5;
const uint8_t BT_NAME_MAX   = 15;

// 4 bombas * 4 bytes c/u = 16 bytes
const int EE_ADDR_CANTML    = EE_ADDR_BTNAME + BT_NAME_MAX;      // 85
const int EE_ADDR_BNAME0    = EE_ADDR_CANTML + 4*4;              // 101
const int EE_ADDR_BNAME1    = EE_ADDR_BNAME0 + 16;               // 117
const int EE_ADDR_BNAME2    = EE_ADDR_BNAME1 + 16;               // 133
const int EE_ADDR_BNAME3    = EE_ADDR_BNAME2 + 16;               // 149

// ========== LCDs I2C ==========
#define LCD12_ADDR 0x26   // Bomba 1 en LCD12
#define LCD34_ADDR 0x25   // Bomba 3 en LCD34

LiquidCrystal_I2C lcd12(LCD12_ADDR, 16, 2);
LiquidCrystal_I2C lcd34(LCD34_ADDR, 16, 2);

// ========== SALIDA SERIE DUAL ==========
void sendFrameRaw(const char* s) {
  Serial.print('%');
  Serial.print(s);
  Serial.print(',');
  Serial.print('$');
  Serial.print("\r\n");

  Serial1.print('%');
  Serial1.print(s);
  Serial1.print(',');
  Serial1.print('$');
  Serial1.print("\r\n");
}

void sendDebug(const char* s) {
  sendFrameRaw(s);
}

void sendBoot() {
  char buf[96];
  snprintf(buf, sizeof(buf),
           "BOOT,%s,%s,CAUSE=%s,FW=%s",
           MACHINE_ID, BT_NAME, lastResetCause, FW_VERSION_STR);
  sendFrameRaw(buf);
}

void enviarInfo() {
  char buf[96];
  snprintf(buf, sizeof(buf),
           "INFO,%s,%s,CAUSE=%s,FW=%s",
           MACHINE_ID, BT_NAME, lastResetCause, FW_VERSION_STR);
  sendFrameRaw(buf);
}

// ========== LCD HELPERS ==========
void lcdInitAll() {
  lcd12.init();
  lcd12.backlight();
  lcd12.clear();

  lcd34.init();
  lcd34.backlight();
  lcd34.clear();

  char line[17];

  // LCD12: bomba 1
  lcd12.setCursor(0, 0);
  snprintf(line, sizeof(line), "%-16s", bombaName[0]);
  lcd12.print(line);
  lcd12.setCursor(0, 1);
  lcd12.print("Listo B1        ");

  // LCD34: bomba 3
  lcd34.setCursor(0, 0);
  snprintf(line, sizeof(line), "%-16s", bombaName[2]);
  lcd34.print(line);
  lcd34.setCursor(0, 1);
  lcd34.print("Listo B3        ");
}

uint8_t calcPercent(uint8_t idx, unsigned long pulsosActuales) {
  unsigned long f = surtidor_fill[idx];
  //if (f == 0) return 0;
  if (pulsosActuales >= f) return 100;
  return (uint8_t)((pulsosActuales * 100UL) / f);
}

// Pulsos que usa cada bomba para el % que se muestra
unsigned long getPulsosParaLCD(uint8_t idx) {
  if (idx == 0) return flujo_counts[1]; // Bomba1 -> sensor2
  if (idx == 2) return flujo_counts[3]; // Bomba3 -> sensor4
  return flujo_counts[idx];
}

// Muestra $, porcentaje y mL; 5 s después del fin muestra "Listo"
void lcdUpdateBomba(uint8_t idx, unsigned long bill, uint8_t percent, bool isLearn, unsigned long now) {
  LiquidCrystal_I2C *lcd = (idx < 2) ? &lcd12 : &lcd34;
  uint8_t line = 1;
  char buf[17];

  if (isLearn) {
    snprintf(buf, sizeof(buf), "Lrn $%lu       ", bill);
  } else if (BombaActiva[idx]) {
    snprintf(buf, sizeof(buf), "$%lu %3u%% %lumL", bill, percent, surtidor_cantML[idx]);
  } else {
    bool tieneStop = (BombaStopTime[idx] != 0);
    bool masDe5s   = tieneStop && (now - BombaStopTime[idx] >= 5000UL);

    if (tieneStop && !masDe5s) {
      uint8_t p = BombaLastPercent[idx];
      snprintf(buf, sizeof(buf), "$%lu %3u%% %lumL", bill, p, surtidor_cantML[idx]);
    } else if (tieneStop && masDe5s) {
      snprintf(buf, sizeof(buf), "$%lu OK  %lumL", bill, surtidor_cantML[idx]);
    } else {
      snprintf(buf, sizeof(buf), "$%lu   0%% %lumL", bill, surtidor_cantML[idx]);
    }
  }

  uint8_t len = strlen(buf);
  for (uint8_t i = len; i < 16; i++) buf[i] = ' ';
  buf[16] = '\0';

  lcd->setCursor(0, line);
  lcd->print(buf);
}

// ========== EEPROM helpers ==========
void eepromWriteULong(int addr, unsigned long value) {
  EEPROM.put(addr, value);
}
unsigned long eepromReadULong(int addr) {
  unsigned long v;
  EEPROM.get(addr, v);
  return v;
}

void cargarDesdeEEPROM() {
  byte flag = EEPROM.read(EE_ADDR_FLAG);
  if (flag != EE_FLAG_INIT) {
    for (int i = 0; i < 5; i++) {
      MACHINE_ID[i] = DEFAULT_MACHINE_ID[i];
      EEPROM.write(EE_ADDR_MACHINEID + i, DEFAULT_MACHINE_ID[i]);
    }
    MACHINE_ID[5] = '\0';

    for (int i = 0; i < 4; i++) {
      surtidor_fill[i]  = DEFAULT_FILL_PULSOS;
      eepromWriteULong(EE_ADDR_FILL  + i * 4, surtidor_fill[i]);

      surtidor_time[i]  = DEFAULT_TIME_MS;
      eepromWriteULong(EE_ADDR_TIME  + i * 4, surtidor_time[i]);

      surtidor_bill[i]  = 0;
      eepromWriteULong(EE_ADDR_BILL  + i * 4, surtidor_bill[i]);

      surtidor_fills[i] = 0;
      eepromWriteULong(EE_ADDR_FILLS + i * 4, surtidor_fills[i]);

      surtidor_cantML[i] = 0;
      eepromWriteULong(EE_ADDR_CANTML + i * 4, surtidor_cantML[i]);
    }

    strncpy(BT_NAME, "AvtBt001", BT_NAME_MAX);
    BT_NAME[BT_NAME_MAX] = '\0';
    for (uint8_t i = 0; i < BT_NAME_MAX; i++) {
      EEPROM.write(EE_ADDR_BTNAME + i, BT_NAME[i]);
    }

    const char* defaultNames[4] = { "Bomba 1", "Bomba 2", "Bomba 3", "Bomba 4" };
    int baseAddr[4] = { EE_ADDR_BNAME0, EE_ADDR_BNAME1, EE_ADDR_BNAME2, EE_ADDR_BNAME3 };
    for (uint8_t i = 0; i < 4; i++) {
      memset(bombaName[i], 0, 16);
      strncpy(bombaName[i], defaultNames[i], 15);
      for (uint8_t j = 0; j < 16; j++) {
        EEPROM.write(baseAddr[i] + j, bombaName[i][j]);
      }
    }

    EEPROM.write(EE_ADDR_FLAG, EE_FLAG_INIT);
  } else {
    for (int i = 0; i < 5; i++) {
      MACHINE_ID[i] = EEPROM.read(EE_ADDR_MACHINEID + i);
    }
    MACHINE_ID[5] = '\0';

    for (int i = 0; i < 4; i++) {
      surtidor_fill[i]   = eepromReadULong(EE_ADDR_FILL  + i * 4);
      surtidor_time[i]   = eepromReadULong(EE_ADDR_TIME  + i * 4);
      surtidor_bill[i]   = eepromReadULong(EE_ADDR_BILL  + i * 4);
      surtidor_fills[i]  = eepromReadULong(EE_ADDR_FILLS + i * 4);
      surtidor_cantML[i] = eepromReadULong(EE_ADDR_CANTML + i * 4);
    }

    for (uint8_t i = 0; i < BT_NAME_MAX; i++) {
      BT_NAME[i] = EEPROM.read(EE_ADDR_BTNAME + i);
    }
    BT_NAME[BT_NAME_MAX] = '\0';

    int baseAddr2[4] = { EE_ADDR_BNAME0, EE_ADDR_BNAME1, EE_ADDR_BNAME2, EE_ADDR_BNAME3 };
    for (uint8_t i = 0; i < 4; i++) {
      for (uint8_t j = 0; j < 16; j++) {
        bombaName[i][j] = EEPROM.read(baseAddr2[i] + j);
      }
      bombaName[i][15] = '\0';
    }
  }
}

void guardarMachineIdEEPROM() {
  for (int i = 0; i < 5; i++) EEPROM.write(EE_ADDR_MACHINEID + i, MACHINE_ID[i]);
}
void guardarFillEEPROM(uint8_t idx)  { eepromWriteULong(EE_ADDR_FILL  + idx * 4, surtidor_fill[idx]);  }
void guardarTimeEEPROM(uint8_t idx)  { eepromWriteULong(EE_ADDR_TIME  + idx * 4, surtidor_time[idx]);  }
void guardarBillEEPROM(uint8_t idx)  { eepromWriteULong(EE_ADDR_BILL  + idx * 4, surtidor_bill[idx]);  }
void guardarFillsEEPROM(uint8_t idx) { eepromWriteULong(EE_ADDR_FILLS + idx * 4, surtidor_fills[idx]); }
void guardarCantEEPROM(uint8_t idx)  { eepromWriteULong(EE_ADDR_CANTML + idx * 4, surtidor_cantML[idx]); }
void guardarBtNameEEPROM() {
  for (uint8_t i = 0; i < BT_NAME_MAX; i++) {
    EEPROM.write(EE_ADDR_BTNAME + i, BT_NAME[i]);
  }
}
void guardarBombaNameEEPROM(uint8_t idx) {
  if (idx > 3) return;
  int baseAddr;
  if (idx == 0) baseAddr = EE_ADDR_BNAME0;
  else if (idx == 1) baseAddr = EE_ADDR_BNAME1;
  else if (idx == 2) baseAddr = EE_ADDR_BNAME2;
  else baseAddr = EE_ADDR_BNAME3;

  for (uint8_t j = 0; j < 16; j++) {
    EEPROM.write(baseAddr + j, bombaName[idx][j]);
  }
}

// ========== VALIDACIÓN ID ==========
IdTipo validarId(char* id) {
  if (!id || strlen(id) != 5) return ID_INVALIDO;
  for (uint8_t i = 0; i < 5; i++) {
    char c = id[i];
    bool isDigit = (c >= '0' && c <= '9');
    bool isUpper = (c >= 'A' && c <= 'Z');
    bool isLower = (c >= 'a' && c <= 'z');
    if (!(isDigit || isUpper || isLower)) return ID_INVALIDO;
  }
  if (strcmp(id, SUPER_ID) == 0) return ID_SUPER;
  if (strcmp(id, MACHINE_ID) == 0) return ID_NORMAL;
  return ID_INVALIDO;
}

// ========== ISR PCINT2 ==========
ISR(PCINT2_vect) {
  uint8_t currentPortK = PINK;
  uint8_t changed = currentPortK ^ lastPortK;
  lastPortK = currentPortK;

  if ((changed & _BV(PK0)) && (currentPortK & _BV(PK0))) flujo_counts[0]++;   // sensor 1 PK2
  if ((changed & _BV(PK1)) && (currentPortK & _BV(PK1))) flujo_counts[1]++;   // sensor 2 -> B1 PK0
  if ((changed & _BV(PK7)) && (currentPortK & _BV(PK7))) flujo_counts[2]++;   // sensor 3  PK3
  if ((changed & _BV(PK3)) && (currentPortK & _BV(PK3))) flujo_counts[3]++;   // sensor 4 -> B3  PK1
}

// ========== AUXILIARES ==========
void Beep_On() {
  digitalWrite(Buzzer_PIN, HIGH);
  delay(25);
  digitalWrite(Buzzer_PIN, LOW);
}

void Beep_Short() {
  digitalWrite(Buzzer_PIN, HIGH);
  delay(25);
  digitalWrite(Buzzer_PIN, LOW);
}

void programarHC05ConBTName() {
  sendDebug("DBG_BT: Entrando AT");
  digitalWrite(BtKey_PIN, HIGH);
  delay(100);

  Serial1.end();
  delay(50);
  Serial1.begin(38400);
  delay(500);

  Serial1.print("AT+NAME=");
  Serial1.print(BT_NAME);
  Serial1.print("\r\n");

  Serial.print("%DBG_BT: AT+NAME=");
  Serial.print(BT_NAME);
  Serial.print(",$\r\n");

  unsigned long t0 = millis();
  while (millis() - t0 < 1000) {
    if (Serial1.available()) {
      char c = Serial1.read();
      Serial.write(c);
    }
  }

  Serial1.end();
  delay(50);
  digitalWrite(BtKey_PIN, LOW);
  delay(50);
  Serial1.begin(9600);

  sendDebug("DBG_BT: Salgo AT");
}

// ========== PROCESAR TRAMA ==========
void procesarTrama(char* trama) {
  const uint8_t MAX_FIELDS = 8;
  char* campos[MAX_FIELDS];
  uint8_t nFields = 0;

  char* p = trama;
  campos[nFields++] = p;

  while (*p != '\0' && nFields < MAX_FIELDS) {
    if (*p == ',') {
      *p = '\0';
      if (*(p + 1) != '\0') {
        campos[nFields++] = p + 1;
      }
    }
    p++;
  }

  if (nFields < 3) {
    sendFrameRaw("ERR: Muy pocos campos");
    return;
  }

  char* pTipo = campos[0];
  char* pId   = campos[1];
  char* pCmd  = campos[2];
  char* pIdx  = (nFields > 3) ? campos[3] : NULL;
  char* pVal  = (nFields > 4) ? campos[4] : NULL;

  if (!pId || !pCmd || strlen(pId) == 0 || strlen(pCmd) == 0) {
    sendFrameRaw("ERR: Trama incompleta (ID/CMD)");
    return;
  }

  IdTipo tipo = validarId(pId);
  if (tipo == ID_INVALIDO) {
    sendFrameRaw("ERR: ID invalido");
    return;
  }

  // chId
  if (strcmp(pCmd, "chId") == 0) {
    if (tipo != ID_SUPER) { sendFrameRaw("ERR: Permiso denegado para chId"); return; }
    if (!pIdx || strlen(pIdx) != 5) { sendFrameRaw("ERR: Nuevo ID invalido"); return; }
    for (uint8_t i = 0; i < 5; i++) {
      char c = pIdx[i];
      bool isDigit = (c >= '0' && c <= '9');
      bool isUpper = (c >= 'A' && c <= 'Z');
      bool isLower = (c >= 'a' && c <= 'z');
      if (!(isDigit || isUpper || isLower)) { sendFrameRaw("ERR: Nuevo ID invalido"); return; }
    }
    strncpy(MACHINE_ID, pIdx, 5);
    MACHINE_ID[5] = '\0';
    guardarMachineIdEEPROM();
    char buf[48];
    snprintf(buf, sizeof(buf), "OK: Nuevo MACHINE_ID = %s", MACHINE_ID);
    sendFrameRaw(buf);
    return;
  }

  // btName
  if (strcmp(pCmd, "btName") == 0) {
    if (tipo != ID_SUPER) { sendFrameRaw("ERR: Permiso denegado para btName"); return; }
    if (!pIdx || strlen(pIdx) == 0) { sendFrameRaw("ERR: Nombre BT ausente"); return; }
    size_t len = strlen(pIdx);
    if (len == 0 || len > BT_NAME_MAX) { sendFrameRaw("ERR: Nombre BT invalido"); return; }
    strncpy(BT_NAME, pIdx, BT_NAME_MAX);
    BT_NAME[BT_NAME_MAX] = '\0';
    guardarBtNameEEPROM();
    char buf[48];
    snprintf(buf, sizeof(buf), "OK: Nuevo BT_NAME = %s", BT_NAME);
    sendFrameRaw(buf);
    return;
  }

  // btProg
  if (strcmp(pCmd, "btProg") == 0) {
    if (tipo != ID_SUPER) { sendFrameRaw("ERR: Permiso denegado para btProg"); return; }
    programarHC05ConBTName();
    sendFrameRaw("OK: HC05 programado con BT_NAME");
    return;
  }

  // btKey
  if (strcmp(pCmd, "btKey") == 0) {
    if (tipo != ID_SUPER) { sendFrameRaw("ERR: Permiso denegado para btKey"); return; }
    if (!pIdx || strlen(pIdx) == 0) { sendFrameRaw("ERR: val btKey ausente"); return; }
    int valKey = atoi(pIdx);
    if (valKey == 0) {
      digitalWrite(BtKey_PIN, LOW);
      sendFrameRaw("OK: BT KEY=LOW (modo datos)");
    } else if (valKey == 1) {
      digitalWrite(BtKey_PIN, HIGH);
      sendFrameRaw("OK: BT KEY=HIGH (modo AT nivel logico)");
    } else {
      sendFrameRaw("ERR: val btKey invalido (use 0/1)");
    }
    return;
  }

  // reset
  if (strcmp(pCmd, "reset") == 0) {
    if (tipo != ID_SUPER) { sendFrameRaw("ERR: Permiso denegado para reset"); return; }
    sendFrameRaw("OK: Reset en 250 ms");
    delay(250);
    wdt_enable(WDTO_15MS);
    while (1) { }
  }

  // info
  if (strcmp(pCmd, "info") == 0) {
    if (tipo != ID_SUPER) { sendFrameRaw("ERR: Permiso denegado para info"); return; }
    enviarInfo();
    return;
  }

  // A partir de aquí comandos con idx 1..4
  if (!pIdx || strlen(pIdx) == 0) { sendFrameRaw("ERR: idx ausente"); return; }
  int idx = atoi(pIdx);
  if (idx < 1 || idx > 4) { sendFrameRaw("ERR: Surtidor invalido"); return; }
  uint8_t pos = idx - 1;

  // setNameB
  if (strcmp(pCmd, "setNameB") == 0) {
    if (!pVal || strlen(pVal) == 0) { sendFrameRaw("ERR: nombre ausente para setNameB"); return; }
    size_t len = strlen(pVal);
    if (len > 15) len = 15;

    memset(bombaName[pos], 0, 16);
    strncpy(bombaName[pos], pVal, len);
    bombaName[pos][15] = '\0';
    guardarBombaNameEEPROM(pos);

    LiquidCrystal_I2C *lcd = (pos < 2) ? &lcd12 : &lcd34;
    if (pos == 0 || pos == 2) {
      lcd->setCursor(0, 0);
      char nbuf[17];
      snprintf(nbuf, sizeof(nbuf), "%-16s", bombaName[pos]);
      lcd->print(nbuf);
    }
    lcd->setCursor(0, 1);
    lcd->print("Listo           ");

    char buf[64];
    snprintf(buf, sizeof(buf), "OK: setNameB %d=%s", idx, bombaName[pos]);
    sendFrameRaw(buf);
    return;
  }

  // learn
  if (strcmp(pCmd, "learn") == 0) {
    if (!pVal || strlen(pVal) == 0) { sendFrameRaw("ERR: val ausente para learn"); return; }
    int accion = atoi(pVal);
    char buf[40];
    if (accion == 1) {
      learnMode[pos] = true;
      snprintf(buf, sizeof(buf), "DBG_LEARN: ON surtidor %d", idx);
      LiquidCrystal_I2C *lcd = (pos < 2) ? &lcd12 : &lcd34;
      lcd->setCursor(0, 1);
      lcd->print("LEARN ON        ");
    } else {
      learnMode[pos] = false;
      snprintf(buf, sizeof(buf), "DBG_LEARN: OFF surtidor %d", idx);
      LiquidCrystal_I2C *lcd = (pos < 2) ? &lcd12 : &lcd34;
      lcd->setCursor(0, 1);
      lcd->print("OK              ");
    }
    sendFrameRaw(buf);
    return;
  }

  // btn
  if (strcmp(pCmd, "btn") == 0) {
    if (!pVal || strlen(pVal) == 0) { sendFrameRaw("ERR: val ausente para btn"); return; }
    int accion = atoi(pVal);
    char buf[48];
    if (accion == 1) {
      btnSimStart[pos] = true;
      snprintf(buf, sizeof(buf), "OK: BTN START sim surtidor %d", idx);
    } else if (accion == 0) {
      btnSimStop[pos] = true;
      snprintf(buf, sizeof(buf), "OK: BTN STOP sim surtidor %d", idx);
    } else {
      snprintf(buf, sizeof(buf), "ERR: val btn invalido (use 0/1)");
    }
    sendFrameRaw(buf);
    return;
  }

  // read
  if (strcmp(pCmd, "read") == 0) {
    char buf[112];
    snprintf(buf, sizeof(buf),
             "READ %d: fill=%lu time=%lu bill=%lu fills=%lu cant=%lu",
             idx,
             surtidor_fill[pos],
             surtidor_time[pos],
             surtidor_bill[pos],
             surtidor_fills[pos],
             surtidor_cantML[pos]);
    sendFrameRaw(buf);
    return;
  }

  // clrBill
  if (strcmp(pCmd, "clrBill") == 0) {
    surtidor_bill[pos] = 0;
    guardarBillEEPROM(pos);
    char buf[40];
    snprintf(buf, sizeof(buf), "OK: CLR BILL surtidor %d", idx);
    sendFrameRaw(buf);
    return;
  }

  // clrFills
  if (strcmp(pCmd, "clrFills") == 0) {
    surtidor_fills[pos] = 0;
    guardarFillsEEPROM(pos);
    char buf[40];
    snprintf(buf, sizeof(buf), "OK: CLR FILLS surtidor %d", idx);
    sendFrameRaw(buf);
    return;
  }

  // setCant: programa cantidad en mL
  if (strcmp(pCmd, "setCant") == 0) {
    if (!pVal || strlen(pVal) == 0) { sendFrameRaw("ERR: val ausente para setCant"); return; }
    unsigned long val = strtoul(pVal, NULL, 10);
    surtidor_cantML[pos] = val;
    guardarCantEEPROM(pos);
    char buf[48];
    snprintf(buf, sizeof(buf), "OK: CANT surtidor %d = %lu mL", idx, val);
    sendFrameRaw(buf);
    return;
  }

  // fill / bill / time
  if (!pVal || strlen(pVal) == 0) { sendFrameRaw("ERR: val ausente"); return; }

  unsigned long val = strtoul(pVal, NULL, 10);

  if (strcmp(pCmd, "fill") == 0) {
    surtidor_fill[pos] = val;
    guardarFillEEPROM(pos);
    char buf[48];
    snprintf(buf, sizeof(buf), "OK: FILL surtidor %d = %lu", idx, val);
    sendFrameRaw(buf);
    return;
  } else if (strcmp(pCmd, "bill") == 0) {
    surtidor_bill[pos] = val;
    guardarBillEEPROM(pos);
    char buf[48];
    snprintf(buf, sizeof(buf), "OK: BILL surtidor %d = %lu", idx, val);
    sendFrameRaw(buf);
    return;
  } else if (strcmp(pCmd, "time") == 0) {
    surtidor_time[pos] = val;
    guardarTimeEEPROM(pos);
    char buf[48];
    snprintf(buf, sizeof(buf), "OK: TIME surtidor %d = %lu", idx, val);
    sendFrameRaw(buf);
    return;
  } else {
    sendFrameRaw("ERR: Comando no reconocido");
    return;
  }
}

// ========== FIN DE CICLO ==========
void finCicloBomba(uint8_t idx, unsigned long now) {
  unsigned long pulsosLCD = getPulsosParaLCD(idx);
  BombaLastPercent[idx] = calcPercent(idx, pulsosLCD);

  digitalWrite((idx==0)?Bomba1_PIN:(idx==1)?Bomba2_PIN:(idx==2)?Bomba3_PIN:Bomba4_PIN, LOW);
  BombaActiva[idx]    = false;
  BombaReadyTime[idx] = now + RESTART_DELAY_MS;
  BombaStopTime[idx]  = now;

  surtidor_fills[idx] += 1;
  guardarFillsEEPROM(idx);

  Beep_On();
}

// ========== SETUP ==========
void setup() {
  pinMode(Bomba1_PIN, OUTPUT);
  pinMode(Bomba2_PIN, OUTPUT);
  pinMode(Bomba3_PIN, OUTPUT);
  pinMode(Bomba4_PIN, OUTPUT);
  pinMode(Buzzer_PIN, OUTPUT);

  pinMode(Opto1_PIN, INPUT_PULLUP);
  pinMode(Opto2_PIN, INPUT_PULLUP);
  pinMode(Opto3_PIN, INPUT_PULLUP);
  pinMode(Opto4_PIN, INPUT_PULLUP);

  pinMode(BtnStart1_PIN, INPUT_PULLUP);
  pinMode(BtnStop1_PIN,  INPUT_PULLUP);
  pinMode(BtnStart2_PIN, INPUT_PULLUP);
  pinMode(BtnStop2_PIN,  INPUT_PULLUP);
  pinMode(BtnStart3_PIN, INPUT_PULLUP);
  pinMode(BtnStop3_PIN,  INPUT_PULLUP);
  pinMode(BtnStart4_PIN, INPUT_PULLUP);
  pinMode(BtnStop4_PIN,  INPUT_PULLUP);

  pinMode(BtKey_PIN, OUTPUT);
  digitalWrite(BtKey_PIN, LOW);

  digitalWrite(Bomba1_PIN, LOW);
  digitalWrite(Bomba2_PIN, LOW);
  digitalWrite(Bomba3_PIN, LOW);
  digitalWrite(Bomba4_PIN, LOW);
  digitalWrite(Buzzer_PIN, LOW);

  Serial.begin(SERIAL_BAUD);
  Serial1.begin(9600);

  pinMode(flujoPin1, INPUT);
  pinMode(flujoPin2, INPUT);
  pinMode(flujoPin3, INPUT);
  pinMode(flujoPin4, INPUT);

  Wire.begin();
  cargarDesdeEEPROM();
  lcdInitAll();

  DDRK = 0x00;
  PORTK = 0xFF;
  PCICR |= (1 << PCIE2);
  PCMSK2 |= 0xFF;
  lastPortK = PINK;

  if (mcusr_mirror & _BV(WDRF))      lastResetCause = "WDT";
  else if (mcusr_mirror & _BV(PORF)) lastResetCause = "POR";
  else if (mcusr_mirror & _BV(BORF)) lastResetCause = "BOR";
  else if (mcusr_mirror & _BV(EXTRF)) lastResetCause = "EXT";
  else lastResetCause = "UNKNOWN";

  sendBoot();
  Beep_On();
}

// ========== LOOP ==========
void loop() {
  unsigned long now = millis();

  // ===== Recepción Serial USB =====
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '$') {
      frameStartedUSB = true;
      rxIndexUSB = 0;
    } else if (frameStartedUSB && c == '%') {
      if (rxIndexUSB < BUF_LEN) rxBufUSB[rxIndexUSB] = '\0';
      else rxBufUSB[BUF_LEN - 1] = '\0';
      frameStartedUSB = false;
      procesarTrama(rxBufUSB);
    } else if (frameStartedUSB) {
      if (rxIndexUSB < BUF_LEN - 1) rxBufUSB[rxIndexUSB++] = c;
    }
  }

  // ===== Recepción Serial BT =====
  while (Serial1.available() > 0) {
    char c = Serial1.read();
    if (c == '$') {
      frameStartedBT = true;
      rxIndexBT = 0;
    } else if (frameStartedBT && c == '%') {
      if (rxIndexBT < BUF_LEN) rxBufBT[rxIndexBT] = '\0';
      else rxBufBT[BUF_LEN - 1] = '\0';
      frameStartedBT = false;
      procesarTrama(rxBufBT);
    } else if (frameStartedBT) {
      if (rxIndexBT < BUF_LEN - 1) rxBufBT[rxIndexBT++] = c;
    }
  }

  // ===== LECTURA DE PULSADORES DEBOUNCE (100 ms) =====
  static int debouncedStart[4] = {HIGH, HIGH, HIGH, HIGH};
  static int debouncedStop [4] = {HIGH, HIGH, HIGH, HIGH};
  static unsigned long lastChangeStart[4] = {0,0,0,0};
  static unsigned long lastChangeStop [4] = {0,0,0,0};
  const unsigned long DEBOUNCE_MS = 100;

  int rawS[4];
  int rawT[4];

  rawS[0] = digitalRead(BtnStart1_PIN);
  rawT[0] = digitalRead(BtnStop1_PIN);
  rawS[1] = digitalRead(BtnStart2_PIN);
  rawT[1] = digitalRead(BtnStop2_PIN);
  rawS[2] = digitalRead(BtnStart3_PIN);
  rawT[2] = digitalRead(BtnStop3_PIN);
  rawS[3] = digitalRead(BtnStart4_PIN);
  rawT[3] = digitalRead(BtnStop4_PIN);

  for (uint8_t i = 0; i < 4; i++) {
    if (rawS[i] != debouncedStart[i]) {
      if (now - lastChangeStart[i] >= DEBOUNCE_MS) {
        if (debouncedStart[i] == HIGH && rawS[i] == LOW) Beep_Short();
        debouncedStart[i] = rawS[i];
        lastChangeStart[i] = now;
      }
    }
    if (rawT[i] != debouncedStop[i]) {
      if (now - lastChangeStop[i] >= DEBOUNCE_MS) {
        if (debouncedStop[i] == HIGH && rawT[i] == LOW) Beep_Short();
        debouncedStop[i] = rawT[i];
        lastChangeStop[i] = now;
      }
    }
  }

  bool startPressed[4];
  bool stopPressed[4];
  bool envaseOk[4];

  startPressed[0] = (debouncedStart[0] == LOW) || btnSimStart[0];
  stopPressed [0] = (debouncedStop [0] == LOW) || btnSimStop[0];
  startPressed[1] = (debouncedStart[1] == LOW) || btnSimStart[1];
  stopPressed [1] = (debouncedStop [1] == LOW) || btnSimStop[1];
  startPressed[2] = (debouncedStart[2] == LOW) || btnSimStart[2];
  stopPressed [2] = (debouncedStop [2] == LOW) || btnSimStop[2];
  startPressed[3] = (debouncedStart[3] == LOW) || btnSimStart[3];
  stopPressed [3] = (debouncedStop[3] == LOW) || btnSimStop[3];

  envaseOk[0] = (digitalRead(Opto1_PIN) == 0);
  envaseOk[1] = (digitalRead(Opto2_PIN) == 0);
  envaseOk[2] = (digitalRead(Opto3_PIN) == 0);
  envaseOk[3] = (digitalRead(Opto4_PIN) == 0);

  // ===== ARRANQUE / GESTION 4 BOMBAS =====
  for (uint8_t i = 0; i < 4; i++) {
    // Arranque
    if (startPressed[i] && envaseOk[i] && !BombaActiva[i] && now >= BombaReadyTime[i]) {
      BombaActiva[i] = true;
      BombaStartTime[i] = now;
      flujo_counts[i] = 0;
      digitalWrite((i==0)?Bomba1_PIN:(i==1)?Bomba2_PIN:(i==2)?Bomba3_PIN:Bomba4_PIN, HIGH);
      Beep_On();
      cicloPorBtn[i] = btnSimStart[i];
      BombaStopTime[i]    = 0;
      BombaLastPercent[i] = 0;
    }

    if (BombaActiva[i]) {
      bool sinEnvase = !envaseOk[i];

      if (learnMode[i]) {
        if (stopPressed[i] || sinEnvase) {
          unsigned long elapsed = now - BombaStartTime[i];
          unsigned long pulsos  = flujo_counts[i];

          surtidor_fill[i] = pulsos;
          surtidor_time[i] = elapsed * 2;
          guardarFillEEPROM(i);
          guardarTimeEEPROM(i);

          finCicloBomba(i, now);

          char buf[64];
          snprintf(buf, sizeof(buf), "LEARN,%d,%lu,%lu", i+1, surtidor_fill[i], surtidor_time[i]);
          sendFrameRaw(buf);

          learnMode[i] = false;
        }
      } else {
        if (stopPressed[i]) {
          finCicloBomba(i, now);
        } else {
          unsigned long c = flujo_counts[i];
          if (!cicloPorBtn[i]) {
            bool fin_pulsos = (c >= surtidor_fill[i]);
            bool fin_time   = (surtidor_time[i] > 0) && (now - BombaStartTime[i] >= surtidor_time[i]);
            if (fin_pulsos || fin_time || sinEnvase) finCicloBomba(i, now);
          } else {
            if (sinEnvase) finCicloBomba(i, now);
          }
        }
      }
    }

    btnSimStart[i] = false;
    btnSimStop[i]  = false;
  }

  // ===== REFRESCO LCD PERIÓDICO =====
  static unsigned long lastLCD = 0;
  if (now - lastLCD >= 500) {
    lastLCD = now;

    noInterrupts();
    unsigned long c1 = flujo_counts[0];
    unsigned long c2 = flujo_counts[1]; // sensor 2 -> B1
    unsigned long c3 = flujo_counts[2];
    unsigned long c4 = flujo_counts[3]; // sensor 4 -> B3
    interrupts();

/*     //Codigo para certificar entrada de pulsos flujo
    Serial.print("Bomba1 Flujo:");
    Serial.print(String(c1));
    Serial.write('\n');
    Serial.print("Bomba2 Flujo:");
    Serial.print(String(c2));
    Serial.write('\n');
    Serial.print("Bomba3 Flujo:");
    Serial.print(String(c3));
    Serial.write('\n');
    Serial.print("Bomba4 Flujo:");
    Serial.print(String(c4));
    Serial.write('\n'); */

    uint8_t pB1 = 0;
    uint8_t pB3 = 0;

    if (BombaActiva[1]) pB1 = calcPercent(1, c2);
    if (BombaActiva[3]) pB3 = calcPercent(3, c4);

    lcdUpdateBomba(1, surtidor_bill[1], pB1, learnMode[1], now);
    lcdUpdateBomba(3, surtidor_bill[3], pB3, learnMode[3], now);
  }
}