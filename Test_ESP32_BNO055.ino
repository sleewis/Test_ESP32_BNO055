// ─── Test_ESP32_BNO055 ────────────────────────────────────────────────────────
// Standalone BNO055 test voor MKS-ESP32FOC V2.0
//
// Architectuur (zelfde als balancing_robot):
//   fastTask  — Core 0, 500 Hz (2 ms)   : IMU uitlezen (÷5 = 100 Hz)
//   slowTask  — Core 1,  10 Hz (100 ms) : Serial telemetry
//
// I²C-bus: Wire (GPIO 19 = SDA, GPIO 18 = SCL)
// BNO055 adres: 0x28
//
// LET OP! Serial0: 115200 baud
//
// Wat het doet:
// - Initialiseert I²C op GPIO 19 (SDA) / 18 (SCL) @ 400 kHz — zelfde bus als de robot
// - Controleert het chip-ID (0xA0) bij opstarten en geeft duidelijke foutmelding als de BNO055 niet gevonden wordt
// - Zet de BNO055 in IMUPLUS-modus (accel + gyro, geen magnetometer) — identiek aan de robot - Print elke 100 ms op 115200 baud:
//
// Heading[°]   Roll[°]   Pitch[°]   Calib(S/G/A/M)
// ─────────────────────────────────────────────────
//   180.44      -1.25       2.31      0/3/3/0
//
// De kalibratiestatus S/G/A/M loopt van 0 (ongekalibreerd) naar 3 (volledig). Voor IMUPLUS is M altijd 0 (geen
// magnetometer). Gyro en Accel moeten 3 worden na een paar seconden bewegen.
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>
#include <Wire.h>

// ─── Pinnen ───────────────────────────────────────────────────────────────────
#define SDA_PIN  19
#define SCL_PIN  18

// ─── Taakinstellingen ─────────────────────────────────────────────────────────
#define FAST_PERIOD_MS   2     // 500 Hz
#define SLOW_PERIOD_MS   100   // 10 Hz
#define IMU_DIVIDER      5     // IMU elke 5e fast-cyclus → 100 Hz

// ─── BNO055 registers ─────────────────────────────────────────────────────────
#define BNO_ADDR         0x28
#define REG_CHIP_ID      0x00
#define REG_SYS_STATUS   0x39
#define REG_SYS_ERROR    0x3A
#define REG_CALIB_STAT   0x35
#define REG_OPR_MODE     0x3D
#define REG_PWR_MODE     0x3E
#define REG_SYS_TRIGGER  0x3F
#define REG_EUL_DATA     0x1A   // 6 bytes: heading, roll, pitch (LSB eerst)

#define MODE_CONFIG   0x00
#define MODE_IMUPLUS  0x08

#define EULER_SCALE  16.0f

// ─── Gedeelde toestand ────────────────────────────────────────────────────────
struct SharedState {
  float heading;
  float roll;
  float pitch;
  uint8_t calib;       // ruwe calibratie-byte van REG_CALIB_STAT
  bool   imuReady;
  bool   i2cError;
};

static volatile SharedState gShared = {};
static SemaphoreHandle_t xMutex;

// ─── I²C hulpfuncties ─────────────────────────────────────────────────────────

static bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(BNO_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

static bool readRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(BNO_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t n = Wire.requestFrom((uint8_t)BNO_ADDR, len);
  if (n != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

static uint8_t readReg(uint8_t reg) {
  uint8_t v = 0xFF;
  readRegs(reg, &v, 1);
  return v;
}

// ─── BNO055 initialisatie ─────────────────────────────────────────────────────

static bool bnoBegin() {
  uint8_t chipId = readReg(REG_CHIP_ID);
  Serial0.printf("  Chip-ID: 0x%02X  (verwacht: 0xA0)  → %s\n",
                chipId, chipId == 0xA0 ? "OK" : "FOUT — check bedrading");
  if (chipId != 0xA0) return false;

  writeReg(REG_OPR_MODE, MODE_CONFIG);  delay(25);
  writeReg(REG_PWR_MODE, 0x00);         delay(10);

  // Extern kristal inschakelen (hogere nauwkeurigheid)
  // Na het inschakelen heeft de BNO055 ~650 ms nodig om het kristal te stabiliseren.
  // Te kort wachten leidt tot foutieve sensor-fusie of een vastgelopen initialisatie.
  writeReg(REG_SYS_TRIGGER, 0x80);
  Serial0.println("  Extern kristal inschakelen — wacht 650 ms...");
  delay(650);

  // Zet in IMUPLUS-modus (accel + gyro, geen magnetometer)
  writeReg(REG_OPR_MODE, MODE_IMUPLUS);
  delay(25);  // BNO055 heeft ~7 ms nodig voor moduswisseling naar fusion

  Serial0.printf("  SYS status: 0x%02X  |  SYS error: 0x%02X\n",
                readReg(REG_SYS_STATUS), readReg(REG_SYS_ERROR));
  return true;
}

// ─── fastTask — Core 0, 500 Hz ────────────────────────────────────────────────

void fastTask(void *) {
  TickType_t lastWake = xTaskGetTickCount();
  uint8_t divCounter = 0;

  for (;;) {
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(FAST_PERIOD_MS));

    if (!gShared.imuReady) continue;

    if (++divCounter >= IMU_DIVIDER) {
      divCounter = 0;

      uint8_t buf[6];
      bool ok = readRegs(REG_EUL_DATA, buf, 6);

      // Calib apart lezen (1 byte)
      uint8_t cal = readReg(REG_CALIB_STAT);

      // Mutex non-blocking (timeout 0) — nooit wachten in de fast loop
      if (xSemaphoreTake(xMutex, 0) == pdTRUE) {
        if (ok) {
          gShared.heading = (int16_t)((buf[1] << 8) | buf[0]) / EULER_SCALE;
          gShared.roll    = (int16_t)((buf[3] << 8) | buf[2]) / EULER_SCALE;
          gShared.pitch   = (int16_t)((buf[5] << 8) | buf[4]) / EULER_SCALE;
          gShared.calib   = cal;
          gShared.i2cError = false;
        } else {
          gShared.i2cError = true;
        }
        xSemaphoreGive(xMutex);
      }
    }
  }
}

// ─── slowTask — Core 1, 10 Hz ─────────────────────────────────────────────────

void slowTask(void *) {
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(SLOW_PERIOD_MS));

    if (!gShared.imuReady) {
      Serial0.println("  Wachten op IMU...");
      continue;
    }

    float heading, roll, pitch;
    uint8_t calib;
    bool err;

    if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      heading = gShared.heading;
      roll    = gShared.roll;
      pitch   = gShared.pitch;
      calib   = gShared.calib;
      err     = gShared.i2cError;
      xSemaphoreGive(xMutex);
    } else {
      Serial0.println("  [mutex timeout]");
      continue;
    }

    if (err) {
      Serial0.println("  I²C leesfout!");
      continue;
    }

    uint8_t cSys  = (calib >> 6) & 0x03;
    uint8_t cGyro = (calib >> 4) & 0x03;
    uint8_t cAccel= (calib >> 2) & 0x03;
    uint8_t cMag  =  calib       & 0x03;

    Serial0.printf("%9.2f    %7.2f    %7.2f      %d/%d/%d/%d\n",
                  heading, roll, pitch, cSys, cGyro, cAccel, cMag);
  }
}

// ─── Setup ────────────────────────────────────────────────────────────────────

void setup() {
  Serial0.begin(115200);
  while (!Serial0) { delay(10); } // wacht op USB verbinding
  Serial0.println("Gestart!");
  Serial0.println("\n=== Test_ESP32_BNO055 ===");
  Serial0.printf("I²C: SDA=GPIO%d  SCL=GPIO%d  adres=0x%02X\n",
                SDA_PIN, SCL_PIN, BNO_ADDR);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  Serial0.println("BNO055 initialiseren...");
  bool ok = bnoBegin();

  if (ok) {
    Serial0.println("  Initialisatie geslaagd.\n");
    Serial0.println("Heading[°]   Roll[°]   Pitch[°]   Calib(S/G/A/M)");
    Serial0.println("─────────────────────────────────────────────────");
  } else {
    Serial0.println("  Initialisatie MISLUKT. Controleer bedrading en adres.");
    // Taken starten wel — slowTask blijft "Wachten op IMU..." printen
  }

  xMutex = xSemaphoreCreateMutex();
  gShared.imuReady = ok;

  xTaskCreatePinnedToCore(fastTask, "fastTask", 4096, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(slowTask, "slowTask", 4096, nullptr, 1, nullptr, 0);
}

// Arduino loop() is niet gebruikt — alles loopt via FreeRTOS taken
void loop() { vTaskDelete(nullptr); }
