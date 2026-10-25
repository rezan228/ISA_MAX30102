/**
 * ISA_MAX30102.cpp
 * Implementasi library sensor MAX30102
 *
 * Dibuat oleh  : Rezan (ISA Magang)
 * Berdasarkan  : SparkFun MAX3010x Library (BSD License)
 */

#include "ISA_MAX30102.h"

// ── Konstanta register (internal, tidak perlu di header) ──────
static const uint8_t PART_ID_EXPECTED = 0x15;

// Mode config
static const uint8_t RESET_MASK     = 0xBF;
static const uint8_t RESET_BIT      = 0x40;
static const uint8_t SHUTDOWN_MASK  = 0x7F;
static const uint8_t SHUTDOWN_BIT   = 0x80;
static const uint8_t MODE_MASK      = 0xF8;
static const uint8_t MODE_RED_ONLY  = 0x02;
static const uint8_t MODE_RED_IR    = 0x03;
static const uint8_t MODE_MULTI     = 0x07;

// ADC range
static const uint8_t ADC_MASK       = 0x9F;
static const uint8_t ADC_2048       = 0x00;
static const uint8_t ADC_4096       = 0x20;
static const uint8_t ADC_8192       = 0x40;
static const uint8_t ADC_16384      = 0x60;

// Sample rate
static const uint8_t SR_MASK        = 0xE3;
static const uint8_t SR_50          = 0x00;
static const uint8_t SR_100         = 0x04;
static const uint8_t SR_200         = 0x08;
static const uint8_t SR_400         = 0x0C;
static const uint8_t SR_800         = 0x10;
static const uint8_t SR_1000        = 0x14;
static const uint8_t SR_1600        = 0x18;
static const uint8_t SR_3200        = 0x1C;

// Pulse width
static const uint8_t PW_MASK        = 0xFC;
static const uint8_t PW_69          = 0x00;
static const uint8_t PW_118         = 0x01;
static const uint8_t PW_215         = 0x02;
static const uint8_t PW_411         = 0x03;

// FIFO config
static const uint8_t AVG_MASK       = 0x1F;
static const uint8_t AVG_1          = 0x00;
static const uint8_t AVG_2          = 0x20;
static const uint8_t AVG_4          = 0x40;
static const uint8_t AVG_8          = 0x60;
static const uint8_t AVG_16         = 0x80;
static const uint8_t AVG_32         = 0xA0;
static const uint8_t ROLLOVER_MASK  = 0xEF;
static const uint8_t ROLLOVER_EN    = 0x10;
static const uint8_t ROLLOVER_DIS   = 0x00;

// Multi-LED slot
static const uint8_t SLOT_NONE      = 0x00;
static const uint8_t SLOT_RED       = 0x01;
static const uint8_t SLOT_IR        = 0x02;

// Interrupt
static const uint8_t INT_TEMP_MASK  = (byte)~0x02;
static const uint8_t INT_TEMP_EN    = 0x02;

// I2C buffer — ESP32 default 128, fallback 32
#if defined(I2C_BUFFER_LENGTH)
  #define ISA_I2C_BUF I2C_BUFFER_LENGTH
#else
  #define ISA_I2C_BUF 32
#endif

// ─────────────────────────────────────────────────────────────

ISA_MAX30102::ISA_MAX30102() {
    _activeLEDs = 2;
    _sense.head = 0;
    _sense.tail = 0;
}

// ── begin ─────────────────────────────────────────────────────
bool ISA_MAX30102::begin(TwoWire &wirePort, uint32_t i2cSpeed, uint8_t addr) {
    _wire = &wirePort;
    _addr = addr;

    _wire->begin();
    _wire->setClock(i2cSpeed);

    if (readPartID() != PART_ID_EXPECTED) return false;

    readRevisionID();
    return true;
}

// ── setup ─────────────────────────────────────────────────────
void ISA_MAX30102::setup(byte powerLevel, byte sampleAverage,
                         byte ledMode, int sampleRate,
                         int pulseWidth, int adcRange) {
    softReset();
    delay(100);

    // FIFO average
    if      (sampleAverage == 1)  bitMask(REG_FIFO_CFG, AVG_MASK, AVG_1);
    else if (sampleAverage == 2)  bitMask(REG_FIFO_CFG, AVG_MASK, AVG_2);
    else if (sampleAverage == 4)  bitMask(REG_FIFO_CFG, AVG_MASK, AVG_4);
    else if (sampleAverage == 8)  bitMask(REG_FIFO_CFG, AVG_MASK, AVG_8);
    else if (sampleAverage == 16) bitMask(REG_FIFO_CFG, AVG_MASK, AVG_16);
    else if (sampleAverage == 32) bitMask(REG_FIFO_CFG, AVG_MASK, AVG_32);
    else                          bitMask(REG_FIFO_CFG, AVG_MASK, AVG_4);

    // FIFO rollover aktif
    enableFIFORollover();

    // LED mode
    if      (ledMode == 1) { setLEDMode(MODE_RED_ONLY); _activeLEDs = 1; }
    else if (ledMode == 2) { setLEDMode(MODE_RED_IR);   _activeLEDs = 2; }
    else                   { setLEDMode(MODE_MULTI);    _activeLEDs = 2; }

    // ADC range
    if      (adcRange < 4096)       setADCRange(ADC_2048);
    else if (adcRange < 8192)       setADCRange(ADC_4096);
    else if (adcRange < 16384)      setADCRange(ADC_8192);
    else                            setADCRange(ADC_16384);

    // Sample rate
    if      (sampleRate < 100)      setSampleRate(SR_50);
    else if (sampleRate < 200)      setSampleRate(SR_100);
    else if (sampleRate < 400)      setSampleRate(SR_200);
    else if (sampleRate < 800)      setSampleRate(SR_400);
    else if (sampleRate < 1000)     setSampleRate(SR_800);
    else if (sampleRate < 1600)     setSampleRate(SR_1000);
    else if (sampleRate < 3200)     setSampleRate(SR_1600);
    else if (sampleRate == 3200)    setSampleRate(SR_3200);
    else                            setSampleRate(SR_100);

    // Pulse width
    if      (pulseWidth < 118)      setPulseWidth(PW_69);
    else if (pulseWidth < 215)      setPulseWidth(PW_118);
    else if (pulseWidth < 411)      setPulseWidth(PW_215);
    else                            setPulseWidth(PW_411);

    // LED current
    setPulseAmplitudeRed(powerLevel);
    setPulseAmplitudeIR(powerLevel);

    // Multi-LED slot assignment
    disableSlots();
    enableSlot(1, SLOT_RED);
    if (ledMode >= 2) enableSlot(2, SLOT_IR);

    // Reset FIFO
    clearFIFO();
}

// ── Baca data langsung ────────────────────────────────────────
uint32_t ISA_MAX30102::getRed() {
    if (safeCheck(250)) return _sense.red[_sense.head];
    return 0;
}

uint32_t ISA_MAX30102::getIR() {
    if (safeCheck(250)) return _sense.ir[_sense.head];
    return 0;
}

// ── FIFO management ───────────────────────────────────────────
uint8_t ISA_MAX30102::available() {
    int8_t n = _sense.head - _sense.tail;
    if (n < 0) n += ISA_STORAGE_SIZE;
    return (uint8_t)n;
}

void ISA_MAX30102::nextSample() {
    if (available()) {
        _sense.tail++;
        _sense.tail %= ISA_STORAGE_SIZE;
    }
}

uint32_t ISA_MAX30102::getFIFORed() { return _sense.red[_sense.tail]; }
uint32_t ISA_MAX30102::getFIFOIR()  { return _sense.ir[_sense.tail];  }

void ISA_MAX30102::clearFIFO() {
    writeRegister8(_addr, REG_FIFO_WR_PTR, 0x00);
    writeRegister8(_addr, REG_FIFO_OVF,    0x00);
    writeRegister8(_addr, REG_FIFO_RD_PTR, 0x00);
}

// ── check() — isi buffer internal dari FIFO chip ──────────────
uint16_t ISA_MAX30102::check() {
    byte rdPtr = getReadPointer();
    byte wrPtr = getWritePointer();

    int numberOfSamples = 0;
    if (rdPtr == wrPtr) return 0;

    numberOfSamples = wrPtr - rdPtr;
    if (numberOfSamples < 0) numberOfSamples += 32;

    int bytesLeft = numberOfSamples * _activeLEDs * 3;

    _wire->beginTransmission(_addr);
    _wire->write(REG_FIFO_DATA);
    _wire->endTransmission(false);

    while (bytesLeft > 0) {
        int toGet = bytesLeft;
        if (toGet > ISA_I2C_BUF) {
            toGet = ISA_I2C_BUF - (ISA_I2C_BUF % (_activeLEDs * 3));
        }
        bytesLeft -= toGet;

        _wire->requestFrom((uint8_t)_addr, (uint8_t)toGet);

        while (toGet > 0) {
            _sense.head = (_sense.head + 1) % ISA_STORAGE_SIZE;

            // Baca 3 byte Red
            byte tmp[4];
            uint32_t val;
            tmp[3] = 0;
            tmp[2] = _wire->read();
            tmp[1] = _wire->read();
            tmp[0] = _wire->read();
            memcpy(&val, tmp, sizeof(val));
            _sense.red[_sense.head] = val & 0x3FFFF;   // 18-bit mask

            if (_activeLEDs > 1) {
                // Baca 3 byte IR
                tmp[3] = 0;
                tmp[2] = _wire->read();
                tmp[1] = _wire->read();
                tmp[0] = _wire->read();
                memcpy(&val, tmp, sizeof(val));
                _sense.ir[_sense.head] = val & 0x3FFFF;
            }

            toGet -= _activeLEDs * 3;
        }
    }

    return (uint16_t)numberOfSamples;
}

bool ISA_MAX30102::safeCheck(uint8_t maxTimeMs) {
    uint32_t t = millis();
    while (millis() - t < maxTimeMs) {
        if (check() > 0) return true;
        delay(1);
    }
    return false;
}

// ── Konfigurasi individual ────────────────────────────────────
void ISA_MAX30102::softReset() {
    bitMask(REG_MODE_CFG, RESET_MASK, RESET_BIT);
    uint32_t t = millis();
    while (millis() - t < 1000) {
        if ((readRegister8(_addr, REG_MODE_CFG) & RESET_BIT) == 0) break;
        delay(10);
    }
}

void ISA_MAX30102::shutDown()  { bitMask(REG_MODE_CFG, SHUTDOWN_MASK, SHUTDOWN_BIT); }
void ISA_MAX30102::wakeUp()    { bitMask(REG_MODE_CFG, SHUTDOWN_MASK, 0x00); }

void ISA_MAX30102::setLEDMode(uint8_t mode)     { bitMask(REG_MODE_CFG, MODE_MASK,  mode); }
void ISA_MAX30102::setADCRange(uint8_t r)       { bitMask(REG_SPO2_CFG, ADC_MASK,   r); }
void ISA_MAX30102::setSampleRate(uint8_t sr)    { bitMask(REG_SPO2_CFG, SR_MASK,    sr); }
void ISA_MAX30102::setPulseWidth(uint8_t pw)    { bitMask(REG_SPO2_CFG, PW_MASK,    pw); }

void ISA_MAX30102::setPulseAmplitudeRed(uint8_t v) { writeRegister8(_addr, REG_LED1_PA, v); }
void ISA_MAX30102::setPulseAmplitudeIR(uint8_t v)  { writeRegister8(_addr, REG_LED2_PA, v); }

void ISA_MAX30102::setFIFOAverage(uint8_t s)    { bitMask(REG_FIFO_CFG, AVG_MASK, s); }
void ISA_MAX30102::enableFIFORollover()         { bitMask(REG_FIFO_CFG, ROLLOVER_MASK, ROLLOVER_EN); }
void ISA_MAX30102::disableFIFORollover()        { bitMask(REG_FIFO_CFG, ROLLOVER_MASK, ROLLOVER_DIS); }

// ── Slot assignment ───────────────────────────────────────────
void ISA_MAX30102::enableSlot(uint8_t slot, uint8_t device) {
    switch (slot) {
        case 1: bitMask(REG_MULTI_LED1, 0xF8, device);        break;
        case 2: bitMask(REG_MULTI_LED1, 0x8F, device << 4);   break;
        case 3: bitMask(REG_MULTI_LED2, 0xF8, device);        break;
        case 4: bitMask(REG_MULTI_LED2, 0x8F, device << 4);   break;
    }
}

void ISA_MAX30102::disableSlots() {
    writeRegister8(_addr, REG_MULTI_LED1, 0x00);
    writeRegister8(_addr, REG_MULTI_LED2, 0x00);
}

// ── Suhu ──────────────────────────────────────────────────────
void ISA_MAX30102::enableDIETEMPRDY() {
    bitMask(REG_INT_EN2, INT_TEMP_MASK, INT_TEMP_EN);
    writeRegister8(_addr, REG_TEMP_CFG, 0x01);
}

float ISA_MAX30102::readTemperature() {
    writeRegister8(_addr, REG_TEMP_CFG, 0x01);
    delay(100);
    int8_t  tempInt  = (int8_t)readRegister8(_addr, REG_TEMP_INT);
    uint8_t tempFrac = readRegister8(_addr, REG_TEMP_FRAC);
    return tempInt + (tempFrac * 0.0625f);
}

float ISA_MAX30102::readTemperatureF() {
    return readTemperature() * 1.8f + 32.0f;
}

// ── Info chip ─────────────────────────────────────────────────
uint8_t ISA_MAX30102::readPartID()    { return readRegister8(_addr, REG_PART_ID); }
uint8_t ISA_MAX30102::getRevisionID() { return _revID; }

void ISA_MAX30102::readRevisionID() {
    _revID = readRegister8(_addr, REG_REV_ID);
}

uint8_t ISA_MAX30102::getWritePointer() { return readRegister8(_addr, REG_FIFO_WR_PTR) & 0x1F; }
uint8_t ISA_MAX30102::getReadPointer()  { return readRegister8(_addr, REG_FIFO_RD_PTR) & 0x1F; }

uint8_t ISA_MAX30102::getINT1() { return readRegister8(_addr, REG_INT_STAT1); }
uint8_t ISA_MAX30102::getINT2() { return readRegister8(_addr, REG_INT_STAT2); }

// ── bitMask helper ────────────────────────────────────────────
void ISA_MAX30102::bitMask(uint8_t reg, uint8_t mask, uint8_t thing) {
    uint8_t orig = readRegister8(_addr, reg);
    writeRegister8(_addr, reg, (orig & mask) | thing);
}

// ── Low-level I2C ─────────────────────────────────────────────
uint8_t ISA_MAX30102::readRegister8(uint8_t addr, uint8_t reg) {
    _wire->beginTransmission(addr);
    _wire->write(reg);
    _wire->endTransmission(false);
    _wire->requestFrom((uint8_t)addr, (uint8_t)1);
    if (_wire->available()) return _wire->read();
    return 0xFF;
}

void ISA_MAX30102::writeRegister8(uint8_t addr, uint8_t reg, uint8_t val) {
    _wire->beginTransmission(addr);
    _wire->write(reg);
    _wire->write(val);
    _wire->endTransmission();
}
