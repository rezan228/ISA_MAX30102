/**
 * ISA_MAX30102.h
 * Library sensor MAX30102 untuk ESP32
 * 
 * Dibuat oleh  : Rezan (ISA Magang)
 * Berdasarkan  : SparkFun MAX3010x Library oleh Peter Jansen & Nathan Seidle
 * Algoritma    : Maxim Integrated (heartRate + SpO2)
 * 
 * Modul target : GY-MAX30102 (ungu)
 * Wiring       : VIN→3.3V, GND→GND, SDA→GPIO21, SCL→GPIO22
 *                + pull-up eksternal 4.7kΩ di SDA & SCL ke 3.3V
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>

// ── Alamat I2C ────────────────────────────────────────────────
#define ISA_MAX30102_ADDR       0x57
#define ISA_I2C_SPEED_STANDARD  100000
#define ISA_I2C_SPEED_FAST      400000

// ── Register Map (dari datasheet MAX30102) ────────────────────
#define REG_INT_STAT1     0x00
#define REG_INT_STAT2     0x01
#define REG_INT_EN1       0x02
#define REG_INT_EN2       0x03
#define REG_FIFO_WR_PTR   0x04
#define REG_FIFO_OVF      0x05
#define REG_FIFO_RD_PTR   0x06
#define REG_FIFO_DATA     0x07
#define REG_FIFO_CFG      0x08
#define REG_MODE_CFG      0x09
#define REG_SPO2_CFG      0x0A
#define REG_LED1_PA       0x0C   // Red LED
#define REG_LED2_PA       0x0D   // IR  LED
#define REG_MULTI_LED1    0x11
#define REG_MULTI_LED2    0x12
#define REG_TEMP_INT      0x1F
#define REG_TEMP_FRAC     0x20
#define REG_TEMP_CFG      0x21
#define REG_REV_ID        0xFE
#define REG_PART_ID       0xFF   // Selalu 0x15 untuk MAX30102

// ── Ukuran buffer internal ────────────────────────────────────
// HARUS >= 32 karena kapasitas FIFO chip MAX30102 adalah 32 sample.
// Jika lebih kecil, sample baru akan menimpa sample lama yang belum
// dibaca (overflow ring buffer internal) sebelum benar-benar overflow
// di FIFO chip itu sendiri -> data Red/IR jadi tidak berurutan/rusak.
#define ISA_STORAGE_SIZE  32

class ISA_MAX30102 {
public:
    ISA_MAX30102();

    // ── Inisialisasi ─────────────────────────────────────────
    /**
     * begin() — inisialisasi sensor
     * @param wirePort  : objek Wire (default Wire)
     * @param i2cSpeed  : kecepatan I2C (default 400000)
     * @param addr      : alamat I2C (default 0x57)
     * @return true jika sensor ditemukan dan Part ID cocok
     */
    bool begin(TwoWire &wirePort = Wire,
               uint32_t i2cSpeed = ISA_I2C_SPEED_FAST,
               uint8_t  addr     = ISA_MAX30102_ADDR);

    /**
     * setup() — konfigurasi sensor sekaligus
     * @param powerLevel   : arus LED 0x00–0xFF (0x28 ≈ 8mA, default)
     * @param sampleAverage: 1,2,4,8,16,32 (default 4)
     * @param ledMode      : 1=Red only, 2=Red+IR (default 2)
     * @param sampleRate   : 50,100,200,400,800,1000,1600,3200 Hz (default 100)
     * @param pulseWidth   : 69,118,215,411 µs (default 411)
     * @param adcRange     : 2048,4096,8192,16384 (default 4096)
     */
    void setup(byte    powerLevel    = 0x28,
               byte    sampleAverage = 4,
               byte    ledMode       = 2,
               int     sampleRate    = 100,
               int     pulseWidth    = 411,
               int     adcRange      = 4096);

    // ── Baca data langsung (polling) ──────────────────────────
    uint32_t getRed();   // Baca Red terbaru (blocking max 250ms)
    uint32_t getIR();    // Baca IR  terbaru (blocking max 250ms)

    // ── FIFO management ───────────────────────────────────────
    uint16_t check();        // Cek FIFO, isi buffer internal, return jumlah sample baru
    uint8_t  available();    // Berapa sample tersedia di buffer internal
    void     nextSample();   // Maju ke sample berikutnya
    uint32_t getFIFORed();   // Ambil Red dari posisi tail
    uint32_t getFIFOIR();    // Ambil IR  dari posisi tail
    void     clearFIFO();    // Reset pointer FIFO

    // ── Konfigurasi individual ────────────────────────────────
    void softReset();
    void shutDown();
    void wakeUp();

    void setLEDMode(uint8_t mode);
    void setADCRange(uint8_t adcRange);
    void setSampleRate(uint8_t sampleRate);
    void setPulseWidth(uint8_t pulseWidth);
    void setPulseAmplitudeRed(uint8_t val);
    void setPulseAmplitudeIR(uint8_t val);
    void setFIFOAverage(uint8_t samples);
    void enableFIFORollover();
    void disableFIFORollover();

    // ── Suhu chip ─────────────────────────────────────────────
    void  enableDIETEMPRDY();
    float readTemperature();    // Celsius
    float readTemperatureF();   // Fahrenheit

    // ── Interrupt ─────────────────────────────────────────────
    uint8_t getINT1();
    uint8_t getINT2();

    // ── Info chip ─────────────────────────────────────────────
    uint8_t readPartID();
    uint8_t getRevisionID();

    // ── Pointer akses ─────────────────────────────────────────
    uint8_t getWritePointer();
    uint8_t getReadPointer();

    // ── Low-level I2C (bisa dipakai langsung kalau perlu) ─────
    uint8_t readRegister8(uint8_t addr, uint8_t reg);
    void    writeRegister8(uint8_t addr, uint8_t reg, uint8_t val);

private:
    TwoWire *_wire;
    uint8_t  _addr;
    uint8_t  _activeLEDs;   // jumlah channel aktif (1 atau 2)
    uint8_t  _revID;

    void readRevisionID();
    void bitMask(uint8_t reg, uint8_t mask, uint8_t thing);
    bool safeCheck(uint8_t maxTimeMs);

    void enableSlot(uint8_t slot, uint8_t device);
    void disableSlots();

    // Buffer internal circular (STORAGE_SIZE = 32 sample, sesuai kapasitas FIFO chip)
    typedef struct {
        uint32_t red[ISA_STORAGE_SIZE];
        uint32_t ir[ISA_STORAGE_SIZE];
        byte head;
        byte tail;
    } SenseBuffer;

    SenseBuffer _sense;
};
