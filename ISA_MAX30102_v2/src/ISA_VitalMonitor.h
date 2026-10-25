/**
 * ISA_VitalMonitor.h
 * Wrapper tingkat-tinggi di atas ISA_MAX30102 + ISA_spo2_algorithm.
 *
 * Tujuan: di file .ino, pengguna tidak perlu lagi menulis logika
 * kalibrasi, sliding window, deteksi stabilisasi sinyal, smoothing EMA,
 * Weighted Moving Average (WMA), atau pembacaan suhu berkala —
 * semua sudah dibungkus di sini.
 *
 * Cara pakai paling sederhana:
 *
 *   ISA_VitalMonitor vital;
 *
 *   void setup() {
 *       Serial.begin(115200);
 *       vital.begin(21, 22, 0x24, 30000);
 *   }
 *
 *   void loop() {
 *       vital.update();
 *       if (vital.isFingerDetected()) {
 *           Serial.println(vital.getSmoothedBPM());    // BPM  (WMA)
 *           Serial.println(vital.getSmoothedSpO2());   // SpO2 (WMA)
 *           Serial.println(vital.getSmoothedSuhu());   // Suhu (WMA)
 *       }
 *   }
 *
 * Dibuat oleh  : Rezan (ISA Magang)
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "ISA_MAX30102.h"
#include "ISA_spo2_algorithm.h"

// ── Ukuran window Maxim algorithm ────────────────────────────────────
#define ISA_VITAL_WINDOW       100
#define ISA_VITAL_SLIDE_STEP   25

// ── Ukuran buffer WMA ─────────────────────────────────────────────────
// Ubah angka ini jika ingin window lebih panjang/pendek.
// Semakin besar → lebih smooth, tapi lebih lambat merespons perubahan.
#ifndef ISA_WMA_SIZE
  #define ISA_WMA_SIZE  5
#endif

class ISA_VitalMonitor {
public:
    ISA_VitalMonitor();

    /**
     * begin() — inisialisasi I2C, sensor, dan konfigurasi default.
     * @param sda, scl      : pin I2C (default 21, 22 — ESP32 umum)
     * @param ledCurrent    : arus LED 0x00–0xFF (default 0x3C ~30mA)
     * @param fingerThresh  : ambang IR untuk anggap jari menempel (default 30000)
     * @return true jika sensor berhasil ditemukan & dikonfigurasi
     */
    bool begin(int sda = 21, int scl = 22,
               uint8_t ledCurrent = 0x3C,
               uint32_t fingerThresh = 30000);

    /**
     * update() — panggil terus-menerus di loop().
     * Menangani: deteksi jari, kalibrasi awal, stabilisasi sinyal,
     * sliding window, kalkulasi BPM/SpO2, smoothing EMA, WMA, dan suhu berkala.
     * Tidak blocking lama (tiap panggilan hanya mengambil beberapa sample).
     */
    void update();

    // ── Getter hasil EMA saja (layer dalam) ──────────────────────────
    // Sama seperti sebelumnya — output dari EMA internal library.
    int    getBPM();
    int    getSpO2();
    float  getSuhu();

    // ── Getter hasil WMA (direkomendasikan untuk tampilan akhir) ─────
    // Nilai ini adalah WMA dari beberapa hasil getBPM()/getSpO2()/getSuhu()
    // terakhir → lebih stabil, tidak melompat-lompat.
    // Return 0 jika buffer WMA belum terisi sama sekali.
    int    getSmoothedBPM();
    int    getSmoothedSpO2();
    float  getSmoothedSuhu();

    // ── Status ──────────────────────────────────────────────────────
    bool   isFingerDetected();
    bool   isReady();   // true setelah kalibrasi awal selesai & ada hasil

    // ── Tuning opsional ─────────────────────────────────────────────
    void setSmoothingAlpha(float bpmAlpha, float spo2Alpha); // EMA alpha (default 0.2/0.3)
    void setBPMRange(int minBPM, int maxBPM);                // default 45–135
    void setSpO2Range(int minSpO2, int maxSpO2);             // default 80–100
    void setTempOffset(float offsetC);                       // default +1.2
    void setTempIntervalMs(unsigned long ms);                // default 8000

    ISA_MAX30102& sensor();   // akses objek sensor mentah jika perlu

private:
    ISA_MAX30102 _sensor;

    uint32_t _irBuffer[ISA_VITAL_WINDOW];
    uint32_t _redBuffer[ISA_VITAL_WINDOW];

    uint32_t _fingerThresh;
    uint8_t  _ledCurrent;
    byte     _sda, _scl;

    // State mesin
    enum State { ST_IDLE, ST_WAIT_STABLE, ST_CALIBRATING, ST_RUNNING };
    State _state;

    // Stabilisasi sinyal
    static const uint8_t STAB_SAMPLES = 10;
    uint32_t _stabBuf[STAB_SAMPLES];
    uint8_t  _stabIdx;
    unsigned long _stabStartMs;

    // ── Hasil EMA (layer dalam) ──────────────────────────────────────
    float _smoothBPM;
    float _smoothSpO2;
    float _suhu;
    float _tempOffset;
    bool  _fingerOn;
    bool  _ready;

    // Range validasi
    int _bpmMin, _bpmMax;
    int _spo2Min, _spo2Max;
    float _alphaBPM, _alphaSpO2;

    // Timer suhu
    unsigned long _tsLastTemp;
    unsigned long _tempIntervalMs;

    // ── Buffer WMA (layer luar) ──────────────────────────────────────
    float   _wmaBPM[ISA_WMA_SIZE];
    float   _wmaSpO2[ISA_WMA_SIZE];
    float   _wmaSuhu[ISA_WMA_SIZE];
    bool    _wmaBPMValid[ISA_WMA_SIZE];
    bool    _wmaSpO2Valid[ISA_WMA_SIZE];
    uint8_t _wmaIdx;
    uint8_t _wmaFilled;

    // ── Helper internal ─────────────────────────────────────────────
    void  resetState();
    bool  fillBuffer(uint32_t *red, uint32_t *ir, byte startIdx, byte count, uint16_t timeoutMs);
    void  runAlgorithmAndSmooth();
    void  updateTemperature();
    bool  checkSignalStable();

    // WMA helpers
    void  resetWMABuffer();
    void  pushWMASample();
    float calcWMA(float* buf, bool* valid);
    float calcWMASuhu(float* buf);
};
