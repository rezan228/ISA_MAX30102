/**
 * ISA_VitalMonitor.cpp
 * Implementasi wrapper tingkat-tinggi ISA_VitalMonitor.
 *
 * Perubahan dari versi sebelumnya:
 *   - WMA (Weighted Moving Average) dipindahkan dari .ino ke dalam library.
 *   - update() otomatis memanggil pushWMASample() setiap kali ada hasil baru.
 *   - Getter getSmoothedBPM() / getSmoothedSpO2() / getSmoothedSuhu()
 *     mengembalikan nilai WMA siap pakai.
 *   - Alpha EMA dikecilkan default-nya (0.2/0.3) karena WMA sudah jadi
 *     filter utama, EMA hanya sebagai pra-filter sebelum masuk WMA.
 *   - resetState() kini juga me-reset buffer WMA agar saat jari dilepas
 *     dan ditempelkan lagi histori lama tidak ikut menghitung.
 */

#include "ISA_VitalMonitor.h"

// ── Konstruktor ──────────────────────────────────────────────────────
ISA_VitalMonitor::ISA_VitalMonitor() {
    _fingerThresh   = 30000;
    _ledCurrent     = 0x3C;
    _sda = 21; _scl = 22;

    _state       = ST_IDLE;
    _stabIdx     = 0;
    _stabStartMs = 0;

    _smoothBPM  = 0;
    _smoothSpO2 = 0;
    _suhu       = 0;
    _tempOffset = 1.2f;
    _fingerOn   = false;
    _ready      = false;

    _bpmMin  = 45;  _bpmMax  = 135;
    _spo2Min = 80;  _spo2Max = 100;

    // Alpha kecil: EMA hanya pra-filter, bukan filter utama.
    // Filter utama adalah WMA di layer luar.
    _alphaBPM  = 0.2f;
    _alphaSpO2 = 0.3f;

    _tsLastTemp     = 0;
    _tempIntervalMs = 8000;

    resetWMABuffer();
}

// ── begin ─────────────────────────────────────────────────────────────
bool ISA_VitalMonitor::begin(int sda, int scl, uint8_t ledCurrent, uint32_t fingerThresh) {
    _sda          = sda;
    _scl          = scl;
    _ledCurrent   = ledCurrent;
    _fingerThresh = fingerThresh;

    Wire.begin(_sda, _scl);
    Wire.setClock(400000);
    delay(200);

    if (!_sensor.begin(Wire, 400000)) {
        return false;
    }

    // powerLevel, sampleAverage=4, ledMode=2(Red+IR), sampleRate=100, pulseWidth=411, adcRange=4096
    _sensor.setup(_ledCurrent, 4, 2, 100, 411, 4096);

    resetState();
    return true;
}

// ── resetState ────────────────────────────────────────────────────────
void ISA_VitalMonitor::resetState() {
    _state      = ST_IDLE;
    _stabIdx    = 0;
    _smoothBPM  = 0;
    _smoothSpO2 = 0;
    _fingerOn   = false;
    _ready      = false;
    resetWMABuffer();          // ← buffer WMA ikut direset saat jari lepas
}

// ── Setter tuning opsional ───────────────────────────────────────────
void ISA_VitalMonitor::setSmoothingAlpha(float bpmAlpha, float spo2Alpha) {
    _alphaBPM  = bpmAlpha;
    _alphaSpO2 = spo2Alpha;
}
void ISA_VitalMonitor::setBPMRange(int minBPM, int maxBPM)     { _bpmMin = minBPM;   _bpmMax = maxBPM; }
void ISA_VitalMonitor::setSpO2Range(int minSpO2, int maxSpO2)  { _spo2Min = minSpO2; _spo2Max = maxSpO2; }
void ISA_VitalMonitor::setTempOffset(float offsetC)            { _tempOffset = offsetC; }
void ISA_VitalMonitor::setTempIntervalMs(unsigned long ms)     { _tempIntervalMs = ms; }

// ── Getter EMA (layer dalam) ──────────────────────────────────────────
int   ISA_VitalMonitor::getBPM()           { return (int)_smoothBPM; }
int   ISA_VitalMonitor::getSpO2()          { return (int)_smoothSpO2; }
float ISA_VitalMonitor::getSuhu()          { return _suhu; }
bool  ISA_VitalMonitor::isFingerDetected() { return _fingerOn; }
bool  ISA_VitalMonitor::isReady()          { return _ready; }
ISA_MAX30102& ISA_VitalMonitor::sensor()   { return _sensor; }

// ── Getter WMA (layer luar — direkomendasikan untuk tampilan) ─────────
int ISA_VitalMonitor::getSmoothedBPM() {
    float v = calcWMA(_wmaBPM, _wmaBPMValid);
    return (v > 0) ? (int)(v + 0.5f) : 0;   // round ke int terdekat
}

int ISA_VitalMonitor::getSmoothedSpO2() {
    float v = calcWMA(_wmaSpO2, _wmaSpO2Valid);
    return (v > 0) ? (int)(v + 0.5f) : 0;
}

float ISA_VitalMonitor::getSmoothedSuhu() {
    return calcWMASuhu(_wmaSuhu);
}

// ── WMA: reset buffer ─────────────────────────────────────────────────
void ISA_VitalMonitor::resetWMABuffer() {
    _wmaIdx    = 0;
    _wmaFilled = 0;
    memset(_wmaBPM,       0, sizeof(_wmaBPM));
    memset(_wmaSpO2,      0, sizeof(_wmaSpO2));
    memset(_wmaSuhu,      0, sizeof(_wmaSuhu));
    memset(_wmaBPMValid,  0, sizeof(_wmaBPMValid));
    memset(_wmaSpO2Valid, 0, sizeof(_wmaSpO2Valid));
}

// ── WMA: simpan sample EMA terbaru ke buffer circular ────────────────
void ISA_VitalMonitor::pushWMASample() {
    int   bpm  = (int)_smoothBPM;
    int   spo2 = (int)_smoothSpO2;
    float suhu = _suhu;

    _wmaBPM[_wmaIdx]       = bpm;
    _wmaSpO2[_wmaIdx]      = spo2;
    _wmaSuhu[_wmaIdx]      = suhu;
    _wmaBPMValid[_wmaIdx]  = (bpm  > 0);
    _wmaSpO2Valid[_wmaIdx] = (spo2 > 0);

    _wmaIdx = (_wmaIdx + 1) % ISA_WMA_SIZE;
    if (_wmaFilled < ISA_WMA_SIZE) _wmaFilled++;
}

// ── WMA: hitung weighted average untuk BPM / SpO2 ─────────────────────
// Bobot naik linear: sample paling lama dapat bobot 1,
// sample paling baru dapat bobot _wmaFilled.
float ISA_VitalMonitor::calcWMA(float* buf, bool* valid) {
    float weightedSum = 0;
    float totalWeight = 0;
    for (uint8_t i = 0; i < _wmaFilled; i++) {
        // Urut dari yang paling lama (idx paling jauh dari head) ke paling baru
        uint8_t idx = (_wmaIdx + ISA_WMA_SIZE - _wmaFilled + i) % ISA_WMA_SIZE;
        float weight = (float)(i + 1);
        if (valid[idx]) {
            weightedSum += buf[idx] * weight;
            totalWeight += weight;
        }
    }
    return (totalWeight > 0) ? (weightedSum / totalWeight) : 0.0f;
}

// ── WMA: hitung weighted average untuk Suhu ───────────────────────────
// Suhu tidak punya flag valid terpisah — cukup cek > 0.
float ISA_VitalMonitor::calcWMASuhu(float* buf) {
    float weightedSum = 0;
    float totalWeight = 0;
    for (uint8_t i = 0; i < _wmaFilled; i++) {
        uint8_t idx = (_wmaIdx + ISA_WMA_SIZE - _wmaFilled + i) % ISA_WMA_SIZE;
        float weight = (float)(i + 1);
        if (buf[idx] > 0) {
            weightedSum += buf[idx] * weight;
            totalWeight += weight;
        }
    }
    return (totalWeight > 0) ? (weightedSum / totalWeight) : 0.0f;
}

// ── checkSignalStable ─────────────────────────────────────────────────
bool ISA_VitalMonitor::checkSignalStable() {
    uint32_t ir = _sensor.getIR();

    if (ir <= _fingerThresh) return false;

    _stabBuf[_stabIdx % STAB_SAMPLES] = ir;
    _stabIdx++;

    if (_stabIdx < STAB_SAMPLES) return false;

    uint32_t irMin = _stabBuf[0], irMax = _stabBuf[0];
    for (uint8_t i = 1; i < STAB_SAMPLES; i++) {
        if (_stabBuf[i] < irMin) irMin = _stabBuf[i];
        if (_stabBuf[i] > irMax) irMax = _stabBuf[i];
    }

    uint32_t irMid = (irMin + irMax) / 2;
    if (irMid == 0) return false;

    return (irMax - irMin) < (irMid * 15 / 100);
}

// ── fillBuffer ────────────────────────────────────────────────────────
bool ISA_VitalMonitor::fillBuffer(uint32_t *red, uint32_t *ir, byte startIdx, byte count, uint16_t timeoutMs) {
    for (byte i = 0; i < count; i++) {
        unsigned long t = millis();
        while (!_sensor.available()) {
            _sensor.check();
            if (millis() - t > timeoutMs) return false;
        }
        red[startIdx + i] = _sensor.getFIFORed();
        ir[startIdx + i]  = _sensor.getFIFOIR();
        _sensor.nextSample();
    }
    return true;
}

// ── runAlgorithmAndSmooth ─────────────────────────────────────────────
// Layer 1: EMA — meredam noise antar-kalkulasi Maxim algorithm.
// Setelah EMA diperbarui, pushWMASample() dipanggil untuk mengisi
// buffer WMA yang nantinya dibaca via getSmoothedBPM() dll.
void ISA_VitalMonitor::runAlgorithmAndSmooth() {
    int32_t heartRate;  int8_t validHeartRate;
    int32_t spo2;       int8_t validSPO2;

    maxim_heart_rate_and_oxygen_saturation(
        _irBuffer, ISA_VITAL_WINDOW, _redBuffer,
        &spo2, &validSPO2, &heartRate, &validHeartRate);

    // Layer 1 — EMA
    if (validHeartRate == 1 && heartRate > _bpmMin && heartRate < _bpmMax) {
        _smoothBPM = (_smoothBPM == 0) ? (float)heartRate
                                        : _smoothBPM * (1.0f - _alphaBPM) + heartRate * _alphaBPM;
    }

    if (validSPO2 == 1 && spo2 >= _spo2Min && spo2 <= _spo2Max) {
        _smoothSpO2 = (_smoothSpO2 == 0) ? (float)spo2
                                          : _smoothSpO2 * (1.0f - _alphaSpO2) + spo2 * _alphaSpO2;
    }

    if (_smoothBPM > 0 || _smoothSpO2 > 0) _ready = true;

    // Layer 2 — Tambahkan hasil EMA ke buffer WMA
    pushWMASample();
}

// ── updateTemperature ─────────────────────────────────────────────────
void ISA_VitalMonitor::updateTemperature() {
    if (millis() - _tsLastTemp >= _tempIntervalMs) {
        _tsLastTemp = millis();
        _sensor.enableDIETEMPRDY();
        delay(50);
        _suhu = _sensor.readTemperature() + _tempOffset;
    }
}

// ── update() — mesin status utama, dipanggil tiap loop() ─────────────
void ISA_VitalMonitor::update() {
    uint32_t ir = _sensor.getIR();

    if (ir <= _fingerThresh) {
        if (_fingerOn || _state != ST_IDLE) resetState();
        return;
    }

    _fingerOn = true;

    switch (_state) {

        case ST_IDLE:
            _stabIdx = 0;
            _state = ST_WAIT_STABLE;
            break;

        case ST_WAIT_STABLE:
            if (checkSignalStable()) {
                _state = ST_CALIBRATING;
            }
            break;

        case ST_CALIBRATING: {
            _sensor.clearFIFO();
            delay(50);

            bool ok = fillBuffer(_redBuffer, _irBuffer, 0, ISA_VITAL_WINDOW, 500);
            if (!ok) {
                resetState();
                return;
            }

            runAlgorithmAndSmooth();
            _state = ST_RUNNING;
            break;
        }

        case ST_RUNNING: {
            for (byte i = ISA_VITAL_SLIDE_STEP; i < ISA_VITAL_WINDOW; i++) {
                _redBuffer[i - ISA_VITAL_SLIDE_STEP] = _redBuffer[i];
                _irBuffer[i - ISA_VITAL_SLIDE_STEP]  = _irBuffer[i];
            }

            bool ok = fillBuffer(_redBuffer, _irBuffer,
                                  ISA_VITAL_WINDOW - ISA_VITAL_SLIDE_STEP,
                                  ISA_VITAL_SLIDE_STEP, 500);
            if (!ok) return;

            runAlgorithmAndSmooth();
            break;
        }
    }

    updateTemperature();
}
