/**
 * VitalMonitorBasic.ino
 * Contoh paling sederhana menggunakan ISA_VitalMonitor
 * dengan WMA yang sudah built-in di library.
 *
 * Tidak perlu menulis logika smoothing apapun di sini.
 * Cukup panggil:
 *   vital.getSmoothedBPM()   → BPM  sudah di-WMA
 *   vital.getSmoothedSpO2()  → SpO2 sudah di-WMA
 *   vital.getSmoothedSuhu()  → Suhu sudah di-WMA
 */

#include "ISA_VitalMonitor.h"

ISA_VitalMonitor vital;

unsigned long tsLastPrint = 0;

void setup() {
    Serial.begin(115200);
    delay(300);

    Serial.println(F("=== ISA_MAX30102 — BPM | SpO2 | Suhu ==="));

    if (!vital.begin(21, 22, 0x24, 30000)) {
        Serial.println(F("[ERROR] Sensor tidak ditemukan! Cek wiring."));
        while (1) delay(1000);
    }

    // Opsional: ubah ukuran window WMA lewat define sebelum #include,
    // atau biarkan default (ISA_WMA_SIZE = 5).

    Serial.println(F("[OK] Sensor siap. Tempelkan jari..."));
}

void loop() {
    vital.update();

    if (millis() - tsLastPrint >= 1000) {
        tsLastPrint = millis();

        if (!vital.isFingerDetected()) {
            Serial.println(F("[INFO] Tempelkan jari ke sensor..."));
            return;
        }

        int   bpm  = vital.getSmoothedBPM();
        int   spo2 = vital.getSmoothedSpO2();
        float suhu = vital.getSmoothedSuhu();

        Serial.print(F("BPM="));
        if (bpm  > 0) Serial.print(bpm);
        else          Serial.print(F("--"));

        Serial.print(F("  SpO2="));
        if (spo2 > 0) { Serial.print(spo2); Serial.print(F("%")); }
        else            Serial.print(F("--"));

        Serial.print(F("  Suhu="));
        if (suhu > 0) Serial.print(suhu, 1);
        else          Serial.print(F("--"));
        Serial.println(F("C"));
    }
}
