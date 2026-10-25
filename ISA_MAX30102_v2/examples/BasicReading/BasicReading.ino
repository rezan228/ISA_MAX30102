/**
 * BasicReading.ino
 * Contoh penggunaan library ISA_MAX30102
 * 
 * Wiring:
 *   MAX30102 VIN → ESP32 3.3V
 *   MAX30102 GND → ESP32 GND
 *   MAX30102 SDA → GPIO 21  (+ pull-up 4.7kΩ ke 3.3V)
 *   MAX30102 SCL → GPIO 22  (+ pull-up 4.7kΩ ke 3.3V)
 */

#include <Wire.h>
#include "ISA_MAX30102.h"
#include "ISA_spo2_algorithm.h"

ISA_MAX30102 sensor;

uint32_t irBuffer[100];
uint32_t redBuffer[100];

int32_t heartRate;  int8_t validHeartRate;
int32_t spo2;       int8_t validSPO2;

float smoothBPM  = 0;
float smoothSpO2 = 0;
float suhuChip   = 0;

unsigned long tsLastPrint = 0;
unsigned long tsLastTemp  = 0;
bool firstRead = true;

bool cekJari() { return sensor.getIR() > 30000; }

void setup() {
    Serial.begin(115200);
    Wire.begin(21, 22);
    Wire.setClock(400000);
    delay(500);

    Serial.println(F("=== ISA_MAX30102 Library ==="));

    if (!sensor.begin(Wire, 400000)) {
        Serial.println(F("[ERROR] Sensor tidak ditemukan!"));
        while (1);
    }

    // powerLevel=0x28(8mA), avg=4, mode=2(Red+IR), rate=100, pw=411, adc=4096
    sensor.setup(0x28, 4, 2, 100, 411, 4096);

    Serial.print(F("[OK] Part ID=0x"));
    Serial.println(sensor.readPartID(), HEX);
    Serial.println(F("Tempelkan jari ke sensor..."));
}

void loop() {
    if (!cekJari()) {
        static unsigned long t = 0;
        if (millis() - t > 2000) {
            Serial.println(F("[INFO] Tempelkan jari..."));
            t = millis();
        }
        smoothBPM = 0; smoothSpO2 = 0;
        firstRead = true;
        delay(200);
        return;
    }

    if (firstRead) {
        Serial.println(F("[...] Kalibrasi awal ~4 detik..."));
        sensor.clearFIFO();
        delay(50);

        for (byte i = 0; i < 100; i++) {
            unsigned long t = millis();
            while (!sensor.available()) {
                sensor.check();
                if (millis() - t > 500) return;
            }
            redBuffer[i] = sensor.getFIFORed();
            irBuffer[i]  = sensor.getFIFOIR();
            sensor.nextSample();
        }

        maxim_heart_rate_and_oxygen_saturation(
            irBuffer, 100, redBuffer,
            &spo2, &validSPO2, &heartRate, &validHeartRate);

        if (validHeartRate == 1 && heartRate > 45 && heartRate < 135
            && heartRate != 125 && heartRate != 141)
            smoothBPM = heartRate;
        if (validSPO2 == 1 && spo2 >= 85 && spo2 <= 100)
            smoothSpO2 = spo2;

        firstRead = false;
    }

    // Sliding window
    for (byte i = 25; i < 100; i++) {
        redBuffer[i-25] = redBuffer[i];
        irBuffer[i-25]  = irBuffer[i];
    }
    for (byte i = 75; i < 100; i++) {
        unsigned long t = millis();
        while (!sensor.available()) {
            sensor.check();
            if (millis() - t > 500) return;
        }
        redBuffer[i] = sensor.getFIFORed();
        irBuffer[i]  = sensor.getFIFOIR();
        sensor.nextSample();
    }

    maxim_heart_rate_and_oxygen_saturation(
        irBuffer, 100, redBuffer,
        &spo2, &validSPO2, &heartRate, &validHeartRate);

    if (validHeartRate == 1 && heartRate > 45 && heartRate < 135
        && heartRate != 125 && heartRate != 141)
        smoothBPM = (smoothBPM == 0) ? heartRate : smoothBPM * 0.6f + heartRate * 0.4f;

    if (validSPO2 == 1 && spo2 >= 85 && spo2 <= 100)
        smoothSpO2 = (smoothSpO2 == 0) ? spo2 : smoothSpO2 * 0.7f + spo2 * 0.3f;

    // Suhu tiap 8 detik
    if (millis() - tsLastTemp >= 8000) {
        tsLastTemp = millis();
        sensor.enableDIETEMPRDY();
        delay(50);
        suhuChip = sensor.readTemperature();
    }

    // Print tiap 1 detik
    if (millis() - tsLastPrint >= 1000) {
        tsLastPrint = millis();

        Serial.print(F("[RAW]  IR="));  Serial.print(irBuffer[99]);
        Serial.print(F("  Red="));      Serial.println(redBuffer[99]);

        Serial.print(F("[DATA] BPM="));
        if (smoothBPM > 0) Serial.print((int)smoothBPM);
        else               Serial.print(F("-- (tunggu)"));

        Serial.print(F("  SpO2="));
        if (smoothSpO2 > 0) { Serial.print((int)smoothSpO2); Serial.print(F("%")); }
        else                 Serial.print(F("-- (tunggu)"));

        Serial.print(F("  Suhu="));
        Serial.print(suhuChip + 1.2f, 1);
        Serial.println(F("C"));
    }
}
