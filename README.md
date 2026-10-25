# ISA_MAX30102

Library Arduino untuk sensor **MAX30102** (modul GY-MAX30102) pada **ESP32**. Membaca **detak jantung (BPM)**, **saturasi oksigen (SpO2)**, dan **suhu chip**, lengkap dengan wrapper siap-pakai `ISA_VitalMonitor` sehingga di file `.ino` kamu cukup memanggil `update()` lalu mengambil nilainya lewat getter.

> ⚠️ **Disclaimer:** library ini dibuat untuk pembelajaran dan prototipe. **Bukan alat medis** dan tidak boleh dipakai untuk diagnosis.

## Fitur

- Driver sensor MAX30102 (I2C, alamat `0x57`) — konfigurasi LED, sample rate, pulse width, ADC range, FIFO, dan suhu die.
- Algoritma resmi **Maxim Integrated** untuk BPM dan SpO2 (`maxim_heart_rate_and_oxygen_saturation`), serta deteksi detak PBA (`checkForBeat`).
- **`ISA_VitalMonitor`** — wrapper tingkat tinggi yang sudah menangani:
  - deteksi jari menempel,
  - stabilisasi sinyal & kalibrasi awal,
  - sliding window (100 sample, geser 25),
  - validasi rentang BPM/SpO2,
  - smoothing **EMA** + **WMA** (Weighted Moving Average),
  - pembacaan suhu berkala dengan offset.
- Dua contoh sketch: pemakaian dasar (manual) dan pemakaian wrapper.

## Perangkat Keras

| Komponen | Keterangan |
|---|---|
| Mikrokontroler | ESP32 (library ini menargetkan `architectures=esp32`) |
| Sensor | MAX30102 — modul GY-MAX30102 |

### Wiring

| MAX30102 | ESP32 |
|---|---|
| VIN | 3.3V |
| GND | GND |
| SDA | GPIO 21 |
| SCL | GPIO 22 |

## Instalasi

**Arduino IDE (manual)**

1. Download repo ini sebagai ZIP (atau `git clone`).
2. Arduino IDE → *Sketch → Include Library → Add .ZIP Library…* dan pilih ZIP-nya,
   atau salin folder `ISA_MAX30102` ke `Documents/Arduino/libraries/`.
3. Restart Arduino IDE. Contoh tersedia di *File → Examples → ISA_MAX30102*.


## Penggunaan Cepat (ISA_VitalMonitor)

```cpp
#include "ISA_VitalMonitor.h"

ISA_VitalMonitor vital;
unsigned long tsLastPrint = 0;

void setup() {
    Serial.begin(115200);

    // begin(sda, scl, ledCurrent, fingerThreshold)
    if (!vital.begin(21, 22, 0x24, 30000)) {
        Serial.println("Sensor tidak ditemukan! Cek wiring.");
        while (1) delay(1000);
    }
}

void loop() {
    vital.update();   // panggil terus-menerus, tidak blocking lama

    if (millis() - tsLastPrint >= 1000) {
        tsLastPrint = millis();

        if (!vital.isFingerDetected()) {
            Serial.println("Tempelkan jari ke sensor...");
            return;
        }

        Serial.print("BPM=");   Serial.print(vital.getSmoothedBPM());
        Serial.print("  SpO2="); Serial.print(vital.getSmoothedSpO2());
        Serial.print("%  Suhu="); Serial.print(vital.getSmoothedSuhu(), 1);
        Serial.println("C");
    }
}
```

Tempelkan jari dengan tekanan ringan dan stabil. Setelah beberapa detik kalibrasi, nilai akan mulai muncul (nilai `0` berarti buffer WMA belum terisi).

## API

### `ISA_VitalMonitor`

| Method | Keterangan |
|---|---|
| `bool begin(int sda=21, int scl=22, uint8_t ledCurrent=0x3C, uint32_t fingerThresh=30000)` | Inisialisasi I2C + sensor. `true` jika sensor ditemukan. |
| `void update()` | Panggil di `loop()`. Mengurus seluruh pipeline pembacaan & smoothing. |
| `int getBPM()` / `int getSpO2()` / `float getSuhu()` | Nilai hasil EMA (layer dalam). |
| `int getSmoothedBPM()` / `int getSmoothedSpO2()` / `float getSmoothedSuhu()` | Nilai hasil **WMA** — **direkomendasikan** untuk ditampilkan. Return `0` jika belum ada data. |
| `bool isFingerDetected()` | Jari menempel pada sensor. |
| `bool isReady()` | `true` setelah kalibrasi awal selesai dan ada hasil. |
| `void setSmoothingAlpha(float bpmAlpha, float spo2Alpha)` | Alpha EMA (default 0.2 / 0.3). |
| `void setBPMRange(int min, int max)` | Rentang BPM valid (default 45–135). |
| `void setSpO2Range(int min, int max)` | Rentang SpO2 valid (default 80–100). |
| `void setTempOffset(float offsetC)` | Offset suhu (default +1.2 °C). |
| `void setTempIntervalMs(unsigned long ms)` | Interval pembacaan suhu (default 8000 ms). |
| `ISA_MAX30102& sensor()` | Akses objek sensor mentah. |

Ukuran window WMA bisa diubah dengan mendefinisikan `ISA_WMA_SIZE` **sebelum** `#include` (default `5`). Makin besar makin halus, tapi respons makin lambat.

```cpp
#define ISA_WMA_SIZE 8
#include "ISA_VitalMonitor.h"
```

### `ISA_MAX30102` (driver level rendah)

```cpp
ISA_MAX30102 sensor;
sensor.begin(Wire, ISA_I2C_SPEED_FAST);   // cek Part ID 0x15
sensor.setup(0x28, 4, 2, 100, 411, 4096); // powerLevel, avg, ledMode, sampleRate, pulseWidth, adcRange
```

| Kelompok | Method |
|---|---|
| Inisialisasi | `begin()`, `setup()` |
| Baca langsung | `getRed()`, `getIR()` |
| FIFO | `check()`, `available()`, `nextSample()`, `getFIFORed()`, `getFIFOIR()`, `clearFIFO()` |
| Konfigurasi | `softReset()`, `shutDown()`, `wakeUp()`, `setLEDMode()`, `setADCRange()`, `setSampleRate()`, `setPulseWidth()`, `setPulseAmplitudeRed()`, `setPulseAmplitudeIR()`, `setFIFOAverage()`, `enableFIFORollover()`, `disableFIFORollover()` |
| Suhu | `enableDIETEMPRDY()`, `readTemperature()` (°C), `readTemperatureF()` (°F) |
| Interrupt & info | `getINT1()`, `getINT2()`, `readPartID()`, `getRevisionID()`, `getWritePointer()`, `getReadPointer()` |
| I2C mentah | `readRegister8()`, `writeRegister8()` |

Parameter `setup()`:

| Parameter | Nilai | Default |
|---|---|---|
| `powerLevel` | `0x00`–`0xFF` (`0x28` ≈ 8 mA) | `0x28` |
| `sampleAverage` | 1, 2, 4, 8, 16, 32 | 4 |
| `ledMode` | 1 = Red saja, 2 = Red + IR | 2 |
| `sampleRate` | 50, 100, 200, 400, 800, 1000, 1600, 3200 Hz | 100 |
| `pulseWidth` | 69, 118, 215, 411 µs | 411 |
| `adcRange` | 2048, 4096, 8192, 16384 | 4096 |

### Algoritma

```cpp
maxim_heart_rate_and_oxygen_saturation(irBuffer, 100, redBuffer,
                                       &spo2, &validSpo2, &heartRate, &validHr);
bool beat = checkForBeat(irSample);   // dari ISA_heartRate.h
```

## Contoh

| Sketch | Isi |
|---|---|
| [`BasicReading`](examples/BasicReading/BasicReading.ino) | Pemakaian manual: baca FIFO, sliding window, algoritma Maxim, smoothing sendiri. Cocok untuk memahami cara kerja di balik layar. |
| [`VitalMonitorBasic`](examples/VitalMonitorBasic/VitalMonitorBasic.ino) | Pemakaian paling sederhana dengan `ISA_VitalMonitor`. |

## Struktur Repo

```
ISA_MAX30102/
├── library.properties
├── README.md
├── src/
│   ├── ISA_MAX30102.h/.cpp        # driver sensor
│   ├── ISA_VitalMonitor.h/.cpp    # wrapper: kalibrasi, EMA, WMA, suhu
│   ├── ISA_spo2_algorithm.h/.cpp  # algoritma BPM + SpO2 Maxim
│   └── ISA_heartRate.h/.cpp       # deteksi detak (PBA)
└── examples/
    ├── BasicReading/
    └── VitalMonitorBasic/
```

## Tips & Troubleshooting

- **"Sensor tidak ditemukan"** — cek wiring SDA/SCL, pastikan sensor dialiri 3.3V. Alamat I2C harus `0x57`, Part ID `0x15`.
- **Nilai tidak stabil / `--`** — tempelkan jari dengan tekanan ringan, jangan bergerak, dan tunggu beberapa detik. Sesuaikan `ledCurrent` bila sinyal IR terlalu lemah/jenuh, atau `fingerThresh` bila jari tidak terdeteksi.
- **Suhu tidak akurat** — yang dibaca adalah suhu *die* chip, bukan suhu tubuh. Atur kalibrasi dengan `setTempOffset()`.

## Kredit

- Berdasarkan [SparkFun MAX3010x Library](https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library) oleh Peter Jansen & Nathan Seidle.
- Algoritma BPM/SpO2 dari **Maxim Integrated** (MAXREFDES117#).
- Dibuat oleh **Rezan (ISA Magang)**.

