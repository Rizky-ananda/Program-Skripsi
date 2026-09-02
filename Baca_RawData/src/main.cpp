/***
 PROGRAM NAME: ZED-F9P_UBX-RXM-RAWX_ESP32
 DESCRIPTION: Contoh penerimaan paket UBX-RXM-RAWX dari modul ZED-F9P menggunakan ESP32. Paket RAWX berisi data pengukuran pseudorange dari satelit GNSS yang terkunci.
 ***/

// 1. MASUKAN LIBRARY, INISIALISASI OBJEK, DEFINISI PIN, DAN KONSTANTA
// MASUKAN LIBRARY
#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

// INISIALISASI OBJEK
SFE_UBLOX_GNSS myGNSS;

// DEFINISI PIN YANG DIGUNAKAN
#define RXD2 16 // GPIO16 ESP32 -> TXD1 ZED-F9P
#define TXD2 17 // GPIO17 ESP32 -> RXD1 ZED-F9P

// DEFINISI KONSTANTA
const double C_SPEED = 299792458.0;

// 2. DEFINISI CALLBACK UNTUK PENERIMAAN PAKET UBX-RXM-RAWX
// Callback otomatis saat paket UBX-RXM-RAWX diterima
void processRAWX(UBX_RXM_RAWX_data_t *packet) {
    uint8_t numMeas = packet->header.numMeas; // Jumlah pengukuran satelit yang terkunci
    
    // Tampilkan data pengukuran pseudorange dari setiap satelit yang terkunci
    Serial.println("======================================================================");
    Serial.printf("Satelit Terkunci: %d | RcvTow: %f s\n", numMeas, packet->header.rcvTow);
    Serial.println("----------------------------------------------------------------------");
    Serial.printf("%-12s | %-25s | %-20s\n", "Satelit ID", "Pseudorange / Jarak (m)", "Waktu Tempuh (ms)");
    Serial.println("----------------------------------------------------------------------");

    // Loop melalui setiap pengukuran satelit dan tampilkan data pseudorange serta waktu tempuh sinyal
    for (int i = 0; i < numMeas; i++) {
        uint8_t svId = packet->blocks[i].svId; // ID satelit GNSS
        
        // Konversi pseudorange dari array byte menjadi double
        double prMes = 0.0;
        memcpy(&prMes, packet->blocks[i].prMes, sizeof(double));

        // Tampilkan data pseudorange dan waktu tempuh sinyal hanya jika pseudorange valid (> 0)
        if (prMes > 0.0) {
            double waktuTempuhMs = (prMes / C_SPEED) * 1000.0;
            Serial.printf("Sat ID #%-5d | %-25.3f | %-20.4f ms\n", svId, prMes, waktuTempuhMs);
        }
    }
    Serial.println("======================================================================\n");
}

// 3. SETUP DAN LOOP UTAMA
void setup() {
    Serial.begin(115200);
    delay(1000);

    // Perbesar buffer Serial2 ESP32 menjadi 2048 byte agar paket RAWX tidak terpotong
    Serial2.setRxBufferSize(2048); 
    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

    Serial.println("\n--- Menginisialisasi ZED-F9P ---");

    // Cek koneksi ke modul ZED-F9P melalui port UART2
    if (myGNSS.begin(Serial2) == false) {
        Serial.println("Gagal terhubung ke ZED-F9P! Periksa wiring/baudrate.");
        while (1);
    }

    Serial.println("Berhasil terhubung ke ZED-F9P!");

    // Setel penerimaan RAWX otomatis via callback
    myGNSS.setAutoRXMRAWXcallbackPtr(&processRAWX);
    
    // Minta ZED-F9P mengirim paket RAWX ke port UART1 setiap 1 detik (1 Hz)
    myGNSS.enableMessage(UBX_CLASS_RXM, UBX_RXM_RAWX, COM_PORT_UART1, 1);
}

void loop() {
    // Wajib dipanggil terus-menerus tanpa delay()
    myGNSS.checkUblox();
    myGNSS.checkCallbacks();
}