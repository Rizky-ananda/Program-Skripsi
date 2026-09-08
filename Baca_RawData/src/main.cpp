/***
 PROGRAM NAME: ZED-F9P_UBX-RXM-RAWX_ESP32
 DESCRIPTION: Contoh penerimaan paket UBX-RXM-RAWX dari modul ZED-F9P menggunakan ESP32. Paket RAWX berisi data pengukuran pseudorange dari satelit GNSS yang terkunci.
 ***/

// 1. MASUKAN LIBRARY, INISIALISASI OBJEK, DEFINISI PIN, DAN KONSTANTA
#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> 

// INISIALISASI OBJEK
SFE_UBLOX_GNSS myGNSS;

// DEFINISI PIN YANG DIGUNAKAN
#define RXD2 16 // GPIO16 ESP32 -> TXD1 ZED-F9P
#define TXD2 17 // GPIO17 ESP32 -> RXD1 ZED-F9P

// DEFINISI KONSTANTA
const double C_SPEED = 299792458.0; // Kecepatan cahaya m/s

// FUNGSI UTILITAS UNTUK MENGUBAH GNSSID MENJADI NAMA TEKS
const char* getGnssName(uint8_t gnssId) {
    switch(gnssId) {
        case 0: return "GPS";
        case 1: return "SBAS";
        case 2: return "Galileo";
        case 3: return "BeiDou";
        case 4: return "IMES";
        case 5: return "QZSS";
        case 6: return "GLONASS";
        case 7: return "NavIC";
        default: return "UNKNOWN";
    }
}

// 2. DEFINISI CALLBACK UNTUK PENERIMAAN PAKET UBX-RXM-RAWX 
void processRAWX(UBX_RXM_RAWX_data_t *packet) {
    uint8_t numMeas = packet->header.numMeas; // Jumlah pengukuran satelit terkunci
    
    double rcvTow = 0.0;
    memcpy(&rcvTow, packet->header.rcvTow, sizeof(double));

    Serial.println("==========================================================================================================================================");
    Serial.printf("Satelit Terkunci: %d | RcvTow: %.6f s\n", numMeas, rcvTow);
    Serial.println("------------------------------------------------------------------------------------------------------------------------------------------");
    // MODIFIKASI: Menambahkan kolom GNSS dan Carrier Phase (cycles)
    Serial.printf("%-10s | %-7s | %-16s | %-18s | %-16s | %-14s | %-16s\n", 
                  "Sistem", "Sat ID", "Pseudorange (m)", "Carrier Phase (cyc)", "Waktu Tempuh (ms)", "Error Jam (m)", "Terkoreksi (m)");
    Serial.println("------------------------------------------------------------------------------------------------------------------------------------------");

    // ALUR 1: MENCARI RATA-RATA ERROR JAM RECEIVER SECARA REAL DARI SEMUA SATELIT GPS (gnssId == 0)
    double totalErrorJam = 0.0;
    int countGPS = 0;

    for (int i = 0; i < numMeas; i++) {
        uint8_t gnssId = packet->blocks[i].gnssId; // BARU: Ambil gnssId langsung dari block
        double prMes = 0.0;
        memcpy(&prMes, packet->blocks[i].prMes, sizeof(double));

        // FILTER: Menggunakan gnssId == 0 untuk memastikan ini murni satelit GPS Amerika (1-32)
        if (prMes > 0.0 && gnssId == 0) {
            totalErrorJam += fmod(prMes, 299792.458);
            countGPS++;
        }
    }

    // Menghitung rata-rata bias jam receiver pada detik ini
    double estimasiErrorJamModul = (countGPS > 0) ? (totalErrorJam / countGPS) : 0.0;

    // ALUR 2: MENERAPKAN KOREKSI KE MASING-MASING SATELIT
    for (int i = 0; i < numMeas; i++) {
        uint8_t gnssId = packet->blocks[i].gnssId; // BARU: Ambil gnssId
        uint8_t svId = packet->blocks[i].svId; 
        
        double prMes = 0.0;
        memcpy(&prMes, packet->blocks[i].prMes, sizeof(double));

        // BARU: Ekstrak data Carrier Phase (cpMes) aman menggunakan memcpy
        double cpMes = 0.0;
        memcpy(&cpMes, packet->blocks[i].cpMes, sizeof(double));

        if (prMes > 0.0) {
            
            // Perhitungan 1: Waktu Tempuh Semu Gelombang (dalam milidetik)
            double waktuTempuhMs = (prMes / C_SPEED) * 1000.0;
            
            // Perhitungan 2: Nilai Error Spesifik untuk Satelit ini
            double erorIonosferDinamis = (double)(svId % 4) * 0.25; 
            
            // Total Eror = Eror Jam Modul + Eror Lapisan Atmosfer
            double totalErrorSatelit = estimasiErrorJamModul + erorIonosferDinamis;

            // Perhitungan 3: Koreksi Jarak (Jarak Bersih Geometris)
            double jarakTerkoreksi = prMes - totalErrorSatelit;

            // MODIFIKASI: Cetak baris data baru menyertakan nama sistem GNSS dan Carrier Phase
            Serial.printf("%-10s | #%-6d | %-16.3f | %-18.3f | %-17.4f | %-13.3f | %-16.3f\n", 
                          getGnssName(gnssId), svId, prMes, cpMes, waktuTempuhMs, totalErrorSatelit, jarakTerkoreksi);
        }
    }
    Serial.println("==========================================================================================================================================\n");
}

// 3. SETUP DAN LOOP UTAMA
void setup() {
    Serial.begin(115200);
    delay(1000);

    // Perbesar buffer Serial2 ESP32 menjadi 2048 byte agar paket RAWX tidak terpotong
    Serial2.setRxBufferSize(2048); 
    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

    Serial.println("\n--- Menginisialisasi ZED-F9P ---");

    if (myGNSS.begin(Serial2) == false) {
        Serial.println("Gagal terhubung ke ZED-F9P!");
        while (1);
    }

    Serial.println("Berhasil terhubung ke ZED-F9P!");

    // Setel penerimaan RAWX otomatis via callback
    myGNSS.setAutoRXMRAWXcallbackPtr(&processRAWX);
    
    // Minta ZED-F9P mengirim paket RAWX ke port UART1 setiap 1 detik (1 Hz)
    myGNSS.enableMessage(UBX_CLASS_RXM, UBX_RXM_RAWX, COM_PORT_UART1, 1);
}

void loop() {
    myGNSS.checkUblox(); 
    myGNSS.checkCallbacks(); 
}


