#include "Audio.h" 
#include "SD.h" 
#include "FS.h"

// Pines adaptados para ESP32-S3
#define SD_CS         10  
#define SPI_MOSI      11  
#define SPI_MISO      13  
#define SPI_SCK       12  
#define I2S_DOUT      6   
#define I2S_BCLK      5   
#define I2S_LRC       4   

Audio audio; 

void setup() {
    Serial.begin(115200);

    // Inicializar la SD
    pinMode(SD_CS, OUTPUT);
    digitalWrite(SD_CS, HIGH);
    if (!SD.begin(SD_CS)) {
        Serial.println("Error al inicializar la tarjeta SD");
        return;
    }
    Serial.println("Tarjeta SD lista.");

    // Crear un archivo WAV en la SD
    createWavFile("sonido.wav");

    // Configurar el audio
    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    audio.setVolume(10); // 0...21
    audio.connecttoFS(SD, "sonido.wav");
}

void loop() {
    audio.loop();
}

// Crear un archivo WAV simple en la SD
void createWavFile(const char *filename) {
    File file = SD.open(filename, FILE_WRITE);
    if (!file) {
        Serial.println("Error al crear el archivo WAV");
        return;
    }

    // Escribir cabecera WAV (PCM, 16 bits, 44.1 kHz, mono)
    uint8_t wavHeader[44] = {
        'R','I','F','F', 36,0,0,0, 'W','A','V','E', 'f','m','t',' ',
        16,0,0,0, 1,0, 1,0, 0x44,0xAC,0x00,0x00, 0x88,0x58,0x01,0x00,
        2,0,16,0, 'd','a','t','a', 0,0,0,0
    };
    file.write(wavHeader, 44);

    // Escribir datos de audio (tono simple)
    for (int i = 0; i < 44100; i++) { 
        int16_t sample = (int16_t)(sin(2.0 * PI * 440.0 * i / 44100) * 32767);
        file.write((uint8_t*)&sample, 2);
    }

    file.seek(40); // Ir a la posición del tamaño del archivo
    uint32_t dataSize = 44100 * 2; // 1 segundo de audio mono 16-bit
    file.write((uint8_t*)&dataSize, 4);

    file.close();
    Serial.println("Archivo WAV creado: sonido.wav");
}