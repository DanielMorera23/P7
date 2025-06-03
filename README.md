# PRACTICA-7
## Práctica 7.1: Reproducción de Audio con I2S

#### CUÁL ES EL FIN DE ESTA PRÁCTICA ? 
Se busca adquirir conocimientos sobre el funcionamiento del bus de comunicación I2S a través de la reproducción de un archivo de audio. Para ello, se desarrollará una prueba en la que se emitirá sonido mediante un altavoz, utilizando un archivo previamente almacenado en la memoria interna del sistema. Esta práctica permitirá verificar el correcto uso del protocolo I2S y su integración con dispositivos de salida de audio.

### MATERIAL :
Se utilizará una placa ESP32-S3 junto con un altavoz compatible con el protocolo I2S y un amplificador que también soporte comunicación I2S. Esta configuración permite la transmisión de audio digital de alta calidad entre los componentes, aprovechando las capacidades del ESP32-S3 para gestionar periféricos mediante este protocolo.

## Código:
```cpp
#include "AudioGeneratorAAC.h" 
#include "AudioOutputI2S.h" 
#include "AudioFileSourcePROGMEM.h" 
#include "sampleaac.h" 
AudioFileSourcePROGMEM *in; 
AudioGeneratorAAC *aac; 
AudioOutputI2S *out; 
void setup(){ 
Serial.begin(115200); 
in = new AudioFileSourcePROGMEM(sampleaac, sizeof(sampleaac)); 
aac = new AudioGeneratorAAC(); 
out = new AudioOutputI2S(); 
out -> SetGain(0.125); 
out -> SetPinout(40,39,38); 
aac->begin(in, out); 
} 
void loop(){ 
if (aac->isRunning()) { 
aac->loop(); 
} else { 
aac -> stop(); 
Serial.printf("Sound Generator\n"); 
delay(1000); 
} 
}
```

### FUNCIONAMIENTO:

Dentro de la función setup(), se establece la conexión serial a una velocidad de 115200 baudios con fines de diagnóstico, mientras que audioLogger se configura para mostrar los mensajes correspondientes en el monitor serial; luego, se declara un objeto de tipo AudioFileSourcePROGMEM al que se le asigna el archivo de audio sampleaac junto con su tamaño, tras lo cual se crean tanto el decodificador AAC (AudioGeneratorAAC) como la salida digital I2S (AudioOutputI2S), y se inicia la reproducción mediante aac->begin(in, out), conectando así la fuente con el destino; finalmente, en la función loop(), si el proceso de generación de audio continúa, se ejecuta aac->loop(), y una vez concluida la reproducción, se muestra el mensaje "AAC done" por consola seguido de una pausa de un segundo antes de reiniciar el procedimiento.

### CONCLUSIONES 
Este ejemplo pone en evidencia la habilidad del ESP32-S3 para manejar la reproducción de audio en formato AAC directamente desde su memoria interna utilizando una interfaz I2S, lo que representa un punto de partida robusto para desarrollos futuros relacionados con audio en sistemas embebidos.

# Practica 7.2: Radio Web con ESP8266/ESP32

## CÚAL ES EL FIN DE ESTA PRÁCTICA:
Este desarrollo consiste en una radio por internet basada en un ESP8266 o ESP32, la cual se conecta a una red WiFi y permite transmitir audio en tiempo real en formatos MP3 y AAC a través de una interfaz web accesible para el usuario.

## MATERIAL
Se hace uso de un microcontrolador ESP8266 o ESP32, el cual se conecta a una red WiFi y transmite el audio a un altavoz mediante una salida digital I2S, lo que permite la reproducción eficiente de sonido en tiempo real.

## CÓDIGO
```cpp
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
```

#### EN QUE CONSISTE EL CÓDIGO? 
Este programa establece la conexión del ESP8266 o ESP32 a una red WiFi y configura un servidor web que permite gestionar la reproducción de una emisora en línea. Para el manejo del audio en streaming en formatos MP3 y AAC, se incorporan bibliotecas especializadas, utilizando la salida I2S para dirigir el sonido hacia un altavoz.

Durante la ejecución de la función setup(), se lleva a cabo la conexión inalámbrica y se pone en marcha el servidor web. Asimismo, se prepara el sistema de salida de audio mediante la función AudioOutputI2S.

En cuanto al comportamiento dentro de la función loop(), esta se encarga de detectar y procesar nuevas peticiones HTTP que permitan modificar la dirección de la estación de radio o ajustar el nivel de volumen, al mismo tiempo que se asegura la continuidad del procesamiento del audio en tiempo real.

### CONCLUSIONES
Este proyecto ilustra el uso del ESP8266 o ESP32 para la reproducción de audio en streaming desde una fuente online, permitiendo su control a través de una interfaz web. Se trata de una opción eficiente y fácil de implementar para aplicaciones de radio en red de bajo consumo.
