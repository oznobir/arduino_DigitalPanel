#include <SPI.h>
#include <mcp_can.h>
//#include <SoftwareSerial.h> // Используем программный порт для инверсии



const int SPI_CS_PIN = 53;
MCP_CAN CAN0(SPI_CS_PIN);

// Создаем инвертированный порт: RX на 19 пине, TX на 18 пине
// параметр (true) заставляет библиотеку инвертировать все сигналы!
//SoftwareSerial StagSerial(19, 18, true); 
unsigned long lastByteTime = 0;
int byteCounter = 0;
// Массив оригинального запроса параметров для блоков Stag ISA2 ( v11.3 )
// 0x3A (:) -> Старт, 0x05 -> Длина, 0x66 -> Команда запроса данных, 0x6B -> Чексумма
//byte stagRequest[] = {0x3A, 0x05, 0x66, 0x6B}; 


void setup() {
  Serial.begin(115200);  // ПК (в Мониторе порта выберите скорость 115200 и "No line ending / Без конца строки")
  Serial1.begin(57600); // ГБО Stag-300 v11.3

  // Принудительно отключаем любые подтяжки, чтобы не вносить помехи в свисток
  pinMode(19, INPUT);
  
  while(Serial1.available() > 0) Serial1.read();

  Serial.println(F("=================================================="));
  Serial.println(F("===        ЗАПУЩЕН ПАССИВНЫЙ СНИФФЕР ГБО       ==="));
  Serial.println(F("===    Слушаем линию RXD работающего свистка   ==="));
  Serial.println(F("=================================================="));

  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    CAN0.setMode(MCP_LISTENONLY); 
  }
}

void loop() {
  if (Serial1.available() > 0) {
    // Если это первый байт после паузы — значит начался новый пакет
    if (millis() - lastByteTime > 25) { 
      Serial.println(F("\n----------------------------------------"));
      Serial.print(F(">>> ПЕРЕХВАЧЕН ПАКЕТ (Длина прошлого: "));
      Serial.print(byteCounter);
      Serial.println(F(" байт) <<<"));
      Serial.println(F("----------------------------------------"));
      byteCounter = 0;
    }
    
    byte c = Serial1.read();
    lastByteTime = millis();

    // Выводим байт в Монитор порта
    Serial.print(F("["));
    if(byteCounter < 10) Serial.print(F("0"));
    Serial.print(byteCounter);
    Serial.print(F("] HEX: 0x"));
    if(c < 16) Serial.print(F("0"));
    Serial.print(c, HEX);
    Serial.print(F(" (DEC: "));
    Serial.print(c);
    Serial.println(F(")"));
    
    byteCounter++;
  }

}
