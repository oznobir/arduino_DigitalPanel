#include <SPI.h>
#include <mcp_can.h>
//#include <SoftwareSerial.h> // Используем программный порт для инверсии



const int SPI_CS_PIN = 53;
MCP_CAN CAN0(SPI_CS_PIN);

// Создаем инвертированный порт: RX на 19 пине, TX на 18 пине
// параметр (true) заставляет библиотеку инвертировать все сигналы!
//SoftwareSerial StagSerial(19, 18, true); 

int byteCounter = 0;
// Массив оригинального запроса параметров для блоков Stag ISA2 ( v11.3 )
// 0x3A (:) -> Старт, 0x05 -> Длина, 0x66 -> Команда запроса данных, 0x6B -> Чексумма
byte stagRequest[] = {0x3A, 0x05, 0x66, 0x6B}; 


void setup() {
  Serial.begin(115200);  // ПК (в Мониторе порта выберите скорость 115200 и "No line ending / Без конца строки")
  Serial1.begin(57600); // ГБО Stag-300 v11.3
  
  while(Serial1.available() > 0) Serial1.read();

  Serial.println(F("=================================================="));
  Serial.println(F("=== ИНТЕРАКТИВНЫЙ АНАЛИЗАТОР ГБО ГОТОВ         ==="));
  Serial.println(F("=== Введите вверху 'S' или цифры для отправки ==="));
  Serial.println(F("=================================================="));

  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    CAN0.setMode(MCP_LISTENONLY); 
  }
}

void loop() {
  // --- ЧАСТЬ 1: ОТПРАВКА С КОМПЬЮТЕРА В ГБО ---
  // Если ввести что-то в Монитор порта на ПК, Ардуино перешлет это в ГБО
  if (Serial.available() > 0) {
    char inputChar = Serial.read();
    if (inputChar == 's') {
      while(Serial1.available() > 0) Serial1.read(); // Чистим буфер
      
      // Отправляем массив {0x3A, 0x05, 0x66, 0x6B}
      Serial1.write(stagRequest, sizeof(stagRequest)); 
      
      Serial.println(F("\n[ПК -> ГБО] Отправлен пакет запроса 45 байт: 0x3A 0x05 0x66 0x6B"));
      
    } else {
    // Очищаем буфер приема перед отправкой, чтобы не перепутать старый мусор с новым ответом
    //while(Serial1.available() > 0) Serial1.read(); 
    
    // Имитируем поведение компьютерного свистка (Сигнал Break)
    //Serial1.end();                 // Временно отключаем UART на пинах 18 и 19
    //pinMode(18, OUTPUT);
    //digitalWrite(18, LOW);         // Жестко прижимаем линию к земле
    //delay(50);                     // Держим 50 мс (Stag проснется от этого падения)
      
    //Serial1.begin(57600);          // Включаем UART обратно
    while(Serial1.available() > 0) Serial1.read(); // Чистим эхо

    Serial1.write(inputChar); // Пуляем байт в ГБО
    
    Serial.print(F("\n[ПК -> ГБО] Отправлен байт: '"));
    Serial.print(inputChar);
    Serial.print(F("' (HEX: 0x"));
    Serial.print((byte)inputChar, HEX);
    Serial.println(F(")"));

    }
    
  }

  // --- ЧАСТЬ 2: СБОР И АНАЛИЗ ОТВЕТА ОТ ГБО ---
  // Слушаем зеленый провод на пине 19
  if (Serial1.available() > 0) {
    Serial.println(F(">>> ПОЙМАН ПОТОК ДАННЫХ ОТ ГБО <<<"));
    Serial.println(F("----------------------------------------"));
    // Пропускаем наше собственное эхо, если оно прилетит мгновенно
    delay(5); 

    byteCounter = 0;
    unsigned long packetStart = millis();

    // Ждем и собираем байты, пока они идут сплошным потоком
    while (millis() - packetStart < 20) { // Таймаут 20мс означает конец пакета
      if (Serial1.available() > 0) {
        byte c = Serial1.read();
        
        Serial.print(F("Байт ["));
        if(byteCounter < 10) Serial.print(F("0"));
        Serial.print(byteCounter);
        Serial.print(F("] -> HEX: 0x"));
        if(c < 16) Serial.print(F("0"));
        Serial.print(c, HEX);
        Serial.print(F(" (DEC: "));
        Serial.print(c);
        Serial.println(F(")"));

        byteCounter++;
        packetStart = millis(); // Сброс таймаута для следующего байта
      }
    }

    Serial.println(F("----------------------------------------"));
    Serial.print(F("ИТОГО в пакете: ")); 
    Serial.print(byteCounter); 
    Serial.println(F(" байт.\n"));
  }
}
