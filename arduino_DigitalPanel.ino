#include <SPI.h>
#include <mcp_can.h>

const int SPI_CS_PIN = 9;
MCP_CAN CAN0(SPI_CS_PIN);

// Массив всех возможных скоростей для блоков Stag
long speeds[] = {9600, 19200, 38400, 57600, 115200};
int currentSpeedIndex = 0;
unsigned long lastSwitchTime = 0;
bool dataFound = false;

void setup() {
  Serial.begin(115200);
  
  // Включаем встроенную подтяжку, чтобы помочь линии пробить просадку уровня!
  pinMode(19, INPUT_PULLUP); 
  
  Serial.println(F("=================================================="));
  Serial.println(F("===        АВТОСКАНЕР СКОРОСТИ ГБО STAG        ==="));
  Serial.println(F("=================================================="));
  
  Serial.print(F("Пробуем скорость: ")); Serial.println(speeds[currentSpeedIndex]);
  Serial1.begin(speeds[currentSpeedIndex]);

  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    CAN0.setMode(MCP_LISTENONLY); 
  }
}

void loop() {
  // Если в течение 3 секунд на текущей скорости идут только нули или тишина — меняем скорость
  if (millis() - lastSwitchTime > 3000) {
    if (!dataFound) {
      currentSpeedIndex++;
      if (currentSpeedIndex >= 5) currentSpeedIndex = 0; // по кругу
      
      Serial1.end();
      Serial.print(F("\n[СКАНЕР] Данных нет. Переключаем на: ")); 
      Serial.print(speeds[currentSpeedIndex]); 
      Serial.println(F(" бод..."));
      
      Serial1.begin(speeds[currentSpeedIndex]);
      while(Serial1.available() > 0) Serial1.read();
    }
    lastSwitchTime = millis();
    dataFound = false;
  }

  if (Serial1.available() > 0) {
    byte c = Serial1.read();
    
    // Если поймали ХОТЬ ЧТО-ТО, кроме чистого нуля — фиксируем скорость!
    if (c != 0x00) {
      dataFound = true;
      Serial.print(F("\n!!! НАЙДЕНЫ ДАННЫЕ НА СКОРОСТИ "));
      Serial.print(speeds[currentSpeedIndex]);
      Serial.println(F(" !!!"));
      Serial.print(F("Поймали байт: 0x"));
      if(c < 16) Serial.print(F("0"));
      Serial.println(c, HEX);
      
      // Замираем на этой скорости на 10 секунд, чтобы вы успели рассмотреть лог
      delay(10000); 
      while(Serial1.available() > 0) Serial1.read();
      lastSwitchTime = millis();
    }
  }
}




// #include <SPI.h>
// #include <mcp_can.h>
// //#include <SoftwareSerial.h> // Используем программный порт для инверсии



// const int SPI_CS_PIN = 53;
// MCP_CAN CAN0(SPI_CS_PIN);

// // Создаем инвертированный порт: RX на 19 пине, TX на 18 пине
// // параметр (true) заставляет библиотеку инвертировать все сигналы!
// //SoftwareSerial StagSerial(19, 18, true); 
// unsigned long lastByteTime = 0;
// int byteCounter = 0;
// // Массив оригинального запроса параметров для блоков Stag ISA2 ( v11.3 )
// // 0x3A (:) -> Старт, 0x05 -> Длина, 0x66 -> Команда запроса данных, 0x6B -> Чексумма
// //byte stagRequest[] = {0x3A, 0x05, 0x66, 0x6B}; 


// void setup() {
//   Serial.begin(115200);  // ПК (в Мониторе порта выберите скорость 115200 и "No line ending / Без конца строки")
//   Serial1.begin(115200); // ГБО Stag-300 v11.3

//   // Принудительно отключаем любые подтяжки, чтобы не вносить помехи в свисток
//   pinMode(19, INPUT);
  
//   while(Serial1.available() > 0) Serial1.read();

//   Serial.println(F("=================================================="));
//   Serial.println(F("===        ЗАПУЩЕН ПАССИВНЫЙ СНИФФЕР ГБО       ==="));
//   Serial.println(F("===    Слушаем линию RXD работающего свистка   ==="));
//   Serial.println(F("=================================================="));

//   if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
//     CAN0.setMode(MCP_LISTENONLY); 
//   }
// }

// void loop() {
//   if (Serial1.available() > 0) {
//     // Если это первый байт после паузы — значит начался новый пакет
//     if (millis() - lastByteTime > 25) { 
//       Serial.println(F("\n----------------------------------------"));
//       Serial.print(F(">>> ПЕРЕХВАЧЕН ПАКЕТ (Длина прошлого: "));
//       Serial.print(byteCounter);
//       Serial.println(F(" байт) <<<"));
//       Serial.println(F("----------------------------------------"));
//       byteCounter = 0;
//     }
    
//     byte c = Serial1.read();
//     lastByteTime = millis();

//     // Выводим байт в Монитор порта
//     Serial.print(F("["));
//     if(byteCounter < 10) Serial.print(F("0"));
//     Serial.print(byteCounter);
//     Serial.print(F("] HEX: 0x"));
//     if(c < 16) Serial.print(F("0"));
//     Serial.print(c, HEX);
//     Serial.print(F(" (DEC: "));
//     Serial.print(c);
//     Serial.println(F(")"));
    
//     byteCounter++;
//   }

// }
