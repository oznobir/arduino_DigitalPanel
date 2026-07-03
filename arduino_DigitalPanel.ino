#include <SPI.h>
#include <mcp_can.h>

const int SPI_CS_PIN = 9;
MCP_CAN CAN0(SPI_CS_PIN);

unsigned long lastByteTime = 0;
int byteCounter = 0;

void setup() {
  Serial.begin(115200);   // Монитор порта ПК на 115200
  Serial1.begin(9600);    // ИСТИННАЯ СКОРОСТЬ ГБО - 9600 БОД!
  
  // Оставляем подтяжку, которая спасла наш сигнал
  pinMode(19, INPUT_PULLUP); 

  while(Serial1.available() > 0) Serial1.read();

  Serial.println(F("=================================================="));
  Serial.println(F("===      ПАССИВНЫЙ СНИФФЕР ГБО НА 9600 БОД     ==="));
  Serial.println(F("===    Слушаем линию RXD работающего свистка   ==="));
  Serial.println(F("=================================================="));

  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    CAN0.setMode(MCP_LISTENONLY); 
  }
}

void loop() {
  if (Serial1.available() > 0) {
    // На 9600 бод байты идут медленнее, поэтому таймаут конца пакета увеличиваем до 45 мс
    if (millis() - lastByteTime > 45) { 
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

// #include <SPI.h>
// #include <mcp_can.h>

// const int SPI_CS_PIN = 9;
// MCP_CAN CAN0(SPI_CS_PIN);

// // Массив всех возможных скоростей для блоков Stag
// long speeds[] = {9600, 19200, 38400, 57600, 115200};
// int currentSpeedIndex = 0;
// unsigned long lastSwitchTime = 0;
// bool dataFound = false;

// void setup() {
//   Serial.begin(115200);
  
//   // Включаем встроенную подтяжку, чтобы помочь линии пробить просадку уровня!
//   pinMode(19, INPUT_PULLUP); 
  
//   Serial.println(F("=================================================="));
//   Serial.println(F("===        АВТОСКАНЕР СКОРОСТИ ГБО STAG        ==="));
//   Serial.println(F("=================================================="));
  
//   Serial.print(F("Пробуем скорость: ")); Serial.println(speeds[currentSpeedIndex]);
//   Serial1.begin(speeds[currentSpeedIndex]);

//   if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
//     CAN0.setMode(MCP_LISTENONLY); 
//   }
// }

// void loop() {
//   // Если в течение 3 секунд на текущей скорости идут только нули или тишина — меняем скорость
//   if (millis() - lastSwitchTime > 3000) {
//     if (!dataFound) {
//       currentSpeedIndex++;
//       if (currentSpeedIndex >= 5) currentSpeedIndex = 0; // по кругу
      
//       Serial1.end();
//       Serial.print(F("\n[СКАНЕР] Данных нет. Переключаем на: ")); 
//       Serial.print(speeds[currentSpeedIndex]); 
//       Serial.println(F(" бод..."));
      
//       Serial1.begin(speeds[currentSpeedIndex]);
//       while(Serial1.available() > 0) Serial1.read();
//     }
//     lastSwitchTime = millis();
//     dataFound = false;
//   }

//   if (Serial1.available() > 0) {
//     byte c = Serial1.read();
    
//     // Если поймали ХОТЬ ЧТО-ТО, кроме чистого нуля — фиксируем скорость!
//     if (c != 0x00) {
//       dataFound = true;
//       Serial.print(F("\n!!! НАЙДЕНЫ ДАННЫЕ НА СКОРОСТИ "));
//       Serial.print(speeds[currentSpeedIndex]);
//       Serial.println(F(" !!!"));
//       Serial.print(F("Поймали байт: 0x"));
//       if(c < 16) Serial.print(F("0"));
//       Serial.println(c, HEX);
      
//       // Замираем на этой скорости на 10 секунд, чтобы вы успели рассмотреть лог
//       delay(10000); 
//       while(Serial1.available() > 0) Serial1.read();
//       lastSwitchTime = millis();
//     }
//   }
// }