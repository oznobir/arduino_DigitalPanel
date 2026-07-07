#include <SPI.h>
#include <mcp_can.h>

const int SPI_CS_PIN = 53;
MCP_CAN CAN0(SPI_CS_PIN);

// Переменные для связи со Stag
byte masterQuery[] = {0xF0, 0x01, 0x01, 0xF2}; 
byte gboBuf[83]; 
unsigned long lastGboQuery = 0;

// === ИТОГОВЫЕ ПЕРЕМЕННЫЕ ДЛЯ ПРИБОРКИ ===
// Данные ГБО Stag
float injBenz1 = 0.0;
float injBenz2 = 0.0;
float injBenz3 = 0.0;
float injBenz4 = 0.0;
float injGas1 = 0.0;
float injGas2 = 0.0;  
float injGas3 = 0.0;  
float injGas4 = 0.0;  
int gboRpm = 0;         // Обороты
float pressGas = 0.0;   // Давление газа
float pressMap = 0.0;   // Давление MAP
int tempRed = 0;        // Температура редуктора
int tempGas = 0;        // Температура газа
bool isGasActive = false;   // На каком топливе едем
// int gasLiters = 0; 

// Данные из CAN-шины автомобиля (Пассивный перехват за приборкой)
int carRpm = 0;       // Обороты из ЭБУ двигателя
int carWaterTemp = 0; // Температура антифриза машины
float carSpeed = 0.0; // Точная скорость от блока ABS


void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);       // Аппаратный Serial для Stag
  pinMode(19, INPUT_PULLUP); // Подтяжка RX линии газа

  while(Serial1.available() > 0) Serial1.read();
  
  Serial.println(F("=================================================="));
  Serial.println(F("===    БОРТОВОЙ КОМПЬЮТЕР: ALMERA G15 + STAG    ==="));
  Serial.println(F("=================================================="));

  // Запуск CAN на 500 Кбит/с (кварц 8 МГц)
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println(F("[CAN] Модуль MCP2515 успешно запущен за приборкой!"));
    CAN0.setMode(MCP_LISTENONLY); // Только слушаем эфир, режим невидимки
  } else {
    Serial.println(F("[CAN] КРИТИЧЕСКАЯ ОШИБКА: MCP2515 не отвечает!"));
  }
}

void loop() {
  unsigned long now = millis();

  // 1. ОПРОС БЛОКА ГБО STAG (Строго раз в 300 мс)
  if (now - lastGboQuery >= 300) {
    lastGboQuery = now;
    while(Serial1.available() > 0) Serial1.read(); // Чистим буфер перед запросом
    Serial1.write(masterQuery, sizeof(masterQuery)); 
  }
  // РАЗБОР ПАКЕТА ГБО (Динамический сбор без delay!)
  if (Serial1.available() > 0) {
    if (Serial1.peek() == 0xF0) {
      
      // Вместо delay(100) плавно ждем, пока в буфер зайдут заголовочные байты
      if (Serial1.available() >= 3) {
        byte m0 = Serial1.read(); 
        byte m1 = Serial1.read(); 
        byte packetLen = Serial1.read(); 
        
        if (packetLen == 0x53) { // Наш целевой пакет параметров (83 байта)
          
          int bytesRead = 3; // 3 байта мы уже считали (m0, m1, packetLen)
          unsigned long startWait = millis();
          
          // Крутим динамический цикл, пока не соберём все 83 байта 
          // (или пока не выйдет таймаут 120мс на случай обрыва связи)
          while (bytesRead < 83 && (millis() - startWait < 120)) {
            
            // 1. Параллельно выхватываем пакеты из CAN-шины, чтобы буфер MCP2515 не переполнялся!
            long unsigned int rxId;
            unsigned char len = 0;
            unsigned char rxBuf[8];
            if(CAN0.checkReceive() == CAN_MSGAVAIL) { 
              CAN0.readMsgBuf(&rxId, &len, rxBuf);    
              if (rxId == 0x11A) {
                int rawRpm = (rxBuf[1] << 8) | rxBuf[0];
                carRpm = rawRpm / 4;
                carWaterTemp = rxBuf[4] - 40;
              }
              if (rxId == 0x354) {
                int rawSpeed = (rxBuf[5] << 8) | rxBuf[4];
                carSpeed = rawSpeed / 100.0;
              }
            }
            
            // 2. Если в Serial1 подоспел следующий байт от Stag — забираем его в буфер
            if (Serial1.available() > 0) {
              gboBuf[bytesRead] = Serial1.read();
              bytesRead++;
            }
          }
          
          // Если успешно собрали все 83 байта — раскладываем переменные
          if (bytesRead == 83) {
            // --- ДЕКОДИРОВАНИЕ ПАРАМЕТРОВ ПО НАЙДЕННЫМ ИНДЕКСАМ ---
            injBenz1 = gboBuf[10] / 10.0;
            injBenz2 = gboBuf[12] / 10.0;
            injBenz3 = gboBuf[14] / 10.0;
            injBenz4 = gboBuf[16] / 10.0;
            
            injGas1 = gboBuf[26] / 10.0;
            injGas2 = gboBuf[28] / 10.0;
            injGas3 = gboBuf[30] / 10.0;
            injGas4 = gboBuf[32] / 10.0;
            gboRpm = (gboBuf[42] * 100) + gboBuf[43];
            pressGas = gboBuf[45] * 0.01; 
            pressMap = gboBuf[47] * 0.01;
            tempRed = gboBuf[48];
            tempGas = gboBuf[49];
            isGasActive = (injGas1 > 0.5 && injGas2 > 0.5 && injGas3 > 0.5 && injGas4 > 0.5);
            printDashboardData(); // Сводный вывод
          }
          
        } else {
          // Сбрасываем левые пакеты карт
          for (int i = 3; i < packetLen; i++) { 
            unsigned long tw = millis();
            while(Serial1.available() == 0 && millis() - tw < 5); // Ждем байт
            if (Serial1.available() > 0) Serial1.read(); 
          }
        }
      }
    } else {
      Serial1.read(); // Пропускаем мусор до стартового 0xF0
    }
  }

}

void printDashboardData() {
  Serial.print(F("[АЛЬМЕРА CAN] Скорость: ")); Serial.print(carSpeed, 1); Serial.print(F(" км/ч"));
  Serial.print(F(" | Т_Мотора: ")); Serial.print(carWaterTemp); Serial.print(F("°C"));
  Serial.print(F(" | Родные_RPM: ")); Serial.print(carRpm);
  
  Serial.print(F("[ГБО STAG] Топливо")); if (isGasActive) Serial.print(F("ГАЗ")); else Serial.print(F("БЕНЗИН"));
  Serial.print(F(" | Газ_RPM: ")); Serial.print(gboRpm);
  Serial.print(F(" | Впр_Бенз: ")); Serial.print(injBenz1, 1);
  Serial.print(F(" | ")); Serial.print(injBenz2, 1);
  Serial.print(F(" | ")); Serial.print(injBenz3, 1);
  Serial.print(F(" | ")); Serial.print(injBenz4, 1); Serial.print(F(" мс"));
  Serial.print(F(" | Впр_Газ: ")); Serial.print(injGas1, 1); 
  Serial.print(F(" | ")); Serial.print(injGas2, 1); 
  Serial.print(F(" | ")); Serial.print(injGas3, 1);
  Serial.print(F(" | ")); Serial.print(injGas4, 1); Serial.print(F(" мс"));
  Serial.print(F(" | Давл_Газ: ")); Serial.print(pressGas, 2);
  Serial.print(F(" | Давл_MAP: ")); Serial.print(pressMap, 2);
  Serial.print(F(" | Т_Ред: ")); Serial.print(tempRed); Serial.print(F("°C"));
  Serial.print(F(" | Т_Газ: ")); Serial.print(tempGas); Serial.print(F("°C"));
  Serial.println(F(")"));
}
