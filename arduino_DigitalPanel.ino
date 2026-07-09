#include <SPI.h>
#include <mcp_can.h>


// --- КОНФИГУРАЦИЯ CAN-МОДУЛЯ ---
const int CAN_CS_PIN = 53;  // Пин CS для MCP2515 (на Arduino Mega)
MCP_CAN CAN0(CAN_CS_PIN);  // Инициализация объекта CAN

// Настройка и переменные для OBD2
unsigned long lastFastQuery = 0;
unsigned long lastSlowQuery = 0;

// Структура RealDash (обязательно 4-байтовое выравнивание)
#pragma pack(push, 1)
struct RealDashPacket {
  unsigned long header = 0x44415348; // Маркер "DASH"
  // Скоростные параметры с CAN машины
  uint16_t rpm = 0;
  uint16_t speed = 0;
  // Медленные параметры с CAN машины
  uint16_t coolantTemp = 0;
  uint16_t voltage = 0;
};
#pragma pack(pop)

RealDashPacket dashData;

byte slowStep = 0; // Очередь для медленных параметров
// 0x03 (длина), 0x01 (режим), 0x0C (RPM), 0x0D (Speed)
byte queryFast[8] = { 0x03, 0x01, 0x0C, 0x0D, 0x00, 0x00, 0x00, 0x00 };
 // 0x03 (длина), 0x01 (режим), 0x05 (Температура ОЖ), 0x42 (вольт)
byte queryCoolant[8] = { 0x03, 0x01, 0x05, 0x42, 0x00, 0x00, 0x00, 0x00 };

void setup() {
  Serial.begin(115200);  // Скорость Монитора порта — 115200 (* 16/12 = 153600)
  while (!Serial);  // Ожидание открытия Монитора порта
  // Инициализация CAN (с использованием F() макроса для экономии памяти)
  Serial.println(F("=== ИНИЦИАЛИЗАЦИЯ CAN-МОДУЛЯ ==="));
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println(F("=== MCP2515 успешно запущен! ==="));
    CAN0.setMode(MCP_NORMAL);  // Режим пассивного прослушивания/работы
  } else {
    Serial.println(F("=== Ошибка инициализации MCP2515! ==="));
  }
}

void loop() {
  // === ОПРОС CAN-ШИНЫ (MCP2515) ===
  unsigned long currentMillis = millis();
  // 1. БЫСТРЫЙ ЗАПРОС (Обороты + Скорость) - каждые 80 мс
  if (currentMillis - lastFastQuery >= 80) {
    lastFastQuery = currentMillis;
    CAN0.sendMsgBuf(0x7E0, 0, 8, queryFast); // 0x7E0 - ID запроса к ЭБУ Continental
  }
  // 2. МЕДЛЕННЫЙ ЗАПРОС (Температура ОЖ) - каждые 2000 мс
  if (currentMillis - lastSlowQuery >= 2000) {
    lastSlowQuery = currentMillis;
    CAN0.sendMsgBuf(0x7E0, 0, 8, queryCoolant);
   }
  // ==== Ожидание и чтение ответа от машины =======
  //
  if (CAN0.checkReceive() == CAN_MSGAVAIL) {
    long unsigned int rxId;
    unsigned char len = 0;
    unsigned char rxBuf[8];  // Буфер под 8 байт ответа OBD2

    // Вычитываем пакет из MCP2515
    CAN0.readMsgBuf(&rxId, &len, rxBuf);
    // Проверяем, что ответил именно ЭБУ двигателя (0x7E8) и это ответ на OBD2 (0x41)
    if (rxId == 0x7E8 && rxBuf[1] == 0x41) {
      
      // Разбор мульти-ответа (Обороты + Скорость)
      if (rxBuf[2] == 0x0C && rxBuf[5] == 0x0D) {
        dashData.rpm = ((rxBuf[3] * 256) + rxBuf[4]) / 4; // Формула RPM
        dashData.speed = rxBuf[6];                        // Скорость напрямую в км/ч
      }
      
      // Разбор ответа по температуре ОЖ
      else if (rxBuf[2] == 0x05 && rxBuf[4] == 0x42) {
        dashData.coolantTemp = rxBuf[3] - 40; // Формула температуры
        dashData.voltage = ((rxBuf[3] * 256) + rxBuf[4]); // Формула OBD2: ((A * 256) + B) / 1000
      }
      
      Serial.print(F("[CAN] Обороты: ")); Serial.print(dashData.rpm); Serial.print(F(" об/мин"));
      Serial.print(F(" | Скорость: ")); Serial.print(dashData.speed); Serial.println(F(" км/ч"));
      Serial.print(F(" | Батарея: ")); Serial.print((dashData.voltage / 1000), 2); Serial.println(F(" в"));
      Serial.print(F(" | Температура: ")); Serial.print(dashData.coolantTemp); Serial.println(F(" С"));
    }

  }
}

// #include <SPI.h>
// #include <mcp_can.h>

// const int SPI_CS_PIN = 53;
// MCP_CAN CAN0(SPI_CS_PIN);

// void setup() {
//   Serial.begin(115200);
  
//   // Внимание: если кварц на MCP2515 равен 16МГц - поменяйте ниже на MCP_16MHZ!
//   if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
//     Serial.println(F("=== СКАНЕР CAN-ШИНЫ ALMERA G15 ЗАПУЩЕН ==="));
//     CAN0.setMode(MCP_LISTENONLY); // Включаем пассивный режим шпиона
//   } else {
//     Serial.println(F("Ошибка инициализации MCP2515!"));
//   }
// }

// void loop() {
//   long unsigned int rxId;
//   unsigned char len = 0;
//   unsigned char rxBuf[8];

//   if(CAN0.checkReceive() == CAN_MSGAVAIL) {
//     CAN0.readMsgBuf(&rxId, &len, rxBuf);
    
//     // Выводим только пассивные пакеты машины (игнорируем длинные ответы OBD 0x7E8)
//     //if (rxId < 0x700) { 
//       Serial.print(F("ID: 0x"));
//       Serial.print(rxId, HEX);
//       Serial.print(F(" | Длина: "));
//       Serial.print(len);
//       Serial.print(F(" | Данные: "));
      
//       for(int i = 0; i<len; i++) {
//         Serial.print(F("0x"));
//         if(rxBuf[i] < 16) Serial.print(F("0"));
//         Serial.print(rxBuf[i], HEX);
//         Serial.print(F(" "));
//       }
//       Serial.println();
//     //}
//   }
// }