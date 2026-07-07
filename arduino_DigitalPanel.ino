#include <SPI.h>
#include <mcp_can.h>


// --- КОНФИГУРАЦИЯ CAN-МОДУЛЯ ---
const int CAN_CS_PIN = 53;  // Пин CS для MCP2515 (на Arduino Mega)
MCP_CAN CAN0(CAN_CS_PIN);  // Инициализация объекта CAN

// Настройка и переменные для OBD2
unsigned long lastCanRequest = 0;
const unsigned long canInterval = 150;  // Будем запрашивать данные из OBD2
int obdRpm = 0;                         // Сюда сохраним обороты от машины
// Строка запроса оборотов (RPM) по стандарту OBD2
byte obdQuery[8] = { 0x02, 0x01, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00 };

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
  // 1. Отправка запроса в OBD2 по таймеру
  if (millis() - lastCanRequest >= canInterval) {
    lastCanRequest = millis();

    // Отправляем 8 байт запроса на ID 0x7E0 (стандартный ID моторного блока)
    // Параметры: ID, тип кадра (0 - стандартный), длина (8 байт), массив данных
    byte sndStat = CAN0.sendMsgBuf(0x7E0, 0, 8, obdQuery);

    if (sndStat == CAN_OK) {
      Serial.println(F("[OBD2] Запрос RPM отправлен!"));
    } else {
      Serial.println(F("[OBD2] Ошибка отправки запроса в шину. Проверьте провода!"));
    }
  }
  // 2. Ожидание и чтение ответа от машины
  //
  if (CAN0.checkReceive() == CAN_MSGAVAIL) {
    long unsigned int rxId;
    unsigned char len = 0;
    unsigned char rxBuf[8];  // Буфер под 8 байт ответа OBD2

    // Вычитываем пакет из MCP2515
    CAN0.readMsgBuf(&rxId, &len, rxBuf);

    // Нам интересен строго ответ от моторного блока (ID 0x7E8)
    if (rxId == 0x7E8) {
      // Проверяем, что это ответ на наш запрос (режим 0x41, PID 0x0C)
      if (rxBuf[1] == 0x41 && rxBuf[2] == 0x0C) {

        // Формула перевода сырых байт OBD2 в реальные обороты двигателя:
        // Обороты = ((БайтA * 256) + БайтB) / 4
        int byteA = rxBuf[3];
        int byteB = rxBuf[4];
        obdRpm = ((byteA * 256) + byteB) / 4;

        Serial.print(F("[CAN] Блок ответил! Обороты мотора: "));
        Serial.print(obdRpm);
        Serial.println(F(" об/мин"));
      }
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