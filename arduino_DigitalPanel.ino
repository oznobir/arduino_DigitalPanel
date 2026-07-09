#include <SPI.h>
#include <mcp_can.h>

const int SPI_CS_PIN = 53;
MCP_CAN CAN0(SPI_CS_PIN);

// Буфер и запрос ГБО Stag
byte masterQuery[] = {0xF0, 0x01, 0x01, 0xF2}; 
byte gboBuf[83]; // Жестко резервируем 83 ячейки памяти в C++

// === ТАЙМЕРЫ ДЛЯ НАШЕЙ АСИНХРОННОЙ МНОГОЗАДАЧНОСТИ ===
unsigned long timerUSB  = 0;  // Для отправки в RealDash (30 мс) или в Монитор Порта (300 мс)
unsigned long timerGBO  = 0;  // Для отправки запросов к Stag (200 мс)
unsigned long timerSlow = 0;  // Для вывода медленных параметров (3000 мс)

#pragma pack(push, 1)
struct RealDashPacket {
  // 1. Обязательный заголовок протокола RealDash CAN (8 байт)
  uint8_t  header[4] = {0x44, 0x33, 0x22, 0x11}; 
  uint32_t frameId   = 3200;                    

  // 2. Данные (Каждая переменная по 2 байта!)
  uint16_t rpmEngine;        // Обороты от машины (CAN)
  uint16_t speedVehicleX10;  // Скорость от машины * 10 (например, 65.4 км/ч -> 6540)
  int16_t  waterTemp;        // Температура мотора машины (CAN)
  uint16_t rpmGbo;           // Обороты от ГБО (Stag)
  uint16_t pressGboX100;     // Давление газа * 100 (Stag) (1.29 Бар -> 129)
  uint16_t pressMapX100;     // Давление MAP * 100 (Stag) (0.36 Бар -> 36)
  int16_t  tempRed;          // Температура редуктора (Stag)
  int16_t  tempGas;          // Температура газа (Stag)
  uint16_t injGas1X10;       // 1 цилиндр Время впрыска газа * 10 (3.3 мс -> 33)
  uint16_t injGas2X10;       // 2 цилиндр Время впрыска газа * 10 (3.3 мс -> 33)
  uint16_t injGas3X10;       // 3 цилиндр Время впрыска газа * 10 (3.3 мс -> 33)
  uint16_t injGas4X10;       // 4 цилиндр Время впрыска газа * 10 (3.3 мс -> 33)
  uint16_t injBenz1X10;      // 1 цилиндр Время впрыска бензина * 10 (3.3 мс -> 33)
  uint16_t injBenz2X10;      // 2 цилиндр Время впрыска бензина * 10 (3.3 мс -> 33)
  uint16_t injBenz3X10;      // 3 цилиндр Время впрыска бензина * 10 (3.3 мс -> 33)
  uint16_t injBenz4X10;      // 4 цилиндр Время впрыска бензина * 10 (3.3 мс -> 33)
  uint16_t isGasActive;      // Состояние: 0 - бензин, 1 - газ
};
#pragma pack(pop)

RealDashPacket dataPacket; // Создаем глобальный экземпляр


void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);       // Аппаратный Serial для связи со Stag
  pinMode(19, INPUT_PULLUP); // Подтяжка RX линии газа

  while(Serial1.available() > 0) Serial1.read(); // Очищаем буфер
  
  Serial.println(F("=================================================="));
  Serial.println(F("===    БОРТОВОЙ КОМПЬЮТЕР: ALMERA G15 + STAG    ==="));
  Serial.println(F("=================================================="));

  // Запуск CAN на 500 Кбит/с)
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println(F("[CAN] Модуль MCP2515 успешно запущен в режиме невидимки!"));
    CAN0.setMode(MCP_LISTENONLY); // Только слушаем эфир ЭБУ-ABS, джампер 120 Ом СНЯТ!
  } else {
    Serial.println(F("[CAN] КРИТИЧЕСКАЯ ОШИБКА: MCP2515 не отвечает на пине 53!"));
  }
}

void loop() {
  unsigned long currentTime = millis(); // Опорные "часы" Ардуино

  // =======================================================================
  // ЗАДАЧА 1: ПАССИВНЫЙ ПЕРЕХВАТ CAN-ШИНЫ (ЭБУ двигателя + ABS)
  // =======================================================================
  // Проверяем в каждом цикле loop, чтобы исключить переполнение буфера чипа MCP
  long unsigned int rxId;
  unsigned char len = 0;
  unsigned char rxBuf[8]; // массив на 8 байт

  if(CAN0.checkReceive() == CAN_MSGAVAIL) { 
    CAN0.readMsgBuf(&rxId, &len, rxBuf);    
    
    // Пакет 0x11A: Родные обороты двигателя и температура ОЖ
    if (rxId == 0x11A) {
      dataPacket.rpmEngine = ((rxBuf[0] << 8) | rxBuf[1])/4;
      dataPacket.waterTemp = rxBuf[4] - 40; // Извлекаем температуру мотора из структуры ЭБУ
    }
    
    // Пакет 0x354: Точная скорость автомобиля от блока ABS
    if (rxId == 0x354) {
      dataPacket.speedVehicleX10 = ((rxBuf[0] << 8) | rxBuf[1])/10;
    }
  }
  // =======================================================================
  // ЗАДАЧА 2: Отправка в RealDash (30 мс) или в Монитор Порта (300 мс) 
  // =======================================================================

  if (currentTime - timerUSB >= 300) {
     timerUSB = currentTime;
    // Передаем нашу структуру в функцию "переборки" (парсинга)
    printFastDashboard(dataPacket);
    // МАГИЯ С++: отправляем всю структуру в USB одной строчкой!
    // Serial.write((byte*)&dataPacket, sizeof(dataPacket));
  }
  // =======================================================================
  // ЗАДАЧА 3: ОПРОС И ДИНАМИЧЕСКИЙ ПРИЕМ ГБО STAG (Раз в 200 мс)
  // =======================================================================
  if (currentTime - timerGBO >= 200) {
    timerGBO = currentTime;
    while(Serial1.available() > 0) Serial1.read();   // Сбрасываем старый хвост
    Serial1.write(masterQuery, sizeof(masterQuery)); // Отправляем запрос к Stag
  }

  // Забираем пакет 83 байта на лету, когда он полностью подошел в Serial-буфер
  if (Serial1.available() >= 83) { 
    if (Serial1.read() == 0xF0) { // Нашли стартовый маркер пакета
      byte m1 = Serial1.read();
      byte packetLen = Serial1.read();
      
      if (packetLen == 0x53) { // Это наш целевой пакет параметров Stag
        for (int i = 3; i < 83; i++) {
          gboBuf[i] = Serial1.read(); // Заполняем массив ячейка за ячейкой
        }
        
        // --- ДЕКОДИРОВАНИЕ ПАРАМЕТРОВ ПО НАЙДЕННЫМ ИНДЕКСАМ ---
        dataPacket.injBenz1X10 = gboBuf[10];
        dataPacket.injBenz2X10 = gboBuf[12];
        dataPacket.injBenz3X10 = gboBuf[14];
        dataPacket.injBenz4X10 = gboBuf[16];
        
        dataPacket.injGas1X10 = gboBuf[26];
        dataPacket.injGas2X10 = gboBuf[28];
        dataPacket.injGas3X10 = gboBuf[30];
        dataPacket.injGas4X10 = gboBuf[32];

        dataPacket.rpmGbo   = (gboBuf[42] * 100) + gboBuf[43];
        dataPacket.pressGboX100 = gboBuf[45]; 
        dataPacket.pressMapX100 = gboBuf[47];
        dataPacket.tempRed  = gboBuf[48];
        dataPacket.tempGas  = gboBuf[49];
        if (dataPacket.injGas1X10 > 0.5) {
          dataPacket.isGasActive = 1;
        } else {
          dataPacket.isGasActive = 0;
        }
      }
    }
  }

  // =======================================================================
  // ЗАДАЧА 3: СЕРВИСНЫЕ И МЕДЛЕННЫЕ ДАННЫЕ (Вывод раз в 3 секунды)
  // =======================================================================
  
  if (currentTime - timerSlow >= 3000) { //температура 
    timerSlow = currentTime;
    printSlowDashboard(dataPacket);
  }
}

void printFastDashboard(const RealDashPacket& dataPacket) {
  Serial.print(F("[АЛЬМЕРА CAN] Скорость: ")); Serial.print(dataPacket.speedVehicleX10 * 0.1, 1); Serial.print(F(" км/ч"));
  Serial.print(F(" | Родные_RPM: ")); Serial.print(dataPacket.rpmEngine);
  Serial.print(F(" | Т_ОЖ: ")); Serial.print(dataPacket.waterTemp); Serial.println(F("°C"));
  
  Serial.print(F("[ГБО] ")); if (dataPacket.isGasActive == 1) Serial.print(F("ГАЗ")); else Serial.print(F("БЕНЗИН"));
  Serial.print(F(" | Газ_RPM: ")); Serial.print(dataPacket.rpmGbo);
  Serial.print(F(" | Бенз: ")); 
  Serial.print(dataPacket.injBenz1X10 * 0.1, 1); Serial.print(F("/")); Serial.print(dataPacket.injBenz2X10 * 0.1, 1); Serial.print(F("/"));
  Serial.print(dataPacket.injBenz3X10 * 0.1, 1); Serial.print(F("/")); Serial.print(dataPacket.injBenz4X10 * 0.1, 1); 
  Serial.print(F(" мс | Газ: "));
  Serial.print(dataPacket.injGas1X10 * 0.1, 1);  Serial.print(F("/")); Serial.print(dataPacket.injGas2X10 * 0.1, 1);  Serial.print(F("/"));
  Serial.print(dataPacket.injGas3X10 * 0.1, 1);  Serial.print(F("/")); Serial.print(dataPacket.injGas4X10 * 0.1, 1);  Serial.print(F(" мс"));
  Serial.print(F("мс | П_Газ: ")); Serial.print(dataPacket.pressGboX100 * 0.01, 2); Serial.println(F(" Бар"));
}

void printSlowDashboard(const RealDashPacket& dataPacket) {
  // Мы можем обращаться к байтам структуры как к обычному массиву через указатель!
  byte* rawBytes = (byte*)&dataPacket;
  int totalBytes = sizeof(dataPacket);
  Serial.println(F("========== ВЫВОД СТРУКТУРЫ REALDASH =========="));
  Serial.print(F("Размер структуры в памяти: ")); Serial.print(totalBytes); Serial.println(F(" байт."));
  // 1. Побайтная переборка (Дамп памяти)
  Serial.print(F("Сырые байты массива (HEX): "));
  for(int i = 0; i < totalBytes; i++) {
    if(rawBytes[i] < 16) Serial.print("0"); // Красивое выравнивание HEX
    Serial.print(rawBytes[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  printFastDashboard(dataPacket); 
  Serial.print(F("[Доп.] Т_Ред: ")); Serial.print(dataPacket.tempRed); Serial.print(F("°C | Т_Газ: ")); 
  Serial.print(dataPacket.tempGas); Serial.println(F("°C"));
  Serial.println(F("==============================================\n"));
}

