#include <SPI.h>
#include <mcp_can.h>
//#include <OneWire.h>
//#include <DallasTemperature.h>

//#define ONE_WIRE_BUS 14 // Датчики сидят на цифровом пине 14
//OneWire oneWire(ONE_WIRE_BUS);
//DallasTemperature sensors(&oneWire);

// Настройка пинов 
const int CAN_CS_PIN = 53;     // Пин CS для MCP2515 (на Arduino Mega)
MCP_CAN CAN0(CAN_CS_PIN);       // Инициализация объекта CAN

// Настройка интервала опроса ГБО
unsigned long lastGboRequest = 0;
const unsigned long gboInterval = 1000; // Опрашиваем ГБО раз в 1 секунду

void setup() {
  Serial.begin(115200); // Скорость Монитора порта — 115200
  while(!Serial); // Ожидание открытия Монитора порта

  Serial.println("=== ТЕСТ CAN-МОДУЛЯ MCP2515 В РЕЖИМЕ LOOPBACK ===");

  // Инициализируем Serial1 для связи с ГБО на штатной скорости 57600
  Serial1.begin(57600); 
  Serial.println("=== МОДУЛЬ ГБО ИНИЦИАЛИЗИРОВАН ===");

  // Инициализация MCP2515. На Almera G15 скорость шины обычно 500 Кбит/с.
  // Кварц на плате MCP2515 стоит на 8 МГц.
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("MCP2515 успешно запущен!");
  } else {
    Serial.println("Ошибка инициализации MCP2515...");
    while(1); // Останавливаем выполнение при ошибке
  }
  /*
  CAN0.setMode(MCP_NORMAL);   // Переводим CAN в рабочий режим
  pinMode(CAN_INT_PIN, INPUT); 

  Serial.println("=== СИСТЕМА ЗАПУЩЕНА И НАДЕЖНО ЗАПИТАНА ПО VIN ===");
  */
  // Переводим модуль в режим LOOPBACK (Сам отправил — сам принял)
  if (CAN0.setMode(MCP_LOOPBACK) == CAN_OK) {
    Serial.println(">>> Режим Loopback успешно включен. Начинаем тест...");
  } else {
    Serial.println("!!! Не удалось переключить модуль в режим Loopback.");
  }

}
unsigned long lastTxTime = 0; 
byte testData[8] = {0xAA, 0xBB, 0xCC, 0x11, 0x22, 0x33, 0x44, 0x55}; // Тестовые байты

void loop() {
  
  // 1. ОТПРАВКА КАН-ПАКЕТА (Раз в 2 секунды)
  if (millis() - lastTxTime >= 2000) {
    lastTxTime = millis();
    
    // Отправляем пакет: ID = 0x100, Стандартный фрейм (0), Длина данных = 8 байт, Массив данных
    byte sndStat = CAN0.sendMsgBuf(0x100, 0, 8, testData);
    
    if (sndStat == CAN_OK) {
      Serial.println("--------------------------------");
      Serial.print("Отправлен CAN-пакет с ID: 0x100 | Данные: ");
      for(int i = 0; i<8; i++) {
        Serial.print("0x");
        Serial.print(testData[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    } else {
      Serial.println("!!! Ошибка отправки пакета.");
    }
    
    // Немного изменяем первый байт для наглядности при следующей отправке
    testData[0]++; 
  }

  // 2. ПРИЕМ КАН-ПАКЕТА (Проверяем буфер каждую миллисекунду)
  if (CAN0.checkReceive() == CAN_MSGAVAIL) {
    long unsigned int rxId;
    unsigned char len = 0;
    unsigned char rxBuf[8];
    
    // Читаем данные из буфера модуля
    CAN0.readMsgBuf(&rxId, &len, rxBuf);
    
    // Если поймали наш же пакет
    Serial.print("УСПЕШНО ПРИНЯТ CAN-пакет! ID: 0x");
    Serial.print(rxId, HEX);
    Serial.print(" | Данные: ");
    for(int i = 0; i<len; i++) {
      Serial.print("0x");
      Serial.print(rxBuf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
/*
  if(!digitalRead(CAN_INT_PIN)) { // Если пришел пакет (пин INT упал в LOW)
    long unsigned int rxId;
    unsigned char len = 0;
    unsigned char rxBuf[8];
    
    CAN0.readMsgBuf(&rxId, &len, rxBuf); // Читаем данные из буфера
    
    // Выводим ID пакета и данные в Serial для анализа
    Serial.print("CAN ID: 0x");
    Serial.print(rxId, HEX);
    Serial.print(" Data: ");
    for(int i = 0; i < len; i++) {
      if(rxBuf[i] < 0x10) Serial.print("0"); // Красивый вывод с ведущим нулем
      Serial.print(rxBuf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
*/
// --- ТАЙМЕР ОПРОСА ГБО ---
  if (millis() - lastGboRequest >= gboInterval) {
    lastGboRequest = millis();

    // Очищаем буфер перед отправкой нового запроса, если там скопился мусор
    while(Serial1.available() > 0) {
      Serial1.read(); 
    }

    // Отправляем стартовый байт запроса 'S' (0x53) в блок ГБО
    Serial1.write(0x53); 
    Serial.println("[ГБО] Отправлен запрос 0x53 ('S'). Ждем ответ...");
  }

  // --- ПРИЕМ ДАННЫХ ОТ ГБО ---
  // Если в буфер Serial1 прилетели байты от газового блока
  if (Serial1.available() > 0) {
    Serial.print("[ГБО] Получены байты (HEX): ");
    
    // Считываем всё, что пришло, и выводим в Монитор порта для изучения
    while (Serial1.available() > 0) {
      byte inByte = Serial1.read();
      Serial.print("0x");
      if (inByte < 16) Serial.print("0");
      Serial.print(inByte, HEX);
      Serial.print(" ");
    }
    Serial.println("\n");
  }
}
