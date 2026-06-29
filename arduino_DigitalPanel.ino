#include <SPI.h>
#include <mcp_can.h>


// --- КОНФИГУРАЦИЯ CAN-МОДУЛЯ ---
const int CAN_CS_PIN = 53;     // Пин CS для MCP2515 (на Arduino Mega)
//const int CAN_INT_PIN = 2;     // Пин прерывания INT для MCP2515
MCP_CAN CAN0(CAN_CS_PIN);       // Инициализация объекта CAN
// Флаг-маркер: сигнализирует, что пришел новый CAN-пакет
//volatile bool flagRecv = false; 
// --- ФУНКЦИЯ ОБРАБОТЧИКА ПРЕРЫВАНИЯ (ISR) ---
// Внутри этой функции нельзя использовать Serial.print и delay! Она должна быть максимально быстрой.
/*
void CAN_ISR() {
  flagRecv = true; 
}
*/
// Настройка интервала опроса OBD2
unsigned long lastCanRequest = 0;
const unsigned long canInterval = 300; // Будем запрашивать данные из OBD2 3 раза в секунду
int obdRpm = 0; // Сюда сохраним обороты от машины

// Настройка интервала опроса ГБО
unsigned long lastGboRequest = 0;
const unsigned long gboInterval = 1000; // Опрашиваем ГБО раз в 1 секунду

// Строка запроса оборотов (RPM) по стандарту OBD2
byte obdQuery[8] = {0x02, 0x01, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00};

// Переменные, куда мы сохраним вытащенные данные ГБО
int gboRpm = 0;
float gboGasPress = 0.0;
//int gboTempReducer = 0;
//int gboTempGas = 0;
//float gboInjPetrol = 0.0;
//float gboInjGas = 0.0;
int bytesRead = 0;
byte gboBuf[64]; // Выделяем фиксированный буфер под пакет ГБО

void setup() {
  Serial.begin(115200); // Скорость Монитора порта — 115200
  while(!Serial); // Ожидание открытия Монитора порта

  // Инициализируем Serial1 для связи с ГБО на штатной скорости 57600 
  Serial1.begin(57600); // Для Stag-300 ISA2 (скорость зафиксирована!)
  // Принудительно очищаем буферы при старте
  while(Serial1.available() > 0) Serial1.read();
  Serial.println(F("=== МОДУЛЬ ГБО ИНИЦИАЛИЗИРОВАН ==="));  

  // Инициализация CAN (с использованием F() макроса для экономии памяти)
  Serial.println(F("=== ИНИЦИАЛИЗАЦИЯ CAN-МОДУЛЯ ==="));
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println(F("=== MCP2515 успешно запущен! ==="));
    CAN0.setMode(MCP_NORMAL); // Режим пассивного прослушивания/работы
  } else {
    Serial.println(F("=== Ошибка инициализации MCP2515! ==="));
  }
  // НАСТРОЙКА ПИНА ПРЕРЫВАНИЯ
  //pinMode(CAN_INT_PIN, INPUT_PULLUP); // Подтягиваем пин к 5V
  
  // Привязываем прерывание: 
  // digitalPinToInterrupt(2) автоматически переведет пин 2 в номер прерывания INT4.
  // FALLING означает, что прерывание сработает, когда на пине 2 напряжение упадет с 5V до 0V (сигнал от MCP2515).
  //attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), CAN_ISR, FALLING);
 
  Serial.println(F("=== СИСТЕМА ГОТОВА К ПРЯМОМУ ТЕСТУ ==="));
}

void loop() {
  // === БЛОК 1. ОПРОС И ПАРСИНГ ГБО (Stag-300 ISA2) ===
  
  // Таймер отправки запроса
  if (millis() - lastGboRequest >= gboInterval) {
    lastGboRequest = millis();
    
    // Очищаем накопившийся мусор перед отправкой новой команды
    while(Serial1.available() > 0) Serial1.read(); 
    
    Serial1.write(0x53); // Отправляем байт 'S' напрямую в Stag-300
    Serial.println(F("[ГБО] Запрос 0x53 отправлен..."));
  }

  // Сбор и чтение ответа от ГБО
  if (Serial1.available() >= 30) {
    bytesRead = 0;

    // Вычитываем байты из буфера в массив
    while (Serial1.available() > 0 && bytesRead < 64) {
      gboBuf[bytesRead] = Serial1.read();
      bytesRead++;
    }

    // Если считали полноценный пакет Stag-300 ISA2
    if (bytesRead >= 30) {
      // Склеиваем обороты (обычно байты под индексом 1 и 2 в ISA2)
      gboRpm = (gboBuf[1] << 8) | gboBuf[2]; 
      
      // Склеиваем давление MAP (обычно байты 9 и 10)
      int rawPress = (gboBuf[9] << 8) | gboBuf[10];
      gboGasPress = rawPress / 100.0; 

      // Время впрыска (в миллисекундах): делим на 100 для точности до сотых
      //gboInjPetrol = ((gboBuf[3] << 8) | gboBuf[4]) / 100.0;
      //gboInjGas    = ((gboBuf[5] << 8) | gboBuf[6]) / 100.0;

      // Температуры редуктора и газа (в протоколе ISA2 они идут со смещением или напрямую)
      //gboTempReducer = gboBuf[7];
      //gboTempGas     = gboBuf[8];

      // Вывод результатов (все строки упакованы в F(), RAM свободна!)
      Serial.print(F("[ГБО] Ответ получен! Обороты: "));
      Serial.print(gboRpm);
      Serial.print(F(" об/мин | Давление: "));
      Serial.print(gboGasPress, 2);
      Serial.println(F(" Бар"));    
      /*
      Serial.print(F("  Впрыск Бензина: ")); 
      Serial.print(gboInjPetrol, 2); 
      Serial.print(F(" мс | Впрыск Газа: "));
      Serial.print(gboInjGas, 2); 
      Serial.println(" мс | Темп. Редуктора: ");
      Serial.print(gboTempReducer); 
      Serial.println(" °C | Темп. Газа: ");
      Serial.print(gboTempGas); 
      Serial.println(" °C");
      */
    }
  }

  
  // === БЛОК 2. ОПРОС CAN-ШИНЫ (MCP2515) ===
  // 2А. Отправка запроса в OBD2 по таймеру
  if (millis() - lastCanRequest >= canInterval) {
    lastCanRequest = millis();
    
    // Отправляем 8 байт запроса на ID 0x7E0 (стандартный ID моторного блока)
    // Параметры: ID, тип кадра (0 - стандартный), длина (8 байт), массив данных
    byte sndStat = CAN0.sendMsgBuf(0x7E0, 0, 8, obdQuery);
    
    if(sndStat == CAN_OK) {
      Serial.println(F("[OBD2] Запрос RPM отправлен!"));
    } else {
      Serial.println(F("[OBD2] Ошибка отправки запроса в шину. Проверьте провода!"));
    }
  }
  // 2Б. Ожидание и чтение ответа от машины
  //
  if (CAN0.checkReceive() == CAN_MSGAVAIL) {
    long unsigned int rxId;
    unsigned char len = 0;
    unsigned char rxBuf[8]; // Буфер под 8 байт ответа OBD2
    
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
/*
  // Проверяем, есть ли физически принятый пакет в буфере MCP2515
  if (CAN0.checkReceive() == CAN_MSGAVAIL) {
    long unsigned int rxId;
    unsigned char len = 0;
    unsigned char rxBuf[8];
    
    // Считываем пакет
    CAN0.readMsgBuf(&rxId, &len, rxBuf);
    
    // Строго отсекаем пустые ID, выводим только реальный трафик
    if (rxId > 0) {
      Serial.print(F("[CAN] Поймали реальный пакет! ID: 0x"));
      Serial.print(rxId, HEX);
      Serial.print(F(" | Длина данных: "));
      Serial.println(len);
    }
  }
  */
}
