#include <SPI.h>
#include <mcp_can.h>


// --- КОНФИГУРАЦИЯ CAN-МОДУЛЯ ---
const int CAN_CS_PIN = 53;  // Пин CS для MCP2515 (на Arduino Mega)
//const int CAN_INT_PIN = 2;     // Пин прерывания INT для MCP2515
MCP_CAN CAN0(CAN_CS_PIN);  // Инициализация объекта CAN
// Флаг-маркер: сигнализирует, что пришел новый CAN-пакет
//volatile bool flagRecv = false;
// --- ФУНКЦИЯ ОБРАБОТЧИКА ПРЕРЫВАНИЯ (ISR) ---
// Внутри этой функции нельзя использовать Serial.print и delay! Она должна быть максимально быстрой.
/*
void CAN_ISR() {
  flagRecv = true; 
}
*/
// Настройка и переменные для OBD2
unsigned long lastCanRequest = 0;
const unsigned long canInterval = 15000;  // Будем запрашивать данные из OBD2 в 15 секунды
int obdRpm = 0;                         // Сюда сохраним обороты от машины
// Строка запроса оборотов (RPM) по стандарту OBD2
byte obdQuery[8] = { 0x02, 0x01, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00 };

// Настройка и переменные для ГБО
unsigned long lastGboRequest = 0;
const unsigned long gboInterval = 150;  // Опрашиваем ГБО раз в 150 мс
const byte REQ_PARAMS = 0x3A; //байт запроса
const int PACKET_SIZE = 45;

int bytesRead = 0;
byte incomeBuffer[PACKET_SIZE]; // Выделяем фиксированный буфер под пакет ГБО

struct StagParameters {
  int rpm;
  float pressure;
  int tempReducer;
  int tempGas;
  float injectionBenz;
  float injectionGas;
};

StagParameters stag;

void setup() {
  Serial.begin(115200);  // Скорость Монитора порта — 115200 (* 16/12 = 153600)
  while (!Serial);  // Ожидание открытия Монитора порта

  // Инициализируем Serial1 для связи с ГБО на штатной скорости 57600 (* 16/12 = 76800)
  Serial1.begin(57600);  // Для Stag-300 ISA2 (скорость зафиксирована!)
  // ВКЛЮЧАЕМ ВНУТРЕННЮЮ ПОДТЯЖКУ ДЛЯ ПОРТА ПРИЕМА ГБО (попытка через оптопары РС817)
  //pinMode(19, INPUT_PULLUP);
  // Принудительно очищаем буферы при старте
  //while (Serial1.available() > 0) Serial1.read();

  Serial.println(F("=== МОДУЛЬ ГБО ИНИЦИАЛИЗИРОВАН ==="));

  // Инициализация CAN (с использованием F() макроса для экономии памяти)
  Serial.println(F("=== ИНИЦИАЛИЗАЦИЯ CAN-МОДУЛЯ ==="));
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println(F("=== MCP2515 успешно запущен! ==="));
    CAN0.setMode(MCP_NORMAL);  // Режим пассивного прослушивания/работы
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

  // Таймер отправки запроса
  if (millis() - lastGboRequest >= gboInterval) {
    lastGboRequest = millis();

    // Очищаем накопившийся мусор перед отправкой новой команды
    while (Serial1.available() > 0) Serial1.read();
    // Некоторые клоны ISA2 требуют перед байтом 'S' (0x53) послать
    // пустой нулевой байт, чтобы выровнять буфер китайского чипа
    //Serial1.write(0x00);
    //delay(5);             // Крошечная пауза
    Serial1.write(REQ_PARAMS);  // Отправляем байт зароса напрямую в Stag-300
    Serial.println(F("[ГБО] Запрос в ГБО отправлен..."));
  }


  // Сбор и чтение ответа от ГБО
  if (Serial1.available() >= PACKET_SIZE) {
    // Пакет STAG 11.3 всегда начинается с байта 0xFF или 0x0F (зависит от чипа клона)
    // Если на нулевой позиции мусор — сдвигаем буфер
    if (Serial1.peek() != 0xFF && Serial1.peek() != 0x0F) { 
      Serial1.read(); 
      return;
    }
    // Читаем пакет целиком
    Serial1.readBytes(incomeBuffer, PACKET_SIZE);
    
    // Расшифровка по сетке прошивки 11.3
    parseStag113();
    
    // Вывод обработанных данных
    sendDataToDisplay();   
  }

  // === БЛОК 2. ОПРОС CAN-ШИНЫ (MCP2515) ===
  // 2А. Отправка запроса в OBD2 по таймеру
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
  // 2Б. Ожидание и чтение ответа от машины
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
void parseStag113() {
  // 1. Обороты (RPM)
  stag.rpm = (incomeBuffer[1] << 8) | incomeBuffer[2];
  if(stag.rpm > 8000 || stag.rpm < 0) stag.rpm = 0; // Защита от всплесков
  
  // 2. Давление газа (Bar)
  int rawPress = (incomeBuffer[3] << 8) | incomeBuffer[4];
  stag.pressure = rawPress * 0.01;
  
  // 3. Температуры (смещение -40 градусов)
  stag.tempReducer = incomeBuffer[11] - 40;
  stag.tempGas = incomeBuffer[12] - 40;
  
  // 4. Время впрыска (миллисекунды)
  int rawBenz = (incomeBuffer[13] << 8) | incomeBuffer[14];
  stag.injectionBenz = rawBenz * 0.01;
  
  int rawGas = (incomeBuffer[15] << 8) | incomeBuffer[16];
  stag.injectionGas = rawGas * 0.01;
}

void sendDataToDisplay() {
  // На данном этапе выводим в консоль. Сюда мы интегрируем код вашего дисплея.
  Serial.print("Обороты: "); Serial.print(stag.rpm);
  Serial.print(" | Давление: "); Serial.print(stag.pressure, 2);
  Serial.print(" | Т_Ред: "); Serial.print(stag.tempReducer);
  Serial.print("°C | Впрыск_Г: "); Serial.print(stag.injectionGas, 2);
  Serial.println(" мс");
}
