#include <SPI.h>
#include <mcp_can.h>
//#include <OneWire.h>
//#include <DallasTemperature.h>

//#define ONE_WIRE_BUS 14 // Датчики сидят на цифровом пине 14
//OneWire oneWire(ONE_WIRE_BUS);
//DallasTemperature sensors(&oneWire);

// Настройка пинов 
const int CAN_CS_PIN = 53;     // Пин CS для MCP2515 (на Arduino Mega)
const int CAN_INT_PIN = 2;     // Пин прерывания INT для MCP2515
MCP_CAN CAN0(CAN_CS_PIN);       // Инициализация объекта CAN
// Флаг-маркер: сигнализирует, что пришел новый CAN-пакет
volatile bool flagRecv = false; 
// --- ФУНКЦИЯ ОБРАБОТЧИКА ПРЕРЫВАНИЯ (ISR) ---
// Внутри этой функции нельзя использовать Serial.print и delay! Она должна быть максимально быстрой.
void CAN_ISR() {
  flagRecv = true; 
}
// Настройка интервала опроса ГБО
unsigned long lastGboRequest = 0;
const unsigned long gboInterval = 1000; // Опрашиваем ГБО раз в 1 секунду
// Настройка интервала опроса OBD2
unsigned long lastObdRequest = 0;
const unsigned long obdInterval = 1000; // Запрос каждую 1 секунду

void setup() {
  Serial.begin(115200); // Скорость Монитора порта — 115200
  while(!Serial); // Ожидание открытия Монитора порта

  // Инициализируем Serial1 для связи с ГБО на штатной скорости 57600
  Serial1.begin(57600); 
  Serial.println("=== МОДУЛЬ ГБО ИНИЦИАЛИЗИРОВАН ===");
  Serial.println("=== ЗАПУСК ЭМУЛЯТОРА ELM327 (ЗАПРОСЫ OBD2) ===");

  // Инициализация MCP2515. На Almera G15 скорость шины обычно 500 Кбит/с.
  // Кварц на плате MCP2515 стоит на 8 МГц.
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHz) == CAN_OK) {
    Serial.println("=== MCP2515 успешно запущен! ===");
  } else {
    Serial.println("=== Ошибка инициализации MCP2515... ===");
    while(1); // Останавливаем выполнение при ошибке
  }
  
  // Включаем NORMAL режим, чтобы отправлять запросы в авто
  if (CAN0.setMode(MCP_NORMAL) == CAN_OK) {
    Serial.println(">>> Режим NORMAL активен. Готов к работе с OBD2.");
  }
  
  // НАСТРОЙКА ПИНА ПРЕРЫВАНИЯ
  pinMode(CAN_INT_PIN, INPUT_PULLUP); // Подтягиваем пин к 5V
  
  // Привязываем прерывание: 
  // digitalPinToInterrupt(2) автоматически переведет пин 2 в номер прерывания INT4.
  // FALLING означает, что прерывание сработает, когда на пине 2 напряжение упадет с 5V до 0V (сигнал от MCP2515).
  attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), CAN_ISR, FALLING);
  
  Serial.println(">>> Аппаратное прерывание INT на пине 2 активировано.");

  Serial.println("=== СИСТЕМА ЗАПУЩЕНА И НАДЕЖНО ЗАПИТАНА ПО VIN ===");
  
}

void loop() {
 // --- 1. ОТПРАВКА ЗАПРОСА ОБОРОТОВ ---
  if (millis() - lastObdRequest >= obdInterval) {
    lastObdRequest = millis();
    byte obdQuery = {0x02, 0x01, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00};
    CAN0.sendMsgBuf(0x7DF, 0, 8, obdQuery);
    Serial.println("[OBD2] Отправлен запрос RPM (PID 0x0C)...");
  }

  // --- 2. ОБРАБОТКА ПРИНЯТОГО ПАКЕТА (По флагу прерывания) ---
  if (flagRecv) {
    flagRecv = false; // Сразу сбрасываем флаг

    // Пока модуль MCP2515 удерживает линию прерывания, вычитываем пакеты
    // (в MCP2515 есть два буфера приема, прерывание может вызваться для обоих)
    while (CAN0.checkReceive() == CAN_MSGAVAIL) {
      long unsigned int rxId;
      unsigned char len = 0;
      unsigned char rxBuf;

      CAN0.readMsgBuf(&rxId, &len, rxBuf);

      if (rxId == 0x7E8) {
        if (rxBuf == 0x41 && rxBuf == 0x0C) {
          int rpm = ((rxBuf * 256) + rxBuf) / 4;
          Serial.print(">>> [CAN ПРЕРЫВАНИЕ УСПЕХ!] ID: 0x");
          Serial.print(rxId, HEX);
          Serial.print(" | ОБОРОТЫ: ");
          Serial.print(rpm);
          Serial.println(" об/мин");
        }
      } else {
        Serial.print("[CAN Попутный] ID: 0x");
        Serial.print(rxId, HEX);
        Serial.println();
      }
    }
  }

// --- ТАЙМЕР ОПРОСА ГБО QMAX (Раз в 1 секунду) ---
  if (millis() - lastGboRequest >= gboInterval) {
    lastGboRequest = millis();

    // Очищаем буфер от старого мусора
    while(Serial1.available() > 0) { Serial1.read(); }

    // Формируем правильный пакет запроса параметров для блоков Q-generation
    // 0xE4 - маркер начала, 0x00 - длина доп. данных, 0x53 ('S') - команда запроса, 0x37 - CRC
    byte qmaxRequest[] = {0xE4, 0x00, 0x53, 0x37};
    
    // Отправляем массив из 4 байт в Serial1
    Serial1.write(qmaxRequest, 4); 
    Serial.println("[ГБО] Отправлен пакет инициализации QMAX. Ждем ответ...");
  }

  // --- ПРИЕМ ДАННЫХ ОТ ГБО ---
  if (Serial1.available() > 0) {
    // Даем блоку QMAX долю секунды, чтобы дописать весь пакет в буфер Ардуино
    delay(20); 
    
    Serial.print("[ГБО] Ответ от QMAX (HEX): ");
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
