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

// Переменные, куда мы сохраним вытащенные данные ГБО
int gboRpm = 0;
float gboGasPress = 0.0;
int gboTempReducer = 0;
int gboTempGas = 0;
float gboInjPetrol = 0.0;
float gboInjGas = 0.0;

// Настройка интервала опроса OBD2
unsigned long lastObdRequest = 0;
const unsigned long obdInterval = 1000; // Запрос каждую 1 секунду

void setup() {
  Serial.begin(115200); // Скорость Монитора порта — 115200
  while(!Serial); // Ожидание открытия Монитора порта

  // Инициализируем Serial1 для связи с ГБО на штатной скорости 57600 
  Serial1.begin(57600); // Для Stag-300 ISA2 (скорость зафиксирована!)

  Serial.println("=== МОДУЛЬ ГБО ИНИЦИАЛИЗИРОВАН ===");
  Serial.println("=== ЗАПУСК ЭМУЛЯТОРА ELM327 (ЗАПРОСЫ OBD2) ===");

  // Инициализация MCP2515. На Almera G15 скорость шины обычно 500 Кбит/с.
  // Кварц на плате MCP2515 стоит на 8 МГц.
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
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
    byte obdQuery[] = {0x02, 0x01, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00};
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
        } else {
        Serial.print("[CAN Попутный] ID: 0x");
        Serial.print(rxId, HEX);
        Serial.println();
        }
      } 
    }
  }
  // На блоке написано STAG-300-4 Qmax basic. Оказалось это китайский клон Stag-300 isa2
  // Cамый первый вариант кода с байт 'S' не сработал, видимо, из-за резисторов 1кОм между RX, TX и пинами Ардуино
  // Второй вариант кода естественно не заработал, т.к. он для Q-генерации блоков STAG
  // Теперь возрат к первому варианту кода, уменьшив резисторы до 220 Ом
  // --- 1. ТАЙМЕР ОТПРАВКИ ЗАПРОСА ---
  if (millis() - lastGboRequest >= gboInterval) {
    lastGboRequest = millis();
    
    // Очищаем старый мусор, если он есть в буфере
    while(Serial1.available() > 0) Serial1.read(); 

    // Отправляем заветный байт 'S'
    Serial1.write(0x53); 
    Serial.println("[ГБО] Отправлен байт 'S'. Ждем ответ...");
  }
  // --- 2. ПРИЕМ И РАСШИФРОВКА ОТВЕТА ---
  // Stag-300 ISA2 обычно присылает пакет длиной более 30 байт. 
  // Ждем, пока в буфере накопится хотя бы 30 байт данных.
  if (Serial1.available() >= 30) {
    byte gboBuf[64]; // Создаем временный массив для пакета
    int bytesRead = 0;

    // Вычитываем байты из порта в наш массив
    while (Serial1.available() > 0 && bytesRead < 64) {
      gboBuf[bytesRead] = Serial1.read();
      bytesRead++;
    }
  // --- ПАРСИНГ (РАСШИФРОВКА) ДАННЫХ ПО ИНДЕКСАМ ---
  // Формулы перевода сырых байт в человеческие значения для ISA2:
  
    // Обороты двигателя: склеиваем два байта (старший и младший)
    gboRpm = (gboBuf[1] << 8) | gboBuf[2]; 

    // Время впрыска (в миллисекундах): делим на 100 для точности до сотых
    gboInjPetrol = ((gboBuf[3] << 8) | gboBuf[4]) / 100.0;
    gboInjGas    = ((gboBuf[5] << 8) | gboBuf[6]) / 100.0;

    // Температуры редуктора и газа (в протоколе ISA2 они идут со смещением или напрямую)
    gboTempReducer = gboBuf[7];
    gboTempGas     = gboBuf[8];

    // Давление газа (МАР): склеиваем два байта и переводим в Бары/Атмосферы
    int rawPress = (gboBuf[9] << 8) | gboBuf[10];
    gboGasPress = rawPress / 100.0; 

    // --- ВЫВОД РЕЗУЛЬТАТОВ В МОНИТОР ПОРТА ---
    Serial.println("====== ДАННЫЕ ИЗ ГАЗОВОГО БЛОКА ======");
    Serial.print("  Обороты: "); Serial.print(gboRpm); Serial.println(" об/мин");
    Serial.print("  Впрыск Бензина: "); Serial.print(gboInjPetrol, 2); Serial.println(" мс");
    Serial.print("  Впрыск Газа: "); Serial.print(gboInjGas, 2); Serial.println(" мс");
    Serial.print("  Темп. Редуктора: "); Serial.print(gboTempReducer); Serial.println(" °C");
    Serial.print("  Темп. Газа: "); Serial.print(gboTempGas); Serial.println(" °C");
    Serial.print("  Давление Газа: "); Serial.print(gboGasPress, 2); Serial.println(" Бар");
    Serial.println("======================================\n");
  }
}
