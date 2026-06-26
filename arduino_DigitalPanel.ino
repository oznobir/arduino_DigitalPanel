#include <SPI.h>
#include <mcp_can.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 14 // Датчики сидят на цифровом пине 14
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Настройка пинов 
const int CAN_CS_PIN = 53;     // Пин CS для MCP2515 (на Arduino Mega)
const int CAN_INT_PIN = 2;     // Пин прерывания INT для MCP2515
const int HANDBRAKE_PIN = 16;   // Подключение ручника (через оптопару)
const int TURN_LEFT_PIN = 17;   // Подключение поворотника (через оптопару)
const int FUEL_PIN = A0;       // Пин ДУТ с фильтром и защитой

MCP_CAN CAN0(CAN_CS_PIN);       // Инициализация объекта CAN

// Переменные таймера для опроса температуры (чтобы не тормозить CAN-шину)
unsigned long lastTempRequest = 0;
const unsigned long tempInterval = 3000; // Опрос температуры раз в 3 секунды

// Переменные для отслеживания изменения состояния входов (защита от спама в порт)
bool lastHandbrakeState = HIGH;
bool lastTurnLeftState = HIGH;

void setup() {
  Serial.begin(115200); // Скорость Монитора порта — 115200
  sensors.begin();
// ------------------------------------------
// Поиск ID датчиков температуры
// выполняется разово
//Датчик под Индексом [0] имеет ID: { 0x28, 0x79, 0xF4, 0xC8, 0x00, 0x00, 0x00, 0x8C }
//Датчик под Индексом [1] имеет ID: { 0x28, 0xC5, 0xD7, 0xC9, 0x00, 0x00, 0x00, 0xCF }
//Третий датчик неработает (может бракованный или плохо обжаты контакты)
/*
  int count = sensors.getDeviceCount();
  Serial.println("--- СКАНИРОВАНИЕ ШИНЫ ONE-WIRE ---");
  Serial.print("Найдено рабочих датчиков: ");
  Serial.println(count);
  Serial.println("----------------------------------");
  
  for (int i = 0; i < count; i++) {
    DeviceAddress address;
    if (sensors.getAddress(address, i)) {
      Serial.print("Датчик под Индексом [");
      Serial.print(i);
      Serial.print("] имеет ID: { ");
      for (uint8_t j = 0; j < 8; j++) {
        Serial.print("0x");
        if (address[j] < 16) Serial.print("0"); // Добавляем ноль для красоты
        Serial.print(address[j], HEX);
        if (j < 7) Serial.print(", ");
      }
      Serial.println(" }");
    }
  }
  Serial.println("----------------------------------");
  Serial.println("Сканирование завершено. Можете скопировать адреса.");

// Поиск ID датчиков температуры
// выполняется разово
// ------------------------------------------
*/

  // Настройка пинов для оптопар PC817 (обязательно PULLUP)
  pinMode(HANDBRAKE_PIN, INPUT_PULLUP);
  pinMode(TURN_LEFT_PIN, INPUT_PULLUP);

  // Инициализация MCP2515. На Almera G15 скорость шины обычно 500 Кбит/с.
  // Кварц на плате MCP2515 стоит на 8 МГц.
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("MCP2515 успешно запущен!");
  } else {
    Serial.println("Ошибка инициализации MCP2515...");
    while(1); // Останавливаем выполнение при ошибке
  }
  
  CAN0.setMode(MCP_NORMAL);   // Переводим CAN в рабочий режим
  pinMode(CAN_INT_PIN, INPUT); 

}

void loop() {
  // --- 1. ОПРОС ДАТЧИКОВ С ОПТОПАРЫ PC817 (Вывод только при изменении) ---
  static unsigned long lastDebounceTimeHandbrake = 0;
  static unsigned long lastDebounceTimeTurn = 0;
  const unsigned long debounceDelay = 50; // Время фильтрации дребезга (50 мс)

  // Проверка РУЧНИКА
  bool readingHandbrake = digitalRead(HANDBRAKE_PIN);
  if (readingHandbrake != lastHandbrakeState) {
    // Если состояние изменилось, запускаем/сбрасываем таймер
    if (millis() - lastDebounceTimeHandbrake > debounceDelay) {
      if (readingHandbrake == LOW) {
       Serial.println(">>> Ручник затянут!");
      } else {
       Serial.println(">>> Ручник отпущен.");
      }
      lastHandbrakeState = readingHandbrake;
      lastDebounceTimeHandbrake = millis();
    }
  } else {
    lastDebounceTimeHandbrake = millis(); // Сброс таймера, если состояние стабильно
  }

  // Проверка ПОВОРОТНИКА
  bool readingTurnLeft = digitalRead(TURN_LEFT_PIN);
  if (readingTurnLeft != lastTurnLeftState) {
   if (millis() - lastDebounceTimeTurn > debounceDelay) {
      if (readingTurnLeft == LOW) {
        Serial.println(">>> Левый поворотник включен!");
      } else {
        Serial.println(">>> Левый поворотник выключен.");
     }
     lastTurnLeftState = readingTurnLeft;
     lastDebounceTimeTurn = millis();
    }
  } else {
    lastDebounceTimeTurn = millis();
  }

  // --- 2. ЧТЕНИЕ ДАННЫХ ИЗ CAN-ШИНЫ АВТОМОБИЛЯ (Мгновенно) ---
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

  // --- 3. ЗАПРОС ТЕМПЕРАТУРЫ DS18B20 ПО ТАЙМЕРУ ---
  if (millis() - lastTempRequest >= tempInterval) {
    lastTempRequest = millis(); // Сброс таймера

    // Чтение датчика уровня топлива
    int rawFuel = analogRead(FUEL_PIN); // Считываем АЦП (получим от 0 до ~769)
  
    // Калибровка под потенциометр B1k и резистор 330 Ом
    // R1 (330 Ом) и R2 (1000 Ом). Всего 1330 Ом. 
    // 5в*(1000 Ом/1330 Ом) = 3,76в
    // 3,76в/5в*1023 = 769 АЦП
    // 0 Ом (пустой бак) = 0 АЦП. 1000 Ом (полный бак) = 769 АЦП.
    int fuelPercent = map(rawFuel, 0, 769, 0, 100); 
  
    // Защита от выхода за границы 0-100% (если потенциометр выдаст чуть больше 769)
    fuelPercent = constrain(fuelPercent, 0, 100); 
    
    sensors.requestTemperatures(); // Запрос у всех датчиков
    
    // Считываем по индексам. Вывод в формате float
    Serial.print("  [TEMP] T1: ");
    Serial.print(sensors.getTempCByIndex(0));
    Serial.print(" °C | T2: ");
    Serial.print(sensors.getTempCByIndex(1));
    Serial.print(" °C | T3: ");
    Serial.print(sensors.getTempCByIndex(2));
    Serial.println(" °C");

    // Вывод уровня топлива в Монитор порта
    Serial.print("  [FUEL] Значение АЦП: ");
    Serial.print(rawFuel);
    Serial.print(" из 769 | Уровень в баке: ");
    Serial.print(fuelPercent);
    Serial.println(" %");
    Serial.println();
  }
}
