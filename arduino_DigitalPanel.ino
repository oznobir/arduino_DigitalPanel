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
  bool currentHandbrakeState = digitalRead(HANDBRAKE_PIN);
  if (currentHandbrakeState != lastHandbrakeState) {
    if (currentHandbrakeState == LOW) {
      Serial.println(">>> Ручник затянут!");
    } else {
      Serial.println(">>> Ручник отпущен.");
    }
    lastHandbrakeState = currentHandbrakeState;
  }

  bool currentTurnLeftState = digitalRead(TURN_LEFT_PIN);
  if (currentTurnLeftState != lastTurnLeftState) {
    if (currentTurnLeftState == LOW) {
      Serial.println(">>> Левый поворотник включен!");
    } else {
      Serial.println(">>> Левый поворотник выключен.");
    }
    lastTurnLeftState = currentTurnLeftState;
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
