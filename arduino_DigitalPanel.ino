#include <Arduino.h>
// =========================================================================
// АВТОМОБИЛЬНЫЙ КОНТРОЛЛЕР ПИТАНИЯ И ДАТЧИКОВ (REALDASH CAN ВЕРСИЯ)
// Arduino Mega Pro. Посекундные таймеры + Бинарный протокол RealDash CAN
// =========================================================================


// --- НАСТРОЙКА ПИНОВ (КОНФИГУРАЦИЯ ДЛЯ ARDUINO MEGA) ---
const int PIN_BATTERY_SENSE = A0;   // Аналоговый вход: замер напряжения АКБ
const int PIN_DOOR_TRIGGER  = 2;    // Цифровой вход (через оптопару): сигнал ЦЗ / Двери / Плафона
const int PIN_IGNITION      = 3;    // Цифровой вход (через оптопару): Зажигание (Клемма 15)
const int PIN_ACC_OUTPUT    = 4;    // Цифровой выход: управление ключом BTS442 (ACC магнитолы)
const int PIN_HOLD_POWER    = 5;    // Цифровой выход: удержание питания схемы (на базу BC547B)

// --- ТАЙМИНГИ И КОНСТАНТЫ ---
const unsigned long TIMEOUT_WAIT_IGNITION = 300000; // Время ожидания зажигания (5 минут)
const unsigned long WAKE_FILTER_DELAY     = 3000;   // Фильтр сигнала ЦЗ / Двери / Плафона (3 секунды)
const float CRITICAL_BATTERY_VOLTAGE      = 11.9;    // Порог защиты аккумулятора от разряда (Вольты)

enum SystemState {
  STATE_SLEEP,
  STATE_PRE_DRIVE_WAKE,
  STATE_DRIVE,
  STATE_SHUTDOWN
};

SystemState currentState = STATE_SLEEP;
unsigned long wakeUpTimerStart = 0;

// --- ТАЙМИНГИ ДАТЧИКОВ ---
unsigned long timerFastSensors = 0;
unsigned long timerNormSensors = 0;
unsigned long timerSlowSensors = 0;

const unsigned long INTERVAL_FAST = 50;   // 50 мс
const unsigned long INTERVAL_NORM = 200;  // 200 мс
const unsigned long INTERVAL_SLOW = 2000; // 2 секунды (Температуры)

// --- СТРУКТУРА ДЛЯ СБОРКИ ПАКЕТА REALDASH CAN ---
// Объединение (union) позволяет записывать данные как переменные, 
// а отправлять в порт как массив байт, не тратя время на конвертацию.
union RealDashFrame {
  struct {
    uint32_t canId;
    uint8_t data[8];
  } __attribute__((packed)) frame;
  uint8_t bytes[12]; // Полный размер кадра (4 байта ID + 8 байт данных)
};

// Функция точного измерения напряжения аккумулятора
float readBatteryVoltage() {
  int rawSum = 0;
  for (int i = 0; i < 10; i++) {
    rawSum += analogRead(PIN_BATTERY_SENSE);
    delay(2); // Стабилизация АЦП
  }
  float averageRaw = (float)rawSum / 10.0;
  
  // Пересчет АЦП (5.0В - опорное, делитель 10кОм / 3.3кОм)
  float vPin = (averageRaw * 5.0) / 1023.0; 
  float vBatt = vPin * ((10.0 + 3.3) / 3.3); 
  return vBatt;
}
// Универсальная функция отправки кадра в RealDash
void sendRealDashFrame(uint32_t canId, uint8_t* data8Bytes) {
  // 1. Отправляем обязательный маркер начала бинарного пакета (4 байта)
  const uint8_t serialBlockHeader[4] = { 0x44, 0x33, 0x22, 0x11 };
  Serial.write(serialBlockHeader, 4);
  
  // 2. Собираем и отправляем сам кадр
  RealDashFrame myFrame;
  myFrame.frame.canId = canId;
  memcpy(myFrame.frame.data, data8Bytes, 8);
  
  Serial.write(myFrame.bytes, 12);
}

void setup() {
  // 1. МГНОВЕННО захватываем питание платы, пока не исчез физический импульс от двери!
  pinMode(PIN_HOLD_POWER, OUTPUT);
  digitalWrite(PIN_HOLD_POWER, HIGH); 
  
  // Инициализация остальных пинов
  pinMode(PIN_ACC_OUTPUT, OUTPUT);
  digitalWrite(PIN_ACC_OUTPUT, LOW); // Магнитола пока строго выключена
  
  pinMode(PIN_DOOR_TRIGGER, INPUT_PULLUP); // Используем подтяжку для оптопары
  pinMode(PIN_IGNITION, INPUT_PULLUP);
  
  // Важно: для бинарного протокола поднимаем скорость порта выше стандартной,
  // чтобы пакеты летели без задержек. В самом RealDash тоже укажем 115200.
  Serial.begin(115200);
  Serial.println(F("--- MCU AWAKED ---"));
  
  // 2. ЭКСПРЕСС-ДИАГНОСТИКА АКБ
  float currentVoltage = readBatteryVoltage();
  Serial.print(F("Battery Voltage: ")); Serial.print(currentVoltage); Serial.println(F("V"));
  
  if (currentVoltage < CRITICAL_BATTERY_VOLTAGE) {
    Serial.println(F("CRITICAL: Battery Low! Emergency Shutdown."));
    currentState = STATE_SHUTDOWN;
    return;
  }
  
  // 3. ФИЛЬТР ЛОЖНЫХ ПРОСЫПАНИЙ (Поворотники / Постановка на охрану)
  Serial.println(F("Waiting for stabilization (Anti-Indicator Filter)..."));
  unsigned long filterStart = millis();
  bool realWakeUpDetected = false;
  
  while (millis() - filterStart < WAKE_FILTER_DELAY) {
    // Если в течение 3 секунд человек включил зажигание — это точно не ложный сигнал
    if (digitalRead(PIN_IGNITION) == LOW) { 
      realWakeUpDetected = true; 
      break; 
    }
    // Если импульс двери/ЦЗ/плафона все еще активен (удерживается), значит машина открыта
    if (digitalRead(PIN_DOOR_TRIGGER) == LOW) {
      realWakeUpDetected = true;
    }
  }
  
  if (!realWakeUpDetected) {
    // Сигнал моргнул и пропал (машину просто закрыли, или это был короткий импульс)
    Serial.println(F("False alarm (Indicator blink detected). Going back to sleep immediately."));
    currentState = STATE_SHUTDOWN;
  } else {
    // Сигнал подтвержден, хозяин открыл машину или завел её
    Serial.println(F("Wake-up confirmed. Turning on Android ACC."));
    digitalWrite(PIN_ACC_OUTPUT, HIGH); // ВКЛЮЧАЕМ BTS442 (Магнитолу)
    currentState = STATE_PRE_DRIVE_WAKE;
    wakeUpTimerStart = millis();
  }
}

void loop() {
  // =========================================================================
  // БЛОК ОТПРАВКИ БИНАРНЫХ ДАННЫХ ПО ТАЙМЕРАМ
  // =========================================================================
  if (currentState == STATE_PRE_DRIVE_WAKE || currentState == STATE_DRIVE) {
    unsigned long currentMillis = millis();
    uint8_t buffer[8]; // Временный буфер для упаковки байт

    // 1. БЫСТРЫЕ ДАТЧИКИ (CAN ID: 100, каждые 50 мс)
    if (currentMillis - timerFastSensors >= INTERVAL_FAST) {
      timerFastSensors = currentMillis;
      
      // Пример упаковки: Обороты (RPM) обычно занимают 2 байта (uint16_t)
      uint16_t rpm = 2500; // Сюда опрос датчика
      uint16_t speed = 60;  // Скорость км/ч
      
      buffer[0] = lowByte(rpm);
      buffer[1] = highByte(rpm);
      buffer[2] = lowByte(speed);
      buffer[3] = highByte(speed);
      buffer[4] = 0; buffer[5] = 0; buffer[6] = 0; buffer[7] = 0; // Остальные зануляем
      
      sendRealDashFrame(100, buffer); // Отправляем кадр 100
    }

    // 2. СТАНДАРТНЫЕ ДАТЧИКИ (CAN ID: 101, каждые 200 мс)
    if (currentMillis - timerNormSensors >= INTERVAL_NORM) {
      timerNormSensors = currentMillis;
      
      // Читаем наш вольтметр АКБ
      // Умножим на 10, чтобы передать красивым целым числом (например, 12.4В станет 124)
      float vBatt = readBatteryVoltage();
      uint8_t packedVoltage = (uint8_t)(vBatt * 10.0); 
      
      uint8_t fuelLevel = 45; // Пример: Уровень топлива в %
      
      buffer[0] = packedVoltage;
      buffer[1] = fuelLevel;
      buffer[2] = 0; buffer[3] = 0; buffer[4] = 0; buffer[5] = 0; buffer[6] = 0; buffer[7] = 0;
      
      sendRealDashFrame(101, buffer); // Отправляем кадр 101
    }

    // 3. МЕДЛЕННЫЕ ДАТЧИКИ (CAN ID: 102, каждые 2 секунды)
    if (currentMillis - timerSlowSensors >= INTERVAL_SLOW) {
      timerSlowSensors = currentMillis;
      
      // Температура ОЖ и температура за бортом (передаем со смещением +40, чтобы не возиться с минусом)
      int tempEngine = 90; // Сюда чтение датчика температуры
      int tempOutside = -5;
      
      buffer[0] = (uint8_t)(tempEngine + 40);  // 90 + 40 = 130
      buffer[1] = (uint8_t)(tempOutside + 40); // -5 + 40 = 35
      buffer[2] = 0; buffer[3] = 0; buffer[4] = 0; buffer[5] = 0; buffer[6] = 0; buffer[7] = 0;
      
      sendRealDashFrame(102, buffer); // Отправляем кадр 102
    }
  }
  // =========================================================================
  // АВТОМАТ СОСТОЯНИЙ ПИТАНИЯ
  // =========================================================================
  
  // Оптопара инвертирует сигнал: когда на ней +12В, пин Ардуино притягивается к GND (LOW)
  bool isIgnitionOn = (digitalRead(PIN_IGNITION) == LOW);
  
  switch (currentState) {
    
    case STATE_PRE_DRIVE_WAKE:
      // Ждем зажигания в течение 5 минут
      if (isIgnitionOn) {
        currentState = STATE_DRIVE;
        Serial.println(F("State Changed: DRIVE. Enjoy your ride."));
      } 
      else if (millis() - wakeUpTimerStart >= TIMEOUT_WAIT_IGNITION) {
        currentState = STATE_SHUTDOWN;
        Serial.println(F("State Changed: SHUTDOWN (Timeout reached)."));
      }
      break;

    case STATE_DRIVE:
      // В режиме поездки нам плевать на любые клацанья ЦЗ или дверей. Мы смотрим только на зажигание.
      if (!isIgnitionOn) {
        currentState = STATE_SHUTDOWN;
        Serial.println(F("State Changed: SHUTDOWN (Ignition Off)."));
      }
      break;

    case STATE_SHUTDOWN:
      Serial.println(F("Shutting down ACC..."));
      digitalWrite(PIN_ACC_OUTPUT, LOW); // Обесточиваем магнитолу через BTS442
      delay(500); // Короткая пауза для записи кэша магнитолы
      
      Serial.println(F("Releasing PIN_HOLD_POWER. Goodbye."));
      digitalWrite(PIN_HOLD_POWER, LOW); // Отпускаем базу BC547B, снимая питание схемы
      
      // ЗАЩИТА: Если палец водителя все еще жмет кнопку ЦЗ, или конденсаторы в сети разряжаются,
      // крутимся в пустом цикле и ждем полной физической смерти питания, блокируя выполнение кода.
      while(true) {
        // Процессор застывает здесь до полного исчезновения напряжения на шине 5V
      }
      break;
      
    case STATE_SLEEP:
      // Сюда программа никогда не дойдет, так как питание отключится физически
      break;
  }
}
// <?xml version="1.0" encoding="utf-8"?>
// <realdashcan version="1.1">
//   <!-- Кадр 100: Быстрые датчики (50 мс) -->
//   <frame id="100">
//     <value targetId="37" offset="0" length="2" units="RPM"></value> <!-- Обороты (длина 2 байта) -->
//     <value targetId="33" offset="2" length="2" units="km/h"></value> <!-- Скорость (длина 2 байта) -->
//   </frame>

//   <!-- Кадр 101: Обычные датчики (200 мс) -->
//   <frame id="101">
//     <!-- Напряжение АКБ: берем 1 байт, делим обратно на 10 (range="0,25.5") -->
//     <value targetId="12" offset="0" length="1" conversion="V/10"></value> 
//     <value targetId="16" offset="1" length="1" units="%"></value> <!-- Топливо -->
//   </frame>

//   <!-- Кадр 102: Медленные датчики (2000 мс) -->
//   <frame id="102">
//     <!-- Температуры: вычитаем обратно смещение 40 (conversion="V-40") -->
//     <value targetId="14" offset="0" length="1" conversion="V-40" units="C"></value> <!-- Мотор -->
//     <value targetId="27" offset="1" length="1" conversion="V-40" units="C"></value> <!-- Улица -->
//   </frame>
// </realdashcan>

// Примечание: Параметр targetId — это внутренний уникальный номер датчика в экосистеме RealDash 
// (например, 37 — это всегда RPM, а 12 — вольтаж батареи). Полный список этих ID есть на официальном сайте RealDash.
