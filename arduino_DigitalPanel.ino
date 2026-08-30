#include <Arduino.h>
// =========================================================================
// АВТОМОБИЛЬНЫЙ КОНТРОЛЛЕР ПИТАНИЯ И ДАТЧИКОВ (REALDASH CAN ВЕРСИЯ)
// Arduino Mega Pro. Посекундные таймеры + Бинарный протокол RealDash CAN
// =========================================================================

// ==========================================================================
// --- КОНФИГУРАЦИЯ ПИНОВ ---
// ==========================================================================
const byte PIN_BATTERY_SENSE = A0; // Вольтметр (делитель 10кОм и 3.3кОм)
const byte PIN_ECT = A1;
const byte PIN_FUEL = A2;

const byte PIN_RPM   = 2;          // Вход RPM через PC817 (Прерывание 0)
const byte PIN_SPEED = 3;          // Вход Скорости через PC817 (Прерывание 1)
const byte PIN_ACC_OUTPUT    = 4;    // Цифровой выход: управление ключом BTS442 (ACC магнитолы)
const byte PIN_HOLD_POWER    = 5;    // Цифровой выход: удержание питания схемы (на базу BC547B)
const byte PIN_DOOR_TRIGGER  = 6;    // Цифровой вход (через оптопару): сигнал ЦЗ / Двери / Плафона
const byte PIN_IGNITION      = 7;    // Цифровой вход (через оптопару): Зажигание (Клемма 15)

// ==========================================================================
// --- СТРУКТУРА И МАССИВЫ ИНДИКАТОРОВ (ЛАМП) ---
// ==========================================================================
struct InputChannel {
  int pin;            // Номер пина Arduino
  byte bitPosition;   // В какой бит байта упаковать (0..7)
  bool currentState;  // Текущее состояние (true/false)
  String name;        // Имя для вывода в Монитор порта
};

// КАДР 101: Стандартные лампы (Интервал: 200 мс)
const int LAMPS_COUNT_101 = 5;
InputChannel indicators101[LAMPS_COUNT_101] = {
  {9, 0, false, "[L_TURN] "}, // Бит 0
  {11, 1, false, "[R_TURN] "}, // Бит 1
  {13, 2, false, "[HIGH_B] "}, // Бит 2
  {15, 3, false, "[OIL!!!] "}, // Бит 3
  {17, 4, false, "[CHECK!] "}  // Бит 4
};

// КАДР 102: Комфорт / Двери / Ручник (Интервал: 2000 мс)
const int LAMPS_COUNT_102 = 3;
InputChannel indicators102[LAMPS_COUNT_102] = {
  {23,  0, false, "[REAR_DEF] "}, // Обогрев стекла (Бит 0)
  {25, 1, false, "[DOOR_OPN] "}, // Открытая дверь (Бит 1)
  {27, 2, false, "[HANDBRK ] "}  // Ручник (Бит 2)
};

// КАДР 100: Резервный массив (пока пустой, размер 0)
const int LAMPS_COUNT_100 = 0;
// InputChannel indicators100[LAMPS_COUNT_100] = {};

// ==========================================================================
// --- ПЕРЕМЕННЫЕ И ТАЙМИНГИ ---
// ==========================================================================
const unsigned long TIMEOUT_WAIT_IGNITION = 300000; // Время ожидания зажигания (5 минут)
const unsigned long WAKE_FILTER_DELAY     = 1000;   // Фильтр сигнала ЦЗ (1 секунда)
const float CRITICAL_BATTERY_VOLTAGE      = 11.9;    // Порог защиты аккумулятора от разряда (Вольты)

volatile unsigned long rpmPulses = 0;
volatile unsigned long speedPulses = 0;
//unsigned long lastUpdateTime = 0;

unsigned long timerFastSensors = 0;
unsigned long timerNormSensors = 0;
unsigned long timerSlowSensors = 0;

const unsigned long INTERVAL_FAST = 50;   // 50 мс
const unsigned long INTERVAL_NORM = 200;  // 200 мс (Скорость, RPM, Кадр 101)
const unsigned long INTERVAL_SLOW = 2000; // 2000 мс (Вольтметр, ДТОЖ, ДУТ, Кадр 102)

// Коэффициенты под Nissan Almera G15
const int pulsesPerTurnRPM = 2;        
const float pulsesPerKmSpeed = 6000.0; 
const float dividerRatio = 4.0303;     // (10кОм + 3.3кОм) / 3.3кОм

// Глобальные переменные данных
uint16_t currentRPM = 0;
uint16_t currentSpeed = 0;
float batteryVoltage = 0;
uint16_t rawECT = 0;
uint16_t rawFuel = 0;
// ==========================================================================
// --- СОСТОЯНИЕ СИСТЕМЫ ---
// ==========================================================================
enum SystemState {
  STATE_SLEEP,
  STATE_PRE_DRIVE_WAKE,
  STATE_DRIVE,
  STATE_SHUTDOWN
};

SystemState currentState = STATE_SLEEP;
unsigned long wakeUpTimerStart = 0;

// ==========================================================================
// --- СТРУКТУРА И ФУНКЦИЯ ОТПРАВКИ REALDASH CAN ---
// ==========================================================================
union RealDashFrame {
  struct {
    uint32_t canId;
    uint8_t data[8];
  } __attribute__((packed)) frame;
  uint8_t bytes[12]; // Полный размер кадра (4 байта ID + 8 байт данных)
};

void sendRealDashFrame(uint32_t canId, uint8_t* data8Bytes) {
  const uint8_t serialBlockHeader[4] = { 0x44, 0x33, 0x22, 0x11 };
  Serial.write(serialBlockHeader, 4);
  
  RealDashFrame myFrame;
  myFrame.frame.canId = canId;
  memcpy(myFrame.frame.data, data8Bytes, 8);
  
  Serial.write(myFrame.bytes, 12);
}

// Функция точного измерения напряжения аккумулятора
float readBatteryVoltage() {
  // 1. ХОЛОСТОЙ ВЫСТРЕЛ (Dummy Read): переключаем АЦП на пин вольтметра
  // и даем зарядиться внутреннему конденсатору. Результат просто выбрасываем.
  analogRead(PIN_BATTERY_SENSE); 
  
  // 2. Основной цикл сбора данных (уже чистые показания)
  int rawSum = 0;
  for (int i = 0; i < 10; i++) {
    rawSum += analogRead(PIN_BATTERY_SENSE);
  }
  
  float averageRaw = (float)rawSum / 10.0;
  float vPin = (averageRaw * 5.0) / 1023.0; 
  return vPin * 4.0303; 
}
// --- ФУНКЦИИ ОБРАБОТКИ ПРЕРЫВАНИЙ ---
void rpmPulseCounter() {
  rpmPulses++;
}

void speedPulseCounter() {
  speedPulses++;
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

  
  // Настройка прерываний скорости и оборотов
  // Важно: для работы схемы с PC817 с внешними резисторами на стороне 5V
  pinMode(PIN_SPEED, INPUT);
  pinMode(PIN_RPM, INPUT);
  // Прерывание срабатывает, когда транзистор в PC817 открывается и прижимает пин к GND
  attachInterrupt(digitalPinToInterrupt(PIN_RPM), rpmPulseCounter, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_SPEED), speedPulseCounter, FALLING);
  
  // Автоматический перебор пинов ламп из массивов кадра 101 и 102
  for (int i = 0; i < LAMPS_COUNT_101; i++) pinMode(indicators101[i].pin, INPUT_PULLUP);
  for (int i = 0; i < LAMPS_COUNT_102; i++) pinMode(indicators102[i].pin, INPUT_PULLUP);

  // Для RealDash ставится 115200. Для тестов в Мониторе Порта оставляем 9600
  Serial.begin(9600); 
  Serial.println(F("=== ПРОВЕРЯЕМ БАТАРЕЮ И КОЛИЧЕСТВО НАЖАТИЙ ЦЗ ==="));
  
  // 2. ЭКСПРЕСС-ДИАГНОСТИКА АКБ
  float currentVoltage = readBatteryVoltage();
  Serial.print(F("Батарея: ")); Serial.print(currentVoltage); Serial.println(F("V"));
  
  if (currentVoltage < CRITICAL_BATTERY_VOLTAGE) {
    Serial.println(F("Батарея разряжена. Не включаем питание вообще"));
    currentState = STATE_SHUTDOWN;
    return;
  }
  
  // 3. ФИЛЬТР НАЖАТИЙ НА ЦЗ
  Serial.println(F("Проверяем нажатие на брелок ЦЗ..."));
  unsigned long filterStart = millis();
  bool realWakeUpDetected = false;
  static byte doorCounter = 0; // Счетчик нажатий
  bool lastDoorState = HIGH;

  while (millis() - filterStart < WAKE_FILTER_DELAY) {
    // Если в течение секунды человек включил зажигание — это точно не ложный сигнал
    if (digitalRead(PIN_IGNITION) == LOW) { 
      realWakeUpDetected = true; 
      break; 
    }
    bool currentDoorState = digitalRead(PIN_DOOR_TRIGGER);
    if (currentDoorState == LOW && lastDoorState == HIGH) {
      doorCounter++; 
      Serial.print(F("Количество нажатий брелка ЦЗ: ")); Serial.println(doorCounter);
      delay(50);
    }// Считаем нажатия
    Serial.print(F("Общее: ")); Serial.println(doorCounter);
    lastDoorState = currentDoorState;
  }
   // Если не нажали, будем заводить. Если нажали еще один и более раз, заводить не будем
  if (doorCounter == 0) {
      realWakeUpDetected = true; 
  }
  if (!realWakeUpDetected) {
    Serial.println(F("Нажали на открытие 2 и более раз. Идем спать дальше..."));
    currentState = STATE_SHUTDOWN;
  } else {
    // Сигнал подтвержден, хозяин открыл машину или завел её
    Serial.println(F("Нажали 1 раз (или завели авто). Включаем питание Андроид..."));
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
      // uint16_t rpm = 2500; // Сюда опрос датчика
      // uint16_t speed = 60;  // Скорость км/ч
      
      // buffer[0] = lowByte(rpm);
      // buffer[1] = highByte(rpm);
      // buffer[2] = lowByte(speed);
      // buffer[3] = highByte(speed);
      // buffer[4] = 0; buffer[5] = 0; buffer[6] = 0; buffer[7] = 0; // Остальные зануляем
      
      // sendRealDashFrame(100, buffer); // Отправляем кадр 100
    }

    // 2. СТАНДАРТНЫЕ ДАТЧИКИ (CAN ID: 101, каждые 200 мс)
    if (currentMillis - timerNormSensors >= INTERVAL_NORM) {
      // Вычисляем точное время, прошедшее с момента последнего расчета
      unsigned long timeElapsed = currentMillis - timerNormSensors;
      timerNormSensors = currentMillis;

      // Блокируем прерывания на доли микросекунды, чтобы безопасно скопировать данные
      noInterrupts();
      unsigned long localRpmPulses = rpmPulses;
      unsigned long localSpeedPulses = speedPulses;
      rpmPulses = 0;     // Сбрасываем счетчик импульсов тахометра
      speedPulses = 0;   // Сбрасываем счетчик импульсов скорости
      interrupts();      // Снова разрешаем прерывания

      // Расчет физических величин
    float calcRPM = ((float)localRpmPulses / pulsesPerTurnRPM) * (60000.0 / timeElapsed);
    currentRPM = (calcRPM < 0) ? 0 : (uint16_t)calcRPM;

    float calcSpeed = ((float)localSpeedPulses / pulsesPerKmSpeed) / ((float)timeElapsed / 3600000.0);
    currentSpeed = (calcSpeed < 1.0) ? 0 : (uint16_t)calcSpeed;

    // --- Сборка байта ламп для Кадра 100 ---
    uint8_t byteFrame101 = 0;
    String textLamps101 = ""; // Текст для Монитора порта

    for (int i = 0; i < LAMPS_COUNT_101; i++) {
      indicators101[i].currentState = (digitalRead(indicators101[i].pin) == LOW);
      if (indicators101[i].currentState) {
        byteFrame101 |= (1 << indicators101[i].bitPosition);
        textLamps101 += indicators101[i].name;
      }
    }
     // --- Подготовка пакета данных для Кадра 101 ---
    uint8_t data101[8] = {0};
    data101[0] = lowByte(currentRPM);
    data101[1] = highByte(currentRPM);
    data101[2] = lowByte(currentSpeed);
    data101[3] = highByte(currentSpeed);
    data101[4] = byteFrame101; // Байт ламп

    // Отправка в RealDash (ВРЕМЕННО ЗАКОММЕНТИРОВАНО ДЛЯ ТЕСТА В МОНИТОРЕ ПОРТА)
    // sendRealDashFrame(100, data100);

    // Вывод быстрых данных в Монитор порта
    Serial.print("RPM: "); Serial.print(currentRPM);
    Serial.print(" | SPD: "); Serial.print(currentSpeed, 1);
    Serial.print(" km/h | LAMPS101: "); Serial.println(textLamps101);
    }

    // 3. МЕДЛЕННЫЕ ДАТЧИКИ (CAN ID: 102, каждые 2 секунды)
    if (currentMillis - timerSlowSensors >= INTERVAL_SLOW) {
      timerSlowSensors = currentMillis;
      
      // Чтение вольтметра
      batteryVoltage = readBatteryVoltage(); 
      uint8_t voltPacked = (uint8_t)(batteryVoltage * 10.0); // 13.8V -> 138

      // Чтение сырого АЦП ДТОЖ (с защитой мультиплексора)
      analogRead(PIN_ECT); 
      int rawSumTemp = 0;
      for(int i = 0; i < 10; i++) rawSumTemp += analogRead(PIN_ECT);
      rawECT = rawSumTemp / 10;

      // Чтение сырого АЦП ДУТ (с защитой мультиплексора)
      analogRead(PIN_FUEL); 
      int rawSumFuel = 0;
      for(int i = 0; i < 10; i++) rawSumFuel += analogRead(PIN_FUEL);
      rawFuel = rawSumFuel / 10;

      // --- Сборка байта ламп для Кадра 102 ---
      uint8_t byteFrame102 = 0;
      String textLamps102 = "";

      for (int i = 0; i < LAMPS_COUNT_102; i++) {
        indicators102[i].currentState = (digitalRead(indicators102[i].pin) == LOW);
        if (indicators102[i].currentState) {
          byteFrame102 |= (1 << indicators102[i].bitPosition);
          textLamps102 += indicators102[i].name;
        }
      }

      // --- Подготовка пакета данных для Кадра 102 ---
      uint8_t data102[8] = {0};
      data102[0] = byteFrame102;       // Байт дверей/ручника
      data102[1] = lowByte(rawECT);    // ДТОЖ (младший)
      data102[2] = highByte(rawECT);   // ДТОЖ (старший)
      data102[3] = lowByte(rawFuel);   // ДУТ (младший)
      data102[4] = highByte(rawFuel);  // ДУТ (старший)
      data102[5] = voltPacked;

      // Отправка в RealDash (ВРЕМЕННО ЗАКОММЕНТИРОВАНО ДЛЯ ТЕСТА В МОНИТОРЕ ПОРТА)
      // sendRealDashFrame(102, data102);


      // Вывод медленных данных в Монитор порта
      Serial.println("----------------------------------------------");
      Serial.print("  [ANALOG] VOLTAGE: "); Serial.print(batteryVoltage, 2); Serial.println(" V");
      Serial.print("  [ANALOG] ДТОЖ ADC: "); Serial.print(rawECT); 
      Serial.print(" | ДУТ ADC: "); Serial.println(rawFuel);
      Serial.print("  [LAMPS102]: "); Serial.println(textLamps102);
      Serial.println("----------------------------------------------");
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
        Serial.println(F("Состояние изменено на DRIVE. Наслаждайтесь поездкой"));
      } 
      else if (millis() - wakeUpTimerStart >= TIMEOUT_WAIT_IGNITION) {
        currentState = STATE_SHUTDOWN;
        Serial.println(F("Состояние изменено на SHUTDOWN. Вышло время"));
      }
      break;

    case STATE_DRIVE:
      // В режиме поездки нам плевать на любые клацанья ЦЗ или дверей. Мы смотрим только на зажигание.
      if (!isIgnitionOn) {
        currentState = STATE_SHUTDOWN;
        Serial.println(F("Состояние изменено на SHUTDOWN. МАшина выключена"));
      }
      break;

    case STATE_SHUTDOWN:
      Serial.println(F("Выключаем питание Андроид..."));
      digitalWrite(PIN_ACC_OUTPUT, LOW); // Обесточиваем магнитолу через BTS442
      delay(500); // Короткая пауза для записи кэша магнитолы
      
      Serial.println(F("Выключаем саму панель. Пока!"));
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
//     
//   </frame>

//   <!-- Кадр 101: Обычные датчики (200 мс) -->
//   <frame id="101">
//     <value targetId="37" offset="0" length="2" units="RPM"></value> <!-- Обороты (длина 2 байта) -->
//     <value targetId="33" offset="2" length="2" units="km/h"></value> <!-- Скорость (длина 2 байта) -->
//     <!-- Напряжение АКБ: берем 1 байт, делим обратно на 10 (range="0,25.5") -->
//     <value targetId="12" offset="0" length="1" conversion="V/10"></value> 
//     <value targetId="16" offset="1" length="1" units="%"></value> <!-- Топливо -->
// <!-- Лампы: берем offset="4" (5-й байт) и читаем побитово -->
//       <value name="Inidicator: Turn Left" offset="4" length="1" bit="0"></value>
//       <value name="Indicator: Turn Right" offset="4" length="1" bit="1"></value>
//       <value name="Indicator: High Beam" offset="4" length="1" bit="2"></value>
//       <value name="Indicator: Oil Pressure" offset="4" length="1" bit="3"></value>
//       <value name="Indicator: Check Engine" offset="4" length="1" bit="4"></value>
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
