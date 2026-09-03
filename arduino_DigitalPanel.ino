#include <Arduino.h>
// =========================================================================
// АВТОМОБИЛЬНЫЙ КОНТРОЛЛЕР ПИТАНИЯ И ДАТЧИКОВ (REALDASH CAN ВЕРСИЯ)
// Arduino Mega Pro. Посекундные таймеры + Бинарный протокол RealDash CAN
// =========================================================================                                                                                  

// ==========================================================================
// --- КОНФИГУРАЦИЯ ПИНОВ ---
// ==========================================================================

const byte PIN_FUEL = A0;           // ДУТ (резистор 330 Ом)
const byte PIN_ECT = A1;            // ДТОЖ (резистор 330 Ом)
const byte PIN_BATTERY_SENSE = A2;  // Вольтметр (делитель 10кОм и 3.3кОм)
const byte PIN_RPM          = 2;    // Вход RPM через PC817 (Прерывание 0)
const byte PIN_SPEED        = 3;    // Вход Скорости через PC817 (Прерывание 1)
const byte PIN_HOLD_POWER   = 4;    // Цифровой выход: удержание питания схемы (на базу BC547B)
const byte PIN_ACC_OUTPUT   = 5;    // Цифровой выход: управление ключом BTS442 (ACC магнитолы)
const byte PIN_DOOR_TRIGGER = 6;    // Цифровой вход (через оптопару): сигнал ЦЗ
const byte PIN_IGNITION     = 7;    // Цифровой вход (через оптопару): Зажигание (Клемма 15)

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

const int LAMPS_COUNT_101 = 8;
InputChannel indicators101[LAMPS_COUNT_101] = {
  {9, 0, false, "[PNP_1] "},
  {11, 1, false, "[PNP_2] "},
  {15, 2, false, "[PNP_3] "},
  {23, 3, false, "[PNP_4] "},
  {25, 4, false, "[PNP_5] "},
  {27, 5, false, "[PNP_6] "},
  {29, 6, false, "[PNP_7] "},
  {31, 7, false, "[PNP_8] "}
};

const int LAMPS_COUNT_102 = 7;
InputChannel indicators102[LAMPS_COUNT_102] = {
  {32, 0, false, "[NPN_1] "}, 
  {34, 1, false, "[NPN_2] "}, 
  {36, 2, false, "[NPN_3] "}, 
  {38, 3, false, "[NPN_4] "}, 
  {40, 4, false, "[NPN_5] "},
  {42, 5, false, "[NPN_6] "},
  {44, 6, false, "[NPN_7] "},
  };

// КАДР 100: Резервный массив (пока пустой, размер 0)
const int LAMPS_COUNT_100 = 0;
// InputChannel indicators100[LAMPS_COUNT_100] = {};

// ==========================================================================
// --- ПЕРЕМЕННЫЕ И ТАЙМИНГИ ---
// ==========================================================================
const unsigned long TIMEOUT_WAIT_IGNITION = 300000; // Время ожидания зажигания (5 минут)
const unsigned long WAKE_FILTER_DELAY     = 1000;   // Фильтр сигнала ЦЗ (1 секунда)
const float CRITICAL_BATTERY_VOLTAGE      = 10.0;    // Порог защиты аккумулятора от разряда (Вольты)

volatile unsigned long rpmPulses = 0;
volatile unsigned long speedPulses = 0;
//unsigned long lastUpdateTime = 0;

unsigned long timerFastSensors = 0;
unsigned long timerNormSensors = 0;
unsigned long timerSlowSensors = 0;

const unsigned long INTERVAL_FAST = 50;   // 50 мс
const unsigned long INTERVAL_NORM = 1000;  // 200 мс (Скорость, RPM, Кадр 101)
const unsigned long INTERVAL_SLOW = 2000; // 2000 мс (Вольтметр, ДТОЖ, ДУТ, Кадр 102)

// Коэффициенты под Nissan Almera G15
const int pulsesPerTurnRPM = 2;        
const float pulsesPerKmSpeed = 6000.0; 
const float dividerRatio = 4.0523;     // (9,92кОм + 3.25кОм) / 3.25кОм

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
  Serial2.write(serialBlockHeader, 4);
  
  RealDashFrame myFrame;
  myFrame.frame.canId = canId;
  memcpy(myFrame.frame.data, data8Bytes, 8);
  
  Serial2.write(myFrame.bytes, 12);
}

// Функция точного измерения напряжения аккумулятора
float readBatteryVoltage(int counter) {
  // 1. ХОЛОСТОЙ ВЫСТРЕЛ (Dummy Read): переключаем АЦП на пин вольтметра
  // и даем зарядиться внутреннему конденсатору. Результат просто выбрасываем.
  analogRead(PIN_BATTERY_SENSE); 
  
  // 2. Основной цикл сбора данных (уже чистые показания)
  int rawSum = 0;
  for (int i = 0; i < counter; i++) {
    rawSum += analogRead(PIN_BATTERY_SENSE);
  }
  
  float averageRaw = (float)rawSum / (float)counter;
  float vPin = (averageRaw * 5.0) / 1023.0; 
  return vPin * dividerRatio; 
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
  // Для RealDash ставится 115200. Для тестов в Мониторе Порта 2 оставляем 9600
  // Serial2.begin(115200); 
  Serial2.begin(9600); 
  pinMode(17, INPUT_PULLUP); // Подтяжка RX линии Serial2

  while(!Serial2); // Ожидание открытия Монитора порта 2
  Serial2.println(F("============ ЗАГРУЗКА СИСТЕМЫ ================"));
  // Инициализация остальных пинов
  pinMode(PIN_ACC_OUTPUT, OUTPUT);
  digitalWrite(PIN_ACC_OUTPUT, LOW); // Магнитола пока строго выключена
  
  pinMode(PIN_DOOR_TRIGGER, INPUT_PULLUP); // Используем подтяжку для оптопары
  pinMode(PIN_IGNITION, INPUT_PULLUP);

  // pinMode(PIN_ECT, INPUT_PULLUP); // Для ДТОЖ
  // pinMode(PIN_FUEL, INPUT_PULLUP); // Для ДУТ
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

  Serial2.println(F("=== ПРОВЕРЯЕМ БАТАРЕЮ И КОЛИЧЕСТВО НАЖАТИЙ ЦЗ ==="));
  
  // 2. ЭКСПРЕСС-ДИАГНОСТИКА АКБ
  float currentVoltage = readBatteryVoltage(3);
  
  Serial2.print(F("Батарея: ")); Serial2.print(currentVoltage); Serial2.println(F("V"));
  
  if (currentVoltage < CRITICAL_BATTERY_VOLTAGE) {
    Serial2.println(F("Батарея разряжена. Не включаем питание вообще"));
    currentState = STATE_SHUTDOWN;
    return;
  }
  
  // 3. ФИЛЬТР НАЖАТИЙ НА ЦЗ
  Serial2.println(F("Проверяем нажатие на брелок ЦЗ..."));
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
      Serial2.print(F("Количество нажатий брелка ЦЗ: ")); Serial2.println(doorCounter);
      delay(50);
    }// Считаем нажатия
    // Serial2.print(F("Общее: ")); Serial2.println(doorCounter);
    lastDoorState = currentDoorState;
  }
  Serial2.print(F("Общее количество: ")); Serial2.println(doorCounter);
  // Первый раз нажали - включили пин
  // Если второй раз не нажали, будем заводить. Если нажали еще один и более раз, заводить не будем
  if (doorCounter == 0) {
      realWakeUpDetected = true; 
  }
  if (!realWakeUpDetected) {
    Serial2.println(F("Нажали на открытие 2 и более раз. Идем спать дальше..."));
    currentState = STATE_SHUTDOWN;
  } else {
    // Сигнал подтвержден, хозяин открыл машину или завел её
    Serial2.println(F("Нажали 1 раз (или завели авто). Включаем питание Андроид..."));
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
    //uint8_t buffer[8]; // Временный буфер для упаковки байт

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

      // --- Сборка байта ламп для Кадра 101 ---
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
      // sendRealDashFrame(101, data101);

      // Вывод быстрых данных в Монитор порта
      Serial2.print(" RPM: "); Serial2.print(currentRPM);
      Serial2.print(" | SPD: "); Serial2.print(currentSpeed, 1); Serial2.println(" km/h");
      Serial2.print(" LAMPS101: "); Serial2.println(textLamps101);
    }

    // 3. МЕДЛЕННЫЕ ДАТЧИКИ (CAN ID: 102, каждые 2 секунды)
    if (currentMillis - timerSlowSensors >= INTERVAL_SLOW) {
      timerSlowSensors = currentMillis;
      
      // Чтение вольтметра
      batteryVoltage = readBatteryVoltage(10); 
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
      Serial2.println("----------------------------------------------");
      Serial2.print(" VOLTAGE: "); Serial2.print(batteryVoltage, 2); Serial2.print(" V");
      Serial2.print(" | ДТОЖ ADC: "); Serial2.print(rawECT); 
      Serial2.print(" | ДУТ ADC: "); Serial2.println(rawFuel);
      Serial2.print(" LAMPS102: "); Serial2.println(textLamps102);
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
        Serial2.println(F("Состояние изменено на DRIVE. Наслаждайтесь поездкой"));
      } 
      else if (millis() - wakeUpTimerStart >= TIMEOUT_WAIT_IGNITION) {
        currentState = STATE_SHUTDOWN;
        Serial2.println(F("Состояние изменено на SHUTDOWN. Вышло время"));
      }
      break;

    case STATE_DRIVE:
      // В режиме поездки нам плевать на любые клацанья ЦЗ или дверей. Мы смотрим только на зажигание.
      if (!isIgnitionOn) {
        currentState = STATE_SHUTDOWN;
        Serial2.println(F("Состояние изменено на SHUTDOWN. МАшина выключена"));
      }
      break;

    case STATE_SHUTDOWN:
      Serial2.println(F("Выключаем питание Андроид..."));
      digitalWrite(PIN_ACC_OUTPUT, LOW); // Обесточиваем магнитолу через BTS442
      delay(500); // Короткая пауза для записи кэша магнитолы
      
      Serial2.println(F("Выключаем саму панель. Пока!"));
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
// <appIdata version="1">
//   <!-- Протокол RealDash CAN для Nissan Almera G15 -->
//   <channels>
//     <!-- Кадр 3200: Обороты и Скорость -->
//     <channel id="3200" name="Nissan: RPM" units="RPM" type="value" dataType="uint16"></channel>
//     <channel id="3200" offset="2" name="Nissan: Speed" units="km/h" type="value" dataType="uint16"></channel>

//     <!-- Кадр 3201: Вольтметр и ДТОЖ -->
//     <!-- Делим на 100 в RealDash, так как передаем целое число (1226 вместо 12.26) -->
//     <channel id="3201" name="Nissan: Battery Voltage" units="V" type="value" dataType="uint16" conversion="V/100"></channel>
//     <channel id="3201" offset="2" name="Nissan: ECT ADC" units="raw" type="value" dataType="uint16"></channel>

//     <!-- Кадр 3202: ДУТ и Дискретные статусы -->
//     <channel id="3202" name="Nissan: Fuel ADC" units="raw" type="value" dataType="uint16"></channel>
//     <!-- Битовые маски для зажигания и дверей -->
//     <channel id="3202" offset="2" name="Nissan: Ignition Status" type="bit" bitIndex="0"></channel>
//     <channel id="3202" offset="2" name="Nissan: Door Trigger" type="bit" bitIndex="1"></channel>

//     <!-- Кадр 3203: Лампы индикации (Байт 101 и Байт 102) -->
//     <!-- Группа 101 (PNP ключи) -->
//     <channel id="3203" name="Indicator: PNP 1" type="bit" bitIndex="0"></channel>
//     <channel id="3203" name="Indicator: PNP 2" type="bit" bitIndex="1"></channel>
//     <channel id="3203" name="Indicator: PNP 3" type="bit" bitIndex="2"></channel>
//     <channel id="3203" name="Indicator: PNP 4" type="bit" bitIndex="3"></channel>
//     <channel id="3203" name="Indicator: PNP 5" type="bit" bitIndex="4"></channel>
//     <channel id="3203" name="Indicator: PNP 6" type="bit" bitIndex="5"></channel>
//     <channel id="3203" name="Indicator: PNP 7" type="bit" bitIndex="6"></channel>
//     <channel id="3203" name="Indicator: PNP 8" type="bit" bitIndex="7"></channel>

//     <!-- Группа 102 (NPN ключи, смещение на 1 байт дальше в камере) -->
//     <channel id="3203" offset="1" name="Indicator: NPN 1" type="bit" bitIndex="0"></channel>
//     <channel id="3203" offset="1" name="Indicator: NPN 2" type="bit" bitIndex="1"></channel>
//     <channel id="3203" offset="1" name="Indicator: NPN 3" type="bit" bitIndex="2"></channel>
//     <channel id="3203" offset="1" name="Indicator: NPN 4" type="bit" bitIndex="3"></channel>
//     <channel id="3203" offset="1" name="Indicator: NPN 5" type="bit" bitIndex="4"></channel>
//     <channel id="3203" offset="1" name="Indicator: NPN 6" type="bit" bitIndex="5"></channel>
//     <channel id="3203" offset="1" name="Indicator: NPN 7" type="bit" bitIndex="6"></channel>

//     <!-- Кадр 3204: Мониторинг управляющих выходов -->
//     <channel id="3204" name="Status: Power Hold" type="bit" bitIndex="0"></channel>
//     <channel id="3204" name="Status: ACC Output" type="bit" bitIndex="1"></channel>
//   </channels>
// </appIdata>


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
