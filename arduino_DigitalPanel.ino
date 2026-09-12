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

// LAMPS 1: Стандартные лампы
const int LAMPS_COUNT_1 = 8;
InputChannel indicators1[LAMPS_COUNT_1] = {
  {9, 0, false, "[Левый поворотник] "},
  {11, 1, false, "[Правый поворотник] "},
  {10, 2, false, "[Дальний свет] "},
  {23, 3, false, "[Ближний свет] "},
  {25, 4, false, "[Передние ПТФ] "},
  {27, 5, false, "[Задний ПТФ] "},
  {29, 6, false, "[Габариты] "},
  {31, 7, false, "[P_10] "}
};
// LAMPS 2: Стандартные лампы
const int LAMPS_COUNT_2 = 8;
InputChannel indicators2[LAMPS_COUNT_2] = {
  {32, 0, false, "[Давление масла] "}, 
  {34, 1, false, "[Ручник] "}, 
  {36, 2, false, "[Check Engine] "}, 
  {38, 3, false, "[Аккумулятор] "}, 
  {40, 4, false, "[ABS] "},
  {42, 5, false, "[Перегрев ОЖ] "},
  {44, 6, false, "[Ремень безопасности] "},
  {33, 7, false, "[Двери открыты] "}
};
// LAMPS 3: Стандартные лампы
const int LAMPS_COUNT_3 = 3;
InputChannel indicators3[LAMPS_COUNT_3] = {
  {35, 0, false, "[Подушки безопасности] "}, 
  {37, 1, false, "[Иммобилайзер] "}, 
  {39, 2, false, "[Пила] "}
};

// ==========================================================================
// --- ПЕРЕМЕННЫЕ И ТАЙМИНГИ ---
// ==========================================================================
const unsigned long TIMEOUT_WAIT_IGNITION = 12000; // Время ожидания зажигания
const unsigned long WAKE_FILTER_DELAY     = 4000;   // Фильтр сигнала ЦЗ
const float CRITICAL_BATTERY_VOLTAGE      = 10.0;    // Порог защиты аккумулятора от разряда (Вольты)

volatile unsigned long rpmPulses = 0;
volatile unsigned long speedPulses = 0;
//unsigned long lastUpdateTime = 0;

unsigned long timerFastSensors = 0;
unsigned long timerNormSensors = 0;
unsigned long timerSlowSensors = 0;

const unsigned long INTERVAL_FAST = 500;   // 500 мс
const unsigned long INTERVAL_NORM = 1000;  // 1000 мс (Скорость, RPM, Кадр 101)
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
uint8_t byteFrame1 = 0;
uint8_t byteFrame2 = 0;
uint8_t byteFrame3 = 0;
// ==========================================================================
// --- СОСТОЯНИЕ СИСТЕМЫ ---
// ==========================================================================
enum SystemState {
  STATE_PRE_DRIVE_WAKE,
  STATE_DRIVE,
  STATE_SHUTDOWN
};

SystemState currentState = STATE_PRE_DRIVE_WAKE;
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
// Временно для тестов в Мониторе порта
String createIndicatorsText (int counterIndicators, InputChannel indicators[]) { 
  String textLamps = ""; // Текст для Монитора порта
  for (int i = 0; i < counterIndicators; i++) {
    indicators[i].currentState = (digitalRead(indicators[i].pin) == LOW);
    if (indicators[i].currentState) {
      textLamps += indicators[i].name;
    }
  }
  return textLamps;
}
uint8_t createIndicatorsByte (int counterIndicators, InputChannel indicators[]) { 
  uint8_t byteFrame = 0;
  for (int i = 0; i < counterIndicators; i++) {
    indicators[i].currentState = (digitalRead(indicators[i].pin) == LOW);
    if (indicators[i].currentState) {
      byteFrame |= (1 << indicators[i].bitPosition);
    }
  }
  return byteFrame;
}
void setup() {
  // 1. МГНОВЕННО захватываем питание платы, пока не исчез физический импульс от двери!
  pinMode(PIN_HOLD_POWER, OUTPUT);
  digitalWrite(PIN_HOLD_POWER, HIGH); 
  // Для RealDash ставится 115200. Для тестов в Мониторе Порта 2 оставляем 9600
  // Serial2.begin(115200); 
  
  Serial2.begin(9600); 
  pinMode(17, INPUT_PULLUP); // Подтяжка RX линии Serial2

  // Отправка в RealDash (ВРЕМЕННО ЗАКОММЕНТИРОВАНО ДЛЯ ТЕСТА В МОНИТОРЕ ПОРТА)
  // sendRealDashFrame(104, { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 });

  //while(!Serial2); // Ожидание открытия Монитора порта 2
  Serial2.println(F("============ ЗАГРУЗКА СИСТЕМЫ ================"));
  // Инициализация остальных пинов

  pinMode(PIN_ACC_OUTPUT, OUTPUT);
  digitalWrite(PIN_ACC_OUTPUT, LOW); // Магнитола пока строго выключена
  
  pinMode(PIN_DOOR_TRIGGER, INPUT_PULLUP); // Используем подтяжку для оптопары
  pinMode(PIN_IGNITION, INPUT_PULLUP);

  
  // 2. ЭКСПРЕСС-ДИАГНОСТИКА АКБ
  Serial2.println(F("============ ПРОВЕРЯЕМ БАТАРЕЮ ==============="));
  float startVolt = readBatteryVoltage(3);
  Serial2.print(F("-------- Батарея: ")); Serial2.print(startVolt); Serial2.println(F("V--------"));
  if (startVolt < CRITICAL_BATTERY_VOLTAGE) {
    Serial2.println(F("=========Батарея разряжена. Не включаем питание вообще======"));
    currentState = STATE_SHUTDOWN;
    return;
  }
  
  // 3. ФИЛЬТР НАЖАТИЙ НА ЦЗ
  Serial2.println(F("======= ПРОВЕРЯЕМ НАЖАТИЕ НА БРЕЛОК ЦЗ ======="));
  unsigned long filterStart = millis();
  bool realWakeUpDetected = false;
  int doorCounter = 0; // Счетчик нажатий
  bool lastDoorState = HIGH;

  while (millis() - filterStart < WAKE_FILTER_DELAY) {
    // Если в течение 2 сек включили зажигание — это точно не ложный сигнал
    if (digitalRead(PIN_IGNITION) == LOW) { 
      realWakeUpDetected = true; 
      break; 
    }
    bool currentDoorState = digitalRead(PIN_DOOR_TRIGGER);
    if (currentDoorState == LOW && lastDoorState == HIGH) {
      doorCounter++; 
      Serial2.print(F("--------Количество нажатий брелка ЦЗ: ")); Serial2.println(doorCounter);
    }
   lastDoorState = currentDoorState; 
  }
  Serial2.print(F("---- Общее количество: ")); Serial2.print(doorCounter);  Serial2.println(F("---------"));
  // Отправка в RealDash (ВРЕМЕННО ЗАКОММЕНТИРОВАНО ДЛЯ ТЕСТА В МОНИТОРЕ ПОРТА)
  uint8_t data103[8] = {0};
  data103[0] = (uint8_t)(startVolt * 10.0); // 13.8V -> 138;
  data103[1] = (uint8_t)(doorCounter);
  // sendRealDashFrame(103, data103);

  // Первый раз нажали - включили пин
  // Если второй раз не нажали, будем заводить. Если нажали еще один и более раз, заводить не будем
  if (doorCounter == 0) {
      realWakeUpDetected = true; 
  }
  if (!realWakeUpDetected) {
    Serial2.println(F("=======Нажали на открытие 2 и более раз. Идем спать дальше...========"));
    currentState = STATE_SHUTDOWN;
    return;
  } else {
    // Сигнал подтвержден, хозяин открыл машину или завел её
    Serial2.println(F("=======Нажали 1 раз (или завели авто). Включаем питание Андроид...======"));
    digitalWrite(PIN_ACC_OUTPUT, HIGH); // ВКЛЮЧАЕМ BTS442 (Магнитолу)
    wakeUpTimerStart = millis();
  }

  pinMode(PIN_ECT, INPUT); // Для ДТОЖ
  pinMode(PIN_FUEL, INPUT); // Для ДУТ
  // Настройка прерываний скорости и оборотов
  // Важно: для работы схемы с PC817 с внешними резисторами на стороне 5V
  pinMode(PIN_SPEED, INPUT);
  pinMode(PIN_RPM, INPUT);
  // Прерывание срабатывает, когда транзистор в PC817 открывается и прижимает пин к GND
  attachInterrupt(digitalPinToInterrupt(PIN_RPM), rpmPulseCounter, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_SPEED), speedPulseCounter, FALLING);
  
  // Автоматический перебор пинов ламп из массивов кадра 101
  for (int i = 0; i < LAMPS_COUNT_1; i++) pinMode(indicators1[i].pin, INPUT_PULLUP);
  for (int i = 0; i < LAMPS_COUNT_2; i++) pinMode(indicators2[i].pin, INPUT_PULLUP);
  for (int i = 0; i < LAMPS_COUNT_3; i++) pinMode(indicators3[i].pin, INPUT_PULLUP);
  
}

void loop() {
  // =========================================================================
  // БЛОК ОТПРАВКИ БИНАРНЫХ ДАННЫХ ПО ТАЙМЕРАМ
  // =========================================================================
  if (currentState == STATE_PRE_DRIVE_WAKE || currentState == STATE_DRIVE) {
    unsigned long currentMillis = millis();
    // 1. БЫСТРЫЕ ДАТЧИКИ (CAN ID: 100)
    if (currentMillis - timerFastSensors >= INTERVAL_FAST) {
      timerFastSensors = currentMillis;    
      
    }

    // 2. СТАНДАРТНЫЕ ДАТЧИКИ (CAN ID: 101)
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
      // --- Подготовка пакета данных для Кадра 101 ---
      uint8_t data101[8] = {0};
      data101[0] = lowByte(currentRPM);
      data101[1] = highByte(currentRPM);
      data101[2] = lowByte(currentSpeed);
      data101[3] = highByte(currentSpeed);
      data101[4] = createIndicatorsByte(LAMPS_COUNT_1, indicators1); // Байт ламп
      data101[5] = createIndicatorsByte(LAMPS_COUNT_2, indicators2); // Байт ламп
      data101[6] = createIndicatorsByte(LAMPS_COUNT_3, indicators3); // Байт ламп
      // Отправка в RealDash (ВРЕМЕННО ЗАКОММЕНТИРОВАНО ДЛЯ ТЕСТА В МОНИТОРЕ ПОРТА)
      // sendRealDashFrame(101, data101);

      // Вывод быстрых данных в Монитор порта
      Serial2.print(F(" RPM: ")); Serial2.print(currentRPM);
      Serial2.print(F(" | SPD: ")); Serial2.print(currentSpeed, 1); Serial2.println(F(" km/h"));
      Serial2.print(F(" LAMPS_1: ")); Serial2.println(createIndicatorsText(LAMPS_COUNT_1, indicators1));
      Serial2.print(F(" LAMPS_2: ")); Serial2.println(createIndicatorsText(LAMPS_COUNT_2, indicators2));
      Serial2.print(F(" LAMPS_3: ")); Serial2.println(createIndicatorsText(LAMPS_COUNT_3, indicators3));

      if (digitalRead(PIN_IGNITION) == LOW)     Serial2.println(F(" [ Зажигание ]")); 
      if (digitalRead(PIN_DOOR_TRIGGER) == LOW) Serial2.println(F(" [ ЦЗ ] "));
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
      // --- Подготовка пакета данных для Кадра 102 ---
      uint8_t data102[8] = {0};
      data102[0] = lowByte(rawECT);    // ДТОЖ (младший)
      data102[1] = highByte(rawECT);   // ДТОЖ (старший)
      data102[2] = lowByte(rawFuel);   // ДУТ (младший)
      data102[3] = highByte(rawFuel);  // ДУТ (старший)
      data102[4] = voltPacked;
      // Отправка в RealDash (ВРЕМЕННО ЗАКОММЕНТИРОВАНО ДЛЯ ТЕСТА В МОНИТОРЕ ПОРТА)
      // sendRealDashFrame(102, data102);


      // Вывод медленных данных в Монитор порта
      Serial2.println(F("----------------------------------------------"));
      Serial2.print(F(" VOLTAGE: ")); Serial2.print(batteryVoltage, 2); Serial2.print(F(" V"));
      Serial2.print(F(" | ДТОЖ ADC: ")); Serial2.print(rawECT); 
      Serial2.print(F(" | ДУТ ADC: ")); Serial2.println(rawFuel);
      Serial.println(F("----------------------------------------------"));
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
      // Отправка в RealDash (ВРЕМЕННО ЗАКОММЕНТИРОВАНО ДЛЯ ТЕСТА В МОНИТОРЕ ПОРТА)
      // sendRealDashFrame(104, { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 });
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
      // Отправка в RealDash (ВРЕМЕННО ЗАКОММЕНТИРОВАНО ДЛЯ ТЕСТА В МОНИТОРЕ ПОРТА)
      // sendRealDashFrame(104, { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 });
      // В режиме поездки нам плевать на любые клацанья ЦЗ или дверей. Мы смотрим только на зажигание.
      if (!isIgnitionOn) {
        currentState = STATE_SHUTDOWN;
        Serial2.println(F("Состояние изменено на SHUTDOWN. МАшина выключена"));
      }
      break;

    case STATE_SHUTDOWN:
      // Отправка в RealDash (ВРЕМЕННО ЗАКОММЕНТИРОВАНО ДЛЯ ТЕСТА В МОНИТОРЕ ПОРТА)
      // sendRealDashFrame(104, { 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 });
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
  }
}
// <?xml version="1.0" encoding="utf-8"?>
// <RealDashCAN version="2">
//   <frames>
//     <!-- Кадр 101: Обычные датчики и Сигналы по ПЛЮСУ (12V) и МИНУСУ -->
//     <frame id="101">
//       <value targetId="37" offset="0" length="2" units="RPM"></value>
//       <value targetId="33" offset="2" length="2" units="km/h"></value>
      
//       <!-- Лампы по ПЛЮСУ -->
//       <value targetId="163" offset="4" length="1" unit="bit" bit="0"></value> <!-- Левый поворотник -->
//       <value targetId="164" offset="4" length="1" unit="bit" bit="1"></value> <!-- Правый поворотник -->
//       <value targetId="157" offset="4" length="1" unit="bit" bit="2"></value> <!-- Дальний свет -->
//       <value targetId="156" offset="4" length="1" unit="bit" bit="3"></value> <!-- Ближний свет -->
//       <value targetId="159" offset="4" length="1" unit="bit" bit="4"></value> <!-- Передние ПТФ -->
//       <value targetId="160" offset="4" length="1" unit="bit" bit="5"></value> <!-- Задний ПТФ -->
//       <value targetId="158" offset="4" length="1" unit="bit" bit="6"></value> <!-- Габариты -->
//       <!-- Лампы по МИНУСУ (Часть 1) -->
//       <value targetId="153" offset="5" length="1" unit="bit" bit="0"></value> <!-- Давление масла -->
//       <value targetId="155" offset="5" length="1" unit="bit" bit="1"></value> <!-- Ручник / Тормозуха -->
//       <value targetId="151" offset="5" length="1" unit="bit" bit="2"></value> <!-- Check Engine -->
//       <value targetId="152" offset="5" length="1" unit="bit" bit="3"></value> <!-- Аккумулятор / Зарядка -->
//       <value targetId="273" offset="5" length="1" unit="bit" bit="4"></value> <!-- ABS -->
//       <value targetId="154" offset="5" length="1" unit="bit" bit="5"></value> <!-- Перегрев ОЖ -->
//       <value targetId="162" offset="5" length="1" unit="bit" bit="6"></value> <!-- Ремень безопасности -->
//       <value targetId="161" offset="5" length="1" unit="bit" bit="7"></value> <!-- Двери открыты (Общий) -->
//       <!-- Лампы по МИНУСУ (Часть 2) -->
//       <value targetId="150" offset="6" length="1" unit="bit" bit="0"></value> <!-- Подушки безопасности (SRS) -->
//       <value targetId="114" offset="6" length="1" unit="bit" bit="1"></value> <!-- Иммобилайзер (Красный диод) -->
//       <value name="Indicator: Saw (Electronics)" offset="6" length="1" unit="bit" bit="2"></value> <!-- Пила (Пользовательская) -->

//       <!-- Статус системы (занимает байт 1) -->
//       <value name="System_Status_Id" offset="7" length="1"></value>

//     </frame>

//     <!-- Кадр 102: Медленные датчики -->
//     <frame id="102">
//       <value targetId="14" offset="0" length="2" units="raw"></value>
//       <value targetId="16" offset="2" length="2" units="raw"></value>
//       <value targetId="12" offset="4" length="1" conversion="V/10"></value>     
//     </frame>

//     <!-- Кадр 103: Проверка при старте -->
//     <frame id="103">
//       <value name="Start_Voltage" offset="0" length="1" conversion="V/10"></value> 
//       <value name="Start_Clicks" offset="1" length="2"></value>                    
//     </frame>
//   </frames>
// </RealDashCAN>

// Примечание: Параметр targetId — это внутренний уникальный номер датчика в экосистеме RealDash 
// (например, 37 — это всегда RPM, а 12 — вольтаж батареи). Полный список этих ID есть на официальном сайте RealDash.



// void sendDataToRealDash() {
//   // 1. Создаем структуру буфера под наши 5 кадров (всего 18 байт данных)
//   uint8_t serialBlock[22]; // 4 байта заголовка + 18 байт данных

//   // 2. Запись стартового заголовка RealDash CAN (4 байта)
//   serialBlock[0] = 0x44; // 'D'
//   serialBlock[1] = 0x33; // '3'
//   serialBlock[2] = 0x22; // '2'
//   serialBlock[3] = 0x11; // '1'

//   // --- КАДР 3200 (Обороты и Скорость) ---
//   uint16_t rd_rpm = (uint16_t)currentRPM;     // Например, 2500
//   uint16_t rd_speed = (uint16_t)currentSpeed; // Например, 60
//   memcpy(&serialBlock[4], &rd_rpm, 2);
//   memcpy(&serialBlock[6], &rd_speed, 2);

//   // --- КАДР 3201 (Вольтметр и ДТОЖ) ---
//   // Умножаем вольты на 100, чтобы передать float как целое число (12.26 -> 1226)
//   uint16_t rd_voltage = (uint16_t)(batteryVoltage * 100.0); 
//   uint16_t rd_ect = (uint16_t)rawECT; // Значение АЦП (0-1023)
//   memcpy(&serialBlock[8], &rd_voltage, 2);
//   memcpy(&serialBlock[10], &rd_ect, 2);

//   // --- КАДР 3202 (ДУТ и Дискретные входы) ---
//   uint16_t rd_fuel = (uint16_t)rawFuel; // Значение АЦП (0-1023)
  
//   // Собираем битовую маску для зажигания и дверей
//   uint16_t rd_digitals = 0;
//   if (digitalRead(PIN_IGNITION) == LOW)     rd_digitals |= (1 << 0); // Бит 0: Зажигание (учитывая полярность оптопары)
//   if (digitalRead(PIN_DOOR_TRIGGER) == LOW) rd_digitals |= (1 << 1); // Бит 1: Центральный замок
  
//   memcpy(&serialBlock[12], &rd_fuel, 2);
//   memcpy(&serialBlock[14], &rd_digitals, 2);

//   // --- КАДР 3203 (Лампы 1 и Лампы 2) --- 
//   uint16_t rd_byteFrame1 = byteFrame1;
//   uint16_t rd_byteFrame2 = byteFrame2;
//   serialBlock[16] = rd_byteFrame1;
//   serialBlock[17] = rd_byteFrame2;

//   // --- КАДР 3204 (Состояние силовых выходов) ---
//   uint16_t rd_outputs = 0;
//   if (digitalRead(PIN_HOLD_POWER) == HIGH) rd_outputs |= (1 << 0); // Бит 0
//   if (digitalRead(PIN_ACC_OUTPUT) == HIGH) rd_outputs |= (1 << 1); // Бит 1
//   memcpy(&serialBlock[18], &rd_outputs, 2);

//   // 3. Отправляем готовый бинарный пакет в UART-свисток
//   Serial2.write(serialBlock, 20); // 4 (заголовок) + 16 байт данных (кадры 3200-3203 полные, 3204 отправляет первые 2 байта)
// }


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
