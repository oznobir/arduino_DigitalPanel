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
  byte pin;            // Номер пина Arduino
  String name;        // Имя для вывода в Монитор порта
};

// LAMPS 1: Стандартные лампы
const int INDICATORS_COUNT = 19;
InputChannel indicators[INDICATORS_COUNT] = {
  {9, "[Левый поворотник] "},
  {11, "[Правый поворотник] "},
  {10, "[Дальний свет] "},
  {23, "[Ближний свет] "},
  {25, "[Передние ПТФ] "},
  {27, "[Задний ПТФ] "},
  {29, "[Габариты] "},
  {31, "[P_10] "},
  {32, "[Давление масла] "}, 
  {34, "[Ручник] "}, 
  {36, "[Check Engine] "}, 
  {38, "[Аккумулятор] "}, 
  {40, "[ABS] "},
  {42, "[Перегрев ОЖ] "},
  {44, "[Ремень безопасности] "},
  {33, "[Двери открыты] "},
  {35, "[Подушки безопасности] "}, 
  {37, "[Иммобилайзер] "}, 
  {39, "[Пила] "}
};

// ==========================================================================
// --- ПЕРЕМЕННЫЕ И ТАЙМИНГИ ---
// ==========================================================================
const unsigned long TIMEOUT_WAIT_IGNITION = 18000; // Время ожидания зажигания
const unsigned long WAKE_FILTER_DELAY     = 1000;   // Фильтр сигнала ЦЗ
const float CRITICAL_BATTERY_VOLTAGE      = 11.7;    // Порог защиты аккумулятора от разряда (Вольты)

volatile unsigned long rpmPulses = 0;
volatile unsigned long speedPulses = 0;

unsigned long timerFastSensors = 0;
unsigned long timerNormSensors = 0;
unsigned long timerSlowSensors = 0;

const unsigned long INTERVAL_FAST = 500;  
const unsigned long INTERVAL_NORM = 1000; 
const unsigned long INTERVAL_SLOW = 2000; 

// Коэффициенты под Nissan Almera G15
const int pulsesPerTurnRPM = 2;        
const float pulsesPerKmSpeed = 6000.0; 
const float dividerRatio = 4.0523;     // (9,92кОм + 3.25кОм) / 3.25кОм

// Глобальные переменные данных
unsigned int currentRPM = 0;
unsigned int currentSpeed = 0;
float batteryVoltage = 0;
unsigned int rawECT = 0;
unsigned int rawFuel = 0;
uint32_t byteIndicators = 0;
String textLampsToSerial = "";
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
// --- ФУНКЦИЯ ОТПРАВКИ REALDASH CAN ---
// ==========================================================================
void sendRealDashFrame(unsigned long canId, const uint8_t* data8Bytes) {
  const uint8_t serialBlockHeader[4] = { 0x44, 0x33, 0x22, 0x11 };
  Serial.write(serialBlockHeader, 4);
  Serial.write((const uint8_t*)&canId, 4);
  Serial.write(data8Bytes, 8);
}

// Функция точного измерения напряжения аккумулятора
int readBatteryVoltageX10(int counter) {
  // 1. ХОЛОСТОЙ ВЫСТРЕЛ (Dummy Read): переключаем АЦП на пин вольтметра
  // и даем зарядиться внутреннему конденсатору. Результат просто выбрасываем.
  analogRead(PIN_BATTERY_SENSE); 
  
  // 2. Основной цикл сбора данных (уже чистые показания)
  int rawSum = 0;
  for (int i = 0; i < counter; i++) {
    rawSum += analogRead(PIN_BATTERY_SENSE);
  }
  
  float averageRaw = (float)rawSum / (float)counter;
  float vPin = ((averageRaw * 5.0) / 1023.0) * dividerRatio;
  return vPin  * 10; 
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
  
  Serial2.begin(9600);
  delay(100);
  Serial2.println(F("============ ЗАГРУЗКА СИСТЕМЫ ================"));
  
  // Инициализация остальных пинов
  pinMode(PIN_ACC_OUTPUT, OUTPUT);
  digitalWrite(PIN_ACC_OUTPUT, LOW); // Магнитола пока строго выключена
  
  pinMode(PIN_DOOR_TRIGGER, INPUT_PULLUP); // Используем подтяжку для оптопары
  pinMode(PIN_IGNITION, INPUT_PULLUP);

  // 2. ЭКСПРЕСС-ДИАГНОСТИКА АКБ
  Serial2.println(F("============ ПРОВЕРЯЕМ БАТАРЕЮ И НАЖАТИЕ НА БРЕЛОК ЦЗ ==============="));
  batteryVoltage = readBatteryVoltageX10(3) * 0.1;
  Serial2.print(F("-------- Батарея: ")); Serial2.print(batteryVoltage); Serial2.println(F("V--------"));
  if ((batteryVoltage) < CRITICAL_BATTERY_VOLTAGE) {
    Serial2.println(F("=========Выключаем питание вообще======"));
    currentState = STATE_SHUTDOWN;
    return;
  }
  
  // 3. ФИЛЬТР НАЖАТИЙ НА ЦЗ
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
      Serial2.println(F("--------Нажали----------------- "));
    }
   lastDoorState = currentDoorState; 
  }
  Serial2.print(F(" Всего: ")); Serial2.println(++doorCounter);
  
  if (doorCounter == 1) {
      realWakeUpDetected = true; 
  }
  if (!realWakeUpDetected) {
    currentState = STATE_SHUTDOWN;
    return;
  } else {
    // Сигнал подтвержден, хозяин открыл машину или завел её
    Serial2.println(F("=======Включаем питание Андроид...========="));
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
  for (int i = 0; i < INDICATORS_COUNT; i++) pinMode(indicators[i].pin, INPUT_PULLUP);

  Serial.begin(115200);
  //pinMode(17, INPUT_PULLUP); // Подтяжка RX линии Serial2 
  delay(100);
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

    // 2. СТАНДАРТНЫЕ ДАТЧИКИ 
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
      currentRPM = (calcRPM < 0) ? 0 : (int)calcRPM;

      float calcSpeed = ((float)localSpeedPulses / pulsesPerKmSpeed) / ((float)timeElapsed / 3600000.0);
      currentSpeed = (calcSpeed < 1.0) ? 0 : (int)calcSpeed;
      
      byteIndicators = 0;
      textLampsToSerial ="";
      for (int i = 0; i < INDICATORS_COUNT; i++) {
        if (digitalRead(indicators[i].pin) == LOW) {
          byteIndicators |= (1UL << i);
          textLampsToSerial += indicators[i].name;
        }
      }
      
      // --- Подготовка пакета данных для Кадра 101 ---
      uint8_t data3200[8] = {0};
      memcpy(data3200, &byteIndicators, 4);
      memcpy(data3200 + 4, &currentRPM, 2);
      memcpy(data3200 + 6, &currentSpeed, 2);
      
      // Отправка в RealDash (ВРЕМЕННО ЗАКОММЕНТИРОВАНО ДЛЯ ТЕСТА В МОНИТОРЕ ПОРТА)
      sendRealDashFrame(3200, data3200);
      // Serial2.println(" -- 3200 -- ");
      // for (byte i = 0; i < 8; i++) {
      //   Serial2.print(data3200[i], BIN);
      //   Serial2.print(" ");
      // }
      // Serial2.println("");
      
      // Вывод быстрых данных в Монитор порта
      Serial2.print(F(" RPM: ")); Serial2.print(currentRPM);
      Serial2.print(F(" | SPD: ")); Serial2.print(currentSpeed, 1); Serial2.println(F(" km/h"));
      Serial2.print(F(" LAMPS: ")); Serial2.println(textLampsToSerial);
      
      if (digitalRead(PIN_IGNITION) == LOW)     Serial2.println(F(" [ Зажигание ]")); 
      if (digitalRead(PIN_DOOR_TRIGGER) == LOW) Serial2.println(F(" [ ЦЗ ] "));
    }

    // 3. МЕДЛЕННЫЕ ДАТЧИКИ (CAN ID: 102, каждые 2 секунды)
    if (currentMillis - timerSlowSensors >= INTERVAL_SLOW) {
      timerSlowSensors = currentMillis;
      
      // Чтение вольтметра
      int voltPacked = readBatteryVoltageX10(10); 
      batteryVoltage = voltPacked * 0.1; 
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
      
      uint8_t data3201[8] = {0};
      memcpy(data3201, &rawECT, 2);
      memcpy(data3201 + 2, &rawFuel, 2);
      memcpy(data3201 + 4, &voltPacked, 1);
      // Serial2.println(" -- 3201 -- ");
      // for (byte i = 0; i < 8; i++) {
      //   Serial2.print(data3201[i], BIN);
      //   Serial2.print(" ");
      // }
      // Serial2.println("");
      // Отправка в RealDash (ВРЕМЕННО ЗАКОММЕНТИРОВАНО ДЛЯ ТЕСТА В МОНИТОРЕ ПОРТА)
      sendRealDashFrame(3201, data3201);
      
      //Вывод медленных данных в Монитор порта
      Serial2.println(F("----------------------------------"));
      Serial2.print(F(" VOLTAGE: ")); Serial2.print(batteryVoltage, 2); Serial2.print(F(" V"));
      Serial2.print(F(" | ДТОЖ ADC: ")); Serial2.print(rawECT); 
      Serial2.print(F(" | ДУТ ADC: ")); Serial2.println(rawFuel);
      Serial2.println(F("-----------------------------------"));
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
      delay(1000); // Короткая пауза для записи кэша магнитолы
      
      //Serial2.println(F("Выключаем саму панель. Пока!"));
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
//     <!-- Кадр 3200: Обычные датчики и cигналы -->
//     <frame id="3200">
//       <!-- Лампы по ПЛЮСУ  -->
//       <value name="Indicator: Turn Signal Left" startbit="0" bitcount="1"></value> <!-- Левый поворотник -->
//       <value name="Indicator: Turn Signal Right" startbit="1" bitcount="1"></value> <!-- Правый поворотник -->
//       <value name="Indicator: High Beam" startbit="2" bitcount="1"></value> <!-- Дальний свет -->
//       <value name="Indicator: Low Beam" startbit="3" bitcount="1"></value> <!-- Ближний свет -->
//       <value name="Indicator: Front Fog Lights" startbit="4" bitcount="1"></value> <!-- Передние ПТФ -->
//       <value name="Indicator: Rear Fog Lights" startbit="5" bitcount="1"></value> <!-- Задний ПТФ -->
//       <value name="Indicator: Parking Lights" startbit="6" bitcount="1"></value> <!-- Габариты -->
//       <value name="Indicator: P_10" startbit="7" bitcount="1"></value> <!-- Indicator: P_10 -->

//       <!-- Лампы по МИНУСУ  -->
//       <value name="Indicator: Oil Pressure" startbit="8" bitcount="1"></value> <!-- Давление масла -->
//       <value name="Indicator: Hand Brake" startbit="9" bitcount="1"></value> <!-- Ручник / Тормозуха -->
//       <value name="Indicator: Check Engine" startbit="10" bitcount="1"></value> <!-- Check Engine -->
//       <value name="Indicator: Battery Charging" startbit="11" bitcount="1"></value> <!-- Аккумулятор / Зарядка -->
//       <value name="Indicator: ABS Warning" startbit="12" bitcount="1"></value> <!-- ABS -->
//       <value name="Indicator: Coolant Warning" startbit="13" bitcount="1"></value> <!-- Перегрев ОЖ -->
//       <value name="Indicator: Seat Belt Warning" startbit="14" bitcount="1"></value> <!-- Ремень безопасности -->
//       <value name="Indicator: Doors Open" startbit="15" bitcount="1"></value> <!-- Двери открыты (Общий) -->
//       <value name="Indicator: Airbag Warning" startbit="16" bitcount="1"></value> <!-- Подушки безопасности (SRS) -->
//       <value name="Indicator: Immobilizer" startbit="17" bitcount="1"></value> <!-- Иммобилайзер (Красный диод) -->
//       <value name="Indicator: Saw (Electronics)" startbit="18" bitcount="1"></value> <!-- Пила -->

//       <value targetId="37" offset="4" length="2" units="RPM"></value> <!-- ENGINE/ECU INPUTS / RPM - Обороты -->
//       <value targetId="64" offset="6" length="2" units="km/h"></value> <!-- ENGINE/ECU INPUTS / Vehicle Speed - Скорость -->
//     </frame>

//     <!-- Кадр 3201: Медленные датчики -->
//     <frame id="3201">
//       <value name="Indicator: Coolant Temperature" offset="0" length="2" conversion="V/1023*90"></value> <!-- ТОЖ Формула временно -->
//       <value name="Indicator: Fuel Level" offset="2" length="2"> conversion="V/1023*20"></value> <!-- Топливо Формула временно -->
//       <value name="Indicator: Battery Voltage" offset="4" length="1" conversion="V/10"></value> <!-- Батарея -->  
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
