#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>    // Библиотека сторожевого таймера
#include <TimeLib.h>    // Библиотека для работы со временем

// =========================================================================
// АВТОМОБИЛЬНЫЙ КОНТРОЛЛЕР ПИТАНИЯ И ДАТЧИКОВ (REALDASH CAN ВЕРСИЯ)
// Arduino Mega Pro. Посекундные таймеры + Бинарный протокол RealDash CAN
// =========================================================================                                                                                  
#define ONE_WIRE_BUS 8 // Датчики сидят на цифровом пине 8
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
// Прописываем уникальные ID адреса датчиков
DeviceAddress interTempSensor = { 0x28, 0x79, 0xF4, 0xC8, 0x00, 0x00, 0x00, 0x8C };
DeviceAddress outerTempSensor = { 0x28, 0xC5, 0xD7, 0xC9, 0x00, 0x00, 0x00, 0xCF };

// ==========================================================================
// --- КОНФИГУРАЦИЯ ПИНОВ ---
// ==========================================================================
const byte PIN_FUEL = A0;           // ДУТ (резистор 330 Ом)
const byte PIN_ECT = A1;            // ДТОЖ (резистор 330 Ом)
const byte PIN_BATTERY_SENSE = A2;  // Вольтметр (делитель 10кОм и 3.3кОм)
const byte PIN_RPM          = 2;    // Вход RPM через PC817 (Прерывание 0)
const byte PIN_SPEED        = 3;    // Вход Скорости через PC817 (Прерывание 1)
const byte PIN_ACC_OUTPUT   = 5;    // Цифровой выход: управление ключом BTS442 (ACC магнитолы)
const byte PIN_DOOR_TRIGGER = 19;    // Цифровой вход (через оптопару): сигнал ЦЗ (Прерывание )
const byte PIN_IGNITION     = 7;    // Цифровой вход (через оптопару): Зажигание (Клемма 15)

// ==========================================================================
// --- СТРУКТУРА И МАССИВЫ ИНДИКАТОРОВ (ЛАМП) ---
// ==========================================================================
struct InputChannel {
  byte pin;            // Номер пина Arduino
  String name;        // Имя для вывода в Монитор порта
};

// LAMPS
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
// --- СОСТОЯНИЕ СИСТЕМЫ ---
// ==========================================================================
enum SystemState {
  STATE_SLEEP,
  STATE_PRE_DRIVE_WAKE,
  STATE_DRIVE,
  STATE_SHUTDOWN
};

SystemState currentState = STATE_SLEEP;
// ==========================================================================
// --- ПЕРЕМЕННЫЕ И ТАЙМИНГИ ---
// ==========================================================================
const unsigned long TIMEOUT_WAIT_IGNITION = 18000; // Время ожидания зажигания
const unsigned long WAKE_FILTER_DELAY     = 1000;   // Фильтр сигнала ЦЗ
const float CRITICAL_BATTERY_VOLTAGE      = 11.7;    // Порог защиты аккумулятора от разряда (Вольты)
const unsigned long DEBOUNCE_DELAY = 250;     // Игнорируем помехи короче 250 мс
volatile int clickCount = 0;        // Переменная счетчика нажатий ЦЗ (volatile обязателен для прерываний)
volatile unsigned long lastDebounceTime = 0; 
volatile bool wdtFired = false;             // Флаг того, что проснулись по таймеру

unsigned long wakeUpTimerStart = 0;

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
uint8_t voltPackedX10 = 0;
unsigned int rawECT = 0;
unsigned int rawFuel = 0;
uint8_t currentInterP40 = 0;
uint8_t currentOuterP40 = 0;
uint32_t byteIndicators = 0;
String textLampsToSerial = "";

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
uint8_t readBatteryVoltageX10(int counter) {
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
  return vPin * 10; 
}

void goToSleep() {
  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  
  ADCSRA &= ~(1 << ADEN); // Отключаем АЦП

  // Настраиваем прерывания
  // attachInterrupt(digitalPinToInterrupt(PIN_IGNITION), wakeUpISR, LOW);
  attachInterrupt(digitalPinToInterrupt(PIN_DOOR_TRIGGER), doorTriggerInterrupt, LOW);

  // Настраиваем Watchdog на пробуждение каждые 1 секунду (для хода часов)
  MCUSR &= ~(1 << WDRF);
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  WDTCSR = (1 << WDIE) | (1 << WDP2) | (1 << WDP1); // 1 секунда
  wdt_reset();
  
  sleep_mode(); // Засыпаем...
  
  // --- ПРОСНУЛИСЬ (от первого клика брелка) ---
  sleep_disable();
  
  // Перенастраиваем прерывание на FALLING (спад сигнала). 
  // Когда МК уже бодрствует, FALLING работает идеально и точнее считает импульсы.
  // detachInterrupt(digitalPinToInterrupt(PIN_IGNITION));
  detachInterrupt(digitalPinToInterrupt(PIN_DOOR_TRIGGER));
  attachInterrupt(digitalPinToInterrupt(PIN_DOOR_TRIGGER), doorTriggerInterrupt, FALLING);
  
  ADCSRA |= (1 << ADEN); // Включаем АЦП обратно
}
// --- ФУНКЦИИ ОБРАБОТКИ ПРЕРЫВАНИЙ ---
// Обработчик прерывания (должен быть максимально коротким!)
void doorTriggerInterrupt() {
  unsigned long currentTime = millis();
  // Защита от дребезга контактов и наводок
  if (currentTime - lastDebounceTime > DEBOUNCE_DELAY) {
    clickCount++;
    lastDebounceTime = currentTime;
  }
}
// void wakeUpISR() {
//   // Пустой обработчик для зажигания
// }
// Прерывание сторожевого таймера (срабатывает раз в секунду во сне)
ISR(WDT_vect) {
  wdtFired = true; // Поднимаем флаг, что нужно прибавить секунду
}
void rpmPulseCounter() {
  rpmPulses++;
}

void speedPulseCounter() {
  speedPulses++;
}

void setup() {
  // Установка стартового времени вручную (Часы, Минуты, Секунды, День, Месяц, Год)
  // В будущем Tanix сможет обновить это время через UART при старте
  setTime(12, 0, 0, 27, 9, 2026);

  Serial2.begin(9600);
  pinMode(17, INPUT_PULLUP); // Подтяжка RX линии Serial2 
  delay(100);
  Serial2.println(F("============ ЗАГРУЗКА СИСТЕМЫ ================"));
  
  // Инициализация остальных пинов
  pinMode(PIN_ACC_OUTPUT, OUTPUT); 
  digitalWrite(PIN_ACC_OUTPUT, LOW);
  pinMode(PIN_DOOR_TRIGGER, INPUT_PULLUP); // Используем подтяжку для оптопары
  pinMode(PIN_IGNITION, INPUT_PULLUP);

  // Настройка ДТОЖ и ДУТ, есть внешняя подтяжка к 5в 1кОм
  pinMode(PIN_ECT, INPUT); // Для ДТОЖ
  pinMode(PIN_FUEL, INPUT); // Для ДУТ
  // Настройка прерываний скорости и оборотов, есть внешняя подтяжка к 5в 1кОм
  pinMode(PIN_SPEED, INPUT);
  pinMode(PIN_RPM, INPUT);
  // Прерывание срабатывает, когда транзистор в PC817 открывается и прижимает пин к GND
  attachInterrupt(digitalPinToInterrupt(PIN_RPM), rpmPulseCounter, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_SPEED), speedPulseCounter, FALLING);
  
  // Автоматический перебор пинов ламп из массивов, внешней подтяжки нет
  for (int i = 0; i < INDICATORS_COUNT; i++) pinMode(indicators[i].pin, INPUT_PULLUP);

  Serial.begin(115200);
  sensors.begin();
  delay(100);

  if (digitalRead(PIN_IGNITION) == LOW) currentState = STATE_DRIVE;
}

void loop() {
  // =========================================================================
  // БЛОК ОТПРАВКИ БИНАРНЫХ ДАННЫХ ПО ТАЙМЕРАМ
  // =========================================================================
  if (currentState == STATE_PRE_DRIVE_WAKE || currentState == STATE_DRIVE) {
    unsigned long currentMillis = millis();
    // 1. БЫСТРЫЕ ДАТЧИКИ
    if (currentMillis - timerFastSensors >= INTERVAL_FAST) {
      // Вычисляем точное время, прошедшее с момента последнего расчета
      unsigned long timeElapsed = currentMillis - timerNormSensors;
      timerFastSensors = currentMillis;    
      
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
           
      // Вывод быстрых данных в Монитор порта
      Serial2.print(hour()); Serial2.print(":"); Serial2.println(minute());
      Serial2.print(F(" RPM: ")); Serial2.print(currentRPM);
      Serial2.print(F(" | SPD: ")); Serial2.print(currentSpeed, 1); Serial2.println(F(" km/h"));
      Serial2.print(F(" LAMPS: ")); Serial2.println(textLampsToSerial);
      
      if (digitalRead(PIN_IGNITION) == LOW)     Serial2.println(F(" [ Зажигание ]")); 
      if (digitalRead(PIN_DOOR_TRIGGER) == LOW) Serial2.println(F(" [ ЦЗ ] "));
    }

    // 2. СТАНДАРТНЫЕ ДАТЧИКИ 
    if (currentMillis - timerNormSensors >= INTERVAL_NORM) {
      timerNormSensors = currentMillis;
    }

    // 3. МЕДЛЕННЫЕ ДАТЧИКИ
    if (currentMillis - timerSlowSensors >= INTERVAL_SLOW) {
      timerSlowSensors = currentMillis;
      
      // Чтение вольтметра
      voltPackedX10 = readBatteryVoltageX10(10); 
      float batteryVoltage = voltPackedX10 * 0.1; 
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

      sensors.requestTemperatures(); // Запрос у всех датчиков

      // Чтение строго по ID адресам
      currentInterP40 = sensors.getTempC(interTempSensor) + 40;
      currentOuterP40 = sensors.getTempC(outerTempSensor) + 40;
            
      uint8_t data3201[8] = {0};
      memcpy(data3201, &rawECT, 2);
      memcpy(data3201 + 2, &rawFuel, 2);
      memcpy(data3201 + 4, &voltPackedX10, 1);
      memcpy(data3201 + 5, &currentInterP40, 1);
      memcpy(data3201 + 6, &currentOuterP40, 1);
      
      // Отправка в RealDash
      sendRealDashFrame(3201, data3201);
      
      //Вывод медленных данных в Монитор порта
      Serial2.println(F("----------------------------------"));
      Serial2.print(hour()); Serial2.print(":"); Serial2.println(minute());
      Serial2.print(F(" VOLTAGE: ")); Serial2.print(batteryVoltage, 2); Serial2.print(F(" V"));
      Serial2.print(F(" | ДТОЖ ADC: ")); Serial2.print(rawECT); 
      Serial2.print(F(" | ДУТ ADC: ")); Serial2.println(rawFuel);
      Serial2.print(F(" | Салон: ")); Serial2.print((currentInterP40 - 40), 1); Serial2.println(F(" °C"));
      Serial2.print(F(" | Улица: ")); Serial2.print((currentOuterP40 - 40), 1); Serial2.println(F(" °C"));
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
      if (digitalRead(PIN_ACC_OUTPUT) == LOW) digitalWrite(PIN_ACC_OUTPUT, HIGH);
      // Ждем зажигания в течение минут
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
      if (digitalRead(PIN_ACC_OUTPUT) == LOW) digitalWrite(PIN_ACC_OUTPUT, HIGH);
      // В режиме поездки нам плевать на любые клацанья ЦЗ или дверей. Мы смотрим только на зажигание.
      if (!isIgnitionOn) {
        currentState = STATE_SHUTDOWN;
        Serial2.println(F("Состояние изменено на SHUTDOWN. Машина выключена"));
      }
      break;

    case STATE_SHUTDOWN:
      Serial2.println(F("Выключаем питание Андроид..."));
      digitalWrite(PIN_ACC_OUTPUT, LOW); // Обесточиваем магнитолу через BTS442
      delay(500); // Короткая пауза для записи кэша магнитолы
      
      Serial2.println(F("Состояние изменено на SLEEP!"));
      currentState = STATE_SLEEP;
      clickCount = 0;
    
      break;

    case STATE_SLEEP:
      // 1. АНАЛИЗ КЛИКОВ (если проснулись по ЦЗ)
      if (clickCount > 0) {
        unsigned long windowTimer = millis();
        int currentClicks = clickCount;
    
        // Ждем клики в течение заданного интервала (WAKE_FILTER_DELAY)
        while (millis() - windowTimer < WAKE_FILTER_DELAY) {
          if (clickCount != currentClicks) {
            currentClicks = clickCount;
            windowTimer = millis(); 
          }
        }
        if (currentClicks == 1) {
          // 1 КЛИК = Точно едем! Проверяем АКБ перед запуском приставки
          float batteryVoltage = readBatteryVoltageX10(3) * 0.1;
          
          if (batteryVoltage > CRITICAL_BATTERY_VOLTAGE) {
              digitalWrite(PIN_ACC_OUTPUT, HIGH); // ВКЛЮЧАЕМ BTS442
              currentState = STATE_PRE_DRIVE_WAKE;
              wdt_disable();
              wakeUpTimerStart = millis();
          }
        } 
        else {
          // 2 и более КЛИКОВ = Просто пришли забрать вещи. 
          // Оставляем BTS442 выключенным, сбрасываем счетчик и loop() отправит нас обратно в сон
          clickCount = 0; 
        }
        
        // Выходим из loop(), чтобы обновить состояния автомата и не провалиться в while ниже
        return; 
      }
      
      // 2. ЦИКЛ ГЛУБОКОГО СНА (если зажигания нет и кликов нет)
      while (digitalRead(PIN_IGNITION) == HIGH && clickCount == 0) {
        goToSleep();
      
        // Ход часов от Watchdog (1 секунда)
        if (wdtFired) {
          wdtFired = false;
          adjustTime(1); 
        }
      }
      
      // Если проснулись от ключа зажигания, минуя ЦЗ (например, сидели внутри машины)
      if (digitalRead(PIN_IGNITION) == LOW) { 
        currentState = STATE_DRIVE;
        wdt_disable();
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
//       <value name="Indicator: Temperature Inter" offset="5" length="1" conversion="V-40"></value> <!-- Температура Салон -->
//       <value name="Indicator: Temperature Outer" offset="6" length="1" conversion="V-40"></value> <!-- Температура Улица --> 
//     </frame>
//   </frames>
// </RealDashCAN>

// Примечание: Параметр targetId — это внутренний уникальный номер датчика в экосистеме RealDash 
// (например, 37 — это всегда RPM, а 12 — вольтаж батареи). Полный список этих ID есть на официальном сайте RealDash.
