#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include <OneWire.h>
#include <DallasTemperature.h>  // Библиотека для работы с цифровыми датчиками
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>            // Библиотека сторожевого таймера
#include <TimeLib.h>            // Библиотека для работы со временем
#include <Wire.h>               // Библиотека для работы с I2C
#include <RTClib.h>             // Библиотека для работы с RTC DS3231
#include <Adafruit_FRAM_I2C.h>  // Библиотека для работы со fram-памятью MB85RC256V

// =========================================================================
// АВТОМОБИЛЬНЫЙ КОНТРОЛЛЕР ПИТАНИЯ И ДАТЧИКОВ (REALDASH CAN ВЕРСИЯ)
// Arduino Mega Pro. Посекундные таймеры + Бинарный протокол RealDash CAN
// =========================================================================

#define RD_SERIAL Serial     // Порт к RealDash
#define TEST_SERIAL Serial2  // Порт для тестов
#define GBO_SERIAL Serial3   // Порт к ЭБУ ГБО

// ==========================================================================
// --- КОНФИГУРАЦИЯ ПИНОВ ---
// ==========================================================================
const byte PIN_FUEL = A0;           // ДУТ (резистор 330 Ом)
const byte PIN_ECT = A1;            // ДТОЖ (резистор 330 Ом)
const byte PIN_BATTERY_SENSE = A2;  // Вольтметр (делитель 10кОм и 3.3кОм)
const byte PIN_RPM = 2;             // Вход RPM через PC817 (Прерывание 0)
const byte PIN_SPEED = 3;           // Вход Скорости через PC817 (Прерывание 1)
const byte PIN_ACC_OUTPUT = 4;      // Цифровой выход: управление ключом BTS442 (ACC магнитолы)
const byte ONE_WIRE_BUS = 5;        // Датчики температуры
const byte SPI_CS_PIN = 6;          // CS SPI модуля MC2515
const byte PIN_DOOR_TRIGGER = 18;   // Цифровой вход (через оптопару): сигнал ЦЗ (Прерывание при включении)
// const byte PIN_IGNITION     = 19;   // Цифровой вход (через оптопару): Зажигание (Клемма 15) (Прерывание при включении)

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
// Прописываем уникальные ID адреса датчиков
DeviceAddress interTempSensor = { 0x28, 0x79, 0xF4, 0xC8, 0x00, 0x00, 0x00, 0x8C };
DeviceAddress outerTempSensor = { 0x28, 0xC5, 0xD7, 0xC9, 0x00, 0x00, 0x00, 0xCF };
RTC_DS3231 rtc;
Adafruit_FRAM_I2C fram = Adafruit_FRAM_I2C();
MCP_CAN CAN(SPI_CS_PIN);

// ==========================================================================
// --- СТРУКТУРА И МАССИВЫ ИНДИКАТОРОВ (ЛАМП) ---
// ==========================================================================
struct InputChannel {
  byte pin;          // Номер пина Arduino
  const char* name;  // Имя для вывода в Монитор порта
};

// LAMPS
const int INDICATORS_COUNT = 20;
const InputChannel indicators[INDICATORS_COUNT] = {
  { 19, "[Зажигание] " },
  { 9, "[Левый поворотник] " },
  { 11, "[Правый поворотник] " },
  { 23, "[Дальний свет] " },
  { 25, "[Ближний свет] " },
  { 27, "[Передние ПТФ] " },
  { 29, "[Задний ПТФ] " },
  { 31, "[Габариты] " },
  { 7, "[P_10] " },
  { 6, "[Давление масла] " },
  { 8, "[Ручник] " },
  { 22, "[Check Engine] " },
  { 24, "[Аккумулятор] " },
  { 26, "[ABS] " },
  { 28, "[Перегрев ОЖ] " },
  { 30, "[Ремень безопасности] " },
  { 33, "[Двери открыты] " },
  { 35, "[Подушки безопасности] " },
  { 37, "[Иммобилайзер] " },
  { 39, "[Пила] " }
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

SystemState currentState = STATE_PRE_DRIVE_WAKE;

bool isFramConnected = true;
bool isRtcConnected = true;

// ==========================================================================
// --- Константы для работы с ГБО ---
// ==========================================================================

const uint8_t GBO_QUERY[] = { 0xF0, 0x01, 0x01, 0xF2 };
const int GBO_PACKET_SIZE = 83;  // Ожидаемый размер
uint8_t gboBuf[GBO_PACKET_SIZE];
int gboIndex = 0;
// Состояния конечного автомата для чтения UART GBO
enum GboReadState {
  WAIT_HEADER,  // Ожидаем 0xF0
  WAIT_0,       // Ожидаем тип ответа (0x00)
  WAIT_SIZE,    // Ожидаем размер (0x53)
  WAIT_TYPE,    // Ожидаем тип ответа (0x01)
  READ_PAYLOAD  // Собираем оставшиеся данные пакета
};
GboReadState gboState = WAIT_HEADER;

// ==========================================================================
// --- ПЕРЕМЕННЫЕ И ТАЙМИНГИ ---
// ==========================================================================

const unsigned long TIMEOUT_WAIT_IGNITION = 10000;  // Время ожидания зажигания
const unsigned long WAKE_FILTER_DELAY = 1000;       // Фильтр сигнала ЦЗ
const float CRITICAL_BATTERY_VOLTAGE = 11.7;        // Порог защиты аккумулятора от разряда (Вольты)
const unsigned long DEBOUNCE_DELAY = 250;           // Игнорируем помехи короче 250 мс
volatile int clickCount = 0;                        // Переменная счетчика нажатий ЦЗ (volatile обязателен для прерываний)
volatile bool wakeUpIgnition = false;               // Флаг того, что проснулись от зажигания
volatile unsigned long sleepSecondsCount = 0;

unsigned long wakeUpTimerStart = 0;

volatile unsigned long rpmPulses = 0;
volatile unsigned long speedPulses = 0;

unsigned long timerFastSensors = 0;
unsigned long timerNormSensors = 0;
unsigned long timerSlowSensors = 0;
unsigned long timerGboRequest = 0;
unsigned long timeElapsed = 0;

const unsigned long INTERVAL_FAST = 100;
const unsigned long INTERVAL_NORM = 1500;
const unsigned long INTERVAL_SLOW = 2000;
const unsigned long INTERVAL_GBO_REQUEST = 200;  // Опрашиваем ГБО

byte old_ADCSRA;

// =========================================================================
// --- Коэффициенты под Nissan Almera G15 ---
// =========================================================================

const int pulsesPerTurnRPM = 2;
const float pulsesPerKmSpeed = 6000.0;
const float dividerRatio = 4.0523;  // (9,92кОм + 3.25кОм) / 3.25кОм

// =========================================================================
// --- Глобальные переменные данных ---
// =========================================================================

unsigned int currentRPM = 0;
unsigned int currentSpeed = 0;
uint8_t voltPackedX10 = 0;
unsigned int rawECT = 0;
unsigned int rawFuel = 0;
uint8_t currentInterP40 = 0;
uint8_t currentOuterP40 = 0;
uint32_t byteIndicators = 0;

// ==========================================================================
// --- ФУНКЦИЯ ОТПРАВКИ REALDASH CAN ---
// ==========================================================================

// --- FRAME ---

void sendRealDashFrame(unsigned long canId, const uint8_t* data8Bytes) {
  const uint8_t serialBlockHeader[4] = { 0x44, 0x33, 0x22, 0x11 };
  RD_SERIAL.write(serialBlockHeader, 4);
  RD_SERIAL.write((const uint8_t*)&canId, 4);
  RD_SERIAL.write(data8Bytes, 8);
}

// --- TEXT ---

void sendRealDashText(unsigned long canTextId, const char* text) {
  if (text) {
    const uint8_t serialBlockHeaderText[4] = { 0x55, 0x33, 0x22, 0x11 };
    RD_SERIAL.write(serialBlockHeaderText, 4);
    RD_SERIAL.write((const uint8_t*)&canTextId, 4);
    RD_SERIAL.write(text, strlen(text) + 1);
  }
}
// НОВАЯ ПЕРЕГРУЖЕННАЯ ФУНКЦИЯ (специально для макроса F())
// void sendRealDashText(unsigned long canTextId, const __FlashStringHelper* text)
// {
//   if (text) {
//     const uint8_t serialBlockHeaderText[4] = { 0x55, 0x33, 0x22, 0x11 };
//     RD_SERIAL.write(serialBlockHeaderText, 4);
//     RD_SERIAL.write((const uint8_t*)&canTextId, 4);
//     // Для F() длину строки нужно определять через strlen_P
//     PGM_P p = (PGM_P)text;
//     size_t len = strlen_P(p);
//     // Отправляем текст побайтово прямо из Flash-памяти (PROGMEM)
//     for (size_t i = 0; i <= len; i++) { // Захватываем и завершающий '\0' (как и выше strlen(text) + 1)
//       RD_SERIAL.write(pgm_read_byte(p++));
//     }
//   }
// }
// =========================================================================
// --- Функция точного измерения напряжения аккумулятора ---
// =========================================================================
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
// =========================================================================
// --- Функция плавного пуска Андроид по ШИМ ---
// =========================================================================
void handleSmoothStart() {
  // Плавный пуск для обхода защиты BTS442E2 по току КЗ
  for (int duty = 0; duty <= 255; duty += 3) {  // ~85 шагов
    analogWrite(PIN_ACC_OUTPUT, duty);
    delay(4);  // 85 * 4 мс = ~340 мс плавного заполнения емкостей конденсаторов XL4016
  }

  // Жестко фиксируем пин в HIGH, переводя BTS442E2 в режим постоянного проводника
  digitalWrite(PIN_ACC_OUTPUT, HIGH);
}
// =========================================================================
// --- Фукция отключения модулей Ардуино ---
// =========================================================================
void powerDisable() {
  old_ADCSRA = ADCSRA;     // Сохраняем настройки АЦП
  ADCSRA &= ~(1 << ADEN);  // Отключаем АЦП
  // Выключаем аппаратные таймеры
  power_timer0_disable();
  power_timer1_disable();
  power_timer2_disable();
  power_timer3_disable();
  power_timer4_disable();
  power_timer5_disable();
  // Отключаем все интерфейсы связи: I2C (FRAM засыпает) и SPI
  power_twi_disable();
  power_spi_disable();
  // Выключаем последовательные порты (включая Serial, Serial1, Serial2, Serial3)
  power_usart0_disable();
  power_usart1_disable();
  power_usart3_disable();
  // power_usart2_disable();
}
// =========================================================================
// --- Фукция включения модулей Ардуино ---
// =========================================================================
void powerEnable() {
  ADCSRA = old_ADCSRA;  // Включаем АЦП обратно (восстанавливаем настройки)
  power_timer0_enable();
  delayMicroseconds(10);  // для millis()
                          // Включаем периферию обратно
  power_timer1_enable();
  power_timer2_enable();
  power_timer3_enable();
  power_timer4_enable();
  power_timer5_enable();

  power_twi_enable();
  power_spi_enable();
  power_usart0_enable();
  power_usart1_enable();
  power_usart3_enable();
  // power_usart2_enable();
}

// =========================================================================
// --- ФУНКЦИИ ОБРАБОТКИ ПРЕРЫВАНИЙ ---
// =========================================================================

// Обработчик прерывания (должен быть максимально коротким!)
void doorTriggerISRSleep() {
  clickCount = 1;  // Нас разбудил первый клик. Фиксируем его.
}
void doorTriggerISRWork() {
  unsigned long currentTime = micros();  // В ISR надежнее использовать micros(), он точнее для дребезга
  static unsigned long lastWorkDebounceTime = 0;

  if (currentTime - lastWorkDebounceTime > (DEBOUNCE_DELAY * 1000)) {
    clickCount++;
    lastWorkDebounceTime = currentTime;
  }
}
void wakeUpISR() {
  wakeUpIgnition = true;
}
//Прерывание сторожевого таймера (срабатывает раз в секунду во сне)
ISR(WDT_vect) {
  sleepSecondsCount++;
}
void rpmPulseCounter() {
  rpmPulses++;
}

void speedPulseCounter() {
  speedPulses++;
}
// =========================================================================
// --- ФУНКЦИЯ ПОЛУЧЕНИЯ И ОТПРАВКИ БИНАРНЫХ ДАННЫХ ПО ТАЙМЕРАМ ---
// =========================================================================
void receiveAndSendDataByTimers() {

  receiveAndSendGboAsync(); // получение данных от ЭБУ ГБО по мере поступления

  unsigned long currentMillis = millis();
  
  // БЫСТРЫЕ ДАТЧИКИ
  if (currentMillis - timerFastSensors >= INTERVAL_FAST) {
    // Вычисляем точное время, прошедшее с момента последнего расчета
    timeElapsed = currentMillis - timerFastSensors;
    timerFastSensors = currentMillis;
    receiveAndSendFastSensors();
  }

  // ЗАПРОС В ЭБУ ГБО
  if (currentMillis - timerGboRequest >= INTERVAL_GBO_REQUEST) {
    timerGboRequest = currentMillis;

    // Мгновенная отправка 4 байт, не блокирующая основной цикл
    GBO_SERIAL.write(GBO_QUERY, sizeof(GBO_QUERY));
  }

  // СТАНДАРТНЫЕ ДАТЧИКИ
  if (currentMillis - timerNormSensors >= INTERVAL_NORM) {
    timerNormSensors = currentMillis;
  }

  // МЕДЛЕННЫЕ ДАТЧИКИ
  if (currentMillis - timerSlowSensors >= INTERVAL_SLOW) {
    timerSlowSensors = currentMillis;
    receiveAndSendSlowSensors();
  }
}

// =========================================================================
// --- ФУНКЦИЯ ПОЛУЧЕНИЯ И ОТПРАВКИ БЫСТРЫХ ДАННЫХ ---
// =========================================================================

void receiveAndSendFastSensors() {
  // Блокируем прерывания на доли микросекунды, чтобы безопасно скопировать данные
  noInterrupts();
  unsigned long localRpmPulses = rpmPulses;
  unsigned long localSpeedPulses = speedPulses;
  rpmPulses = 0;    // Сбрасываем счетчик импульсов тахометра
  speedPulses = 0;  // Сбрасываем счетчик импульсов скорости
  interrupts();     // Снова разрешаем прерывания

  // Расчет физических величин
  float calcRPM = ((float)localRpmPulses / pulsesPerTurnRPM) * (60000.0 / timeElapsed);
  currentRPM = (calcRPM < 0) ? 0 : (int)calcRPM;

  float calcSpeed = ((float)localSpeedPulses / pulsesPerKmSpeed) / ((float)timeElapsed / 3600000.0);
  currentSpeed = (calcSpeed < 1.0) ? 0 : (int)calcSpeed;

  // Вывод быстрых данных в Монитор порта
  sendTimeToSerial();
  TEST_SERIAL.print(F(" RPM: ")); TEST_SERIAL.print(currentRPM);
  TEST_SERIAL.print(F(" | SPD: ")); TEST_SERIAL.print(currentSpeed, 1); TEST_SERIAL.println(F(" km/h"));

  byteIndicators = 0;
  bool hasActiveLamps = false;

  for (int i = 0; i < INDICATORS_COUNT; i++) {
    if (digitalRead(indicators[i].pin) == LOW) {
      byteIndicators |= (1UL << i);

      if (!hasActiveLamps) {
        TEST_SERIAL.print(F(" LAMPS: "));
        hasActiveLamps = true;
      }
      TEST_SERIAL.print(indicators[i].name);
    }
  }
  if (hasActiveLamps) TEST_SERIAL.println();  // Перенос строки в конце списка

  // if (digitalRead(PIN_IGNITION) == LOW)     TEST_SERIAL.println(F(" [ Зажигание ]"));

  // --- Подготовка пакета данных для Кадра 101 ---
  uint8_t data3200[8] = { 0 };
  memcpy(data3200, &byteIndicators, 4);
  memcpy(data3200 + 4, &currentRPM, 2);
  memcpy(data3200 + 6, &currentSpeed, 2);

  // Отправка в RealDash
  sendRealDashFrame(3200, data3200);
}

// =========================================================================
// --- ФУНКЦИЯ ПОЛУЧЕНИЯ И ОТПРАВКИ МЕДЛЕННЫХ ДАННЫХ ---
// =========================================================================

void receiveAndSendSlowSensors() {
  // Чтение вольтметра
  voltPackedX10 = readBatteryVoltageX10(10);
  float batteryVoltage = voltPackedX10 * 0.1;
  // Чтение сырого АЦП ДТОЖ (с защитой мультиплексора)
  analogRead(PIN_ECT);
  int rawSumTemp = 0;
  for (int i = 0; i < 10; i++) rawSumTemp += analogRead(PIN_ECT);
  rawECT = rawSumTemp / 10;

  // Чтение сырого АЦП ДУТ (с защитой мультиплексора)
  analogRead(PIN_FUEL);
  int rawSumFuel = 0;
  for (int i = 0; i < 10; i++) rawSumFuel += analogRead(PIN_FUEL);
  rawFuel = rawSumFuel / 10;

  sensors.requestTemperatures();  // Запрос у всех датчиков

  // Чтение строго по ID адресам
  currentInterP40 = sensors.getTempC(interTempSensor) + 40;
  currentOuterP40 = sensors.getTempC(outerTempSensor) + 40;

  uint8_t data3201[8] = { 0 };
  memcpy(data3201, &rawECT, 2);
  memcpy(data3201 + 2, &rawFuel, 2);
  memcpy(data3201 + 4, &voltPackedX10, 1);
  memcpy(data3201 + 5, &currentInterP40, 1);
  memcpy(data3201 + 6, &currentOuterP40, 1);

  // Отправка в RealDash
  sendRealDashFrame(3201, data3201);

  //Вывод медленных данных в Монитор порта
  TEST_SERIAL.println(F("----------------------------------"));
  sendTimeToSerial();
  TEST_SERIAL.print(F(" VOLTAGE: ")); TEST_SERIAL.print(batteryVoltage, 2); TEST_SERIAL.print(F(" V"));
  TEST_SERIAL.print(F(" | ДТОЖ ADC: ")); TEST_SERIAL.print(rawECT); 
  TEST_SERIAL.print(F(" | ДУТ ADC: ")); TEST_SERIAL.println(rawFuel);
  TEST_SERIAL.print(F(" | Салон: ")); TEST_SERIAL.print((currentInterP40 - 40), 1); TEST_SERIAL.println(F(" °C"));
  TEST_SERIAL.print(F(" | Улица: ")); TEST_SERIAL.print((currentOuterP40 - 40), 1); TEST_SERIAL.println(F(" °C"));
  TEST_SERIAL.println(F("-----------------------------------"));
}

// =========================================================================
// --- ФУНКЦИЯ НЕБЛОКИРУЮЩЕГО ЧТЕНИЯ ДАННЫХ ОТ ЭБУ ГБО ---
// =========================================================================

void receiveAndSendGboAsync() {
  while (GBO_SERIAL.available() > 0) {
    uint8_t inByte = GBO_SERIAL.read();

    switch (gboState) {
      case WAIT_HEADER:
        if (inByte == 0xF0) {
          gboIndex = 0;
          gboBuf[gboIndex++] = inByte;
          gboState = WAIT_0;
        }

        break;

      case WAIT_0:
        if (inByte == 0x00) {
          gboBuf[gboIndex++] = inByte;
          gboState = WAIT_SIZE;
        } else gboState = WAIT_HEADER;

        break;

      case WAIT_SIZE:
        if (inByte == 0x53) {  // 0x53 = 83 байта
          gboBuf[gboIndex++] = inByte;
          gboState = WAIT_TYPE;
        } else gboState = WAIT_HEADER;  // Если размер не совпал, сбрасываем и ищем новый заголовок
        
        break;

      case WAIT_TYPE:
        if (inByte == 0x01) {  // Это ответ на наш запрос
          gboBuf[gboIndex++] = inByte;
          gboState = READ_PAYLOAD;
        } else gboState = WAIT_HEADER;  // Чужой или неверный пакет

        break;

      case READ_PAYLOAD:
        gboBuf[gboIndex++] = inByte;

        // Если собрали все 83 байта (индексы от 0 до 82)
        if (gboIndex == GBO_PACKET_SIZE) {
          // --- УСПЕШНО ПОЛУЧЕН ВЕСЬ ПАКЕТ ДАННЫХ (83 байта) ---
          parseGboData();
          gboState = WAIT_HEADER;  // Сбрасываем автомат для следующего пакета
        }

        break;
    }
  }
}
// =========================================================================
// --- Функция обработки данных от ЭБУ ГБО  ---
// =========================================================================

void parseGboData() {

}

// =========================================================================
// --- Тестовая функция проверки чтения-записи данных в память ---
// =========================================================================

void testWriteReadFram() {
  // Тестовая запись во FRAM (адрес 0x0000, значение 42)
  uint16_t testAddr = 0x0000;
  uint8_t writeVal = 42;
  fram.write(testAddr, writeVal);

  // Тестовое чтение из FRAM
  uint8_t readVal = fram.read(testAddr);
  TEST_SERIAL.print(F("Считано из FRAM: "));
  TEST_SERIAL.println(readVal);
}

// =========================================================================
// --- Функция отправки времени ---
// =========================================================================

void sendTimeToSerial() {
  if (isRtcConnected) {
    // Получаем текущее время из RTC
    DateTime now = rtc.now();
    // time_t t = rtc.now().unixtime(); // Получаем UNIX-время
    // Выводим время в консоль
    TEST_SERIAL.print(now.year(), DEC);
    TEST_SERIAL.print(F("/"));
    TEST_SERIAL.print(now.month(), DEC);
    TEST_SERIAL.print(F("/"));
    TEST_SERIAL.print(now.day(), DEC);
    TEST_SERIAL.print(F(" "));
    TEST_SERIAL.print(now.hour(), DEC);
    TEST_SERIAL.print(F(":"));
    TEST_SERIAL.print(now.minute(), DEC);
    TEST_SERIAL.print(F(":"));
    TEST_SERIAL.print(now.second(), DEC);
    TEST_SERIAL.println();
  } else {
    //  time_t t = now(); // Получаем UNIX-время
    TEST_SERIAL.print(year());
    TEST_SERIAL.print(F("/"));
    TEST_SERIAL.print(month());
    TEST_SERIAL.print(F("/"));
    TEST_SERIAL.print(day());
    TEST_SERIAL.print(F(" "));
    TEST_SERIAL.print(hour());
    TEST_SERIAL.print(F(":"));
    TEST_SERIAL.print(minute());
    TEST_SERIAL.print(F(":"));
    TEST_SERIAL.println(second());
  }
}

// *************************************************************************
// *************************        SETUP          *************************
// *************************************************************************

void setup() {
  // Установка стартового времени вручную (Часы, Минуты, Секунды, День, Месяц, Год)
  // В будущем Tanix сможет обновить это время через UART при старте
  // setTime(12, 0, 0, 5, 10, 2026);

  TEST_SERIAL.begin(9600);
  pinMode(17, INPUT_PULLUP);  // Подтяжка RX линии TEST_SERIAL
  delay(100);

  GBO_SERIAL.begin(9600);
  pinMode(15, INPUT_PULLUP);  // Подтяжка RX линии GBO_SERIAL
  delay(100);
  
  GBO_SERIAL.write(GBO_QUERY, sizeof(GBO_QUERY));

  TEST_SERIAL.println(F("============ ЗАГРУЗКА СИСТЕМЫ ================"));
  TEST_SERIAL.println(F("Инициализация устройств..."));
  // Инициализация остальных пинов
  pinMode(PIN_ACC_OUTPUT, OUTPUT);
  digitalWrite(PIN_ACC_OUTPUT, LOW);
  pinMode(PIN_DOOR_TRIGGER, INPUT_PULLUP);  // Используем подтяжку для оптопары
  // pinMode(PIN_IGNITION, INPUT_PULLUP);

  // Настройка ДТОЖ и ДУТ
  pinMode(PIN_ECT, INPUT_PULLUP);   // Для ДТОЖ
  pinMode(PIN_FUEL, INPUT_PULLUP);  // Для ДУТ
  // Настройка прерываний скорости и оборотов, есть внешняя подтяжка к 5в 1кОм
  pinMode(PIN_SPEED, INPUT);
  pinMode(PIN_RPM, INPUT);
  // Прерывание срабатывает, когда транзистор в PC817 открывается и прижимает пин к GND
  attachInterrupt(digitalPinToInterrupt(PIN_RPM), rpmPulseCounter, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_SPEED), speedPulseCounter, FALLING);

  // Автоматический перебор пинов ламп из массивов, внешней подтяжки нет
  for (int i = 0; i < INDICATORS_COUNT; i++) pinMode(indicators[i].pin, INPUT_PULLUP);

  RD_SERIAL.begin(115200);

  sensors.begin();

  Wire.begin();  // Инициализация I2C шины для Mega (пины 20 и 21 инициализируются автоматически)

  if (!rtc.begin()) {  // 1. Проверка RTC DS3231
    TEST_SERIAL.println(F("Не удалось найти модуль RTC!"));
    isRtcConnected = false;
    setTime(12, 0, 0, 1, 10, 2026);
  }

  if (rtc.lostPower()) {  // Если RTC потерял питание, устанавливаем время компиляции
    TEST_SERIAL.println(F("RTC потерял питание, устанавливаем время..."));
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  if (!fram.begin(0x50)) {  // 2. Проверка FRAM MB85RC256V. По умолчанию адрес чипа 0x50 (если выводы A0, A1, A2 замкнуты на GND)
    TEST_SERIAL.println(F("Не удалось найти память FRAM!"));
    isFramConnected = false;
  }

  // testWriteReadFram();
  if (isFramConnected && isRtcConnected) TEST_SERIAL.println(F("Все устройства успешно подключены!"));

  delay(100);

  if (digitalRead(indicators[0].pin) == LOW) currentState = STATE_DRIVE;
}

// *************************************************************************
// *************************         LOOP          *************************
// *************************************************************************

void loop() {
  // =========================================================================
  // --- АВТОМАТ СОСТОЯНИЙ ПИТАНИЯ ---
  // =========================================================================

  // Оптопара инвертирует сигнал: когда на ней +12В, пин Ардуино притягивается к GND (LOW)
  bool isIgnitionOn = (digitalRead(indicators[0].pin) == LOW);

  switch (currentState) {

    case STATE_PRE_DRIVE_WAKE:

      if (digitalRead(PIN_ACC_OUTPUT) == LOW) handleSmoothStart();

      receiveAndSendDataByTimers();

      // Ждем зажигания в течение минут
      if (isIgnitionOn) {
        currentState = STATE_DRIVE;
        TEST_SERIAL.println(F("Состояние изменено на DRIVE. Наслаждайтесь поездкой"));
      } else if (millis() - wakeUpTimerStart >= TIMEOUT_WAIT_IGNITION) {
        currentState = STATE_SHUTDOWN;
        TEST_SERIAL.println(F("Состояние изменено на SHUTDOWN. Вышло время"));
      }

      break;

    case STATE_DRIVE:

      if (digitalRead(PIN_ACC_OUTPUT) == LOW) handleSmoothStart();

      receiveAndSendDataByTimers();

      // В режиме поездки нам плевать на любые клацанья ЦЗ или дверей. Мы смотрим только на зажигание.
      if (!isIgnitionOn) {
        currentState = STATE_SHUTDOWN;
        TEST_SERIAL.println(F("Состояние изменено на SHUTDOWN. Машина выключена"));
      }
      break;

    case STATE_SHUTDOWN:

      TEST_SERIAL.println(F("Состояние изменено на SLEEP! Выключаем питание Андроид...")); TEST_SERIAL.flush();
      digitalWrite(PIN_ACC_OUTPUT, LOW);  // Обесточиваем магнитолу через BTS442
      delay(500);                         // Короткая пауза для записи кэша магнитолы
      currentState = STATE_SLEEP;

      break;

    case STATE_SLEEP:

      // Сбрасываем счетчик секунд сна перед погружением
      sleepSecondsCount = 0;
      clickCount = 0;
      // Настраиваем прерывания
      attachInterrupt(digitalPinToInterrupt(indicators[0].pin), wakeUpISR, LOW);
      attachInterrupt(digitalPinToInterrupt(PIN_DOOR_TRIGGER), doorTriggerISRSleep, LOW);

      powerDisable();

      if (!isRtcConnected) {
        // ЦИКЛ ГЛУБОКОГО СНА
        while (!wakeUpIgnition && clickCount == 0) {

          cli();
          wdt_reset();
          WDTCSR |= (1 << WDCE) | (1 << WDE);
          WDTCSR = (1 << WDIE) | (1 << WDP2) | (1 << WDP1);
          sei();

          set_sleep_mode(SLEEP_MODE_PWR_DOWN);
          sleep_enable();
          sleep_cpu();  // Засыпаем...

          // --- ПРОСНУЛИСЬ (от первого клика брелка, зажигания или Watchdog) ---
          wdt_disable();
          sleep_disable();
        }
      } else {
        set_sleep_mode(SLEEP_MODE_PWR_DOWN);
        sleep_enable();
        sleep_cpu();  // Засыпаем...

        // --- ПРОСНУЛИСЬ (от первого клика брелка, зажигания) ---
        sleep_disable();
      }


      powerEnable();

      detachInterrupt(digitalPinToInterrupt(indicators[0].pin));  //зажигание
      detachInterrupt(digitalPinToInterrupt(PIN_DOOR_TRIGGER));

      // Корректируем время в TimeLib на то количество секунд, что мы спали
      if (!isRtcConnected) adjustTime(sleepSecondsCount);

      if (clickCount > 0) {
        // Перенастраиваем прерывание на FALLING (спад сигнала) с другой функцией.
        attachInterrupt(digitalPinToInterrupt(PIN_DOOR_TRIGGER), doorTriggerISRWork, FALLING);
        unsigned long windowTimer = millis();
        int currentClicks = clickCount;

        // Ждем клики в течение заданного интервала (WAKE_FILTER_DELAY)
        while (millis() - windowTimer < WAKE_FILTER_DELAY) {
          if (clickCount != currentClicks) {
            currentClicks = clickCount;
            windowTimer = millis();
          }
        }
        detachInterrupt(digitalPinToInterrupt(PIN_DOOR_TRIGGER));

        // РАСПРЕДЕЛЕНИЕ ЛОГИКИ ПО КОЛИЧЕСТВУ КЛИКОВ:
        TEST_SERIAL.print(F("Всего зафиксировано кликов: "));
        TEST_SERIAL.println(currentClicks);
        TEST_SERIAL.flush();

        if (currentClicks == 1) {
          // 1 КЛИК = Точно едем! Проверяем АКБ перед запуском приставки
          float batteryVoltage = readBatteryVoltageX10(3) * 0.1;

          if (batteryVoltage > CRITICAL_BATTERY_VOLTAGE) {
            handleSmoothStart();  // ВКЛЮЧАЕМ BTS442
            currentState = STATE_PRE_DRIVE_WAKE;
            wakeUpTimerStart = millis();
          } else {
            TEST_SERIAL.println(F("Батарея разряжена...")); TEST_SERIAL.flush();
          }
        } else {
          TEST_SERIAL.println(F("Кликов больше двух. Идем спать...")); TEST_SERIAL.flush();
          return;
        }
      }

      if (wakeUpIgnition) {
        wakeUpIgnition = false;
        // Если проснулись от ключа зажигания, минуя ЦЗ (например, сидели внутри машины)
        if (digitalRead(indicators[0].pin) == LOW) {
          currentState = STATE_DRIVE;
          TEST_SERIAL.println(F("Появилось зажигание. Состояние изменено на DRIVE."));
        }
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
//       <value name="Indicator: ACC Ignition" startbit="0" bitcount="1"></value> <!-- Зажигание -->
//       <value name="Indicator: Turn Signal Left" startbit="1" bitcount="1"></value> <!-- Левый поворотник -->
//       <value name="Indicator: Turn Signal Right" startbit="2" bitcount="1"></value> <!-- Правый поворотник -->
//       <value name="Indicator: High Beam" startbit="3" bitcount="1"></value> <!-- Дальний свет -->
//       <value name="Indicator: Low Beam" startbit="4" bitcount="1"></value> <!-- Ближний свет -->
//       <value name="Indicator: Front Fog Lights" startbit="5" bitcount="1"></value> <!-- Передние ПТФ -->
//       <value name="Indicator: Rear Fog Lights" startbit="6" bitcount="1"></value> <!-- Задний ПТФ -->
//       <value name="Indicator: Parking Lights" startbit="7" bitcount="1"></value> <!-- Габариты -->
//       <value name="Indicator: P_10" startbit="8" bitcount="1"></value> <!-- Indicator: P_10 -->
//       <!-- Лампы по МИНУСУ  -->
//       <value name="Indicator: Oil Pressure" startbit="9" bitcount="1"></value> <!-- Давление масла -->
//       <value name="Indicator: Hand Brake" startbit="10" bitcount="1"></value> <!-- Ручник / Тормозуха -->
//       <value name="Indicator: Check Engine" startbit="11" bitcount="1"></value> <!-- Check Engine -->
//       <value name="Indicator: Battery Charging" startbit="12" bitcount="1"></value> <!-- Аккумулятор / Зарядка -->
//       <value name="Indicator: ABS Warning" startbit="13" bitcount="1"></value> <!-- ABS -->
//       <value name="Indicator: Coolant Warning" startbit="14" bitcount="1"></value> <!-- Перегрев ОЖ -->
//       <value name="Indicator: Seat Belt Warning" startbit="15" bitcount="1"></value> <!-- Ремень безопасности -->
//       <value name="Indicator: Doors Open" startbit="16" bitcount="1"></value> <!-- Двери открыты (Общий) -->
//       <value name="Indicator: Airbag Warning" startbit="17" bitcount="1"></value> <!-- Подушки безопасности (SRS) -->
//       <value name="Indicator: Immobilizer" startbit="18" bitcount="1"></value> <!-- Иммобилайзер (Красный диод) -->
//       <value name="Indicator: Saw (Electronics)" startbit="19" bitcount="1"></value> <!-- Пила -->

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
//     <frame id="512">
//       <value name="Arduino: Message" units="text"></value>
//     </frame>
//   </frames>
// </RealDashCAN>

// Примечание: Параметр targetId — это внутренний уникальный номер датчика в экосистеме RealDash
// (например, 37 — это всегда RPM, а 12 — вольтаж батареи). Полный список этих ID есть на официальном сайте RealDash.
