#include <Arduino.h>
#include <SPI.h>                // Библиотека для работы с SPI
#include <mcp_can.h>            // Библиотека для работы с CAN MC2515
#include <OneWire.h>            // Библиотека для работы с 1-Wire
#include <DallasTemperature.h>  // Библиотека для работы с цифровыми датчиками
#include <avr/sleep.h>          // Библиотека сна МК AVR
#include <avr/power.h>          // Библиотека питания МК AVR
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

// ==========================================================================
// --- Константы для работы с ГБО ---
// ==========================================================================

const uint8_t GBO_QUERY[] = { 0xF0, 0x01, 0x01, 0xF2 };
const int GBO_PACKET_SIZE = 79;  // Ожидаемый размер
size_t gboBuf[GBO_PACKET_SIZE];
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

// ============================================================================
// --- СТРУКТУРЫ ДАННЫХ С ВЫРАВНИВАНИЕМ ДЛЯ ХРАНЕНИЯ В FRAM ---
// ============================================================================
#pragma pack(push, 4)
struct HistoryRecord {
  uint32_t startTime;
  uint32_t endTime;
  uint32_t distanceMeters;
  uint32_t totalInjSeconds;
  uint32_t durationSeconds;
  float initialFuelLiters;
  uint8_t fuelType;  // 1 - Бензин, 2 - Газ
  uint8_t status;    // 0 - Активна, 1 - Закрыта, 2 - Повреждена (Битый CRC)
  uint8_t reserved;  // Выравнивание структуры до кратности 4 байтам
  uint8_t crc;
};

struct SysHeader {
  uint8_t signature;
  uint16_t currentLpgIndex;
  uint16_t currentPetrolIndex;
  uint16_t nextFreeIndex;
  uint8_t crc;
};

struct OdoData {
  uint32_t totalOdoMeters;
  uint32_t petrolOdoMeters;
  uint32_t lpgOdoMeters;
  uint8_t crc;
};
#pragma pack(pop)

SysHeader header;
OdoData odo;
HistoryRecord activeLpg;
HistoryRecord activePetrol;

// ==========================================================================
// --- ПЕРЕМЕННЫЕ, КОНСТАНТЫ И ТАЙМИНГИ ---
// ==========================================================================

bool isLpgActive = false;
// Переменные времени (Защита I2C)
uint32_t systemUnixTime = 0;
bool isRtcConnected = false;

const float CRITICAL_BATTERY_VOLTAGE = 11.7;  // Порог защиты аккумулятора от разряда (Вольты)

const unsigned long TIMEOUT_WAIT_IGNITION = 10000;  // Время ожидания зажигания
const unsigned long TIMEOUT_WAIT_CLICK = 1000;      // Время ожидания сигнала ЦЗ
const unsigned long DEBOUNCE_DELAY = 250;           // Игнорируем помехи короче 250 мс

volatile int wakeUpClickCount = 0;     // Переменная счетчика нажатий ЦЗ (volatile обязателен для прерываний)
volatile bool wakeUpIgnition = false;  // Флаг того, что проснулись от зажигания
volatile unsigned long sleepSecondsCount = 0;

volatile unsigned long rpmPulses = 0;
volatile unsigned long speedPulses = 0;

unsigned long wakeUpTimerStart = 0;
unsigned long timerFastSensors = 0;
unsigned long timerNormSensors = 0;
unsigned long timerSlowSensors = 0;
unsigned long timerGboRequest = 0;
unsigned long timerGboInj = 90;

unsigned long lastClockTick = 0;
unsigned long lastRtcSync = 0;
unsigned long lastSaveTime = 0;

const unsigned long INTERVAL_FAST = 100;
const unsigned long INTERVAL_NORM = 1500;
const unsigned long INTERVAL_SLOW = 2000;
const unsigned long INTERVAL_GBO_REQUEST = 200;  // Опрашиваем ГБО
const unsigned long INTERVAL_BACKUP_FRAM = 10000;

// ==========================================================================
// --- КОНСТАНТЫ И ПЕРЕМЕННЫЕ ДЛЯ FRAM-ПАМЯТИ ---
// ==========================================================================
// Физические адреса FRAM MB85RC256V (32 Кбайт)
const uint16_t FRAM_SYS_MAIN = 0x0000;
const uint16_t FRAM_SYS_MIRROR = 0x0010;
const uint16_t FRAM_ODO_MAIN = 0x0020;
const uint16_t FRAM_ODO_MIRROR = 0x0030;
const uint16_t FRAM_HISTORY_START = 0x0100;
const uint16_t FRAM_MAX_SIZE = 32768;
const uint16_t FRAM_SIGNATURE = 0x5A;  // Уникальный ID структуры для первичной разметки
const uint16_t MAX_HISTORY_RECORDS = (FRAM_MAX_SIZE - FRAM_HISTORY_START) / sizeof(HistoryRecord);

// =========================================================================
// --- Коэффициенты под Nissan Almera G15 ---
// =========================================================================

// const int pulsesPerTurnRPM = 2;
// const float pulsesPerKmSpeed = 6000.0;
// const float dividerRatio = 4.0523;  // (9,92кОм + 3.25кОм) / 3.25кОм

// =========================================================================
// --- Глобальные переменные данных ---
// =========================================================================

unsigned int rpm = 0;
unsigned int speed = 0;
uint8_t voltageX10 = 0;
unsigned int rawECT = 0;
unsigned int rawFuel = 0;
uint8_t tempInterP40 = 0;
uint8_t tempOuterP40 = 0;
uint32_t byteIndicators = 0;
unsigned int rpmGBO = 0;
uint16_t sumInjPetrolMs = 0;
uint16_t sumInjLpgMs = 0;



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
// --- Функция подсчета контрольной суммы (CRC8, CRC32 или аналог) ---
// =========================================================================
uint32_t calculateCRC(uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 1) crc = (crc >> 1) ^ 0xEDB88320;
      else crc >>= 1;
    }
  }
  return ~crc;
}
uint8_t calculateCRC8(const uint8_t* data, size_t length) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x80) crc = (crc << 1) ^ 0x07;
      else crc <<= 1;
    }
  }
  return crc;
}
// =========================================================================
// --- Функция точного измерения напряжения аккумулятора ---
// =========================================================================
uint8_t readBatteryVoltageX10() {
  analogRead(PIN_BATTERY_SENSE);  // Dummy Read (прогрев АЦП)

  uint16_t rawSum = 0;
  for (int i = 0; i < 10; i++) {
    rawSum += analogRead(PIN_BATTERY_SENSE);
  }
  // Быстрый целочисленный расчет с округлением без float
  // В AVR float программный с точностью до 6-7 знаков, включая знаки до запятой
  // (9,92кОм + 3.25кОм) / 3.25кОм = 4.0523 (делитель напряжения)
  // voltageX10 = (((rawSum / 10) * (5.0 / 1023.0) * 4.0523) * 10 = rawSum * 19806 / 1000000
  // математическое округление: voltageX10 = ((rawSum * 19806) + (1000000 / 2)) / 1000000
  uint32_t roundedNumerator = ((uint32_t)rawSum * 19806UL) + 500000UL;
  return (uint8_t)(roundedNumerator / 1000000UL);
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
byte powerDisable() {
  byte old = ADCSRA;       // Сохраняем настройки АЦП
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
  return old;
}
// =========================================================================
// --- Функция включения модулей Ардуино ---
// =========================================================================
void powerEnable(const byte old) {
  ADCSRA = old;  // Включаем АЦП обратно (восстанавливаем настройки)
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
  wakeUpClickCount = 1;  // Нас разбудил первый клик. Фиксируем его.
}
void doorTriggerISRWork() {
  unsigned long currentTime = micros();  // В ISR надежнее использовать micros(), он точнее для дребезга
  static unsigned long lastWorkDebounceTime = 0;

  if (currentTime - lastWorkDebounceTime > (DEBOUNCE_DELAY * 1000)) {
    wakeUpClickCount++;
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
// --- ФУНКЦИЯ ПОЛУЧЕНИЯ И ОТПРАВКИ БЫСТРЫХ ДАННЫХ ---
// =========================================================================
void collectFastData(const unsigned long timeEl) {
  // Блокируем прерывания на доли микросекунды, чтобы безопасно скопировать данные
  noInterrupts();
  unsigned long localRpmPulses = rpmPulses;
  unsigned long localSpeedPulses = speedPulses;
  rpmPulses = 0;    // Сбрасываем счетчик импульсов тахометра
  speedPulses = 0;  // Сбрасываем счетчик импульсов скорости
  interrupts();     // Снова разрешаем прерывания

  // Расчет физических величин

  // Данные для Nissan Almera G15
  // 1 оборот RPM - 2 импульса
  // 1 мин - 60000.0 мс, timeEl - время в мс
  // float calcRPM = (float)localRpmPulses * (30000.0 / timeEl);
  // rpm = (calcRPM < 0) ? 0 : (int)calcRPM;
  uint32_t numerator = (uint32_t)localRpmPulses * 30000UL;
  rpm = (numerator + ((timeEl + 1) / 2)) / timeEl;
  if (rpm < 0) speed = 0;
  // Данные для Nissan Almera G15
  // в 1 км - 6000 импульсов
  // 1 час - 3600000.0 мс, timeEl - время в мс
  // float calcSpeed = (float)localSpeedPulses * (600.0 / timeEl);
  // speed = (calcSpeed < 1.0) ? 0 : (int)calcSpeed;
  uint32_t numerator1 = (uint32_t)localSpeedPulses * 600UL;
  speed = (numerator1 + ((timeEl + 1) / 2)) / timeEl;
  if (speed < 1) speed = 0;

  // в 1 м - 6 импульсов (1000 м / 6000 имп)
  uint32_t deltaMeters = (localRpmPulses + 3) / 6;
  if (deltaMeters > 0) {
    odo.totalOdoMeters += deltaMeters;
    if (isLpgActive) {
      odo.lpgOdoMeters += deltaMeters;
      activeLpg.distanceMeters += deltaMeters;
    } else {
      odo.petrolOdoMeters += deltaMeters;
      activePetrol.distanceMeters += deltaMeters;
    }
  }

  // Вывод быстрых данных в Монитор порта
  sendTimeToSerial();
  TEST_SERIAL.print(F(" RPM: "));
  TEST_SERIAL.print(rpm);
  TEST_SERIAL.print(F(" | SPD: "));
  TEST_SERIAL.print(speed, 1);
  TEST_SERIAL.println(F(" km/h"));

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

  // --- Подготовка пакета данных для Кадра 101 ---
  uint8_t data3200[8] = { 0 };
  memcpy(data3200, &byteIndicators, 4);
  memcpy(data3200 + 4, &rpm, 2);
  memcpy(data3200 + 6, &speed, 2);

  // Отправка в RealDash
  sendRealDashFrame(3200, data3200);
}
// =========================================================================
// --- ФУНКЦИЯ ПОЛУЧЕНИЯ И ОТПРАВКИ НОРМАЛЬНЫХ ДАННЫХ ---
// =========================================================================
void collectNormData() {

  // Кадр 3205 (Глобальные одометры в КМ)
  // float totalOdoKm = (float)odo.totalOdoMeters / 1000.0;
  // float gasOdoKm = (float)odo.lpgOdoMeters / 1000.0;
  uint32_t totalOdoKm = (odo.totalOdoMeters + 500) / 1000;
  uint32_t lpgOdoKm = (odo.lpgOdoMeters + 500) / 1000;

  uint8_t data3205[8] = { 0 };
  memcpy(data3205, &totalOdoKm, 4);
  memcpy(data3205 + 4, &lpgOdoKm, 4);

  // Отправка в RealDash
  sendRealDashFrame(3205, data3205);

  // Кадр 3206 (Текущие трипы заправок в КМ)
  // float tripGasKm = (float)activeGas.distanceMeters / 1000.0;
  // float tripPetKm = (float)activePet.distanceMeters / 1000.0;
  uint32_t tripLpgKm = (activeLpg.distanceMeters + 500) / 1000;
  uint32_t tripPetrolKm = (activePetrol.distanceMeters + 500) / 1000;

  uint8_t data3206[8] = { 0 };
  memcpy(data3206, &tripLpgKm, 4);
  memcpy(data3206 + 4, &tripPetrolKm, 4);

  // Отправка в RealDash
  sendRealDashFrame(3206, data3206);

  // Кадр 0x200 (Остатки)
  // float curLpgLiters = max(0.0f, activeLpg.initialFuelLiters - ((float)activeLpg.totalInjMs * LPG_LITERS_PER_MS));
  // float curPetrolLiters = max(0.0f, activePetrol.initialFuelLiters - ((float)activePetrol.totalInjMs * PETROL_LITERS_PER_MS));
  uint32_t curLpgLitersX10 = max(0, activeLpg.initialFuelLiters - (tripLpgKm * 105 + 500) / 1000);
  uint32_t curPetrolLitersX10 = max(0, activePetrol.initialFuelLiters - (tripPetrolKm * 125 + 500) / 1000);

  uint8_t data3207[8] = { 0 };
  memcpy(data3207, &curLpgLitersX10, 4);
  memcpy(data3207 + 4, &curLpgLitersX10, 4);

  // Отправка в RealDash
  sendRealDashFrame(3207, data3207);
}
// =========================================================================
// --- ФУНКЦИЯ ПОЛУЧЕНИЯ И ОТПРАВКИ МЕДЛЕННЫХ ДАННЫХ ---
// =========================================================================
void collectSlowData() {
  // Чтение вольтметра
  voltageX10 = readBatteryVoltageX10();
  float batteryVoltage = voltageX10 * 0.1;
  // Чтение сырого АЦП ДТОЖ (с защитой мультиплексора)
  analogRead(PIN_ECT);
  int rawSumTemp = 0;
  for (int i = 0; i < 10; i++) rawSumTemp += analogRead(PIN_ECT);
  rawECT = (rawSumTemp + 5) / 10;

  // Чтение сырого АЦП ДУТ (с защитой мультиплексора)
  analogRead(PIN_FUEL);
  int rawSumFuel = 0;
  for (int i = 0; i < 10; i++) rawSumFuel += analogRead(PIN_FUEL);
  rawFuel = (rawSumFuel + 5) / 10;

  sensors.requestTemperatures();  // Запрос у всех датчиков

  // Чтение строго по ID адресам
  tempInterP40 = sensors.getTempC(interTempSensor) + 40;
  tempOuterP40 = sensors.getTempC(outerTempSensor) + 40;

  uint8_t data3201[8] = { 0 };
  memcpy(data3201, &rawECT, 2);
  memcpy(data3201 + 2, &rawFuel, 2);
  memcpy(data3201 + 4, &voltageX10, 1);
  memcpy(data3201 + 5, &tempInterP40, 1);
  memcpy(data3201 + 6, &tempOuterP40, 1);

  // Отправка в RealDash
  sendRealDashFrame(3201, data3201);

  //Вывод медленных данных в Монитор порта
  TEST_SERIAL.println(F("----------------------------------"));
  sendTimeToSerial();
  TEST_SERIAL.print(F(" VOLTAGE: "));
  TEST_SERIAL.print(batteryVoltage, 2);
  TEST_SERIAL.print(F(" V"));
  TEST_SERIAL.print(F(" | ДТОЖ ADC: "));
  TEST_SERIAL.print(rawECT);
  TEST_SERIAL.print(F(" | ДУТ ADC: "));
  TEST_SERIAL.println(rawFuel);
  TEST_SERIAL.print(F(" | Салон: "));
  TEST_SERIAL.print((tempInterP40 - 40), 1);
  TEST_SERIAL.println(F(" °C"));
  TEST_SERIAL.print(F(" | Улица: "));
  TEST_SERIAL.print((tempOuterP40 - 40), 1);
  TEST_SERIAL.println(F(" °C"));
  TEST_SERIAL.println(F("-----------------------------------"));
}
// =========================================================================
// --- ФУНКЦИЯ НЕБЛОКИРУЮЩЕГО ЧТЕНИЯ ДАННЫХ ОТ ЭБУ ГБО ---
// =========================================================================
void collectGboData() {

  while (GBO_SERIAL.available() > 0) {
    uint8_t inByte = GBO_SERIAL.read();

    switch (gboState) {
      case WAIT_HEADER:
        if (inByte == 0xF0) gboState = WAIT_0;
        break;

      case WAIT_0:
        if (inByte == 0x00) gboState = WAIT_SIZE;
        else gboState = WAIT_HEADER;
        break;

      case WAIT_SIZE:
        if (inByte == 0x53) gboState = WAIT_TYPE;
        else gboState = WAIT_HEADER;
        break;

      case WAIT_TYPE:
        if (inByte == 0x01) gboState = READ_PAYLOAD;
        else gboState = WAIT_HEADER;
        break;

      case READ_PAYLOAD:
        gboBuf[gboIndex++] = inByte;
        // Если собрали все 83 байта (индексы от 0 до 82)
        if (gboIndex == GBO_PACKET_SIZE) {
          // --- УСПЕШНО ПОЛУЧЕН ВЕСЬ ПАКЕТ ДАННЫХ (83 байта) ---
          parseGboBuffer();
          gboState = WAIT_HEADER;  // Сбрасываем автомат для следующего пакета
          gboIndex = 0;
        }
        break;
    }
  }
}
// =========================================================================
// --- Функция обработки данных от ЭБУ ГБО  ---
// =========================================================================
void parseGboBuffer() {
  unsigned long currentMillis = millis();
  unsigned long deltaTimeGboMs = currentMillis - timerGboInj;
  timerGboInj = currentMillis;

  rpmGBO = (gboBuf[38] * 100) + gboBuf[39];
  // 1 впрыск происходит за 2 оборота
  // результат = сумма впрысков * (rpm за интервал / 2)
  // rpm за интервал = rpm / 60000 * интервал в мс
  // результатХ10 = (сумма впрысков * 10) * rpm / 60000 * интервал в мс * 1/2
  // результатХ10 = ((сумма впрысков * 10) * rpm * интервал в мс ) / 120000
  // математическое округление: (((сумма впрысков * 10) * rpm * интервал в мс) + (120000 / 2)) / 120000
  // результат = (((сумма впрысков * 10) * rpm * интервал в мс) + (600000)) / 1200000
  uint16_t sumInjPetrolX10 = gboBuf[6] + gboBuf[8] + gboBuf[10] + gboBuf[12];  // ms*10
  uint16_t sumInjLpgX10 = gboBuf[22] + gboBuf[24] + gboBuf[26] + gboBuf[28];   // ms*10

  if (sumInjPetrolX10 > 0) {
    uint64_t numeratorPetrolX10 = (uint64_t)sumInjPetrolX10 * rpmGBO * deltaTimeGboMs;
    sumInjPetrolMs = (numeratorPetrolX10 + 600000ULL) / 1200000ULL;
    while (sumInjPetrolMs >= 1000) {
      activePetrol.totalInjSeconds++;
      sumInjPetrolMs -= 1000;
    }
  }
  if (sumInjLpgX10 > 0) {
    uint64_t numeratorLpgX10 = (uint64_t)sumInjLpgX10 * rpmGBO * deltaTimeGboMs;
    sumInjLpgMs = (numeratorLpgX10 + 600000ULL) / 1200000ULL;
    while (sumInjLpgMs >= 1000) {
      activeLpg.totalInjSeconds++;
      sumInjLpgMs -= 1000;
    }
  }

  if (rpmGBO > 400 && sumInjLpgX10 > 8) isLpgActive = true;
  else isLpgActive = false;  // Едем на бензине (или мотор заглох)


  // --- Кадр 3203 (Все форсунки: 4 бензин + 4 газ) ---
  uint8_t frameInjections[8] = {
    gboBuf[6], gboBuf[8], gboBuf[10], gboBuf[12],   // Бензин 1-4
    gboBuf[22], gboBuf[24], gboBuf[26], gboBuf[28]  // Газ 1-4
  };
  sendRealDashFrame(3203, frameInjections);

  // --- Кадр 3204 (Двигатель, Давления, Температуры) ---
  uint8_t frameEngine[8] = {
    (uint8_t)(rpmGBO & 0xFF),         // Обороты (низший байт) - смещение 0
    (uint8_t)((rpmGBO >> 8) & 0xFF),  // Обороты (высший байт) - смещение 1
    gboBuf[41],                       // pressGboX100          - смещение 2
    gboBuf[43],                       // pressMapX100          - смещение 3
    gboBuf[44],                       // tempRed               - смещение 4
    gboBuf[45],                       // tempGas               - смещение 5
    gboBuf[62],                       // нагрузка на двигатель - смещение 6
    0x00                              // Запас                 - смещения 7
  };
  sendRealDashFrame(3204, frameEngine);
}
// == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == =
// --- Функция проверки входяших данных от RealDash ---
// =========================================================================
void checkIncomingRealDashCAN() {
  if (Serial.available() >= 16) {
    if (Serial.read() == 0x44 && Serial.read() == 0x33 && Serial.read() == 0x22 && Serial.read() == 0x11) {
      uint32_t canId;
      Serial.readBytes((char*)&canId, 4);
      uint8_t frameData[8];
      Serial.readBytes((char*)frameData, 8);

      if (canId == 0x300) {               // данные по поездкам
        uint8_t fuelType = frameData[0];  // 1 - Бензин, 2 - Газ
        uint8_t command = frameData[1];   // 1 - Заправка
        float fuelAmount;
        memcpy(&fuelAmount, &frameData[2], 4);

        if (command == 1) handleRefuelCommand(fuelType, fuelAmount);
      }
    }
  }
}
// =========================================================================
// --- Функция обработки команды (1) закрытия текущей и открытия новой  ---
// --- поездки при заправке топлива                                     ---
// =========================================================================
void handleRefuelCommand(uint8_t fuelType, float fuelAmount) {
  if (fuelType == 2) {
    activeLpg.status = 1;  // Закрыта
    activeLpg.endTime = systemUnixTime;
    activeLpg.crc = calculateCRC8((uint8_t*)&activeLpg, sizeof(activeLpg) - 1);
    fram.write(FRAM_HISTORY_START + (header.currentLpgIndex * sizeof(HistoryRecord)), (uint8_t*)&activeLpg, sizeof(activeLpg));
    header.currentLpgIndex = header.nextFreeIndex;
    memset(&activeLpg, 0, sizeof(activeLpg));
    activeLpg.startTime = systemUnixTime;
    activeLpg.endTime = systemUnixTime;
    activeLpg.initialFuelLiters = fuelAmount;
    activeLpg.fuelType = 2;
  } else if (fuelType == 1) {
    activePetrol.status = 1;
    activePetrol.endTime = systemUnixTime;
    activePetrol.crc = calculateCRC8((uint8_t*)&activePetrol, sizeof(activePetrol) - 1);
    fram.write(FRAM_HISTORY_START + (header.currentPetrolIndex * sizeof(HistoryRecord)), (uint8_t*)&activePetrol, sizeof(activePetrol));
    header.currentPetrolIndex = header.nextFreeIndex;
    memset(&activePetrol, 0, sizeof(activePetrol));
    activePetrol.startTime = systemUnixTime;
    activePetrol.endTime = systemUnixTime;
    activePetrol.initialFuelLiters = fuelAmount;
    activePetrol.fuelType = 1;
  }
  header.nextFreeIndex = (header.nextFreeIndex + 1) % MAX_HISTORY_RECORDS;
  header.crc = calculateCRC8((uint8_t*)&header, sizeof(header) - 1);
  fram.write(FRAM_SYS_MAIN, (uint8_t*)&header, sizeof(header));
  fram.write(FRAM_SYS_MIRROR, (uint8_t*)&header, sizeof(header));
}
// =========================================================================
// --- Функция синхронизация времени с RTC-часами ---
// =========================================================================
void syncWithPhysicalRTC() {
  if (Wire.getWireTimeoutFlag()) {
    Wire.clearWireTimeoutFlag();
    isRtcConnected = false;
    triggerRealDashAlert(3);
    return;
  }
  Wire.beginTransmission(0x68);
  if (Wire.endTransmission() == 0) {
    uint32_t exactTime = rtc.now().unixtime();
    if (abs((long)(systemUnixTime - exactTime)) > 5) systemUnixTime = exactTime;
    isRtcConnected = true;
  } else {
    isRtcConnected = false;
    triggerRealDashAlert(3);
  }
}
// =========================================================================
// --- Функция инициализации и проверки памяти ---
// =========================================================================
void systemMemoryInitAndValidate() {
  bool sysMainOk = false, sysMirrorOk = false;
  bool odoMainOk = false, odoMirrorOk = false;

  fram.read(FRAM_SYS_MAIN, (uint8_t*)&header, sizeof(header));

  if (header.signature == FRAM_SIGNATURE && calculateCRC8((uint8_t*)&header, sizeof(header) - 1) == header.crc) sysMainOk = true;

  SysHeader backupHeader;
  fram.read(FRAM_SYS_MIRROR, (uint8_t*)&backupHeader, sizeof(backupHeader));

  if (backupHeader.signature == FRAM_SIGNATURE && calculateCRC8((uint8_t*)&backupHeader, sizeof(backupHeader) - 1) == backupHeader.crc) sysMirrorOk = true;

  if (sysMainOk && !sysMirrorOk) fram.write(FRAM_SYS_MIRROR, (uint8_t*)&header, sizeof(header));
  else if (!sysMainOk && sysMirrorOk) {
    header = backupHeader;
    fram.write(FRAM_SYS_MAIN, (uint8_t*)&header, sizeof(header));
  } else if (!sysMainOk && !sysMirrorOk) {
    header = { FRAM_SIGNATURE, 0, 1, 2, 0 };
    header.crc = calculateCRC8((uint8_t*)&header, sizeof(header) - 1);
    fram.write(FRAM_SYS_MAIN, (uint8_t*)&header, sizeof(header));
    fram.write(FRAM_SYS_MIRROR, (uint8_t*)&header, sizeof(header));
  }

  fram.read(FRAM_ODO_MAIN, (uint8_t*)&odo, sizeof(odo));

  if (calculateCRC8((uint8_t*)&odo, sizeof(odo) - 1) == odo.crc) odoMainOk = true;

  OdoData backupOdo;
  fram.read(FRAM_ODO_MIRROR, (uint8_t*)&backupOdo, sizeof(backupOdo));

  if (calculateCRC8((uint8_t*)&backupOdo, sizeof(backupOdo) - 1) == backupOdo.crc) odoMirrorOk = true;

  if (odoMainOk && !odoMirrorOk) {
    fram.write(FRAM_ODO_MIRROR, (uint8_t*)&odo, sizeof(odo));
  } else if (!odoMainOk && odoMirrorOk) {
    odo = backupOdo;
    fram.write(FRAM_ODO_MAIN, (uint8_t*)&odo, sizeof(odo));
  } else if (!odoMainOk && !odoMirrorOk) {
    memset(&odo, 0, sizeof(odo));
    odo.crc = calculateCRC8((uint8_t*)&odo, sizeof(odo) - 1);
    fram.write(FRAM_ODO_MAIN, (uint8_t*)&odo, sizeof(odo));
    fram.write(FRAM_ODO_MIRROR, (uint8_t*)&odo, sizeof(odo));
    triggerRealDashAlert(2);  // Ошибка: Одометры сброшены
  }
  validateActiveSession(2);
  validateActiveSession(1);
}
// =========================================================================
// --- Функция проверки активного сеанса поездок  ---
// =========================================================================
void validateActiveSession(uint8_t fuelType) {
  uint16_t currentIndex = (fuelType == 2) ? header.currentLpgIndex : header.currentPetrolIndex;
  uint32_t addr = FRAM_HISTORY_START + (currentIndex * sizeof(HistoryRecord));

  HistoryRecord tempRecord;
  fram.read(addr, (uint8_t*)&tempRecord, sizeof(tempRecord));

  if (calculateCRC8((uint8_t*)&tempRecord, sizeof(tempRecord) - 1) != tempRecord.crc) {
    tempRecord.status = 2;  // Корректная изоляция сбоя
    tempRecord.crc = calculateCRC8((uint8_t*)&tempRecord, sizeof(tempRecord) - 1);
    fram.write(addr, (uint8_t*)&tempRecord, sizeof(tempRecord));

    uint16_t newIndex = header.nextFreeIndex;
    header.nextFreeIndex = (header.nextFreeIndex + 1) % MAX_HISTORY_RECORDS;

    if (fuelType == 2) header.currentLpgIndex = newIndex;
    else header.currentPetrolIndex = newIndex;

    header.crc = calculateCRC8((uint8_t*)&header, sizeof(header) - 1);
    fram.write(FRAM_SYS_MAIN, (uint8_t*)&header, sizeof(header));
    fram.write(FRAM_SYS_MIRROR, (uint8_t*)&header, sizeof(header));

    HistoryRecord* active = (fuelType == 2) ? &activeLpg : &activePetrol;
    memset(active, 0, sizeof(HistoryRecord));
    active->startTime = systemUnixTime;
    active->endTime = systemUnixTime;
    active->fuelType = fuelType;
    active->initialFuelLiters = 0;  //= (fuelType == 2) ? 40.0f : 50.0f;
    triggerRealDashAlert(1);        // Предупреждение: Сессия сброшена помехой
  } else {
    if (fuelType == 2) activeLpg = tempRecord;
    else activePetrol = tempRecord;
  }
}
// =========================================================================
// --- Функция автоматического бэкапа текущих данных во FRAM ---
// =========================================================================
void backupDataToFram() {

  odo.crc = calculateCRC8((uint8_t*)&odo, sizeof(odo) - 1);
  fram.write(FRAM_ODO_MAIN, (uint8_t*)&odo, sizeof(odo));
  fram.write(FRAM_ODO_MIRROR, (uint8_t*)&odo, sizeof(odo));

  activeLpg.endTime = systemUnixTime;
  activeLpg.crc = calculateCRC8((uint8_t*)&activeLpg, sizeof(activeLpg) - 1);
  fram.write(FRAM_HISTORY_START + (header.currentLpgIndex * sizeof(HistoryRecord)), (uint8_t*)&activeLpg, sizeof(activeLpg));

  activePetrol.endTime = systemUnixTime;
  activePetrol.crc = calculateCRC8((uint8_t*)&activePetrol, sizeof(activePetrol) - 1);
  fram.write(FRAM_HISTORY_START + (header.currentPetrolIndex * sizeof(HistoryRecord)), (uint8_t*)&activePetrol, sizeof(activePetrol));
}
// =========================================================================
// --- Функция отправки сообшения об ошибке в RealDash ---
// =========================================================================
void triggerRealDashAlert(uint8_t errorCode) {
  if (errorCode == 1) {
    TEST_SERIAL.println(F("Ошибка (код 1): Данные поездки сброшены"));
    // sendRealDashText(3210, "Ошибка: Данные поездки сброшены");
  }
  if (errorCode == 2) {
    TEST_SERIAL.println(F("Ошибка (код 2): Одометры сброшены"));
    // sendRealDashText(3210, "Ошибка: Одометры сброшены");
  }
  if (errorCode == 3) {
    TEST_SERIAL.println(F("Ошибка (код 3): Проблема с часами"));
    // sendRealDashText(3210, "Ошибка: часы перенастроены");
  }

  uint8_t packet[8] = { errorCode, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  sendRealDashFrame(3211, packet);
}
// =========================================================================
// --- Функция проверки событий выхода из сна ---
// =========================================================================
void checkWakeUpTriggers() {
  if (wakeUpClickCount > 0) {
    // Перенастраиваем прерывание на FALLING (спад сигнала) с другой функцией.
    attachInterrupt(digitalPinToInterrupt(PIN_DOOR_TRIGGER), doorTriggerISRWork, FALLING);
    unsigned long windowTimer = millis();
    int currentClicks = wakeUpClickCount;

    // Ждем клики в течение заданного интервала (TIMEOUT_WAIT_CLICK)
    while (millis() - windowTimer < TIMEOUT_WAIT_CLICK) {
      if (wakeUpClickCount != currentClicks) {
        currentClicks = wakeUpClickCount;
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
      float batteryVoltage = readBatteryVoltageX10() * 0.1;

      if (batteryVoltage > CRITICAL_BATTERY_VOLTAGE) {
        // handleSmoothStart();  // ВКЛЮЧАЕМ BTS442
        currentState = STATE_PRE_DRIVE_WAKE;
        wakeUpTimerStart = millis();
      } else {
        TEST_SERIAL.println(F("Батарея разряжена..."));
        TEST_SERIAL.flush();
      }
    } else {
      TEST_SERIAL.println(F("Кликов больше двух. Идем спать..."));
      TEST_SERIAL.flush();
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
  // setTime(12, 0, 0, 1, 1, 2026);

  TEST_SERIAL.begin(115200);
  pinMode(17, INPUT_PULLUP);  // Подтяжка RX линии TEST_SERIAL
  delay(100);

  GBO_SERIAL.begin(9600);
  pinMode(15, INPUT_PULLUP);  // Подтяжка RX линии GBO_SERIAL
  delay(100);

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

  Wire.begin();                     // Инициализация I2C шины для Mega (пины 20 и 21 инициализируются автоматически)
  Wire.setWireTimeout(3000, true);  // Безопасный запуск I2C с аппаратным таймаутом

  // Инициализация службы времени
  Wire.beginTransmission(0x68);
  if (Wire.endTransmission() == 0 && rtc.begin()) {
    isRtcConnected = true;
    systemUnixTime = rtc.lostPower() ? 1767225600 : rtc.now().unixtime();
  } else {
    systemUnixTime = 1767225600;  // Безопасный старт при отказе RTC
    triggerRealDashAlert(3);      // Код 3: Ошибка часов
  }
  lastClockTick = millis();
  lastRtcSync = millis();

  if (fram.begin()) {  // Запуск и валидация памяти FRAM
    systemMemoryInitAndValidate();
  }
  // if (!rtc.begin()) {  // 1. Проверка RTC DS3231
  //   TEST_SERIAL.println(F("Не удалось найти модуль RTC!"));
  //   isRtcConnected = false;
  //   setTime(12, 0, 0, 1, 10, 2026);
  // }

  // if (rtc.lostPower()) {  // Если RTC потерял питание, устанавливаем время компиляции
  //   TEST_SERIAL.println(F("RTC потерял питание, устанавливаем время..."));
  //   rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // }

  // if (!fram.begin(0x50)) {  // 2. Проверка FRAM MB85RC256V. По умолчанию адрес чипа 0x50 (если выводы A0, A1, A2 замкнуты на GND)
  //   TEST_SERIAL.println(F("Не удалось найти память FRAM!"));
  // }
  // delay(100);

  // currentState = STATE_PRE_DRIVE_WAKE;
  wakeUpTimerStart = millis();
}

// *************************************************************************
// *************************         LOOP          *************************
// *************************************************************************

void loop() {
  unsigned long currentMillis = millis();

  // Аппаратный секундный шаг внутренних "живых" часов
  if (currentMillis - (lastClockTick * 1000) >= 1000) {
    systemUnixTime++;
    lastClockTick++;
    if (!isLpgActive && activePetrol.status == 0) activePetrol.durationSeconds++;
    if (isLpgActive && activeLpg.status == 0) activeLpg.durationSeconds++;
  }
  // Фоновая синхронизация с физическим чипом RTC (Раз в 5 минут)
  if (currentMillis - lastRtcSync >= 300000) {
    lastRtcSync = currentMillis;
    syncWithPhysicalRTC();
  }

  // =========================================================================
  // --- АВТОМАТ СОСТОЯНИЙ ПИТАНИЯ ---
  // =========================================================================

  // Оптопара инвертирует сигнал: когда на ней +12В, пин Ардуино притягивается к GND (LOW)
  bool isIgnitionOn = (digitalRead(indicators[0].pin) == LOW);

  switch (currentState) {

    case STATE_PRE_DRIVE_WAKE:

      if (digitalRead(PIN_ACC_OUTPUT) == LOW) handleSmoothStart();

      checkIncomingRealDashCAN();  // Слушаем входящие команды из RealDash

      // БЫСТРЫЕ ДАТЧИКИ
      if (currentMillis - timerFastSensors >= INTERVAL_FAST) {
        // Вычисляем точное время, прошедшее с момента последнего расчета
        unsigned long timeElapsed = currentMillis - timerFastSensors;
        timerFastSensors = currentMillis;
        collectFastData(timeElapsed);
      }
      // СТАНДАРТНЫЕ ДАТЧИКИ
      if (currentMillis - timerNormSensors >= INTERVAL_NORM) {
        timerNormSensors = currentMillis;
        collectNormData();
      }
      // МЕДЛЕННЫЕ ДАТЧИКИ
      if (currentMillis - timerSlowSensors >= INTERVAL_SLOW) {
        timerSlowSensors = currentMillis;
        collectSlowData();
      }
      if (currentMillis - lastSaveTime >= INTERVAL_BACKUP_FRAM) {
        lastSaveTime = currentMillis;
        backupDataToFram();
      }
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
      // ЗАПРОС В ЭБУ ГБО
      if (currentMillis - timerGboRequest >= INTERVAL_GBO_REQUEST) {
        // Мгновенная отправка 4 байт, не блокирующая основной цикл
        GBO_SERIAL.write(GBO_QUERY, sizeof(GBO_QUERY));
        timerGboRequest = currentMillis;
      }
      collectGboData();

      checkIncomingRealDashCAN();  // Слушаем входящие команды из RealDash

      // БЫСТРЫЕ ДАТЧИКИ
      if (currentMillis - timerFastSensors >= INTERVAL_FAST) {
        // Вычисляем точное время, прошедшее с момента последнего расчета
        unsigned long timeElapsed = currentMillis - timerFastSensors;
        timerFastSensors = currentMillis;
        collectFastData(timeElapsed);
      }
      // СТАНДАРТНЫЕ ДАТЧИКИ
      if (currentMillis - timerNormSensors >= INTERVAL_NORM) {
        timerNormSensors = currentMillis;
        collectNormData();
      }
      // МЕДЛЕННЫЕ ДАТЧИКИ
      if (currentMillis - timerSlowSensors >= INTERVAL_SLOW) {
        timerSlowSensors = currentMillis;
        collectSlowData();
      }
      if (currentMillis - lastSaveTime >= INTERVAL_BACKUP_FRAM) {
        lastSaveTime = currentMillis;
        backupDataToFram();
      }
      // В режиме поездки нам плевать на любые клацанья ЦЗ или дверей. Мы смотрим только на зажигание.
      if (!isIgnitionOn) {
        currentState = STATE_SHUTDOWN;
        TEST_SERIAL.println(F("Состояние изменено на SHUTDOWN. Машина выключена"));
      }

      break;

    case STATE_SHUTDOWN:

      TEST_SERIAL.println(F("Состояние изменено на SLEEP! Выключаем питание Андроид..."));
      TEST_SERIAL.flush();
      digitalWrite(PIN_ACC_OUTPUT, LOW);  // Обесточиваем магнитолу через BTS442
      delay(500);                         // Короткая пауза для записи кэша магнитолы
      currentState = STATE_SLEEP;

      break;

    case STATE_SLEEP:

      // Сбрасываем счетчик секунд сна перед погружением
      sleepSecondsCount = 0;
      wakeUpClickCount = 0;
      // Настраиваем прерывания
      attachInterrupt(digitalPinToInterrupt(indicators[0].pin), wakeUpISR, LOW);
      attachInterrupt(digitalPinToInterrupt(PIN_DOOR_TRIGGER), doorTriggerISRSleep, LOW);

      byte old_ADCSRA = powerDisable();

      if (!isRtcConnected) {
        // ЦИКЛ ГЛУБОКОГО СНА
        while (!wakeUpIgnition && wakeUpClickCount == 0) {

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

      powerEnable(old_ADCSRA);

      detachInterrupt(digitalPinToInterrupt(indicators[0].pin));  //зажигание
      detachInterrupt(digitalPinToInterrupt(PIN_DOOR_TRIGGER));

      // Корректируем время в TimeLib на то количество секунд, что мы спали
      if (!isRtcConnected) {
        adjustTime(sleepSecondsCount);
        systemUnixTime = now();
        lastClockTick = systemUnixTime + 1;
      }
      checkWakeUpTriggers();

      break;
  }
}
// <?xml version="1.0" encoding="utf-8"?>
// <RealDashCAN version="2">
//   <frames>
//     <!-- Кадр 3200: Обычные датчики и cигналы -->
//     <frame id="3200">
//       <!-- Лампы по ПЛЮСУ  -->
//       <value name="Indicator: ACC Ignition"      startbit="0" bitcount="1"></value> <!-- Зажигание -->
//       <value name="Indicator: Turn Signal Left"  startbit="1" bitcount="1"></value> <!-- Левый поворотник -->
//       <value name="Indicator: Turn Signal Right" startbit="2" bitcount="1"></value> <!-- Правый поворотник -->
//       <value name="Indicator: High Beam"         startbit="3" bitcount="1"></value> <!-- Дальний свет -->
//       <value name="Indicator: Low Beam"          startbit="4" bitcount="1"></value> <!-- Ближний свет -->
//       <value name="Indicator: Front Fog Lights"  startbit="5" bitcount="1"></value> <!-- Передние ПТФ -->
//       <value name="Indicator: Rear Fog Lights"   startbit="6" bitcount="1"></value> <!-- Задний ПТФ -->
//       <value name="Indicator: Parking Lights"    startbit="7" bitcount="1"></value> <!-- Габариты -->
//       <value name="Indicator: P_10"              startbit="8" bitcount="1"></value> <!-- Indicator: P_10 -->
//       <!-- Лампы по МИНУСУ  -->
//       <value name="Indicator: Oil Pressure"      startbit="9"  bitcount="1"></value> <!-- Давление масла -->
//       <value name="Indicator: Hand Brake"        startbit="10" bitcount="1"></value> <!-- Ручник / Тормозуха -->
//       <value name="Indicator: Check Engine"      startbit="11" bitcount="1"></value> <!-- Check Engine -->
//       <value name="Indicator: Battery Charging"  startbit="12" bitcount="1"></value> <!-- Аккумулятор / Зарядка -->
//       <value name="Indicator: ABS Warning"       startbit="13" bitcount="1"></value> <!-- ABS -->
//       <value name="Indicator: Coolant Warning"   startbit="14" bitcount="1"></value> <!-- Перегрев ОЖ -->
//       <value name="Indicator: Seat Belt Warning" startbit="15" bitcount="1"></value> <!-- Ремень безопасности -->
//       <value name="Indicator: Doors Open"        startbit="16" bitcount="1"></value> <!-- Двери открыты (Общий) -->
//       <value name="Indicator: Airbag Warning"    startbit="17" bitcount="1"></value> <!-- Подушки безопасности (SRS) -->
//       <value name="Indicator: Immobilizer"       startbit="18" bitcount="1"></value> <!-- Иммобилайзер (Красный диод) -->
//       <value name="Indicator: Saw (Electronics)" startbit="19" bitcount="1"></value> <!-- Пила -->

//       <value targetId="37" offset="4" length="2" units="RPM"></value> <!-- ENGINE/ECU INPUTS / RPM - Обороты -->
//       <value targetId="64" offset="6" length="2" units="km/h"></value> <!-- ENGINE/ECU INPUTS / Vehicle Speed - Скорость -->
//     </frame>

//     <!-- Кадр 3201: Медленные датчики -->
//     <frame id="3201">
//       <value name="Indicator: Coolant Temperature" offset="0" length="2" conversion="V/1023*90"></value> <!-- ТОЖ Формула временно -->
//       <value name="Indicator: Fuel Level"          offset="2" length="2" conversion="V/1023*20"></value> <!-- Топливо Формула временно -->
//       <value name="Indicator: Battery Voltage"     offset="4" length="1" conversion="V/10"></value> <!-- Батарея -->
//       <value name="Indicator: Temperature Inter"   offset="5" length="1" conversion="V-40"></value> <!-- Температура Салон -->
//       <value name="Indicator: Temperature Outer"   offset="6" length="1" conversion="V-40"></value> <!-- Температура Улица -->
//     </frame>
//     <!-- Кадр 3203: Данные ГБО форсунки -->
//     <frame id="3203">
//       <value name="ГБО: Впрыск Бензин 1" offset="0" length="1" conversion="V/10" units="ms"></value>
//       <value name="ГБО: Впрыск Бензин 2" offset="1" length="1" conversion="V/10" units="ms"></value>
//       <value name="ГБО: Впрыск Бензин 3" offset="2" length="1" conversion="V/10" units="ms"></value>
//       <value name="ГБО: Впрыск Бензин 4" offset="3" length="1" conversion="V/10" units="ms"></value>
//       <value name="ГБО: Впрыск Газ 1"    offset="4" length="1" conversion="V/10" units="ms"></value>
//       <value name="ГБО: Впрыск Газ 2"    offset="5" length="1" conversion="V/10" units="ms"></value>
//       <value name="ГБО: Впрыск Газ 3"    offset="6" length="1" conversion="V/10" units="ms"></value>
//       <value name="ГБО: Впрыск Газ 4"    offset="7" length="1" conversion="V/10" units="ms"></value>
//     </frame>
//     <!-- Кадр 3204: Данные ГБО двигатель -->
//     <frame id="3204">
//       <!-- Обороты привязываем к стандартному targetId="37" (Обороты двигателя в RealDash) -->
//       <value name="ГБО: RPM"                offset="0" length="2" conversion="V" units="rpm"></value>

//       <!-- Давление газа и MAP -->
//       <value name="ГБО: Давление Газа"      offset="2" length="1" conversion="V/100" units="bar"></value>
//       <value name="ГБО: Давление MAP"       offset="3" length="1" conversion="V/100" units="bar"></value>

//       <!-- Температура Редуктора, Газа-->
//       <value name="ГБО: Темп. Редуктора"    offset="4" length="1" conversion="V" units="C"></value>
//       <value name="ГБО: Темп. Газа"         offset="5" length="1" conversion="V" units="C"></value>
//       <value name="ГБО: Нагрузка двигателя" offset="6" length="1" conversion="V" units="%"></value>
//     </frame>

//     <frame id="3210">
//       <value name="Arduino: Message" units="text"></value>
//     </frame>
//   </frames>
// </RealDashCAN>

// Примечание: Параметр targetId — это внутренний уникальный номер датчика в экосистеме RealDash
// (например, 37 — это всегда RPM, а 12 — вольтаж батареи). Полный список этих ID есть на официальном сайте RealDash.
