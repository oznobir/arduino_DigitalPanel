#include <Arduino.h>
#include <SPI.h>                // Библиотека для работы с SPI
#include <mcp_can.h>            // Библиотека для работы с CAN MC2515
#include <OneWire.h>            // Библиотека для работы с 1-Wire
#include <DallasTemperature.h>  // Библиотека для работы с цифровыми датчиками
#include <avr/sleep.h>          // Библиотека сна МК AVR
#include <avr/power.h>          // Библиотека питания МК AVR
#include <avr/wdt.h>            // Библиотека сторожевого таймера МК AVR
#include <Wire.h>               // Библиотека для работы с I2C
#include <RTClib.h>             // Библиотека для работы с RTC DS3231
#include <Adafruit_FRAM_I2C.h>  // Библиотека для работы со fram-памятью MB85RC256V
#include <AltSoftSerial.h>      // Библиотека программных портов Serial

// =========================================================================
// АВТОМОБИЛЬНЫЙ КОНТРОЛЛЕР ПИТАНИЯ И ДАТЧИКОВ (REALDASH CAN ВЕРСИЯ)
// Arduino Mega Pro. Посекундные таймеры + Бинарный протокол RealDash CAN
// =========================================================================

#define RD_SERIAL Serial     // Порт к RealDash
#define TEST_SERIAL Serial2  // Порт для тестов PuTTY
#define TW2_SERIAL Serial3   // Порт к Tanix W2 через преобразователь 5v <=> 3.3v
// #define GBO_SERIAL Serial3  // Порт к ЭБУ ГБО
// #define PIN_RX_GBO = 15
AltSoftSerial GBO_SERIAL;
// ==========================================================================
// --- КОНФИГУРАЦИЯ ПИНОВ ---
// ==========================================================================
const byte PIN_FUEL = A0;           // ДУТ (резистор 330 Ом)
const byte PIN_ECT = A1;            // ДТОЖ (резистор 330 Ом)
const byte PIN_BATTERY_SENSE = A2;  // Вольтметр (делитель 10кОм и 3.3кОм)
const byte PIN_RTC = A8;            // Пин прерывания для будильника SQW, при смене необходимо поменять ручную настройку регистров и ISR()
const byte PIN_RPM = 2;             // Вход RPM через PC817 (Прерывание 0)
const byte PIN_SPEED = 3;           // Вход Скорости через PC817 (Прерывание 1)
const byte PIN_ACC_OUTPUT = 4;      // Цифровой выход: управление ключом BTS442 (ACC магнитолы)
const byte PIN_ONE_WIRE = 5;        // Датчики температуры
const byte PIN_SPI_CS = 6;          // CS SPI модуля MC2515
const byte PIN_DOOR_TRIGGER = 18;   // Цифровой вход (через оптопару): сигнал ЦЗ (Прерывание при включении)

OneWire oneWire(PIN_ONE_WIRE);
DallasTemperature sensors(&oneWire);
// Прописываем уникальные ID адреса датчиков
DeviceAddress interTempSensor = { 0x28, 0x79, 0xF4, 0xC8, 0x00, 0x00, 0x00, 0x8C };
DeviceAddress outerTempSensor = { 0x28, 0xC5, 0xD7, 0xC9, 0x00, 0x00, 0x00, 0xCF };
RTC_DS3231 rtc;
RTC_Millis rtcBackup;
Adafruit_FRAM_I2C fram = Adafruit_FRAM_I2C();
MCP_CAN CAN(PIN_SPI_CS);

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
  STATE_PRE_SLEEP,
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
  WAIT_TYPE,    // Ожидаем тип ответа (0x00)
  WAIT_SIZE,    // Ожидаем размер (0x53)
  WAIT_QUERY,   // Ожидаем тип ответа (0x01)
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
  uint16_t remainderFuelLitersX10;
  uint16_t initialFuelLitersX10;
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
bool isRtcConnected = false;

const float CRITICAL_BATTERY_VOLTAGE = 11.7;  // Порог защиты аккумулятора от разряда (Вольты)

const unsigned long TIMEOUT_WAIT_IGNITION = 10000;  // Время ожидания зажигания
const unsigned long TIMEOUT_WAIT_CLICK = 1000;      // Время ожидания сигнала ЦЗ
const unsigned long DEBOUNCE_DELAY = 250;           // Игнорируем помехи короче 250 мс
const unsigned long MAX_SHUTDOWN_TIMEOUT = 15000;
const unsigned long TIMEOUT_WAIT_UART_SILENCE = 2500;
const unsigned long TANIX_SLEEP_DURATION = 7200000;

volatile int wakeUpClickCount = 0;             // Переменная счетчика нажатий ЦЗ
volatile unsigned long sleepSecondsCount = 0;  // Переменная счетчика секунд, если RTC не работает
volatile bool wakeUpIgnition = false;          // Флаг того, что проснулись от зажигания
volatile bool wokenByRtc = false;              // Флаг того, что сработал будильник RTC
volatile unsigned long lastWorkDebounceTime = 0;

volatile unsigned long rpmPulses = 0;
volatile unsigned long speedPulses = 0;

unsigned long wakeUpTimerStart = 0;
unsigned long shutdownStartMillis = 0;
unsigned long timerFastSensors = 0;
unsigned long timerNormSensors = 0;
unsigned long timerSlowSensors = 0;
unsigned long timerGboRequest = 0;
unsigned long timerGboInj = 90;

unsigned long lastClockTick = 0;
unsigned long lastRtcSync = 0;
unsigned long lastSaveTime = 0;
unsigned long lastUartDataMillis = 0;

const unsigned long INTERVAL_FAST = 100;
const unsigned long INTERVAL_NORM = 1500;
const unsigned long INTERVAL_SLOW = 2000;
const unsigned long INTERVAL_GBO_REQUEST = 200;  // Опрашиваем ГБО
const unsigned long INTERVAL_BACKUP_FRAM = 10000;
const unsigned long INTERVAL_SYNC = 300000;

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
uint16_t remInjPetrolMs = 0;
uint16_t remInjLpgMs = 0;

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
byte sleepPeripherals() {
  RD_SERIAL.flush();
  TW2_SERIAL.flush();
  TEST_SERIAL.flush();
  // GBO_SERIAL.flush();
  RD_SERIAL.end();
  TW2_SERIAL.end();
  TEST_SERIAL.end();
  // GBO_SERIAL.end();

  Wire.end();

  byte old = ADCSRA;       // Сохраняем настройки АЦП
  ADCSRA &= ~(1 << ADEN);  // Отключаем АЦП
  power_adc_disable();

  // Выключаем аппаратные таймеры
  // power_timer0_disable(); // Лучше оставить
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
  power_usart2_disable();

  pinMode(0, INPUT);  // Убираем +5В с линии RX0
  pinMode(1, INPUT);  // Убираем +5В с линии TX0

  pinMode(14, INPUT);  // TX3 в Z-состояние
  pinMode(15, INPUT);  // RX3 в Z-состояние

  pinMode(16, INPUT);  // TX2 в Z-состояние
  pinMode(17, INPUT);  // RX2 в Z-состояние

  pinMode(20, INPUT);  // SDA
  pinMode(21, INPUT);  // SCL
  return old;
}
// =========================================================================
// --- Функция включения модулей Ардуино ---
// =========================================================================
void wakeUpPeripherals(const byte old) {
  power_adc_enable();
  ADCSRA = old;  // Включаем АЦП обратно (восстанавливаем настройки)
  // Включаем периферию обратно
  // power_timer0_enable();
  // delayMicroseconds(10);  // для millis()
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
  power_usart2_enable();

  RD_SERIAL.begin(115200);
  TW2_SERIAL.begin(115200);
  TEST_SERIAL.begin(115200);
  // GBO_SERIAL.begin(9600);
  // pinMode(PIN_RX_GBO, INPUT_PULLUP);  // Подтяжка RX линии GBO_SERIAL
  sensors.begin();
  Wire.begin();
}
// =========================================================================
// --- ФУНКЦИИ ОБРАБОТКИ ПРЕРЫВАНИЙ ---
// =========================================================================

// Обработчик прерывания (должен быть максимально коротким!)
void doorTriggerISRSleep() {
  wakeUpClickCount = 1;  // Нас разбудил первый клик. Фиксируем его.
}
void doorTriggerISRWork() {  // Считаем следующие нажатия ЦЗ
  unsigned long currentTime = micros();
  if (currentTime - lastWorkDebounceTime > (DEBOUNCE_DELAY * 1000UL)) {
    wakeUpClickCount++;
    lastWorkDebounceTime = currentTime;
  }
}
void wakeUpISR() {  // Прерывание появления зажигания
  wakeUpIgnition = true;
}
ISR(WDT_vect) {  // Прерывание сторожевого таймера (срабатывает раз в секунду во сне)
  sleepSecondsCount++;
}
ISR(PCINT2_vect) {  // Аппаратный вектор для группы будильника RTC от DS3231
  // Так как прерывание срабатывает на любое изменение, проверяем, что пин A8 действительно упал в LOW
  // if (digitalRead(PIN_RTC) == LOW) wokenByRtc = true;
  if ((PINK & (1 << PK0)) == 0) wokenByRtc = true;
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
  TEST_SERIAL.print(F(" LAMPS: "));

  byteIndicators = 0;
  for (int i = 0; i < INDICATORS_COUNT; i++) {
    if (digitalRead(indicators[i].pin) == LOW) {
      byteIndicators |= (1UL << i);
      TEST_SERIAL.print(indicators[i].name);
    }
  }
  TEST_SERIAL.println();  // Перенос строки в конце списка

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

  // Кадр 3207 (Остатки)
  // float curLpgLiters = max(0.0f, activeLpg.initialFuelLiters - ((float)activeLpg.totalInjMs * LPG_LITERS_PER_MS));
  // float curPetrolLiters = max(0.0f, activePetrol.initialFuelLiters - ((float)activePetrol.totalInjMs * PETROL_LITERS_PER_MS));
  // 12,5 л на 100 км
  uint16_t curLpgLitersX10 = max(0, activeLpg.remainderFuelLitersX10 + activeLpg.initialFuelLitersX10 - (uint16_t)(tripLpgKm * 125 + 500) / 1000);
  // 10,5 л на 100 км
  uint16_t curPetrolLitersX10 = max(0, activePetrol.remainderFuelLitersX10 + activePetrol.initialFuelLitersX10 - (uint16_t)(tripPetrolKm * 105 + 500) / 1000);

  uint8_t data3207[8] = { 0 };
  memcpy(data3207, &curLpgLitersX10, 2);
  memcpy(data3207 + 2, &curPetrolLitersX10, 2);

  // Отправка в RealDash
  sendRealDashFrame(3207, data3207);
}
// =========================================================================
// --- ФУНКЦИЯ ПОЛУЧЕНИЯ И ОТПРАВКИ МЕДЛЕННЫХ ДАННЫХ ---
// =========================================================================
void collectSlowData() {
  // Чтение вольтметра
  voltageX10 = readBatteryVoltageX10();
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

  float rawInter = sensors.getTempC(interTempSensor);
  float rawOuter = sensors.getTempC(outerTempSensor);

  tempInterP40 = (rawInter == DEVICE_DISCONNECTED_C || rawInter < -40.0) ? 0 : (uint8_t)(rawInter + 40.5);
  tempOuterP40 = (rawOuter == DEVICE_DISCONNECTED_C || rawOuter < -40.0) ? 0 : (uint8_t)(rawOuter + 40.5);
  sensors.requestTemperatures();  // Даем команду датчикам начать НОВОЕ измерение в фоне (займет 0 мс ожидания!)

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
  float batteryVoltage = voltageX10 * 0.1;
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
// --- ФУНКЦИЯ ОТПРАВКИ ДАННЫХ ПО ПОЗДКАМ ПОСЛЕ СНА В REALDASH ---
// =========================================================================
void sendTripsDataAfterWakeUp() {
}
// =========================================================================
// --- ФУНКЦИЯ НЕБЛОКИРУЮЩЕГО ЧТЕНИЯ ДАННЫХ ОТ ЭБУ ГБО ---
// =========================================================================
void collectGboData() {
  while (GBO_SERIAL.available() > 0) {
    uint8_t inByte = GBO_SERIAL.read();

    switch (gboState) {
      case WAIT_HEADER: gboState = (inByte == 0xF0) ? WAIT_TYPE : WAIT_HEADER; break;
      case WAIT_TYPE: gboState = (inByte == 0x00) ? WAIT_SIZE : WAIT_HEADER; break;
      case WAIT_SIZE: gboState = (inByte == 0x53) ? WAIT_QUERY : WAIT_HEADER; break;
      case WAIT_QUERY: gboState = (inByte == 0x01) ? READ_PAYLOAD : WAIT_HEADER; break;
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

  isLpgActive = (rpmGBO > 400 && sumInjLpgX10 > 8) ? true : false;  // Едем на бензине (или мотор заглох)

  if (!isLpgActive && sumInjPetrolX10 > 0) {
    uint64_t numeratorPetrolX10 = (uint64_t)sumInjPetrolX10 * rpmGBO * deltaTimeGboMs;
    remInjPetrolMs += (numeratorPetrolX10 + 600000ULL) / 1200000ULL;
    while (remInjPetrolMs >= 1000) {
      activePetrol.totalInjSeconds++;
      remInjPetrolMs -= 1000;
    }
  }
  if (isLpgActive && sumInjLpgX10 > 0) {
    uint64_t numeratorLpgX10 = (uint64_t)sumInjLpgX10 * rpmGBO * deltaTimeGboMs;
    remInjLpgMs += (numeratorLpgX10 + 600000ULL) / 1200000ULL;
    while (remInjLpgMs >= 1000) {
      activeLpg.totalInjSeconds++;
      remInjLpgMs -= 1000;
    }
  }
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
// =========================================================================
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
        uint8_t command = frameData[1];   // 1 - Новая заправка, 2 - Изменение данных текущей поездки
        uint16_t fuelRemainder;
        uint16_t fuelAmount;
        memcpy(&fuelRemainder, &frameData[2], 2);
        memcpy(&fuelAmount, &frameData[3], 2);

        if (command == 1) handleRefuelCommand(fuelType, fuelRemainder, fuelAmount);
      }
    }
  }
}
// =========================================================================
// --- Функция обработки команды (1) закрытия текущей и открытия новой  ---
// --- поездки при заправке топлива                                     ---
// =========================================================================
void handleRefuelCommand(uint8_t fuelType, uint16_t fuelRemainder, uint16_t fuelAmount) {
  if (fuelType == 2) {
    activeLpg.status = 1;  // Закрыта
    activeLpg.endTime = rtcBackup.now().unixtime();
    activeLpg.crc = calculateCRC8((uint8_t*)&activeLpg, sizeof(activeLpg) - 1);
    fram.write(FRAM_HISTORY_START + (header.currentLpgIndex * sizeof(HistoryRecord)), (uint8_t*)&activeLpg, sizeof(activeLpg));
    header.currentLpgIndex = header.nextFreeIndex;
    memset(&activeLpg, 0, sizeof(activeLpg));
    activeLpg.startTime = rtcBackup.now().unixtime();
    activeLpg.endTime = rtcBackup.now().unixtime();
    activeLpg.remainderFuelLitersX10 = fuelRemainder;
    activeLpg.initialFuelLitersX10 = fuelAmount;
    activeLpg.fuelType = 2;
  } else if (fuelType == 1) {
    activePetrol.status = 1;
    activePetrol.endTime = rtcBackup.now().unixtime();
    activePetrol.crc = calculateCRC8((uint8_t*)&activePetrol, sizeof(activePetrol) - 1);
    fram.write(FRAM_HISTORY_START + (header.currentPetrolIndex * sizeof(HistoryRecord)), (uint8_t*)&activePetrol, sizeof(activePetrol));
    header.currentPetrolIndex = header.nextFreeIndex;
    memset(&activePetrol, 0, sizeof(activePetrol));
    activePetrol.startTime = rtcBackup.now().unixtime();
    activePetrol.endTime = rtcBackup.now().unixtime();
    activePetrol.remainderFuelLitersX10 = fuelRemainder;
    activePetrol.initialFuelLitersX10 = fuelAmount;
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
void syncTimeManager(bool syncNow = false) {
  // Если мы только что проснулись из глубокого сна,
  // компенсируем софт-часы на время сна, посчитанное Watchdog
  if (sleepSecondsCount > 0) {
    uint32_t adjustedUnix = rtcBackup.now().unixtime() + sleepSecondsCount;
    rtcBackup.begin(DateTime(adjustedUnix));
    sleepSecondsCount = 0;  // Сброс счетчика сна
  }

  // Проверяем, пришло ли время физической синхронизации по таймеру (раз в 5 минут)
  if (millis() - lastRtcSync >= INTERVAL_SYNC || lastRtcSync == 0 || syncNow) {
    if (!syncNow) lastRtcSync = millis();
    if (Wire.getWireTimeoutFlag()) {
      Wire.clearWireTimeoutFlag();
      isRtcConnected = false;
      triggerRealDashAlert(3);
      return;
    }
    Wire.beginTransmission(0x68);
    if (Wire.endTransmission() == 0) {
      DateTime realTime = rtc.now();
      rtcBackup.begin(realTime);
      isRtcConnected = true;
      TEST_SERIAL.println(F("Успешная синхронизация с RTC."));
    } else {
      isRtcConnected = false;
      TEST_SERIAL.println(F("RTC перестал отвечать!"));
      triggerRealDashAlert(3);
    }
  }
}
// void syncWithPhysicalRTC() {
//   if (Wire.getWireTimeoutFlag()) {
//     Wire.clearWireTimeoutFlag();
//     isRtcConnected = false;
//     triggerRealDashAlert(3);
//     return;
//   }
//   Wire.beginTransmission(0x68);
//   if (Wire.endTransmission() == 0) {
//     uint32_t exactTime = rtc.now().unixtime();
//     rtcBackup.begin(exactTime);
//     isRtcConnected = true;
//   } else {
//     isRtcConnected = false;
//     triggerRealDashAlert(3);
//   }
// }
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
    active->startTime = rtcBackup.now().unixtime();
    active->endTime = rtcBackup.now().unixtime();
    active->fuelType = fuelType;
    active->remainderFuelLitersX10 = 0;
    active->initialFuelLitersX10 = 0;  //= (fuelType == 2) ? 40.0f : 50.0f;
    triggerRealDashAlert(1);           // Предупреждение: Сессия сброшена помехой
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

  activeLpg.endTime = rtcBackup.now().unixtime();
  activeLpg.crc = calculateCRC8((uint8_t*)&activeLpg, sizeof(activeLpg) - 1);
  fram.write(FRAM_HISTORY_START + (header.currentLpgIndex * sizeof(HistoryRecord)), (uint8_t*)&activeLpg, sizeof(activeLpg));

  activePetrol.endTime = rtcBackup.now().unixtime();
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
// ФУНКЦИЯ ПОГРУЖЕНИЯ В СОН
// useAlarm = true  -> спим с будильником (или WDT с лимитом времени) для STATE_PRE_SLEEP
// useAlarm = false -> спим бесконечно (до клика/зажигания) для STATE_SLEEP
// =========================================================================
void enterDeepSleep(bool useAlarm, uint32_t maxSeconds) {
  wakeUpIgnition = false;
  wakeUpClickCount = 0;
  wokenByRtc = false;
  sleepSecondsCount = 0;

  // Настраиваем RTC, пока шина I2C еще включена
  if (useAlarm && isRtcConnected) {
    DateTime alarmTime = rtcBackup.now() + TimeSpan(0, 0, 0, maxSeconds);
    rtc.clearAlarm(1);
    rtc.setAlarm1(alarmTime, DS3231_A1_Hour);
  }

  // Гасим всю периферию до входа в цикл сна
  byte old_ADCSRA = sleepPeripherals();

  // Цикл сна (для RTC выполнится 1 раз; для WDT будет тикать раз в секунду)
  while (!wakeUpIgnition && wakeUpClickCount == 0 && !wokenByRtc) {
    if (useAlarm && !isRtcConnected && sleepSecondsCount >= maxSeconds) {
      break;  // Вышли по таймеру Watchdog (приставка пора выключать)
    }

    cli();
    EIFR = (1 << INTF2) | (1 << INTF3);  // Сбрасываем старые флаги пинов 18 и 19
    attachInterrupt(digitalPinToInterrupt(indicators[0].pin), wakeUpISR, LOW);
    attachInterrupt(digitalPinToInterrupt(PIN_DOOR_TRIGGER), doorTriggerISRSleep, LOW);

    if (useAlarm && isRtcConnected) {
      PCICR |= (1 << PCIE2);     // Включаем прерывание группы A8-A15
      PCMSK2 |= (1 << PCINT16);  // Пин A8
    }

    // Если RTC не работает (в любом из стейтов) — включаем WDT для хода софт-часов
    if (!isRtcConnected) {
      wdt_reset();
      WDTCSR |= (1 << WDCE) | (1 << WDE);
      WDTCSR = (1 << WDIE) | (1 << WDP2) | (1 << WDP1);  // 1 секунда
    }

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sei();
    sleep_cpu();  // <--- КОНТРОЛЛЕР ЗАСЫПАЕТ

    // --- ТОЧКА ПРОБУЖДЕНИЯ ---
    sleep_disable();
    if (!isRtcConnected) wdt_disable();
  }

  // Отключаем внешние прерывания сна и поднимаем периферию (включая I2C!)
  detachInterrupt(digitalPinToInterrupt(indicators[0].pin));
  detachInterrupt(digitalPinToInterrupt(PIN_DOOR_TRIGGER));
  PCICR &= ~(1 << PCIE2);

  wakeUpPeripherals(old_ADCSRA);

  // Когда I2C снова работает, безопасно отключаем будильник в RTC
  if (useAlarm && isRtcConnected) {
    rtc.disableAlarm(1);
    rtc.clearAlarm(1);
  }
}

// =========================================================================
// ФУНКЦИЯ АНАЛИЗА ПРИЧИНЫ ПРОБУЖДЕНИЯ
// Возвращает true, если нужно выйти из loop() (поменяли стейт или идем дальше спать)
// =========================================================================
bool processWakeUpReason() {
  // Вариант А: Разбудил ЦЗ (клики)
  if (wakeUpClickCount > 0) {
    lastWorkDebounceTime = micros();
    EIFR = (1 << INTF2);
    attachInterrupt(digitalPinToInterrupt(PIN_DOOR_TRIGGER), doorTriggerISRWork, FALLING);

    unsigned long windowTimer = millis();
    int currentClicks = wakeUpClickCount;

    while (millis() - windowTimer < TIMEOUT_WAIT_CLICK) {
      // Если во время ожидания кликов повернули ключ зажигания — сразу едем!
      if (digitalRead(indicators[0].pin) == LOW) {
        detachInterrupt(digitalPinToInterrupt(PIN_DOOR_TRIGGER));
        wakeUpClickCount = 0;
        wakeUpIgnition = false;
        syncTimeManager(true);
        currentState = STATE_DRIVE;
        TEST_SERIAL.println(F("Зажигание во время кликов. Переход в DRIVE."));
        return true;
      }

      if (wakeUpClickCount != currentClicks) {
        currentClicks = wakeUpClickCount;
        windowTimer = millis();
      }
    }

    detachInterrupt(digitalPinToInterrupt(PIN_DOOR_TRIGGER));
    wakeUpClickCount = 0;

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
        TW2_SERIAL.print("\n\n");
        delay(2000);
        TW2_SERIAL.print("reboot -p\n");
        delay(10000);
        digitalWrite(PIN_ACC_OUTPUT, LOW);  // Обесточиваем приставку через BTS442

        syncTimeManager(true);
        currentState = STATE_SLEEP;  // Переходим в бесконечный сон
      }
    } else {
      // Если 2 клика (закрыли) или больше — идем спать в бесконечный сон
      // (если приставка еще была в mem, остаемся в PRE_SLEEP или переводим в SLEEP)
      TEST_SERIAL.println(F("Кликов больше 1. Возврат в сон..."));
    }
    syncTimeManager(true);
    return true;
  }
  // Вариант Б: Разбудил ключ зажигания напрямую
  if (wakeUpIgnition) {
    wakeUpIgnition = false;
    if (digitalRead(indicators[0].pin) == LOW) {
      syncTimeManager(true);
      currentState = STATE_DRIVE;
      TEST_SERIAL.println(F("Появилось зажигание. DRIVE."));
      return true;
    }
  }
  return false;  // Значит, сработал таймер будильника или WDT
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
  // Получаем текущее время из RTC-millis
  DateTime now = rtcBackup.now();
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
}

// *************************************************************************
// *************************        SETUP          *************************
// *************************************************************************

void setup() {
  // setTime(12, 0, 0, 1, 1, 2026);
  TW2_SERIAL.begin(115200);
  TEST_SERIAL.begin(115200);
  GBO_SERIAL.begin(9600);
  // pinMode(PIN_RX_GBO, INPUT_PULLUP);  // Подтяжка RX линии GBO_SERIAL
  RD_SERIAL.begin(115200);

  TEST_SERIAL.println(F("============ ЗАГРУЗКА СИСТЕМЫ ================"));

  // Инициализация остальных пинов
  pinMode(PIN_ACC_OUTPUT, OUTPUT);
  digitalWrite(PIN_ACC_OUTPUT, LOW);
  pinMode(PIN_DOOR_TRIGGER, INPUT_PULLUP);
  pinMode(PIN_RTC, INPUT_PULLUP);

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

  sensors.begin();

  Wire.begin();                     // Инициализация I2C шины для Mega (пины 20 и 21 инициализируются автоматически)
  Wire.setWireTimeout(3000, true);  // Безопасный запуск I2C с аппаратным таймаутом

  // Инициализация службы времени
  Wire.beginTransmission(0x68);
  if (Wire.endTransmission() == 0 && rtc.begin()) {
    isRtcConnected = true;
    if (rtc.lostPower()) rtcBackup.begin(DateTime(1767225600));
    else rtcBackup.begin(rtc.now());

    // Отключаем генерацию меандра на пине SQW, переводим его в режим генерации прерываний будильника
    rtc.writeSqwPinMode(DS3231_OFF);
    rtc.disableAlarm(1);
    rtc.disableAlarm(2);
    rtc.clearAlarm(1);
    rtc.clearAlarm(2);

  } else {
    rtcBackup.begin(DateTime(1767225600));
    triggerRealDashAlert(3);  // Код 3: Ошибка часов
  }

  if (fram.begin()) {  // Запуск и валидация памяти FRAM
    systemMemoryInitAndValidate();
  }
  // currentState = STATE_PRE_DRIVE_WAKE;
  wakeUpTimerStart = millis();

  TEST_SERIAL.println(F("Система загружена"));
}

// *************************************************************************
// *************************         LOOP          *************************
// *************************************************************************

void loop() {

  syncTimeManager();

  // =========================================================================
  // --- АВТОМАТ СОСТОЯНИЙ ПИТАНИЯ ---
  // =========================================================================

  unsigned long currentMillis = millis();
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
        TEST_SERIAL.println(F("Состояние изменено на DRIVE."));

      } else if (millis() - wakeUpTimerStart >= TIMEOUT_WAIT_IGNITION) {

        TW2_SERIAL.print("su -c \"echo mem > /sys/power/state\"\n");  // Отправляем Tanix команду ухода в сон через UART консоль
        currentState = STATE_SHUTDOWN;
        TEST_SERIAL.println(F("Состояние изменено на SHUTDOWN. Вышло время"));
      }

      break;

    case STATE_DRIVE:

      if (digitalRead(PIN_ACC_OUTPUT) == LOW) handleSmoothStart();
      // ЗАПРОС В ЭБУ ГБО
      if (currentMillis - timerGboRequest >= INTERVAL_GBO_REQUEST) {
        GBO_SERIAL.write(GBO_QUERY, sizeof(GBO_QUERY));
        timerGboRequest = currentMillis;
      }

      collectGboData();

      checkIncomingRealDashCAN();  // Слушаем входящие команды из RealDash

      // БЫСТРЫЕ ДАТЧИКИ
      if (currentMillis - timerFastSensors >= INTERVAL_FAST) {
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
        backupDataToFram();
        TW2_SERIAL.print("su -c \"echo mem > /sys/power/state\"\n");  // Отправляем Tanix команду ухода в сон через UART консоль
        // время в Tanix
        // uint32_t unixTime = rtcBackup.now().unixtime();
        // // Команда date -s @<timestamp> работает в стандартном BusyBox Android/Linux
        // TW2_SERIAL.print(F("su -c \"date -s @"));
        // TW2_SERIAL.print(unixTime);
        // TW2_SERIAL.println(F("\""));
        // время от Tanix
        // TW2_SERIAL.println(F("echo TIME:$(date +%s)")); // Отправляем Tanix команду запроса текущего Unix-времени
        unsigned long windowTimer = millis();
        while (millis() - windowTimer < TIMEOUT_WAIT_UART_SILENCE) {
          if (TW2_SERIAL.available() > 0) {
            while (TW2_SERIAL.available() > 0) {
              // TW2_SERIAL.read();
              char inByte = TW2_SERIAL.read();  // Считываем байт из буфера ПК
              TEST_SERIAL.write(inByte);
            }
            windowTimer = millis();
          }
        }
        TEST_SERIAL.println(F("Зажигание выключено. STATE_PRE_SLEEP. "));
        syncTimeManager(true);
        currentState = STATE_PRE_SLEEP;
      }
      break;

    case STATE_SHUTDOWN:

      // digitalWrite(PIN_ACC_OUTPUT, LOW);  // Обесточиваем магнитолу через BTS442
      // delay(500);                         // Короткая пауза для записи кэша магнитолы
      // currentState = STATE_SLEEP;

      break;

    case STATE_PRE_SLEEP:
      // Спим с активным таймером (через RTC или WDT)
      enterDeepSleep(true, TANIX_SLEEP_DURATION / 1000UL);

      // Разбудил ЦЗ или зажигание
      if (processWakeUpReason()) return;

      // Сработал таймер выключения
      wokenByRtc = false;
      TW2_SERIAL.print("\n\n");
      delay(2000);
      TW2_SERIAL.print("reboot -p\n");
      delay(10000);
      digitalWrite(PIN_ACC_OUTPUT, LOW);  // Обесточиваем приставку через BTS442

      syncTimeManager(true);
      currentState = STATE_SLEEP;  // Переходим в бесконечный сон

      break;

    case STATE_SLEEP:
      // Бесконечный сон (без будильника; если RTC отвалился, WDT тикает для хода софт-часов)
      enterDeepSleep(false, 0);
      // Анализируем, кто разбудил (1 клик ЦЗ -> PRE_DRIVE_WAKE, зажигание -> DRIVE)
      processWakeUpReason();

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
