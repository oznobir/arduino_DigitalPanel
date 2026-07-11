#include <Arduino.h>

// --- КОНФИГУРАЦИЯ ПИНОВ ---
const byte SPEED_PIN = 2;   // Пин прерывания (Пин 2 = INT 0 на Mega). Сюда подключаем скорость через делитель.
const byte VOLT_PIN = A0;   // Аналоговый пин вольтметра (через делитель 10к/4.7к)
const byte COOLANT_PIN = A1;// Аналоговый пин ТОЖ (через делитель 10к/4.7к)
const byte FUEL_PIN = A2;   // Аналоговый пин датчика уровня топлива (через делитель 10к/4.7к)
const byte RPM_PIN = 3;     // Пин прерывания тахометра (Пин 3 = INT 1 на Mega)

// Переменные для расчета оборотов по импульсам
volatile unsigned long rpmPulseCount = 0;
unsigned long lastRpmCheck = 0;
const unsigned long RPM_INTERVAL = 200; // Обороты обновляем чаще (раз в 200 мс) для плавности стрелки

// Переменные для расчета скорости по импульсам
volatile unsigned long speedPulseCount = 0;
unsigned long lastSpeedCheck = 0;
const unsigned long SPEED_INTERVAL = 500; // Проверяем скорость раз в 500 мс

// Таймер для отправки данных в RealDash
unsigned long lastRealDashSend = 0;

// Структура RealDash (обязательно 4-байтовое выравнивание)
#pragma pack(push, 1)
struct RealDashPacket {
  unsigned long header = 0x44415348; // Маркер "DASH"
  // Аналоговые параметры и импульсы (считает сама Ардуино)
  uint16_t speed = 0;
  uint16_t rpm = 0;
  uint16_t voltage = 0;
  uint16_t fuelLevel = 0;
  uint16_t coolantTemp = 0;
  // Параметры от ГБО Stag (принимаем по Serial)
  uint16_t rpmGbo = 0;
};
#pragma pack(pop)

RealDashPacket dashData;

// Функция обработки импульса оборотов (вызывается аппаратно)
void onRpmPulse() {
  rpmPulseCount++;
}
// --- Функция обработки импульса скорости (вызывается аппаратно) ---
void onSpeedPulse() {
  speedPulseCount++;
}

void setup() {
  // Настраиваем порты
  Serial.begin(115200);   // Порт для связи с RealDash (основной USB)
  Serial1.begin(9600);    // Порт Serial1 (пины 19-RX, 18-TX) под ГБО Stag
  pinMode(SPEED_PIN, INPUT);
  // Привязываем прерывание: ловим переход сигнала из 0 в 1 (RISING)
  attachInterrupt(digitalPinToInterrupt(SPEED_PIN), onSpeedPulse, RISING);
  pinMode(RPM_PIN, INPUT);
  // Привязываем второе прерывание для оборотов (по фронту RISING)
  attachInterrupt(digitalPinToInterrupt(RPM_PIN), onRpmPulse, RISING);
}

void loop() {
  unsigned long currentMillis = millis();
  // 1. РАСЧЕТ ОБОРОТОВ ДВИГАТЕЛЯ (Раз в 200 мс)
  if (currentMillis - lastRpmCheck >= RPM_INTERVAL) {
    noInterrupts();
    unsigned long rpmPulses = rpmPulseCount;
    rpmPulseCount = 0;
    interrupts();

    // По стандарту Renault/Nissan, ЭБУ выдает 2 импульса на 1 оборот коленвала (для 4-цилиндрового мотора)
    // Обороты (об/мин) = (импульсы / время в сек) * (60 сек / 2 импульса)
    // Для интервала 200 мс (0.2 сек) формула: (pulses / 0.2) * 30 -> pulses * 150
    float calculatedRpm = (rpmPulses / (RPM_INTERVAL / 1000.0)) * 30.0;
    
    dashData.rpm = (uint16_t)calculatedRpm;
    lastRpmCheck = currentMillis;
  }
  // 2. РАСЧЕТ СКОРОСТИ АВТОМОБИЛЯ (Раз в 500 мс)
  if (currentMillis - lastSpeedCheck >= SPEED_INTERVAL) {
    // Временно отключаем прерывания, чтобы безопасно считать переменную pulseCount
    noInterrupts();
    unsigned long pulses = speedPulseCount;
    speedPulseCount = 0;
    interrupts();

    // Формула для платформы B0 (Logan/Almera G15): Датчик дает ровно 6 импульсов на 1 метр пути.
    // Скорость (км/ч) = (импульсы / время в сек) * (3600 сек / 6000 импульсов в 1 км)
    // Для интервала 500 мс (0.5 сек) формула упрощается до: pulses * 1.2
    float calculatedSpeed = (pulses / (SPEED_INTERVAL / 1000.0)) * 0.6;
    
    dashData.speed = (uint16_t)calculatedSpeed;
    lastSpeedCheck = currentMillis;
  }

  // 3. ЧТЕНИЕ ДАННЫХ ОТ ГБО STAG (Через Serial1)
  if (Serial1.available()) {
    // Здесь должен быть готовый код разбора пакета Stag
  }

  // 4. ИЗМЕРЕНИЕ ВОЛЬТАЖА, ТОПЛИВА И ТОЖ (Раз в 200 мс, чтобы не спамить АЦП)
  static unsigned long lastAnalogRead = 0;
  if (currentMillis - lastAnalogRead >= 200) {
    lastAnalogRead = currentMillis;

    // Вольтметр бортовой сети (Делитель 10кОм и 4.7кОм)
    int rawVolt = analogRead(VOLT_PIN);
    float realVoltage = (rawVolt * 5.0 / 1023.0) * 3.1276; // 3.1276 — коэффициент делителя
    dashData.voltage = (uint16_t)(realVoltage * 1000.0);   // Переводим в формат V/1000 для RealDash

    // Уровень топлива (ДУТ Logan/Almera: полный бак ~30 Ом, пустой ~330 Ом)
    int rawFuel = analogRead(FUEL_PIN);
    // Калибровка ДУТ: преобразуем сырое напряжение АЦП сразу в литры (от 0 до 50)
    // map(значение, пустой_значение_АЦП, полный_значение_АЦП, 0 литров, 50 литров)
    // Точные значения АЦП для пустого и полного бака нужно настроить при калибровке на машине
    long liters = map(rawFuel, 800, 100, 0, 50); 
    if (liters < 0) liters = 0;
    if (liters > 50) liters = 50;
    dashData.fuelLevel = (uint16_t)liters;
    // Чтение аналоговой температуры ОЖ (Пин А1)
    int rawCoolant = analogRead(COOLANT_PIN);
    // Калибровка ДТОЖ: преобразуем вольты АЦП в реальные градусы
    // Примерные значения: 900 на АЦП — это холодный мотор (около 20°C), 
    // 150 на АЦП — это горячий прогретый мотор (около 90°C).
    // Точные значения АЦП нужно подогонать на машине по датчику ГБО Stag для калибровки!
    long exactDeg = map(rawCoolant, 900, 150, 20, 90);
    // Записываем в структуру со смещением +40 (для XML)
    dashData.coolantTemp = (uint16_t)(exactDeg + 40);
  }
  // 5. ОТПРАВКА СТРУКТУРЫ В МОНИТОР ПОРТА (Каждые 30 мс)
  if (currentMillis - lastRealDashSend >= 1000) {
    lastRealDashSend = currentMillis;
    printDashboard(dashData);
  }
  // // 5. ОТПРАВКА СТРУКТУРЫ В REALDASH (Каждые 30 мс)
  // if (currentMillis - lastRealDashSend >= 30) {
  //   lastRealDashSend = currentMillis;
  //   Serial.write((byte*)&dashData, sizeof(dashData));
  // }
}
void printDashboard(const RealDashPacket& dataPacket) {
  Serial.print(F("[Аналог] Скорость: ")); Serial.print(dataPacket.speed); Serial.print(F(" км/ч"));
  Serial.print(F(" | Обороты: ")); Serial.print(dataPacket.rpm);
  Serial.print(F(" | ТОЖ: ")); Serial.print(dataPacket.coolantTemp); Serial.print(F("°C"));
  Serial.print(F(" | Батарея: ")); Serial.print(dataPacket.voltage); Serial.print(F(" в"));
  Serial.print(F(" | Бензин: ")); Serial.print(dataPacket.fuelLevel); Serial.println(F(" л"));
  
  Serial.print(F("[ГБО] Газ_Обороты: ")); Serial.print(dataPacket.rpmGbo);
}
// <?xml version="1.0" encoding="utf-8"?>
// <realdash>
//   <data baseId="3200">
//     <!-- Маркер DASH занимает первые 4 байта (offset 0, 1, 2, 3) -->
    
//     <!-- Скорость: смещение 4, длина 2 -->
//     <value targetId="0" offset="4" length="2" signed="false"></value>
    
//     <!-- Вольтметр: смещение 6, длина 2 (RealDash сам разделит на 1000) -->
//     <value targetId="12" offset="6" length="2" signed="false" conversion="V/1000"></value>
    
//     <!-- Уровень топлива: смещение 8, длина 2 (уже сразу в литрах) -->
//     <value targetId="18" offset="8" length="2" signed="false"></value>
    
//     <!-- Обороты от Stag: смещение 10, длина 2 -->
//     <value targetId="1" offset="10" length="2" signed="false"></value>
    
//     <!-- Температура Редуктора от Stag: смещение 12, длина 2 (вычитаем 40 для смещения) -->
//     <value targetId="14" offset="12" length="2" signed="false" conversion="V-40"></value>
//   </data>
// </realdash>

