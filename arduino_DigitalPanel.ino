#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 14 // Датчики сидят на цифровом пине 14
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Настройка пинов 
const int CAN_CS_PIN = 53;     // Пин CS для MCP2515
const int CAN_INT_PIN = 2;     // Пин прерывания INT для MCP2515
const int HANDBRAKE_PIN = 16;   // Пример подключения ручника (датчик по минусу)
const int TURN_LEFT_PIN = 17;   // Пример подключения поворотника (датчик по плюсу)
MCP_CAN CAN0(CAN_CS_PIN);     // Инициализация объекта CAN


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  sensors.begin();

  // Настройка пинов для оптопар PC817 (обязательно PULLUP)
  pinMode(HANDBRAKE_PIN, INPUT_PULLUP);
  pinMode(TURN_LEFT_PIN, INPUT_PULLUP);

  // Инициализация MCP2515. На Almera G15 скорость шины обычно 500 Кбит/с.
  // Кварц на плате MCP2515 стоит на 8 МГц.
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("MCP2515 успешно запущен!");
  } else {
    Serial.println("Ошибка инициализации MCP2515...");
    while(1); // Останавливаем выполнение при ошибке
  }
  
  CAN0.setMode(MCP_NORMAL);   // Переводим CAN в рабочий режим
  pinMode(CAN_INT_PIN, INPUT); 

}

void loop() {
  // put your main code here, to run repeatedly:
  // 1. Опрос датчиков с оптопары PC817
  if (digitalRead(HANDBRAKE_PIN) == LOW) {
    // Оптопара открылась, значит датчик в авто выдал активный сигнал
    Serial.println("Ручник затянут!");
  }
  if (digitalRead(TURN_LEFT_PIN) == LOW) {
    // Оптопара открылась, значит датчик в авто выдал активный сигнал
    Serial.println("Левый поворотник включен!");
  }
  // 2. Чтение данных из CAN-шины автомобиля
  if(!digitalRead(CAN_INT_PIN)) { // Если пришел пакет (пин INT упал в LOW)
    long unsigned int rxId;
    unsigned char len = 0;
    unsigned char rxBuf[8];
    
    CAN0.readMsgBuf(&rxId, &len, rxBuf); // Читаем данные из буфера
    
    // Выводим ID пакета и данные в Serial для анализа
    Serial.print("ID: 0x");
    Serial.print(rxId, HEX);
    Serial.print(" Data: ");
    for(int i = 0; i<len; i++) {
      if(rxBuf[i] < 0x10) Serial.print("0");
      Serial.print(rxBuf[i], HEX);
      Serial.print(" ");
    }
  Serial.println();
  }
  //3. Запрос температуры у всех датчиков DS18B20 на шине
  sensors.requestTemperatures();
  // Считываем по индексам (индекс зависит от того, какой датчик плата найдет первым)
  Serial.print("Temp.1: ");
  Serial.println((char)sensors.getTempCByIndex(0)); 
  Serial.print("Temp.2: ");
  Serial.println((char)sensors.getTempCByIndex(1)); 
  Serial.print("Temp.3: ");
  Serial.println((char)sensors.getTempCByIndex(2)); 

  Serial.println(); 
  delay(50); // Небольшая задержка для стабильности цикла
}
