#include <SPI.h>
#include <mcp_can.h>

const int SPI_CS_PIN = 9;
MCP_CAN CAN0(SPI_CS_PIN);

// Структура для управления каждым каналом
struct SerialChannel {
  HardwareSerial* port;
  String name;
  byte buf[100];
  int count;
  unsigned long lastByteTime;
  bool isCapture;
};

// Инициализируем каналы: ПК (Serial2, пин 17) и ГБО (Serial1, пин 19)
SerialChannel chPC =  {&Serial2, "   [ПК -> ГБО]  ", {0}, 0, 0, false};
SerialChannel chGBO = {&Serial1, "   [ГБО -> ПК]  ", {0}, 0, 0, false};

void setup() {
  Serial.begin(115200);   // Монитор порта для связи с компьютером (Ардуино -> ПК)
  Serial1.begin(9600);    // Слушаем RXD свистка (Ответы ГБО)
  Serial2.begin(9600);    // Слушаем TXD свистка (Запросы ПК)
  
  // Активируем спасительную подтяжку на обоих принимающих пинах
  pinMode(19, INPUT_PULLUP); // RX1
  pinMode(17, INPUT_PULLUP); // RX2

  // Очищаем буферы
  while(Serial1.available() > 0) Serial1.read();
  while(Serial2.available() > 0) Serial2.read();

  Serial.println(F("=================================================="));
  Serial.println(F("===        ДВУХКАНАЛЬНЫЙ СНИФФЕР ПК <=> ГБО    ==="));
  Serial.println(F("=== Скорость: 9600 | Ждем маркер старта 0xF0   ==="));
  Serial.println(F("=================================================="));

  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    CAN0.setMode(MCP_LISTENONLY); 
  }
}

// Функция обработки потока данных для конкретного канала
void processChannel(SerialChannel& ch) {
  while (ch.port->available() > 0) {
    byte c = ch.port->read();
    unsigned long now = millis();

    // Если мы еще не ловим пакет, ищем маркер начала 0xF0
    if (!ch.isCapture) {
      if (c == 0xF0) {
        ch.isCapture = true;
        ch.count = 0;
        ch.buf[ch.count++] = c;
        ch.lastByteTime = now;
      }
    } 
    // Если пакет уже захвачен, складываем байты в буфер
    else {
      if (ch.count < 100) {
        ch.buf[ch.count++] = c;
      }
      ch.lastByteTime = now;
    }
  }

  // Если пакет захвачен и наступила пауза (нет байт более 30 мс) -> выводим лог кадра
  if (ch.isCapture && (millis() - ch.lastByteTime > 30)) {
    Serial.println(ch.name);
    Serial.print(F("Старт: 0xF0 | Длина: ")); 
    Serial.print(ch.count); 
    Serial.println(F(" байт"));
    
    // Красивый побайтный вывод пакета в одну строчку для легкого чтения
    Serial.print(F("Дамп: "));
    for (int i = 0; i < ch.count; i++) {
      Serial.print(F("0x"));
      if (ch.buf[i] < 16) Serial.print(F("0"));
      Serial.print(ch.buf[i], HEX);
      Serial.print(F(" "));
    }
    Serial.println(F("\n--------------------------------------------------"));
    
    // Сбрасываем триггер для ожидания следующего пакета
    ch.isCapture = false;
    ch.count = 0;
  }
}

void loop() {
  processChannel(chPC);  // Опрашиваем линию отправки компьютера
  processChannel(chGBO); // Опрашиваем линию ответа газового блока
}

// #include <SPI.h>
// #include <mcp_can.h>

// const int SPI_CS_PIN = 9;
// MCP_CAN CAN0(SPI_CS_PIN);

// // Массив всех возможных скоростей для блоков Stag
// long speeds[] = {9600, 19200, 38400, 57600, 115200};
// int currentSpeedIndex = 0;
// unsigned long lastSwitchTime = 0;
// bool dataFound = false;

// void setup() {
//   Serial.begin(115200);
  
//   // Включаем встроенную подтяжку, чтобы помочь линии пробить просадку уровня!
//   pinMode(19, INPUT_PULLUP); 
  
//   Serial.println(F("=================================================="));
//   Serial.println(F("===        АВТОСКАНЕР СКОРОСТИ ГБО STAG        ==="));
//   Serial.println(F("=================================================="));
  
//   Serial.print(F("Пробуем скорость: ")); Serial.println(speeds[currentSpeedIndex]);
//   Serial1.begin(speeds[currentSpeedIndex]);

//   if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
//     CAN0.setMode(MCP_LISTENONLY); 
//   }
// }

// void loop() {
//   // Если в течение 3 секунд на текущей скорости идут только нули или тишина — меняем скорость
//   if (millis() - lastSwitchTime > 3000) {
//     if (!dataFound) {
//       currentSpeedIndex++;
//       if (currentSpeedIndex >= 5) currentSpeedIndex = 0; // по кругу
      
//       Serial1.end();
//       Serial.print(F("\n[СКАНЕР] Данных нет. Переключаем на: ")); 
//       Serial.print(speeds[currentSpeedIndex]); 
//       Serial.println(F(" бод..."));
      
//       Serial1.begin(speeds[currentSpeedIndex]);
//       while(Serial1.available() > 0) Serial1.read();
//     }
//     lastSwitchTime = millis();
//     dataFound = false;
//   }

//   if (Serial1.available() > 0) {
//     byte c = Serial1.read();
    
//     // Если поймали ХОТЬ ЧТО-ТО, кроме чистого нуля — фиксируем скорость!
//     if (c != 0x00) {
//       dataFound = true;
//       Serial.print(F("\n!!! НАЙДЕНЫ ДАННЫЕ НА СКОРОСТИ "));
//       Serial.print(speeds[currentSpeedIndex]);
//       Serial.println(F(" !!!"));
//       Serial.print(F("Поймали байт: 0x"));
//       if(c < 16) Serial.print(F("0"));
//       Serial.println(c, HEX);
      
//       // Замираем на этой скорости на 10 секунд, чтобы вы успели рассмотреть лог
//       delay(10000); 
//       while(Serial1.available() > 0) Serial1.read();
//       lastSwitchTime = millis();
//     }
//   }
// }