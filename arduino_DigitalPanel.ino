#include <SPI.h>
#include <mcp_can.h>

const int SPI_CS_PIN = 9;
MCP_CAN CAN0(SPI_CS_PIN);

byte txBuffer[128]; // Буфер для отправки
int txCount = 0;
byte rxBuffer[256]; // Буфер для приема

void setup() {
  Serial.begin(115200); // Связь с ПК (настраивайте Монитор порта на 115200)
  Serial1.begin(9600);  // Связь со Stag (9600 бод)
  
  pinMode(19, INPUT_PULLUP); // Наша спасительная подтяжка уровня

  // Настройка перевода строки в Мониторе порта: ОБЯЗАТЕЛЬНО выберите "Newline" (Новая строка) или "Both NL & CR"
  while(Serial1.available() > 0) Serial1.read();
  
  Serial.println(F("=================================================="));
  Serial.println(F("===         ИНТЕРАКТИВНЫЙ ТЕРМИНАЛ STAG        ==="));
  Serial.println(F("=== Вводите байты через пробел. Пример: F0 01 1 0хF2 ==="));
  Serial.println(F("=================================================="));

  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    CAN0.setMode(MCP_LISTENONLY); 
  }
}

void loop() {
  // 1. ЧТЕНИЕ КОМАНДЫ ИЗ МОНИТОРА ПОРТА (ПК -> Ардуино)
  if (Serial.available() > 0) {
    String inputStr = Serial.readStringUntil('\n');
    inputStr.trim(); // Убираем лишние пробелы по краям
    
    if (inputStr.length() > 0) {
      txCount = 0;
      
      // Парсим строку, разбивая по пробелам
      int pos = 0;
      while (pos < inputStr.length() && txCount < 128) {
        // Пропускаем пробелы
        while (pos < inputStr.length() && inputStr[pos] == ' ') pos++;
        if (pos >= inputStr.length()) break;
        
        // Выделяем отдельный токен (байт в виде текста)
        String token = "";
        while (pos < inputStr.length() && inputStr[pos] != ' ') {
          token += inputStr[pos];
          pos++;
        }
        
        // Убираем префикс "0x" или "0X", если вы случайно его ввели
        if (token.startsWith("0x") || token.startsWith("0X")) {
          token = token.substring(2);
        }
        
        // Преобразуем HEX-текст в реальный байт
        if (token.length() > 0) {
          txBuffer[txCount++] = (byte) strtol(token.c_str(), NULL, 16);
        }
      }
      
      // Отправляем сформированный пакет в ГБО
      if (txCount > 0) {
        while(Serial1.available() > 0) Serial1.read(); // Чистим приемный буфер перед отправкой
        
        Serial.print(F("\n[ПК -> ГБО] Отправлено "));
        Serial.print(txCount);
        Serial.print(F(" байт: "));
        for(int i=0; i<txCount; i++) {
          Serial.print(F("0x"));
          if(txBuffer[i] < 16) Serial.print(F("0"));
          Serial.print(txBuffer[i], HEX);
          Serial.print(F(" "));
        }
        Serial.println();
        
        // Физическая отправка в белый провод
        Serial1.write(txBuffer, txCount);
        delay(20); // Даем небольшую паузу Stag на обработку
      }
    }
  }

  // 2. СБОР И ВЫВОД ОТВЕТА (ГБО -> ПК)
  if (Serial1.available() > 0) {
    delay(40); // Даем пакету накопиться в буфере UART (особенно длинным ответам)
    
    int rxCount = 0;
    unsigned long packetTimer = millis();
    
    // Вычитываем всё, что пришло от Stag в ответ
    while (millis() - packetTimer < 25 && rxCount < 256) {
      if (Serial1.available() > 0) {
        rxBuffer[rxCount++] = Serial1.read();
        packetTimer = millis();
      }
    }
    
    // Красиво выводим ответ на экран
    if (rxCount > 0) {
      Serial.print(F("[ГБО -> ПК] Получен ответ (Длина: "));
      Serial.print(rxCount);
      Serial.println(F(" байт)"));
      Serial.println(F("--------------------------------------------------"));
      
      for (int i = 0; i < rxCount; i++) {
        //Serial.print(F("["));
        //if(i < 10) Serial.print(F("0"));
        //Serial.print(i);
        //Serial.print(F("] HEX: 0x"));
        Serial.print(F(" 0x"));
        if(rxBuffer[i] < 16) Serial.print(F("0"));
        Serial.print(rxBuffer[i], HEX);
        //Serial.print(F(" (DEC: "));
        //Serial.print(rxBuffer[i]);
        //Serial.println(F(")"));
      }
      Serial.println();
      Serial.println(F("--------------------------------------------------"));
    }
  }
}

// #include <SPI.h>
// #include <mcp_can.h>

// const int SPI_CS_PIN = 9;
// MCP_CAN CAN0(SPI_CS_PIN);

// // Структура для управления каждым каналом
// struct SerialChannel {
//   HardwareSerial* port;
//   String name;
//   byte buf[100];
//   int count;
//   unsigned long lastByteTime;
//   bool isCapture;
// };

// // Инициализируем каналы: ПК (Serial2, пин 17) и ГБО (Serial1, пин 19)
// SerialChannel chPC =  {&Serial2, "   [ПК -> ГБО]  ", {0}, 0, 0, false};
// SerialChannel chGBO = {&Serial1, "   [ГБО -> ПК]  ", {0}, 0, 0, false};


// void setup() {
//   Serial.begin(115200);   // Монитор порта для связи с компьютером (Ардуино -> ПК)
//   Serial1.begin(9600);    // Слушаем RXD свистка (Ответы ГБО)
//   Serial2.begin(9600);    // Слушаем TXD свистка (Запросы ПК)
  
//   // Активируем спасительную подтяжку на обоих принимающих пинах
//   pinMode(19, INPUT_PULLUP); // RX1
//   pinMode(17, INPUT_PULLUP); // RX2

//   // Очищаем буферы
//   while(Serial1.available() > 0) Serial1.read();
//   while(Serial2.available() > 0) Serial2.read();

//   Serial.println(F("=================================================="));
//   Serial.println(F("===        ДВУХКАНАЛЬНЫЙ СНИФФЕР ПК <=> ГБО    ==="));
//   Serial.println(F("=== Скорость: 9600 | Ждем маркер старта 0xF0   ==="));
//   Serial.println(F("=================================================="));

//   if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
//     CAN0.setMode(MCP_LISTENONLY); 
//   }
// }

// // Функция обработки потока данных для конкретного канала
// void processChannel(SerialChannel& ch) {
//   while (ch.port->available() > 0) {
//     byte c = ch.port->read();
//     unsigned long now = millis();

//     // Если мы еще не ловим пакет, ищем маркер начала 0xF0
//     if (!ch.isCapture) {
//       if (c == 0xF0) {
//         ch.isCapture = true;
//         ch.count = 0;
//         ch.buf[ch.count++] = c;
//         ch.lastByteTime = now;
//       }
//     } 
//     // Если пакет уже захвачен, складываем байты в буфер
//     else {
//       if (ch.count < 100) {
//         ch.buf[ch.count++] = c;
//       }
//       ch.lastByteTime = now;
//     }
//   }

//   // Если пакет захвачен и наступила пауза (нет байт более 30 мс) -> выводим лог кадра
//   if (ch.isCapture && (millis() - ch.lastByteTime > 30)) {
//     Serial.println(ch.name);
//     Serial.print(F("Старт: 0xF0 | Длина: ")); 
//     Serial.print(ch.count); 
//     Serial.println(F(" байт"));
    
//     // Красивый побайтный вывод пакета в одну строчку для легкого чтения
//     Serial.print(F("Дамп: (HEX|DEC) "));
//     for (int i = 0; i < ch.count; i++) {
//       Serial.print(F("0x"));
//       if (ch.buf[i] < 16) Serial.print(F("0"));
//       Serial.print(ch.buf[i], HEX);
//       Serial.print(F("|"));
//       Serial.print(ch.buf[i]);
//       Serial.print(F(" "));
//     }
//     Serial.println(F("\n--------------------------------------------------"));
    
//     // Сбрасываем триггер для ожидания следующего пакета
//     ch.isCapture = false;
//     ch.count = 0;
//   }
// }

// void loop() {
//   processChannel(chPC);  // Опрашиваем линию отправки компьютера
//   processChannel(chGBO); // Опрашиваем линию ответа газового блока
// }