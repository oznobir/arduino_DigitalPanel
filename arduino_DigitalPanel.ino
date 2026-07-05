#include <SPI.h>
#include <mcp_can.h>

const int SPI_CS_PIN = 9;
MCP_CAN CAN0(SPI_CS_PIN);

//ПОБЕДНЫЙ ЗАПРОС (4 байта)
byte masterQuery[] = {0xF0, 0x01, 0x01, 0xF2}; 
byte gboBuf[85];
unsigned long lastQueryTime = 0;

// ПЕРЕМЕННЫЕ ДЛЯ ПРИБОРНОЙ ПАНЕЛИ
float injBenz1 = 0.0; // Время впрыска бензина (цил 1)
float injBenz2 = 0.0;
float injBenz3 = 0.0;
float injBenz4 = 0.0;
float injGas1 = 0.0;  // Время впрыска газа (цил 1)
float injGas2 = 0.0;  
float injGas3 = 0.0;  
float injGas4 = 0.0;  
int engineRpm = 0;   // Обороты
float pressGas = 0.0; // Давление газа
float pressMap = 0.0; // Давление MAP
int tempGas = 0;     // Температура газа
int tempRed = 0;     // Температура редуктора
float batteryVolt = 0.0;
int engineLoad = 0;
bool isGasActive = false; // На каком топливе едем
int gasLiters = 0; 


void setup() {
  Serial.begin(115200);
  Serial1.begin(9600); // 9600 бод
  pinMode(19, INPUT_PULLUP); // Подтяжка

  while(Serial1.available() > 0) Serial1.read();
  Serial.println(F("=== СТАГ v11.3: БОРТОВОЙ КОМПЬЮТЕР ==="));

  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    CAN0.setMode(MCP_LISTENONLY); 
  }
}

void loop() {
  unsigned long now = millis();

  // Запрашиваем данные из Stag каждые 300 мс
  if (now - lastQueryTime >= 300) {
    lastQueryTime = now;
    while(Serial1.available() > 0) Serial1.read(); // Очищаем старый буфер
    Serial1.write(masterQuery, sizeof(masterQuery)); 
  }

  // Принимаем пакет параметров
  if (Serial1.available() > 0) {
    if (Serial1.peek() == 0xF0) {
      delay(100); // Даем 83 байтам полностью зайти в порт (45 мс мало)
      
      byte m0 = Serial1.read();
      byte m1 = Serial1.read();
      byte packetLen = Serial1.read();
      
      // Если пришел наш пакет параметров (83 байта)
      if (packetLen == 0x53) {
        // Читаем оставшиеся 80 байт в буфер (начиная с индекса 3)
        for (int i = 3; i < 83; i++) {
          gboBuf[i] = Serial1.read();
        }
        
        // --- ДЕКОДИРОВАНИЕ ПАРАМЕТРОВ ПО НАЙДЕННЫМ ИНДЕКСАМ ---
        
        // 1. Время впрыска (Бензин и Газ по 1-му цилиндру)
        
        injBenz1 = gboBuf[10] / 10.0;
        injBenz2 = gboBuf[12] / 10.0;
        injBenz3 = gboBuf[14] / 10.0;
        injBenz4 = gboBuf[16] / 10.0;
        
        injGas1 = gboBuf[26] / 10.0;
        injGas2 = gboBuf[28] / 10.0;
        injGas3 = gboBuf[30] / 10.0;
        injGas4 = gboBuf[32] / 10.0;
        // 2. Обороты двигателя
        engineRpm = (gboBuf[42] * 100) + gboBuf[43];
        
        // 3. Давления (Газ и MAP)
        pressGas = gboBuf[45] * 0.01; 
        pressMap = gboBuf[47] * 0.01;
        
        
        // 4. Температуры
        tempRed = gboBuf[48];
        tempGas = gboBuf[49];
        
        // 5. Текущее топливо
        if (injGas1 > 0.5 && injGas2 > 0.5 && injGas3 > 0.5  && injGas4 > 0.5) {
          isGasActive = true;
        } else {
          isGasActive = false;
        }

        // 6. Остаток газа
        // Неоходимо проверить физическое подключение, т.к. уровень газа на кнопке не работает
        //byte rawLevel = gboBuf[неизвестно]; // Получаем сырое значение (сейчас там 42)
        // Переводим попугаи датчика (считаем, что пустой ~40, полный ~210) в реальные литры (от 0 до 48)
        // !!!!!Внимание: точные цифры 40 и 210 НУЖНО скорректировать, когда баллон будет полностью пустой!!!!!
        //gasLiters = map(rawLevel, 40, 210, 0, 48); 
        //if (gasLiters < 0) gasLiters = 0;
        //if (gasLiters > 48) gasLiters = 48;

        // лямбда1 и лямбда2 неизвестно

        // Бортовое напряжение (Батарея)
        //batteryVolt = gboBuf[неизвестно]*0.01 + 12.0;

        // ВЫВОД НА ПРИБОРКУ
        printToDashboard();
      } else {
        // Если это 100-байтовый пакет карты — просто очищаем его из порта
        for (int i = 3; i < packetLen; i++) {
          if (Serial1.available() > 0) Serial1.read();
        }
      }
    } else {
      Serial1.read(); // Синхронизация
    }
  }
}

void printToDashboard() {
  Serial.print(F("[ГБО] Топливо: "));
  if (isGasActive) Serial.print(F("ГАЗ")); else Serial.print(F("БЕНЗИН"));
  Serial.print(F(" | Обороты: ")); Serial.print(engineRpm);
  Serial.print(F(" | Впр_Бенз: ")); Serial.print(injBenz1, 1);
  Serial.print(F(" | ")); Serial.print(injBenz2, 1);
  Serial.print(F(" | ")); Serial.print(injBenz3, 1);
  Serial.print(F(" | ")); Serial.print(injBenz4, 1); Serial.print(F(" мс"));
  Serial.print(F(" | Впр_Газ: ")); Serial.print(injGas1, 1); 
  Serial.print(F(" | ")); Serial.print(injGas2, 1); 
  Serial.print(F(" | ")); Serial.print(injGas3, 1);
  Serial.print(F(" | ")); Serial.print(injGas4, 1); Serial.print(F(" мс"));
  Serial.print(F(" | Давл_Газ: ")); Serial.print(pressGas, 2);
  Serial.print(F(" | Давл_MAP: ")); Serial.print(pressMap, 2);
  Serial.print(F(" | Т_Ред: ")); Serial.print(tempRed); Serial.print(F("°C"));
  Serial.print(F(" | Т_Газ: ")); Serial.print(tempGas); Serial.print(F("°C"));
  Serial.print(F(" | Ост_газа: ")); Serial.print(gasLiters); Serial.print(F(" л "));
  Serial.println(F(")"));
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