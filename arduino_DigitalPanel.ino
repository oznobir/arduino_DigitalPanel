#include <SPI.h>
#include <mcp_can.h>

const int SPI_CS_PIN = 9;
MCP_CAN CAN0(SPI_CS_PIN);

// Буфер и запрос ГБО Stag
byte masterQuery[] = {0xF0, 0x01, 0x01, 0xF2}; 
byte gboBuf[83]; // Жестко резервируем 83 ячейки памяти в C++

// === ТАЙМЕРЫ ДЛЯ НАШЕЙ АСИНХРОННОЙ МНОГОЗАДАЧНОСТИ ===
unsigned long timerGBO  = 0;  // Для отправки запросов к Stag (200 мс)
unsigned long timerSlow = 0;  // Для вывода медленных параметров (3000 мс)

// === ПЕРЕМЕННЫЕ ДЛЯ ВЫВОДА НА БУДУЩИЙ ЭКРАН ===
// Данные ГБО Stag (По вашей точной, проверенной карте байт!)
float injBenz1 = 0.0, injBenz2 = 0.0, injBenz3 = 0.0, injBenz4 = 0.0;
float injGas1 = 0.0, injGas2 = 0.0, injGas3 = 0.0, injGas4 = 0.0;
int gboRpm = 0;
float pressGas = 0.0, pressMap = 0.0;
int tempRed = 0, tempGas = 0;
bool isGasActive = false;

// Данные из родной CAN-шины автомобиля (Пассивный перехват на 12 и 13 пинах ЦКБЭ)
int carRpm = 0;       // Родные обороты из ЭБУ двигателя
int carWaterTemp = 0; // Температура антифриза машины
float carSpeed = 0.0; // Точная скорость от блока ABS

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);       // Аппаратный Serial для связи со Stag
  pinMode(19, INPUT_PULLUP); // Подтяжка RX линии газа

  while(Serial1.available() > 0) Serial1.read(); // Очищаем буфер
  
  Serial.println(F("=================================================="));
  Serial.println(F("===    БОРТОВОЙ КОМПЬЮТЕР: ALMERA G15 + STAG    ==="));
  Serial.println(F("=================================================="));

  // Запуск CAN на 500 Кбит/с (высокоскоростная моторная шина у левой ноги)
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println(F("[CAN] Модуль MCP2515 успешно запущен в режиме невидимки!"));
    CAN0.setMode(MCP_LISTENONLY); // Только слушаем эфир ЭБУ-ABS, джампер 120 Ом СНЯТ!
  } else {
    Serial.println(F("[CAN] КРИТИЧЕСКАЯ ОШИБКА: MCP2515 не отвечает на пине 9!"));
  }
}

void loop() {
  unsigned long currentTime = millis(); // Опорные "часы" Ардуино

  // =======================================================================
  // ЗАДАЧА 1: ПАССИВНЫЙ ПЕРЕХВАТ CAN-ШИНЫ (ЭБУ двигателя + ABS)
  // =======================================================================
  // Проверяем в каждом цикле loop, чтобы исключить переполнение буфера чипа MCP
  long unsigned int rxId;
  unsigned char len = 0;
  unsigned char rxBuf[8]; // массив на 8 байт

  if(CAN0.checkReceive() == CAN_MSGAVAIL) { 
    CAN0.readMsgBuf(&rxId, &len, rxBuf);    
    
    // Пакет 0x11A: Родные обороты двигателя и температура ОЖ
    if (rxId == 0x11A) {
      int rawRpm = (rxBuf[0] << 8) | rxBuf[1]; // Собираем байты в правильном порядке
      carRpm = rawRpm / 4;
      carWaterTemp = rxBuf[4] - 40; // Извлекаем температуру мотора из структуры ЭБУ
    }
    
    // Пакет 0x354: Точная скорость автомобиля от блока ABS
    if (rxId == 0x354) {
      int rawSpeed = (rxBuf[0] << 8) | rxBuf[1];
      carSpeed = rawSpeed / 100.0;
    }
  }

  // =======================================================================
  // ЗАДАЧА 2: ОПРОС И ДИНАМИЧЕСКИЙ ПРИЕМ ГБО STAG (Раз в 200 мс)
  // =======================================================================
  if (currentTime - timerGBO >= 200) {
    timerGBO = currentTime;
    while(Serial1.available() > 0) Serial1.read();   // Сбрасываем старый хвост
    Serial1.write(masterQuery, sizeof(masterQuery)); // Отправляем запрос к Stag
  }

  // Забираем пакет 83 байта на лету, когда он полностью подошел в Serial-буфер
  if (Serial1.available() >= 83) { 
    if (Serial1.read() == 0xF0) { // Нашли стартовый маркер пакета
      byte m1 = Serial1.read();
      byte packetLen = Serial1.read();
      
      if (packetLen == 0x53) { // Это наш целевой пакет параметров Stag
        for (int i = 3; i < 83; i++) {
          gboBuf[i] = Serial1.read(); // Заполняем массив ячейка за ячейкой
        }
        
        // --- ДЕКОДИРОВАНИЕ ПАРАМЕТРОВ ПО НАЙДЕННЫМ ИНДЕКСАМ ---
        injBenz1 = gboBuf[10] / 10.0;
        injBenz2 = gboBuf[12] / 10.0;
        injBenz3 = gboBuf[14] / 10.0;
        injBenz4 = gboBuf[16] / 10.0;
        
        injGas1 = gboBuf[26] / 10.0;
        injGas2 = gboBuf[28] / 10.0;
        injGas3 = gboBuf[30] / 10.0;
        injGas4 = gboBuf[32] / 10.0;

        gboRpm   = (gboBuf[42] * 100) + gboBuf[43];
        pressGas = gboBuf[45] * 0.01; 
        pressMap = gboBuf[47] * 0.01;
        tempRed  = gboBuf[48];
        tempGas  = gboBuf[49];
        
        isGasActive = (injGas1 > 0.5 && injGas2 > 0.5 && injGas3 > 0.5 && injGas4 > 0.5);
        
        printFastDashboard(); // Моментальный вывод в консоль
      }
    }
  }

  // =======================================================================
  // ЗАДАЧА 3: СЕРВИСНЫЕ И МЕДЛЕННЫЕ ДАННЫЕ (Вывод раз в 3 секунды)
  // =======================================================================
  if (currentTime - timerSlow >= 3000) {
    timerSlow = currentTime;
    printSlowDashboard();
  }
}

void printFastDashboard() {
  Serial.print(F("[АЛЬМЕРА CAN] Скорость: ")); Serial.print(carSpeed, 1); Serial.print(F(" км/ч"));
  Serial.print(F(" | Родные_RPM: ")); Serial.print(carRpm);
  Serial.print(F(" | Т_Мотора: ")); Serial.print(carWaterTemp); Serial.print(F("°C"));
  
  Serial.print(F("[STAG] ")); if (isGasActive) Serial.print(F("ГАЗ")); else Serial.print(F("БЕНЗИН"));
  Serial.print(F(" | Газ_RPM: ")); Serial.print(gboRpm);
  Serial.print(F(" | Впр_Г1: ")); Serial.print(injGas1, 1);
  Serial.print(F("мс | П_Газ: ")); Serial.print(pressGas, 2); Serial.println(F(" Бар"));
}

void printSlowDashboard() {
  Serial.println(F("---------------------------------------------------------------------------------"));
  Serial.print(F("[ГБО] Бенз: ")); 
  Serial.print(injBenz1, 1); Serial.print(F("/")); Serial.print(injBenz2, 1); Serial.print(F("/"));
  Serial.print(injBenz3, 1); Serial.print(F("/")); Serial.print(injBenz4, 1); 
  Serial.print(F(" мс | Газ: "));
  Serial.print(injGas1, 1);  Serial.print(F("/")); Serial.print(injGas2, 1);  Serial.print(F("/"));
  Serial.print(injGas3, 1);  Serial.print(F("/")); Serial.print(injGas4, 1);  Serial.print(F(" мс"));
  Serial.print(F(" | Т_Ред: ")); Serial.print(tempRed); Serial.print(F("°C | Т_Газ: ")); Serial.print(tempGas); Serial.println(F("°C"));
  Serial.println(F("---------------------------------------------------------------------------------"));
}

