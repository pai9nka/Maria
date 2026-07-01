#include <Arduino.h>
#include <FS.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_wifi.h>
#include <esp_sleep.h>
#include <driver/ledc.h>

// ==================== КОНФИГУРАЦИЯ ДЛЯ ESP32-C3 ====================
const int SERVO_1_PIN = 2;   // GPIO2 - безопасный PWM пин
const int SERVO_2_PIN = 3;   // GPIO3 - безопасный PWM пин

#define SERVO_CHANNEL_1 0
#define SERVO_CHANNEL_2 1

#define SERVO_FREQ 50        // 50 Гц для SG90
#define SERVO_RES 12         // 12 бит разрешение (0-4095)
#define MAX_DUTY 4095

// Лазер и динамик
const int LASER_PIN = 10;
const int BUZZER_PIN = 1;

// Преобразование микросекунд в duty cycle (для 50 Гц: период = 20000 мкс)
uint32_t pulseToDuty(float pulseUs) {
  // (pulseUs / 20000) * 4095
  return (uint32_t)((pulseUs / 20000.0) * MAX_DUTY);
}

const char* AP_SSID = "Smart_Toy_C3";
const char* AP_PASS = "12345678";

// ==================== СТРУКТУРА СЕРВО ====================
struct S_CurveServo {
  int pin;
  int id;
  int ledcChannel;
  float startPulse;
  float targetPulse;
  volatile float currentPulse;
  unsigned long moveDurationMs;
  unsigned long startTimeMs;
  volatile bool isMoving;
  volatile bool isAttached;
  volatile int progressPct;
};

// ==================== ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ====================
enum SystemState { STATE_RUNNING, STATE_PAUSED, STATE_WAIT_USER };
volatile SystemState currentSystemState = STATE_RUNNING;

char currentRunningLine[64] = "Ожидание команды СТАРТ...";
String webTerminalLog = "";

S_CurveServo servo1 = {SERVO_1_PIN, 1, SERVO_CHANNEL_1, 1450.0, 1450.0, 1450.0, 2000, 0, false, true, 0};
S_CurveServo servo2 = {SERVO_2_PIN, 2, SERVO_CHANNEL_2, 1450.0, 1450.0, 1450.0, 2000, 0, false, true, 0};

volatile bool conusModeActive = false;
volatile int webScriptLineNumber = 0;

float conusCenterPulse1 = 1450.0;
float conusCenterPulse2 = 1450.0;
float conusRadiusPulse = 0.0;
float conusAngleRad = 0.0;

TaskHandle_t TaskTrajectoryAssignment;
TaskHandle_t TaskParserAssignment;
TaskHandle_t TaskWebServerAssignment;

WebServer server(80);

volatile int commandsExecutedCounter = 0;  // Текущий счётчик команд

// ==================== RTC-ПЕРЕМЕННЫЕ ДЛЯ ГЛУБОКОГО СНА ====================
RTC_DATA_ATTR int sleepLineNumber = 0;           // Номер строки для продолжения
RTC_DATA_ATTR int sleepBytePosition = 0;         // Позиция в файле (байт)
RTC_DATA_ATTR bool sleepRestoreState = false;    // Флаг пробуждения
RTC_DATA_ATTR unsigned long sleepServo1Pulse = 1450;  // Позиция серво 1
RTC_DATA_ATTR unsigned long sleepServo2Pulse = 1450;  // Позиция серво 2
RTC_DATA_ATTR bool sleepLaserState = false;      // Состояние лазера
RTC_DATA_ATTR int sleepCommandsExecuted = 0;     // Количество выполненных команд ДО сна
RTC_DATA_ATTR uint8_t sleepMagicNumber = 0;  // Добавляем маркер для проверки

// ==================== ПРОТОТИПЫ ФУНКЦИЙ ====================
void initHardwarePWM(S_CurveServo &servo);
void writeHardwarePulse(S_CurveServo &servo, float pulseUs);
void logMessage(String msg);
void setTargetAngle(S_CurveServo &servo, int targetAngle, unsigned long durationMs);
float angleToPulse(int angle);
void createTestScript();
void Task_Trajectory_Calculator(void * pvParameters);
void Task_File_Parser(void * pvParameters);
void Task_Web_Server(void * pvParameters);
uint32_t pulseToDuty(float pulseUs);

// Функции лазера
void laserOn();
void laserOff();
void laserBlink(int durationMs, int count);
void laserOnFor(int durationMs);

// Функции звука
void beep(int frequency, int durationMs);
void mouseSqueak();
void dragonflySound();
void sirenSound();

// Функции сна
void goToDeepSleep(int seconds);

// ==================== ИНИЦИАЛИЗАЦИЯ LEDC (ПРОСТОЙ СПОСОБ) ====================
// new
// ==================== ИНИЦИАЛИЗАЦИЯ ====================
void initLEDC() {
  ledcAttach(servo1.pin, SERVO_FREQ, SERVO_RES);
  ledcAttach(servo2.pin, SERVO_FREQ, SERVO_RES);
  ledcAttach(BUZZER_PIN, 4000, 12);
  
  ledcWrite(servo1.pin, 0);
  ledcWrite(servo2.pin, 0);
  ledcWrite(BUZZER_PIN, 0);
  
  Serial.println("[LEDC] Инициализация завершена");
}

// ==================== ЗАПИСЬ ИМПУЛЬСА ====================
void writeHardwarePulse(S_CurveServo &servo, float pulseUs) {
  if (!servo.isAttached) {
    ledcWrite(servo.pin, 0);
  } else {
    uint32_t duty = pulseToDuty(pulseUs);
    ledcWrite(servo.pin, duty);
  }
}

// ==================== ИСПРАВЛЕННЫЙ БЛОК ЗВУКА ====================
void beep(int frequency, int durationMs) {
  if (frequency <= 0 || durationMs <= 0) return;
  // tone() сама настроит пин BUZZER_PIN, ledcAttach для него НЕ НУЖЕН
  tone(BUZZER_PIN, frequency, durationMs);
  vTaskDelay(pdMS_TO_TICKS(durationMs + 10));
  noTone(BUZZER_PIN);
}

void mouseSqueak() {
  beep(3500, 80);
  vTaskDelay(pdMS_TO_TICKS(50));
  beep(3800, 60);
}

void dragonflySound() {
  for (int i = 0; i < 3; i++) {
    beep(2500 + i * 300, 40);
    vTaskDelay(pdMS_TO_TICKS(30));
  }
  for (int i = 0; i < 5; i++) {
    beep(3400 - i * 200, 25);
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void sirenSound() {
  for (int i = 0; i < 10; i++) {
    tone(BUZZER_PIN, 1000 + i * 100, 50);
    vTaskDelay(pdMS_TO_TICKS(60));
  }
  for (int i = 9; i >= 0; i--) {
    tone(BUZZER_PIN, 1000 + i * 100, 50);
    vTaskDelay(pdMS_TO_TICKS(60));
  }
  noTone(BUZZER_PIN);
}


void playTone(int frequency, int durationMs) {
  beep(frequency, durationMs);
}

// Передаем секунды в скобках, как и раньше
void goToDeepSleep(int sleepTimeSeconds) {
  Serial.println("[Sleep] Подготовка к глубокому сну...");
  
  noTone(1); 
  pinMode(1, INPUT_PULLDOWN); 

  // Считаем микросекунды (переводим int в uint64_t для функции сна)
  uint64_t microSeconds = (uint64_t)sleepTimeSeconds * 1000000ULL;
  esp_sleep_enable_timer_wakeup(microSeconds);
  
  // Безопасный вывод без %llu
  Serial.print("[Sleep] Таймер установлен на ");
  Serial.print(sleepTimeSeconds);
  Serial.println(" секунд");

  Serial.println("[Sleep] Все системы отключены. Ухожу в глубокий сон.");
  Serial.flush();
  delay(100); 

  esp_deep_sleep_start();
}

// ==================== ИСПРАВЛЕННАЯ ИНИЦИАЛИЗАЦИЯ LEDC (ДЛЯ ESP32 CORE 3.x) ====================
void initHardwarePWM(S_CurveServo &servo) {
  // Теперь ничего не делаем — вся инициализация в setup()
  // Функция оставлена для совместимости
  /*
  static bool ledcInitialized = false;
  
  if (!ledcInitialized) {
    // В Core 3.x все настройки выполняются через ledcAttach()
    // ledcAttach(пин, частота, разрешение)
    ledcAttach(servo.pin, SERVO_FREQ, SERVO_RES);
    ledcInitialized = true;
    Serial.println("[LEDC] Драйвер инициализирован");
  }
  
  // Привязываем пин к каналу (в Core 3.x канал назначается автоматически)
  // ledcAttach() уже привязал пин, просто устанавливаем значение
  uint32_t duty = pulseToDuty(servo.currentPulse);
  ledcWrite(servo.pin, duty);  // В Core 3.x ledcWrite принимает пин, а не канал!
  
  logMessage("[LEDC] Пин " + String(servo.pin) + " настроен, частота " + String(SERVO_FREQ) + " Гц");
  */
}
 
float angleToPulse(int angle) {
  return 500.0 + (angle * (1900.0 / 180.0));
}

void logMessage(String msg) {
  Serial.println(msg);
  webTerminalLog += msg + "\n";
  if (webTerminalLog.length() > 2000) {
    webTerminalLog = webTerminalLog.substring(webTerminalLog.length() - 2000);
  }
}

void setTargetAngle(S_CurveServo &servo, int targetAngle, unsigned long durationMs) {
  servo.startPulse = servo.currentPulse;
  servo.targetPulse = angleToPulse(targetAngle);
  servo.moveDurationMs = durationMs;
  servo.startTimeMs = millis();
  servo.isMoving = true;
  servo.isAttached = true;
  logMessage("[ПРИКАЗ] Мотор " + String(servo.id) + " -> " + String(targetAngle) + "° (за " + String(durationMs) + " мс)");
}
void createTestScript() {
  // ВСЕГДА создаём тестовый скрипт, если его нет ИЛИ он пустой
  File testFile = LittleFS.open("/script.txt", FILE_READ);
  bool needsCreation = false;
  
  if (!testFile) {
    needsCreation = true;
    Serial.println("[ФС] Файл script.txt отсутствует");
  } else {
    // Проверяем, не пустой ли файл
    if (testFile.size() == 0) {
      needsCreation = true;
      Serial.println("[ФС] Файл script.txt пустой");
    } else {
      // Проверяем содержимое
      String content = testFile.readString();
      if (content.length() < 10) {
        needsCreation = true;
        Serial.println("[ФС] Файл script.txt слишком короткий");
      } else {
        Serial.println("[ФС] Файл script.txt найден, содержимое:");
        Serial.println(content);
      }
    }
    testFile.close();
  }
  
  if (needsCreation) {
    Serial.println("[ФС] Создаю тестовый script.txt...");
    File file = LittleFS.open("/script.txt", FILE_WRITE);
    if (file) {
      file.println("Move 30 150 2000");
      file.println("Move 150 30 2000");
      file.println("Move 90 90 1500");
      file.println("Conus 81 82 20");
      file.close();
      Serial.println("[ФС] Тестовый скрипт создан (4 команды)");
      
      // Проверяем, что записалось
      File checkFile = LittleFS.open("/script.txt", FILE_READ);
      Serial.println("[ФС] Проверка созданного файла:");
      while (checkFile.available()) {
        Serial.println("  " + checkFile.readStringUntil('\n'));
      }
      checkFile.close();
    } else {
      Serial.println("[ОШИБКА] Не могу создать script.txt!");
    }
  }
}

// ==================== HTML ИНТЕРФЕЙС ====================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=yes">
    <title>Умная Игрушка C3</title>
    <style>
        * {
            box-sizing: border-box;
            -webkit-tap-highlight-color: transparent;
        }
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Helvetica, Arial, sans-serif;
            background: #f0f2f5;
            text-align: center;
            padding: 16px;
            color: #333;
            margin: 0;
        }
        .card {
            background: white;
            padding: 20px;
            border-radius: 16px;
            box-shadow: 0 2px 8px rgba(0,0,0,0.1);
            max-width: 500px;
            margin: 0 auto 16px;
        }
        button {
            background: #007bff;
            color: white;
            border: none;
            padding: 12px 20px;
            font-size: 16px;
            border-radius: 10px;
            cursor: pointer;
            margin: 6px;
            font-weight: 500;
            transition: all 0.2s;
        }
        button:active {
            transform: scale(0.97);
            background: #0056b3;
        }
        .btn-danger { background: #dc3545; }
        .btn-success { background: #28a745; }
        .btn-warning { background: #ffc107; color: #333; }
        #terminal {
            background: #1e1e1e;
            color: #0f0;
            font-family: 'Courier New', monospace;
            text-align: left;
            padding: 12px;
            height: 200px;
            overflow-y: scroll;
            border-radius: 12px;
            font-size: 12px;
            line-height: 1.4;
        }
        .progress-bar {
            background: #e9ecef;
            border-radius: 20px;
            height: 24px;
            width: 100%;
            margin: 12px 0 5px;
            overflow: hidden;
        }
        .progress-fill {
            background: #28a745;
            height: 100%;
            width: 0%;
            transition: width 0.2s;
            border-radius: 20px;
        }
        .file-input {
            margin-top: 16px;
            padding: 16px;
            background: #f8f9fa;
            border-radius: 12px;
            border: 2px dashed #007bff;
        }
        .file-input input {
            display: block;
            width: 100%;
            padding: 12px;
            font-size: 14px;
            border: 1px solid #ccc;
            border-radius: 10px;
            background: white;
            margin-bottom: 12px;
        }
        .status-text {
            font-size: 14px;
            background: #e9ecef;
            padding: 10px;
            border-radius: 10px;
            word-break: break-word;
        }
        h2 { margin: 0 0 12px 0; font-size: 1.5rem; }
        h3 { margin: 0 0 12px 0; font-size: 1.2rem; }
    </style>
</head>
<body>
    <div class="card">
        <h2>🤖 Контроллер Игрушки</h2>
        <div class="status-text">
            <b>📟 Текущая команда:</b><br>
            <span id="curCmd" style="font-family: monospace;">Ожидание...</span>
        </div>
        <div class="progress-bar"><div id="p1" class="progress-fill"></div></div>
        <div style="font-size:12px; margin-top:-5px;">🔧 Мотор 1 Прогресс</div>
        <div class="progress-bar"><div id="p2" class="progress-fill"></div></div>
        <div style="font-size:12px; margin-top:-5px;">🔧 Мотор 2 Прогресс</div>
    </div>

    <div class="card">
        <h3>🎮 Управление</h3>
        <button onclick="sendCmd('resume')" class="btn-success">▶ СТАРТ / Продолжить</button>
        <button onclick="sendCmd('pause')" class="btn-danger">⏸ ПАУЗА</button>
        <button onclick="sendCmd('restart')" class="btn-warning">🔄 СБРОС скрипта</button>
        <div class="file-input">
          <form id="uploadForm" enctype="multipart/form-data">
            <input type="file" id="scriptFile" name="update" accept=".txt,text/plain">
            <button type="button" onclick="uploadScript()" style="background: #28a745;">⬆ ЗАГРУЗИТЬ и ЗАПУСТИТЬ</button>
          </form>
          <small style="color: #666;">📄 Выберите файл .txt - он заменит текущий скрипт</small>
        </div>
    </div>

    <div class="card">
        <h3>📊 Лог трассировки</h3>
        <div id="terminal"></div>
    </div>

    <script>
        // Функция отправки команд
        function sendCmd(action) {
            fetch('/api/' + action)
                .then(r => {
                    if (!r.ok) throw new Error('Ошибка сервера');
                    console.log("Команда отправлена:", action);
                })
                .catch(e => console.error("Ошибка:", e));
        }

        // Функция загрузки файла (работает на телефонах!)
     
         function uploadScript() {
            let fileInput = document.getElementById('scriptFile');
            let file = fileInput.files[0];
            
            if (!file) {
                alert("📁 Пожалуйста, выберите файл .txt");
                return;
            }
            
            if (!file.name.endsWith('.txt')) {
                alert("❌ Файл должен иметь расширение .txt");
                return;
            }
            
            let formData = new FormData();
            formData.append('update', file);
            
            let btn = event.target;
            let originalText = btn.innerText;
            btn.innerText = '⏳ Загрузка...';
            btn.disabled = true;
            
            fetch('/update_script', {
                method: 'POST',
                body: formData
            })
            .then(response => {
                if (response.ok) {
                    alert('✅ Скрипт загружен и запущен!');
                    fileInput.value = ''; // Очищаем поле выбора файла
                } else {
                    throw new Error('Ошибка загрузки');
                }
            })
            .catch(error => {
                console.error('Ошибка:', error);
                alert('❌ Ошибка загрузки файла');
            })
            .finally(() => {
                btn.innerText = originalText;
                btn.disabled = false;
            });
        }



        // Обновление статуса
        function updateStatus() {
            fetch('/api/status')
                .then(response => {
                    if (!response.ok) throw new Error("Статус ответа: " + response.status);
                    return response.json();
                })
                .then(data => {
                    document.getElementById('curCmd').innerHTML = escapeHtml(data.cmd);
                    document.getElementById('p1').style.width = data.p1 + '%';
                    document.getElementById('p2').style.width = data.p2 + '%';
                    
                    let term = document.getElementById('terminal');
                    let logText = data.log.replace(/\\\\n/g, '<br>').replace(/\\n/g, '<br>');
                    term.innerHTML = logText;
                    term.scrollTop = term.scrollHeight;
                })
                .catch(err => {
                    console.error("Ошибка обновления:", err);
                    let term = document.getElementById('terminal');
                    if (term) {
                        term.innerHTML = "<span style='color:#f00;'>⚠️ Ошибка связи с ESP32-C3</span><br>" + term.innerHTML;
                    }
                });
        }

        // Защита от XSS
        function escapeHtml(text) {
            let div = document.createElement('div');
            div.textContent = text;
            return div.innerHTML;
        }

        // Автообновление каждые 300 мс
        setInterval(updateStatus, 300);
        
        // Принудительно запрашиваем статус при загрузке страницы
        setTimeout(updateStatus, 100);
    </script>
</body>
</html>
)rawliteral";



// ==================== ЗАДАЧА 1: ТРАЕКТОРИИ ====================
void Task_Trajectory_Calculator(void * pvParameters) {
  const TickType_t xDelay = pdMS_TO_TICKS(20);
  
  while(1) {
    unsigned long now = millis();

    if (currentSystemState == STATE_RUNNING) {
      if (!conusModeActive) {
        
        if (servo1.isMoving) {
          unsigned long elapsed = now - servo1.startTimeMs;
          
          if (elapsed >= servo1.moveDurationMs) { 
            servo1.currentPulse = servo1.targetPulse; 
            servo1.isMoving = false; 
            servo1.startTimeMs = now;
            servo1.progressPct = 100; 
          } else {
            float progress = (float)elapsed / (float)servo1.moveDurationMs;
            servo1.progressPct = (int)(progress * 100.0);
            float sCurve = (1.0 - cos(progress * PI)) / 2.0;
            servo1.currentPulse = servo1.startPulse + (servo1.targetPulse - servo1.startPulse) * sCurve;
          }
        } else if (servo1.isAttached && (now - servo1.startTimeMs > 500)) {
          servo1.isAttached = false; 
          servo1.progressPct = 0;
          logMessage("[ЭКОНОМИЯ] Мотор 1 заснул.");
        }

        if (servo2.isMoving) {
          unsigned long elapsed = now - servo2.startTimeMs;
          
          if (elapsed >= servo2.moveDurationMs) { 
            servo2.currentPulse = servo2.targetPulse; 
            servo2.isMoving = false; 
            servo2.startTimeMs = now;
            servo2.progressPct = 100; 
          } else {
            float progress = (float)elapsed / (float)servo2.moveDurationMs;
            servo2.progressPct = (int)(progress * 100.0);
            float sCurve = (1.0 - cos(progress * PI)) / 2.0;
            servo2.currentPulse = servo2.startPulse + (servo2.targetPulse - servo2.startPulse) * sCurve;
          }
        } else if (servo2.isAttached && (now - servo2.startTimeMs > 500)) {
          servo2.isAttached = false; 
          servo2.progressPct = 0;
          logMessage("[ЭКОНОМИЯ] Мотор 2 заснул.");
        }
        
      } else {
        conusAngleRad += 0.04; 
        if (conusAngleRad >= 2.0 * PI) conusAngleRad -= 2.0 * PI;
        servo1.currentPulse = conusCenterPulse1 + (conusRadiusPulse * cos(conusAngleRad));
        servo2.currentPulse = conusCenterPulse2 + (conusRadiusPulse * sin(conusAngleRad));
        servo1.progressPct = 50; 
        servo2.progressPct = 50;
      }
    }

    writeHardwarePulse(servo1, servo1.currentPulse);
    writeHardwarePulse(servo2, servo2.currentPulse);

    vTaskDelay(xDelay);
  }
}

// ==================== ЗАДАЧА 2: ПАРСЕР СКРИПТОВ ====================
void Task_File_Parser(void * pvParameters) {
  Serial.println("[Парсер] Задача запущена!");
  logMessage("[Парсер] Поток чтения скрипта активен.");
  
  // === ПРОВЕРКА: ПРОДОЛЖЕНИЕ ПОСЛЕ СНА ===
  bool isResume = false;
  if (sleepRestoreState && sleepBytePosition > 0) {
    isResume = true;
    Serial.printf("[Парсер] Продолжение после сна: строка %d, позиция %d, команд до сна: %d\n", 
                  sleepLineNumber, sleepBytePosition, sleepCommandsExecuted);
    logMessage("[Парсер] Продолжаем со строки " + String(sleepLineNumber));
    
    // === СБРАСЫВАЕМ ФЛАГ СНА (ВАЖНО!) ===
    // Это предотвращает повторное восстановление при следующем запуске
    sleepRestoreState = false;
  }
  
  // Проверяем наличие файла
  if (!LittleFS.exists("/script.txt")) {
    Serial.println("[Парсер] ОШИБКА: файл script.txt не существует!");
    logMessage("[ОШИБКА] Нет файла script.txt!");
    TaskParserAssignment = NULL;
    vTaskDelete(NULL);
    return;
  }
  
  File scriptFile = LittleFS.open("/script.txt", FILE_READ);
  if (!scriptFile) { 
    Serial.println("[Парсер] ОШИБКА: не могу открыть script.txt!");
    logMessage("[ОШИБКА] Не удалось открыть файл!");
    TaskParserAssignment = NULL;
    vTaskDelete(NULL);
    return;
  }
  
  // === ВОССТАНОВЛЕНИЕ ПОЗИЦИИ В ФАЙЛЕ ===
  if (isResume && sleepBytePosition > 0) {
    if (scriptFile.seek(sleepBytePosition)) {
      Serial.printf("[Парсер] Восстановлена позиция: %d байт\n", sleepBytePosition);
      logMessage("[Парсер] Продолжение чтения с байта " + String(sleepBytePosition));
    } else {
      Serial.println("[Парсер] ОШИБКА: не могу восстановить позицию!");
    }
  }
  
  Serial.printf("[Парсер] Файл открыт, размер: %d байт\n", scriptFile.size());
  
  if (scriptFile.size() == 0) {
    Serial.println("[Парсер] ФАЙЛ ПУСТОЙ!");
    logMessage("[ОШИБКА] Файл script.txt пустой!");
    scriptFile.close();
    TaskParserAssignment = NULL;
    vTaskDelete(NULL);
    return;
  }

  // Используем sleepCommandsExecuted для восстановления
  int localLineCounter = isResume ? sleepLineNumber : 0;
  int commandsExecuted = isResume ? sleepCommandsExecuted : 0;
 
  // Сбрасываем флаг сна после восстановления
  sleepRestoreState = false;
  
  // === ОСНОВНОЙ ЦИКЛ ПАРСЕРА ===
  while (scriptFile.available()) {
    // Обработка глобальной паузы
    while (currentSystemState == STATE_PAUSED || currentSystemState == STATE_WAIT_USER) {
      vTaskDelay(pdMS_TO_TICKS(20));
    }

    // Запоминаем позицию в файле перед чтением строки
    long currentPos = scriptFile.position();
    
    String line = scriptFile.readStringUntil('\n');
    line.trim();
    localLineCounter++;
    
    Serial.printf("[Парсер] Строка %d: '%s'\n", localLineCounter, line.c_str());

    if (line.length() == 0 || line.startsWith("#")) {
      Serial.println("[Парсер] Пропускаем пустую строку или комментарий");
      continue;
    }

    // Обновляем глобальные переменные состояния
    webScriptLineNumber = localLineCounter; 
    strncpy(currentRunningLine, line.c_str(), sizeof(currentRunningLine) - 1);
    currentRunningLine[sizeof(currentRunningLine) - 1] = '\0';
    
    logMessage("\n[Сценарий] Строка " + String(webScriptLineNumber) + ": \"" + line + "\"");

    // Парсим команду
    char cmdName[16] = {0}; 
    int param1 = 0, param2 = 0, param3 = 0;
    int parsedItems = sscanf(line.c_str(), "%15s %d %d %d", cmdName, &param1, &param2, &param3);
    
    Serial.printf("[Парсер] Распознано %d параметров, команда: %s\n", parsedItems, cmdName);
    
    if (parsedItems >= 1) {
      String command = String(cmdName);
      
      // === ОБРАБОТКА MOVE ===
      if (command.equalsIgnoreCase("Move")) {
        if (parsedItems >= 4) {
          Serial.printf("[Парсер] Выполняем MOVE: %d %d за %d мс\n", param1, param2, param3);
          conusModeActive = false;
          setTargetAngle(servo1, param1, param3);
          setTargetAngle(servo2, param2, param3);
          
          while (servo1.isMoving || servo2.isMoving) { 
            if(currentSystemState == STATE_PAUSED) {
              servo1.startTimeMs += 20;
              servo2.startTimeMs += 20;
            }
            vTaskDelay(pdMS_TO_TICKS(20)); 
          }
          logMessage("[Выполнение] Шаг Move завершен.");
          commandsExecuted++;
        } else {
          logMessage("[ОШИБКА] Move требует 3 параметра: угол1 угол2 время (мс)");
        }
      }
      
      // === ОБРАБОТКА CONUS ===
      else if (command.equalsIgnoreCase("Conus")) {
        if (parsedItems >= 4) {
          Serial.printf("[Парсер] Выполняем CONUS: центр(%d,%d) радиус=%d\n", param1, param2, param3);
          
          setTargetAngle(servo1, param1, 2000);
          setTargetAngle(servo2, param2, 2000);
          while (servo1.isMoving || servo2.isMoving) { 
            vTaskDelay(pdMS_TO_TICKS(20)); 
          }
          
          conusCenterPulse1 = angleToPulse(param1);
          conusCenterPulse2 = angleToPulse(param2);
          conusRadiusPulse = (param3 / 2.0) * (1900.0 / 180.0);
          conusAngleRad = 0.0;
          servo1.isAttached = true; 
          servo2.isAttached = true;
          conusModeActive = true;
          
          logMessage("[Conus] Траектория конуса запущена на 8 сек.");
          unsigned long startConusTime = millis();
          while (millis() - startConusTime < 8000) {
            if (currentSystemState == STATE_PAUSED) {
              startConusTime += 20;
            } 
            vTaskDelay(pdMS_TO_TICKS(20));
          }
          conusModeActive = false;
          commandsExecuted++;
          logMessage("[Выполнение] Шаг Conus завершен.");
        } else {
          logMessage("[ОШИБКА] Conus требует 3 параметра: центр1 центр2 радиус");
        }
      }
      
      // === ОБРАБОТКА LASER ===
      else if (command.equalsIgnoreCase("Laser")) {
        // Laser 1       - включить
        // Laser 0       - выключить
        // Laser 2 100 5 - мигать 100мс, 5 раз
        // Laser 3 500   - включить на 500мс
        if (param1 == 1) {
          laserOn();
          commandsExecuted++;
          logMessage("[Лазер] Команда: ВКЛ");
        } 
        else if (param1 == 0) {
          laserOff();
          commandsExecuted++;
          logMessage("[Лазер] Команда: ВЫКЛ");
        } 
        else if (param1 == 2 && parsedItems >= 4) {
          laserBlink(param2, param3);
          commandsExecuted++;
          logMessage("[Лазер] Мигание: " + String(param2) + "мс, " + String(param3) + " раз");
        } 
        else if (param1 == 3 && parsedItems >= 3) {
          laserOnFor(param2);
          commandsExecuted++;
          logMessage("[Лазер] Включен на " + String(param2) + "мс");
        } 
        else {
          logMessage("[ОШИБКА] Неверные параметры Laser. Форматы: Laser 1|0 | Laser 2 100 5 | Laser 3 500");
        }
      }
      
      // === ОБРАБОТКА SOUND ===
      else if (command.equalsIgnoreCase("Sound")) {
        // Sound 1          - писк мышки
        // Sound 2          - трель стрекозы
        // Sound 3          - сирена
        // Sound 4 3500 100 - частота 3500 Гц, 100 мс
        if (param1 == 1) {
          mouseSqueak();
          commandsExecuted++;
          logMessage("[Звук] Писк мышки");
        } 
        else if (param1 == 2) {
          dragonflySound();
          commandsExecuted++;
          logMessage("[Звук] Трель стрекозы");
        } 
        else if (param1 == 3) {
          sirenSound();
          commandsExecuted++;
          logMessage("[Звук] Сирена");
        } 
        else if (param1 == 4 && parsedItems >= 4) {
          beep(param2, param3);
          commandsExecuted++;
          logMessage("[Звук] " + String(param2) + " Гц, " + String(param3) + "мс");
        } 
        else {
          logMessage("[ОШИБКА] Неверные параметры Sound. Форматы: Sound 1|2|3 | Sound 4 3500 100");
        }
      }
      
      // === ОБРАБОТКА PAUSE ===
      else if (command.equalsIgnoreCase("Pause")) {
        // Pause 1000 - пауза на 1000 мс
        if (parsedItems >= 2 && param1 > 0) {
          Serial.printf("[Парсер] ПАУЗА: %d мс\n", param1);
          logMessage("[Пауза] Остановка на " + String(param1) + " мс");
          
          // Разбиваем паузу на маленькие кусочки, чтобы можно было прервать глобальной паузой
          int totalDelay = param1;
          int stepDelay = 50;  // Проверяем каждые 50 мс
          while (totalDelay > 0) {
            if (currentSystemState == STATE_PAUSED || currentSystemState == STATE_WAIT_USER) {
              while (currentSystemState == STATE_PAUSED || currentSystemState == STATE_WAIT_USER) {
                vTaskDelay(pdMS_TO_TICKS(20));
              }
            }
            int delayStep = (totalDelay < stepDelay) ? totalDelay : stepDelay;
            vTaskDelay(pdMS_TO_TICKS(delayStep));
            totalDelay -= delayStep;
          }
          commandsExecuted++;
          logMessage("[Пауза] Продолжение выполнения");
        } else {
          logMessage("[ОШИБКА] Pause требует 1 параметр: время в мс (Pause 1000)");
        }
      }
      
      // === ОБРАБОТКА SLEEP ===
      else if (command.equalsIgnoreCase("Sleep")) {
        if (parsedItems >= 2 && param1 > 0) {
          Serial.printf("[Парсер] КОМАНДА SLEEP: %d секунд\n", param1);
          logMessage("[Sleep] Засыпание на " + String(param1) + " секунд");
          
          // Сохраняем состояние перед сном
          sleepLineNumber = localLineCounter;
          sleepBytePosition = currentPos;  // Позиция в файле
          sleepCommandsExecuted = commandsExecuted;
          sleepRestoreState = true;
          
          // Закрываем файл перед сном
          scriptFile.close();
          
          commandsExecuted++;
          commandsExecutedCounter = commandsExecuted;
          
          // Переходим в глубокий сон
          goToDeepSleep(param1);
          
          // Код сюда не дойдёт (после пробуждения будет перезагрузка)
        } else {
          logMessage("[ОШИБКА] Sleep требует 1 параметр: время в секундах (Sleep 3600)");
        }
      }
      
      // === НЕИЗВЕСТНАЯ КОМАНДА ===
      else {
        Serial.printf("[Парсер] Неизвестная команда: %s\n", cmdName);
        logMessage("[ОШИБКА] Неизвестная команда: " + String(cmdName));
      }
      
    } else {
      Serial.printf("[Парсер] ОШИБКА: пустая строка %d\n", localLineCounter);
      logMessage("[ОШИБКА] Строка " + String(localLineCounter) + " пустая");
    }
    
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  
  scriptFile.close();
  
  Serial.printf("[Парсер] Сценарий завершён. Обработано строк: %d, команд: %d\n", 
                localLineCounter, commandsExecuted);
  logMessage("\n[СЦЕНАРИЙ ЗАВЕРШЕН] Выполнено " + String(commandsExecuted) + " команд.");
  
  webScriptLineNumber = 0;
  strncpy(currentRunningLine, "Сценарий полностью выполнен!", sizeof(currentRunningLine) - 1);
  currentRunningLine[sizeof(currentRunningLine) - 1] = '\0';

  logMessage("[Парсер] Задача завершается, моторы обесточены.");
  
  TaskHandle_t tempHandle = TaskParserAssignment;
  TaskParserAssignment = NULL; 
  vTaskDelete(tempHandle);
}




void handleScriptUpload() {
  HTTPUpload& upload = server.upload();

  if (upload.status == UPLOAD_FILE_START) {
    Serial.println("[Upload] Начало загрузки: " + String(upload.filename));
    logMessage("[Загрузка] Начало загрузки: " + String(upload.filename));

    // Останавливаем текущий скрипт
    if (TaskParserAssignment != NULL) {
      Serial.println("[Upload] Останавливаем текущий парсер");
      vTaskDelete(TaskParserAssignment);
      TaskParserAssignment = NULL;
    }

    conusModeActive = false;
    servo1.isMoving = false;
    servo2.isMoving = false;

    // ВАЖНО: Всегда сохраняем как script.txt (перезаписываем)
    // Удаляем старый файл, если существует
    if (LittleFS.exists("/script.txt")) {
      LittleFS.remove("/script.txt");
      Serial.println("[Upload] Старый script.txt удалён");
    }

    // Создаём новый файл с фиксированным именем script.txt
    File file = LittleFS.open("/script.txt", FILE_WRITE);
    if (file) {
      file.close();
      Serial.println("[Upload] Создан новый script.txt для записи");
    } else {
      Serial.println("[Upload] ОШИБКА: не могу создать файл!");
    }

  } else if (upload.status == UPLOAD_FILE_WRITE) {
    Serial.printf("[Upload] Пишу %d байт...\n", upload.currentSize);

    // Открываем script.txt для добавления данных
    File file = LittleFS.open("/script.txt", FILE_APPEND);
    if (file) {
      file.write(upload.buf, upload.currentSize);
      file.close();
    } else {
      Serial.println("[Upload] ОШИБКА: не могу открыть файл для записи!");
    }

  } else if (upload.status == UPLOAD_FILE_END) {
    Serial.printf("[Upload] Загрузка завершена, всего %d байт\n", upload.totalSize);
    logMessage("[Загрузка] Файл загружен как script.txt, размер: " + String(upload.totalSize) + " байт");

    // Проверяем, что файл создался и не пустой
    if (LittleFS.exists("/script.txt")) {
      File checkFile = LittleFS.open("/script.txt", FILE_READ);
      int fileSize = checkFile.size();
      Serial.printf("[Upload] Размер script.txt после загрузки: %d байт\n", fileSize);

      // Выводим содержимое для отладки
      Serial.println("[Upload] Содержимое загруженного script.txt:");
      while (checkFile.available()) {
        String line = checkFile.readStringUntil('\n');
        Serial.println("  > " + line);
      }
      checkFile.close();

      if (fileSize > 0) {
        // Запускаем парсер
        currentSystemState = STATE_RUNNING;
        BaseType_t result = xTaskCreate(Task_File_Parser, "TaskParser", 8192, NULL, 1, &TaskParserAssignment);

        if (result == pdPASS) {
          Serial.println("[Upload] Парсер запущен с новым скриптом!");
          logMessage("[Загрузка] Новый скрипт загружен и запущен!");
        } else {
          Serial.println("[Upload] ОШИБКА: не могу запустить парсер!");
        }
      } else {
        Serial.println("[Upload] ОШИБКА: загружен пустой файл!");
        logMessage("[ОШИБКА] Загружен пустой файл!");
      }
    } else {
      Serial.println("[Upload] ОШИБКА: файл script.txt не создан!");
    }
  }
}


// ==================== ЗАДАЧА 3: WEB-СЕРВЕР (ИСПРАВЛЕННЫЙ) ====================
void Task_Web_Server(void * pvParameters) {
  // Главная страница
  server.on("/", HTTP_GET, []() { 
    server.send_P(200, "text/html", INDEX_HTML); 
  });
  
  // Обработчик паузы
  server.on("/api/pause", HTTP_GET, []() { 
    Serial.println("[Web] Получена команда PAUSE");
    currentSystemState = STATE_PAUSED; 
    logMessage("[Пауза] Заморозка."); 
    server.send(200, "text/plain", "OK"); 
  });
  
  // Обработчик старта/продолжения (ИСПРАВЛЕННЫЙ)
  server.on("/api/resume", HTTP_GET, []() { 
    Serial.println("[Web] Получена команда START/RESUME");
    
    if (TaskParserAssignment == NULL) {
      Serial.println("[Web] Парсер не запущен, создаю новую задачу...");
      
      // Проверяем, существует ли файл
      if (!LittleFS.exists("/script.txt")) {
        Serial.println("[ОШИБКА] Файл script.txt не найден!");
        server.send(500, "text/plain", "ERROR: script.txt not found");
        return;
      }
      
      // Проверяем размер файла
      File checkFile = LittleFS.open("/script.txt", FILE_READ);
      int fileSize = checkFile.size();
      Serial.printf("[Web] Размер файла script.txt: %d байт\n", fileSize);
      
      if (fileSize == 0) {
        Serial.println("[ОШИБКА] Файл script.txt пустой!");
        checkFile.close();
        server.send(500, "text/plain", "ERROR: script.txt is empty");
        return;
      }
      
      // Выводим содержимое файла в монитор порта для отладки
      Serial.println("[Web] Содержимое script.txt:");
      while (checkFile.available()) {
        String line = checkFile.readStringUntil('\n');
        Serial.println("  > " + line);
      }
      checkFile.close();
      
      conusModeActive = false; 
      servo1.isMoving = false; 
      servo2.isMoving = false;
      currentSystemState = STATE_RUNNING;
      
      BaseType_t result = xTaskCreate(Task_File_Parser, "TaskParser", 8192, NULL, 1, &TaskParserAssignment);
      
      if (result == pdPASS) {
        Serial.println("[Web] Парсер успешно запущен!");
        server.send(200, "text/plain", "OK");
      } else {
        Serial.println("[ОШИБКА] Не удалось создать задачу парсера!");
        server.send(500, "text/plain", "ERROR: task creation failed");
      }
    } else { 
      Serial.println("[Web] Парсер уже запущен, просто снимаю паузу");
      currentSystemState = STATE_RUNNING; 
      server.send(200, "text/plain", "OK");
    }
  });
  
  // Обработчик перезапуска скрипта
  server.on("/api/restart", HTTP_GET, []() {
    Serial.println("[Web] Получена команда RESTART");
    
    if (TaskParserAssignment != NULL) { 
      Serial.println("[Web] Останавливаю текущий парсер...");
      vTaskDelete(TaskParserAssignment); 
      TaskParserAssignment = NULL; 
    }
    
    conusModeActive = false; 
    servo1.isMoving = false; 
    servo2.isMoving = false;
    currentSystemState = STATE_RUNNING;
    
    // Проверяем файл перед запуском
    if (!LittleFS.exists("/script.txt")) {
      Serial.println("[ОШИБКА] Файл script.txt не найден!");
      server.send(500, "text/plain", "ERROR: script.txt not found");
      return;
    }
    
    BaseType_t result = xTaskCreate(Task_File_Parser, "TaskParser", 8192, NULL, 1, &TaskParserAssignment);
    
    if (result == pdPASS) {
      Serial.println("[Web] Парсер перезапущен!");
      server.send(200, "text/plain", "OK");
    } else {
      Serial.println("[ОШИБКА] Не удалось перезапустить парсер!");
      server.send(500, "text/plain", "ERROR: restart failed");
    }
  });

  // Обработчик статуса (телеметрия)
  server.on("/api/status", HTTP_GET, []() {
    int pr1 = servo1.progressPct;
    int pr2 = servo2.progressPct;

    String safeLog = webTerminalLog;
    safeLog.replace("\r", "");      
    safeLog.replace("\n", "\\n");   
    safeLog.replace("\"", "\\\"");  

    String displayCommand = String(currentRunningLine);
    if (webScriptLineNumber > 0) {
      displayCommand = "[Строка " + String(webScriptLineNumber) + "]: " + String(currentRunningLine);
    }

    String json = "{";
    json += "\"cmd\":\"" + displayCommand + "\",";
    json += "\"p1\":" + String(pr1) + ",";
    json += "\"p2\":" + String(pr2) + ",";
    json += "\"log\":\"" + safeLog + "\"";
    json += "}";
    
    server.send(200, "application/json", json);
  });

  // Обработчик загрузки файлов (исправленный)
  server.on("/update_script", HTTP_POST, []() { 
    server.send(200, "text/plain", "OK"); 
  }, handleScriptUpload);
  
  // Запускаем сервер
  server.begin();
  Serial.println("[Web] Сервер запущен на порту 80");
  logMessage("[Web] Сервер запущен на порту 80");

  // ВАЖНО: Разрешаем подключение с любых устройств
  server.enableCORS(true);  // Если есть такая функция

  while(1) { 
    server.handleClient(); 
  
    // Проверяем, что Wi-Fi всё ещё работает
    if (WiFi.softAPgetStationNum() == 0) {
      // Нет подключённых клиентов — ничего не делаем, ждём
    }
    vTaskDelay(pdMS_TO_TICKS(10)); 
  }  
}

// ==================== ФУНКЦИИ ЛАЗЕРА ====================
void laserOn() {
  digitalWrite(LASER_PIN, HIGH);
  logMessage("[Лазер] Включен");
}

void laserOff() {
  digitalWrite(LASER_PIN, LOW);
  logMessage("[Лазер] Выключен");
}

// Мигание с использованием vTaskDelay (неблокирующее)
void laserBlink(int durationMs, int count) {
  for (int i = 0; i < count; i++) {
    laserOn();
    vTaskDelay(pdMS_TO_TICKS(durationMs));
    laserOff();
    if (i < count - 1) {
      vTaskDelay(pdMS_TO_TICKS(durationMs));
    }
  }
}

// Включить лазер на время (с неблокирующей задержкой)
void laserOnFor(int durationMs) {
  laserOn();
  vTaskDelay(pdMS_TO_TICKS(durationMs));
  laserOff();
}

// ==================== SETUP ====================
//new
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n========================================");
  Serial.println("   ESP32-C3 Super Mini - КИНЕМАТИКА");
  Serial.println("========================================");

  // === ДИАГНОСТИКА ПРОБУЖДЕНИЯ ===
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  Serial.printf("[DIAG] Причина пробуждения: %d\n", wakeup_reason);

  bool isDeepSleepWake = false;
  if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER && sleepMagicNumber == 0xAA) {
    Serial.println("=== ПРОБУЖДЕНИЕ ПО ТАЙМЕРУ ИЗ ГЛУБОКОГО СНА! ===");
    isDeepSleepWake = true;
    delay(500);
  } else if (wakeup_reason == ESP_SLEEP_WAKEUP_UNDEFINED) {
    Serial.println("=== ПЕРВЫЙ ЗАПУСК (НОРМАЛЬНАЯ ЗАГРУЗКА) ===");
  } else {
    Serial.printf("=== ДРУГАЯ ПРИЧИНА: %d ===\n", wakeup_reason);
  }

  // === ИНИЦИАЛИЗАЦИЯ ПИНОВ (только GPIO, без LEDC) ===
  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);
  
  // НЕ ИНИЦИАЛИЗИРУЕМ LEDC ЗДЕСЬ! — перенесли позже

  // === ИНИЦИАЛИЗАЦИЯ LittleFS ===
  if (!LittleFS.begin(true)) {
    Serial.println("[ОШИБКА] LittleFS не смонтирована!");
    while (1);
  }
  Serial.println("[ФС] LittleFS смонтирована");

  // === СОЗДАНИЕ ТЕСТОВОГО СКРИПТА ===
  if (!LittleFS.exists("/script.txt") || LittleFS.open("/script.txt", FILE_READ).size() == 0) {
    createTestScript();
    Serial.println("[ФС] Создан тестовый script.txt");
  }

  // === ВКЛЮЧЕНИЕ Wi-Fi (ПЕРВЫМ!) ===
  Serial.println("[WiFi] Запуск точки доступа...");

  WiFi.mode(WIFI_OFF);
  delay(100);

  WiFi.mode(WIFI_AP);
  esp_wifi_set_ps(WIFI_PS_NONE);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);

  bool apStarted = WiFi.softAP(AP_SSID, AP_PASS, 6, 0, 4);
  if (apStarted) {
    Serial.println("[WiFi] Точка доступа запущена!");
    Serial.println("[WiFi] IP: " + WiFi.softAPIP().toString());
  } else {
    Serial.println("[ОШИБКА] Не удалось запустить точку доступа!");
  }

  delay(500);
    // === ИНИЦИАЛИЗАЦИЯ LEDC (ПОСЛЕ Wi-Fi) ===
  Serial.println("[LEDC] Инициализация...");
  ledcAttach(servo1.pin, SERVO_FREQ, SERVO_RES);
  ledcAttach(servo2.pin, SERVO_FREQ, SERVO_RES);
  
  // !!! СТРОКУ НИЖЕ УДАЛЯЕМ, ЧТОБЫ НЕ БЫЛО КОНФЛИКТА С tone() !!!
  // ledcAttach(BUZZER_PIN, 4000, 12); 
  
  ledcWrite(servo1.pin, 0);
  ledcWrite(servo2.pin, 0);
  // Вместо ledcWrite для бузера используем noTone
  noTone(BUZZER_PIN); 
  Serial.println("[LEDC] Инициализация завершена");

  // Восстанавливаем позиции серво
  if (isDeepSleepWake && sleepServo1Pulse > 0) {
    servo1.currentPulse = (float)sleepServo1Pulse;
    servo2.currentPulse = (float)sleepServo2Pulse;
    servo1.isAttached = true;
    servo2.isAttached = true;
  } else {
    servo1.currentPulse = 1450.0;
    servo2.currentPulse = 1450.0;
    servo1.isAttached = true;
    servo2.isAttached = true;
  }
  writeHardwarePulse(servo1, servo1.currentPulse);
  writeHardwarePulse(servo2, servo2.currentPulse);

  delay(100);

  // === ЗАПУСК ЗАДАЧ ===
  xTaskCreate(Task_Trajectory_Calculator, "TaskTrajectory", 8192, NULL, 2, &TaskTrajectoryAssignment);
  xTaskCreate(Task_Web_Server, "TaskWebServer", 16384, NULL, 1, &TaskWebServerAssignment);

  // Сразу начинаем выполнять  (ПРОБА1)
  currentSystemState = STATE_RUNNING;
  xTaskCreate(Task_File_Parser, "TaskParser", 8192, NULL, 1, &TaskParserAssignment);


  
  /*
  // === ЗАПУСК ПАРСЕРА ===
  if (isDeepSleepWake && sleepBytePosition > 0) {
    Serial.printf("[Восстановление] Продолжаем со строки %d, позиция %d\n",
                  sleepLineNumber, sleepBytePosition);
    currentSystemState = STATE_RUNNING;
    xTaskCreate(Task_File_Parser, "TaskParser", 8192, NULL, 1, &TaskParserAssignment);
  } else {
    currentSystemState = STATE_WAIT_USER;
    strncpy(currentRunningLine, "Ожидание команды СТАРТ...", sizeof(currentRunningLine) - 1);
    currentRunningLine[sizeof(currentRunningLine) - 1] = '\0';
    TaskParserAssignment = NULL;
  }
  */

  Serial.println("[Система] Готова к работе!");
}

void loop() { 
  vTaskDelay(pdMS_TO_TICKS(1000)); 
}