#include <LiquidCrystal_I2C.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// --- LCD ---
LiquidCrystal_I2C lcd(0x27, 16, 2);

// --- Declaración de pines ---
const int TEMP_PIN = 2;  // LM35 on A2 (GPIO4)
const int ledPin = 5;     // Use as needed (GPIO6, adjust if you wish)
#define trigPin 10  // trigPin connected to GPIO6
#define echoPin 9  // echoPin connected to GPIO7
const int ACCEL_X_PIN = 3; // Accelerometer X analog input
const int ACCEL_Y_PIN = 4; // Accelerometer Y analog input
const int PWM_SPEAKER    = 21;    // PWM output (Speaker/Alarm)
const int PWM_FAN        = 20;    // PWM output (Fan/Motor)

// --- Configuración BLE ---
BLEServer* pServer = nullptr;
BLECharacteristic* pControlCharacteristic = nullptr;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// UUIDs
#define SERVICE_UUID        "19b10000-e8f2-537e-4f6c-d104768a1214"
#define CONTROL_CHAR_UUID   "19b10002-e8f2-537e-4f6c-d104768a1214"

#define MODE_NONE      0
#define MODE_CONFORT   1
#define MODE_ANTIRROBO 2

uint8_t currentMode = MODE_NONE;
bool alarmOn = false;
bool inAntirroboMode = false; 

// ********** Declaracion de variables ********//
//Modo Confort
unsigned int Vbin_x, Vbin_y;
float Vana_x, Vana_y;
float x_ant = 0.0, y_ant = 0.0;
bool fanManual = false;  // true: manual, false: auto
int fanLevel = 0;        // 0 = off, 1 = med, 2 = max

//Modo Antirrobo
float mov_total = 0;
float distance = 0;
char accel_first_time = 1; // flag para ignorar la primera lectura
unsigned int accel_sample_counter = 0;
#define ACCEL_SAMPLE_PERIOD 20      // Frecuencia de actualización del acelerómetro (aprox. 500 ms)
unsigned long lastAlarmCheck = 0;
const unsigned long alarmCheckInterval = 500; // ms
int alarm_triggered = 0;

//Auxiliares para LCD
int lcd_counter = 0;
const int lcd_update_period = 25; // 500ms / 20ms ≈ 25

//Auxiliares para HTML
unsigned long lastNotify = 0;
const unsigned long notifyInterval = 500; // ms


// --- Función actualización LCD  ---
void updateLCDStatus() {
  lcd.setCursor(0, 0);

  if (currentMode == MODE_CONFORT) {
    lcd.print("Modo Confort    ");
    lcd.setCursor(0, 1);
    lcd.print("Temp: ");
    float temp = readTemperature();
    lcd.print(temp, 1);
    lcd.print((char)223);
    lcd.print("C   ");
  } else if (currentMode == MODE_ANTIRROBO) {
    lcd.print("Modo Antirrobo  ");
    lcd.setCursor(0, 1);
    if (!alarmOn) {
      lcd.print("Alarm OFF       ");
    } else if (alarm_triggered) {
      lcd.print("ALERT!!         ");
    } else {
      lcd.print("Alarm ON        ");
    }
  } else {
    lcd.print("Bienvenido      ");
    lcd.setCursor(0, 1);
    lcd.print("Seleccione modo ");
  }
}

// --- BLE Callbacks ---
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override { deviceConnected = true; }
  void onDisconnect(BLEServer* pServer) override { deviceConnected = false; }
};

class ControlCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
    String value = pControlCharacteristic->getValue();
    
    //Modo Confort
    if (value == "c") { 
      currentMode = MODE_CONFORT;
      alarmOn = false;
      digitalWrite(ledPin, LOW);
      analogWrite(PWM_SPEAKER, 0);
      analogWrite(PWM_FAN, 0);   // Off
    }else if (value == "fan:auto") {
      fanManual = false;
      fanLevel = 0;
    } else if (value == "fan:off") {
      fanManual = true;
      fanLevel = 0;
    } else if (value == "fan:med") {
      fanManual = true;
      fanLevel = 1;
    } else if (value == "fan:max") {
      fanManual = true;
      fanLevel = 2;
    } 

    //Modo Antirrobo
    else if (value == "a") { 
      currentMode = MODE_ANTIRROBO;
      alarmOn = false;
      digitalWrite(ledPin, LOW);
      analogWrite(PWM_SPEAKER, 0);
      analogWrite(PWM_FAN, 0);   // Off
    } else if (value == "alarm:on" && currentMode == MODE_ANTIRROBO) {
      alarmOn = true;
      digitalWrite(ledPin, LOW); 
      accel_first_time = 1; // Reseteamos el flag cada vez que se activa el modo
    } else if (value == "alarm:off" && currentMode == MODE_ANTIRROBO) {
      alarmOn = false;
      digitalWrite(ledPin, LOW);
      analogWrite(PWM_SPEAKER, 0);
      analogWrite(PWM_FAN, 0);   // Off
    }
    //Actualizamos LCD
    updateLCDStatus();
    lcd_counter = 0;
  }
};


// ********** Funciones auxiliares **********// 
// Test I2C address for LCD
bool i2CAddrTest(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

//Lectura LM35
float readTemperature() {
  uint16_t val = analogRead(TEMP_PIN);
  return ((double)val * (3.3 / 4095.0) * 100.0); // 10mV/°C, 12-bit ADC, 3.3V ref
}

//Lectura HC-SR04
float getSonar() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long pingTime = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
  if (pingTime == 0) return -1;
  float distance = (float)pingTime / 58.0;
  return distance;
}

void checkAlarmSensors() {
  // Primera lectura para evitar falsos positivos
  if (accel_first_time == 1) {
    Vbin_x = analogRead(ACCEL_X_PIN);
    Vbin_y = analogRead(ACCEL_Y_PIN);
    Vana_x = 5000.0 * (float)Vbin_x / 1023.0;
    Vana_y = 5000.0 * (float)Vbin_y / 1023.0;
    x_ant = Vana_x;
    y_ant = Vana_y;
    getSonar(); 
    accel_sample_counter = 0;
    accel_first_time = 0;
    alarm_triggered = 0;
    return; 
  }

  // Lectura real de los sensores
  Vbin_x = analogRead(ACCEL_X_PIN);
  Vbin_y = analogRead(ACCEL_Y_PIN);
  Vana_x = 5000.0 * (float)Vbin_x / 1023.0;
  Vana_y = 5000.0 * (float)Vbin_y / 1023.0;
  float delta_x = fabs(Vana_x - x_ant);
  float delta_y = fabs(Vana_y - y_ant);
  mov_total = delta_x + delta_y;

  accel_sample_counter++;
  if (accel_sample_counter >= ACCEL_SAMPLE_PERIOD) {
    x_ant = Vana_x;
    y_ant = Vana_y;
    accel_sample_counter = 0;
  }

  distance = getSonar();

  // Lógica de activación de la alarma
  if (distance >= 20) {
    alarm_triggered = 0; // Apagamos si la distancia supera los 20 cm
    x_ant = Vana_x;
    y_ant = Vana_y;
  } else if (distance > 0 && mov_total > 500) {
    alarm_triggered = 1; 
  }
}


// --- Funciones configuración BLE  ---
void setupBLE() {
  BLEDevice::init("ESP32");
  pServer = BLEDevice::createServer();

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pControlCharacteristic = pService->createCharacteristic(
    CONTROL_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
  );
  pControlCharacteristic->setCallbacks(new ControlCallbacks());
  pControlCharacteristic->addDescriptor(new BLE2902());

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);
  BLEDevice::startAdvertising();
}

// ************ FUNCIONES PROPIAS ARDUINO ************** //
void setup() {
  //LCD
  Serial.begin(115200);
  delay(100);
  Wire.begin();

  //Pines como E/S
  pinMode(ledPin, OUTPUT);
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT);  
  pinMode(ACCEL_X_PIN, INPUT);
  pinMode(ACCEL_Y_PIN, INPUT);
  pinMode(PWM_SPEAKER, OUTPUT);
  pinMode(PWM_FAN, OUTPUT);

  digitalWrite(ledPin, LOW);
  analogWrite(PWM_FAN, 0);
  analogWrite(PWM_SPEAKER, 0);

  // LCD INIT
  if (!i2CAddrTest(0x27)) {
    lcd = LiquidCrystal_I2C(0x3F, 16, 2);
  }
  lcd.init();
  lcd.backlight();
  updateLCDStatus();

  // BLE INIT
  setupBLE();
}


void loop() {
  //Variables locales
  static int last_alarm_triggered = -1;
  static int lastFanLevel = -1;
  static int lastFanManual = false;
  
  // Lectura temperatura
  float temp = readTemperature();

  // --- BLE Notify ---
  if (currentMode == MODE_CONFORT && (millis() - lastNotify > notifyInterval)) {
    updateLCDStatus();
    if (fanManual) {
      // Control Manual
      if (fanLevel == 1) {
        analogWrite(PWM_FAN, 128);
      } else if (fanLevel == 2) {
        analogWrite(PWM_FAN, 255);
      } else {
        analogWrite(PWM_FAN, 0);
      }
    } else {
      // Control Automatico
      if (temp >= 30 && temp <= 32) {
        fanLevel = 1;
        analogWrite(PWM_FAN, 128);
      } else if (temp > 32) {
        fanLevel = 2;
        analogWrite(PWM_FAN, 255);
      } else {
        fanLevel = 0;
        analogWrite(PWM_FAN, 0);
      }
    }
    //Envio de temperatura a HTML
    char tempStr[8];
    dtostrf(temp, 1, 1, tempStr);
    pControlCharacteristic->setValue(tempStr);
    pControlCharacteristic->notify();

    //Envio de modo de ventilador a HTML
    if (fanLevel != lastFanLevel || fanManual != lastFanManual) {
      String modeStr = fanManual ? "manual" : "auto";
      String levelStr = (fanLevel == 2) ? "max" : (fanLevel == 1) ? "med" : "off";
      String fanStatus = "{\"fan_mode\":\"" + modeStr + "\",\"fan_level\":\"" + levelStr + "\"}";
      pControlCharacteristic->setValue(fanStatus.c_str());
      pControlCharacteristic->notify();
      lastFanLevel = fanLevel;
      lastFanManual = fanManual;
    }
    lastNotify = millis(); 
  }

  if (currentMode == MODE_ANTIRROBO) {
    if (alarmOn && (millis() - lastAlarmCheck > alarmCheckInterval)) {
      checkAlarmSensors(); // Comprobamos sensores para actualizar alarm_triggered
      if (alarm_triggered != last_alarm_triggered) {
      
      //Envio notificación robo
      if (pControlCharacteristic != nullptr) {
        String motionJson = "{\"motion\":";
        motionJson += (alarm_triggered ? "true" : "false");
        motionJson += "}";
        pControlCharacteristic->setValue(motionJson.c_str());
        pControlCharacteristic->notify();
      }
       last_alarm_triggered = alarm_triggered;
      }

      if (alarm_triggered) {
       // Activamos LED y altavoz
       digitalWrite(ledPin, HIGH);
       analogWrite(PWM_SPEAKER, 51); // ~20% duty
       delay(200);
       analogWrite(PWM_SPEAKER, 191);// ~75% duty
       digitalWrite(ledPin, LOW);
       delay(80);
      } else {
       analogWrite(PWM_SPEAKER, 0); 
      }
       lastAlarmCheck = millis();
    }else if (!alarmOn) {
       alarm_triggered = 0;
    }
  } 
  
  // Frecuencia actualización LCD
  lcd_counter++;
  if (lcd_counter >= lcd_update_period) {
    updateLCDStatus();
    lcd_counter = 0;
  }
  
  // --- Conexión BLE  ---
  if (!deviceConnected && oldDeviceConnected) {
    Serial.println("Device disconnected.");
    delay(500);
    pServer->startAdvertising();
    Serial.println("Restart advertising");
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
    Serial.println("Device Connected");
    oldDeviceConnected = deviceConnected;
  }

  delay(20);
}