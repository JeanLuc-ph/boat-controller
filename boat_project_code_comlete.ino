*#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <MPU6050.h>
#include <QMC5883LCompass.h>

// Configuration WiFi
const char* ssid = "ESP32_Moteur";
const char* password = "12345678";

WebServer server(80);

// Configuration ACS712
const int ACS712_PIN = 34;       // GPIO34 (ADC1)
const float ACS712_OFFSET = 2.5;  // Tension à 0A (VCC/2)
const float ACS712_SENSITIVITY = 0.1; // 100mV/A pour 20A

// Configuration Moteur DC
const int motorPin = 25;          // GPIO25 -> 2N2222 base
bool motorRunning = false;
int motorSpeed = 0;

// Configuration Servo
const int servoPin = 16;          // GPIO16
Servo myServo;
int servoAngle = 90;
const int SERVO_MIN = 55;   // Limites physiques
const int SERVO_MAX = 125;

// Capteurs
MPU6050 mpu;
QMC5883LCompass compass;
bool autoMode = false;
bool sensorsReady = false;

// Variables MPU6050
int16_t ax, ay, az, gx, gy, gz;
float accAngleX, accAngleY;
float gyroAngleX = 0, gyroAngleY = 0;
float angleX = 0, angleY = 0;  // Angles fusionnés
float accelErrorX = 0, accelErrorY = 0, accelErrorZ = 0;
float gyroErrorX = 0, gyroErrorY = 0, gyroErrorZ = 0;

// Variables boussole
int heading = 0;
int targetHeading = 0; // Nord magnétique

// PID
const float Kp = 1.2f;
const float Kd = 0.4f;
float previousError = 0;

void setup() {
  Serial.begin(115200);

  // Initialisation I2C
  Wire.begin(21, 22); // SDA=GPIO21, SCL=GPIO22
  Wire.setClock(400000);

  // Initialisation capteurs
  initializeSensors();

  // Initialisation ACS712
  pinMode(ACS712_PIN, INPUT);

  // Initialisation Moteur DC
  pinMode(motorPin, OUTPUT);
  digitalWrite(motorPin, LOW); // Moteur OFF initialement

  // Initialisation Servo
  myServo.attach(servoPin);
  myServo.write(servoAngle);

  // Point d'accès WiFi
  WiFi.softAP(ssid, password);
  
  Serial.println("\nPoint d'accès créé : " + String(ssid));
  Serial.println("Adresse IP : " + WiFi.softAPIP().toString());

  // Routes serveur web
  server.on("/", handleRoot);
  server.on("/start", handleStart);
  server.on("/stop", handleStop);
  server.on("/setSpeed", handleSetSpeed);
  server.on("/setAngle", handleSetAngle);
  server.on("/current", handleCurrent);
  server.on("/setmode", handleSetMode);
  server.on("/calibrate", handleCalibrate);
  server.on("/navigation", handleNavigation);
  
  server.begin();
}

void initializeSensors() {
  // Initialisation MPU6050
  mpu.initialize();
  if(mpu.testConnection()) {
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    calibrateMPU6050();
  } else {
    Serial.println("MPU6050 non détecté!");
  }

  // Initialisation boussole
  Wire.beginTransmission(0x0D); // Adresse I2C du QMC5883L
  if(Wire.endTransmission() == 0) {
    compass.init();
    compass.setCalibration(-1767, 1350, -1558, 960, -2098, 1117);
    sensorsReady = true;
    Serial.println("Boussole initialisée");
  } else {
    Serial.println("Boussole non détectée!");
  }
}

void calibrateMPU6050() {
  Serial.println("Calibration MPU6050 - Ne pas bouger!");
  delay(1000);
  
  // Moyenne sur 200 échantillons
  int16_t sumAccX = 0, sumAccY = 0, sumAccZ = 0;
  int16_t sumGyroX = 0, sumGyroY = 0, sumGyroZ = 0;
  
  for (int i = 0; i < 200; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sumAccX += ax;
    sumAccY += ay;
    sumAccZ += az;
    sumGyroX += gx;
    sumGyroY += gy;
    sumGyroZ += gz;
    delay(5);
  }
  
  // Calcul des erreurs
  accelErrorX = sumAccX / 200;
  accelErrorY = sumAccY / 200;
  accelErrorZ = sumAccZ / 200 - 16384; // 1g = 16384 LSB
  gyroErrorX = sumGyroX / 200;
  gyroErrorY = sumGyroY / 200;
  gyroErrorZ = sumGyroZ / 200;
  
  Serial.println("Calibration MPU6050 terminée");
}

void updateIMU() {
  static unsigned long lastUpdate = micros();
  float dt = (micros() - lastUpdate) / 1000000.0f;
  lastUpdate = micros();
  
  // Lecture MPU6050
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Conversion et calibration
  float accelX = (ax - accelErrorX) / 16384.0f;
  float accelY = (ay - accelErrorY) / 16384.0f;
  float accelZ = (az - accelErrorZ) / 16384.0f;
  float gyroX = (gx - gyroErrorX) / 131.0f;
  float gyroY = (gy - gyroErrorY) / 131.0f;
  float gyroZ = (gz - gyroErrorZ) / 131.0f;

  // Calcul des angles accéléromètre
  accAngleX = atan(accelY / sqrt(pow(accelX, 2) + pow(accelZ, 2))) * 180 / PI;
  accAngleY = atan(-1 * accelX / sqrt(pow(accelY, 2) + pow(accelZ, 2))) * 180 / PI;

  // Intégration gyroscopique
  gyroAngleX += gyroX * dt;
  gyroAngleY += gyroY * dt;

  // Filtre complémentaire
  angleX = 0.96 * gyroAngleX + 0.04 * accAngleX;
  angleY = 0.96 * gyroAngleY + 0.04 * accAngleY;
}

void updateCompass() {
  compass.read();
  heading = compass.getAzimuth();
  Serial.print("Cap: "); Serial.println(heading); // Debug
}

void autoPilot() {
  if (!sensorsReady) return;
  
  updateCompass();
  updateIMU();
  
  // Calcul d'erreur amélioré avec wrap-around
  float error = atan2(sin((heading-targetHeading)*PI/180), 
                    cos((heading-targetHeading)*PI/180)) * 180/PI;
  
  // PID
  float correction = Kp * error + Kd * (error - previousError);
  previousError = error;
  
  // Compensation inclinaison
  correction += constrain(angleY * 0.8f, -15.0f, 15.0f);
  
  // Calcul angle servo
  int newAngle = map(constrain(correction, -45.0f, 45.0f), -45, 45, SERVO_MIN, SERVO_MAX);
  
  servoAngle = newAngle; // Pas de filtre pour une réponse plus rapide
  myServo.write(servoAngle);

  // Debug
  Serial.print("Error: "); Serial.print(error);
  Serial.print(" | Correction: "); Serial.print(correction);
  Serial.print(" | Servo: "); Serial.println(servoAngle);
}

void loop() {
  server.handleClient();
  
  if (autoMode && sensorsReady) {
    autoPilot();
  }
  
  // Debug des valeurs brutes (optionnel)
  static unsigned long lastDebug = 0;
  if(millis() - lastDebug > 1000) {
    lastDebug = millis();
    checkCompassValues();
  }
}

void checkCompassValues() {
  compass.read();
  Serial.print("X: "); Serial.print(compass.getX());
  Serial.print(" Y: "); Serial.print(compass.getY());
  Serial.print(" Z: "); Serial.println(compass.getZ());
}

float readCurrent() {
  float sum = 0;
  for(int i=0; i<10; i++) {
    sum += analogRead(ACS712_PIN) * (3.3 / 4095.0);
    delay(1);
  }
  float voltage = sum / 10.0;
  return (voltage - ACS712_OFFSET) / ACS712_SENSITIVITY;
}

void setMotorSpeed(int speed) {
  speed = constrain(speed, 0, 255);
  motorSpeed = speed;
  
  if (speed == 0) {
    motorRunning = false;
    digitalWrite(motorPin, LOW);
  } else {
    motorRunning = true;
    analogWrite(motorPin, speed);
  }
}

void handleRoot() {
  String html = R"=====(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Contrôle Bateau ESP32</title>
  <style>
    body {font-family: Arial; text-align: center; margin: 0 auto; padding: 20px;}
    .panel {background: #f5f5f5; border-radius: 10px; padding: 20px; margin: 10px;}
    .slider {width: 80%; margin: 15px 0;}
    .status {font-weight: bold; color: #e74c3c;}
    .btn-start {background-color: #4CAF50; color: white; padding: 10px 20px; border: none; border-radius: 4px; margin: 5px;}
    .btn-stop {background-color: #f44336; color: white; padding: 10px 20px; border: none; border-radius: 4px; margin: 5px;}
    .slider:disabled {opacity: 0.6;}
    .status-indicator {
      display: inline-block;
      width: 15px;
      height: 15px;
      border-radius: 50%;
      margin-right: 10px;
    }
    .status-online {background-color: #2ecc71;}
    .status-offline {background-color: #e74c3c;}
  </style>
</head>
<body>
  <h1>🚤 Contrôle Bateau ESP32</h1>
  
  <div class="panel">
    <h2>🚗 Moteur DC</h2>
    <button class="btn-start" onclick="startMotor()">Démarrer</button>
    <button class="btn-stop" onclick="stopMotor()">Arrêter</button>
    <p>Vitesse: <span id="speedValue">0</span>% <input type="range" min="0" max="255" value="0" class="slider" id="speedSlider" oninput="updateSpeed(this.value)" disabled></p>
    <p id="motorStatus">État: Arrêté</p>
  </div>

  <div class="panel">
    <h2>🔄 Servo</h2>
    <p>Angle: <span id="angleValue">90</span>° <input type="range" min="0" max="180" value="90" class="slider" id="angleSlider" oninput="updateAngle(this.value)"></p>
    <div>
      <span class="status-indicator" id="auto-status"></span>
      <label>Mode automatique: <input type="checkbox" id="auto-mode" onchange="toggleAutoMode()"></label>
    </div>
  </div>

  <div class="panel">
    <h2>🌍 Navigation</h2>
    <p>Cap actuel: <span id="heading-value">0</span>°</p>
    <p>Inclinaison: <span id="inclination-value">0.0</span>°</p>
    <button onclick="calibrateSensors()">Calibrer capteurs</button>
  </div>

  <div class="panel">
    <h2>🔌 Courant Moteur</h2>
    <p id="currentDisplay">0.00 A</p>
  </div>

  <script>
    function updateCurrent() {
      fetch('/current')
        .then(r => r.text())
        .then(t => document.getElementById('currentDisplay').innerHTML = t + ' A');
      setTimeout(updateCurrent, 1000);
    }
    
    function updateNavigation() {
      fetch('/navigation')
        .then(r => r.json())
        .then(data => {
          document.getElementById('heading-value').textContent = data.heading;
          document.getElementById('inclination-value').textContent = data.inclination.toFixed(1);
          document.getElementById('auto-status').className = 
            'status-indicator ' + (data.sensorsReady ? 'status-online' : 'status-offline');
          document.getElementById('auto-mode').checked = data.autoMode;
        });
      setTimeout(updateNavigation, 500);
    }
    
    function startMotor() { 
      fetch('/start')
        .then(() => {
          document.getElementById('motorStatus').innerText = 'État: En marche';
          document.getElementById('speedSlider').value = 128;
          document.getElementById('speedSlider').disabled = false;
          document.getElementById('speedValue').innerText = '50';
        });
    }
    
    function stopMotor() { 
      fetch('/stop')
        .then(() => {
          document.getElementById('motorStatus').innerText = 'État: Arrêté';
          document.getElementById('speedSlider').value = 0;
          document.getElementById('speedSlider').disabled = true;
          document.getElementById('speedValue').innerText = '0';
        });
    }
    
    function updateSpeed(val) { 
      fetch('/setSpeed?value=' + val);
      document.getElementById('speedValue').innerText = Math.round(val/255*100);
    }
    
    function updateAngle(val) { 
      fetch('/setAngle?value=' + val);
      document.getElementById('angleValue').textContent = val;
    }
    
    function toggleAutoMode() {
      const autoMode = document.getElementById('auto-mode').checked;
      fetch('/setmode?mode=' + (autoMode ? 'auto' : 'manual'));
    }
    
    function calibrateSensors() {
      fetch('/calibrate')
        .then(r => {
          if (r.ok) alert('Calibration réussie!');
          else alert('Erreur de calibration');
        });
    }
    
    // Initialisation
    updateCurrent();
    updateNavigation();
  </script>
</body>
</html>
)=====";

  server.send(200, "text/html", html);
}

void handleStart() {
  if(!motorRunning) {
    setMotorSpeed(128);
    motorRunning = true;
  }
  server.send(200, "text/plain", "OK");
}

void handleStop() {
  if(motorRunning) {
    setMotorSpeed(0);
    motorRunning = false;
  }
  server.send(200, "text/plain", "OK");
}

void handleSetSpeed() {
  if(server.hasArg("value") && motorRunning) {
    setMotorSpeed(server.arg("value").toInt());
  }
  server.send(200, "text/plain", "OK");
}

void handleSetAngle() {
  if(server.hasArg("value")) {
    servoAngle = server.arg("value").toInt();
    myServo.write(servoAngle);
  }
  server.send(200, "text/plain", "OK");
}

void handleCurrent() {
  server.send(200, "text/plain", String(readCurrent(), 2));
}

void handleSetMode() {
  if(server.hasArg("mode")) {
    autoMode = (server.arg("mode") == "auto");
    server.send(200, "text/plain", "OK");
  }
}

void handleCalibrate() {
  calibrateMPU6050();
  compass.setCalibration(-1767, 1350, -1558, 960, -2098, 1117);
  sensorsReady = mpu.testConnection();
  server.send(200, "text/plain", "Calibration effectuée");
}

void handleNavigation() {
  updateCompass(); // Force la mise à jour avant l'envoi
  String json = "{";
  json += "\"heading\":" + String(heading) + ",";
  json += "\"inclination\":" + String(angleY, 2) + ",";
  json += "\"autoMode\":" + String(autoMode) + ",";
  json += "\"sensorsReady\":" + String(sensorsReady);
  json += "}";
  server.send(200, "application/json", json);
}