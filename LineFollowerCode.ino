#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <QTRSensors.h>

// --- CONFIGURATION PINS ---
const int IN1 = 26; const int IN2 = 27; // Moteur Gauche
const int IN3 = 14; const int IN4 = 12; // Moteur Droit
const int ENA = 25; const int ENB = 28; // PWM Vitesse
const int LED_INIT = 13;    // LED Initialisation
const int LED_RUNNING = 10; // LED Pilotage actif

// --- PARAMÈTRES PID & VITESSE ---
float Kp = 0.4; 
float Kd = 1.2;
int lastError = 0;
int baseSpeed = 160;
volatile bool robotStop = false;      // Arrêt d'urgence
volatile bool robotForward = true;    // Direction: true=avant, false=arrière
uint16_t lastPosition = 3500;         // Dernière position capteur
unsigned long lastMovementTime = 0;   // Timestamp du dernier mouvement
bool ledBlinking = false;             // État LED clignotante

// --- OBJETS ---
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
AsyncWebServer server(80);

// --- INTERFACE WEB (Stockée en Flash) ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head><title>ZEBI Robot Control</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  * { margin: 0; padding: 0; box-sizing: border-box; }
  body { font-family: Arial, sans-serif; background: #1a1a1a; color: #fff; padding: 10px; }
  .container { max-width: 500px; margin: 0 auto; }
  h1 { text-align: center; margin: 20px 0; font-size: 28px; }
  .status { text-align: center; padding: 15px; background: #222; border-radius: 8px; margin-bottom: 20px; font-size: 14px; }
  .status.running { background: #1a5c1a; }
  .status.stopped { background: #5c1a1a; }
  
  .button-group { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-bottom: 20px; }
  .btn { padding: 15px; border: none; border-radius: 8px; font-size: 16px; font-weight: bold; cursor: pointer; transition: all 0.3s; }
  .btn-stop { background: #ff3333; color: white; grid-column: 1 / -1; font-size: 24px; padding: 30px; }
  .btn-stop:active { background: #cc0000; transform: scale(0.95); }
  .btn-start { background: #33a833; color: white; }
  .btn-start:active { background: #228022; }
  
  .control-group { background: #222; padding: 20px; border-radius: 8px; margin-bottom: 15px; }
  .control-group h3 { font-size: 14px; margin-bottom: 10px; text-transform: uppercase; color: #aaa; }
  
  label { display: block; margin-bottom: 5px; font-size: 13px; }
  input[type="range"] { width: 100%; height: 8px; cursor: pointer; accent-color: #ff6600; }
  input[type="number"] { width: 70px; padding: 5px; background: #333; color: #fff; border: 1px solid #444; border-radius: 4px; }
  
  .value { display: inline-block; margin-left: 10px; background: #333; padding: 3px 8px; border-radius: 4px; font-size: 13px; }
  .info { font-size: 12px; color: #999; margin-top: 20px; text-align: center; }
</style></head>
<body>
  <div class="container">
    <h1>🤖 ZEBI Robot</h1>
    
    <div class="status running" id="status">Statut: PRÊT</div>
    
    <div class="button-group">
      <button class="btn btn-stop" onclick="stopRobot()">🛑 ARRÊT D'URGENCE</button>
      <button class="btn btn-start" onclick="startRobot()">▶ DÉMARRER</button>
    </div>
    
    <div class="control-group">
      <h3>PID Contrôle</h3>
      <label>Kp (Proportionnel): <span class="value" id="kpVal">0.4</span></label>
      <input type="range" id="kp" min="0" max="2" step="0.1" value="0.4" oninput="updateKp()">
      
      <label style="margin-top: 15px;">Kd (Dérivée): <span class="value" id="kdVal">1.2</span></label>
      <input type="range" id="kd" min="0" max="3" step="0.1" value="1.2" oninput="updateKd()">
    </div>
    
    <div class="control-group">
      <h3>Vitesse</h3>
      <label>Vitesse de base: <span class="value" id="spVal">160</span></label>
      <input type="range" id="sp" min="50" max="255" step="5" value="160" oninput="updateSpeed()">
    </div>
    
    <div class="info">
      📡 Connecté à Robot-Z.E.B.I<br>
      IP: 192.168.4.1<br>
      Appuie longtemps sur ARRÊT EN CAS D'URGENCE
    </div>
  </div>

  <script>
    let isStopped = false;
    
    async function fetchAPI(endpoint) {
      try {
        await fetch('/' + endpoint);
      } catch(e) { console.log(e); }
    }
    
    function stopRobot() {
      document.getElementById('status').textContent = '🔴 ARRÊTÉ D\'URGENCE';
      document.getElementById('status').className = 'status stopped';
      isStopped = true;
      fetchAPI('stop');
    }
    
    function startRobot() {
      document.getElementById('status').textContent = '🟢 EN COURS D\'EXÉCUTION';
      document.getElementById('status').className = 'status running';
      isStopped = false;
      fetchAPI('start');
    }
    
    function updateKp() {
      const val = document.getElementById('kp').value;
      document.getElementById('kpVal').textContent = val;
      updateAll();
    }
    
    function updateKd() {
      const val = document.getElementById('kd').value;
      document.getElementById('kdVal').textContent = val;
      updateAll();
    }
    
    function updateSpeed() {
      const val = document.getElementById('sp').value;
      document.getElementById('spVal').textContent = val;
      updateAll();
    }
    
    function updateAll() {
      if (isStopped) {
        alert('⚠️ Démarre le robot d\'abord!');
        return;
      }
      const kp = document.getElementById('kp').value;
      const kd = document.getElementById('kd').value;
      const sp = document.getElementById('sp').value;
      fetchAPI('set?kp=' + kp + '&kd=' + kd + '&sp=' + sp);
    }
  </script>
</body></html>)rawliteral";

// --- TÂCHE DE PILOTAGE (CŒUR 1) ---
void TaskRobot(void * pvParameters) {
  unsigned long blinkTimer = 0;
  
  for(;;) {
    if (robotStop) {
      // Arrêt d'urgence : couper les moteurs
      ledcWrite(0, 0);  // Channel 0 = ENA
      ledcWrite(1, 0);  // Channel 1 = ENB
      digitalWrite(LED_RUNNING, LOW);
      vTaskDelay(100 / portTICK_PERIOD_MS);
      continue;
    }

    uint16_t position = qtr.readLineBlack(sensorValues);
    int error = position - 3500;

    int motorSpeed = (Kp * error) + (Kd * (error - lastError));
    lastError = error;

    int speedG = baseSpeed + motorSpeed;
    int speedD = baseSpeed - motorSpeed;

    // Constrain entre 0 et 65535 (16 bits)
    speedG = constrain(speedG, 0, 65535);
    speedD = constrain(speedD, 0, 65535);

    // Détection de blocage : position inchangée depuis 2 secondes
    unsigned long currentTime = millis();
    if (abs((int)position - (int)lastPosition) > 100) {
      // Robot bouge : réinitialiser le timer
      lastMovementTime = currentTime;
      ledBlinking = false;
      digitalWrite(LED_RUNNING, HIGH);  // LED stable
    } else if (currentTime - lastMovementTime > 2000) {
      // Bloqué depuis 2 sec : faire clignoter
      ledBlinking = true;
      if ((currentTime - lastMovementTime) % 200 < 100) {
        digitalWrite(LED_RUNNING, HIGH);
      } else {
        digitalWrite(LED_RUNNING, LOW);
      }
    }
    lastPosition = position;

    // Contrôle Moteurs selon direction
    if (robotForward) {
      // AVANCE
      digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    } else {
      // MARCHE ARRIÈRE
      digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
    }
    ledcWrite(0, speedG);  // Channel 0 = ENA
    ledcWrite(1, speedD);  // Channel 1 = ENB

    vTaskDelay(5 / portTICK_PERIOD_MS); // Boucle de 5ms pour la stabilité
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== ROBOT LINE FOLLOWER START ===");
  Serial.println("Initializing pins...");

  // Pins Moteurs
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(LED_INIT, OUTPUT); pinMode(LED_RUNNING, OUTPUT);
  digitalWrite(LED_INIT, HIGH);    // LED Init = ON
  digitalWrite(LED_RUNNING, LOW);  // LED Running = OFF
  
  // PWM Moteurs - 16 bits pour meilleure précision
  ledcSetup(0, 5000, 16);      // Channel 0: 5kHz, 16 bits
  ledcAttachPin(ENA, 0);       // Attach ENA to Channel 0
  ledcSetup(1, 5000, 16);      // Channel 1: 5kHz, 16 bits
  ledcAttachPin(ENB, 1);       // Attach ENB to Channel 1

  // Setup Capteurs
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){15,2,4,35,32,33,25,26}, SensorCount);
  Serial.println("QTR Sensors initialized");

  // Wi-Fi Access Point
  bool wifi_ok = WiFi.softAP("Robot-Z.E.B.I", "ZEBI");
  if(wifi_ok) {
    Serial.print("Wi-Fi AP started. IP: ");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("WARNING: Wi-Fi AP failed!");
  }
  
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });

  server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *request){
    robotStop = true;
    Serial.println("!!! EMERGENCY STOP !!!");
    request->send(200, "text/plain", "STOPPED");
  });

  server.on("/start", HTTP_GET, [](AsyncWebServerRequest *request){
    robotStop = false;
    lastError = 0;
    Serial.println("Robot restarted");
    request->send(200, "text/plain", "RUNNING");
  });

  server.on("/forward", HTTP_GET, [](AsyncWebServerRequest *request){
    robotForward = true;
    Serial.println("Direction: FORWARD");
    request->send(200, "text/plain", "FORWARD");
  });

  server.on("/backward", HTTP_GET, [](AsyncWebServerRequest *request){
    robotForward = false;
    Serial.println("Direction: BACKWARD");
    request->send(200, "text/plain", "BACKWARD");
  });

  server.begin();
  Serial.println("Web server started on /0.0.0.0:80");

  // Calibration (Le robot doit bouger sur la ligne)
  Serial.println("Calibrating sensors...");
  for (uint16_t i = 0; i < 200; i++) {
    qtr.calibrate();
    delay(10);
  }

  // Lancement du cœur de pilotage
  digitalWrite(LED_INIT, LOW);      // LED Init OFF
  digitalWrite(LED_RUNNING, HIGH);  // LED Running ON
  Serial.println("PID Control task started on Core 1");
  Serial.println("=== ROBOT READY ===");
  xTaskCreatePinnedToCore(TaskRobot, "TaskRobot", 4096, NULL, 1, NULL, 1);
}

void loop() {
  // Vide car tout est géré par les tâches et le serveur Async
}