#include <Arduino.h>
#include <QTRSensors.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

QTRSensors qtr;
WebServer serveur(80);
Preferences preferences;

const uint8_t NOMBRE_CAPTEURS = 8;

// Ordre physique des capteurs QTR de gauche a droite.
// La position lue va de 0 a 7000, avec 3500 au centre.
const uint8_t BROCHES_CAPTEURS[NOMBRE_CAPTEURS] = {
  18, 25, 33, 32, 5, 4, 22, 23
  
};

// Broche LEDON/CTRL du QTR.
// Laisse -1 si cette broche n'est pas connectee.
const int8_t BROCHE_EMETTEUR = -1;

// Broches du L298N.
const uint8_t PWM_GAUCHE = 16;   // L298N ENA
const uint8_t GAUCHE_IN1 = 26;   // L298N IN1
const uint8_t GAUCHE_IN2 = 27;   // L298N IN2
const uint8_t PWM_DROITE = 19;   // L298N ENB
const uint8_t DROITE_IN1 = 14;   // L298N IN3
const uint8_t DROITE_IN2 = 12;   // L298N IN4

// Reglages moteur/PID: commence doucement, puis augmente petit a petit. 
// Si le robot est stable mais trop lent : augmente VITESSE_BASE petit à petit, par exemple 145, puis 155, etc.
int vitesseBase = 135;
const int VITESSE_MIN = 0;
const int VITESSE_MAX = 230;

// KP corrige l'ecart a la ligne, KD calme les oscillations.
// Si le robot reagit trop lentement: augmente KP. S'il zigzague: augmente KD ou baisse KP.
// Si le robot réagit trop lentement aux virages : augmente un peu KP.
// Si le robot zigzague/oscille : augmente un peu KD, ou baisse KP.
float kp = 0.075;
float kd = 0.45;

// Si ton robot corrige dans le mauvais sens, mets true.
bool inverserCorrection = false;

const bool INVERSER_MOTEUR_GAUCHE = false;
const bool INVERSER_MOTEUR_DROITE = false;

const uint16_t QTR_TIMEOUT_US = 2500;
const uint16_t SEUIL_LIGNE_PERDUE = 120;
const bool DEBUG = true;

uint16_t valeursCapteurs[NOMBRE_CAPTEURS];
int derniereErreur = 0;
int dernierePosition = 3500;
int derniereVitesseGauche = 0;
int derniereVitesseDroite = 0;
bool robotActif = false;
bool derniereLigneDetectee = false;

const char* NOM_WIFI = "Robot Z.E.B.I";
const char* MOT_DE_PASSE_WIFI = "12345678";

const char PAGE_WEB[] PROGMEM = R"HTML(
<!doctype html>
<html lang="fr">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>LineFollower ESP32</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #0f1115;
      --panel: #181b22;
      --panel-2: #20242d;
      --text: #f4f7fb;
      --muted: #9aa4b2;
      --line: #303642;
      --accent: #18c29c;
      --danger: #ff5d5d;
      --warn: #f2b84b;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      background: var(--bg);
      color: var(--text);
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }
    main {
      width: min(980px, 100%);
      margin: 0 auto;
      padding: 20px;
    }
    header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      gap: 16px;
      margin-bottom: 18px;
    }
    h1 {
      margin: 0;
      font-size: 26px;
      font-weight: 750;
      letter-spacing: 0;
    }
    .status {
      padding: 8px 12px;
      border: 1px solid var(--line);
      border-radius: 8px;
      color: var(--muted);
      background: var(--panel);
      white-space: nowrap;
    }
    .grid {
      display: grid;
      grid-template-columns: repeat(12, 1fr);
      gap: 12px;
    }
    .card {
      grid-column: span 6;
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 16px;
    }
    .card.wide { grid-column: span 12; }
    h2 {
      margin: 0 0 14px;
      font-size: 15px;
      color: var(--muted);
      font-weight: 650;
    }
    .big {
      font-size: 42px;
      font-weight: 800;
      line-height: 1;
    }
    .sub {
      margin-top: 6px;
      color: var(--muted);
      font-size: 14px;
    }
    .buttons {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 10px;
    }
    button {
      border: 0;
      border-radius: 8px;
      padding: 13px 14px;
      color: #07110f;
      background: var(--accent);
      font-weight: 750;
      font-size: 15px;
      cursor: pointer;
    }
    button.secondary {
      color: var(--text);
      background: var(--panel-2);
      border: 1px solid var(--line);
    }
    button.danger {
      color: #170606;
      background: var(--danger);
    }
    label {
      display: grid;
      gap: 8px;
      margin: 14px 0;
      color: var(--muted);
      font-size: 14px;
    }
    .row {
      display: flex;
      justify-content: space-between;
      align-items: center;
      gap: 12px;
    }
    output {
      color: var(--text);
      font-variant-numeric: tabular-nums;
    }
    input[type="range"] {
      width: 100%;
      accent-color: var(--accent);
    }
    .sensors {
      display: grid;
      grid-template-columns: repeat(8, 1fr);
      gap: 8px;
      align-items: end;
      height: 150px;
    }
    .sensor {
      min-width: 0;
      display: grid;
      align-content: end;
      gap: 6px;
      height: 100%;
    }
    .bar {
      min-height: 4px;
      border-radius: 6px 6px 2px 2px;
      background: var(--accent);
    }
    .sensor span {
      color: var(--muted);
      font-size: 11px;
      text-align: center;
      overflow-wrap: anywhere;
    }
    .metrics {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 10px;
    }
    .metric {
      background: var(--panel-2);
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 12px;
    }
    .metric strong {
      display: block;
      margin-top: 4px;
      font-size: 22px;
      font-variant-numeric: tabular-nums;
    }
    @media (max-width: 720px) {
      main { padding: 14px; }
      header { align-items: flex-start; flex-direction: column; }
      .card, .card.wide { grid-column: span 12; }
      .metrics { grid-template-columns: 1fr; }
      .buttons { grid-template-columns: 1fr; }
      .big { font-size: 36px; }
    }
  </style>
</head>
<body>
  <main>
    <header>
      <h1>LineFollower ESP32</h1>
      <div class="status" id="etat">Connexion...</div>
    </header>

    <section class="grid">
      <article class="card">
        <h2>Position ligne</h2>
        <div class="big" id="position">3500</div>
        <div class="sub">Centre ideal: 3500</div>
      </article>

      <article class="card">
        <h2>Commandes</h2>
        <div class="buttons">
          <button id="startStop" onclick="toggleRobot()">Start</button>
          <button class="secondary" onclick="calibrer()">Calibrer</button>
          <button class="secondary" onclick="sauver()">Sauvegarder</button>
          <button class="danger" onclick="stopRobot()">Stop</button>
        </div>
      </article>

      <article class="card wide">
        <h2>Reglages</h2>
        <label>
          <div class="row"><span>Vitesse base</span><output id="baseOut">135</output></div>
          <input id="base" type="range" min="60" max="230" step="1" value="135" oninput="majReglages()">
        </label>
        <label>
          <div class="row"><span>KP</span><output id="kpOut">0.075</output></div>
          <input id="kp" type="range" min="0" max="0.300" step="0.001" value="0.075" oninput="majReglages()">
        </label>
        <label>
          <div class="row"><span>KD</span><output id="kdOut">0.450</output></div>
          <input id="kd" type="range" min="0" max="1.500" step="0.005" value="0.450" oninput="majReglages()">
        </label>
        <label>
          <div class="row"><span>Correction inversee</span><output id="invOut">Non</output></div>
          <input id="inv" type="range" min="0" max="1" step="1" value="0" oninput="majReglages()">
        </label>
      </article>

      <article class="card wide">
        <h2>Capteurs QTR</h2>
        <div class="sensors" id="sensors"></div>
      </article>

      <article class="card wide">
        <h2>Telemetrie</h2>
        <div class="metrics">
          <div class="metric">Erreur<strong id="erreur">0</strong></div>
          <div class="metric">Moteur gauche<strong id="vg">0</strong></div>
          <div class="metric">Moteur droit<strong id="vd">0</strong></div>
        </div>
      </article>
    </section>
  </main>

  <script>
    const sensors = document.getElementById('sensors');
    let envoiTimer = null;

    for (let i = 0; i < 8; i++) {
      sensors.insertAdjacentHTML('beforeend', `<div class="sensor"><div class="bar" id="b${i}"></div><span id="s${i}">0</span></div>`);
    }

    function valeur(id) {
      return document.getElementById(id).value;
    }

    function majLabels() {
      baseOut.value = valeur('base');
      kpOut.value = Number(valeur('kp')).toFixed(3);
      kdOut.value = Number(valeur('kd')).toFixed(3);
      invOut.value = valeur('inv') === '1' ? 'Oui' : 'Non';
    }

    function majReglages() {
      majLabels();
      clearTimeout(envoiTimer);
      envoiTimer = setTimeout(() => {
        fetch(`/set?base=${valeur('base')}&kp=${valeur('kp')}&kd=${valeur('kd')}&inv=${valeur('inv')}`);
      }, 120);
    }

    function toggleRobot() {
      fetch('/toggle').then(lireEtat);
    }

    function stopRobot() {
      fetch('/stop').then(lireEtat);
    }

    function calibrer() {
      etat.textContent = 'Calibration...';
      fetch('/calibrate').then(lireEtat);
    }

    function sauver() {
      fetch('/save').then(() => {
        etat.textContent = 'Reglages sauvegardes';
      });
    }

    function appliquer(data) {
      position.textContent = data.position;
      erreur.textContent = data.error;
      vg.textContent = data.left;
      vd.textContent = data.right;
      etat.textContent = data.active ? 'Robot actif' : 'Robot stop';
      startStop.textContent = data.active ? 'Pause' : 'Start';
      startStop.className = data.active ? 'danger' : '';

      base.value = data.base;
      kp.value = data.kp;
      kd.value = data.kd;
      inv.value = data.inv ? 1 : 0;
      majLabels();

      data.sensors.forEach((v, i) => {
        document.getElementById(`s${i}`).textContent = v;
        document.getElementById(`b${i}`).style.height = `${Math.max(4, Math.min(100, v / 10))}%`;
      });
    }

    function lireEtat() {
      fetch('/state')
        .then(r => r.json())
        .then(appliquer)
        .catch(() => { etat.textContent = 'Connexion perdue'; });
    }

    majLabels();
    lireEtat();
    setInterval(lireEtat, 250);
  </script>
</body>
</html>
)HTML";

// Compatibilite PWM entre ESP32 Arduino core 3.x et les anciennes versions.
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void configurerPwm()
{
  ledcAttach(PWM_GAUCHE, 1000, 8);
  ledcAttach(PWM_DROITE, 1000, 8);
}

void ecrirePwm(uint8_t broche, uint8_t valeur)
{
  ledcWrite(broche, valeur);
}
#else
const uint8_t CANAL_PWM_GAUCHE = 0;
const uint8_t CANAL_PWM_DROITE = 1;

void configurerPwm()
{
  ledcSetup(CANAL_PWM_GAUCHE, 1000, 8);
  ledcSetup(CANAL_PWM_DROITE, 1000, 8);
  ledcAttachPin(PWM_GAUCHE, CANAL_PWM_GAUCHE);
  ledcAttachPin(PWM_DROITE, CANAL_PWM_DROITE);
}

void ecrirePwm(uint8_t broche, uint8_t valeur)
{
  ledcWrite(broche == PWM_GAUCHE ? CANAL_PWM_GAUCHE : CANAL_PWM_DROITE, valeur);
}
#endif

void reglerMoteur(uint8_t pwm, uint8_t in1, uint8_t in2, int vitesse, bool inverser)
{
  // Vitesse positive = marche avant, vitesse negative = marche arriere.
  vitesse = constrain(vitesse, -255, 255);

  bool avancer = vitesse >= 0;
  if (inverser) {
    avancer = !avancer;
  }

  digitalWrite(in1, avancer ? HIGH : LOW);
  digitalWrite(in2, avancer ? LOW : HIGH);
  ecrirePwm(pwm, abs(vitesse));
}

void reglerMoteurs(int vitesseGauche, int vitesseDroite)
{
  // Limite les commandes pour ne pas depasser la plage PWM choisie.
  vitesseGauche = constrain(vitesseGauche, -VITESSE_MAX, VITESSE_MAX);
  vitesseDroite = constrain(vitesseDroite, -VITESSE_MAX, VITESSE_MAX);

  if (abs(vitesseGauche) < VITESSE_MIN) {
    vitesseGauche = 0;
  }
  if (abs(vitesseDroite) < VITESSE_MIN) {
    vitesseDroite = 0;
  }

  reglerMoteur(PWM_GAUCHE, GAUCHE_IN1, GAUCHE_IN2, vitesseGauche, INVERSER_MOTEUR_GAUCHE);
  reglerMoteur(PWM_DROITE, DROITE_IN1, DROITE_IN2, vitesseDroite, INVERSER_MOTEUR_DROITE);
}

void arreterMoteurs()
{
  derniereVitesseGauche = 0;
  derniereVitesseDroite = 0;
  ecrirePwm(PWM_GAUCHE, 0);
  ecrirePwm(PWM_DROITE, 0);
  digitalWrite(GAUCHE_IN1, LOW);
  digitalWrite(GAUCHE_IN2, LOW);
  digitalWrite(DROITE_IN1, LOW);
  digitalWrite(DROITE_IN2, LOW);
}

bool ligneDetectee()
{
  // Apres readLineBlack(), une valeur haute veut dire que le capteur voit du noir.
  for (uint8_t i = 0; i < NOMBRE_CAPTEURS; i++) {
    if (valeursCapteurs[i] > SEUIL_LIGNE_PERDUE) {
      return true;
    }
  }
  return false;
}

void calibrerCapteurs()
{
  Serial.println("Calibration QTR: bouge le robot sur la ligne pendant 5 secondes...");

  // 250 * 20 ms = environ 5 secondes de calibration.
  for (uint16_t i = 0; i < 250; i++) {
    qtr.calibrate();
    delay(20);
  }

  Serial.println("Calibration terminee.");

  if (DEBUG) {
    Serial.print("Min: ");
    for (uint8_t i = 0; i < NOMBRE_CAPTEURS; i++) {
      Serial.print(qtr.calibrationOn.minimum[i]);
      Serial.print('\t');
    }
    Serial.println();

    Serial.print("Max: ");
    for (uint8_t i = 0; i < NOMBRE_CAPTEURS; i++) {
      Serial.print(qtr.calibrationOn.maximum[i]);
      Serial.print('\t');
    }
    Serial.println();
  }
}

void chargerReglages()
{
  preferences.begin("linebot", false);
  vitesseBase = preferences.getInt("base", vitesseBase);
  kp = preferences.getFloat("kp", kp);
  kd = preferences.getFloat("kd", kd);
  inverserCorrection = preferences.getBool("inv", inverserCorrection);
}

void sauvegarderReglages()
{
  preferences.putInt("base", vitesseBase);
  preferences.putFloat("kp", kp);
  preferences.putFloat("kd", kd);
  preferences.putBool("inv", inverserCorrection);
}

void envoyerEtat()
{
  String json = "{";
  json += "\"active\":";
  json += robotActif ? "true" : "false";
  json += ",\"position\":";
  json += dernierePosition;
  json += ",\"error\":";
  json += dernierePosition - 3500;
  json += ",\"left\":";
  json += derniereVitesseGauche;
  json += ",\"right\":";
  json += derniereVitesseDroite;
  json += ",\"base\":";
  json += vitesseBase;
  json += ",\"kp\":";
  json += String(kp, 3);
  json += ",\"kd\":";
  json += String(kd, 3);
  json += ",\"inv\":";
  json += inverserCorrection ? "true" : "false";
  json += ",\"line\":";
  json += derniereLigneDetectee ? "true" : "false";
  json += ",\"sensors\":[";

  for (uint8_t i = 0; i < NOMBRE_CAPTEURS; i++) {
    if (i > 0) {
      json += ',';
    }
    json += valeursCapteurs[i];
  }

  json += "]}";
  serveur.send(200, "application/json", json);
}

void appliquerReglagesDepuisWeb()
{
  if (serveur.hasArg("base")) {
    vitesseBase = constrain(serveur.arg("base").toInt(), 0, VITESSE_MAX);
  }
  if (serveur.hasArg("kp")) {
    kp = constrain(serveur.arg("kp").toFloat(), 0.0f, 1.0f);
  }
  if (serveur.hasArg("kd")) {
    kd = constrain(serveur.arg("kd").toFloat(), 0.0f, 3.0f);
  }
  if (serveur.hasArg("inv")) {
    inverserCorrection = serveur.arg("inv").toInt() == 1;
  }

  serveur.send(200, "text/plain", "OK");
}

void configurerInterfaceWeb()
{
  WiFi.mode(WIFI_AP);
  WiFi.softAP(NOM_WIFI, MOT_DE_PASSE_WIFI);

  serveur.on("/", []() {
    serveur.send_P(200, "text/html", PAGE_WEB);
  });

  serveur.on("/state", envoyerEtat);
  serveur.on("/set", appliquerReglagesDepuisWeb);

  serveur.on("/toggle", []() {
    robotActif = !robotActif;
    if (!robotActif) {
      arreterMoteurs();
    }
    envoyerEtat();
  });

  serveur.on("/stop", []() {
    robotActif = false;
    arreterMoteurs();
    envoyerEtat();
  });

  serveur.on("/save", []() {
    sauvegarderReglages();
    serveur.send(200, "text/plain", "OK");
  });

  serveur.on("/calibrate", []() {
    bool etaitActif = robotActif;
    robotActif = false;
    arreterMoteurs();
    calibrerCapteurs();
    robotActif = etaitActif;
    envoyerEtat();
  });

  serveur.begin();

  Serial.print("Interface web: connecte-toi au Wi-Fi ");
  Serial.print(NOM_WIFI);
  Serial.print(" puis ouvre http://");
  Serial.println(WiFi.softAPIP());
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  chargerReglages();

  pinMode(GAUCHE_IN1, OUTPUT);
  pinMode(GAUCHE_IN2, OUTPUT);
  pinMode(DROITE_IN1, OUTPUT);
  pinMode(DROITE_IN2, OUTPUT);

  configurerPwm();
  arreterMoteurs();

  // Configuration du QTR-8RC: type RC, pins utilisees, timeout de lecture.
  qtr.setTypeRC();
  qtr.setSensorPins(BROCHES_CAPTEURS, NOMBRE_CAPTEURS);
  qtr.setTimeout(QTR_TIMEOUT_US);

  if (BROCHE_EMETTEUR >= 0) {
    qtr.setEmitterPin(BROCHE_EMETTEUR);
  }

  calibrerCapteurs();
  configurerInterfaceWeb();
  delay(500);
}

void loop()
{
  serveur.handleClient();

  // Ligne noire sur fond clair. Si tu suis une ligne blanche, utilise readLineWhite().
  uint16_t position = qtr.readLineBlack(valeursCapteurs);
  dernierePosition = position;
  derniereLigneDetectee = ligneDetectee();

  if (!robotActif) {
    arreterMoteurs();
    return;
  }

  if (!derniereLigneDetectee) {
    // Recherche simple: tourner du cote ou la ligne a ete vue en dernier.
    int sensRecherche = dernierePosition < 3500 ? -1 : 1;
    derniereVitesseGauche = 90 * sensRecherche;
    derniereVitesseDroite = -90 * sensRecherche;
    reglerMoteurs(derniereVitesseGauche, derniereVitesseDroite);
    return;
  }

  int erreur = (int)position - 3500;
  int derivee = erreur - derniereErreur;
  derniereErreur = erreur;

  // Meme logique que le code de reference: correction = KP * erreur + KD * variation.
  int correction = (int)(kp * erreur + kd * derivee);
  if (inverserCorrection) {
    correction = -correction;
  }

  int vitesseGauche = vitesseBase + correction;
  int vitesseDroite = vitesseBase - correction;
  derniereVitesseGauche = constrain(vitesseGauche, -VITESSE_MAX, VITESSE_MAX);
  derniereVitesseDroite = constrain(vitesseDroite, -VITESSE_MAX, VITESSE_MAX);

  reglerMoteurs(vitesseGauche, vitesseDroite);

  if (DEBUG) {  
    static uint32_t dernierAffichage = 0;
    if (millis() - dernierAffichage > 100) {
      dernierAffichage = millis();
      Serial.print("pos=");
      Serial.print(position);
      Serial.print(" err=");
      Serial.print(erreur);
      Serial.print(" L=");
      Serial.print(vitesseGauche);
      Serial.print(" R=");
      Serial.println(vitesseDroite);
    }
  }
}
