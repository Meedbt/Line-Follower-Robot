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

// Capteur ultrason HC-SR04 pour detecter les barres d'arrivee.
// Important: ECHO sort souvent en 5V, mets un diviseur de tension vers l'ESP32.
const uint8_t ULTRASON_TRIG = 21;
const uint8_t ULTRASON_ECHO = 13;

// Mesure batterie NiMH 7.2V via pont diviseur:
// + batterie -> R1 100k -> GPIO35 -> R2 47k -> GND commun.
const uint8_t BATTERIE_ADC = 34;
const float BAT_R1 = 100000.0;
const float BAT_R2 = 47000.0;
const float BAT_TENSION_MAX = 8.4;
const float BAT_TENSION_MIN = 6.0;
const float BAT_CALIBRATION = 1.00;

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
bool modeArretMarqueur = false;
bool marqueurDejaCompte = false;
bool marqueurGaucheActif = false;
bool marqueurDroiteActif = false;
bool stopUltrasonActif = false;
bool barriereDetecteeUltrason = false;
bool modeIntersections = false;
bool intersectionDetectee = false;
bool intersectionDejaComptee = false;
bool actionIntersectionActive = false;
bool modeParcoursAvance = false;
bool phaseRetour = false;
bool demiTourActif = false;
bool pauseParkingActive = false;
bool pauseBDejaFaite = false;
bool pauseCDejaFaite = false;
int compteurMarqueurs = 0;
int cibleMarqueurs = 3;
int compteurIntersections = 0;
int demiTourApresMarqueur = 0;
int demiTourApresIntersection = 0;
int pauseBApresMarqueur = 0;
int pauseCApresMarqueur = 0;
int arretFinalApresMarqueur = 0;
int arretFinalApresIntersection = 0;
int distanceStopCm = 15;
int distanceUltrasonCm = 999;
float tensionBatterie = 0.0;
int pourcentageBatterie = 0;
bool modeVariationVitesse = false;
bool repereExtremeActif = false;
bool repereExtremeDejaCompte = false;
int compteurReperesVitesse = 0;
int etapeVitesse = 0;
int vitesseLente = 100;
int vitesseMoyenne = 135;
int vitesseRapide = 170;
String sequenceVitesse = "R,L,R,L,R";
char codeVitesseActuelle = 'M';
String sequenceIntersections = "S";
String sequenceAller = "S";
String sequenceRetour = "S";
char decisionIntersection = 'S';
uint32_t dernierMarqueurMs = 0;
uint32_t derniereMesureUltrasonMs = 0;
uint32_t derniereMesureBatterieMs = 0;
uint32_t debutActionIntersectionMs = 0;
uint32_t debutDemiTourMs = 0;
uint32_t debutPauseParkingMs = 0;
uint8_t confirmationsUltrason = 0;
uint8_t confirmationsBarriereTombee = 0;

const uint16_t SEUIL_MARQUEUR = 650;
const uint16_t SEUIL_REPERE_EXTREME = 650;
const uint16_t SEUIL_CENTRE_LIGNE = 180;
const uint8_t NB_CAPTEURS_NOIRS_INTERSECTION = 5;
const uint16_t DELAI_ANTI_DOUBLE_MS = 250;
const uint8_t CONFIRMATIONS_STOP_ULTRASON = 3;
const uint8_t CONFIRMATIONS_BARRIERE_TOMBEE = 5;
const uint16_t PERIODE_ULTRASON_MS = 60;
const uint16_t PERIODE_BATTERIE_MS = 500;
const uint16_t SEUIL_INTERSECTION = 650;
const uint8_t NB_CAPTEURS_NOIRS_INTERSECTION_ACTIVE = 5;
const uint8_t NB_CAPTEURS_NOIRS_SORTIE_INTERSECTION = 2;
const int VITESSE_ACTION_INTERSECTION = 115;
const uint16_t DUREE_TOUT_DROIT_INTERSECTION_MS = 180;
const uint16_t DUREE_MIN_ROTATION_INTERSECTION_MS = 180;
const uint16_t DUREE_MAX_ROTATION_INTERSECTION_MS = 950;
const int VITESSE_DEMI_TOUR = 120;
const uint16_t DUREE_MIN_DEMI_TOUR_MS = 700;
const uint16_t DUREE_MAX_DEMI_TOUR_MS = 2200;
const uint16_t DUREE_PAUSE_PARKING_MS = 5000;

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
    input[type="text"] {
      width: 100%;
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 11px 12px;
      color: var(--text);
      background: var(--panel-2);
      font: inherit;
      text-transform: uppercase;
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
          <div class="row"><span>Mode arret marqueur</span><output id="modeOut">Non</output></div>
          <input id="mode" type="range" min="0" max="1" step="1" value="0" oninput="majReglages()">
        </label>
        <label>
          <div class="row"><span>Marqueurs avant stop</span><output id="targetOut">3</output></div>
          <input id="target" type="range" min="1" max="30" step="1" value="3" oninput="majReglages()">
        </label>
        <label>
          <div class="row"><span>Stop ultrason</span><output id="usOut">Non</output></div>
          <input id="us" type="range" min="0" max="1" step="1" value="0" oninput="majReglages()">
        </label>
        <label>
          <div class="row"><span>Distance stop</span><output id="usDistOut">15 cm</output></div>
          <input id="usDist" type="range" min="5" max="40" step="1" value="15" oninput="majReglages()">
        </label>
        <label>
          <div class="row"><span>Mode intersections</span><output id="interOut">Non</output></div>
          <input id="inter" type="range" min="0" max="1" step="1" value="0" oninput="majReglages()">
        </label>
        <label>
          <div class="row"><span>Sequence intersections</span><output id="seqOut">S</output></div>
          <input id="seq" type="text" value="S" maxlength="100" oninput="majReglages()">
        </label>
        <label>
          <div class="row"><span>Parcours avance</span><output id="advOut">Non</output></div>
          <input id="adv" type="range" min="0" max="1" step="1" value="0" oninput="majReglages()">
        </label>
        <label>
          <div class="row"><span>Sequence aller</span><output id="seqGoOut">S</output></div>
          <input id="seqGo" type="text" value="S" maxlength="100" oninput="majReglages()">
        </label>
        <label>
          <div class="row"><span>Sequence retour</span><output id="seqBackOut">S</output></div>
          <input id="seqBack" type="text" value="S" maxlength="100" oninput="majReglages()">
        </label>
        <label>
          <div class="row"><span>Demi-tour apres marqueur</span><output id="uMarkOut">0</output></div>
          <input id="uMark" type="range" min="0" max="20" step="1" value="0" oninput="majReglages()">
        </label>
        <label>
          <div class="row"><span>Demi-tour apres intersection</span><output id="uInterOut">0</output></div>
          <input id="uInter" type="range" min="0" max="30" step="1" value="0" oninput="majReglages()">
        </label>
        <label>
          <div class="row"><span>Pause B apres marqueur</span><output id="parkBOut">0</output></div>
          <input id="parkB" type="range" min="0" max="20" step="1" value="0" oninput="majReglages()">
        </label>
        <label>
          <div class="row"><span>Pause C apres marqueur</span><output id="parkCOut">0</output></div>
          <input id="parkC" type="range" min="0" max="20" step="1" value="0" oninput="majReglages()">
        </label>
        <label>
          <div class="row"><span>Arret final marqueur</span><output id="finalMarkOut">0</output></div>
          <input id="finalMark" type="range" min="0" max="20" step="1" value="0" oninput="majReglages()">
        </label>
        <label>
          <div class="row"><span>Arret final intersection</span><output id="finalInterOut">0</output></div>
          <input id="finalInter" type="range" min="0" max="30" step="1" value="0" oninput="majReglages()">
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
          <div class="metric">Marqueurs<strong id="marks">0 / 3</strong></div>
          <div class="metric">Marqueur gauche<strong id="mg">Non</strong></div>
          <div class="metric">Marqueur droit<strong id="md">Non</strong></div>
          <div class="metric">Ultrason<strong id="usNow">-- cm</strong></div>
          <div class="metric">Stop ultrason<strong id="usState">Non</strong></div>
          <div class="metric">Batterie<strong id="batVolt">-- V</strong></div>
          <div class="metric">Charge<strong id="batPct">-- %</strong></div>
          <div class="metric">Intersections<strong id="interCount">0</strong></div>
          <div class="metric">Decision<strong id="interDecision">S</strong></div>
          <div class="metric">Phase<strong id="phaseNow">Aller</strong></div>
          <div class="metric">Action<strong id="actionNow">Suivi</strong></div>
        </div>
        <div style="margin-top:10px">
          <button class="secondary" onclick="resetCompteur()">Reset compteur</button>
          <button class="secondary" onclick="resetIntersections()">Reset intersections</button>
          <button class="secondary" onclick="resetParcours()">Reset parcours</button>
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
      modeOut.value = valeur('mode') === '1' ? 'Oui' : 'Non';
      targetOut.value = valeur('target');
      usOut.value = valeur('us') === '1' ? 'Oui' : 'Non';
      usDistOut.value = `${valeur('usDist')} cm`;
      interOut.value = valeur('inter') === '1' ? 'Oui' : 'Non';
      seqOut.value = valeur('seq').toUpperCase() || 'S';
      advOut.value = valeur('adv') === '1' ? 'Oui' : 'Non';
      seqGoOut.value = valeur('seqGo').toUpperCase() || 'S';
      seqBackOut.value = valeur('seqBack').toUpperCase() || 'S';
      uMarkOut.value = valeur('uMark');
      uInterOut.value = valeur('uInter');
      parkBOut.value = valeur('parkB');
      parkCOut.value = valeur('parkC');
      finalMarkOut.value = valeur('finalMark');
      finalInterOut.value = valeur('finalInter');
    }

    function majReglages() {
      majLabels();
      clearTimeout(envoiTimer);
      envoiTimer = setTimeout(() => {
        fetch(`/set?base=${valeur('base')}&kp=${valeur('kp')}&kd=${valeur('kd')}&mode=${valeur('mode')}&target=${valeur('target')}&us=${valeur('us')}&usDist=${valeur('usDist')}&inter=${valeur('inter')}&seq=${encodeURIComponent(valeur('seq'))}&adv=${valeur('adv')}&seqGo=${encodeURIComponent(valeur('seqGo'))}&seqBack=${encodeURIComponent(valeur('seqBack'))}&uMark=${valeur('uMark')}&uInter=${valeur('uInter')}&parkB=${valeur('parkB')}&parkC=${valeur('parkC')}&finalMark=${valeur('finalMark')}&finalInter=${valeur('finalInter')}`);
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

    function resetCompteur() {
      fetch('/resetCount').then(lireEtat);
    }

    function resetIntersections() {
      fetch('/resetIntersections').then(lireEtat);
    }

    function resetParcours() {
      fetch('/resetCourse').then(lireEtat);
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
      mode.value = data.stopMode ? 1 : 0;
      target.value = data.target;
      us.value = data.usStop ? 1 : 0;
      usDist.value = data.usLimit;
      inter.value = data.interMode ? 1 : 0;
      seq.value = data.interSeq;
      adv.value = data.advMode ? 1 : 0;
      seqGo.value = data.seqGo;
      seqBack.value = data.seqBack;
      uMark.value = data.uMark;
      uInter.value = data.uInter;
      parkB.value = data.parkB;
      parkC.value = data.parkC;
      finalMark.value = data.finalMark;
      finalInter.value = data.finalInter;
      majLabels();
      marks.textContent = `${data.count} / ${data.target}`;
      mg.textContent = data.markLeft ? 'Oui' : 'Non';
      md.textContent = data.markRight ? 'Oui' : 'Non';
      usNow.textContent = data.usDistance > 0 && data.usDistance < 999 ? `${data.usDistance} cm` : '-- cm';
      usState.textContent = data.usStop ? 'Oui' : 'Non';
      batVolt.textContent = `${Number(data.batteryV).toFixed(2)} V`;
      batPct.textContent = `${data.batteryPct} %`;
      interCount.textContent = data.interCount;
      interDecision.textContent = data.interDecision;
      phaseNow.textContent = data.returnPhase ? 'Retour' : 'Aller';
      actionNow.textContent = data.action;

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

const char PAGE_VITESSE[] PROGMEM = R"HTML(
<!doctype html>
<html lang="fr">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Robot Z.E.B.I - Challenge 21 & 22</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #10141a;
      --panel: #171c24;
      --panel2: #202633;
      --line: #313a48;
      --text: #f8fbff;
      --muted: #aab6c8;
      --fast: #20c66b;
      --medium: #f2b84b;
      --slow: #ff5258;
      --accent: #20c7aa;
      --danger: #ff5b61;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      background: var(--bg);
      color: var(--text);
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }
    main {
      width: min(900px, 100%);
      margin: 0 auto;
      padding: 22px 14px 34px;
    }
    header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      gap: 12px;
      margin-bottom: 28px;
    }
    h1 {
      margin: 0;
      font-size: clamp(28px, 6vw, 42px);
      line-height: 1.05;
    }
    .subtitle {
      color: var(--muted);
      margin-top: 8px;
      font-weight: 650;
    }
    .pill {
      border: 1px solid var(--line);
      background: var(--panel2);
      color: var(--muted);
      border-radius: 8px;
      padding: 10px 13px;
      white-space: nowrap;
    }
    section {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 16px;
      margin-bottom: 12px;
    }
    h2 {
      margin: 0 0 14px;
      color: var(--muted);
      font-size: 16px;
    }
    select, input, button {
      width: 100%;
      border-radius: 8px;
      font: inherit;
    }
    select, input {
      border: 1px solid var(--line);
      background: var(--panel2);
      color: var(--text);
      padding: 12px;
      margin-bottom: 12px;
    }
    input { text-transform: uppercase; }
    .buttons {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 10px;
    }
    button {
      border: 0;
      padding: 14px 12px;
      font-weight: 800;
      color: #06120f;
      background: var(--accent);
      cursor: pointer;
    }
    button.secondary {
      color: var(--text);
      border: 1px solid var(--line);
      background: var(--panel2);
    }
    button.danger {
      color: #180607;
      background: var(--danger);
    }
    .grid {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 9px 18px;
    }
    .row {
      display: grid;
      grid-template-columns: 1fr auto;
      gap: 10px;
      align-items: baseline;
    }
    .row span { color: var(--muted); }
    .row strong { font-size: 18px; font-variant-numeric: tabular-nums; }
    .fast { color: var(--fast); }
    .medium { color: var(--medium); }
    .slow { color: var(--slow); }
    @media (max-width: 680px) {
      header { align-items: flex-start; flex-direction: column; }
      .buttons, .grid { grid-template-columns: 1fr; }
    }
  </style>
</head>
<body>
  <main>
    <header>
      <div>
        <h1>Robot Z.E.B.I</h1>
        <div class="subtitle">Challenge 21 & 22 - Variation de vitesse</div>
      </div>
      <div class="pill" id="etat">Connexion...</div>
    </header>

    <section>
      <h2>Mode vitesse manuel</h2>
      <input id="speedSeq" value="R,L,R,L,R" maxlength="39" oninput="updateSequence()" placeholder="Sequence vitesse: R,L,M...">
      <div class="buttons">
        <button class="secondary" onclick="calibrer()">CALIBRER</button>
        <button onclick="startRobot()">START</button>
        <button class="danger" onclick="stopRobot()">STOP</button>
      </div>
    </section>

    <section>
      <h2>Telemetrie</h2>
      <div class="grid">
        <div class="row"><span>Position ligne</span><strong id="position">0</strong></div>
        <div class="row"><span>Vitesse actuelle</span><strong id="speed">--</strong></div>
        <div class="row"><span>Etape vitesse</span><strong id="step">0</strong></div>
        <div class="row"><span>Reperes extremes</span><strong id="marks">0</strong></div>
        <div class="row"><span>Repere detecte</span><strong id="markNow">Non</strong></div>
        <div class="row"><span>Ultrason</span><strong id="ultrason">-- cm</strong></div>
        <div class="row"><span>Batterie</span><strong id="battery">-- V</strong></div>
      </div>
    </section>
  </main>

  <script>
    function val(id) { return document.getElementById(id).value; }

    function updateSequence() {
      fetch(`/setSpeedSeq?seq=${encodeURIComponent(val('speedSeq'))}`);
    }

    function calibrer() {
      etat.textContent = 'Calibration...';
      fetch('/calibrate').then(readState);
    }

    function startRobot() {
      fetch('/startSpeedMode').then(readState);
    }

    function stopRobot() {
      fetch('/stop').then(readState);
    }

    function speedLabel(code) {
      if (code === 'R') return 'Rapide';
      if (code === 'L') return 'Lente';
      return 'Moyenne';
    }

    function speedClass(code) {
      if (code === 'R') return 'fast';
      if (code === 'L') return 'slow';
      return 'medium';
    }

    function applyState(data) {
      speedSeq.value = data.speedSeq;
      etat.textContent = data.active ? 'En course' : 'Pret';
      position.textContent = data.position;
      speed.textContent = `${speedLabel(data.speedCode)} (${data.base})`;
      speed.className = speedClass(data.speedCode);
      step.textContent = data.speedStep;
      marks.textContent = data.speedMarks;
      markNow.textContent = data.speedMarkNow ? 'Oui' : 'Non';
      ultrason.textContent = data.usDistance > 0 && data.usDistance < 999 ? `${data.usDistance} cm` : '-- cm';
      battery.textContent = `${Number(data.batteryV).toFixed(2)} V (${data.batteryPct}%)`;
    }

    function readState() {
      fetch('/state')
        .then(r => r.json())
        .then(applyState)
        .catch(() => { etat.textContent = 'Connexion perdue'; });
    }

    readState();
    setInterval(readState, 250);
  </script>
</body>
</html>
)HTML";

const char PAGE_MODE_VITESSE[] PROGMEM = R"HTML(
<!doctype html>
<html lang="fr">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Robot Z.E.B.I - Mode vitesse</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #10141a;
      --panel: #171c24;
      --panel2: #202633;
      --line: #313a48;
      --text: #f8fbff;
      --muted: #aab6c8;
      --fast: #20c66b;
      --medium: #f2b84b;
      --slow: #ff5258;
      --accent: #20c7aa;
      --danger: #ff5b61;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      background: var(--bg);
      color: var(--text);
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }
    main {
      width: min(920px, 100%);
      margin: 0 auto;
      padding: 22px 14px 34px;
    }
    header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      gap: 12px;
      margin-bottom: 28px;
    }
    h1 {
      margin: 0;
      font-size: clamp(28px, 6vw, 42px);
      line-height: 1.05;
    }
    .subtitle {
      color: var(--muted);
      margin-top: 8px;
      font-weight: 650;
    }
    .pill {
      border: 1px solid var(--line);
      background: var(--panel2);
      color: var(--muted);
      border-radius: 8px;
      padding: 10px 13px;
      white-space: nowrap;
    }
    section {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 16px;
      margin-bottom: 12px;
    }
    h2 {
      margin: 0 0 14px;
      color: var(--muted);
      font-size: 16px;
    }
    label {
      display: grid;
      gap: 8px;
      margin-bottom: 12px;
      color: var(--muted);
      font-size: 14px;
    }
    input, button {
      width: 100%;
      border-radius: 8px;
      font: inherit;
    }
    input {
      border: 1px solid var(--line);
      background: var(--panel2);
      color: var(--text);
      padding: 12px;
    }
    input[type="text"] { text-transform: uppercase; }
    input[type="range"] { accent-color: var(--accent); }
    .buttons {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 10px;
      margin-top: 12px;
    }
    button {
      border: 0;
      padding: 14px 12px;
      font-weight: 800;
      color: #06120f;
      background: var(--accent);
      cursor: pointer;
    }
    button.secondary {
      color: var(--text);
      border: 1px solid var(--line);
      background: var(--panel2);
    }
    button.danger {
      color: #180607;
      background: var(--danger);
    }
    .grid {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 9px 18px;
    }
    .row {
      display: grid;
      grid-template-columns: 1fr auto;
      gap: 10px;
      align-items: baseline;
    }
    .row span { color: var(--muted); }
    .row strong { font-size: 18px; font-variant-numeric: tabular-nums; }
    .fast { color: var(--fast); }
    .medium { color: var(--medium); }
    .slow { color: var(--slow); }
    @media (max-width: 680px) {
      header { align-items: flex-start; flex-direction: column; }
      .buttons, .grid { grid-template-columns: 1fr; }
    }
  </style>
</head>
<body>
  <main>
    <header>
      <div>
        <h1>Robot Z.E.B.I</h1>
        <div class="subtitle">Mode vitesse manuel - Missions 21 & 22</div>
      </div>
      <div class="pill" id="etat">Connexion...</div>
    </header>

    <section>
      <h2>Reglages vitesse</h2>
      <label>
        <div class="row"><span>Mode vitesse</span><strong id="modeOut">Non</strong></div>
        <input id="speedMode" type="range" min="0" max="1" step="1" value="0" oninput="sendConfig()">
      </label>
      <label>
        <span>Sequence vitesse (R = rapide, M = moyenne, L = lente)</span>
        <input id="speedSeq" type="text" value="R,L,R,L,R" maxlength="39" oninput="sendConfig()">
      </label>
      <div class="grid">
        <label>
          <div class="row"><span>Vitesse lente</span><strong id="slowOut">100</strong></div>
          <input id="slow" type="range" min="60" max="180" step="1" value="100" oninput="sendConfig()">
        </label>
        <label>
          <div class="row"><span>Vitesse moyenne</span><strong id="midOut">135</strong></div>
          <input id="mid" type="range" min="80" max="210" step="1" value="135" oninput="sendConfig()">
        </label>
        <label>
          <div class="row"><span>Vitesse rapide</span><strong id="fastOut">170</strong></div>
          <input id="fast" type="range" min="100" max="230" step="1" value="170" oninput="sendConfig()">
        </label>
        <label>
          <div class="row"><span>Arrivee ultrason</span><strong id="usOut">Non</strong></div>
          <input id="us" type="range" min="0" max="1" step="1" value="0" oninput="sendConfig()">
        </label>
        <label>
          <div class="row"><span>Distance detection barriere</span><strong id="usDistOut">15 cm</strong></div>
          <input id="usDist" type="range" min="5" max="40" step="1" value="15" oninput="sendConfig()">
        </label>
      </div>
      <div class="buttons">
        <button class="secondary" onclick="calibrer()">CALIBRER</button>
        <button onclick="startRobot()">START</button>
        <button class="danger" onclick="stopRobot()">STOP</button>
      </div>
    </section>

    <section>
      <h2>Telemetrie</h2>
      <div class="grid">
        <div class="row"><span>Position ligne</span><strong id="position">0</strong></div>
        <div class="row"><span>Vitesse actuelle</span><strong id="speed">--</strong></div>
        <div class="row"><span>Etape vitesse</span><strong id="step">0</strong></div>
        <div class="row"><span>Reperes extremes</span><strong id="marks">0</strong></div>
        <div class="row"><span>Repere detecte</span><strong id="markNow">Non</strong></div>
        <div class="row"><span>Ultrason</span><strong id="ultrason">-- cm</strong></div>
        <div class="row"><span>Barriere</span><strong id="barrier">Non</strong></div>
        <div class="row"><span>Batterie</span><strong id="battery">-- V</strong></div>
      </div>
    </section>
  </main>

  <script>
    let sendTimer = null;
    function val(id) { return document.getElementById(id).value; }

    function label(code) {
      if (code === 'R') return 'Rapide';
      if (code === 'L') return 'Lente';
      return 'Moyenne';
    }

    function cls(code) {
      if (code === 'R') return 'fast';
      if (code === 'L') return 'slow';
      return 'medium';
    }

    function updateLabels() {
      modeOut.textContent = val('speedMode') === '1' ? 'Oui' : 'Non';
      slowOut.textContent = val('slow');
      midOut.textContent = val('mid');
      fastOut.textContent = val('fast');
      usOut.textContent = val('us') === '1' ? 'Oui' : 'Non';
      usDistOut.textContent = `${val('usDist')} cm`;
    }

    function sendConfig() {
      updateLabels();
      clearTimeout(sendTimer);
      sendTimer = setTimeout(() => {
        fetch(`/setSpeedConfig?mode=${val('speedMode')}&seq=${encodeURIComponent(val('speedSeq'))}&slow=${val('slow')}&mid=${val('mid')}&fast=${val('fast')}&us=${val('us')}&usDist=${val('usDist')}`);
      }, 120);
    }

    function calibrer() {
      etat.textContent = 'Calibration...';
      fetch('/calibrate').then(readState);
    }

    function startRobot() {
      fetch('/startSpeedMode').then(readState);
    }

    function stopRobot() {
      fetch('/stop').then(readState);
    }

    function applyState(data) {
      speedMode.value = data.speedMode ? 1 : 0;
      speedSeq.value = data.speedSeq;
      slow.value = data.speedSlow;
      mid.value = data.speedMedium;
      fast.value = data.speedFast;
      us.value = data.usStop ? 1 : 0;
      usDist.value = data.usLimit;
      updateLabels();
      etat.textContent = data.active ? 'En course' : 'Pret';
      position.textContent = data.position;
      speed.textContent = `${label(data.speedCode)} (${data.base})`;
      speed.className = cls(data.speedCode);
      step.textContent = data.speedStep;
      marks.textContent = data.speedMarks;
      markNow.textContent = data.speedMarkNow ? 'Oui' : 'Non';
      ultrason.textContent = data.usDistance > 0 && data.usDistance < 999 ? `${data.usDistance} cm` : '-- cm';
      barrier.textContent = data.usStop ? (data.barrierSeen ? (data.barrierLostCount >= 5 ? 'Tombee' : 'Vue') : 'Attente') : 'Non';
      battery.textContent = `${Number(data.batteryV).toFixed(2)} V (${data.batteryPct}%)`;
    }

    function readState() {
      fetch('/state')
        .then(r => r.json())
        .then(applyState)
        .catch(() => { etat.textContent = 'Connexion perdue'; });
    }

    updateLabels();
    readState();
    setInterval(readState, 250);
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

uint8_t compterCapteursNoirs(uint16_t seuil)
{
  uint8_t nbCapteursNoirs = 0;
  for (uint8_t i = 0; i < NOMBRE_CAPTEURS; i++) {
    if (valeursCapteurs[i] > seuil) {
      nbCapteursNoirs++;
    }
  }
  return nbCapteursNoirs;
}

String nettoyerSequenceIntersections(String sequence)
{
  sequence.toUpperCase();
  String nettoyee = "";
  bool dernierSeparateur = true;

  for (uint16_t i = 0; i < sequence.length() && nettoyee.length() < 31; i++) {
    char c = sequence.charAt(i);
    if (c == 'S' || c == 'G' || c == 'D') {
      if (!dernierSeparateur) {
        nettoyee += ',';
      }
      nettoyee += c;
      dernierSeparateur = false;
    }
  }

  if (nettoyee.length() == 0) {
    nettoyee = "S";
  }

  return nettoyee;
}

String nettoyerSequenceVitesse(String sequence)
{
  sequence.toUpperCase();
  String nettoyee = "";
  bool dernierSeparateur = true;

  for (uint16_t i = 0; i < sequence.length() && nettoyee.length() < 39; i++) {
    char c = sequence.charAt(i);
    if (c == 'R' || c == 'M' || c == 'L') {
      if (!dernierSeparateur) {
        nettoyee += ',';
      }
      nettoyee += c;
      dernierSeparateur = false;
    }
  }

  if (nettoyee.length() == 0) {
    nettoyee = "M";
  }

  return nettoyee;
}

char vitessePourEtape(int etape)
{
  int index = 0;
  char derniereVitesse = 'M';

  for (uint16_t i = 0; i < sequenceVitesse.length(); i++) {
    char c = sequenceVitesse.charAt(i);
    if (c == 'R' || c == 'M' || c == 'L') {
      derniereVitesse = c;
      if (index == etape) {
        return c;
      }
      index++;
    }
  }

  return derniereVitesse;
}

void appliquerCodeVitesse(char code)
{
  codeVitesseActuelle = code;
  if (code == 'R') {
    vitesseBase = vitesseRapide;
  } else if (code == 'L') {
    vitesseBase = vitesseLente;
  } else {
    vitesseBase = vitesseMoyenne;
  }
}

void resetVariationVitesse()
{
  compteurReperesVitesse = 0;
  etapeVitesse = 0;
  repereExtremeActif = false;
  repereExtremeDejaCompte = false;
  if (modeVariationVitesse) {
    appliquerCodeVitesse(vitessePourEtape(0));
  } else {
    appliquerCodeVitesse('M');
  }
}

char decisionPourIntersection(int numeroIntersection)
{
  int index = 1;
  char derniereDecision = 'S';
  String sequenceActive = sequenceIntersections;

  if (modeParcoursAvance) {
    sequenceActive = phaseRetour ? sequenceRetour : sequenceAller;
  }

  for (uint16_t i = 0; i < sequenceActive.length(); i++) {
    char c = sequenceActive.charAt(i);
    if (c == 'S' || c == 'G' || c == 'D') {
      derniereDecision = c;
      if (index == numeroIntersection) {
        return c;
      }
      index++;
    }
  }

  return derniereDecision;
}

void resetParcours()
{
  phaseRetour = false;
  demiTourActif = false;
  pauseParkingActive = false;
  pauseBDejaFaite = false;
  pauseCDejaFaite = false;
  actionIntersectionActive = false;
  intersectionDejaComptee = false;
  marqueurDejaCompte = false;
  compteurMarqueurs = 0;
  compteurIntersections = 0;
  derniereErreur = 0;
  decisionIntersection = decisionPourIntersection(1);
}

void passerEnPhaseRetour()
{
  phaseRetour = true;
  compteurMarqueurs = 0;
  compteurIntersections = 0;
  marqueurDejaCompte = false;
  intersectionDejaComptee = false;
  actionIntersectionActive = false;
  derniereErreur = 0;
  decisionIntersection = decisionPourIntersection(1);
}

bool ligneCentree()
{
  return valeursCapteurs[3] > SEUIL_CENTRE_LIGNE ||
         valeursCapteurs[4] > SEUIL_CENTRE_LIGNE;
}

void detecterIntersections()
{
  uint8_t nbCapteursNoirs = compterCapteursNoirs(SEUIL_INTERSECTION);
  intersectionDetectee = nbCapteursNoirs >= NB_CAPTEURS_NOIRS_INTERSECTION_ACTIVE;

  if ((!modeIntersections && !modeParcoursAvance) || actionIntersectionActive) {
    return;
  }

  if (intersectionDetectee && !intersectionDejaComptee) {
    compteurIntersections++;
    decisionIntersection = decisionPourIntersection(compteurIntersections);
    actionIntersectionActive = true;
    debutActionIntersectionMs = millis();
    intersectionDejaComptee = true;
  }

  if (!intersectionDetectee && nbCapteursNoirs <= NB_CAPTEURS_NOIRS_SORTIE_INTERSECTION) {
    intersectionDejaComptee = false;
  }
}

bool executerActionIntersection()
{
  if (!actionIntersectionActive) {
    return false;
  }

  uint32_t tempsAction = millis() - debutActionIntersectionMs;

  if (decisionIntersection == 'S') {
    derniereVitesseGauche = vitesseBase;
    derniereVitesseDroite = vitesseBase;
    reglerMoteurs(vitesseBase, vitesseBase);
    if (tempsAction >= DUREE_TOUT_DROIT_INTERSECTION_MS) {
      actionIntersectionActive = false;
    }
    return true;
  }

  if (decisionIntersection == 'G') {
    derniereVitesseGauche = -VITESSE_ACTION_INTERSECTION;
    derniereVitesseDroite = VITESSE_ACTION_INTERSECTION;
    reglerMoteurs(-VITESSE_ACTION_INTERSECTION, VITESSE_ACTION_INTERSECTION);
  } else {
    derniereVitesseGauche = VITESSE_ACTION_INTERSECTION;
    derniereVitesseDroite = -VITESSE_ACTION_INTERSECTION;
    reglerMoteurs(VITESSE_ACTION_INTERSECTION, -VITESSE_ACTION_INTERSECTION);
  }

  if ((tempsAction >= DUREE_MIN_ROTATION_INTERSECTION_MS && ligneCentree()) ||
      tempsAction >= DUREE_MAX_ROTATION_INTERSECTION_MS) {
    actionIntersectionActive = false;
    derniereErreur = 0;
  }

  return true;
}

bool executerDemiTour()
{
  if (!demiTourActif) {
    return false;
  }

  uint32_t tempsAction = millis() - debutDemiTourMs;
  derniereVitesseGauche = VITESSE_DEMI_TOUR;
  derniereVitesseDroite = -VITESSE_DEMI_TOUR;
  reglerMoteurs(VITESSE_DEMI_TOUR, -VITESSE_DEMI_TOUR);

  if ((tempsAction >= DUREE_MIN_DEMI_TOUR_MS && ligneCentree()) ||
      tempsAction >= DUREE_MAX_DEMI_TOUR_MS) {
    demiTourActif = false;
    passerEnPhaseRetour();
  }

  return true;
}

bool executerPauseParking()
{
  if (!pauseParkingActive) {
    return false;
  }

  arreterMoteurs();
  if (millis() - debutPauseParkingMs >= DUREE_PAUSE_PARKING_MS) {
    pauseParkingActive = false;
  }

  return true;
}



void gererParcoursAvance()
{
  if (!modeParcoursAvance || !robotActif || demiTourActif || pauseParkingActive) {
    return;
  }

  if (!phaseRetour) {
    if (pauseBApresMarqueur > 0 && !pauseBDejaFaite &&
        compteurMarqueurs >= pauseBApresMarqueur) {
      pauseBDejaFaite = true;
      pauseParkingActive = true;
      debutPauseParkingMs = millis();
      return;
    }

    if (pauseCApresMarqueur > 0 && !pauseCDejaFaite &&
        compteurMarqueurs >= pauseCApresMarqueur) {
      pauseCDejaFaite = true;
      pauseParkingActive = true;
      debutPauseParkingMs = millis();
      return;
    }

    if ((demiTourApresMarqueur > 0 && compteurMarqueurs >= demiTourApresMarqueur) ||
        (demiTourApresIntersection > 0 && compteurIntersections >= demiTourApresIntersection)) {
      demiTourActif = true;
      debutDemiTourMs = millis();
      return;
    }
  } else {
    if ((arretFinalApresMarqueur > 0 && compteurMarqueurs >= arretFinalApresMarqueur) ||
        (arretFinalApresIntersection > 0 && compteurIntersections >= arretFinalApresIntersection)) {
      robotActif = false;
      arreterMoteurs();
      return;
    }
  }
}

void detecterMarqueurs()
{
  uint8_t nbCapteursNoirs = compterCapteursNoirs(SEUIL_MARQUEUR);

  bool intersectionDetectee = nbCapteursNoirs >= NB_CAPTEURS_NOIRS_INTERSECTION;

  // On demande aussi une lecture au centre pour eviter de compter un virage serre
  // comme un marqueur lateral.
  bool centreSurLigne = valeursCapteurs[2] > SEUIL_CENTRE_LIGNE ||
                        valeursCapteurs[3] > SEUIL_CENTRE_LIGNE ||
                        valeursCapteurs[4] > SEUIL_CENTRE_LIGNE ||
                        valeursCapteurs[5] > SEUIL_CENTRE_LIGNE;

  marqueurGaucheActif = !intersectionDetectee &&
                        centreSurLigne &&
                        valeursCapteurs[0] > SEUIL_MARQUEUR &&
                        valeursCapteurs[1] > SEUIL_MARQUEUR;

  marqueurDroiteActif = !intersectionDetectee &&
                       centreSurLigne &&
                       valeursCapteurs[6] > SEUIL_MARQUEUR &&
                       valeursCapteurs[7] > SEUIL_MARQUEUR;

  bool marqueurActif = marqueurGaucheActif || marqueurDroiteActif;
  uint32_t maintenant = millis();

  if ((modeArretMarqueur || modeParcoursAvance) && robotActif &&
      marqueurActif && !marqueurDejaCompte &&
      maintenant - dernierMarqueurMs > DELAI_ANTI_DOUBLE_MS) {
    compteurMarqueurs++;
    marqueurDejaCompte = true;
    dernierMarqueurMs = maintenant;
  }

  if (!marqueurActif) {
    marqueurDejaCompte = false;
  }
}

void detecterReperesVitesse()
{
  repereExtremeActif = valeursCapteurs[0] > SEUIL_REPERE_EXTREME ||
                       valeursCapteurs[7] > SEUIL_REPERE_EXTREME;

  if (!modeVariationVitesse || !robotActif) {
    if (!repereExtremeActif) {
      repereExtremeDejaCompte = false;
    }
    return;
  }

  if (repereExtremeActif && !repereExtremeDejaCompte) {
    compteurReperesVitesse++;
    etapeVitesse++;
    appliquerCodeVitesse(vitessePourEtape(etapeVitesse));
    repereExtremeDejaCompte = true;
  }

  if (!repereExtremeActif) {
    repereExtremeDejaCompte = false;
  }
}

int mesurerDistanceUltrasonCm()
{
  digitalWrite(ULTRASON_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASON_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASON_TRIG, LOW);

  unsigned long duree = pulseIn(ULTRASON_ECHO, HIGH, 25000);
  if (duree == 0) {
    return 999;
  }

  return (int)(duree / 58);
}

void resetBarriereUltrason()
{
  barriereDetecteeUltrason = false;
  confirmationsUltrason = 0;
  confirmationsBarriereTombee = 0;
}

void mettreAJourUltrason()
{
  if (millis() - derniereMesureUltrasonMs < PERIODE_ULTRASON_MS) {
    return;
  }

  derniereMesureUltrasonMs = millis();
  distanceUltrasonCm = mesurerDistanceUltrasonCm();

  bool obstacleProche = distanceUltrasonCm > 0 &&
                        distanceUltrasonCm <= distanceStopCm;

  if (!stopUltrasonActif) {
    resetBarriereUltrason();
    return;
  }
  if (!robotActif) {
    return;
  }

  if (!barriereDetecteeUltrason && obstacleProche) {
    if (confirmationsUltrason < CONFIRMATIONS_STOP_ULTRASON) {
      confirmationsUltrason++;
    }
    if (confirmationsUltrason >= CONFIRMATIONS_STOP_ULTRASON) {
      barriereDetecteeUltrason = true;
      confirmationsBarriereTombee = 0;
    }
    return;
  }

  if (!barriereDetecteeUltrason) {
    confirmationsUltrason = 0;
    return;
  }

  if (obstacleProche) {
    confirmationsBarriereTombee = 0;
  } else {
    if (confirmationsBarriereTombee < CONFIRMATIONS_BARRIERE_TOMBEE) {
      confirmationsBarriereTombee++;
    }
  }
}

void mettreAJourBatterie()
{
  if (millis() - derniereMesureBatterieMs < PERIODE_BATTERIE_MS) {
    return;
  }

  derniereMesureBatterieMs = millis();

  uint32_t sommeMv = 0;
  const uint8_t nbLectures = 12;
  for (uint8_t i = 0; i < nbLectures; i++) {
    sommeMv += analogReadMilliVolts(BATTERIE_ADC);
    delayMicroseconds(300);
  }

  float tensionAdc = (sommeMv / (float)nbLectures) / 1000.0;
  tensionBatterie = tensionAdc * ((BAT_R1 + BAT_R2) / BAT_R2) * BAT_CALIBRATION;

  float pct = (tensionBatterie - BAT_TENSION_MIN) * 100.0 /
              (BAT_TENSION_MAX - BAT_TENSION_MIN);
  pourcentageBatterie = constrain((int)pct, 0, 100);
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
  modeArretMarqueur = preferences.getBool("mode", modeArretMarqueur);
  cibleMarqueurs = preferences.getInt("target", cibleMarqueurs);
  stopUltrasonActif = preferences.getBool("us", stopUltrasonActif);
  distanceStopCm = preferences.getInt("usDist", distanceStopCm);
  modeIntersections = preferences.getBool("inter", modeIntersections);
  sequenceIntersections = nettoyerSequenceIntersections(preferences.getString("seq", sequenceIntersections));
  modeParcoursAvance = preferences.getBool("adv", modeParcoursAvance);
  sequenceAller = nettoyerSequenceIntersections(preferences.getString("seqGo", sequenceAller));
  sequenceRetour = nettoyerSequenceIntersections(preferences.getString("seqBack", sequenceRetour));
  demiTourApresMarqueur = preferences.getInt("uMark", demiTourApresMarqueur);
  demiTourApresIntersection = preferences.getInt("uInter", demiTourApresIntersection);
  pauseBApresMarqueur = preferences.getInt("parkB", pauseBApresMarqueur);
  pauseCApresMarqueur = preferences.getInt("parkC", pauseCApresMarqueur);
  arretFinalApresMarqueur = preferences.getInt("finalM", arretFinalApresMarqueur);
  arretFinalApresIntersection = preferences.getInt("finalI", arretFinalApresIntersection);
  modeVariationVitesse = preferences.getBool("speedMode", modeVariationVitesse);
  sequenceVitesse = nettoyerSequenceVitesse(preferences.getString("speedSeq", sequenceVitesse));
  vitesseLente = preferences.getInt("speedSlow", vitesseLente);
  vitesseMoyenne = preferences.getInt("speedMid", vitesseMoyenne);
  vitesseRapide = preferences.getInt("speedFast", vitesseRapide);
  resetVariationVitesse();
}

void sauvegarderReglages()
{
  preferences.putInt("base", vitesseBase);
  preferences.putFloat("kp", kp);
  preferences.putFloat("kd", kd);
  preferences.putBool("mode", modeArretMarqueur);
  preferences.putInt("target", cibleMarqueurs);
  preferences.putBool("us", stopUltrasonActif);
  preferences.putInt("usDist", distanceStopCm);
  preferences.putBool("inter", modeIntersections);
  preferences.putString("seq", sequenceIntersections);
  preferences.putBool("adv", modeParcoursAvance);
  preferences.putString("seqGo", sequenceAller);
  preferences.putString("seqBack", sequenceRetour);
  preferences.putInt("uMark", demiTourApresMarqueur);
  preferences.putInt("uInter", demiTourApresIntersection);
  preferences.putInt("parkB", pauseBApresMarqueur);
  preferences.putInt("parkC", pauseCApresMarqueur);
  preferences.putInt("finalM", arretFinalApresMarqueur);
  preferences.putInt("finalI", arretFinalApresIntersection);
  preferences.putBool("speedMode", modeVariationVitesse);
  preferences.putString("speedSeq", sequenceVitesse);
  preferences.putInt("speedSlow", vitesseLente);
  preferences.putInt("speedMid", vitesseMoyenne);
  preferences.putInt("speedFast", vitesseRapide);
}

void envoyerEtat()
{
  String json = "{";
  json += "\"active\":";
  json += robotActif ? "true" : "false";
  json += ",\"speedSeq\":\"";
  json += sequenceVitesse;
  json += "\"";
  json += ",\"speedCode\":\"";
  json += codeVitesseActuelle;
  json += "\"";
  json += ",\"speedStep\":";
  json += etapeVitesse;
  json += ",\"speedMarks\":";
  json += compteurReperesVitesse;
  json += ",\"speedMarkNow\":";
  json += repereExtremeActif ? "true" : "false";
  json += ",\"speedMode\":";
  json += modeVariationVitesse ? "true" : "false";
  json += ",\"speedSlow\":";
  json += vitesseLente;
  json += ",\"speedMedium\":";
  json += vitesseMoyenne;
  json += ",\"speedFast\":";
  json += vitesseRapide;
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
  json += ",\"line\":";
  json += derniereLigneDetectee ? "true" : "false";
  json += ",\"stopMode\":";
  json += modeArretMarqueur ? "true" : "false";
  json += ",\"count\":";
  json += compteurMarqueurs;
  json += ",\"target\":";
  json += cibleMarqueurs;
  json += ",\"markLeft\":";
  json += marqueurGaucheActif ? "true" : "false";
  json += ",\"markRight\":";
  json += marqueurDroiteActif ? "true" : "false";
  json += ",\"usStop\":";
  json += stopUltrasonActif ? "true" : "false";
  json += ",\"usLimit\":";
  json += distanceStopCm;
  json += ",\"usDistance\":";
  json += distanceUltrasonCm;
  json += ",\"barrierSeen\":";
  json += barriereDetecteeUltrason ? "true" : "false";
  json += ",\"barrierLostCount\":";
  json += confirmationsBarriereTombee;
  json += ",\"batteryV\":";
  json += String(tensionBatterie, 2);
  json += ",\"batteryPct\":";
  json += pourcentageBatterie;
  json += ",\"interMode\":";
  json += modeIntersections ? "true" : "false";
  json += ",\"interCount\":";
  json += compteurIntersections;
  json += ",\"interDecision\":\"";
  json += decisionIntersection;
  json += "\"";
  json += ",\"interSeq\":\"";
  json += sequenceIntersections;
  json += "\"";
  json += ",\"advMode\":";
  json += modeParcoursAvance ? "true" : "false";
  json += ",\"returnPhase\":";
  json += phaseRetour ? "true" : "false";
  json += ",\"seqGo\":\"";
  json += sequenceAller;
  json += "\"";
  json += ",\"seqBack\":\"";
  json += sequenceRetour;
  json += "\"";
  json += ",\"uMark\":";
  json += demiTourApresMarqueur;
  json += ",\"uInter\":";
  json += demiTourApresIntersection;
  json += ",\"parkB\":";
  json += pauseBApresMarqueur;
  json += ",\"parkC\":";
  json += pauseCApresMarqueur;
  json += ",\"finalMark\":";
  json += arretFinalApresMarqueur;
  json += ",\"finalInter\":";
  json += arretFinalApresIntersection;
  json += ",\"action\":\"";
  if (demiTourActif) {
    json += "Demi-tour";
  } else if (pauseParkingActive) {
    json += "Pause";
  } else if (actionIntersectionActive) {
    json += "Intersection";
  } else {
    json += "Suivi";
  }
  json += "\"";
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
  if (serveur.hasArg("mode")) {
    modeArretMarqueur = serveur.arg("mode").toInt() == 1;
  }
  if (serveur.hasArg("target")) {
    cibleMarqueurs = constrain(serveur.arg("target").toInt(), 1, 30);
  }
  if (serveur.hasArg("us")) {
    stopUltrasonActif = serveur.arg("us").toInt() == 1;
    if (!stopUltrasonActif) {
      resetBarriereUltrason();
    }
  }
  if (serveur.hasArg("usDist")) {
    distanceStopCm = constrain(serveur.arg("usDist").toInt(), 5, 40);
    resetBarriereUltrason();
  }
  if (serveur.hasArg("inter")) {
    modeIntersections = serveur.arg("inter").toInt() == 1;
  }
  if (serveur.hasArg("seq")) {
    sequenceIntersections = nettoyerSequenceIntersections(serveur.arg("seq"));
  }
  if (serveur.hasArg("adv")) {
    modeParcoursAvance = serveur.arg("adv").toInt() == 1;
  }
  if (serveur.hasArg("seqGo")) {
    sequenceAller = nettoyerSequenceIntersections(serveur.arg("seqGo"));
  }
  if (serveur.hasArg("seqBack")) {
    sequenceRetour = nettoyerSequenceIntersections(serveur.arg("seqBack"));
  }
  if (serveur.hasArg("uMark")) {
    demiTourApresMarqueur = constrain(serveur.arg("uMark").toInt(), 0, 20);
  }
  if (serveur.hasArg("uInter")) {
    demiTourApresIntersection = constrain(serveur.arg("uInter").toInt(), 0, 30);
  }
  if (serveur.hasArg("parkB")) {
    pauseBApresMarqueur = constrain(serveur.arg("parkB").toInt(), 0, 20);
  }
  if (serveur.hasArg("parkC")) {
    pauseCApresMarqueur = constrain(serveur.arg("parkC").toInt(), 0, 20);
  }
  if (serveur.hasArg("finalMark")) {
    arretFinalApresMarqueur = constrain(serveur.arg("finalMark").toInt(), 0, 20);
  }
  if (serveur.hasArg("finalInter")) {
    arretFinalApresIntersection = constrain(serveur.arg("finalInter").toInt(), 0, 30);
  }

  serveur.send(200, "text/plain", "OK");
}

void configurerInterfaceWeb()
{
  WiFi.mode(WIFI_AP);
  WiFi.softAP(NOM_WIFI, MOT_DE_PASSE_WIFI);

  serveur.on("/", []() {
    serveur.send_P(200, "text/html", PAGE_MODE_VITESSE);
  });

  serveur.on("/state", envoyerEtat);
  serveur.on("/set", appliquerReglagesDepuisWeb);

  serveur.on("/setSpeedConfig", []() {
    if (serveur.hasArg("mode")) {
      modeVariationVitesse = serveur.arg("mode").toInt() == 1;
    }
    if (serveur.hasArg("seq")) {
      sequenceVitesse = nettoyerSequenceVitesse(serveur.arg("seq"));
    }
    if (serveur.hasArg("slow")) {
      vitesseLente = constrain(serveur.arg("slow").toInt(), 0, VITESSE_MAX);
    }
    if (serveur.hasArg("mid")) {
      vitesseMoyenne = constrain(serveur.arg("mid").toInt(), 0, VITESSE_MAX);
    }
    if (serveur.hasArg("fast")) {
      vitesseRapide = constrain(serveur.arg("fast").toInt(), 0, VITESSE_MAX);
    }
    if (serveur.hasArg("us")) {
      stopUltrasonActif = serveur.arg("us").toInt() == 1;
      if (!stopUltrasonActif) {
        resetBarriereUltrason();
      }
    }
    if (serveur.hasArg("usDist")) {
      distanceStopCm = constrain(serveur.arg("usDist").toInt(), 5, 40);
      resetBarriereUltrason();
    }
    resetVariationVitesse();
    sauvegarderReglages();
    serveur.send(200, "text/plain", "OK");
  });

  serveur.on("/startSpeedMode", []() {
    resetVariationVitesse();
    resetBarriereUltrason();
    robotActif = true;
    envoyerEtat();
  });

  serveur.on("/toggle", []() {
    bool etaitActif = robotActif;
    robotActif = !robotActif;
    if (!etaitActif && robotActif && modeVariationVitesse) {
      resetVariationVitesse();
    }
    if (!etaitActif && robotActif && modeArretMarqueur) {
      compteurMarqueurs = 0;
      marqueurDejaCompte = false;
    }
    if (!etaitActif && robotActif && modeIntersections) {
      compteurIntersections = 0;
      intersectionDejaComptee = false;
      actionIntersectionActive = false;
      decisionIntersection = decisionPourIntersection(1);
    }
    if (!etaitActif && robotActif && modeParcoursAvance) {
      resetParcours();
    }
    if (!etaitActif && robotActif) {
      resetBarriereUltrason();
    }
    if (!robotActif) {
      resetBarriereUltrason();
      arreterMoteurs();
    }
    envoyerEtat();
  });

  serveur.on("/stop", []() {
    robotActif = false;
    resetBarriereUltrason();
    arreterMoteurs();
    envoyerEtat();
  });

  serveur.on("/save", []() {
    sauvegarderReglages();
    serveur.send(200, "text/plain", "OK");
  });

  serveur.on("/resetCount", []() {
    compteurMarqueurs = 0;
    marqueurDejaCompte = false;
    envoyerEtat();
  });

  serveur.on("/resetIntersections", []() {
    compteurIntersections = 0;
    intersectionDejaComptee = false;
    actionIntersectionActive = false;
    decisionIntersection = decisionPourIntersection(1);
    envoyerEtat();
  });

  serveur.on("/resetCourse", []() {
    resetParcours();
    envoyerEtat();
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
  pinMode(ULTRASON_TRIG, OUTPUT);
  pinMode(ULTRASON_ECHO, INPUT);
  digitalWrite(ULTRASON_TRIG, LOW);
  analogReadResolution(12);
  analogSetPinAttenuation(BATTERIE_ADC, ADC_11db);

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
  mettreAJourUltrason();
  mettreAJourBatterie();

  // Ligne noire sur fond clair. Si tu suis une ligne blanche, utilise readLineWhite().
  uint16_t position = qtr.readLineBlack(valeursCapteurs);
  dernierePosition = position;
  derniereLigneDetectee = ligneDetectee();
  detecterReperesVitesse();

  if (!robotActif) {
    arreterMoteurs();
    return;
  }

  if (modeArretMarqueur && !modeParcoursAvance && compteurMarqueurs >= cibleMarqueurs) {
    robotActif = false;
    arreterMoteurs();
    return;
  }

  if (stopUltrasonActif && barriereDetecteeUltrason &&
      confirmationsBarriereTombee >= CONFIRMATIONS_BARRIERE_TOMBEE) {
    robotActif = false;
    arreterMoteurs();
    return;
  }

  gererParcoursAvance();

  if (!robotActif) {
    return;
  }
  if (executerPauseParking()) {
    return;
  }

  if (executerDemiTour()) {
    return;
  }

  if (executerActionIntersection()) {
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
