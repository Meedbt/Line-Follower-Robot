#include <Arduino.h>
#include <QTRSensors.h>

QTRSensors qtr;

const uint8_t NOMBRE_CAPTEURS = 8;

// Ordre physique des capteurs QTR de gauche a droite.
// La position lue va de 0 a 7000, avec 3500 au centre.
const uint8_t BROCHES_CAPTEURS[NOMBRE_CAPTEURS] = {
  15, 2, 4, 5, 32, 33, 25, 18
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
const int VITESSE_BASE = 135;
const int VITESSE_MIN = 0;
const int VITESSE_MAX = 230;

// KP corrige l'ecart a la ligne, KD calme les oscillations.
// Si le robot reagit trop lentement: augmente KP. S'il zigzague: augmente KD ou baisse KP.
// Si le robot réagit trop lentement aux virages : augmente un peu KP.
// Si le robot zigzague/oscille : augmente un peu KD, ou baisse KP.
const float KP = 0.075;
const float KD = 0.45;

// Si ton robot corrige dans le mauvais sens, mets true.
const bool INVERSER_CORRECTION = false;

const bool INVERSER_MOTEUR_GAUCHE = false;
const bool INVERSER_MOTEUR_DROITE = false;

const uint16_t QTR_TIMEOUT_US = 2500;
const uint16_t SEUIL_LIGNE_PERDUE = 120;
const bool DEBUG = true;

uint16_t valeursCapteurs[NOMBRE_CAPTEURS];
int derniereErreur = 0;
int dernierePosition = 3500;

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

void setup()
{
  Serial.begin(115200);
  delay(1000);

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
  delay(500);
}

void loop()
{
  // Ligne noire sur fond clair. Si tu suis une ligne blanche, utilise readLineWhite().
  uint16_t position = qtr.readLineBlack(valeursCapteurs);
  dernierePosition = position;

  if (!ligneDetectee()) {
    // Recherche simple: tourner du cote ou la ligne a ete vue en dernier.
    int sensRecherche = dernierePosition < 3500 ? -1 : 1;
    reglerMoteurs(90 * sensRecherche, -90 * sensRecherche);
    return;
  }

  int erreur = (int)position - 3500;
  int derivee = erreur - derniereErreur;
  derniereErreur = erreur;

  // Meme logique que le code de reference: correction = KP * erreur + KD * variation.
  int correction = (int)(KP * erreur + KD * derivee);
  if (INVERSER_CORRECTION) {
    correction = -correction;
  }

  int vitesseGauche = VITESSE_BASE + correction;
  int vitesseDroite = VITESSE_BASE - correction;

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
