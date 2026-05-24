# Conversation Complete - Robot Suiveur De Ligne ESP32

Ce document resume toute la conversation et l'evolution du projet de robot suiveur de ligne: materiel, logique de suivi, interface, marqueurs, intersections, ultrason, batterie, challenges, parcours avance et nouvelle interface competition.

## 1. Materiel Et Objectif

Le projet principal utilise:

- ESP32
- QTR-8RC
- L298N
- deux moteurs DC
- capteur ultrason HC-SR04
- batterie NiMH 7.2V
- interface web embarquee

Objectif:

- suivre une ligne noire sur fond clair
- executer des challenges de robotique
- gerer les marqueurs et intersections
- detecter l'arrivee/barriere
- automatiser les missions via une interface simple

## 2. Code De Reference Initial

Le code de reference fourni au debut utilisait:

- `AFMotor.h`
- `QTRSensors.h`
- 5 capteurs
- un controleur PD

Principe du code original:

```cpp
position = qtrrc.readLine(sensors);
error = position - 2000;

motorSpeed = KP * error + KD * (error - lastError);
lastError = error;

leftMotorSpeed = baseSpeed + motorSpeed;
rightMotorSpeed = baseSpeed - motorSpeed;
```

Il ne pouvait pas etre repris directement car il etait fait pour un shield moteur Adafruit, pas pour un ESP32 avec L298N.

## 3. Pins Actuelles

Pins QTR-8RC, dans l'ordre physique gauche vers droite:

```cpp
const uint8_t BROCHES_CAPTEURS[NOMBRE_CAPTEURS] = {
  18, 25, 33, 32, 5, 4, 22, 23
};
```

Pins L298N:

```cpp
const uint8_t PWM_GAUCHE = 16;
const uint8_t GAUCHE_IN1 = 26;
const uint8_t GAUCHE_IN2 = 27;
const uint8_t PWM_DROITE = 19;
const uint8_t DROITE_IN1 = 14;
const uint8_t DROITE_IN2 = 12;
```

Pins ultrason:

```cpp
const uint8_t ULTRASON_TRIG = 21;
const uint8_t ULTRASON_ECHO = 13;
```

Pin batterie:

```cpp
const uint8_t BATTERIE_ADC = 35;
```

Notes importantes:

- GPIO34 et GPIO35 ne doivent pas etre utilises pour le QTR-8RC car ils sont entree uniquement.
- GPIO35 est par contre tres bien pour mesurer la batterie.
- Si le HC-SR04 est alimente en 5V, il faut un diviseur de tension sur `ECHO`.

## 4. Suivi De Ligne

La version ESP32 utilise:

```cpp
qtr.readLineBlack(valeursCapteurs);
```

Donc:

```text
0    = blanc
1000 = noir
```

Position:

```text
0    = ligne tout a gauche
3500 = ligne au centre
7000 = ligne tout a droite
```

Controle PD:

```cpp
erreur = position - 3500;
derivee = erreur - derniereErreur;
correction = kp * erreur + kd * derivee;

vitesseGauche = vitesseBase + correction;
vitesseDroite = vitesseBase - correction;
```

Reglages principaux:

```cpp
int vitesseBase = 135;
float kp = 0.075;
float kd = 0.45;
```

Reglage:

```text
Robot trop lent en virage  -> augmenter KP
Robot zigzague             -> augmenter KD ou baisser KP
Robot stable mais lent     -> augmenter vitesseBase
```

## 5. Ancienne Interface Web De Reglage

Une premiere interface web complete avait ete creee.

Elle permettait de regler:

- vitesse base
- KP
- KD
- mode arret marqueur
- nombre de marqueurs avant stop
- stop ultrason
- distance stop ultrason
- mode intersections
- sequence intersections
- parcours avance
- sequence aller
- sequence retour
- demi-tour apres marqueur/intersection
- pauses parking B/C
- arret final marqueur/intersection

Elle affichait:

- position
- erreur
- vitesses moteurs
- capteurs QTR
- marqueurs
- intersections
- ultrason
- batterie
- phase
- action

L'option `Correction inversee` a ete supprimee car elle ne servait pas.

## 6. Nouvelle Interface Competition

Une nouvelle interface simplifiee a ete creee pour le jour du challenge.

Elle remplace l'affichage de l'ancienne interface:

```cpp
serveur.send_P(200, "text/html", PAGE_COMPETITION);
```

L'ancienne interface `PAGE_WEB` est encore dans le code comme sauvegarde interne, mais elle n'est plus affichee.

Nouvelle interface:

- selection du challenge
- bouton `CALIBRER`
- bouton `START CHALLENGE`
- bouton `URGENCE STOP`
- challenge actif
- position ligne
- ultrason barriere
- batterie
- priorite droite
- marqueurs
- intersections
- phase
- etape/action actuelle

Wi-Fi:

```text
Nom : Robot Z.E.B.I
Mot de passe : 12345678
Adresse : http://192.168.4.1
```

## 7. Profils De Challenges

Les challenges sont maintenant pre-codes dans:

```cpp
const ChallengeConfig CHALLENGES[] = { ... };
```

Chaque challenge contient:

- nom
- arret marqueur oui/non
- cible marqueur
- stop ultrason oui/non
- distance stop
- mode intersections oui/non
- sequence simple
- parcours avance oui/non
- sequence aller
- sequence retour
- demi-tour apres marqueur
- demi-tour apres intersection
- pause B apres marqueur
- pause C apres marqueur
- arret final apres marqueur
- arret final apres intersection

Quand un challenge est selectionne:

```cpp
appliquerChallenge(numeroChallenge);
```

Le robot charge automatiquement la configuration correspondante.

## 8. Marqueurs Lateraux

Les petits tires noirs sur les cotes sont traites comme des marqueurs.

Capteurs utilises:

```text
valeursCapteurs[0] et [1] = cote gauche
valeursCapteurs[6] et [7] = cote droit
```

Un marqueur est compte si:

- le centre voit encore la ligne
- un capteur extreme voit du noir
- ce n'est pas une intersection
- le robot est actif
- le marqueur n'a pas deja ete compte

Le code evite de compter plusieurs fois le meme marqueur avec:

```cpp
marqueurDejaCompte
DELAI_ANTI_DOUBLE_MS
```

## 9. Difference Entre Marqueur Et Intersection

La difference se fait avec le nombre de capteurs noirs.

Marqueur:

```text
centre noir + extreme noir + peu de capteurs noirs
```

Intersection:

```text
beaucoup de capteurs noirs en meme temps
```

Seuil:

```cpp
const uint8_t NB_CAPTEURS_NOIRS_INTERSECTION = 5;
```

Donc:

```text
moins de 5 capteurs noirs -> possible marqueur
5 capteurs noirs ou plus  -> intersection, pas marqueur
```

## 10. Mode Arret Marqueur

Ce mode sert aux challenges ou le robot doit s'arreter a un endroit demande.

Principe:

```text
Mode arret marqueur = Oui
Marqueurs avant stop = N
Quand compteurMarqueurs >= N -> stop
```

En mode parcours avance, cet arret simple est ignore pour ne pas bloquer les aller-retour:

```cpp
if (modeArretMarqueur && !modeParcoursAvance && compteurMarqueurs >= cibleMarqueurs)
```

## 11. Ultrason Et Barriere

Le capteur ultrason detecte les barres d'arrivee.

Pins:

```cpp
const uint8_t ULTRASON_TRIG = 21;
const uint8_t ULTRASON_ECHO = 13;
```

Principe:

```text
Stop ultrason = Oui
Distance <= distanceStop
pendant 3 mesures d'affilee
=> stop
```

Pour faire tomber une barriere:

```text
Stop ultrason = Non
```

Pour eviter la barriere:

```text
Stop ultrason = Oui
Distance stop = environ 10 a 20 cm
```

## 12. Batterie NiMH 7.2V

La batterie est une NiMH 7.2V.

Tensions:

```text
pleine  : 8.4V
nominale: 7.2V
faible  : 6.0V
```

Montage pont diviseur:

```text
+ batterie
   |
  R1 = 100k
   |
   +---- GPIO35 ESP32
   |
  R2 = 47k
   |
GND batterie / GND ESP32
```

Constantes:

```cpp
const float BAT_R1 = 100000.0;
const float BAT_R2 = 47000.0;
const float BAT_TENSION_MAX = 8.4;
const float BAT_TENSION_MIN = 6.0;
const float BAT_CALIBRATION = 1.00;
```

Si l'affichage ne correspond pas au multimetre, ajuster:

```cpp
BAT_CALIBRATION
```

## 13. Gestion Des Intersections

Le mode intersections permet d'utiliser une sequence de decisions.

Lettres:

```text
S = tout droit
G = gauche
D = droite
```

Exemples valides:

```text
SSG
S,S,G
S D G
```

Le code nettoie automatiquement la sequence et ne garde que `S`, `G`, `D`.

Exemple:

```text
Sequence = SSG
```

Signifie:

```text
intersection 1 -> tout droit
intersection 2 -> tout droit
intersection 3 -> gauche
```

Si le robot voit plus d'intersections que la sequence, il repete la derniere decision.

Exemple:

```text
S,D
```

Donne:

```text
intersection 1 -> S
intersection 2 -> D
intersection 3 -> D
intersection 4 -> D
```

## 14. Parcours Avance

Le mode parcours avance sert pour les missions avec:

- aller
- demi-tour
- retour
- pauses parking
- arret final

Reglages:

```text
Parcours avance
Sequence aller
Sequence retour
Demi-tour apres marqueur
Demi-tour apres intersection
Pause B apres marqueur
Pause C apres marqueur
Arret final marqueur
Arret final intersection
```

Valeur `0`:

```text
desactive la fonction
```

## 15. Sequence Aller / Retour

Si:

```text
Parcours avance = Oui
Sequence aller = SSG
Sequence retour = vide
```

Alors la sequence retour est automatiquement nettoyee et devient:

```text
S
```

Mais le robot ne passe en phase retour que si un demi-tour est defini:

```text
Demi-tour apres marqueur > 0
ou
Demi-tour apres intersection > 0
```

Sans demi-tour, le robot reste en phase `Aller`.

## 16. Demi-Tour

Deux facons de declencher le demi-tour:

```text
Demi-tour apres marqueur
Demi-tour apres intersection
```

Exemple:

```text
Sequence aller = SSG
Demi-tour apres intersection = 3
Demi-tour apres marqueur = 0
```

Le robot:

```text
intersection 1 -> S
intersection 2 -> S
intersection 3 -> G
puis demi-tour
passe en phase Retour
```

Utiliser:

```text
Demi-tour apres intersection
```

si le point de demi-tour est lie a un croisement.

Utiliser:

```text
Demi-tour apres marqueur
```

si le point de demi-tour est lie a un petit tire lateral.

## 17. Arret Final

Les options:

```text
Arret final marqueur
Arret final intersection
```

servent seulement en phase `Retour`.

Arret final marqueur:

```text
en phase Retour, arrete-toi apres N marqueurs
```

Arret final intersection:

```text
en phase Retour, arrete-toi apres N intersections
```

Exemple:

```text
Arret final marqueur = 3
```

Le robot:

```text
Aller
Demi-tour
Retour
compte 3 marqueurs
Stop
```

Si les deux valent `0`, le robot ne s'arrete pas automatiquement apres le retour.

## 18. Pauses Parking

Pour le challenge 20, des pauses parking ont ete prevues:

```text
Pause B apres marqueur
Pause C apres marqueur
```

Quand le compteur de marqueurs atteint la valeur choisie:

```text
robot stop 5 secondes
puis reprend
```

Duree:

```cpp
const uint16_t DUREE_PAUSE_PARKING_MS = 5000;
```

## 19. Challenges Et Capacites

Le PDF des challenges a ete analyse.

### Challenges simples

Faisables avec le suivi simple:

```text
Challenge 1
Challenge 2
```

Reglage:

```text
Stop ultrason = Non
```

pour faire tomber la barriere.

### Stop a endroit demande

Faisables avec les marqueurs:

```text
Challenge 3
Challenge 4
```

Reglage:

```text
Mode arret marqueur = Oui
Marqueurs avant stop = N
```

### Pistes marquees

Partiellement gerees avec sequences d'intersections:

```text
Challenges 8, 9, 10, 11, 16
```

Il faut ajuster les sequences sur piste.

### Aller-retour / demi-tour

Des fonctions ont ete ajoutees pour:

```text
Challenges 5, 6, 7, 12, 13, 14, 15, 17, 18, 19
```

Mais les valeurs exactes doivent etre testees sur la piste.

### Challenge 20

Le code contient maintenant les pauses B/C:

```text
Pause B apres marqueur
Pause C apres marqueur
```

Les numeros de marqueurs doivent etre calibres sur piste.

## 20. Priorite A Droite

Le reglement mentionne:

```text
priorite a droite
```

Le robot actuel n'a pas encore de vraie detection laterale droite.

Avec l'ultrason frontal, il peut detecter un obstacle devant, mais pas vraiment appliquer la priorite a droite.

Decision:

```text
laisser cette partie pour plus tard
```

## 21. Idee Alternative Arduino Nano

Une idee alternative a ete discutee:

- Arduino Nano
- QTR-8RC
- hacheur moteur
- boutons
- ecran OLED
- pas d'interface web

Avantages:

- plus simple terrain
- pas de Wi-Fi
- plus rapide au demarrage
- plus robuste

Inconvenients:

- moins confortable pour regler
- moins de memoire
- interface OLED plus limitee

Cette piste a ete mise de cote pour revenir au programme ESP32.

## 22. Etat Actuel Du Fichier

Fichier actuel:

```text
C:\Users\mouha\Downloads\linefollowerV2.ino
```

Fonctions presentes:

- suivi ligne QTR
- controle moteur L298N
- interface competition
- profils de 20 challenges
- calibration
- start challenge
- urgence stop
- marqueurs
- intersections
- sequences S/G/D
- parcours avance
- phase aller/retour
- demi-tour
- pauses parking
- arret final
- ultrason barriere
- batterie

## 23. Points A Tester En Priorite

1. Compiler dans l'IDE Arduino.
2. Verifier que l'interface competition s'affiche.
3. Tester challenge 1 avec:

```text
Stop ultrason = Non dans le profil
```

4. Tester calibration QTR.
5. Tester position ligne autour de 3500.
6. Tester marqueurs.
7. Tester intersections avec une sequence simple.
8. Tester demi-tour.
9. Tester ultrason.
10. Ajuster les profils de challenges.

## 24. Compilation

Impossible de compiler depuis Codex car:

```text
arduino-cli n'est pas installe
```

La verification finale doit etre faite dans l'IDE Arduino.

Si une erreur apparait, envoyer le message exact pour correction.

## 25. Notes Importantes

Les profils `CHALLENGES[]` sont une base.

Pour les missions complexes:

- les sequences `S/G/D`
- les numeros de marqueurs
- les numeros d'intersections
- les pauses
- les arrets finaux

doivent etre ajustes sur la vraie piste.

Le code est maintenant oriente competition:

```text
selectionner challenge
calibrer
start
urgence stop si besoin
```
