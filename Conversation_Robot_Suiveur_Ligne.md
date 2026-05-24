# Conversation - Projet Robot Suiveur de Ligne ESP32

Ce document reprend toute la conversation de travail autour du robot suiveur de ligne: choix techniques, logique du code, interface web, capteurs, batterie, challenges et etat actuel du fichier `linefollowerV2.ino`.

## 1. Contexte De Depart

Le projet concerne un robot suiveur de ligne base sur:

- ESP32
- QTR-8RC
- L298N
- deux moteurs DC
- interface web embarquee
- capteur ultrason pour l'arrivee
- mesure batterie NiMH 7.2V

Le code de reference initial venait d'un autre robot utilisant:

- `AFMotor.h`
- `QTRSensors.h`
- 5 capteurs
- un controleur de type PD:

```cpp
motorSpeed = KP * error + KD * (error - lastError);
```

Ce code n'etait pas directement compatible avec l'ESP32 + L298N, donc une version adaptee a ete creee.

## 2. Pins Et Materiel

Pins QTR actuelles, dans l'ordre physique gauche vers droite:

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

Les pins `34` et `35` avaient ete identifiees comme incompatibles avec le QTR-8RC, car elles sont entree uniquement sur ESP32. Elles ont donc ete remplacees.

## 3. Logique De Suivi De Ligne

Le robot utilise une logique inspiree du code de reference:

```cpp
position = qtr.readLineBlack(valeursCapteurs);
erreur = position - 3500;
derivee = erreur - derniereErreur;
correction = kp * erreur + kd * derivee;

vitesseGauche = vitesseBase + correction;
vitesseDroite = vitesseBase - correction;
```

Avec 8 capteurs, la position va de:

```text
0    = ligne completement a gauche
3500 = ligne centree
7000 = ligne completement a droite
```

Les valeurs capteurs calibrees vont environ de:

```text
0    = blanc
1000 = noir
```

Le code actuel suit une ligne noire sur fond clair avec:

```cpp
qtr.readLineBlack(valeursCapteurs);
```

## 4. Interface Web ESP32

Une interface web a ete ajoutee. L'ESP32 cree son propre reseau Wi-Fi:

```text
Wi-Fi : Robot Z.E.B.I
Mot de passe : 12345678
Adresse : http://192.168.4.1
```

L'interface permet de voir ou modifier:

- Start / Stop
- calibration QTR
- vitesse de base
- `KP`
- `KD`
- mode arret marqueur
- nombre de marqueurs avant stop
- stop ultrason
- distance de stop ultrason
- mode intersections
- sequence intersections
- valeurs des 8 capteurs
- position de ligne
- erreur
- vitesses gauche/droite
- compteur marqueurs
- marqueur gauche/droite
- distance ultrason
- tension batterie
- pourcentage batterie
- compteur intersections
- decision intersection actuelle

L'option `Correction inversee` a ete supprimee car elle n'etait pas utilisee.

## 5. Marqueurs Lateraux

Les petits tires noirs sur la piste ont ete interpretes comme des marqueurs lateraux.

Le robot les detecte avec les capteurs extremes:

```text
valeursCapteurs[0] et [1] = cote gauche
valeursCapteurs[6] et [7] = cote droit
```

Un marqueur est compte seulement si:

- le centre voit encore la ligne
- un cote extreme voit beaucoup de noir
- le mode arret marqueur est actif
- le robot est en marche
- le meme marqueur n'a pas deja ete compte

Un filtre a ete ajoute pour ne pas confondre marqueur et intersection:

```cpp
const uint8_t NB_CAPTEURS_NOIRS_INTERSECTION = 5;
```

Si 5 capteurs ou plus voient du noir, le code considere plutot que c'est une intersection ou une grosse zone noire, pas un petit tire lateral.

## 6. Mode Arret Marqueur

Le mode arret marqueur sert surtout pour un challenge ou le robot doit s'arreter a un endroit demande.

Principe:

```text
Mode arret marqueur = Oui
Marqueurs avant stop = N
le robot compte les tires
quand compteur >= N, il s'arrete
```

Le compteur se remet a zero au demarrage d'un essai en mode arret marqueur.

## 7. Capteur Ultrason Pour L'Arrivee

Un capteur ultrason a ete ajoute pour detecter les barres d'arrivee.

Pins par defaut:

```cpp
const uint8_t ULTRASON_TRIG = 21;
const uint8_t ULTRASON_ECHO = 13;
```

Important:

```text
Si le HC-SR04 est alimente en 5V, ECHO sort souvent du 5V.
L'ESP32 accepte 3.3V maximum.
Il faut donc un diviseur de tension sur ECHO.
```

Logique:

```text
Stop ultrason = Oui
Distance mesuree <= Distance stop
pendant 3 mesures d'affilee
=> arret du robot
```

Pour faire tomber une barriere, il faut mettre:

```text
Stop ultrason = Non
```

Pour ne pas faire tomber une barriere, il faut mettre:

```text
Stop ultrason = Oui
```

## 8. Batterie NiMH 7.2V

La batterie utilisee est une NiMH 7.2V.

Tensions approximatives:

```text
pleine  : 8.4V
nominale: 7.2V
faible  : 6.0V
```

La mesure batterie a ete ajoutee sur:

```cpp
const uint8_t BATTERIE_ADC = 35;
```

Montage obligatoire:

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

Constantes de conversion:

```cpp
const float BAT_R1 = 100000.0;
const float BAT_R2 = 47000.0;
const float BAT_TENSION_MAX = 8.4;
const float BAT_TENSION_MIN = 6.0;
const float BAT_CALIBRATION = 1.00;
```

Si la tension affichee ne correspond pas au multimetre, ajuster:

```cpp
const float BAT_CALIBRATION = 1.00;
```

## 9. Gestion Des Intersections

Une premiere version de gestion des intersections a ete ajoutee.

Dans l'interface:

```text
Mode intersections : Oui / Non
Sequence intersections : S,D,G...
Reset intersections
```

Lettres utilisees:

```text
S = tout droit
G = gauche
D = droite
```

Exemple:

```text
S,D,G
```

Signifie:

```text
intersection 1 : tout droit
intersection 2 : droite
intersection 3 : gauche
```

Si le robot voit plus d'intersections que la sequence, il repete la derniere decision.

Detection:

```text
intersection = beaucoup de capteurs noirs en meme temps
```

Execution:

- `S`: avance un court moment tout droit
- `G`: tourne a gauche jusqu'a retrouver une ligne centree
- `D`: tourne a droite jusqu'a retrouver une ligne centree

Le robot evite de compter plusieurs fois la meme intersection avec un verrou interne.

## 10. Challenges Analysees

Le PDF `Challenges vendredi matin 2025.pdf` contient 20 challenges.

Le code actuel peut gerer correctement ou partiellement:

### Faisables ou presque faisables maintenant

```text
Challenge 1 : Depart 2 -> Arrivee 2, tout droit, faire tomber la premiere barriere
Challenge 2 : Depart 1 -> Arrivee 1, tout droit, faire tomber la premiere barriere
Challenge 3 : Depart 1, s'arreter a l'endroit demande
Challenge 4 : Depart 2, s'arreter a l'endroit demande
```

Pour les challenges 1 et 2:

```text
Stop ultrason = Non
```

Pour les challenges 3 et 4:

```text
Mode arret marqueur = Oui
Marqueurs avant stop = nombre a tester
```

### Partiellement faisables avec la gestion intersections

```text
Challenge 8
Challenge 9
Challenge 10
Challenge 11
Challenge 16
```

Ces challenges demandent de prendre les pistes marquees. Le mode intersections avec sequence peut aider, mais il faudra tester sur la vraie piste pour regler les decisions.

### Pas encore geres par le code actuel

Les challenges suivants demandent des fonctions non encore codees:

```text
Challenge 5  : demi-tour + retour
Challenge 6  : demi-tour + retour
Challenge 7  : demi-tour + retour
Challenge 12 : aller + retour
Challenge 13 : aller + retour
Challenge 14 : aller + retour
Challenge 15 : aller + retour
Challenge 17 : aller + retour
Challenge 18 : depart vers depart + demi-tour dans gabarit
Challenge 19 : depart vers depart + demi-tour dans gabarit
Challenge 20 : pause 5 secondes parking B et parking C
```

Il manque encore:

- demi-tour programme
- aller/retour complet
- detection parking B/C
- pause 5 secondes
- eventuellement priorite a droite avec un capteur lateral

## 11. Article 10 - Collision Entre Robots

Regle:

```text
priorite a droite
```

Le robot actuel ne sait pas encore appliquer cette regle.

Avec un seul ultrason a l'avant, il peut detecter un obstacle devant, mais pas vraiment un robot venant de droite.

Pour gerer proprement cette regle, il faudrait:

- soit orienter l'ultrason vers avant-droite
- soit ajouter un deuxieme capteur a droite
- soit faire une logique speciale aux intersections

Decision prise:

```text
laisser cette partie pour plus tard
```

## 12. Etat Actuel Du Code `linefollowerV2.ino`

Le code actuel contient:

- suivi de ligne QTR-8RC
- controle moteur L298N
- interface web ESP32
- reglages `vitesseBase`, `kp`, `kd`
- calibration capteurs depuis l'interface
- affichage des 8 capteurs
- mode arret marqueur
- filtre anti-intersection pour les marqueurs
- capteur ultrason arrivee
- mesure batterie NiMH
- suppression de correction inverse
- gestion de sequence intersections

## 13. Prochaines Evolutions Possibles

Ordre conseille:

1. Tester le suivi simple sur piste.
2. Tester les marqueurs.
3. Tester l'ultrason pour l'arrivee.
4. Tester le mode intersections avec une sequence simple.
5. Ajouter le demi-tour programme.
6. Ajouter les modes aller/retour.
7. Ajouter parking B/C avec pause 5 secondes.
8. Ajouter gestion priorite a droite si necessaire.

## 14. Notes De Test

Pour regler le robot:

```text
Si le robot sort en virage:
  augmenter KP ou baisser vitesseBase

Si le robot zigzague:
  augmenter KD ou baisser KP

Si le robot est stable mais lent:
  augmenter vitesseBase petit a petit
```

Pour tester les intersections:

```text
Mode intersections = Oui
Sequence intersections = S
```

Puis:

```text
S,D
S,G
S,D,G
```

selon le parcours.

Pour tester l'arrivee:

```text
Stop ultrason = Oui
Distance stop = 10 a 20 cm
```

Pour faire tomber la barriere:

```text
Stop ultrason = Non
```

## 15. Fichiers Mentionnes

Fichiers principaux:

```text
C:\Users\mouha\Downloads\linefollowerV2.ino
C:\Users\mouha\Downloads\Challenges vendredi matin 2025.pdf
```

Anciennes versions ou fichiers sources:

```text
C:\Users\mouha\Downloads\Line-Follower-Robot-QTR-8RC.ino
C:\Users\mouha\OneDrive\Documents\Arduino\linefollower\linefollower.ino
C:\Users\mouha\OneDrive\Documents\New project\Robot_Suiveur_Ligne_ESP32_QTR8RC.ino
```

## 16. Remarque Finale

Le code n'a pas pu etre compile via `arduino-cli` car l'outil n'est pas installe sur la machine. Les modifications ont donc ete verifiees par lecture et recherche de references, mais la validation finale doit se faire dans l'IDE Arduino.
