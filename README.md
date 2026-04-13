# 🤖 Line Follower Robot — GEII BUT1

Robot autonome suiveur de ligne développé dans le cadre des **Rencontres de Robotique BUT GEII 2026** .

Projet réalisé en 1ère année de BUT GEII, en équipe de 3 étudiants.

## Matériel utilisé
- **Microcontrôleur :** ESP32
- **Capteur de ligne :** Pololu QTR-8A (8 capteurs IR)
- **Hacheur moteur :** L298N (Joy-IT SBC-MotoDriver2)
- **Moteurs :** 2× moteurs DC TT + roue folle
- **Détection barrière :** Capteur ultrason HC-SR04
- **Alimentation :** Batterie 12V

## Fonctionnalités
- Suivi de ligne par algorithme PID
- Interface web via Wi-Fi pour régler les paramètres PID en temps réel
- Gestion des lignes en pointillés et segments inversés (blanc sur noir)
- Arrêt automatique à l'arrivée via l'ultrason
- Marche arrière gérée

## Châssis
- Style châssis RC, cadre ouvert
- Imprimé en 3D (PLA), modélisé sur Fusion 360

## 1. Schéma de câblage
<img width="729" height="669" alt="image" src="https://github.com/user-attachments/assets/add4c46d-6830-4269-b8b5-44654aa5e953" />


---
*IUT — BUT GEII 1ère année | Rencontres Robotique 2026*
