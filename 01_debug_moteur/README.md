# Robot ESP32 Mecanum : pilotage à distance par interface web

Programme pour un robot à 4 roues mecanum piloté par un ESP32, avec un lidar LD06.
L'ESP32 crée son propre réseau WiFi et héberge une page web qui permet de :

- **piloter le robot en direct** (flèches à l'écran ou clavier) ;
- **lui faire suivre une trajectoire** (suite d'étapes, modèles prêts, ou parcours dessiné sur une grille) ;
- **régler les moteurs** (sens, position des roues, puissance) sans recompiler ;
- **sécuriser les moteurs** : ils ne tournent jamais sans votre autorisation.

---

## 1. Matériel

| Élément | Détail |
|---|---|
| Carte | ESP32 Dev Module (sur carte d'extension) |
| Moteurs | 4 moteurs CC, 2 broches de commande chacun |
| Roues | Mecanum (déplacement en crabe possible) |
| Lidar | LD06 (UART) |
| Alimentation | 2 × 18650 |

### Câblage des moteurs

| Moteur | Broche A | Broche B |
|---|---|---|
| Moteur 1 | D15 | D2 |
| Moteur 2 | D4 | D16 |
| Moteur 3 | D17 | D5 |
| Moteur 4 | D18 | D19 |

### Câblage du lidar LD06

| Signal | Broche ESP32 |
|---|---|
| Données lidar (RX) | GPIO 33 |
| Commande vitesse lidar | GPIO 32 |

---

## 2. Installation

### Logiciels

- **Arduino IDE** 1.8.19 (ou 2.x)
- **Carte ESP32** : gestionnaire de cartes → « esp32 by Espressif », version **2.0.17** (la 3.x est aussi gérée)
- Réglage carte : **ESP32 Dev Module**

### Bibliothèques

| Bibliothèque | Rôle |
|---|---|
| `AsyncTCP` | réseau asynchrone |
| `ESPAsyncWebServer` | serveur web |
| `ld06.h` + son `.cpp` | lecture du lidar (à placer dans le dossier du sketch) |
| `Preferences` | mémoire des réglages (incluse avec la carte ESP32) |

> `MecanumCar_v2.h` / `.cpp` ne sont **plus utilisés** : les moteurs sont commandés directement par les broches de l'ESP32.
> Si deux dossiers `AsyncTCP` et `Async_TCP` existent dans `Documents\Arduino\libraries`, gardez `AsyncTCP` et retirez l'autre.

### Structure du dossier

```
ESP32RobotControl/
├── ESP32RobotControl.ino   ← programme complet (interface web incluse)
├── ld06.h
├── ld06.cpp
└── README.md
```

### Téléversement

1. **Coupez l'alimentation des moteurs** (interrupteur batteries) et soulevez le robot.
2. Branchez l'ESP32 en USB, choisissez le bon port, puis **Téléverser**.
3. Si le téléversement échoue (« Failed to connect »), débranchez les moteurs pendant l'envoi : les broches D2, D5 et D15 sont sensibles au démarrage de l'ESP32.

---

## 3. Utilisation

### Se connecter

1. Allumez le robot.
2. Connectez votre téléphone ou ordinateur au WiFi **`RobotESP32`**, mot de passe **`12345678`**.
3. Ouvrez un navigateur à l'adresse **http://192.168.4.1**.
4. Le point vert « connecté » apparaît en haut à droite.

> Le téléphone peut indiquer « pas d'accès Internet » : c'est normal, restez connecté.

### Autoriser les moteurs

Au démarrage, les moteurs sont **verrouillés** 🔒 : toute commande est refusée.
Appuyez sur le bouton orange **« Moteurs verrouillés »** → il devient vert 🔓.
Ils se reverrouillent automatiquement après **2 minutes sans mouvement**.

### Onglet « Moteurs » (première utilisation)

Robot soulevé, moteurs autorisés :

1. Appuyez sur **Avant ▶** pour chaque moteur : la roue doit tourner dans le sens qui fait **avancer** le robot.
2. Sinon, cochez **Inverser**.
3. Choisissez la **position** de la roue (avant gauche, arrière gauche, avant droit, arrière droit).
4. Réglez la **puissance** avec le curseur (128 = 50 %, 255 = 100 %).

Les réglages sont enregistrés dans l'ESP32 et conservés après redémarrage.

### Onglet « Pilotage »

Gardez le doigt appuyé sur une flèche : le robot roule tant que vous appuyez, et s'arrête au relâchement.

| Bouton | Clavier | Mouvement |
|---|---|---|
| ▲ | Z / ↑ | Avancer |
| ▼ | S / ↓ | Reculer |
| ◀ | Q / ← | Crabe à gauche |
| ▶ | D / → | Crabe à droite |
| ↺ | A | Rotation gauche |
| ↻ | E | Rotation droite |
| ■ STOP | Espace | Arrêt immédiat |

Le gros bouton rouge **STOP** en bas de l'écran est disponible sur tous les onglets.

### Onglet « Trajectoire »

- Ajoutez des étapes (mouvement + durée en ms), réorganisez-les avec ↑ ↓, supprimez avec ✕.
- Modèles prêts : carré, carré en crabe, aller-retour, slalom, tour sur place.
- **Répétitions** : nombre de tours (0 = en boucle).
- **▶ Lancer** / **■ Arrêter**.
- Les parcours peuvent être **enregistrés** dans le navigateur.

La trajectoire est exécutée **par l'ESP32 lui-même** : elle continue même si le WiFi est coupé.

### Onglet « Tracé »

1. Touchez la grille pour poser des points de passage (le robot part du centre, face vers le haut).
2. Choisissez le mode : **Rotation puis avance** ou **Crabe** (sans tourner).
3. **Convertir en étapes** ou **Convertir et lancer**.

**Calibration** (indispensable pour un tracé fidèle, le robot n'ayant pas d'encodeurs) :
lancez « Test avance 1 s », mesurez la distance parcourue en mm et saisissez-la ; faites de même pour le crabe (mm) et la rotation (degrés).
Refaites la calibration après tout changement de puissance.

---

## 4. Réglages dans le code

| Constante | Valeur | Rôle |
|---|---|---|
| `ssid` / `password` | `RobotESP32` / `12345678` | Nom et mot de passe du WiFi |
| `MOTOR_PINS` | voir câblage | Broches des 4 moteurs |
| `motorSpeed` | `128` | Puissance par défaut (0–255) |
| `INVERSER_TOUT` | `true` | Inverse le sens de tous les moteurs |
| `PWM_FREQ` | `1000` | Fréquence PWM (Hz) |
| `AUTO_LOCK_MS` | `120000` | Reverrouillage auto (ms, 0 = jamais) |
| `MAX_MOVE_MS` | `2000` | Durée max d'une commande manuelle |
| `STEP_GAP_MS` | `80` | Pause entre deux étapes de trajectoire |

---

## 5. Sécurité moteurs

- Les 8 broches moteur sont mises à **LOW dès le démarrage**, avant même `setup()`.
- Moteurs **verrouillés** au démarrage, autorisation manuelle obligatoire.
- Au repos, les moteurs sont remis à 0 toutes les 300 ms.
- Chaque commande manuelle s'arrête seule après sa durée (max 2 s).
- Reverrouillage automatique après 2 min d'inactivité.

> Pendant les quelques millisecondes du démarrage de l'ESP32 (avant que le programme tourne), les broches D2, D5 et D15 peuvent provoquer un petit à-coup. Solution matérielle : une résistance de 10 kΩ entre chacune de ces broches et GND.

---

## 6. API HTTP

Toutes les routes sont en `GET` sur `http://192.168.4.1`.

| Route | Paramètres | Effet |
|---|---|---|
| `/` | — | Interface web |
| `/arm` | — | Autorise les moteurs |
| `/disarm` | — | Verrouille et arrête tout |
| `/forward` `/backward` `/left` `/right` | `time` (ms, max 2000) | Mouvement temporisé |
| `/rotateleft` `/rotateright` | `time` (ms) | Rotation (durée divisée par 2) |
| `/stop` | — | Arrêt immédiat |
| `/traj` | `seq`, `repeat` | Lance une trajectoire |
| `/traj/stop` | — | Arrête la trajectoire |
| `/motortest` | `m` (1–4), `v` (−255…255), `t` (ms) | Fait tourner un seul moteur |
| `/config` | — | Lit les réglages moteurs (JSON) |
| `/config/set` | `pos`, `inv`, `speed` | Modifie et enregistre les réglages |
| `/status` | — | État du robot (JSON) |
| `/scan` | — | Dernier scan lidar (binaire) |
| `/pose` | `x`, `y`, `z` | Met à jour la pose du robot |
| `/ping` | — | Test de connexion |

Les commandes de mouvement renvoient **403 `LOCKED`** si les moteurs sont verrouillés.

### Format d'une trajectoire

```
/traj?seq=F:1000,TR:450,W:300,L:800&repeat=1
```

| Code | Mouvement |
|---|---|
| `F` | Avancer |
| `B` | Reculer |
| `L` | Crabe gauche |
| `R` | Crabe droite |
| `TL` | Rotation gauche |
| `TR` | Rotation droite |
| `W` | Pause |

Chaque étape : `CODE:durée_ms` (max 20 000 ms, 100 étapes max). `repeat=0` → boucle infinie.

---

## 7. Dépannage

| Problème | Solution |
|---|---|
| `webpage.h: No such file` | Utilisez la version en un seul fichier `.ino` |
| `TrajStep has not been declared` | Utilisez la dernière version du code |
| Page web inaccessible | Vérifiez la connexion au WiFi `RobotESP32` et l'adresse `192.168.4.1` |
| Rien ne bouge | Autorisez les moteurs (bouton orange → vert) |
| Le robot recule au lieu d'avancer | Changez `INVERSER_TOUT` dans le code |
| Une seule roue tourne à l'envers | Onglet Moteurs → cochez/décochez « Inverser » |
| Le crabe part du mauvais côté | Vérifiez la position des roues dans l'onglet Moteurs |
| Moteurs lents ou faibles | Augmentez la puissance, chargez les batteries, vérifiez l'interrupteur moteurs |
| Échec du téléversement | Coupez l'alimentation des moteurs pendant l'envoi |
| Le tracé n'est pas fidèle | Refaites la calibration dans l'onglet Tracé |
