# TP LD06 : extraction des scans et cartographie par superposition

## Contenu

| Fichier | Rôle |
|---|---|
| `ESP32_LD06_WiFi.ino` | Firmware WiFi + USB : partie WiFi reprise du firmware du cours `ESP32RobotControl.ino` (réseau RobotESP32, mêmes routes), plus les commandes USB de capture |
| `ESP32_LD06_Capture.ino` | Firmware ESP32 : envoie un scan 360° en CSV **sur demande** par USB |
| `lidar_core.js` | Bibliothèque de calcul : grid map (Bresenham, mise à jour, export), segments (extraction, angle, proximité, fusion, intersection, découpe) et carte vectorielle |
| `lidar_loc.js` | Jour 2 : localisation (grid map, carte vectorielle, entre scans), simulateur et banc d'évaluation |
| `bench_localisation.js` | Banc d'évaluation en ligne de commande (`node bench_localisation.js --real session.json`) |
| `lidar_maptools.js` | Traitement des cartes et des obstacles : prétraitement, clusters, cercles / RANSAC / Douglas-Peucker, grid map enrichie et nettoyée, obstacles, régularisation de la carte vectorielle, carte de coût, trajet A*, recalage global des poses |
| `DEMARCHE_traitement_cartes.md` | Démarche et résultats du traitement des cartes et des obstacles |
| `resultats_traitement.md` | Rapport de la chaîne de traitement sur les 14 scans réels (avec recalage) |
| `test_lidar_core.js` | 106 tests automatiques (`node test_lidar_core.js`), dont les cas limites de la découpe |
| `DEMARCHE_parties_3_4.md` | Démarche des parties 3 et 4 : choix techniques, hypothèses, difficultés, limites |
| `DEMARCHE_partie_5.md` | Démarche de la partie 5 (carte vectorielle) |
| `capture_lidar.py` | (Optionnel) capture en ligne de commande, 1 fichier CSV par position |
| `lidar_viewer.html` | Capture des scans + visualisation et recalage (Chrome/Edge, rien à installer) |

---

## 1. Extraction des données lidar

### Principe
Le LD06 envoie en continu (230 400 bauds) des paquets de 12 mesures. La librairie `ld06.h` les décode
(vérification CRC, angle de début et de fin, distance en mm, intensité de 0 à 255) et les regroupe en
un tour complet (`readScan()` renvoie `true`). Chaque point est accessible par `ld06.getPoints(i)`,
avec `angle` (°), `distance` (mm) et `intensity`.

Le firmware fourni n'envoie rien tant qu'on ne lui demande rien. Sur la commande `s`, il envoie le
tour suivant :

```
#SCAN,<id>,<nbPoints>,<vitesse °/s>
angle_deg,distance_mm,intensite      (une ligne par point, distances nulles exclues)
#END
```
Autres commandes : `1`…`9` (N tours), `c` (flux continu), `t` (Teleplot temps réel), `x` (stop).

### Connexion WiFi (méthode du cours, RobotControl)
Deux firmwares possibles, avec les mêmes routes WiFi :
- `ESP32_LD06_WiFi.ino` (ce projet) : la partie WiFi du firmware du cours, plus les commandes USB de capture.
- `ESP32RobotControl.ino` (ressources du module, `Tools/RobotControl/ESP32RobotControl/`).

Bibliothèques : LD06, MecanumCar_v2 (archives .ZIP du module), AsyncTCP et ESPAsyncWebServer (projet ESP32Async).

1. Téléverser le firmware (carte *ESP32 Dev Module*), puis ouvrir le moniteur série à 115200 bauds pour lire l'adresse.
2. Connecter le PC au WiFi **RobotESP32** (mot de passe `12345678`).
3. Dans l'outil, choisir **WiFi (RobotControl)**, garder l'adresse `192.168.4.1` (ou celle affichée), puis **Connecter le robot**.
   Si Chrome bloque l'accès au réseau local, l'autoriser pour la page dans les paramètres du site.
4. Capturer comme en USB. Chaque tour est lu sur `GET /scan` (format binaire du cours : pose + N points), une requête à
   la fois, 5 par seconde au maximum. Plusieurs tours sont fusionnés dans un même scan.
5. *Utiliser la pose envoyée par le robot* : le scan est placé à la pose saisie dans RobotControl (`/pose`). Ce n'est
   pas une localisation calculée ; si elle n'a pas été saisie, elle vaut (0, 0, 0).

**Pilotage depuis l'outil** (panneau affiché une fois connecté en WiFi) : boutons ou touches
`I`/`K` avant/arrière, `J`/`L` translation gauche/droite, `U`/`O` rotation, `Espace` arrêt immédiat
(routes `/forward`, `/backward`, `/left`, `/right`, `/rotateleft`, `/rotateright` avec `?time=ms`, max 2000 ms ;
les rotations durent la moitié côté robot). La déconnexion envoie `/stop`.
*Envoyer la pose au robot* appelle `/pose?x=mm&y=mm&z=centièmes de degré` : cette pose revient avec chaque scan.
*Pose du scan sélectionné* reprend la pose d'un scan déjà superposé. Le robot n'a pas d'odométrie :
après un déplacement, sa pose n'est pas mise à jour automatiquement.

Fichiers des outils du cours acceptés par *Charger fichiers* :
- scans RobotControl `angle_deg,distance_m,intensity` (distances en mètres, pose lue dans le commentaire `#`) ;
- scans MapTool `angle_deg,r_m,x_rel_m,y_rel_m` (`r_m = 0` : pas d'impact, ignoré) ;
- cartes vectorielles `x1,y1,x2,y2` (onglets Segments et Vectorielle).

### Procédure de capture (directement dans `lidar_viewer.html`)
1. Téléverser `ESP32_LD06_Capture.ino` (avec `ld06.h`/`ld06.cpp` de la librairie).
   **Fermer le moniteur série d'Arduino**, car le port ne peut être ouvert qu'une seule fois.
2. Ouvrir `lidar_viewer.html` dans **Chrome ou Edge** (Firefox et Safari ne gèrent pas la connexion série).
3. **Connecter le robot**, puis choisir le port COM de l'ESP32. Le baudrate doit correspondre à `PC_BAUD` (115200 par défaut).
4. À chaque position dans la salle :
   - poser le robot. L'*aperçu en direct* aide à vérifier ce qu'il voit ;
   - **ne plus le toucher et ne pas passer dans le champ** ;
   - donner un nom (`pos1` est proposé par défaut), choisir le nombre de tours (3 conseillés), puis **Capturer** (ou Entrée).
5. Prendre 4 à 8 positions, avec **au moins 50 % de murs en commun** entre deux positions voisines.
6. Exporter **Scans bruts (CSV)** : un seul fichier en coordonnées polaires
   (`#SCAN,nom` puis `angle_deg,distance_mm,intensity`), relisible plus tard avec *Charger fichiers*.

Les scans sont aussi sauvegardés automatiquement dans le navigateur. Après une fermeture ou un
rechargement, le bouton *Restaurer la dernière session* les fait revenir.

*Alternative sans navigateur :* `python capture_lidar.py --port COM5 --tours 3` enregistre un CSV par position.

---

## 2. Visualisation et cartographie

### Conversion polaire → cartésien
Le LD06 tourne physiquement en **sens horaire**, mais la librairie convertit déjà l'angle en sens
trigonométrique (`angle = 360° − angle_physique`). Donc, pour un point (α, d) de `getPoints()` :

```
xs = d · cos α          ys = d · sin α        (repère du robot, x vers l'avant du lidar)
```

Puis, avec la pose du scan (x, y, θ) dans la carte (rotation puis translation) :

```
X = x + xs·cos θ − ys·sin θ
Y = y + xs·sin θ + ys·cos θ
```

> Remarque : `printScanTeleplot()` calcule `y = −d·sin α`, donc l'image affichée sur Teleplot est
> **en miroir**. L'outil corrige ce miroir à l'import pour rester cohérent avec les captures directes.

### Importer un enregistrement Teleplot
Sur teleplot.fr, exporter la série `lidar` en **CSV** (lignes `"x","y","timestamp"`). Le fichier
`teleplot_layout_….json` ne contient que la mise en page, il n'est pas nécessaire.

Dans l'outil, **Charger fichiers**, puis choisir le CSV : une fenêtre d'import s'ouvre.
- Chaque ligne de l'export est un point (x, y en mm) et les points d'un même tour partagent le même timestamp.
- L'outil regroupe les tours et **détecte automatiquement les périodes où le robot est immobile**.
  Chaque période devient un scan net. Pendant que le robot bouge, les tours sont flous et ignorés.
- Cliquer sur une période pour la prévisualiser, ou cliquer-glisser sur la chronologie pour choisir une période à la main.
- *Ajouter la période comme scan*, ou *Ajouter toutes les périodes détectées*.

Conseil : pendant l'enregistrement Teleplot, laisser le robot **immobile 5 à 10 s** à chaque position.

### Utilisation de `lidar_viewer.html`
- Les scans capturés apparaissent directement. Sinon, **Charger fichiers** (CSV, JSON), ou les glisser sur la carte.
  *Données démo* simule une salle pour s'entraîner.
- Chaque scan a sa couleur, un bouton afficher/masquer, un verrou et ses champs x, y, θ modifiables.
- **Souris** : clic sur un scan pour le sélectionner, glisser pour le translater. Rotation : tirer le bouton ⟳ sur le cercle autour du scan sélectionné, ou passer en mode *Tourner* (barre en haut de la carte, touche R), ou utiliser le curseur et les boutons ⟲ ⟳ de chaque scan ;
  Shift + glisser pour le tourner autour de son origine, Shift + molette pour une rotation fine.
- **Taille d'un scan** : boutons −5 / −1 / −.1 / +.1 / +1 / +5 (en %) et champ « Taille % » dans la carte du scan,
  ou touches `-` / `+` (±0,5 %). Le scan grandit ou rétrécit autour de son origine (le robot) ; 1:1 revient à 100 %.
  En principe les distances du LD06 sont justes (100 %) : une taille différente sert à compenser un écart de mesure.
- **Clavier** : flèches (1 cm, Shift 10 cm), A/E (±0,5°, Shift 5°), Tab (scan suivant), F (recadrer), Ctrl+Z (annuler).
- **Filtres** : intensité minimale, plage de distances, sens des angles, décalage angulaire.
- **Exports** :
  - **Fusion : données complètes (CSV)** → `fusion_scans_complet.csv`, une ligne par point brut des scans visibles :
    `scan, point, angle_deg, distance_mm, intensite, pose_x_m, pose_y_m, pose_theta_deg, taille,
    x_local_m, y_local_m, x_global_m, y_global_m, garde`.
    La colonne `garde` vaut 1 si le point est utilisé dans la carte, et 0 s'il est retiré par les filtres
    (distance, intensité, points isolés).
  - **Carte fusionnée (CSV)** : seulement `x_m, y_m, scan` des points gardés.
  - **Poses (CSV)** → `poses_scans.csv` : `scan, x_m, y_m, theta_deg, taille, verrouille, visible, points_bruts, points_gardes`.
  - Poses (JSON), image PNG, scans bruts (CSV polaire, sans pose), session complète (JSON, rechargeable).

### Superposition automatique (le plus rapide)
1. Verrouiller 🔒 **un seul** scan, de préférence un scan qui voit bien les murs : il sert de référence et fixe le repère.
2. Cliquer sur **Tout superposer automatiquement**. Pour chaque scan, l'outil :
   - garde des points répartis uniformément (1 tous les 8 cm environ) et ignore ceux à moins de 40 cm, pour que les
     obstacles collés au lidar (pieds de table, meuble contre le robot) ne pèsent pas plus que les murs ;
   - teste toutes les orientations (pas de 3°) et toutes les positions (pas de 20 cm) sur une carte construite avec les
     scans déjà placés, puis affine les 24 meilleurs candidats par ICP point-à-ligne ;
   - calcule un score : points qui retombent sur la carte − 0,3 × faisceaux qui traversent un mur connu (segments
     d'au moins 1 m extraits des scans placés).
3. Un scan est placé (et verrouillé) si sa meilleure pose est **nettement meilleure** que toute pose très différente
   (à plus de 1 m ou 20°). Les moins ambigus sont placés en premier et servent aussitôt de référence aux suivants.
4. Les scans **ambigus** (par exemple un coin encombré qui ressemble au coin opposé) ne sont **pas** placés : une pose
   fausse gâcherait la carte. Ils sont listés dans le compte rendu. Pour chacun : le glisser à peu près à sa place
   (à 1 m et 30° près suffit), puis **Placer automatiquement**.
5. Ctrl+Z annule toute la superposition d'un coup. Une barre de progression avec un bouton **Arrêter** s'affiche
   pendant le calcul.

**Placer automatiquement** (scan sélectionné) : si le scan a été déplacé à la main, la recherche se fait autour de sa
position (±1,5 m, ±35°) ; s'il est encore à l'origine, elle se fait partout.

**Résultats mesurés :**
- Salle de classe réelle (14 scans, dont 10 pris au milieu des tables) : 11 scans placés automatiquement, tous justes
  (à 3 cm et 1° près), et 3 signalés ambigus. Ceux-ci sont replacés à 1–2 cm près avec *Placer automatiquement*
  après les avoir posés grossièrement.
- Salles simulées (pièce symétrique, salle encombrée de 160 pieds de chaises) : tous les scans justes, à 2 cm et 0,2° près.

### Méthode manuelle
1. Charger tous les scans, puis **verrouiller** `pos1` : il définit le repère de la carte.
2. Masquer les autres scans et afficher seulement `pos2`.
3. Le placer **à la main** en faisant coïncider un coin ou un long mur
   (d'abord la rotation, puis la translation).
4. Cliquer sur **Affiner (ICP)** : l'outil corrige automatiquement les derniers centimètres et degrés
   en se calant sur les scans verrouillés (ICP point-à-ligne). Vérifier visuellement, puis **verrouiller** `pos2`.
5. Recommencer pour chaque scan.
6. Exporter **Fusion : données complètes (CSV)** et **Poses (CSV)**.

Sur données simulées (bruit de 1 cm), partir d'une erreur manuelle d'environ 20 cm et 5° donne
une pose finale à moins de 1 cm et 0,2° près.

### Pièges fréquents
- **Carte en miroir** : cocher « Inverser le sens des angles » (par exemple si le LD06 est monté à l'envers).
- **Points parasites** autour du robot (châssis, câbles) : augmenter « Distance min ».
- **Vitres et surfaces noires** : peu de retours ou des retours faux. Monter « Intensité min ».
- **Personnes qui passent** : refaire le scan, ou les ignorer, car l'ICP rejette les 15 % de pires appariements.
- **L'ICP diverge** : le placement manuel est trop loin (> 30–50 cm). Rapprocher davantage à la main.

---

## 3. Grid map (onglet *Grid map*)
- **Outil 1** : *Créer depuis le scan sélectionné*. Le scan et sa pose donnent une nouvelle carte d'occupation.
- **Outil 2** : *Mettre à jour avec le scan sélectionné*. La carte existante (en mémoire ou chargée depuis un `.json`) est mise à jour avec le scan et sa pose : les cellules traversées (Bresenham) deviennent libres, les cellules d'impact occupées.
- *Construire avec tous les scans visibles* enchaîne les deux.
- Exports : `.json` (rechargeable), `.pgm` + `.yaml` (ROS), `.png`.

## 4. Segments (onglet *Segments*)
- *Extraire* transforme le nuage de points en segments (split-and-merge), puis fusionne les segments proches de tous les scans.
- Clic = segment A, Shift + clic = segment B. L'outil affiche leur angle, s'ils sont « proches » et leur intersection.
  Boutons : *Fusionner*, *Couper A par B*, *Couper B par A*.
- *Tracer un segment* sert à dessiner ses propres cas de test. *Lancer les tests* exécute les 88 tests dans la page.
- Export au format du cours : `x1,y1,x2,y2` par ligne.

## 5. Carte vectorielle (onglet *Vectorielle*)
- *Intégrer tous les scans visibles*, ou *Pas à pas* (un scan à la fois, changements surlignés).
- À chaque scan : extraction des segments → gestion des absences (un faisceau qui traverse un segment le raccourcit,
  le scinde ou baisse sa confiance) → association / fusion avec la carte → nettoyage → coins.
- Propriétés de chaque segment : confiance, nombre d'observations, premier et dernier scan (durée de vie réglable).
- Affichage par confiance (rouge → vert), filtre « confiance minimum », coins (carré jaune + bissectrice).
- Fichiers : export `.txt` (`x1,y1,x2,y2,confiance,observations,premier,dernier` + coins), chargement d'une carte
  initiale (murs connus), copie vers l'onglet Segments.

## Jour 2 : localisation (onglet *Localisation*)
**Localiser un scan :**
1. Superposer les scans, puis *Mémoriser les poses de référence* : ce sont les « vraies » poses utilisées pour mesurer l'erreur.
2. Sélectionner un scan, le *Déplacer* (perturbation choisie) ou le glisser à la main.
3. Choisir la référence et la méthode, puis *Localiser le scan sélectionné*. L'écart à la référence s'affiche.

La grid map et la carte vectorielle de référence sont construites avec les **autres** scans visibles. En mode « Scan de
référence », on choisit le scan pris à proximité.

| Référence | Méthodes |
|---|---|
| Grid map | recherche exhaustive (5 cm / 1°), multi-résolution (20 cm / 4° → 2,5 cm / 0,5°), descente de gradient (Gauss-Newton robuste sur la carte des distances) |
| Carte vectorielle | recherche exhaustive (10 cm / 2° puis 2,5 cm / 0,5°), descente de gradient (ICP point-segment), calcul direct (appariement de coins, sans pose initiale) |
| Entre scans | ICP point-à-point, ICP point-à-ligne, corrélation multi-résolution |

**Banc d'évaluation** (onglet *Localisation* ou `node bench_localisation.js --real session.json`) :
- mêmes scans et mêmes poses initiales (même graine) pour toutes les méthodes ;
- perturbations de la pose initiale : 0 ; 0,1 m / 5° ; 0,25 m / 10° ; 0,5 m / 20° ; 1 m / 45° ;
- mesures : erreur de position, erreur angulaire, temps, taux de réussite (≤ 5 cm et ≤ 2°, réglables) ;
- simulation à poses exactes, et scans réels (carte construite sans le scan testé) avec estimation de l'incertitude
  des poses de référence ;
- résultats en tableaux et courbes, export CSV et Markdown.

## Traitement des cartes et des obstacles (onglet *Traitement*)
Chaîne complète sur les scans visibles, chaque étape activable, avec un compte rendu par étape et la qualité de la carte
avant / après :
1. (option) **recalage global des poses** : chaque scan est relocalisé sur la carte des autres (passe progressive puis
   passes d'affinage) ;
2. **prétraitement** : rayon du robot, portée, intensité, points isolés, points mixtes ;
3. **grid map enrichie** : par cellule, nombre d'impacts, d'observations et numéro du dernier scan ;
4. **nettoyage** : traces d'obstacles mobiles (persistance), taches isolées, lissage majoritaire, fermeture des murs,
   et en option obstacles obsolètes et décroissance temporelle ;
5. **obstacles** : composantes connexes classées petit / objet / structure, et obstacles circulaires suivis
   (confiance, absence, persistance) ;
6. **carte vectorielle** (split-and-merge, Douglas-Peucker ou RANSAC) puis **régularisation** : murs redressés sur les
   axes de la salle, alignés, fusionnés, coins raccordés, dépassements rognés ;
7. **carte de coût** (obstacles gonflés du rayon du robot) et **trajet A\*** : clic départ, clic arrivée.

Couches affichables : grid map traitée ou brute, différences, persistance, obstacles par classe, carte de coût.
Les résultats peuvent être envoyés aux onglets Grid map et Vectorielle, et exportés (PGM + YAML, .txt, CSV, PNG, rapport .md).

## Développement
`lidar_viewer.html` est autonome : il contient une copie de `lidar_core.js`, `lidar_loc.js`, `lidar_maptools.js`, `test_lidar_core.js` et `src/maps_ui.js`.
Après modification de ces fichiers, lancer `python3 build.py` pour mettre la page à jour.
