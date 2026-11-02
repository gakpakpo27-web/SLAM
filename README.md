# TP LD06 : extraction des scans et cartographie par superposition

## Contenu

| Fichier | Rôle |
|---|---|
| `ESP32_LD06_Capture.ino` | Firmware ESP32 : envoie un scan 360° en CSV **sur demande** par USB |
| `lidar_core.js` | Bibliothèque de calcul : grid map (Bresenham, mise à jour, export) et segments (extraction, angle, proximité, fusion, intersection, découpe) |
| `test_lidar_core.js` | 60 tests automatiques (`node test_lidar_core.js`), dont les cas limites de la découpe |
| `DEMARCHE_parties_3_4.md` | Démarche des parties 3 et 4 : choix techniques, hypothèses, difficultés, limites |
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
- **Exports** : carte fusionnée (CSV x,y), poses (JSON), image PNG, session complète (JSON, rechargeable).

### Superposition automatique (le plus rapide)
1. Verrouiller 🔒 le scan le plus complet : il sert de référence et fixe le repère de la carte.
2. Cliquer sur **Tout superposer automatiquement**. Pour chaque scan, l'outil :
   - teste toutes les orientations (pas de 3°) et toutes les positions (pas de 20 cm) sur une « carte de
     vraisemblance » construite à partir des scans déjà placés ;
   - affine les meilleurs candidats par ICP point-à-ligne ;
   - garde la pose où le plus de points retombent sur la carte (pourcentage affiché).
3. Les scans au-dessus du **seuil d'acceptation** (35 % par défaut) sont placés et verrouillés, puis servent
   de référence aux suivants. Les autres sont signalés : les rapprocher à la main, puis *Affiner (ICP)*.
4. Ctrl+Z annule toute la superposition d'un coup.

*Placer automatiquement* fait la même chose pour le seul scan sélectionné.
Sur une salle simulée avec 10 positions éparpillées au départ, tout est placé en environ 20 s à ±2 cm et ±0,2°.
Si deux poses très différentes obtiennent presque le même score (pièce symétrique), le scan attend que la carte soit plus complète ; s’il reste ambigu, il est signalé « ⚠ ambigu ».

### Méthode manuelle
1. Charger tous les scans, puis **verrouiller** `pos1` : il définit le repère de la carte.
2. Masquer les autres scans et afficher seulement `pos2`.
3. Le placer **à la main** en faisant coïncider un coin ou un long mur
   (d'abord la rotation, puis la translation).
4. Cliquer sur **Affiner (ICP)** : l'outil corrige automatiquement les derniers centimètres et degrés
   en se calant sur les scans verrouillés (ICP point-à-ligne). Vérifier visuellement, puis **verrouiller** `pos2`.
5. Recommencer pour chaque scan.
6. Exporter la carte fusionnée et les poses.

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
- *Tracer un segment* sert à dessiner ses propres cas de test. *Lancer les tests* exécute les 60 tests dans la page.
- Export au format du cours : `x1,y1,x2,y2` par ligne.

## Développement
`lidar_viewer.html` est autonome : il contient une copie de `lidar_core.js`, `test_lidar_core.js` et `src/maps_ui.js`.
Après modification de ces fichiers, lancer `python3 build.py` pour mettre la page à jour.
