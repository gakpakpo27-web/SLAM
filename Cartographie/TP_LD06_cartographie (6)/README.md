# TP LD06 : extraction des scans et cartographie par superposition

## Contenu

| Fichier | Rôle |
|---|---|
| `ESP32_LD06_Capture.ino` | Firmware ESP32 : envoie un scan 360° en CSV **sur demande** |
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
- **Souris** : clic sur un scan pour le sélectionner, glisser pour le translater,
  Shift + glisser pour le tourner autour de son origine, Shift + molette pour une rotation fine.
- **Clavier** : flèches (1 cm, Shift 10 cm), A/E (±0,5°, Shift 5°), Tab (scan suivant), F (recadrer), Ctrl+Z (annuler).
- **Filtres** : intensité minimale, plage de distances, sens des angles, décalage angulaire.
- **Exports** : carte fusionnée (CSV x,y), poses (JSON), image PNG, session complète (JSON, rechargeable).

### Méthode pour obtenir une bonne carte
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
