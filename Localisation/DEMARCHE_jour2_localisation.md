# Jour 2 : localisation par scan matching

**Code :** `lidar_loc.js` (méthodes, simulateur, banc).
**Outil :** onglet **Localisation** de `lidar_viewer.html`.
**Banc en ligne de commande :** `node bench_localisation.js --real session.json`.
**Résultats complets :** `resultats_localisation.md` (tableaux) et `resultats_localisation.csv` (les 4347 essais).
**Tests :** 14 tests « Localisation » dans `test_lidar_core.js` (88 tests au total, tous réussis).

---

## 1. Le problème

On cherche la pose (x, y, θ) d'un scan, c'est-à-dire la transformation qui fait coïncider ses points avec une **référence**. L'énoncé distingue trois types de référence :

| Référence | Ce qui est comparé | Fonction de coût utilisée |
|---|---|---|
| Grid map | points du scan ↔ cellules occupées | distance au plus proche obstacle, lue dans une carte des distances |
| Carte vectorielle | points du scan ↔ segments | distance au segment le plus proche |
| Entre scans | points du scan ↔ points d'un autre scan pris à proximité | distance au point, ou à la droite locale, la plus proche |

On distingue deux familles de méthodes :
- les **méthodes globales**, qui explorent une fenêtre autour de la pose initiale. Elles sont robustes à une mauvaise initialisation, mais plus lentes ;
- les **méthodes locales**, qui améliorent la pose initiale pas à pas. Elles sont rapides, mais ne convergent que si la pose initiale est proche.

**Préparation commune des points du scan :**
1. conversion polaire → cartésien local (convention de la librairie LD06) ;
2. suppression des points à moins de 40 cm du lidar ;
3. sous-échantillonnage **uniforme dans l'espace** : un point par case de 8 cm, 350 points au maximum.

Sans l'étape 3, les obstacles proches, mesurés tous les 0,8°, pèsent jusqu'à 30 fois plus que les murs. C'est la leçon tirée de la superposition automatique du Jour 1.

## 2. Méthodes réalisées

### 2.1 Grid map

La grid map (partie 3, 5 cm) est transformée en **carte des distances** : pour chaque cellule, la distance à la cellule occupée la plus proche. Elle est calculée par la **transformée de distance euclidienne exacte** de Felzenszwalb et Huttenlocher, en O(cellules).

On en déduit un *likelihood field* : la valeur exp(−d²/2σ²) vaut 1 sur un obstacle et décroît avec la distance. Le score d'une pose est la moyenne de ce champ sous les points du scan.

| Méthode | Principe | Réglages |
|---|---|---|
| **Recherche exhaustive** | toutes les poses d'une fenêtre ±1,2 m, ±50° autour de la pose initiale | pas de 5 cm / 1°, σ = 5 cm, 150 points ; ≈ 240 000 poses testées |
| **Multi-résolution** | grille grossière (20 cm / 4°), champ lissé (σ = 20 cm) qui « voit » de loin ; on garde les 8 meilleures poses, puis on raffine autour de chacune en divisant les pas par 2 | niveaux 20 cm/4° → 10 cm/2° → 5 cm/1° → 2,5 cm/0,5° |
| **Descente de gradient** | Gauss-Newton / Levenberg-Marquardt : on minimise Σ d(T(p))², grâce au gradient de la carte des distances (interpolation bilinéaire) | poids de Cauchy (c = 10 cm), points à plus de 50 cm puis 20 cm ignorés, 60 itérations max |

**Pourquoi les poids de Cauchy ?** Sans eux, les points tombés sur des objets absents de la carte (pieds de chaises vus d'un autre angle, personnes) tirent la solution à tort. Sur les scans réels, depuis la bonne pose, le taux de réussite passe de **57 % à 93 %**.

### 2.2 Carte vectorielle

Les segments de la carte (partie 5) sont rangés dans un **index spatial** : des cases de 50 cm qui listent les segments proches. Il donne en temps quasi constant le point le plus proche sur le segment le plus proche.

| Méthode | Principe | Réglages |
|---|---|---|
| **Recherche exhaustive** | même fenêtre que la grid map ; score = moyenne de exp(−d²/2σ²), avec d la distance au segment le plus proche | pas de 10 cm / 2°, puis recherche exhaustive locale à 2,5 cm / 0,5° (une distance à des segments coûte plus cher qu'une lecture de cellule) |
| **Descente de gradient (ICP point-segment)** | Gauss-Newton sur la distance de chaque point à son segment le plus proche : à la droite si la projection tombe dans le segment, à l'extrémité sinon ; 15 % des pires appariements rejetés ; rayon d'appariement 60 cm → 10 cm | 60 itérations max |
| **Calcul direct** | on extrait les segments du scan (partie 4) et on recolle les morceaux d'un même mur ; on forme des **coins** (deux segments à plus de 35°, intersection à moins de 1,5 m de leurs extrémités). Chaque paire coin du scan ↔ coin de la carte dont les deux directions concordent (à 8° près) donne la pose **par une formule** : θ = différence des orientations, puis t = coin carte − R(θ)·coin scan. Toutes les hypothèses sont notées, la meilleure est affinée par ICP | **n'utilise pas la pose initiale** : c'est une localisation globale |

### 2.3 Entre scans

La référence est un scan pris **à proximité**, comme l'exige l'énoncé :
- **en simulation :** à 0,3–0,6 m et ±20° du scan testé ;
- **en réel :** le scan le plus proche, à moins de 1,5 m (paires `pos5`/`pos6`, `pos8`/`pos9`, etc.).

La vérité est la pose **relative** des deux scans.

| Méthode | Principe |
|---|---|
| **ICP point-à-point** | appariement au point le plus proche (index spatial), solution fermée de la rotation et de la translation (décomposition en valeurs singulières, en 2D : un atan2), 20 % des pires appariements rejetés, rayon 1 m → 10 cm |
| **ICP point-à-ligne** | normales du scan de référence (ACP sur les voisins à moins de 20 cm), Gauss-Newton sur la distance aux droites locales |
| **Corrélation multi-résolution** | le scan de référence devient une mini grid map (partie 3), puis multi-résolution (2.1) et affinage par gradient |

## 3. Protocole d'évaluation

Il respecte la consigne : mêmes scans et mêmes poses initiales pour toutes les méthodes.

**Perturbation de la pose initiale.** Pour chaque scan testé et chaque niveau, la pose initiale est la vraie pose décalée d'une distance **exacte** dans une direction aléatoire, et tournée d'un angle exact ± :

| Niveau | 0 | 1 | 2 | 3 | 4 |
|---|---|---|---|---|---|
| Distance / angle | 0 | 0,1 m / 5° | 0,25 m / 10° | 0,5 m / 20° | 1 m / 45° |

- 5 tirages par niveau (1 seul au niveau 0).
- Les tirages sont faits **une seule fois** (graine 7) et réutilisés par **toutes** les méthodes.

**Métriques :**
- erreur de position (m) et erreur angulaire (°) ;
- temps de calcul (Node.js 22, un cœur ; ordre de grandeur identique dans Chrome) ;
- taux de réussite.

**Seuils de réussite :**
- **≤ 5 cm et ≤ 2°.** 5 cm correspond à une cellule de grille et à la précision annoncée du LD06 (30 mm typique, 45 mm max). 2° est l'erreur angulaire maximale de la fiche technique, et donne 10 cm d'écart à 3 m.
- Une **réussite grossière** (≤ 20 cm et ≤ 5°) est aussi comptée : elle indique qu'une méthode locale pourrait finir le travail.
- Une méthode qui ne rend **aucun** résultat (le calcul direct sans coin visible) compte comme un échec.

**Simulation, à poses exactes :**
- salle en L de 13,5 × 9,8 m, avec un décroché, un meuble central, une armoire et 70 pieds de chaises ;
- bruit de distance σ = 1,5 cm, bruit angulaire σ = 0,15°, 8 % de points perdus ;
- carte construite avec 9 scans de 3 tours ;
- 10 scans de test d'un seul tour, à des poses aléatoires dans l'espace libre.

**Réel :**
- tes 14 scans, avec comme poses de référence celles de la superposition (Jour 1) ;
- pour chaque scan testé, la grid map et la carte vectorielle sont reconstruites **sans ce scan** (validation croisée « leave-one-out »). Sinon, le scan se retrouverait lui-même dans la carte.

**Incertitude des références réelles.** Chaque scan a été recalé (ICP point-segment) sur la carte des autres, depuis sa pose de référence perturbée de 3 cm et 1,5°, 10 fois.
- Écart à la référence : médiane **1,4 cm / 0,11°**, 90e centile **2,3 cm / 0,55°**.
- Les références réelles sont donc connues à environ **±3 cm et ±0,6°**. En réel, une erreur inférieure à cela n'est pas significative, et le seuil de 5 cm reste au-dessus de ce bruit.

## 4. Résultats

Chaque case donne : taux de réussite (≤ 5 cm et ≤ 2°), puis erreur médiane de position et d'angle. Le temps moyen par localisation est dans la dernière colonne.

### 4.1 Simulation (poses exactes)

| Référence | Méthode | 0 | 0,1 m / 5° | 0,25 m / 10° | 0,5 m / 20° | 1 m / 45° | Temps |
|---|---|---|---|---|---|---|---|
| Grid map | Recherche exhaustive | 100 % | 100 % · 1,5 cm | 100 % · 2,0 cm | 100 % · 1,9 cm | 100 % · 2,0 cm | 211 ms |
| Grid map | Multi-résolution | 100 % | 100 % · 1,3 cm | 100 % · 1,6 cm | 100 % · 1,4 cm | 100 % · 1,4 cm | 12 ms |
| Grid map | Gradient (Gauss-Newton) | 100 % · 0,9 cm | 100 % · 0,9 cm | 100 % · 0,9 cm | 70 % | 0 % | **2 ms** |
| Carte vect. | Recherche exhaustive | 100 % | 100 % · 0,9 cm | 100 % · 1,0 cm | 100 % · 1,1 cm | 100 % · 1,0 cm | 432 ms |
| Carte vect. | ICP point-segment | 100 % · 0,3 cm | 100 % · 0,4 cm | 100 % · 0,4 cm | 8 % | 0 % | **1 ms** |
| Carte vect. | Calcul direct | 90 % · 0,3 cm | 90 % | 90 % | 90 % | 90 % | 2 ms |
| Entre scans | ICP point-à-point | 100 % · 0,3 cm | 98 % | 52 % | 0 % | 0 % | 4 ms |
| Entre scans | ICP point-à-ligne | 100 % · 0,3 cm | 100 % · 0,5 cm | 100 % · 0,5 cm | 98 % | 4 % | 4 ms |
| Entre scans | Corrélation multi-rés. | 100 % · 0,4 cm | 100 % | 100 % | 100 % | 100 % · 0,4 cm | 14 ms |

Erreurs angulaires médianes : 0,00° à 0,07° pour toutes les méthodes en réussite.

### 4.2 Scans réels (14 scans, carte sans le scan testé)

| Référence | Méthode | 0 | 0,1 m / 5° | 0,25 m / 10° | 0,5 m / 20° | 1 m / 45° | Temps |
|---|---|---|---|---|---|---|---|
| Grid map | Recherche exhaustive | 93 % | 90 % · 2,1 cm | 91 % · 2,3 cm | 93 % · 2,6 cm | 91 % · 2,1 cm | 193 ms |
| Grid map | Multi-résolution | 100 % · 2,5 cm | 97 % · 1,8 cm | 96 % · 1,9 cm | 97 % · 2,1 cm | 96 % · 1,9 cm | 14 ms |
| Grid map | Gradient (Gauss-Newton) | 93 % · 1,5 cm | 94 % · 1,5 cm | 93 % · 1,6 cm | 11 % | 0 % | **2 ms** |
| Carte vect. | Recherche exhaustive | 100 % · 2,5 cm | 100 % · 2,0 cm | 100 % · 1,9 cm | 100 % · 1,6 cm | 100 % · 1,6 cm | 447 ms |
| Carte vect. | ICP point-segment | 100 % · 1,4 cm | 100 % · 1,4 cm | 89 % · 1,7 cm | 3 % | 0 % | **1 ms** |
| Carte vect. | Calcul direct | 21 % | 21 % | 21 % | 21 % | 21 % | 1 ms |
| Entre scans | ICP point-à-point | 100 % · 1,3 cm | 95 % · 1,1 cm | 64 % · 1,8 cm | 0 % | 0 % | 4 ms |
| Entre scans | ICP point-à-ligne | 100 % · 0,9 cm | 100 % · 0,9 cm | 98 % · 0,9 cm | 42 % | 0 % | 5 ms |
| Entre scans | Corrélation multi-rés. | 100 % · 0,8 cm | 100 % | 100 % | 100 % | 100 % · 0,8 cm | 17 ms |

Erreurs angulaires médianes en réussite : 0,00° à 0,35°.

## 5. Analyse

1. **Méthode minimale qui fonctionne pour chaque type de référence :**
   - grid map : multi-résolution, 96–100 % en réel jusqu'à 1 m / 45°, en 14 ms ;
   - carte vectorielle : ICP point-segment, 100 % jusqu'à 10 cm / 5°, en 1 ms ; et la recherche exhaustive, 100 % partout ;
   - entre scans : corrélation multi-résolution, 100 % partout, en 17 ms.
2. **Sensibilité à l'initialisation :** c'est la différence majeure entre les familles.
   - Les **méthodes locales** (gradient, ICP) sont 10 à 400 fois plus rapides, mais s'effondrent au-delà de 25 cm / 10° : 0 à 11 % de réussite à 0,5 m / 20°. Elles convergent vers le minimum le plus proche.
   - Les **méthodes globales** ne dépendent pas de la perturbation tant qu'elle reste dans leur fenêtre (±1,2 m, ±50°). Au-delà, elles échouent par construction.
   - L'**ICP point-à-ligne** a un bassin de convergence nettement plus large que le point-à-point : 98 % contre 52–64 % à 25 cm / 10°. Il ne « glisse » pas le long des murs.
3. **Précision :** toutes les méthodes en réussite sont au centimètre, bien en dessous de la précision du LD06.
   - La précision des méthodes de recherche est limitée par leur pas (≈ 2 cm, 0,5°).
   - Les méthodes de gradient atteignent 0,3 à 1,5 cm, car elles ne sont pas discrétisées.
   - En réel, les erreurs de 1 à 2,5 cm sont du même ordre que l'incertitude des références (±2,3 cm) : on ne peut pas départager les méthodes plus finement.
4. **Coût :** la recherche exhaustive est 15 à 30 fois plus lente que la multi-résolution, pour la même robustesse. C'est tout l'intérêt de la multi-résolution. Le coût de la carte vectorielle par pose testée est plus élevé (distance à des segments, plutôt qu'une lecture de cellule).
5. **Le calcul direct** n'a besoin d'aucune initialisation, et il est très rapide (2 ms).
   - En simulation, il réussit 90 % des cas (les 10 % restants sont des scans sans coin visible, qui ne rendent pas de résultat, et non une pose fausse).
   - En réel, seulement 21 % : tes scans pris au milieu des tables voient rarement deux murs qui se rejoignent près de leurs extrémités.
   - Il est donc idéal pour **initialiser** une méthode locale, quand des coins sont visibles.
6. **Les échecs réels se concentrent sur `pos2`** (5 essais sur 11 aux niveaux 0–2, en multi-résolution) et, dans une moindre mesure, sur `pos13` et `pos14`.
   - `pos2` est le scan déjà jugé **ambigu** lors de la superposition : il voit surtout des obstacles proches et un seul coin de mur.
   - Sa pose de référence est elle-même moins sûre. Une partie de ces « échecs » vient probablement de la référence, et non de la méthode.
7. **Recommandation pour le Jour 3 (SLAM) :**
   - initialiser avec la commande de mouvement envoyée au robot, ou avec le calcul direct quand il trouve des coins ;
   - ensuite, multi-résolution sur la grid map (robuste, 14 ms) ;
   - enfin, affinage par ICP point-segment sur la carte vectorielle (1 ms, au centimètre).

## 6. Limites et pistes

- **Fenêtre de recherche fixe** (±1,2 m, ±50°) : au-delà, il faut une recherche globale sur toute la carte (plus lente), ou le calcul direct.
- **Méthode probabiliste (filtre particulaire) non réalisée.** Elle permettrait de suivre plusieurs hypothèses dans une salle presque symétrique. C'est une piste pour le Jour 3, si le temps le permet.
- **Références réelles limitées :** elles viennent de la même chaîne de recalage (superposition). Une validation indépendante serait plus solide : marquer quelques positions au sol et mesurer au mètre (±2 cm, ±2°).
- **Bruit simulé idéalisé :** pas de reflets, pas de vitres, pas de personnes qui bougent.
- **Temps mesurés sur un seul cœur,** en JavaScript : les ordres de grandeur comptent plus que les valeurs absolues.

## 7. Robot en direct : se repérer sur la carte du Jour 1

**Objectif :** la carte de la salle est faite (Jour 1). On pose le robot n'importe où, et il doit **se retrouver tout seul** sur cette carte, puis se suivre pendant qu'il roule. La vue suit le robot comme un GPS.

**Outils :**
- onglet **Robot en direct** de `lidar_viewer.html` ;
- fonctions `prepareMap`, `globalLocalize`, `trackerUpdate` de `lidar_loc.js` ;
- banc `node bench_suivi.js --real session.json`, dont les résultats sont dans `resultats_suivi.md`.

### 7.1 Démarche

1. **Carte figée.** On prend la grid map et la carte vectorielle du Jour 1. Elles peuvent venir des scans visibles, de l'onglet Traitement ou de fichiers exportés. On calcule une fois pour toutes :
   - la carte des distances ;
   - l'index des segments ;
   - les **positions candidates** : toutes les cellules vues libres, tous les 20 cm.
2. **Localisation globale, sans pose de départ :**
   - chaque position candidate est testée sur 360°, tous les 4°, avec 80 points du scan sur un champ lissé (≈ 2 800 positions × 90 angles, environ 0,15 s) ;
   - on ajoute l'hypothèse du calcul direct par les coins ;
   - les 8 meilleures hypothèses bien distinctes (plus de 50 cm ou de 20° d'écart) sont affinées (multi-résolution fine, puis ICP point-segment) ;
   - chaque hypothèse est notée par la **part des points du scan à moins de 10 cm d'un obstacle de la carte**.
3. **Confiance,** calée sur les 14 scans réels :
   - **sûr :** au moins 30 % des points collent, et la 2ᵉ hypothèse fait moins de 80 % du score de la 1ʳᵉ ;
   - **ambigu :** une autre hypothèse est presque aussi bonne ;
   - **incertain :** rien ne colle.
4. **Suivi :** à chaque nouveau scan, on part de la pose précédente, plus le déplacement commandé s'il est connu (commandes WiFi `/forward`… × vitesse réglée, ou odométrie simulée). La recherche se fait dans une petite fenêtre (±35 cm et ±15°, élargie avec le déplacement) : 5 à 10 ms.
5. **Se rendre compte qu'on est perdu.** Un scan est suspect s'il colle à moins de 25 % de la carte, ou à moins de 70 % de la moyenne des scans précédents (seuil relatif, utile car tes scans encombrés ne collent qu'à 35–65 %). On relance alors une recherche globale, gardée seulement si elle colle nettement mieux : c'est le cas du robot « kidnappé », déplacé à la main.
6. **Ambiguïté.** Si la recherche globale est ambiguë, on suit jusqu'à 3 hypothèses en parallèle : le mouvement du robot les départage. On peut aussi **indiquer la position à la main** (clic, puis glisser vers l'avant du robot), comme le « 2D Pose Estimate » de ROS : le scan suivant se recale autour de cette position.

### 7.2 Vue « GPS »

- **Cap en haut :** la carte tourne pour que l'avant du robot soit toujours vers le haut. Le robot est placé aux deux tiers de l'écran, pour voir plus loin devant lui.
- **Autres modes :** nord en haut, ou vue libre (on glisse la carte ; la touche C recentre).
- **Repères :** une boussole montre où sont les axes x et y de la carte, et une échelle donne les distances.
- **Points du scan :** en **vert** s'ils collent à la carte, en **rouge** sinon. Le rouge, c'est un obstacle nouveau (chaise déplacée, personne), ou le signe d'une pose fausse.
- **Autres éléments affichés :** la trajectoire, les hypothèses en pointillés orange quand c'est ambigu, la vraie pose (pointillés blancs) en relecture et en simulation, et le bandeau d'état.

### 7.3 Résultats

**Localisation globale sur les 14 scans réels.** Aucune pose de départ ; chaque scan est cherché sur la carte des 13 autres.

| Mesure | Résultat |
|---|---|
| Scans retrouvés (≤ 5 cm et ≤ 2°) | **12 / 14** |
| Scans classés « sûr » | **11, tous justes** |
| Scans faux (pos2, pos3) | tous deux classés **« ambigu »**, jamais « sûr » |
| Bonne pose parmi les hypothèses gardées | 13 / 14 |
| Temps médian | ≈ 0,2 s |

**Suivi en simulation.** Poses exactes ; 264 scans, avec des pas de 15 cm et des rotations de 15° ; odométrie avec 10 % d'erreur, plus un biais de 1 cm et 0,5° par pas.

| Scénario | Scans justes | Temps médian par scan |
|---|---|---|
| Avec odométrie | 264 / 264 | 6 ms |
| Sans odométrie (pose précédente seule) | 264 / 264 | 5 ms |
| 25 objets absents de la carte (chaises, cartons, personnes) | 264 / 264 | 5 ms |
| Robot déplacé à la main à mi-parcours | 264 / 264 : relocalisé dès le scan du déplacement | 7 ms |

**Relecture des 14 scans réels dans l'onglet** (carte sans le scan rejoué) : 12 / 14 justes. pos2 et pos3 sont affichés « ambigu ». Avec une indication à la main, pos3 se recale à 0,2 cm et 0,08°.

### 7.4 Limites

- **La simulation est plus facile que le réel** (pas de reflets, bruit idéal) : les erreurs y sont de 0,3 cm. En réel, il faut compter 1 à 3 cm, soit l'incertitude des poses de référence.
- **Vitesses de la prédiction :** celles des commandes WiFi (0,25 m/s, 90°/s) sont à mesurer sur ton robot. Une prédiction fausse n'est pas grave : le suivi tolère ±35 cm et ±15°, et la recherche globale rattrape le reste.
- **Scans encombrés pris au milieu des tables** (pos2, pos3) : ils restent ambigus tant que le robot ne bouge pas. Il faut alors avancer de quelques dizaines de centimètres, ou indiquer la position à la main.
- **Fréquence des scans :** la recherche globale (0,2 s) dépasse l'intervalle entre deux scans en WiFi (5 Hz). Pendant qu'elle tourne, les scans arrivés entre-temps sont sautés : on ne garde que le plus récent.
