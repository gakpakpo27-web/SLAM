# Traitement des cartes et des obstacles

**Code :** `lidar_maptools.js`.
**Outil :** onglet **Traitement** de `lidar_viewer.html`.
**Tests :** 23 tests « Traitement » dans `test_lidar_core.js` (114 au total, tous réussis).
**Résultats sur les 14 scans réels :** `resultats_traitement.md`.

---

## 1. Ce que demandent les documents du cours, et où c'est fait

Les parties 3 à 5 couvraient la construction des cartes. Les documents décrivent aussi des traitements qui n'étaient pas encore outillés :

| Document (section) | Demande | Outil |
|---|---|---|
| Grid map §1 | cellules enrichies : *timestamp*, compteur de confiance (nombre d'impacts) | `createLayers`, `updateGridLayered` |
| Grid map §2, vectorielle §2 | filtrer les points aberrants (trop loin, intensité faible, incohérents) | `preprocessScan` |
| Grid map §5 | décroissance temporelle | `decayGrid` |
| Grid map §5 | effacement des obstacles obsolètes | `clearObsolete` |
| Grid map §5 | lissage spatial (valeur moyenne ou majoritaire des voisines) | `smoothGrid` (et `closeGaps`) |
| Grid map §5 | cellules d'intérêt (isolées mais très probables) | `removeSpeckles` les garde ; `gridObstacles` |
| Cartographie §2 | durée de vie et **probabilité de persistance** (objets mobiles) | `removeMobile`, persistance des obstacles circulaires |
| Vectorielle §3.1 | segmentation en clusters (distance, angle, intensité) | `clusterScan` |
| Vectorielle §3.2 | régression, **RANSAC**, **cercles / arcs**, **Douglas-Peucker** | `fitCircle`, `ransacLines`, `douglasPeucker` / `dpSegments`, `extractFeatures` |
| Vectorielle §4 | fusionner deux cercles proches (moyenne pondérée) | `updateObstacleMap` |
| Vectorielle §5 | absence : intersection faisceau–cercle | `updateObstacleMap` |
| Vectorielle §6 | correction des collisions géométriques, fusion des redondants, suppression des petites features | `trimCollisions`, `mergeKeepProps`, `regularizeVectorMap` |
| Cartographie §6 et §9 | alignement des mesures ; correction géométrique des poses | `refinePoses` |
| Grid map, remarques | carte « directement exploitable pour la navigation » | `costmap`, `planPath` |

L'onglet enchaîne tous ces outils (`processMap`). Chaque étape est activable, et l'onglet affiche un compte rendu par étape et des mesures de qualité avant et après traitement.

## 2. Démarche, étape par étape

### 2.1 Prétraitement d'un scan

On écarte quatre catégories de points :
- **Portée :** les points à moins de 10 cm ou à plus de 12 m.
  - Sous 10 cm, c'est le robot lui-même (châssis, câbles). Sans ce filtre, chaque position de scan laissait une petite tache d'obstacle sous le robot.
- **Intensité :** les points sous un seuil réglable (désactivé par défaut, car les intensités du LD06 restent élevées sur nos surfaces).
- **Points isolés :** aucun des 2 voisins angulaires de chaque côté n'est à moins de max(8 cm ; 2,5 × l'écart entre deux faisceaux à cette distance). Le seuil grandit avec la distance, car les points s'espacent (0,8° ≈ 14 cm à 10 m).
- **Points mixtes :** quand un faisceau tombe à cheval sur un objet proche et sur le fond, le lidar rend une distance **intermédiaire**, un point fantôme entre les deux.
  - Détection : le point est à plus de 20 cm de ses deux voisins, avec une distance comprise entre les leurs.

### 2.2 Grid map enrichie

On garde la carte uint8 de la partie 3. On y ajoute, pour chaque cellule :
- le **nombre de scans où elle a été un impact** (`hits`) ;
- le **nombre de scans où elle a été observée**, libre ou occupée (`seen`) ;
- le **numéro du dernier scan** qui l'a observée, et de celui qui l'a vue occupée (horodatage).

Cela coûte 12 octets de plus par cellule. C'est le compromis mémoire / information dont parle le cours : la carte uint8 suffit pour naviguer, les couches servent au nettoyage.

On en déduit la **persistance** d'une cellule : impacts ÷ observations.
- 1 : toujours vue occupée (un mur).
- Proche de 0 : vue occupée une fois, puis traversée par d'autres faisceaux (une chaise déplacée, une personne).

`lidar_core.updateGrid` accepte pour cela un rappel facultatif `onCells`. Il ne change rien aux usages existants.

### 2.3 Nettoyage de la grid map

1. **Traces d'obstacles mobiles :** une cellule observée au moins 3 fois, mais occupée dans moins d'un tiers de ces observations, est rendue libre.
   - Exception : une cellule vue comme impact dans au moins 3 scans est **gardée**. Un pied de chaise de 2 cm occupe mal une cellule de 5 cm : les faisceaux venus d'ailleurs la « traversent » souvent sans le toucher.
2. **Taches isolées :** une composante d'une ou deux cellules vue dans un seul scan est du bruit, et elle est remise à « inconnu ».
   - Une tache isolée vue plusieurs fois est une **cellule d'intérêt** (pied de chaise, poteau) : elle est gardée et comptée.
3. **Lissage majoritaire :** une cellule **inconnue** prend l'état d'au moins 5 de ses 8 voisines.
   - On ne lisse pas les cellules occupées : un mur fait une cellule d'épaisseur, et un filtre majoritaire 3 × 3 l'effacerait (3 voisines occupées sur 8). Le mode « moyenne » existe aussi, avec la même protection.
4. **Fermeture des murs :** dilatation puis érosion des cellules occupées (fermeture morphologique). Cela bouche les trous d'une ou deux cellules dans les murs, laissés entre deux faisceaux.
   - Une cellule **franchement libre** (traversée plusieurs fois, valeur < 30) n'est jamais bouchée, pour ne pas fermer une porte.
5. **En option, pour une acquisition en mouvement :**
   - **obstacles obsolètes :** occupée mais plus revue comme impact depuis N scans → libre si on l'a revue libre depuis, sinon inconnue ;
   - **décroissance temporelle :** les cellules non observées depuis N scans se rapprochent de « inconnu ».

   Elles sont désactivées par défaut, car nos 14 scans ne sont pas une séquence dans le temps : le n° de scan n'est pas une durée.

### 2.4 Obstacles

**Dans la grid map :** composantes connexes (8-connexité) des cellules occupées. Pour chacune, on calcule :
- la surface ;
- le centre ;
- la longueur et la largeur, par analyse en composantes principales (pour une répartition uniforme, longueur = √(12 λ)) ;
- l'orientation et le nombre d'impacts maximum.

Classes, par encombrement :
- **petit :** ≤ 15 cm (pied, poteau) ;
- **objet :** ≤ 1 m (chaise, sac, poubelle) ;
- **structure :** au-delà (mur, meuble).

Le lidar ne voit que le **contour** des objets : l'épaisseur ne permet pas de distinguer un mur d'une table, c'est pourquoi on classe sur la taille.

**Features circulaires** (carte vectorielle §1 : cercles, poteaux) :
1. **Clustering** du scan (§3.1) :
   - rupture de distance (10 cm + 3 % de la distance) ;
   - ou trou angulaire de plus de 4° (retours manquants) ;
   - ou saut d'intensité (option).
2. **Reconnaissance**, pour chaque cluster :
   - **plus petit que 15 cm :** petit objet. On ne voit que sa moitié avant, dont le barycentre est à 2r/π ≈ 0,64 r du centre, côté robot : le centre est reculé d'autant.
   - **jusqu'à 60 cm :** ajustement de cercle, d'abord algébrique (Kåsa, système linéaire), puis géométrique (Gauss-Newton sur Σ(|p − c| − r)²). On l'accepte si l'erreur résiduelle est ≤ 12 mm, si 2 cm ≤ r ≤ 30 cm, et si l'objet présente sa face **convexe** au robot. Sinon, un coin de mur concave serait pris pour un cercle.
   - **sinon :** segments, par split-and-merge (partie 4), Douglas-Peucker ou RANSAC.
3. **Carte d'obstacles** (§4 et §5) :
   - **association :** un cercle observé à moins de 15 cm d'un obstacle connu est fusionné par moyenne pondérée (confiance +1) ;
   - **absence :** un faisceau qui passe à travers un obstacle connu et s'arrête plus loin prouve qu'il n'y est plus (confiance −1, compteur « vu absent » +1). L'obstacle est supprimé à confiance nulle ;
   - **persistance** = observations ÷ (observations + absences). L'obstacle est **mobile** s'il a été vu absent au moins 2 fois avec une persistance < 50 % ;
   - les « petits obstacles » posés sur un segment de la carte vectorielle sont retirés : ce sont des **morceaux de mur** trop courts pour devenir des segments.

### 2.5 Carte vectorielle : régularisation

Dans une salle, les murs sont parallèles ou perpendiculaires. La chaîne :
1. **Direction dominante :** histogramme des orientations modulo 90°, pondéré par la longueur, puis moyenne circulaire autour du pic (angle × 4, pour une période de 90°).
2. **Redressement :** un segment à moins de 6° des axes (θ₀ ou θ₀ + 90°) est tourné autour de son milieu pour s'y aligner exactement.
3. **Alignement :** des segments parallèles dont les droites sont à moins de 6 cm sont ramenés sur une même droite (décalage moyen pondéré). Un mur vu par plusieurs scans devient une seule droite.
4. **Fusion** des segments proches, en gardant les propriétés : confiance maximum, observations cumulées.
5. **Coins :** si l'intersection de deux segments (à plus de 60° l'un de l'autre) est à moins de 25 cm d'une de leurs extrémités, l'extrémité est déplacée **exactement** sur l'intersection.
   - Si un seul des deux bouts est proche et que l'intersection tombe dans l'autre segment, c'est une jonction en T : seul le premier bouge.
6. **Collisions géométriques** (§6) : deux segments qui se croisent avec un dépassement de moins de 12 cm → le dépassement est rogné.

### 2.6 Carte de coût et trajet

La carte de coût est calculée à partir de la carte des distances exacte du Jour 2. Valeurs (convention `costmap` de ROS) :
- **254 :** obstacle ;
- **253 :** à moins du rayon du robot (15 cm), le centre du robot ne peut pas y être ;
- **252 → 1 :** dans la marge de sécurité (35 cm), le coût décroît de façon exponentielle ;
- **0 :** libre ;
- **255 :** inconnu.

Le trajet est calculé par **A\*** 8-connexe :
- coût d'un pas = longueur × (1 + 3 × coût de la cellule ÷ 252) : le trajet s'écarte des obstacles quand il le peut ;
- pas de passage en diagonale entre deux cellules bloquées ;
- ensuite, **lissage par visibilité** : on saute les points intermédiaires tant que la ligne droite ne traverse aucune cellule bloquée ou trop coûteuse. On obtient quelques tronçons droits, exécutables par le robot (`/forward`, `/rotateleft`…).

### 2.7 Recalage global des poses

C'est la correction géométrique du document « cartographie » (§6, §9) : on relocalise chaque scan sur la carte des autres, avec les méthodes du Jour 2 (multi-résolution ±30 cm / ±8°, puis ICP point-segment).
- **Passe 1, progressive :** le premier scan fixe le repère. Les autres sont recalés du plus proche au plus loin, chacun sur les scans **déjà recalés**.
  - Une première version recalait chaque scan sur tous les autres d'emblée. Quand tous sont décalés, la référence est incohérente, et les résultats étaient irréguliers (jusqu'à aucune amélioration depuis 15 cm / 5°).
- **Passes 2 à 4 :** affinage sur tous les autres scans, avec arrêt quand plus aucun scan ne bouge de plus de 5 mm et 0,1°.
- **Garde-fous :** une correction est refusée si elle dépasse 30 cm ou 8°, ou si le score d'appariement baisse.

### 2.8 Édition manuelle : gomme et filtres « distance minimum »

Aucun traitement automatique n'est parfait : une personne restée immobile, ou une chaise qui touchait le robot, reste dans la carte. Deux outils permettent de corriger à la main.

**Gomme** (`eraseDisc`) : on la glisse sur la carte, et dans le disque de la gomme :
- les cellules non libres deviennent libres (ou inconnues, au choix) ;
- les segments sont **coupés exactement au bord du disque** : on calcule l'intersection droite–cercle, |P(t) − C|² = r², et les morceaux restants gardent la confiance et l'historique du segment d'origine ;
- les cercles qui touchent le disque sont supprimés.

Entre deux positions de la souris, le passage est interpolé tous les demi-rayons, pour ne pas laisser de trous. Chaque coup de gomme est annulable (Ctrl+Z). La carte de coût, les obstacles et la qualité sont recalculés au relâchement.

**Filtres** (`filterSmall`, `filterNearPoses`), 0 pour désactiver :
- segments plus courts que L ;
- cercles de diamètre inférieur à D ;
- taches de points (composantes d'obstacle de la grid map) plus petites que T ;
- **tout ce qui est à moins de R d'une position du robot**, effacé comme avec une gomme posée sur chaque position de scan : objets qui touchaient le robot, câbles, jambes de l'opérateur.

Ils s'appliquent en fin de chaîne, ou au résultat affiché avec *Appliquer les filtres maintenant*.

### 2.9 Des murs nets

**Constat :** sur la carte réelle, les murs sont épais de 2 cellules, en pointillés noirs et gris, avec des trous. Dans la salle simulée, ils sont nets.

**Cause :** deux scans voient le même mur avec 1 à 2 cm d'écart (précision du LD06 et des poses). En plus, un faisceau qui arrive en **rasant** un mur traverse les cellules de ce mur vues occupées par un autre scan, et les « libère » de 45. Une cellule vue occupée une fois (127 + 70 = 197) retombe alors à 152, c'est-à-dire en gris, après un seul faisceau rasant.

**Deux remèdes :**
1. **Une cellule déjà occupée résiste.** Un faisceau qui la traverse ne la baisse que de 15 au lieu de 45. Les cellules libres ou inconnues, elles, se libèrent toujours de 45 : la surface libre ne change pas.
   - Réglage dans l'onglet Grid map, par défaut 15 ; c'est aussi le réglage de la chaîne de traitement.
   - En mettant 45, on retrouve le comportement d'origine.
2. **Murs nets** (`sharpenWalls`), après la régularisation de la carte vectorielle :
   - pour chaque mur d'au moins 50 cm, on repère le côté intérieur (celui qui a le plus de cellules libres à 30 cm) ;
   - les cellules occupées à moins de 10 cm du mur sont effacées : libres côté intérieur, inconnues côté extérieur ;
   - le mur est ensuite tracé en trait continu d'une cellule.

**Résultats sur la carte réelle.** On mesure la part de la longueur des murs (segments de plus de 1 m) qui a une cellule occupée à moins de 5 cm :

| Réglage | Murs continus | Surface libre |
|---|---|---|
| Réglage d'origine (libération de 45 partout) | 80 % | 115 m² |
| Libération de 20 partout | 95 % | 99 m² : les zones vues une seule fois restent grises |
| **Cellule occupée qui résiste (15)** | **90 %** | **115 m²** |
| + murs nets | 100 % des murs redessinés, d'une cellule d'épaisseur | 115 m² |

**Limite :** les murs nets ne sont aussi justes que la carte vectorielle. Un mur que l'extraction n'a pas trouvé (trop court, trop bruité) n'est pas redessiné : il reste tel qu'il est dans la grille.

### 2.10 Plan de la salle : contours seuls, obstacles supprimés

**But :** obtenir automatiquement le **contour de la salle**, tracé proprement (côtés droits, angles nets), et se débarrasser des obstacles : chaises, pieds, sacs, personnes. Les grands éléments fixes sont gardés.

**Fonction :** `roomPlan`, dans l'onglet Traitement : option « Construire le plan de la salle » et couche « Plan de la salle ».

**Étapes :**
1. **Obstacles effacés.** Chaque tache d'obstacle de la grid map plus petite que 1 m (réglable) compte comme du sol.
2. **Sol de la salle.** On remplit l'espace libre à partir des positions du robot.
3. **Fuites coupées.** Le lidar voit par les portes ouvertes et les fentes. Une ouverture morphologique (érosion puis dilatation) coupe les passages plus étroits que 90 cm, une largeur de porte ; on ne garde que la partie reliée au robot. Sur ta carte, le couloir vu par la porte du haut est coupé : 7,2 m² retirés.
4. **Forme pleine.** Une fermeture morphologique (dilatation puis érosion) bouche les creux du contour plus étroits que 60 cm (objets contre les murs). On bouche ensuite tous les trous.
5. **Structures intérieures.** Les grandes structures (plus de 1 m) situées à plus de 30 cm des murs sont regroupées quand elles sont à moins de 30 cm les unes des autres. Puis :
   - **meuble plein** (le robot n'est jamais passé dedans) : gardé comme le **rectangle** orienté sur les axes de la salle qui englobe ses morceaux. Le lidar n'en voit souvent que 2 ou 3 côtés ;
   - **espace où le robot est passé** (tables en U, comptoir) ou **structure fine** (moins de 25 cm) : gardé comme **cloisons**, c'est-à-dire les segments de la carte vectorielle à cet endroit. L'intérieur reste du sol.
6. **Contour.** On suit le bord de la forme pleine, case par case. On le simplifie (Douglas-Peucker, 8 cm), puis on ajuste une droite sur chaque côté (moindres carrés totaux). Chaque côté à moins de 8° des axes de la salle y est calé. On fusionne les côtés alignés et on retire les crans de moins de 15 cm. Les **coins** sont les intersections des côtés voisins.
7. **Nouvelle grid map.** Le sol est libre, les contours et les cloisons sont occupés, et l'extérieur comme l'intérieur des meubles sont inconnus. Le plan est aussi fourni en segments (carte vectorielle), et sa carte de coût est calculée.

**Résultat sur tes 14 scans :**
- contour de **11 coins**, 45 m de murs, 110 m² de sol ;
- **128 obstacles effacés** ;
- 7,2 m² de fuites (couloir) retirés ;
- l'espace entouré de tables où était pos12 est gardé en 6 cloisons.

**Remarque pour la localisation :** le plan ne contient plus les obstacles, alors qu'en réel 35 à 65 % des points d'un scan tombent sur eux. Pour se repérer, mieux vaut donc la grid map traitée, qui garde les obstacles. Le plan sert à l'affichage, au rapport et à la navigation.

### 2.11 Fermer le contour de la salle à partir de la carte vectorielle

**Constat sur la carte réelle** (`carte_vectorielle_proprietes.txt`, 52 segments) :
- les murs principaux sont bien là, mais pas tout à fait droits (jusqu'à 0,7°) ;
- le contour n'est pas fermé :
  - le mur du bas s'arrête à 2,4 m du coin, puis il manque 1,4 m de mur ;
  - un trou de 0,7 m sur le mur de droite ;
  - une porte de 0,86 m en haut ;
  - les coins ne se rejoignent pas ;
- 30 segments sur 52 n'ont été vus qu'**une seule fois** : chaises, sacs, pieds de table ;
- un segment est **dehors** (y = 10,8 m), vu à travers la porte.

**Outil.** Onglet Vectorielle, bouton **« Fermer le contour »** (`closeOutline`, `lidar_maptools.js`) :
1. **Murs fiables.** On garde les segments longs d'au moins 1 m **et** vus au moins 2 fois, ou ceux vus au moins 5 fois. On y ajoute les segments vus une fois mais posés exactement dans le prolongement d'un mur fiable, car ils le confirment, ainsi que ceux qui referment une forme (4e côté d'un meuble).
2. **Redressement** sur les axes de la salle, puis alignement des morceaux d'un même mur.
3. **Trous comblés jusqu'à 1,6 m.** Un mur caché par un meuble ou une porte est refermé. Les trous de plus de 60 cm sont notés comme **ouvertures**, dessinées en bleu et écrites à la fin du fichier.
4. **Coins en L.** Chaque extrémité est raccordée au mur le plus proche, en prolongeant jusqu'à 2 m, puis jusqu'à 4 m pour les extrémités restées libres. Un raccord n'est accepté que s'il ne fait ni un T au milieu d'un mur, ni pencher un mur de plus de 2°.
5. **Intérieur.** C'est la plus grande zone fermée d'une grille où les murs sont tracés. Ce qui ne la borde pas (dehors, par la porte) est supprimé. Si le contour n'est pas fermé, l'outil le dit et ne supprime rien.

**Résultat sur la carte réelle** (`carte_vectorielle_contour.txt`, `contour_ferme_avant_apres.png`) :
- 52 segments deviennent 10 : les 6 murs de la salle en L et les 4 côtés du meuble central ;
- 10 coins raccordés ;
- le segment vu par la porte est supprimé ;
- contour fermé, 108 m² ;
- 3 ouvertures notées : la porte de 0,86 m en haut, 1,39 m en bas (mur caché, probablement par des tables), 0,70 m à droite.

Test unitaire : salle de 7 × 5 m avec un trou de 1,2 m, des coins non raccordés, un segment dehors et deux chaises. Résultat : 4 murs, 35 m², ouverture de 1,2 m notée.

**Règles ajoutées après l'essai sur l'onglet Segments** (les 14 scans réels) :
- **Murs sur les axes uniquement.** Un segment penché de plus de 2° par rapport aux axes de la salle n'est pas un mur. Sans cette règle, un segment penché de 8° avait été raccroché au meuble.
- **Doublons.** Un trait parallèle à moins de 35 cm d'un mur plus sûr, et qui le longe, est écarté. C'était le cas d'une deuxième ligne 20 cm sous le mur du bas.
- **Marche jamais vue.** Deux murs parallèles dont les extrémités libres se font face (décalées de 0,3 à 3 m, à moins de 60 cm l'une de l'autre dans le sens des murs) : le moins sûr est prolongé et un mur perpendiculaire les relie. Il est marqué « ajouté : jamais vu » et dessiné en bleu. Cela a refermé le décroché du haut de la salle, que personne n'avait vu.
- **Côté manquant d'un meuble.** Deux côtés parallèles qui s'arrêtent au même niveau, à moins de 2 m l'un de l'autre, sont refermés et notés comme ouverture (le 4e côté du meuble central, 1,59 m).
- **Dehors.** Un segment n'est gardé que si au moins la moitié de sa longueur borde l'intérieur. Ainsi, le chambranle et le mur du couloir vus par la porte sont supprimés.

**Onglet Segments** : même bouton « Fermer le contour ». La fiabilité d'un segment y est le nombre de scans qui le voient, c'est-à-dire qui ont des points à moins de 5 cm de lui. Sur les 14 scans réels :
- 70 segments deviennent 12 ;
- contour fermé, 108 m², la même surface que par la carte vectorielle ;
- marche du haut et 4e côté du meuble ajoutés ;
- 2 segments vus par la porte supprimés.

Il reste un petit objet en L près de la position `pos1`, vu par 6 ou 7 scans : un objet réel, ou le pied d'un meuble, à vérifier sur place.

## 3. Résultats

### 3.1 Chaîne complète sur les 14 scans réels (sans recalage, 0,5 s)

| Étape | Résultat |
|---|---|
| Prétraitement | 923 points écartés sur 19 077 : 228 hors portée (dont le robot), 566 isolés, 129 mixtes |
| Traces mobiles | 2548 cellules rendues libres, dont 425 encore « inconnues » (aucune encore occupée : le modèle additif les avait déjà déclassées) |
| Taches isolées | 68 cellules de bruit retirées, 77 petits obstacles confirmés gardés |
| Lissage majoritaire | 441 cellules inconnues tranchées |
| Fermeture des murs | 245 trous bouchés |
| Obstacles de la grille | 93 : 10 structures, 34 objets, 49 petits |
| Obstacles circulaires | 842 cercles observés → 195 obstacles suivis, dont 35 de confiance ≥ 3 ; 91 écartés car posés sur un mur |
| Carte vectorielle | 52 → 46 segments : 31 redressés (axes 0,7° / 90,7°), 15 alignés, 4 fusions, 30 coins raccordés, 3 dépassements rognés |
| Carte de coût | 107 m² accessibles au centre du robot |

**Qualité de la grid map**, brute puis traitée :

| Mesure | Brute | Traitée |
|---|---|---|
| Netteté (cellules observées dont l'état est tranché) | 98,0 % | 100 % |
| Cellules occupées | 1349 | 1570 (murs refermés) |
| Surface libre | 115,0 m² | 116,8 m² |

**Points des scans sur un obstacle :** 56 % en réel, contre 97 % en simulation.
- Cette mesure dépend surtout de l'alignement des scans, pas du nettoyage.
- En réel, beaucoup de points tombent sur des objets fins (pieds de chaises et de tables) vus d'un seul endroit et traversés depuis ailleurs : leur cellule finit libre ou inconnue.
- La couche *Persistance* le montre : murs en jaune (toujours vus), intérieur en violet (persistance < 35 %).

### 3.2 Méthodes d'extraction des segments (14 scans réels)

| Méthode | Segments par scan | Temps par scan | Écart quadratique moyen points → segments | Carte brute | Carte régularisée |
|---|---|---|---|---|---|
| Split-and-merge (partie 4) | 15,1 | 6,6 ms | 1,70 cm | 52 segments, 62,9 m | 46 segments, 10 coins |
| Douglas-Peucker | 19,0 | 1,9 ms | 1,66 cm | 38 segments, 51,4 m | 30 segments, 5 coins |
| RANSAC | 15,0 | 6,9 ms | **1,56 cm** | 40 segments, 60,5 m | 37 segments, 10 coins |

- **RANSAC** colle le mieux aux points : il ignore les chaises devant les murs.
- **Douglas-Peucker** est le plus rapide, mais il découpe davantage, et les petits morceaux sont filtrés (carte plus courte).
- **Split-and-merge** garde le plus de longueur de murs.

### 3.3 Recalage global des poses

**En simulation** (10 scans, poses exactes connues ; tous les scans sauf le premier décalés d'une distance et d'un angle exacts ; 3 graines) :

| Décalage initial | Erreur après recalage | Points sur obstacle | Cellules en conflit |
|---|---|---|---|
| 3 cm / 1° | 1,5 – 2,4 cm / 0,18 – 0,36° | 79–86 % → 97–99 % | ≈ 1100 → ≈ 500 |
| 8 cm / 3° | 1,5 – 2,4 cm / 0,18 – 0,36° | 37–55 % → 97–99 % | ≈ 1100 → ≈ 500 |
| 15 cm / 5° | 1,4 – 2,5 cm / 0,17 – 0,36° | 29–37 % → 97–99 % | ≈ 1100 → ≈ 500 |

Durée : environ 6,5 s pour 10 scans.

**Sur les 14 scans réels** (≈ 12 s) :
- corrections médiane **0,9 cm**, maximum **2,3 cm** (`pos8`) et **0,5°** (`pos14`) ;
- points sur obstacle : 56,2 % → 56,9 %.

Ces corrections sont du même ordre que l'incertitude des poses de référence mesurée au Jour 2 (±3 cm, ±0,6°). Ta superposition était donc déjà bonne : le recalage confirme les poses plus qu'il ne les corrige. Il sera utile au Jour 3, quand les poses viendront des commandes de mouvement, qui sont bien moins précises.

### 3.4 Trajets

On a calculé un trajet A\* entre toutes les paires de positions de scan (robot de 15 cm de rayon, marge de 35 cm) :
- **143 trajets trouvés,** en quelques millisecondes chacun.
- **Distance minimum aux obstacles le long des trajets lissés : 15,0 cm** (le rayon du robot), vérifiée tous les centimètres : aucune collision.
- **39 refus :** la position d'arrivée est elle-même à moins de 15 cm d'un obstacle (`pos1` 6 cm, `pos14` 9 cm, `pos3` 12 cm, `pos12` 13 cm). Le message l'explique.

Ces positions ont pourtant été atteintes par le robot. Il faut donc soit réduire le rayon du robot dans la carte de coût, soit reconnaître que ces points proches viennent d'objets touchant le robot au moment du scan.

Avec le **filtre de distance au robot à 20 cm**, on efface 29 cellules, on coupe 4 segments et on supprime 3 cercles autour des positions de scan. Les **182 trajets** passent alors tous. Ce sont bien des objets collés au robot au moment des scans, et non de vrais obstacles de la salle.

## 4. Limites

- **Persistance, durée de vie, obsolescence :** elles supposent des scans successifs dans le temps. Avec 14 positions statiques, on mesure surtout la cohérence entre points de vue.
- **Obstacles mobiles :** aucun obstacle circulaire n'a été jugé « mobile » sur les données réelles. Il faut qu'un faisceau passe à moins de 2,4 cm du centre d'un pied de chaise pour prouver son absence, ce qui arrive rarement en 14 scans. La grid map, plus dense, détecte bien mieux ces traces.
- **Classes d'obstacles :** elles ne reposent que sur la taille. Un meuble bas sous le plan du lidar (table vue par ses pieds) est vu comme plusieurs petits obstacles : c'est le comportement attendu d'un lidar 2D.
- **Régularisation :** elle suppose une salle « à angles droits ». Les murs penchés de plus de 6° ne sont pas redressés (réglable, 0 pour désactiver).
- **Trajet :** A\* donne une ligne brisée. Le robot Mecanum peut la suivre en translation pure, mais l'exécution réelle (boucle de commande avec relocalisation) reste à faire au Jour 3.
