# Exploration autonome : le robot cartographie seul la salle

## 1. Objectif

On pose le robot à une **position de départ**, on lance l'exploration, et le robot :

1. choisit seul où aller pour voir ce qu'il ne connaît pas encore ;
2. contourne les murs, les meubles et les pieds de chaises ;
3. s'arrête régulièrement pour scanner, et **superpose lui-même** ses scans en tenant compte de ses déplacements ;
4. revient à son point de départ quand toute la salle est vue ;
5. annonce « cartographie terminée » à l'écran, avec un signal sonore et une phrase lue à voix haute.

Tout se trouve dans l'onglet **Scans**, encadré *Cartographie automatique*, partie *Exploration autonome*. Le code est dans `lidar_explore.js` ; l'interface est dans `src/maps_ui.js`.

## 2. Superposer les scans en tenant compte des déplacements

C'est le cartographe de la cartographie automatique (`createMapper` / `mapperUpdate`, voir `DEMARCHE_cartographie_auto.md`).

- **Prédiction.** Chaque ordre envoyé au robot donne un déplacement prévu : par exemple « avance 1 200 ms à 0,25 m/s », soit 30 cm. La position prédite du robot est la précédente plus ce déplacement.
- **Recalage.** Le scan pris après l'arrêt est recalé sur la carte déjà construite, en partant de la prédiction : recherche multirésolution puis affinage. La prédiction n'est qu'un point de départ, car un robot sans encodeurs ne fait jamais exactement ce qu'on lui demande. C'est le recalage qui donne la vraie position.
- **Nouvelle position.** Tous les 40 cm ou 25°, le scan est ajouté à la carte (« image clé ») avec sa pose. Il apparaît dans la liste des scans (`auto1_01`, `auto1_02`…), déjà superposé.

**Stop and go.** Le robot avance par petits pas : au plus 35 cm ou 60° par ordre. Les scans reçus pendant qu'il roule sont ignorés. Le scan utilisé est pris à l'arrêt, au moins 450 ms après la fin du mouvement, donc sans déformation due au mouvement.

## 3. Où aller : les frontières

Une **frontière** est une cellule libre de la grille qui touche une cellule encore inconnue. C'est la méthode classique de Yamauchi (1997).

1. Les cellules frontières sont regroupées en amas. Les amas de moins de 30 cm sont ignorés, par exemple les petites ombres derrière un pied de chaise.
2. Un **Dijkstra** part de la position du robot sur la carte de coût. Il donne la longueur du meilleur chemin vers chaque cellule.
3. La frontière choisie est celle qui minimise `longueur du chemin − 1 × min(taille de la frontière, 3 m)`. Le robot va donc au plus proche, mais un grand pan inconnu vaut un petit détour.
4. Le chemin est la remontée de l'arbre de Dijkstra, lissée : on garde la ligne droite tant qu'elle ne frôle aucun obstacle.
5. Le plan est recalculé à chaque nouvelle position enregistrée, puisque la carte a changé.

Une frontière atteinte, ou impossible à atteindre en 40 ordres, est mise sur une liste noire (rayon de 45 cm). On ne la retente pas, car elle peut rester inconnue indéfiniment : surface noire mate, vitre.

**Fin.** Quand plus aucune frontière n'est atteignable, la salle est entièrement vue. Le robot planifie alors un retour au départ (option cochée par défaut). Une fois arrivé, il annonce la fin.

Le réglage « Rester à moins de … m du départ » interdit d'explorer au-delà de cette distance. C'est utile si la porte est ouverte.

## 4. Éviter les obstacles

Le robot évite les obstacles à trois niveaux.

1. **Carte de coût.** Les obstacles de la carte sont gonflés du rayon du robot (17 cm par défaut) : son centre ne peut pas entrer dans cette zone. Au-delà, le coût décroît sur 30 cm, et le trajet s'écarte des obstacles quand il le peut.

2. **Pare-chocs lidar.** Avant chaque avance, le scan courant est lu directement, sans passer par la carte. On cherche le point le plus proche dans un couloir droit devant, de la largeur du robot plus 6 cm de chaque côté. L'avance est limitée pour garder 12 cm entre l'avant du robot et ce point.
   - S'il ne reste presque plus de place, l'obstacle est noté sur la carte de coût (« obstacle vu en route ») et le trajet est recalculé.
   - Au bout de 3 blocages d'affilée, le robot recule de 15 cm si c'est libre derrière. Sinon, il tourne sur place vers la direction la plus dégagée.

3. **Plan de secours.** Si rien n'est atteignable, on refait le plan sans les obstacles vus en route, qui ont pu partir ou être mal placés. Ensuite, on le refait avec un rayon du robot réduit de 3 cm, pour un passage étroit. Un obstacle vu en route est de toute façon oublié après 25 ordres.

### 4 ter. Passer au plus près, longer les murs, sortir d'un coin

Ces réglages viennent des premiers essais réels :
- **Obstacles proches seulement.** Le robot passe entre deux obstacles dès qu'il y a la place.
  - Couloir de sécurité à 3 cm de chaque côté du robot, au lieu de 6.
  - 8 cm gardés devant, au lieu de 12.
  - Les obstacles vus en route sont gonflés juste du rayon du robot.

  En simulation, il franchit un passage de 50 cm ; il faut au moins 2 × 17 cm, plus une cellule.
- **Anticipation sur 1 m.** À chaque arrêt, les points du scan (lidar et ultrason) sont comparés au trajet prévu sur le prochain mètre. Un obstacle nouveau qui le barre est noté tout de suite, et le trajet est recalculé avant d'arriver dessus. À moins de 1 m d'un obstacle devant, les pas passent de 35 à 20 cm.
- **Crabe (roues mecanum).** Si la cible est sur le côté (65° à 115°), le robot fait un pas de côté au lieu de tourner, ce qui lui permet de longer un mur. Face à une impasse, il essaie dans l'ordre :
  1. longer l'obstacle en crabe, du côté du but ;
  2. reculer ;
  3. tourner vers la direction la plus dégagée.

  La vitesse en crabe se corrige seule, comme les autres.
- **Départ dans un coin.** Avant, le contrôle du cap exigeait de la place devant et derrière : dans un coin, le robot enchaînait des rotations de 20° à gauche puis à droite, et semblait tourner tout seul. Maintenant, il tourne **une seule fois** vers la direction la plus dégagée (jusqu'à 180°), puis avance. S'il manque encore de place, le contrôle se fait en route, sur les mouvements suivants. Le verdict « lidar mal monté » n'est rendu que sur au moins 3 mesures concordantes.
- **Anti-toupie.** Après 4 rotations de suite sans avancer, le robot change de manœuvre (avance, crabe ou recul, vers le plus dégagé) et change de but.
- **Le robot n'avance plus** (obstacle invisible, roue qui patine) : il note l'obstacle, abandonne le but, recule ou tourne de 90°, et continue ailleurs. Il ne s'arrête qu'après 3 fois.

En simulation, avec 5 départs dans des coins, face au mur : 1 à 2 rotations avant de bouger, au plus 3 rotations de suite pendant toute l'exploration, aucun choc, 100 % de couverture.

### 4 bis. Capteur ultrason avant sur servo

Le robot porte à l'avant un HC-SR04 monté sur un servo, qui balaie 180°. Il complète le lidar là où celui-ci est aveugle : une vitre (l'ultrason rebondit dessus) ou un objet plus bas que la nappe du lidar (sac, carton, marche, pied de chaise à roulettes).

**Dans le firmware** (`ESP32_LD06_WiFi.ino`) :
- **Balayage continu** de 0 à 180° par pas de 15°, soit 13 directions et environ 1,3 s pour un aller. Pendant une marche avant, le balayage se réduit à 60–120°, pour voir devant plus souvent.
- **Pare-chocs autonome.** Pendant une marche avant, si une mesure récente (moins de 0,4 s) place un obstacle dans le couloir du robot à moins de 15 cm du capteur, les moteurs s'arrêtent tout de suite. Il n'y a ni aller-retour WiFi ni attente du PC : c'est le dernier filet de sécurité.
- **Route `/us`** : dernières mesures, au format `[angle, mm, âge en ms]`. `/us?on=0` coupe le balayage, `/us?on=1` le relance. La touche `u` sur l'USB affiche les mesures.

**Dans l'exploration**, les mesures prises robot arrêté deviennent des points dans le repère du robot (angle, distance, position du capteur devant le centre). Ces points s'ajoutent à ceux du lidar pour le pare-chocs logiciel du § 4 : un obstacle vu seulement par l'ultrason arrête l'avance, est noté sur la carte de coût et fait recalculer le trajet. Ils ne vont **pas** dans la carte : l'ultrason est trop imprécis (cône d'environ 15° à 30°) pour dessiner des murs.

**Résultat en simulation** (8 salles, 8 cartons bas invisibles au lidar) : avec le lidar seul, 24 chocs et 3 explorations terminées sur 8 ; avec le lidar et l'ultrason, 0 choc et 8 sur 8 (`resultats_exploration.md`).

**Broches à vérifier.** Elles sont réglées par défaut sur TRIG GPIO13, ECHO GPIO14 et servo GPIO25 (`#define US_TRIG`, `US_ECHO`, `US_SERVO`). Si le capteur est alimenté en 5 V, sa broche ECHO sort 5 V : il faut un pont diviseur (par exemple 1 kΩ et 2 kΩ) avant l'ESP32. Si un objet à gauche apparaît à droite, mettre `US_SERVO_INV` à 1, ou cocher « gauche et droite inversées » dans l'outil.

**Perdu.** Si le scan ne colle plus à la carte, le robot ne bouge plus. Au bout de 6 scans, il annule son dernier mouvement. Au bout de 24, il s'arrête et le signale.

## 5. Contrôle du montage et des vitesses, aux premiers mouvements

Si le 0° du lidar n'est pas l'avant du robot, l'ordre « avancer » envoie le robot dans une autre direction que celle que le lidar croit regarder. Les obstacles « devant » ne seraient alors pas ceux qu'il va percuter. L'exploration commence donc par une **phase de contrôle** :

- 2 petites avances de 15 cm, seulement s'il y a de la place devant **et** derrière, au cas où le lidar serait monté à l'envers ;
- 2 rotations de 20°.

Ces mouvements sont recalés **sans prédiction**, donc sans a priori : on mesure le vrai déplacement.

| Mesure | Conclusion |
|---|---|
| Le robot avance à plus de 25° de l'avant du lidar | Arrêt. Le message donne le décalage d'angle à régler dans les options d'affichage |
| Les rotations mesurées sont à l'envers de la commande | Arrêt : scan en miroir, cocher ou décocher « miroir » |
| Sinon | Les vitesses réelles mesurées remplacent celles du réglage. Les durées des ordres sont plus justes, et la prédiction aussi |

Ensuite, si le robot n'avance plus alors qu'il en reçoit l'ordre (4 fois de suite), il s'arrête. C'est le cas d'un robot coincé contre un obstacle trop bas pour le lidar, ou d'une roue qui patine.

## 6. Position de départ

Les champs *Départ x, y, cap* donnent la pose du robot dans le repère de la carte au moment du lancement.

- **0 ; 0 ; 0 (par défaut).** Le départ devient l'origine de la nouvelle carte. Le marquer au sol (ruban adhésif, avec une flèche pour le sens) : c'est la « base » qu'utilise ensuite le bouton *Le robot est sur sa base* de l'onglet Robot en direct.
- **Autres valeurs.** La nouvelle carte est construite directement dans le repère d'une carte existante, à condition de connaître la pose du robot dans ce repère.

## 7. Résultats en simulation

Le détail est dans `resultats_exploration.md` (commande : `node bench_exploration.js`). Conditions :

- salle de 13,5 × 9,8 m en L, avec meuble central, armoire et 60 pieds de chaises ;
- 12 départs tirés au hasard ;
- vitesses réelles différentes du réglage (× 0,8 à 1,2), odométrie à ±10 %.

| | Résultat |
|---|---|
| Cartographie terminée et retour au départ | 12/12 |
| Couverture de la surface libre | 100 % en médiane (minimum 99 %) |
| Chocs | 0 |
| Erreur de position du robot pendant l'exploration | 2 cm en médiane, 10 cm au pire |
| Trajet | 36 m et environ 170 ordres en médiane, soit 6 à 10 min sur le vrai robot |
| Lidar tourné de 90° ou de 180°, scan en miroir | Détecté en 4 ordres, arrêt sans choc, avec le réglage à faire |

Le même code a aussi été testé dans `lidar_viewer.html` :

- avec le robot simulé : exploration complète et annonce ;
- avec un faux robot WiFi qui exécute les ordres HTTP reçus, en divisant la durée des rotations par 2 comme le firmware : vitesses mesurées à 5 % près, position à 2 cm.

## 8. Limites (à surveiller en vrai)

- **Ce que le lidar ne voit pas.** C'est une nappe horizontale à la hauteur du lidar. Vers l'avant, l'ultrason couvre les vitres et les objets bas, mais il voit mal les objets fins ou mous (tissu) et ce qui le frappe en biais. Un plateau de table au-dessus des deux capteurs reste invisible, et **rien ne surveille l'arrière** pendant un recul (au plus 15 cm, et seulement si le lidar voit libre derrière). Il faut rester à côté, avec le bouton **STOP** ou la touche Espace.
- **Robot sans encodeurs.** Il avance « au temps ». La prédiction peut être fausse de 30 % : le recalage la corrige, mais sur un sol glissant ou un tapis, réduire la vitesse (onglet Moteurs du robot).
- **Pieds de chaises serrés.** Deux pieds espacés de moins de 2 × 17 cm sont infranchissables pour le planificateur. Une rangée de chaises fait donc mur, ce qui est voulu.
- **Salle qui bouge.** Une personne qui passe est vue comme un obstacle. Elle peut laisser une trace sur la carte : nettoyer ensuite avec l'onglet Traitement (objets mobiles, gomme).
