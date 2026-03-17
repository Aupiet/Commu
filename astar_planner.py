#!/usr/bin/env python3
"""
Sprint 3 – A* Path Planner
Branché directement sur le launch file de mapping existant.

Pipeline :
  /map  (slam_toolbox)  →  frontières  →  A*  →  /planned_path

Lancement :
  python3 astar_planner.py
"""

import heapq
import numpy as np
import rclpy
import rclpy.time
from scipy.ndimage import binary_dilation, gaussian_filter
from math import sqrt
from rclpy.node import Node
from rclpy.duration import Duration
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from scipy.ndimage import distance_transform_edt
from tf2_ros import Buffer, TransformListener, LookupException, \
                    ConnectivityException, ExtrapolationException

# ─── Valeurs de la grille slam_toolbox ───
FREE     =  0
OCCUPIED = 100
UNKNOWN  = -1


# ─────────────────────────────────────────────────────────────
#  MODULE 2 – DÉTECTION DES FRONTIÈRES DU CIRCUIT
# ─────────────────────────────────────────────────────────────

def compute_boundary_map(grid: np.ndarray) -> dict:
    """
    À partir de la carte SLAM, extrait :
      - inner_boundary : frontière intérieure (île centrale)
      - outer_boundary : frontière extérieure (murs)
      - centerline_candidates : zone médiane libre

    Stratégie :
      1. Distance transform sur les cellules libres
      2. Les maxima locaux de la distance transform = ligne médiane naturelle
    """
        
    free_mask     = (grid == FREE)
    occupied_mask = (grid == OCCUPIED)

    # Distance de chaque cellule libre à l'obstacle le plus proche
    dist_transform = distance_transform_edt(free_mask)

    # Frontières = cellules libres adjacentes à un obstacle
    struct  = np.ones((3, 3), dtype=bool)
    dilated = binary_dilation(occupied_mask, structure=struct)
    boundary = free_mask & dilated   # cellules libres sur le bord d'un obstacle

    # Ligne médiane = cellules libres dont la distance aux obstacles est maximale localement
    # On lisse d'abord pour éviter les artéfacts
    dist_smooth = gaussian_filter(dist_transform.astype(float), sigma=2.0)

    # Seuil : garder seulement les cellules suffisamment éloignées des deux côtés
    median_threshold = dist_smooth.max() * 0.45
    centerline_mask  = (dist_smooth > median_threshold) & free_mask

    return {
        'dist_transform'    : dist_transform,
        'boundary_mask'     : boundary,
        'centerline_mask'   : centerline_mask,
        'dist_smooth'       : dist_smooth,
    }

# ═══════════════════════════════════════════════════════
#  ÉTAPE 2 – ALGORITHME A*
#  Entrée  : grille binaire, start (col,row), goal (col,row)
#  Sortie  : liste de (col, row) = chemin optimal
#
#  Particularité rover :
#    Le coût de chaque cellule est majoré si elle est proche
#    d'un mur (via dist_transform), ce qui pousse A* à rester
#    naturellement au centre de la piste.
# ═══════════════════════════════════════════════════════

def create_node(position, g=float('inf'), h=0.0, parent=None):
    return {'position': position, 'g': g, 'h': h, 'f': g + h, 'parent': parent}


def heuristic(a, b):
    return sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)


def get_neighbors(binary_grid, position, dist_transform, safety_radius=3):
    x, y = position
    rows, cols = binary_grid.shape
    moves = [
        (x+1, y, 1.0), (x-1, y, 1.0), (x, y+1, 1.0), (x, y-1, 1.0),
        (x+1, y+1, 1.414), (x-1, y-1, 1.414),
        (x+1, y-1, 1.414), (x-1, y+1, 1.414),
    ]
    result = []
    for nx, ny, base_cost in moves:
        if not (0 <= nx < rows and 0 <= ny < cols):
            continue
        if binary_grid[nx, ny] != 0:
            continue
        d = dist_transform[nx, ny]
        penalty = 10.0 * (safety_radius - d + 1) if d < safety_radius \
                  else max(0.0, float(safety_radius + 2 - d))
        result.append(((nx, ny), base_cost * (1.0 + penalty)))
    return result


def reconstruct_path(goal_node):
    path, cur = [], goal_node
    while cur:
        path.append(cur['position'])
        cur = cur['parent']
    return path[::-1]


def find_path(binary_grid, start, goal, dist_transform, safety_radius=3):
    node0     = create_node(start, g=0.0, h=heuristic(start, goal))
    open_list = [(node0['f'], start)]
    open_dict = {start: node0}
    closed    = set()

    while open_list:
        _, cur_pos  = heapq.heappop(open_list)
        cur_node    = open_dict[cur_pos]

        if cur_pos == goal:
            return reconstruct_path(cur_node)

        closed.add(cur_pos)

        for nb_pos, cost in get_neighbors(binary_grid, cur_pos,
                                          dist_transform, safety_radius):
            if nb_pos in closed:
                continue
            new_g = cur_node['g'] + cost

            if nb_pos not in open_dict:
                nb = create_node(nb_pos, g=new_g,
                                 h=heuristic(nb_pos, goal), parent=cur_node)
                heapq.heappush(open_list, (nb['f'], nb_pos))
                open_dict[nb_pos] = nb

            elif new_g < open_dict[nb_pos]['g']:
                nb           = open_dict[nb_pos]
                nb['g']      = new_g
                nb['f']      = new_g + nb['h']
                nb['parent'] = cur_node
                heapq.heappush(open_list, (nb['f'], nb_pos))

    return []   # aucun chemin


# ═══════════════════════════════════════════════════════
#  NŒUD ROS2
#  Se branche directement sur le launch file existant :
#    /map          ← slam_toolbox
#    TF map→base_link ← slam_toolbox + rf2o
#    /planned_path → publié (visible dans rviz2)
# ═══════════════════════════════════════════════════════

class AStarPlannerNode(Node):

    def __init__(self):
        super().__init__('astar_planner')

        # Lecture de la position robot via TF
        # Chaîne TF produite par votre launch file :
        #   map → odom       (slam_toolbox)
        #   odom → base_link (rf2o)
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Abonnement à /map (slam_toolbox publie ici la grille)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        # Publication du chemin calculé (à visualiser dans rviz2)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)

        self.planned = False   # on ne planifie qu'une fois
        self.get_logger().info('A* Planner démarré – en attente de /map...')

    # ── Lecture position robot ──────────────────────────
    def get_robot_position(self):
        """
        Retourne (x, y) du robot en mètres dans le repère map,
        ou None si la TF n'est pas encore disponible.
        """
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
            return t.transform.translation.x, t.transform.translation.y
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'TF non disponible : {e}')
            return None

    # ── Réception de la carte ───────────────────────────
    def map_callback(self, msg: OccupancyGrid):
        if self.planned:
            return   # déjà fait

        # Reconstruction de la grille numpy depuis le message ROS2
        # msg.data = liste 1D row-major (0=libre, 100=occupé, -1=inconnu)
        grid = np.array(msg.data, dtype=np.int8).reshape(
                   msg.info.height, msg.info.width)

        resolution = msg.info.resolution
        origin     = (msg.info.origin.position.x, msg.info.origin.position.y)

        n_free = int(np.sum(grid == FREE))
        self.get_logger().info(f'/map reçu : {msg.info.width}×{msg.info.height}, '
                               f'{n_free} cellules libres')

        if n_free < 100:
            self.get_logger().warn('Carte trop vide, attente...')
            return

        # ── ÉTAPE 1 : frontières ──
        data = compute_boundary_map(grid)

	binary = (grid != FREE).astype(np.uint8)
	dist_tr = data['dist_transform']

        # ── Start : position actuelle du robot ──
        pos = self.get_robot_position()
        if pos is None:
            return
        start = (int((pos[0] - origin[0]) / resolution),
                 int((pos[1] - origin[1]) / resolution))

        # ── Goal : centre géométrique opposé de la carte ──
        # À remplacer par un point cliqué dans rviz2 si besoin
        H, W = grid.shape
        goal = (W // 2, H // 2)

        self.get_logger().info(f'A* : start={start}  goal={goal}')

        # ── ÉTAPE 2 : A* ──
        path_grid = find_path(binary, start, goal, dist_tr)

        if not path_grid:
            self.get_logger().error('A* : aucun chemin trouvé')
            return

        self.get_logger().info(f'A* : {len(path_grid)} waypoints trouvés')

        # ── Publication /planned_path ──
        path_msg              = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp    = self.get_clock().now().to_msg()

        for col, row in path_grid:
            p = PoseStamped()
            p.header             = path_msg.header
            p.pose.position.x    = origin[0] + (col + 0.5) * resolution
            p.pose.position.y    = origin[1] + (row + 0.5) * resolution
            p.pose.orientation.w = 1.0
            path_msg.poses.append(p)

        self.path_pub.publish(path_msg)
        self.planned = True
        self.get_logger().info(
            f'✓ /planned_path publié ({len(path_msg.poses)} points) '
            f'– visualisable dans rviz2')


def main():
    rclpy.init()
    node = AStarPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
