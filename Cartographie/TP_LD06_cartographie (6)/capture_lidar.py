#!/usr/bin/env python3
"""
Capture de scans LD06 depuis l'ESP32 (firmware ESP32_LD06_Capture.ino).

Chaque capture est enregistrée dans un fichier CSV en coordonnées polaires :
    angle_deg,distance_mm,intensity

Usage :
    pip install pyserial matplotlib
    python capture_lidar.py --port COM5            (Windows)
    python capture_lidar.py --port /dev/ttyUSB0    (Linux)
    python capture_lidar.py --port COM5 --tours 3 --preview

À chaque position du robot :
  1. Poser le robot, ne plus le toucher.
  2. Taper un nom (ex. "pos1") puis Entrée -> capture.
  3. (Optionnel) saisir la pose mesurée au mètre ruban "x y theta"
     -> enregistrée dans poses_initiales.json, importable dans l'outil.
"""
import argparse
import json
import os
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial manquant : pip install pyserial")


def read_one_scan(ser, timeout=6.0):
    """Lit un bloc #SCAN ... #END et renvoie la liste (angle, distance, intensité)."""
    pts, inside = [], False
    t0 = time.time()
    while time.time() - t0 < timeout:
        line = ser.readline().decode("ascii", errors="ignore").strip()
        if not line:
            continue
        if line.startswith("#SCAN"):
            inside, pts = True, []
            continue
        if line.startswith("#END"):
            if inside:
                return pts
            continue
        if inside:
            parts = line.split(",")
            if len(parts) >= 3:
                try:
                    pts.append((float(parts[0]), int(parts[1]), int(parts[2])))
                except ValueError:
                    pass  # ligne corrompue : ignorée
    raise TimeoutError("Pas de scan reçu (vérifier port, baudrate, alimentation du lidar)")


def capture(ser, tours):
    ser.reset_input_buffer()
    ser.write(str(tours).encode() if tours > 1 else b"s")
    allpts = []
    for k in range(tours):
        pts = read_one_scan(ser)
        print(f"   tour {k + 1}/{tours} : {len(pts)} points")
        allpts.extend(pts)
    return allpts


def save_csv(path, pts):
    with open(path, "w", encoding="utf-8") as f:
        f.write("angle_deg,distance_mm,intensity\n")
        for a, d, i in pts:
            f.write(f"{a:.2f},{d},{i}\n")


def preview(pts, title):
    import math
    import matplotlib.pyplot as plt
    xs = [d / 1000 * math.cos(math.radians(a)) for a, d, _ in pts]  # angles trigo (librairie)
    ys = [d / 1000 * math.sin(math.radians(a)) for a, d, _ in pts]
    plt.figure(title)
    plt.scatter(xs, ys, s=2)
    plt.plot(0, 0, "r^")
    plt.axis("equal"); plt.grid(True); plt.title(title)
    plt.xlabel("x (m)"); plt.ylabel("y (m)")
    plt.show()


def main():
    ap = argparse.ArgumentParser(description="Capture de scans LD06")
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--out", default="scans", help="dossier de sortie")
    ap.add_argument("--tours", type=int, default=1,
                    help="nombre de tours fusionnés par capture (1-9) pour densifier")
    ap.add_argument("--preview", action="store_true", help="affiche chaque scan (matplotlib)")
    args = ap.parse_args()
    args.tours = max(1, min(9, args.tours))

    os.makedirs(args.out, exist_ok=True)
    poses_path = os.path.join(args.out, "poses_initiales.json")
    poses = {"scans": []}
    if os.path.exists(poses_path):
        with open(poses_path, encoding="utf-8") as f:
            poses = json.load(f)

    ser = serial.Serial(args.port, args.baud, timeout=0.5)
    time.sleep(2.0)  # l'ESP32 redémarre à l'ouverture du port
    ser.reset_input_buffer()
    print(f"Connecté sur {args.port}. Fichiers -> {os.path.abspath(args.out)}")

    n = len([f for f in os.listdir(args.out) if f.endswith(".csv")])
    while True:
        default = f"scan_{n + 1:02d}"
        try:
            name = input(f"\nNom du scan [{default}] (q = quitter) : ").strip() or default
        except (EOFError, KeyboardInterrupt):
            break
        if name.lower() == "q":
            break
        try:
            pts = capture(ser, args.tours)
        except TimeoutError as e:
            print("  !!", e)
            continue
        path = os.path.join(args.out, name + ".csv")
        save_csv(path, pts)
        n += 1
        print(f"  -> {path} ({len(pts)} points)")

        pose = input("  Pose mesurée 'x y theta' en m, m, ° (Entrée pour ignorer) : ").strip()
        if pose:
            try:
                x, y, th = map(float, pose.replace(",", " ").split())
                poses["scans"] = [p for p in poses["scans"] if p["name"] != name]
                poses["scans"].append({"name": name, "x": x, "y": y, "theta_deg": th})
                with open(poses_path, "w", encoding="utf-8") as f:
                    json.dump(poses, f, indent=2)
            except ValueError:
                print("  format invalide, pose ignorée")

        if args.preview:
            preview(pts, name)

    ser.close()


if __name__ == "__main__":
    main()
