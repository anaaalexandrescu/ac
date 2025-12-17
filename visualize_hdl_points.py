import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm

# ============================================================
# SETARI GLOBALE PENTRU GRAFIC (CULORI, STIL ETC)
# ============================================================
plt.style.use("seaborn-v0_8-darkgrid")
plt.rcParams["figure.figsize"] = (12, 10)
plt.rcParams["font.size"] = 12


# ------------------------------------------------------------
# DEMO 1: Euclidean vs DBSCAN
# ------------------------------------------------------------
def demo1_euclidean_vs_dbscan():
    np.random.seed(0)

    # cluster circular
    theta = np.linspace(0, 2*np.pi, 40)
    circle = np.column_stack([10*np.cos(theta), 10*np.sin(theta)])
    
    # cluster liniar
    line = np.column_stack([np.linspace(100, 180, 20), np.ones(20)*80])
    
    # zgomot
    noise = np.array([[150, -90], [-90, 150], [0, -80]])

    plt.subplot(2, 2, 1)
    plt.title("Demo 1: Euclidean vs DBSCAN (57 puncte)", fontsize=14, fontweight="bold")

    plt.scatter(circle[:,0], circle[:,1], color="red", label="Cluster 0 (cerc)", s=60)
    plt.scatter(line[:,0], line[:,1], color="blue", label="Cluster 1 (linie)", s=60)
    plt.scatter(noise[:,0], noise[:,1], color="black", marker="x", s=80, label="Zgomot")

    plt.legend()
    plt.xlabel("X")
    plt.ylabel("Y")


# ------------------------------------------------------------
# DEMO 2: Paralelism
# ------------------------------------------------------------
def demo2_parallel_points():
    # puncte distribuite paralel
    pts_x = []
    pts_y = []

    for i in range(0, 200):
        pts_x.append(i * 0.6 - 100)
        pts_y.append((i % 20) * 5 - 50)

    plt.subplot(2, 2, 2)
    plt.title("Demo 2: Paralelism (primele 200/1000 puncte)", fontsize=14, fontweight="bold")

    plt.scatter(pts_x, pts_y, s=40, color="#00aa55", label="Stream de puncte")
    plt.legend()
    plt.xlabel("X")
    plt.ylabel("Y")


# ------------------------------------------------------------
# DEMO 3: Scalabilitate
# ------------------------------------------------------------
def demo3_scalability():
    clusters = {
        0: (0, 0, "red"),
        1: (50, 40, "blue"),
        2: (100, 80, "green"),
        3: (150, 120, "orange"),
        4: (200, 160, "purple"),
    }

    plt.subplot(2, 2, 3)
    plt.title("Demo 3: Scalabilitate (100 puncte, 5 clustere)", fontsize=14, fontweight="bold")

    for cid, (cx, cy, color) in clusters.items():
        x = np.linspace(cx, cx + 25, 20)
        y = np.ones_like(x) * cy
        plt.scatter(x, y, s=60, label=f"Cluster {cid}", color=color)

    plt.legend()
    plt.xlabel("X")
    plt.ylabel("Y")


# ------------------------------------------------------------
# DEMO 4: Fixed-Point Precizie
# ------------------------------------------------------------
def demo4_fixed_point():
    # valori exemplu pentru conversie
    original = np.array([0.1, 0.5, 1.2, 2.7, 6.0, 128.0])
    fixed_point = np.round(original * 256) / 256  # simulăm format 8.8

    plt.subplot(2, 2, 4)
    plt.title("Demo 4: Precizie Fixed-Point", fontsize=14, fontweight="bold")

    indices = np.arange(len(original))

    plt.bar(indices - 0.15, original, width=0.3, color="crimson", label="Valoare originală")
    plt.bar(indices + 0.15, fixed_point, width=0.3, color="darkcyan", label="După conversie")

    plt.xticks(indices, [str(v) for v in original])
    plt.legend()
    plt.xlabel("Valorile")
    plt.ylabel("Magnitudine")


# ------------------------------------------------------------
# MASTER FUNCTION — generează toate graficele într-o singură imagine
# ------------------------------------------------------------
def main():
    plt.figure()

    demo1_euclidean_vs_dbscan()
    demo2_parallel_points()
    demo3_scalability()
    demo4_fixed_point()

    plt.tight_layout()

    # salvăm imaginea la calitate înaltă
    plt.savefig("hdl_demos_visualization.png", dpi=220)
    print("[✔] Imagine generată: hdl_demos_visualization.png")

    plt.show()


if __name__ == "__main__":
    main()
