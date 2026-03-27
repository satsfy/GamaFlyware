import json
import time
import matplotlib.pyplot as plt


def load_point_cloud(json_path):
    with open(json_path, "r") as f:
        return json.load(f)


plt.ion()
fig, (ax_hist, ax_scatter) = plt.subplots(1, 2, figsize=(12, 5))
thre = 0.7
try:
    while True:
        try:
            points = load_point_cloud("point_cloud.json")
            xs = [p["x"] for p in points]
            ys = [p["y"] for p in points]
            zs = [p["z"] for p in points]

            # Z histogram
            ax_hist.clear()
            ax_hist.hist(zs, bins=30, range=(0, 3), edgecolor="black")
            ax_hist.set_xlabel("Z Height")
            ax_hist.set_ylabel("Frequency")
            ax_hist.set_title("Histogram of Z Heights")
            ax_hist.set_xlim(0, 3)

            # XY scatter for Z > 1 and within x, y ranges
            ax_scatter.clear()
            filtered = [p for p in points if p["z"] > thre]
            x_sel = [p["x"] for p in filtered]
            y_sel = [p["y"] for p in filtered]
            ax_scatter.scatter(x_sel, y_sel, s=5, c="red")
            ax_scatter.set_xlabel("X")
            ax_scatter.set_ylabel("Y")
            ax_scatter.set_title(f"XY Scatter (Z > {thre})")
            # ax_scatter.set_xlim(-1.5, 1.5)
            # ax_scatter.set_ylim(-1.0, 1.0)

            fig.tight_layout()
            fig.canvas.draw()
            fig.canvas.flush_events()
            time.sleep(1)
        except Exception as e:
            print(f"Error loading or processing point cloud: {e}")
            time.sleep(1)

except KeyboardInterrupt:
    plt.close(fig)
