"""Create a compact dimensioned preview for the mount package."""

from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyBboxPatch, RegularPolygon


OUT = Path(__file__).resolve().parent / "ZED2i_Tripod_Mount_preview.png"


def setup_view(ax, title):
    ax.set_aspect("equal")
    ax.set_xlim(-47, 47)
    ax.set_ylim(-27, 27)
    ax.axis("off")
    ax.set_title(title, fontsize=14, fontweight="bold", color="#162238", pad=12)
    ax.add_patch(
        FancyBboxPatch(
            (-40, -17.5), 80, 35,
            boxstyle="round,pad=0,rounding_size=4",
            facecolor="#d9e5f3", edgecolor="#173b67", linewidth=2.2,
        )
    )


fig, axes = plt.subplots(1, 2, figsize=(12, 4.5), facecolor="#f7f9fc")
fig.suptitle("ZED 2i — 1/4-20 Tripod Adaptör Plakası", fontsize=18,
             fontweight="bold", color="#10243e", y=0.98)

setup_view(axes[0], "Üst yüz — kamera tarafı")
for x in (-18, 18):
    axes[0].add_patch(Circle((x, 0), 1.7, fill=False, edgecolor="#d1495b", linewidth=2))
axes[0].add_patch(Circle((0, 0), 3.5, fill=False, edgecolor="#245b9b", linewidth=1.7))
axes[0].add_patch(RegularPolygon((0, 0), 6, radius=11.3 / 3**0.5,
                                 orientation=0, fill=False,
                                 edgecolor="#245b9b", linewidth=2.4))
axes[0].annotate("36 mm M3 aralığı", xy=(0, 3), xytext=(0, 10), ha="center",
                 arrowprops=dict(arrowstyle="-[,widthB=4.2", lw=1.2, color="#d1495b"),
                 fontsize=10, color="#8e2232")
axes[0].text(0, -11, "1/4-20 somun yuvası\n11.3 AF × 5.8 derin", ha="center",
             va="center", fontsize=9, color="#173b67")

setup_view(axes[1], "Alt yüz — tripod tarafı")
for x in (-18, 18):
    axes[1].add_patch(Circle((x, 0), 3.25, facecolor="#f7f9fc",
                             edgecolor="#d1495b", linewidth=2))
    axes[1].add_patch(Circle((x, 0), 1.7, fill=False,
                             edgecolor="#d1495b", linewidth=1.2))
axes[1].add_patch(Circle((0, 0), 3.5, facecolor="#f7f9fc",
                         edgecolor="#245b9b", linewidth=2.4))
axes[1].text(0, -10.5, "Ø7 mm tripod geçişi", ha="center", fontsize=10,
             color="#173b67")
axes[1].text(-18, 7, "Ø6.5 × 3.2\nM3 baş yuvası", ha="center", fontsize=9,
             color="#8e2232")

for ax in axes:
    ax.annotate("", xy=(-40, -22), xytext=(40, -22),
                arrowprops=dict(arrowstyle="<->", color="#51647b", lw=1.2))
    ax.text(0, -25, "80 mm", ha="center", fontsize=10, color="#354b65")

fig.text(0.5, 0.045, "Gövde: 80 × 35 × 8 mm  |  Köşe R4  |  Önerilen baskı: PETG/ASA",
         ha="center", fontsize=11, color="#354b65")
plt.tight_layout(rect=(0.02, 0.08, 0.98, 0.93))
fig.savefig(OUT, dpi=180, bbox_inches="tight", facecolor=fig.get_facecolor())
print(OUT)
