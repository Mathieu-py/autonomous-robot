"""Colours and styling shared by the plotting tools."""
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_2 = "#52514e"
GRID = "#e4e3df"
STOP_FILL = "#ecebe7"
BLUE = "#2a78d6"
ORANGE = "#eb6834"

plt.rcParams.update({
    "font.size": 10,
    "figure.facecolor": SURFACE,
    "axes.facecolor": SURFACE,
    "axes.edgecolor": GRID,
    "axes.labelcolor": INK_2,
    "axes.titlecolor": INK,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "axes.grid": True,
    "axes.grid.axis": "y",
    "grid.color": GRID,
    "grid.linewidth": 0.8,
    "xtick.color": INK_2,
    "ytick.color": INK_2,
    "legend.frameon": False,
})


def plot_target(ax, t, values, label):
    """The commanded value, as a dashed step."""
    ax.step(t, values, where="post", color=INK_2, linewidth=1.5,
            linestyle=(0, (4, 3)), label=label)


def label_events(ax, events, fontsize):
    """Writes each (time, text) sideways from the top of the axes."""
    for x, text in events:
        ax.annotate(text, (x, 1.0), xycoords=("data", "axes fraction"),
                    xytext=(3, -3), textcoords="offset points", color=INK_2,
                    fontsize=fontsize, va="top", rotation=90)
