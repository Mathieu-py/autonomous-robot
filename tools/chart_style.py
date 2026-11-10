"""Colours, styling and the log reader shared by the plotting tools."""
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
    "axes.edgecolor": GRID,
    "axes.labelcolor": INK_2,
    "xtick.color": INK_2,
    "ytick.color": INK_2,
    "axes.titlecolor": INK,
})


def style_axes(ax, grid_axis="y"):
    ax.set_facecolor(SURFACE)
    ax.grid(axis=grid_axis, color=GRID, linewidth=0.8)
    ax.spines[["top", "right"]].set_visible(False)


def read_log_rows(path, column_count, text_columns=0):
    """The firmware's CSV rows, also from a PuTTY log with other lines mixed in.
    The last text_columns fields stay text; the rest are parsed as floats."""
    rows = []
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            parts = line.strip().split(",")
            if len(parts) != column_count:
                continue
            numbers = parts[:column_count - text_columns]
            try:
                rows.append([float(p) for p in numbers]
                            + parts[len(numbers):])
            except ValueError:
                continue
    return rows
