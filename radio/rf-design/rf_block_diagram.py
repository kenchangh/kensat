#!/usr/bin/env python3
"""Generate RF architecture block diagram as PNG."""

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as patches

OUTPUT = "/Users/kenchan/Documents/kensat_radio/src/rf_architecture.png"


def draw():
    fig, ax = plt.subplots(1, 1, figsize=(7, 3.5), dpi=200)
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 5)
    ax.set_aspect("equal")
    ax.axis("off")

    box_kw = dict(linewidth=1.2, edgecolor="#222", facecolor="#e8edf2")
    label_kw = dict(ha="center", va="center", fontsize=8, fontweight="bold", family="sans-serif")
    small_kw = dict(ha="center", va="center", fontsize=6.5, family="sans-serif", color="#444")
    arrow_kw = dict(arrowstyle="->, head_width=0.15, head_length=0.1",
                    connectionstyle="arc3", color="#333", lw=1.2)

    # Si4463 box
    si_x, si_y, si_w, si_h = 3.5, 3.0, 3.0, 1.5
    ax.add_patch(patches.FancyBboxPatch((si_x, si_y), si_w, si_h,
                 boxstyle="round,pad=0.1", **box_kw))
    ax.text(si_x + si_w/2, si_y + si_h/2 + 0.2, "Si4463", **label_kw)
    ax.text(si_x + si_w/2, si_y + si_h/2 - 0.2, "UHF Transceiver", **small_kw)

    # TX out label
    ax.text(si_x + 0.5, si_y - 0.15, "TX out", fontsize=6, ha="center", color="#555")
    # RX in label
    ax.text(si_x + si_w - 0.5, si_y - 0.15, "RX in", fontsize=6, ha="center", color="#555")

    # TX Match Network box
    tx_x, tx_y, tx_w, tx_h = 2.5, 1.4, 1.8, 0.7
    ax.add_patch(patches.FancyBboxPatch((tx_x, tx_y), tx_w, tx_h,
                 boxstyle="round,pad=0.08", linewidth=1, edgecolor="#555", facecolor="#f0f4e8"))
    ax.text(tx_x + tx_w/2, tx_y + tx_h/2 + 0.1, "TX Match", fontsize=7, ha="center", fontweight="bold")
    ax.text(tx_x + tx_w/2, tx_y + tx_h/2 - 0.18, "Network", fontsize=6.5, ha="center", color="#555")

    # RX Match Network box
    rx_x, rx_y, rx_w, rx_h = 5.7, 1.4, 1.8, 0.7
    ax.add_patch(patches.FancyBboxPatch((rx_x, rx_y), rx_w, rx_h,
                 boxstyle="round,pad=0.08", linewidth=1, edgecolor="#555", facecolor="#f0f4e8"))
    ax.text(rx_x + rx_w/2, rx_y + rx_h/2 + 0.1, "RX Match", fontsize=7, ha="center", fontweight="bold")
    ax.text(rx_x + rx_w/2, rx_y + rx_h/2 - 0.18, "Network", fontsize=6.5, ha="center", color="#555")

    # RF Switch box
    sw_x, sw_y, sw_w, sw_h = 3.5, 0.15, 3.0, 0.7
    ax.add_patch(patches.FancyBboxPatch((sw_x, sw_y), sw_w, sw_h,
                 boxstyle="round,pad=0.08", **box_kw))
    ax.text(sw_x + sw_w/2, sw_y + sw_h/2, "AS179-92LF  SPDT Switch", fontsize=7, ha="center", fontweight="bold")

    # Antenna label + symbol
    ax.annotate("", xy=(sw_x - 0.05, sw_y + sw_h/2), xytext=(1.3, sw_y + sw_h/2),
                arrowprops=dict(arrowstyle="-", color="#333", lw=1.5))
    # Simple antenna icon
    ant_x, ant_y = 1.0, sw_y + sw_h/2
    ax.plot([ant_x, ant_x - 0.3, ant_x + 0.3], [ant_y + 0.5, ant_y, ant_y], color="#333", lw=1.5)
    ax.plot([ant_x, ant_x], [ant_y, ant_y + 0.5], color="#333", lw=1.5)
    ax.text(ant_x, ant_y - 0.3, "Antenna\n(SMA, 50\u03A9)", fontsize=6, ha="center", color="#555")

    # Arrows: Si4463 TX out -> TX Match
    ax.annotate("", xy=(tx_x + tx_w/2, tx_y + tx_h),
                xytext=(si_x + 0.5, si_y),
                arrowprops=arrow_kw)

    # Arrows: Si4463 RX in -> RX Match
    ax.annotate("", xy=(si_x + si_w - 0.5, si_y),
                xytext=(rx_x + rx_w/2, rx_y + rx_h),
                arrowprops=arrow_kw)

    # Arrows: TX Match -> RF Switch
    ax.annotate("", xy=(sw_x + 0.5, sw_y + sw_h),
                xytext=(tx_x + tx_w/2, tx_y),
                arrowprops=arrow_kw)

    # Arrows: RX Match -> RF Switch
    ax.annotate("", xy=(rx_x + rx_w/2, rx_y),
                xytext=(sw_x + sw_w - 0.5, sw_y + sw_h),
                arrowprops=arrow_kw)

    plt.tight_layout(pad=0.3)
    fig.savefig(OUTPUT, bbox_inches="tight", facecolor="white")
    plt.close()
    print(f"Saved: {OUTPUT}")


if __name__ == "__main__":
    draw()
