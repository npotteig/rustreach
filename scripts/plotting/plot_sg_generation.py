import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.transforms as transforms
from matplotlib import rc
import numpy as np

rc('font', **{'family': 'serif', 'serif': ['Times New Roman']})
rc('text', usetex=True)
plt.rcParams['font.size'] = 24

def plot_sg_select(save_path=None):
    idx = 15
    fig, ax = plt.subplots(figsize=(10, 8))
    # ax.set_xlim(rects_df.iloc[-1]['min0'], rects_df.iloc[-1]['max0'])
    # ax.set_ylim(rects_df.iloc[-1]['min1'], rects_df.iloc[-1]['max1'])
    ax.set_xlim(-0.2, 4.2)
    ax.set_ylim(-1, 1)
    ax.tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)
    ax.set_aspect('equal')
    # ax.set_xlabel('x(m)')
    # ax.set_ylabel('y(m)')
    
    min_0 = 1.75 - 0.25 / 2
    min_1 = 0.75 - 0.15 / 2
    max_0 = 1.75 + 0.25 / 2
    max_1 = 0.75 + 0.15 / 2
    rect = patches.Rectangle(
                (min_0, min_1),
                max_0 - min_0,
                max_1 - min_1,
                # angle=np.degrees(row['dim3']),
                edgecolor='none',
                facecolor='green',
                alpha=0.5
            )
    ax.add_patch(rect)
    ax.plot(1.75, 0.75, color='black', marker='o', markersize=7)
    ax.text(1.75+0.05, 0.75 - 0.17, r"$x_t$", fontsize=20, color='black', fontweight='bold')
    
    ax.plot([0, 4], [0, 0], color='black', linestyle='--')
    # ax.plot([2], [0], color='blue', marker='o', markersize=10)
    # ax.text(2 - 0.5, 0 - 0.2, "SubGoal", fontsize=18, color='black', fontweight='bold')
    ax.plot([4], [0], color='green', marker='o', markersize=25)
    ax.plot([0], [0], color='green', marker='o', markersize=25)
    
    
    
    ax.text(4-0.1, 0 - 0.5, r"$W_j$", fontsize=24, color='black', fontweight='bold')
    ax.text(0-0.1, 0 - 0.5, r"$W_i$", fontsize=24, color='black', fontweight='bold')
    
    sg_cand_line_xs = [1.4, 3]
    sg_cand_line_ys = [0.0, 0.0]
    sg_dx = (sg_cand_line_xs[1] - sg_cand_line_xs[0]) / 4
    ax.plot(sg_cand_line_xs, sg_cand_line_ys, color='gray', linewidth=6)
    ax.plot([sg_cand_line_xs[0], sg_cand_line_xs[0]], [sg_cand_line_ys[0] - 0.1, sg_cand_line_ys[0] + 0.1], color='gray', linewidth=6)
    ax.text(sg_cand_line_xs[0] - 0.1, sg_cand_line_ys[0] - 0.25, r"$e_{prev}$", fontsize=20, color='black', fontweight='bold')
    ax.plot([sg_cand_line_xs[1], sg_cand_line_xs[1]], [sg_cand_line_ys[1] - 0.1, sg_cand_line_ys[1] + 0.1], color='gray', linewidth=6)
    ax.text(sg_cand_line_xs[1] - 0.1, sg_cand_line_ys[1] - 0.25, r"$e_{next}$", fontsize=20, color='black', fontweight='bold')
    
    
    for i in range(5):
        ax.plot(sg_cand_line_xs[0] + i * sg_dx, sg_cand_line_ys[0], color='blue', marker='o', markersize=10, alpha=0.5)
        # ax.text(sg_cand_line_xs[0] + i * sg_dx, sg_cand_line_ys[0] - 0.2, r"$g_{}$".format(i), fontsize=18, color='black', fontweight='bold')
    
    ax.plot([1.75, 1.75], [0.75, 0.0], color='black', linestyle=':')
    ax.plot(1.75, 0.0, color='black', marker='o', markersize=7)
    ax.text(1.75, 0.0 - 0.17, r"$proj_d(x_t)$", fontsize=20, color='black', fontweight='bold')
    if save_path is not None:
        plt.savefig(save_path, bbox_inches='tight')

if __name__ == '__main__':
    plot_sg_select(save_path='figs/paper/sg_generation.pdf')