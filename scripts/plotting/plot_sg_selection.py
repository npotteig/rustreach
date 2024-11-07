import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.transforms as transforms
from matplotlib import rc
import numpy as np

rc('font', **{'family': 'serif', 'serif': ['Times New Roman']})
rc('text', usetex=True)
plt.rcParams['font.size'] = 24

def plot_sg_select(states_df, sg_df, rects_good_df, rects_bad_df, save_path=None):
    idx = 15
    fig, ax = plt.subplots(figsize=(10, 8))
    # ax.set_xlim(rects_df.iloc[-1]['min0'], rects_df.iloc[-1]['max0'])
    # ax.set_ylim(rects_df.iloc[-1]['min1'], rects_df.iloc[-1]['max1'])
    ax.set_xlim(-0.2, 4.2)
    ax.set_ylim(-1, 1)
    ax.set_aspect('equal')
    ax.set_xlabel('x(m)')
    ax.set_ylabel('y(m)')
    w = 0.5
    h = 0.5
    
    pt_0 = [2, 0.7]
    ax.add_patch(patches.Rectangle(
        (pt_0[0] - w/2, pt_0[1] - h/2),
        w,
        h,
        edgecolor='black',
        facecolor='blue',
        alpha=0.5
    ))
    ax.text(pt_0[0]-0.05, pt_0[1]-0.05, r"$o_0$", fontsize=18, color='black', fontweight='bold')
    
    pt_1 = [2, -0.7]
    ax.add_patch(patches.Rectangle(
        (pt_1[0] - w/2, pt_1[1] - h/2),
        w,
        h,
        edgecolor='black',
        facecolor='blue',
        alpha=0.5
    ))
    ax.text(pt_1[0]-0.05, pt_1[1]-0.05, r"$o_1$", fontsize=18, color='black', fontweight='bold')
    
    for j in range(0, len(rects_bad_df) - 1, 50):
        rt = rects_bad_df.iloc[j]
        min_0 = rt['min0'] - 0.25
        min_1 = rt['min1'] - 0.15
        max_0 = rt['max0'] + 0.25
        max_1 = rt['max1'] + 0.15
        rect = patches.Rectangle(
            (min_0, min_1),
            max_0 - min_0,
            max_1 - min_1,
            edgecolor='none',
            facecolor='red',
            alpha=0.5
        )
        ax.add_patch(rect)
    
    reachtubes = rects_good_df[rects_df['time'] == idx]
    for j in range(0, len(reachtubes) - 1, 50):
        rt = reachtubes.iloc[j]
        min_0 = rt['min0'] - 0.25
        min_1 = rt['min1'] - 0.15
        max_0 = rt['max0'] + 0.25
        max_1 = rt['max1'] + 0.15
        rect = patches.Rectangle(
            (min_0, min_1),
            max_0 - min_0,
            max_1 - min_1,
            edgecolor='none',
            facecolor='lightgreen',
            alpha=0.5
        )
        ax.add_patch(rect)
    
    state = states_df.iloc[idx]
    min_0 = state['dim0'] - 0.25
    min_1 = state['dim1'] - 0.15
    max_0 = state['dim0'] + 0.25
    max_1 = state['dim1'] + 0.15
    rect = patches.Rectangle(
                (min_0, min_1),
                max_0 - min_0,
                max_1 - min_1,
                # angle=np.degrees(row['dim3']),
                edgecolor='none',
                facecolor='green',
                alpha=0.5
            )
    rotation = transforms.Affine2D().rotate_deg_around(state['dim0'], state['dim1'], np.degrees(state['dim3']))
    
    rect.set_transform(rotation + ax.transData)
    ax.add_patch(rect)
    ax.plot(state['dim0'], state['dim1'], color='black', marker='o', markersize=5)
    ax.text(state['dim0'] - 0.1, state['dim1'] - 0.1, r"$s_t$", fontsize=18, color='black', fontweight='bold')
    
    ax.plot([0, 4], [0, 0], color='black', linestyle='--')
    # ax.plot([2], [0], color='blue', marker='o', markersize=10)
    # ax.text(2 - 0.5, 0 - 0.2, "SubGoal", fontsize=18, color='black', fontweight='bold')
    ax.plot([4], [0], color='green', marker='o', markersize=25)
    ax.plot([0], [0], color='green', marker='o', markersize=25)
    
    sg = sg_df.iloc[idx]
    ax.plot(sg['dim0'], sg['dim1'], color='blue', marker='o', markersize=10)
    ax.text(sg['dim0'], sg['dim1'] - 0.2, r"$g_2$", fontsize=18, color='black', fontweight='bold')
    ax.plot([4], [0], color='blue', marker='o', markersize=10)
    ax.text(4, 0 - 0.2, r"$g_4$", fontsize=18, color='black', fontweight='bold')
    ax.plot([3], [0], color='blue', marker='o', markersize=10)
    ax.text(3, 0 - 0.2, r"$g_3$", fontsize=18, color='black', fontweight='bold')
    ax.plot([1], [0], color='blue', marker='o', markersize=10)
    ax.text(1, 0 - 0.2, r"$g_1$", fontsize=18, color='black', fontweight='bold')
    ax.plot([0], [0], color='blue', marker='o', markersize=10)
    ax.text(0, 0 - 0.2, r"$g_0$", fontsize=18, color='black', fontweight='bold')
    
    ax.text(4-0.1, 0 - 0.5, r"$W_j$", fontsize=24, color='black', fontweight='bold')
    ax.text(0-0.1, 0 - 0.5, r"$W_i$", fontsize=24, color='black', fontweight='bold')
    if save_path is not None:
        plt.savefig(save_path, bbox_inches='tight')

if __name__ == '__main__':
    states_df = pd.read_csv('data/bicycle/paper/approach/sg_selection/ctrl_states.csv')
    sg_df = pd.read_csv('data/bicycle/paper/approach/sg_selection/subgoals.csv')
    rects_df = pd.read_csv('data/bicycle/paper/approach/sg_selection/reachtubes.csv')
    rects_other_df = pd.read_csv('data/bicycle/paper/approach/sg_selection/rects_dc.csv')
    plot_sg_select(states_df, sg_df, rects_df, rects_other_df, save_path='figs/paper/sg_select.pdf')