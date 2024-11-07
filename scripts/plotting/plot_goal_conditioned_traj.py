import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.transforms as transforms
from matplotlib import rc
import numpy as np

rc('font', **{'family': 'serif', 'serif': ['Times New Roman']})
rc('text', usetex=True)
plt.rcParams['font.size'] = 24

def plot_states(states_df, states2_df=None, save_path=None, title=None):
    fig, ax = plt.subplots(figsize=(10, 8))
    # ax.set_xlim(rects_df.iloc[-1]['min0'], rects_df.iloc[-1]['max0'])
    # ax.set_ylim(rects_df.iloc[-1]['min1'], rects_df.iloc[-1]['max1'])
    ax.set_xlim(-0.2, 4.2)
    ax.set_ylim(-1, 1)
    ax.set_aspect('equal')
    ax.set_xlabel('x(m)')
    ax.set_ylabel('y(m)')
    if title is not None:
        ax.set_title(title)
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
    
    idxes = list(range(0, len(states_df), 4))
    idxes.append(len(states_df) - 1)
    for i, idx in enumerate(idxes):
        row = states_df.iloc[idx]
        min_0 = row['dim0'] - 0.25
        min_1 = row['dim1'] - 0.15
        max_0 = row['dim0'] + 0.25
        max_1 = row['dim1'] + 0.15
        if i == len(idxes) - 1:
            rect = patches.Rectangle(
                (min_0, min_1),
                max_0 - min_0,
                max_1 - min_1,
                # angle=np.degrees(row['dim3']),
                edgecolor='none',
                facecolor='green',
                alpha=0.5,
                label='w/ SG'
            )
        else:
            rect = patches.Rectangle(
                (min_0, min_1),
                max_0 - min_0,
                max_1 - min_1,
                # angle=np.degrees(row['dim3']),
                edgecolor='none',
                facecolor='green',
                alpha=0.5
            )
        rotation = transforms.Affine2D().rotate_deg_around(row['dim0'], row['dim1'], np.degrees(row['dim3']))
        
        rect.set_transform(rotation + ax.transData)
        ax.add_patch(rect)
        ax.plot(row['dim0'], row['dim1'], color='black', marker='o', markersize=5)
    
    if states2_df is not None:
        idxes = list(range(0, len(states2_df), 4))
        idxes.append(len(states2_df) - 1)
        for i, idx in enumerate(idxes):
            row = states2_df.iloc[idx]
            min_0 = row['dim0'] - 0.25
            min_1 = row['dim1'] - 0.15
            max_0 = row['dim0'] + 0.25
            max_1 = row['dim1'] + 0.15
            if i == len(idxes) - 1:
                rect = patches.Rectangle(
                    (min_0, min_1),
                    max_0 - min_0,
                    max_1 - min_1,
                    # angle=np.degrees(row['dim3']),
                    edgecolor='none',
                    facecolor='red',
                    alpha=0.5,
                    label='w/o SG'
                )
            else:
                rect = patches.Rectangle(
                    (min_0, min_1),
                    max_0 - min_0,
                    max_1 - min_1,
                    # angle=np.degrees(row['dim3']),
                    edgecolor='none',
                    facecolor='red',
                    alpha=0.5
                )
            rotation = transforms.Affine2D().rotate_deg_around(row['dim0'], row['dim1'], np.degrees(row['dim3']))
            
            rect.set_transform(rotation + ax.transData)
            ax.add_patch(rect)
            ax.plot(row['dim0'], row['dim1'], color='black', marker='o', markersize=5)
    ax.plot([0, 4], [0, 0], color='black', linestyle='--')
    ax.plot([2], [0], color='blue', marker='o', markersize=10)
    ax.text(2 - 0.5, 0 - 0.2, "SubGoal", fontsize=18, color='black', fontweight='bold')
    ax.plot([4], [0], color='green', marker='o', markersize=25)
    ax.text(4-0.1, 0 - 0.5, r"$W_j$", fontsize=24, color='black', fontweight='bold')
    ax.plot([0], [0], color='green', marker='o', markersize=25)
    ax.text(0-0.1, 0 - 0.5, r"$W_i$", fontsize=24, color='black', fontweight='bold')
    ax.legend()
    if save_path is not None:
        fig.savefig(save_path, bbox_inches='tight')

if __name__ == '__main__':
    states_df = pd.read_csv('data/bicycle/paper/formulation/ctrl_states_sg.csv')
    states2_df = pd.read_csv('data/bicycle/paper/formulation/ctrl_states_coll.csv')
    plot_states(states_df, states2_df, save_path='figs/paper/bicycle_trajs.pdf')