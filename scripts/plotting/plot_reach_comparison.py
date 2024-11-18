import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.transforms as transforms
from matplotlib import rc
import numpy as np

rc('font', **{'family': 'serif', 'serif': ['Times New Roman']})
rc('text', usetex=True)
plt.rcParams['font.size'] = 24

def plot_reach(states_df, rects_fc_df, rects_dc_df, save_path=None):
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_xlim(-0.1, 1.1)
    ax.set_ylim(-0.1, 1.1)
    ax.set_aspect('equal')
    ax.set_xlabel('x(m)')
    ax.set_ylabel('y(m)')
    
    for i, row in rects_dc_df.iloc[:-1].iterrows():
        min_0 = row['min0']
        min_1 = row['min1']
        max_0 = row['max0']
        max_1 = row['max1']
        if i == len(rects_dc_df) - 2:
            ax.add_patch(patches.Rectangle(
                (min_0, min_1),
                max_0 - min_0,
                max_1 - min_1,
                edgecolor='none',
                facecolor='lightgreen',
                alpha=0.75,
                label='RLC Reach'
            ))
        else:
            ax.add_patch(patches.Rectangle(
                (min_0, min_1),
                max_0 - min_0,
                max_1 - min_1,
                edgecolor='none',
                facecolor='lightgreen',
                alpha=0.75
            ))
    
    for i, row in rects_fc_df.iloc[:-1].iterrows():
        min_0 = row['min0']
        min_1 = row['min1']
        max_0 = row['max0']
        max_1 = row['max1']
        if i == len(rects_fc_df) - 2:
            ax.add_patch(patches.Rectangle(
                (min_0, min_1),
                max_0 - min_0,
                max_1 - min_1,
                edgecolor='none',
                facecolor='orange',
                alpha=0.3,
                label='FC Reach'
            ))
        else:
            ax.add_patch(patches.Rectangle(
                (min_0, min_1),
                max_0 - min_0,
                max_1 - min_1,
                edgecolor='none',
                facecolor='orange',
                alpha=0.3
            ))
    
    ax.plot([1], [1], color='gold', marker='*', markersize=50)
    ax.plot(states_df['dim0'], states_df['dim1'], color='black', linewidth=10, label='Trajectory')
    ax.text(0.92, 0.92, 'Goal', fontsize=24, ha='center', va='center')
    ax.legend()
    
    if save_path is not None:
        plt.savefig(save_path, bbox_inches='tight')

if __name__ == '__main__':
    states_df = pd.read_csv('data/bicycle/paper/approach/reach_comparision/gt_ctrl_states.csv')
    rects_fc_df = pd.read_csv('data/bicycle/paper/approach/reach_comparision/rects_fc.csv')
    rects_dc_df = pd.read_csv('data/bicycle/paper/approach/reach_comparision/rects_dc.csv')
    plot_reach(states_df, rects_fc_df, rects_dc_df, save_path='figs/paper/reach_comparison.pdf')
    