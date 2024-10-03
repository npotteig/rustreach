import argparse
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.transforms as transforms
import numpy as np

def plot_states(states_df, states2_df=None, save_path=None, title=None):
    fig, ax = plt.subplots()
    # ax.set_xlim(rects_df.iloc[-1]['min0'], rects_df.iloc[-1]['max0'])
    # ax.set_ylim(rects_df.iloc[-1]['min1'], rects_df.iloc[-1]['max1'])
    ax.set_xlim(0, 4.2)
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
    pt_1 = [2, -0.7]
    ax.add_patch(patches.Rectangle(
        (pt_1[0] - w/2, pt_1[1] - h/2),
        w,
        h,
        edgecolor='black',
        facecolor='blue',
        alpha=0.5
    ))

    for i in range(0, len(states_df), 4):
        row = states_df.iloc[i]
        min_0 = row['dim0'] - 0.25
        min_1 = row['dim1'] - 0.15
        max_0 = row['dim0'] + 0.25
        max_1 = row['dim1'] + 0.15
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
        for i in idxes:
            row = states2_df.iloc[i]
            min_0 = row['dim0'] - 0.25
            min_1 = row['dim1'] - 0.15
            max_0 = row['dim0'] + 0.25
            max_1 = row['dim1'] + 0.15
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
    ax.plot([1.5], [0], color='blue', marker='o', markersize=10)
    # ax.plot([2.25], [1.5], color='red', marker='o', markersize=10)
    ax.plot([4], [0], color='gold', marker='*', markersize=25)
    if save_path is not None:
        fig.savefig(save_path)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-s1", "--state1_csv_file", type=str)
    parser.add_argument("-s2", "--state2_csv_file", default=None, type=str)
    parser.add_argument("-o", "--output_path", default=None, type=str)
    args = parser.parse_args()

    states1_df = pd.read_csv(args.state1_csv_file)
    states2_df = pd.read_csv(args.state2_csv_file)
    title = f'Safe Trajectory w/ Subgoal'
    plot_states(states1_df, states2_df, save_path=args.output_path, title=title)
    