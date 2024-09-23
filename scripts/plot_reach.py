import argparse
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches

def plot_states_and_rects(states_df, rects_df, save_path=None, title=None):
    fig, ax = plt.subplots()
    # ax.set_xlim(rects_df.iloc[-1]['min0'], rects_df.iloc[-1]['max0'])
    # ax.set_ylim(rects_df.iloc[-1]['min1'], rects_df.iloc[-1]['max1'])
    ax.set_xlim(0, 3.2)
    ax.set_ylim(0, 2.2)
    ax.set_aspect('equal')
    ax.set_xlabel('x(m)')
    ax.set_ylabel('y(m)')
    if title is not None:
        ax.set_title(title)
    # Plot Total Hull
    # ax.add_patch(patches.Rectangle(
    #     (rects_df.iloc[-1]['min0'], rects_df.iloc[-1]['min1']),
    #     rects_df.iloc[-1]['max0'] - rects_df.iloc[-1]['min0'],
    #     rects_df.iloc[-1]['max1'] - rects_df.iloc[-1]['min1'],
    #     edgecolor='none',
    #     facecolor='red',
    #     alpha=0.1
    # ))
    ax.add_patch(patches.Circle(
        (2.25, 0.75),
        0.4,
        edgecolor='black',
        facecolor='blue',
        alpha=0.5
    ))
    ax.add_patch(patches.Circle(
        (0.75, 1.5),
        0.4,
        edgecolor='black',
        facecolor='blue',
        alpha=0.5
    ))
    # Plot Intermediate Hulls
    for _, row in rects_df.iloc[:-2].iterrows():
        min_0 = row['min0']
        min_1 = row['min1']
        max_0 = row['max0']
        max_1 = row['max1']
        ax.add_patch(patches.Rectangle(
            (min_0, min_1),
            max_0 - min_0,
            max_1 - min_1,
            edgecolor='none',
            facecolor='#FFB6C1',
            alpha=0.5
        ))
    for _, row in states_df.iloc[:-2].iterrows():
        min_0 = row['min0']
        min_1 = row['min1']
        max_0 = row['max0']
        max_1 = row['max1']
        ax.add_patch(patches.Rectangle(
            (min_0, min_1),
            max_0 - min_0,
            max_1 - min_1,
            edgecolor='none',
            facecolor='lightgreen',
            alpha=0.5
        ))
    ax.plot([0, 3], [0, 2], color='black', linestyle='--')
    ax.plot([1.5], [1], color='green', marker='o', markersize=10)
    ax.plot([2.25], [1.5], color='red', marker='o', markersize=10)
    ax.plot([3], [2], color='gold', marker='*', markersize=25)
    if save_path is not None:
        fig.savefig(save_path)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-r1", "--rect1_csv_file", type=str)
    parser.add_argument("-r2", "--rects2_csv_file", type=str)
    parser.add_argument("-o", "--output_path", default=None, type=str)
    args = parser.parse_args()

    reach_time = 2
    states_df = pd.read_csv(args.rect1_csv_file)
    rects_df = pd.read_csv(args.rects2_csv_file)
    title = f'Reachtube Subgoal Search, reachtime={reach_time} s'
    plot_states_and_rects(states_df, rects_df, save_path=args.output_path, title=title)
    