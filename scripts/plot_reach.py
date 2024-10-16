import argparse
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches

def plot_states_and_rects(states_df=None, rects_df=None, save_path=None, title=None):
    fig, ax = plt.subplots()
    # ax.set_xlim(rects_df.iloc[-1]['min0'], rects_df.iloc[-1]['max0'])
    # ax.set_ylim(rects_df.iloc[-1]['min1'], rects_df.iloc[-1]['max1'])
    ax.set_xlim(0, 1.1)
    ax.set_ylim(0, 1.1)
    ax.set_aspect('equal')
    ax.set_xlabel('x(m)')
    ax.set_ylabel('y(m)')
    if title is not None:
        ax.set_title(title)
    # Plot Total Hull
    # ax.add_patch(patches.Rectangle(
    #     (0, 0),
    #     rects_df.iloc[-1]['max0'],
    #     rects_df.iloc[-1]['max1'],
    #     edgecolor='none',
    #     facecolor='red',
    #     alpha=0.1
    # ))
    # ax.add_patch(patches.Circle(
    #     (2.25, 0.75),
    #     0.4,
    #     edgecolor='black',
    #     facecolor='blue',
    #     alpha=0.5
    # ))
    # ax.add_patch(patches.Circle(
    #     (0.75, 1.5),
    #     0.4,
    #     edgecolor='black',
    #     facecolor='blue',
    #     alpha=0.5
    # ))
    # Plot Intermediate Hulls
    if rects_df is not None:
        for _, row in rects_df.iloc[:-1].iterrows():
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
    # for _, row in states_df.iloc.iterrows():
    #     min_0 = row['min0']
    #     min_1 = row['min1']
    #     max_0 = row['max0']
    #     max_1 = row['max1']
    #     ax.add_patch(patches.Rectangle(
    #         (min_0, min_1),
    #         max_0 - min_0,
    #         max_1 - min_1,
    #         edgecolor='none',
    #         facecolor='lightgreen',
    #         alpha=0.5
    #     ))
    # ax.plot([0, 3], [0, 2], color='black', linestyle='--')
    # ax.plot([1.5], [1], color='green', marker='o', markersize=10)
    # ax.plot([2.25], [1.5], color='red', marker='o', markersize=10)
    ax.plot([1], [1], color='gold', marker='*', markersize=25)
    if states_df is not None:
        ax.plot(states_df['dim0'], states_df['dim1'], color='purple', marker='o', markersize=5)
    if save_path is not None:
        fig.savefig(save_path)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--states_csv_file", default=None, type=str)
    parser.add_argument("-r", "--rects_csv_file", default=None, type=str)
    parser.add_argument("-o", "--output_path", default=None, type=str)
    args = parser.parse_args()

    reach_time = 2
    states_df = pd.read_csv(args.states_csv_file) if args.states_csv_file is not None else None
    rects_df = pd.read_csv(args.rects_csv_file) if args.rects_csv_file is not None else None
    title = f'Reachtube Dynamic Control, reachtime={reach_time} s'
    plot_states_and_rects(states_df, rects_df, save_path=args.output_path, title=title)
    