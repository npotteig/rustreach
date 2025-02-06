from typing import Optional

import argparse
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.transforms as transforms
import numpy as np
import imageio
from tqdm import tqdm

USING_RTREACH = True
USING_QUAD = False

OBSTACLE_SPEED = 0.05
OBSTACLE_OSCILLATION_TIME = 28

def convert_to_rgb_array(states: pd.DataFrame, subgoals: Optional[pd.DataFrame] = None, reachtubes: Optional[pd.DataFrame] = None):
    fig, ax = plt.subplots(figsize=(10, 5))
    w = 0.5
    h = 0.5
    
    frame_data = []
    states_so_far = []
    for i in tqdm(range(len(states))):
        row = states.iloc[i]
        
        ax.set_xlim(-0.2, 4.2)
        ax.set_ylim(-2, 2)
        ax.set_aspect('equal')
        ax.set_xlabel('x(m)')
        ax.set_ylabel('y(m)')
        # ax.set_title("Vehicle Simulation", fontsize=30)
        if i < OBSTACLE_OSCILLATION_TIME:
            offset = OBSTACLE_SPEED * i
        else:
            offset = 0
        pt_0 = [2, 0.7 - offset]
        ax.add_patch(patches.Rectangle(
            (pt_0[0] - w/2, pt_0[1] - h/2),
            w,
            h,
            edgecolor='black',
            facecolor='blue',
            alpha=0.5
        ))
        pt_1 = [2, -0.7 + offset]
        ax.add_patch(patches.Rectangle(
            (pt_1[0] - w/2, pt_1[1] - h/2),
            w,
            h,
            edgecolor='black',
            facecolor='blue',
            alpha=0.5
        ))
        # pt_3 = [2, -1.4]
        # ax.add_patch(patches.Rectangle(
        #     (pt_3[0] - w/2, pt_3[1] - h/2),
        #     w,
        #     h,
        #     edgecolor='black',
        #     facecolor='blue',
        #     alpha=0.5
        # ))
        # pt_4 = [2, 1.4]
        # ax.add_patch(patches.Rectangle(
        #     (pt_4[0] - w/2, pt_4[1] - h/2),
        #     w,
        #     h,
        #     edgecolor='black',
        #     facecolor='blue',
        #     alpha=0.5
        # ))
        if not USING_RTREACH:
            ax.add_patch(patches.Circle(
                pt_0,
                0.25 * np.sqrt(2),
                edgecolor='black',
                facecolor='lightblue',
                alpha=0.5
            ))
            ax.add_patch(patches.Circle(
                pt_1,
                0.25 * np.sqrt(2),
                edgecolor='black',
                facecolor='lightblue',
                alpha=0.5
            ))
        ax.plot([0, 4], [0, 0], color='black', linestyle='--')
        ax.plot([4], [0], color='green', marker='o', markersize=25)
        ax.plot([0], [0], color='green', marker='o', markersize=25)
        
        if subgoals is not None:
            if i == len(states) - 1:
                sg = subgoals.iloc[-1]
            else:
                sg = subgoals.iloc[i]
            ax.plot(sg['dim0'], sg['dim1'], color='blue', marker='o', markersize=10)
        
        if USING_RTREACH and reachtubes is not None and i != len(states) - 1:
            reachtubes_at_i = reachtubes[reachtubes['time'] == i]
            for j in range(0, len(reachtubes_at_i) - 1, 50):
                rt = reachtubes_at_i.iloc[j]
                if not USING_QUAD:
                    min_0 = rt['min0'] - 0.25
                    min_1 = rt['min1'] - 0.15
                    max_0 = rt['max0'] + 0.25
                    max_1 = rt['max1'] + 0.15
                else:
                    min_0 = rt['min0'] - 0.16
                    min_1 = rt['min1'] - 0.16
                    max_0 = rt['max0'] + 0.16
                    max_1 = rt['max1'] + 0.16
                rect = patches.Rectangle(
                    (min_0, min_1),
                    max_0 - min_0,
                    max_1 - min_1,
                    edgecolor='none',
                    facecolor='lightgreen',
                    alpha=0.5
                )
                ax.add_patch(rect)
        elif (not USING_RTREACH) and reachtubes is not None and i != len(states) - 1:
            reachtubes_at_i = reachtubes[reachtubes['time'] == i]
            rt = reachtubes_at_i.iloc[0]
            min_0 = rt['min0']
            min_1 = rt['min1']
            max_0 = rt['max0']
            max_1 = rt['max1']
            center = [(min_0 + max_0) / 2, (min_1 + max_1) / 2]
            radius = (max_0 - min_0) / 2
            circle = patches.Circle(
                center,
                radius,
                edgecolor='none',
                facecolor='lightblue',
                alpha=0.5
            )
            ax.add_patch(circle)
            
        
        np_states_so_far = np.array(states_so_far)
        if len(np_states_so_far) > 0:
            ax.scatter(np_states_so_far[:, 0], np_states_so_far[:, 1], color='grey', marker='o', s=5)
        
        if not USING_QUAD:
            min_0 = row['dim0'] - 0.25
            min_1 = row['dim1'] - 0.15
            max_0 = row['dim0'] + 0.25
            max_1 = row['dim1'] + 0.15
            theta = row['dim3']
        else:
            min_0 = row['dim0'] - 0.16
            min_1 = row['dim1'] - 0.16
            max_0 = row['dim0'] + 0.16
            max_1 = row['dim1'] + 0.16
            theta = row['dim5']
        # facecolor = 'green' if i < len(states) - 1 else 'red'
        facecolor = 'green'
        rect = patches.Rectangle(
            (min_0, min_1),
            max_0 - min_0,
            max_1 - min_1,
            edgecolor='none',
            facecolor=facecolor,
            alpha=0.5
        )
        rotation = transforms.Affine2D().rotate_deg_around(row['dim0'], row['dim1'], np.degrees(theta))
        
        rect.set_transform(rotation + ax.transData)
        ax.add_patch(rect)
        if USING_QUAD:
            triangle = patches.Polygon(
                [[row['dim0'], row['dim1'] + 0.16], [row['dim0'] + 0.16, row['dim1']], [row['dim0'], row['dim1'] - 0.16]],
                edgecolor='black',
                facecolor='orange',
                alpha=0.5
            )
            triangle.set_transform(rotation + ax.transData)
            ax.add_patch(triangle)
        
        ax.plot(row['dim0'], row['dim1'], color='black', marker='o', markersize=5)
        states_so_far.append((row['dim0'], row['dim1']))
        # if i == 26:
        #     fig.savefig('figs/bicycle/subgoal_demo.pdf')
        fig.canvas.draw()
        ax.cla()
        
        data = np.array(fig.canvas.renderer.buffer_rgba(), dtype=np.uint8)
        
        frame_data.append(data)
    return frame_data    

def save_video(states: pd.DataFrame, output_path: str, subgoals: Optional[pd.DataFrame] = None, reachtubes: Optional[pd.DataFrame] = None, fps=10):
    frame_data = convert_to_rgb_array(states, subgoals=subgoals, reachtubes=reachtubes)
    with imageio.get_writer(output_path, fps=fps) as writer:
        for data in tqdm(frame_data):
            writer.append_data(data)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--state_csv_file", type=str)
    parser.add_argument("-sg", "--subgoal_csv_file", default=None, type=str)
    parser.add_argument("-rt", "--reachtube_csv_file", default=None, type=str)
    parser.add_argument("-o", "--output_path", type=str)
    args = parser.parse_args()
    
    states = pd.read_csv(args.state_csv_file)
    subgoals = pd.read_csv(args.subgoal_csv_file) if args.subgoal_csv_file is not None else None
    reachtubes = pd.read_csv(args.reachtube_csv_file) if args.reachtube_csv_file is not None else None
    save_video(states, args.output_path, subgoals=subgoals, reachtubes=reachtubes)