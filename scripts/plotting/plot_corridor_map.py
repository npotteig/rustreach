import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib import rc
rc('font', **{'family': 'serif', 'serif': ['Times New Roman']})
rc('text', usetex=True)
plt.rcParams['font.size'] = 24

if __name__ == '__main__':
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_xlim(0, 4.2)
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
    pt_1 = [2, -0.7]
    ax.add_patch(patches.Rectangle(
        (pt_1[0] - w/2, pt_1[1] - h/2),
        w,
        h,
        edgecolor='black',
        facecolor='blue',
        alpha=0.5
    ))

    plt.savefig('figs/paper/corridor_map.pdf', bbox_inches='tight')