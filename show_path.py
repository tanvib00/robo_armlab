import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg


def show_path(path, a, b, c):
    
    image = mpimg.imread('./waypoints/config_space copy 3.png')

    plt.imshow(image, extent=[0,180,-180,180])
    plt.xlabel("Theta 1 (deg)")
    plt.ylabel("Theta 2 (deg)")
    plt.xticks(np.arange(0, 181, 45))
    plt.yticks(np.arange(-180, 181, 45))

    # draw lines
    lines = []
    for i in range(len(path)-1):
        lines.append([path[i],path[i+1]])
    
    for l in lines:
        if abs(l[0][1] - l[1][1]) > 180:
            if l[0][1] > l[1][1]:
                plt.plot([l[0][0],l[1][0]], [l[0][1],l[1][1]+360], 'w')
                plt.plot([l[0][0],l[1][0]], [l[0][1]-360,l[1][1]], 'w')
            else:
                plt.plot([l[0][0],l[1][0]], [l[0][1],l[1][1]-360], 'w')
                plt.plot([l[0][0],l[1][0]], [l[0][1]+360,l[1][1]], 'w')
        else:
            plt.plot([l[0][0],l[1][0]], [l[0][1],l[1][1]], 'w')

    plt.plot(a[0], a[1], 'bo')
    plt.plot(b[0], b[1], 'bo')
    plt.plot(c[0], c[1], 'bo')

    axes = plt.gca()
    axes.set_xlim([0,180])
    axes.set_ylim([-180,180])

    plt.show()
