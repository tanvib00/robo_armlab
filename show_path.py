import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

path = [(0, 0), (24, 29), (12, 89), (58, 101), (71.39681928670433, 78.33306129395558), (71.39681928670433, 78.33306129395558), (58, 101), (105, 123), (85.78158253829884, 105.07006214488884), (85.78158253829884, 105.07006214488884), (105, 123), (58, 101), (12, 89), (9.895047045165176, 94.71631062820448)]

def show_path(path):
    #image = calc_space()
    image = mpimg.imread('./1.2_images/config_space.png')

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

    axes = plt.gca()
    axes.set_xlim([0,180])
    axes.set_ylim([-180,180])


    #plt.plot(path)
    plt.show()

