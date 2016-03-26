import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np  
import sys         

def viz(score):
    fig = plt.figure()     
    ax = fig.add_subplot(111, projection='3d')
    ul = ax.scatter(scores[0,0], scores[1,1], scores[2,2], c='r', marker = 'o' )    
    pl = ax.scatter(scores[:,0], scores[:,1], scores[:,2], c='b', marker = 's' )
        
    ax.set_xlabel("Objective 1", fontsize='x-large')
    ax.set_ylabel("Objective 2", fontsize='x-large')
    ax.set_zlabel("Objective 2", fontsize='x-large')
    ax.legend([pl, ul], ["Score of Path", "Utopia Reference Vector"], numpoints=1, loc='upper right', shadow=True, fontsize='x-large')    
    plt.show()


def load(filename):
 
    scores = []
    with open(filename, 'r') as f:
        for line in f:
            line = line.replace('\n','')      
            ss = line.split('\t')
            print ss
            scores.append([ float(ss[0]), float(ss[1]), float(ss[2]) ])
    return np.array(scores);

if __name__ == '__main__':

    filename = sys.argv[1]

    scores = load(filename)
    viz(scores)
