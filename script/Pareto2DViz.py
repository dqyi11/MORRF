import matplotlib.pyplot as plt
import numpy as np  
from Dominate import *
import sys         

def viz(score):
    fig = plt.figure()     
    ax = fig.add_subplot(111)
    pl = ax.plot(scores[:,0], scores[:,1],'bs',label="Score of Path")
        
    ul = ax.plot(scores[0,0], scores[1,1], 'ro', label="Utopia Reference Vector")    
    ax.set_xlabel("Objective 1", fontsize='x-large')
    ax.set_ylabel("Objective 2", fontsize='x-large')
    ax.legend( numpoints=1, loc='upper right', shadow=True, fontsize='x-large')    
    plt.show()



def load(filename):
 
    scores = []
    with open(filename, 'r') as f:
        for line in f:
            line = line.replace('\n','')      
            ss = line.split('\t')
            #print ss
            scores.append([ float(ss[0]), float(ss[1]) ])
    return np.array(scores);

if __name__ == '__main__':

    filename = sys.argv[1]

    scores = load(filename)
    print str(scores)
    print non_dominate_rate(scores)
    viz(scores)

