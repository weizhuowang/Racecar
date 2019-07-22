import matplotlib
import multiprocessing
import time
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math

def main():
    #Create a queue to share data between process
    q = multiprocessing.Queue()

    #Create and start the simulation process
    simulate=multiprocessing.Process(target=simulation,args=(q,))
    simulate.start()

    #Create the base plot
    plot()

    #Call a function to update the plot when there is new data
    updateplot(q)

    print('Done')


def plot():    #Function to create the base plot, make sure to make global the lines, axes, canvas and any part that you would want to update later

    global line1,ax,fig
    size = 50
    x_vec = np.linspace(0,1,size+1)[0:-1]
    y_vec = np.zeros(len(x_vec))
    # this is the call to matplotlib that allows dynamic plotting
    plt.ion()
    fig = plt.figure(figsize=(13,6))
    ax = fig.add_subplot(111)
    # create a variable for the line so we can later update it
    line1, = ax.plot(x_vec,y_vec,'-o',alpha=0.8)        
    #update plot label/title
#        plt.ylabel('Y Label')
#        plt.title('Title: {}'.format(identifier))
    plt.show()

def updateplot(q):
    try:       #Try to check if there is data in the queue
#        result=q.get_nowait()
        resList = []
        for items in range(0, q.qsize()):
            resList.append(q.get_nowait())
        print('res',resList)
        if (resList[-1] !='Q') and (len(resList)>0):
            y1_data = line1.get_ydata(orig=False)
            y1_data = np.append(y1_data[len(resList):],np.array(resList))
            line1.set_ydata(y1_data)
            # adjust limits if new data goes beyond bounds
            if np.min(y1_data)<=line1.axes.get_ylim()[0] or np.max(y1_data)>=line1.axes.get_ylim()[1]:
                plt.ylim([np.min(y1_data)-np.std(y1_data),np.max(y1_data)+np.std(y1_data)])
            # this pauses the data so the figure/axis can catch up - the amount of pause can be altered above
            plt.pause(0.001)
            updateplot(q)
        else:
             print('done')
    except:
#        print("empty")
        plt.pause(0.1)
        updateplot(q)


def simulation(q):
    iterations = range(100)
    for i in iterations:
        rand_val = math.sin(0.5*i) #np.random.randn(1)
#        y_vec[-1] = rand_val
#        y_vec = np.append(y_vec[1:],0.0)
        #here send any data you want to send to the other process, can be any pickable object
        time.sleep(0.11)
        print(i,rand_val)
        q.put(rand_val)
    q.put('Q')

if __name__ == '__main__':
    main()
