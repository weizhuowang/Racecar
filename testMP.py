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
    

    #Create the base plot
    plot()
    
    simulate.start()
    
    #Call a function to update the plot when there is new data
    status = 0
    while status == 0:
        status = updateplot(q)

    print('Done')


def plot():    #Function to create the base plot, make sure to make global the lines, axes, canvas and any part that you would want to update later

    global line1,ax,fig
    size = 50
    x_vec = np.linspace(0,0.0001,size+1)[0:-1]
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
        y1list = []
        xlist = []
        for items in range(0, q.qsize()):
            datachuck = q.get_nowait()
            y1list.append(datachuck[1])
            xlist.append(datachuck[0])
#            curr_x += 1
        print('res',xlist,y1list)
        
        if (y1list[-1] !='Q'):
            if (len(y1list)>0):
                x_data = line1.get_xdata(orig=False)
                y1_data = line1.get_ydata(orig=False)
                x_data = np.append(x_data[len(xlist):],np.array(xlist))
                y1_data = np.append(y1_data[len(y1list):],np.array(y1list))
                line1.set_xdata(x_data)
                line1.set_ydata(y1_data)
                
                # adjust limits if new data goes beyond bounds
                if np.min(y1_data)<=line1.axes.get_ylim()[0] or np.max(y1_data)>=line1.axes.get_ylim()[1]:
                    plt.ylim([np.min(y1_data)-np.std(y1_data),np.max(y1_data)+np.std(y1_data)])
                # this pauses the data so the figure/axis can catch up - the amount of pause can be altered above
                plt.xlim([np.min(x_data),np.max(x_data)])
                plt.pause(0.0001)
            return 0
        else:
            print('done')
            return 1
    except:
#        print("empty")
        plt.pause(0.0001)
        return 0


def simulation(q):
    iterations = range(100)
    for i in iterations:
        rand_val = math.sin(0.5*i) #np.random.randn(1)
        timeval = i
#        y_vec[-1] = rand_val
#        y_vec = np.append(y_vec[1:],0.0)
        #here send any data you want to send to the other process, can be any pickable object
        time.sleep(0.11)
        print(i,rand_val)
        q.put([timeval,rand_val])
    q.put(['Q','Q'])

if __name__ == '__main__':
    main()
