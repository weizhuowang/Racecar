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
    simulate=multiprocessing.Process(target=getdata1,args=(q,))
    

    #Create the base plot
    plot()
    
    simulate.start()
    
    #Call a function to update the plot when there is new data
    status = 0
    while status == 0:
        status = updateplot(q)

    print('Done')


def plot():    #Function to create the base plot, make sure to make global the lines, axes, canvas and any part that you would want to update later

    global line1,line2,ax1,ax2,fig
    size = 50
    x_vec = np.linspace(0,0.0001,size+1)[0:-1]
    y_vec = np.zeros(len(x_vec))
    # this is the call to matplotlib that allows dynamic plotting
    plt.ion()
    fig = plt.figure(figsize=(13,6))
    ax1 = fig.add_subplot(121)
    ax2 = fig.add_subplot(122)
    # create a variable for the line so we can later update it
    line1, = ax1.plot(x_vec,y_vec,'-o',alpha=0.8)
    line2, = ax2.plot(x_vec,y_vec,'-o',alpha=0.8)
    #update plot label/title
#        plt.ylabel('Y Label')
#        plt.title('Title: {}'.format(identifier))
    plt.show()

def redraw_figure():
    plt.gcf().canvas.flush_events()
    plt.show(block=False)
    plt.show(block=False)
#    plt.pause(0.0001)
    
def update_plot_info(ax,line,xdata,ydata):
    line.set_xdata(xdata)
    line.set_ydata(ydata)
    # adjust limits if new data goes beyond bounds
    ax.set_xlim([np.min(xdata),np.max(xdata)])
    if np.min(ydata)<=line.axes.get_ylim()[0] or np.max(ydata)>=line.axes.get_ylim()[1]:
        ax.set_ylim([np.min(ydata)-np.std(ydata),np.max(ydata)+np.std(ydata)])
                    
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
                
                update_plot_info(ax1,line1,x_data,y1_data)
                update_plot_info(ax2,line2,x_data,y1_data)
                
                # this pauses the data so the figure/axis can catch up - the amount of pause can be altered above
                redraw_figure()
            return 0
        else:
            print('done')
            return 1
    except:
#        print("empty")
        redraw_figure()
        return 0


def getdata1(q):
    iterations = range(500)
    for i in iterations:
        rand_val = math.sin(0.5*i) #np.random.randn(1)
        timeval = i
#        y_vec[-1] = rand_val
#        y_vec = np.append(y_vec[1:],0.0)
        #here send any data you want to send to the other process, can be any pickable object
        time.sleep(0.02)
        print(i,rand_val)
        q.put([timeval,rand_val])
    q.put(['Q','Q'])

if __name__ == '__main__':
    main()
