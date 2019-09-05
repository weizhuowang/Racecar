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

# ======================================================
# ======================================================
# ======================================================

def plot():    #Function to create the base plot, make sure to make global the lines, axes, canvas and any part that you would want to update later

    global line1,line2,line3,line4,ax1,ax2,ax3,fig
    size = 50
    x_vec = np.linspace(0,0.0001,size+1)[0:-1]
    y_vec = np.zeros(len(x_vec))
    # this is the call to matplotlib that allows dynamic plotting
    plt.ion()
    fig = plt.figure(figsize=(13,6))
    ax1 = fig.add_subplot(221)
    plt.grid()
    ax2 = fig.add_subplot(222)
    plt.grid()
    ax3 = fig.add_subplot(223)
    plt.grid()
    # create a variable for the line so we can later update it
    line1, = ax1.plot(x_vec,y_vec,'-o',alpha=0.8)
    line2, = ax2.plot(x_vec,y_vec,'-o',alpha=0.8)
    line3, = ax3.plot(x_vec,y_vec,'-o',alpha=0.8)
    line4, = ax3.plot(x_vec,y_vec,'-o',alpha=0.8)
    
    #update plot label/title
#        plt.ylabel('Y Label')
#        plt.title('Title: {}'.format(identifier))

    # ========== BLIT =========
    # cache the background
#    ax1bg = fig.canvas.copy_from_bbox(ax1.bbox)
#    ax2bg = fig.canvas.copy_from_bbox(ax2.bbox)
    # =========================
    
    
    plt.show()

def redraw_figure():
    fig.canvas.flush_events()
#    plt.show(block=False)
#    plt.show(block=False)


#    # ==========BLIT===========
#    fig.canvas.restore_region(ax1bg)
#    fig.canvas.restore_region(ax2bg)

#    # redraw just the points
#    ax1.draw_artist(line1)
#    ax2.draw_artist(line2)

#    # fill in the axes rectangle
#    fig.canvas.blit(ax1.bbox)
#    fig.canvas.blit(ax2.bbox)
    
#    plt.pause(0.00000001)
    
def update_plot_info(ax,line,xlist,ylist,xlabel='time (s)',ylabel=''):
    xdata = line.get_xdata(orig=False)
    xdata = np.append(xdata[len(xlist):],np.array(xlist))
    ydata = line.get_ydata(orig=False)
    ydata = np.append(ydata[len(ylist):],np.array(ylist))
    line.set_xdata(xdata)
    line.set_ydata(ydata)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    
    # adjust limits if new data goes beyond bounds
    ax.set_xlim([np.min(xdata),np.max(xdata)])
    if np.min(ydata)<=line.axes.get_ylim()[0] or np.max(ydata)>=line.axes.get_ylim()[1]:
        ax.set_ylim([np.min(ydata)-np.std(ydata),np.max(ydata)+np.std(ydata)])
                    
def updateplot(q):
    try:       #Try to check if there is data in the queue
        xlist,ylist = [[],[]]
        for items in range(0, q.qsize()):
            datachuck = q.get_nowait()
            xlist.append(datachuck[0])
            ylist.append(datachuck[1:])
            
        if (len(ylist)>0):
            ylist = np.array(ylist)
            if (ylist[-1,-1] !='Q'):
                ax1.set_ylim([-3,3])
                update_plot_info(ax1,line1,xlist,ylist[:,0],ylabel='torque (N*cm)')
                update_plot_info(ax2,line2,xlist,ylist[:,1],ylabel='RPM')
                update_plot_info(ax3,line3,xlist,ylist[:,2])
                update_plot_info(ax3,line4,xlist,ylist[:,3],ylabel='Current & Voltage')
                ax3.set_ylim([-15,15])
                ax3.legend(['Current (A)','Voltage (V)'],loc='upper right')
                
                redraw_figure()
                return 0
            else:
                print('done')
                return 1
        else:
            return 0
    except:
#        print("empty")
#        redraw_figure()
        return 0


def getdata1(q):
    iterations = range(300)
    for i in iterations:
        rand_val = math.sin(0.1*i) #np.random.randn(1)
        timeval = i
        #here send any data you want to send to the other process, can be any pickable object
        time.sleep(0.02)
        print(i,rand_val)
        q.put([timeval,rand_val,rand_val**2-rand_val,rand_val,rand_val+2,rand_val**2])
    q.put(['Q','Q','Q','Q','Q','Q'])

if __name__ == '__main__':
    main()
