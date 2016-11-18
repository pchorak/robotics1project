# Demonstrate path planning with the roadmap
import numpy as np
import time

import Simulation
import Roadmap
import SerialInterface

table = np.array([[[-245,-345,-30],[-245,345,-30],[345,345,-30]], \
    [[-245,-345,-30],[345,-345,-30],[345,345,-30]]]) # table
# obstacle = np.array([[[230.0,-10,0],[230,10,0],[240,0,200]], \
#     [[230,-10,0],[250,-10,0],[240,0,200]], \
#     [[230,10,0],[250,10,0],[240,0,200]], \
#     [[250,-10,0],[250,10,0],[240,0,200]]]) # poll
obstacle = np.array([[[150,-200,-30],[150,200,-30],[150,-200,50]], \
    [[150,200,-30],[150,200,50],[150,-200,50]]]) # wall
sim = Simulation.Simulation()
sim.add_obstacles(table)
sim.add_obstacles(obstacle)
prm = Roadmap.Roadmap(sim,150,np.array([[-90,90],[0,60],[0,60]]))
prm.display()
# path = prm.get_path((45,60,0),(-45,60,0))
path = prm.get_path((0,0,60),(0,45,20))
prm.plot_path(path)
# sim.display(path[0])

interface = SerialInterface.SerialInterface('/dev/tty.usbmodemFD121')
time.sleep(2)

# execute path
for p in path:
    interface.send_absolute_angles(p[0],p[1],p[2],0)
    while (msg == interface.current_status):
        time.sleep(0.1)
    msg = interface.current_status
