from matplotlib import pyplot as plt
import os
import realtime
import numpy as np


def play_anim():
    shoulderLen =9.88
    armLen = 22.0
    feetLen = 22.2
    toDeg = 180.0/np.pi
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    width = 15
    length = 41
    poses = [(width,length),(-width,length),(-width,-length),(width,-length)]

    def drawLeg(id,shoulder,arm,feet):
        
        
        top = np.asarray([poses[id][0],poses[id][1],0],dtype = np.float32)
        pts = [top]
        if id == 0 or id == 3:
            curPt = top+np.asarray([shoulderLen* np.cos(shoulder),0,shoulderLen*np.sin(shoulder)])
        else:
            curPt = top+np.asarray([-shoulderLen* np.cos(shoulder),0,shoulderLen*np.sin(shoulder)])
        pts.append(curPt)
        if id == 0 or id == 3:
            trans = np.asarray([
                [np.cos(shoulder), 0, -np.sin(shoulder)],
                [0, 1, 0],
                [np.sin(shoulder), 0, np.cos(shoulder)]
            ])
        else:
            trans = np.asarray([
                [np.cos(shoulder), 0, np.sin(shoulder)],
                [0, 1, 0],
                [-np.sin(shoulder), 0, np.cos(shoulder)]
            ])
        relPt =  np.asarray([0,armLen*np.cos(arm),-armLen*np.sin(arm)])
        pts.append(curPt + trans@relPt)
        relPt += np.asarray([0,-feetLen*np.cos(feet+arm),feetLen*np.sin(feet+arm)])
        pts.append(curPt + trans@relPt)
        pts = np.stack(pts,-1)
        #print(pts)
        ax.plot(pts[0], pts[1], pts[2])
        #ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2])
        ax.scatter([0], [0], [0],"r")
        ax.set_xlim(-50, 50)
        ax.set_ylim(-50, 50)
        ax.set_zlim(-50, 50)

    loaded = [[0]*3,[0]*3,[0]*3,[0]*3]
    status = [False]*4
    with open("../log/record.txt",'r') as f:
        cnts = [0,0,0,0]
        while True:
            line = f.readline()
            if len(line) == 0:
                print("finished")
                break
            ang = line.split(",")
            if len(ang) != 4:
                break
            id = int(ang[0])
            cnts[id]+=1
            if cnts[id] == 10:
                status[id] = True
                loaded[id][0] = float(ang[1])
                loaded[id][1] = float(ang[2])
                loaded[id][2] = float(ang[3])
                #if id == 0:
                    #print(line)
                cnts[id] = 0
            if status[0] and status[1] and status[2] and status[3]:
                ax.cla()
                drawLeg(0,loaded[0][0],loaded[0][1],loaded[0][2])
                drawLeg(1,loaded[1][0],loaded[1][1],loaded[1][2])
                drawLeg(2,loaded[2][0],loaded[2][1],loaded[2][2])
                drawLeg(3,loaded[3][0],loaded[3][1],loaded[3][2])
                status[0] = status[1] = status[2] = status[3] = False
                plt.pause(0.001)

# 按间距中的绿色按钮以运行脚本。
if __name__ == '__main__':
    print("************************")
    print("select mode: \"R\" for realtime, \"S\" for showing log file\n")
    cmd = input("mode:")
    if cmd == 'S':
        play_anim()
    elif cmd == 'R':
        realtime.connect()