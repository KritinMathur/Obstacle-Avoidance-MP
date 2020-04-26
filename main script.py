from tkinter import * 
from tkinter import ttk 
import tkinter.filedialog as f
import os
import re

wp_file=''
obs_file=''
output_file=''
ftypes=[('waypoints file','.waypoints'),('All Files','*')]
ftypesobs=[('Keyhole Markup Language','.kml')]
path=[]
obstacleslist=[]
raw_obs=[]
obno=-1
dell = []
dell_ind = 0


def wppick():
    global wp_file
    wp_file = f.askopenfilename(filetypes=ftypes,defaultextension='.waypoints')
    dnwp = wp_file.split('/')
    Label(root,text = dnwp[-1],bg="black",fg="white").grid(row = 2, column = 0, sticky = N, pady = 2)
    
def obspick():
    global obs_file
    obs_file = f.askopenfilename(filetypes=ftypesobs,defaultextension='.kml')
    dnob = obs_file.split('/')
    Label(root,text = dnob[-1],bg="black",fg="white").grid(row = 4, column = 0, sticky = N, pady = 2)

def save():
    global output_file
    output_file=f.asksaveasfilename(filetypes=ftypes,defaultextension='.waypoints')
    root.destroy()

root = Tk()
root.title("Static Obstacle Avoidance")
root.configure(background='black')
l1=Label(root,text = "SELECT WAYPOINT AND OBSTACLE FILES:",bg='black',fg='white')
wp=Button(root,text="WAYPOINTS",pady='10',padx = 20,bg="cyan",fg="black",bd='4',command=wppick)
obs=Button(root,text="OBSTACLES",pady='10',padx = 25,bg="cyan",fg="black",bd='4',command=obspick)
op=Button(root,text="SAVE-AS",pady='10',padx = 30,bg="cyan",fg="black",bd='4',command=save)

l1.grid(row = 0, column = 0, sticky = N, pady = 2)
wp.grid(row = 1, column = 0, sticky = N, pady = 50, padx = 150)
obs.grid(row = 3, column = 0, sticky = N, pady = 50,padx = 150)
op.grid(row = 5, column = 0, sticky = N, pady = 50, padx = 150)

root.mainloop()


class wp:
    def __init__(self,x,y):
        self.x = x
        self.y = y        

def ccw(A,B,C):
    return (C.y-A.y) * (B.x-A.x) > (B.y-A.y) * (C.x-A.x)

def intersect(A,B,C,D):
    return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)

def line_intersection(line1, line2):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
       raise Exception('lines do not intersect')

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return x, y

def dis(A,B):
    return(((A.x-B.x)**2+(A.y-B.y)**2)**0.5)

def point_inside_polygon(x,y,poly):

    n = len(poly)
    inside =False

    p1x,p1y = poly[0]
    for i in range(n+1):
        p2x,p2y = poly[i % n]
        if y > min(p1y,p2y):
            if y <= max(p1y,p2y):
                if x <= max(p1x,p2x):
                    if p1y != p2y:
                        xinters = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x,p1y = p2x,p2y

    return inside 

#extracting waypoints 
with open(wp_file, 'r') as f:
    allwp=f.read()
    allwp = re.sub("( )+"," ",allwp)
    allwp = re.sub("\t"," ",allwp)
    allwp = re.sub(" 0.(0)+ "," 0 ",allwp)
    allwp = re.sub(" 0.(0)+ "," 0 ",allwp)
    allwp = re.sub("\.(0)+",".0",allwp)
    allwp = re.sub(" (\d)+\.0 1"," ",allwp)
    onlyxy=re.findall('(\d+\.\d+)\s(\d+\.\d+)',allwp)
   #making waypoint objects 
    for wps in onlyxy:
        x=wps[0]
        y=wps[1]
        x=float(x)
        y=float(y)
        path.append(wp(y,x)) 
        
f.close()

#extracting obstacles
with open(obs_file, 'r') as f:
    alltext=f.read()
    alltext = re.sub("\s"," ",alltext)
    alltext = re.sub("(\s)+"," ",alltext)
    raw_obs = re.findall(r'<coordinates>(.*?)</coordinates>',str(alltext))
f.close()

#making obstacle objects
for itr in raw_obs:
    obno=obno+1
    obstacleslist.append([])
    coord_list=itr.split(" ")
    while("" in coord_list): 
        coord_list.remove("") 
    for itr2 in coord_list:
        coord_list_sep=itr2.split(",")
        obstacleslist[obno].append(wp(float(coord_list_sep[0]),float(coord_list_sep[1])))
        

#removing internal obstacle

for way in path:
    for ob in obstacleslist:
        conv_arr = []
        for i in range(len(ob)):
            conv_arr.append([ob[i].x,ob[i].y])
        if point_inside_polygon(way.x,way.y,conv_arr):
            dell.append(dell_ind)
    dell_ind = dell_ind + 1

for i in reversed(dell):
    path.pop(i)
## MAIN CODE STARTS

super_cp = []
super_fp = []
pin = 0
while pin < len(path)-1:
    #finding colliding obstacles    
    collider = []
    for ob in obstacleslist:
        lenob = len(ob)
        for edge in range(lenob):
            if (intersect(path[pin],path[pin+1],ob[edge],ob[(edge+1)%lenob])):
                collider.append(ob)
                break


    acp = []

    #finding collision points
    for ob in collider:
        lenob = len(ob)
        for edge in range(lenob):
            if (intersect(path[pin],path[pin+1],ob[edge],ob[(edge+1)%lenob])):
                l1 = [[path[pin].x,path[pin].y],[path[pin+1].x,path[pin+1].y]]
                l2 = [[ob[edge].x,ob[edge].y],[ob[(edge+1)%lenob].x,ob[(edge+1)%lenob].y]]
                lat,lon = line_intersection(l1,l2)
                acp.append((lat,lon))

    #sorting according to nearest collision
    def disfs(cp):
        d = dis(wp(cp[0],cp[1]),path[pin])
        return d
        
    acp.sort(key = disfs)

    #adding filler waypoints
    afp = []
    pair = 0
    while pair < len(acp):

        
        for ob in collider:
            temp_filler = []
            case = []
            lenob = len(ob)
            flag = 0
            
            for edge in range(lenob):
                
                if intersect(path[pin],path[pin+1],ob[edge],ob[(edge+1)%lenob]):
                    l1 = [[path[pin].x,path[pin].y],[path[pin+1].x,path[pin+1].y]]
                    l2 = [[ob[edge].x,ob[edge].y],[ob[(edge+1)%lenob].x,ob[(edge+1)%lenob].y]]
                    i1,i2 = line_intersection(l1,l2)
                    
                    if (i1 == acp[pair][0] and i2 == acp[pair][1]):
                        flag = flag+1
                        case.append('N')

                    if (i1 == acp[pair+1][0] and i2 == acp[pair+1][1]):
                        flag = flag+1
                        case.append('R')

                if (flag == 1):
                    temp_filler.append((ob[(edge+1)%lenob].x,ob[(edge+1)%lenob].y))
                    
            if(len(case)>0):
                if(case[0] == 'R'):
                    temp_filler.reverse()
                afp.append(temp_filler)
                    
        pair = pair + 2

    super_cp.append(acp)
    super_fp.append(afp)

    pin = pin + 1

## MAIN CODE ENDS

inind = 0
for cps_ind in range(len(super_cp)):
    for cpitr in range(int(len(super_cp[cps_ind])/2)):
        inind = inind + 1
        path.insert(inind,wp(super_cp[cps_ind][cpitr*2][0],super_cp[cps_ind][cpitr*2][1]))
        
        for fps_ind in range(len(super_fp[cps_ind][cpitr])):
            inind = inind + 1
            path.insert(inind,wp(super_fp[cps_ind][cpitr][fps_ind][0],super_fp[cps_ind][cpitr][fps_ind][1]))

        inind = inind + 1
        path.insert(inind,wp(super_cp[cps_ind][cpitr*2 + 1][0],super_cp[cps_ind][cpitr*2+1][1]))
 

    inind = inind + 1

#creating output data
outputlist=[]
wpl=[]
num_lines = 1
with open(wp_file, 'r') as f:
    for line in f:
        num_lines += 1
f.close()
with open(wp_file, 'r') as lines:
    for itr in range(num_lines):
        outputlist.append(lines.readline())
    altstring=outputlist[3]
    altstring = re.sub("( )+"," ",altstring)
    altstring = re.sub("\t"," ",altstring)
    altstring2=altstring.split(" ")    
 
#creating and writing output file
with open(output_file, 'w') as opfile:
    opfile.write(outputlist[0])
    opfile.write(outputlist[1])
    for itr in range(1,len(path)+1):
        s=str(itr)+"	0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	"+str(path[itr-1].y)+"	"+str(path[itr-1].x)+"	"+altstring2[10]+"	1\n"
        opfile.write(s)

