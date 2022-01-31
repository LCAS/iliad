import matplotlib.pyplot as plt
import numpy as np
import matplotlib 

from pylab import plot, show, savefig, xlim, figure, \
                hold, ylim, legend, boxplot, setp, axes

import math

def set_colors(bp):
    setp(bp['boxes'][0], color='blue',linewidth=2)
    setp(bp['caps'][0], color='blue',linewidth=2)
    setp(bp['caps'][1], color='blue',linewidth=2)
    setp(bp['whiskers'][0], color='blue',linewidth=2)
    setp(bp['whiskers'][1], color='blue',linewidth=2)
    setp(bp['fliers'][0], color='blue',linewidth=2)
    setp(bp['fliers'][1], color='blue',linewidth=2)
    setp(bp['medians'][0], color='blue',linewidth=2)

    setp(bp['boxes'][1], color='orange',linewidth=2)
    setp(bp['caps'][2], color='orange',linewidth=2)
    setp(bp['caps'][3], color='orange',linewidth=2)
    setp(bp['whiskers'][2], color='orange',linewidth=2)
    setp(bp['whiskers'][3], color='orange',linewidth=2)
    setp(bp['fliers'][2], color='orange',linewidth=2)
    setp(bp['fliers'][3], color='orange',linewidth=2)
    setp(bp['medians'][1], color='orange',linewidth=2)

    setp(bp['boxes'][2], color='green',linewidth=2)
    setp(bp['caps'][4], color='green',linewidth=2)
    setp(bp['caps'][5], color='green',linewidth=2)
    setp(bp['whiskers'][4], color='green',linewidth=2)
    setp(bp['whiskers'][5], color='green',linewidth=2)
    setp(bp['fliers'][2], color='green',linewidth=2)
    setp(bp['fliers'][3], color='green',linewidth=2)
    setp(bp['medians'][2], color='green',linewidth=2)

    setp(bp['boxes'][3], color='red',linewidth=2)
    setp(bp['caps'][6], color='red',linewidth=2)
    setp(bp['caps'][7], color='red',linewidth=2)
    setp(bp['whiskers'][6], color='red',linewidth=2)
    setp(bp['whiskers'][7], color='red',linewidth=2)
    setp(bp['fliers'][2], color='red',linewidth=2)
    setp(bp['fliers'][3], color='red',linewidth=2)
    setp(bp['medians'][3], color='red',linewidth=2)

# read the data
path1 = "afbcdfba"
path2 = "cbedc"
savefig = True
ini = 1
end = 20
path = path2

s1 = True
s2 = True
s3 = True
s4 = True


s1_1 = np.array([])
s1_2 = np.array([])
s1_3 = np.array([])

s2_1 = np.array([])
s2_2 = np.array([])
s2_3 = np.array([])

s3_1 = np.array([])
s3_2 = np.array([])
s3_3 = np.array([])

s4_1 = np.array([])
s4_2 = np.array([])
s4_3 = np.array([])

for r in range(ini, end + 1):
    if s1:
        try:
            with open("../data/"+path+"_12h_sce1_layer1/"+path+"_12h_sce1_layer1_"+str(r)+"_robot_human_status.txt") as file :
                for line in file: 
                    current_line = line.split(',')
                    human_distance = float(current_line[7])
                    relative_speed = abs(float(current_line[8]))
                    if (human_distance > 0 and human_distance <= 3):
                        if relative_speed < 2:
                            s1_1 = np.append(s1_1,relative_speed)

                    if (human_distance > 3 and human_distance <=6):
                        if relative_speed < 2:
                            s1_2 = np.append(s1_2,relative_speed)

                    if (human_distance > 6 and human_distance <=9):
                        if relative_speed < 2:
                            s1_3 = np.append(s1_3,relative_speed)
        except:
            pass
    if s2:
        try:
            with open("../data/"+path+"_12h_sce1_layer12/"+path+"_12h_sce1_layer12_"+str(r)+"_robot_human_status.txt") as file :
                for line in file: 
                    current_line = line.split(',')
                    human_distance = float(current_line[7])
                    relative_speed = abs(float(current_line[8]))
                    if (human_distance > 0 and human_distance <= 3):
                        if relative_speed < 2:
                            s2_1 = np.append(s2_1,relative_speed)

                    if (human_distance > 3 and human_distance <=6):
                        if relative_speed < 2:
                            s2_2 = np.append(s2_2,relative_speed)

                    if (human_distance > 6 and human_distance <=9):
                        if relative_speed < 2:
                            s2_3 = np.append(s2_3,relative_speed)
        except:
            pass
    if s3:
        try:
            with open("../data/"+path+"_12h_sce1_layer123/"+path+"_12h_sce1_layer123_"+str(r)+"_robot_human_status.txt") as file :
                for line in file: 
                    current_line = line.split(',')
                    human_distance = float(current_line[7])
                    relative_speed = abs(float(current_line[8]))
                    if (human_distance > 0 and human_distance <= 3):
                        if relative_speed < 2:
                            s3_1 = np.append(s3_1,relative_speed)

                    if (human_distance > 3 and human_distance <=6):
                        if relative_speed < 2:
                            s3_2 = np.append(s3_2,relative_speed)

                    if (human_distance > 6 and human_distance <=9):
                        if relative_speed < 2:
                            s3_3 = np.append(s3_3,relative_speed)
        except:
            pass
    if s4:
        try:
            with open("../data/"+path+"_12h_sce1_layer1235/"+path+"_12h_sce1_layer1235_"+str(r)+"_robot_human_status.txt") as file :
                for line in file: 
                    current_line = line.split(',')
                    human_distance = float(current_line[7])
                    relative_speed = abs(float(current_line[8]))
                    if (human_distance > 0 and human_distance <= 3):
                        if relative_speed < 2:
                            s4_1 = np.append(s4_1,relative_speed)

                    if (human_distance > 3 and human_distance <=6):
                        if relative_speed < 2:
                            s4_2 = np.append(s4_2,relative_speed)

                    if (human_distance > 6 and human_distance <=9):
                        if relative_speed < 2:
                            s4_3 = np.append(s4_3,relative_speed)
        except:
            pass

print np.median(s1_1)
print np.median(s1_2)
print np.median(s1_3)

fig = plt.figure(figsize=(8,4))
#fig = plt.figure()
ax = plt.subplot(111)
#hold(True)

size = 13

bp = ax.boxplot([s1_1,s2_1,s3_1,s4_1],positions=[1,2,3,4])
set_colors(bp)

bp = ax.boxplot([s1_2,s2_2,s3_2,s4_2],positions=[6,7,8,9])
set_colors(bp)

bp = ax.boxplot([s1_3,s2_3,s3_3,s4_3],positions=[11,12,13,14])
set_colors(bp)


ax.set_axisbelow(True)
ax.yaxis.grid(linestyle='-', linewidth='0.2')
xlim(0,15)
ylim(0,1)
ax.set_xticklabels(['0-3','3-6','6-9'])
ax.set_xticks([2.5, 7.5 , 12.5])
ax.get_xaxis().tick_bottom()
ax.get_yaxis().tick_left()

plt.xlabel('Human distance range (m)',fontsize=size)
plt.ylabel('Relative speed human-robot (m/s)',fontsize=size)

hB, = plot([1,1],'blue')
hO, = plot([1,1],'orange')
hG, = plot([1,1],'green')
hR, = plot([1,1],'red')
legend((hB, hO,hG,hR),('S1', 'S2', 'S3', 'S4'))
hB.set_visible(False)
hO.set_visible(False)
hG.set_visible(False)
hR.set_visible(False)


print 

if path == path1:
    plt.title("Scenario 1: path AFBCDFBA")
    if savefig:
        plt.savefig("times_AFBCDFBA.png",bbox_inches='tight')
    plt.show()
else:
    plt.title("Scenario 2: path CBEDC")
    if savefig:
        plt.savefig("times_CBEDC.png",bbox_inches='tight')
    plt.show()