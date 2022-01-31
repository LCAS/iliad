import matplotlib.pyplot as plt
import numpy as np
import matplotlib 


# read the data
path1 = "afbcdfba"
path2 = "cbedc"
savefig = True
ini = 1
end = 20
path = path2

baseline = True
s1 = True
s2 = True
s3 = True
s4 = True


baseline_times = np.array([])
s1_times = np.array([])
s2_times = np.array([])
s3_times = np.array([])
s4_times = np.array([])

for r in range(ini, end + 1):
    if baseline:
        try:
            with open("../data/"+path+"_baseline/"+path+"_baseline_"+str(r)+"_metrics.txt") as file :
                for line in file: 
                    current_line = line.split(':')
                    if current_line[0]=="total mission time":
                        baseline_times = np.append(baseline_times,float(current_line[1]))
        except:
            pass

    if s1:
        try:
            with open("../data/"+path+"_12h_sce1_layer1/"+path+"_12h_sce1_layer1_"+str(r)+"_metrics.txt") as file :
                for line in file: 
                    current_line = line.split(':')
                    if current_line[0]=="total mission time":
                        s1_times = np.append(s1_times,float(current_line[1]))
        except:
            pass
    if s2:
        try:
            with open("../data/"+path+"_12h_sce1_layer12/"+path+"_12h_sce1_layer12_"+str(r)+"_metrics.txt") as file :
                for line in file: 
                    current_line = line.split(':')
                    if current_line[0]=="total mission time":
                        s2_times = np.append(s2_times,float(current_line[1]))
        except:
            pass
    if s3:
        try:
            with open("../data/"+path+"_12h_sce1_layer123/"+path+"_12h_sce1_layer123_"+str(r)+"_metrics.txt") as file :
                for line in file: 
                    current_line = line.split(':')
                    if current_line[0]=="total mission time":
                        s3_times = np.append(s3_times,float(current_line[1]))
        except:
            pass
    if s4:
        try:
            with open("../data/"+path+"_12h_sce1_layer1235/"+path+"_12h_sce1_layer1235_"+str(r)+"_metrics.txt") as file :
                for line in file: 
                    current_line = line.split(':')
                    if current_line[0]=="total mission time":
                        s4_times = np.append(s4_times,float(current_line[1]))
        except:
            pass

fig = plt.figure(figsize=(4,3))
#fig = plt.figure()
ax = plt.subplot(111)

size = 13

flierprops = dict(marker='o', markerfacecolor='r',linestyle='none', markeredgecolor='r',alpha=0.3)

bp = ax.boxplot([baseline_times,s1_times,s2_times,s3_times,s4_times],patch_artist=True,flierprops=flierprops)
ax.set_axisbelow(True)
ax.yaxis.grid(linestyle='-', linewidth='0.2')

ax.set_xticklabels(['Baseline','S1','S2','S3','S4'])
ax.get_xaxis().tick_bottom()
ax.get_yaxis().tick_left()

## change outline color, fill color and linewidth of the boxes
for box in bp['boxes']:
    # change outline color
    box.set( color='#7570b3', linewidth=2)
    # change fill color
    box.set( facecolor = '#ffffff')

## change color and linewidth of the whiskers
for whisker in bp['whiskers']:
    whisker.set(color='#7570b3', linewidth=2)

## change color and linewidth of the caps
for cap in bp['caps']:
    cap.set(color='#7570b3', linewidth=2)

## change color and linewidth of the medians
for median in bp['medians']:
    #median.set(color='#b2df8a', linewidth=2)
    median.set(color='#7570b3', linewidth=2)

## change the style of fliers and their fill
# for flier in bp['fliers']:
#     flier.set(marker='o', color='#e7298a', alpha=0.3)



plt.ylabel('Time (s)',fontsize=size)
plt.xlabel('Configuration',fontsize=size)

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