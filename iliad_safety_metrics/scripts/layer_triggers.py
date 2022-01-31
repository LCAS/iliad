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

s1 = True
s2 = True
s3 = True
s4 = True


s1_brakes = np.array([])
s1_vsmu = np.array([])
s1_constraints = np.array([])
s1_replans = np.array([])

s2_brakes = np.array([])
s2_vsmu = np.array([])
s2_constraints = np.array([])
s2_replans = np.array([])

s3_brakes = np.array([])
s3_vsmu = np.array([])
s3_constraints = np.array([])
s3_replans = np.array([])

s4_brakes = np.array([])
s4_vsmu = np.array([])
s4_constraints = np.array([])
s4_replans = np.array([])


for r in range(ini, end + 1):
    if s1:
        try:
            with open("../data/"+path+"_12h_sce1_layer1/"+path+"_12h_sce1_layer1_"+str(r)+"_metrics.txt") as file :
                for line in file: 
                    current_line = line.split(':')
                    if current_line[0]=="brakes":
                        s1_brakes = np.append(s1_brakes,float(current_line[1]))
                    if current_line[0]=="vsmu constraints":
                        s1_vsmu = np.append(s1_vsmu,float(current_line[1]))
                    if current_line[0]=="hri constraints":
                        s1_constraints = np.append(s1_constraints,float(current_line[1]))
                    if current_line[0]=="hri replans":
                        s1_replans = np.append(s1_replans,float(current_line[1]))
        except:
            pass
    if s2:
        try:
            with open("../data/"+path+"_12h_sce1_layer12/"+path+"_12h_sce1_layer12_"+str(r)+"_metrics.txt") as file :
                for line in file: 
                    current_line = line.split(':')
                    if current_line[0]=="brakes":
                        s2_brakes = np.append(s2_brakes,float(current_line[1]))
                    if current_line[0]=="vsmu constraints":
                        s2_vsmu = np.append(s2_vsmu,float(current_line[1]))
                    if current_line[0]=="hri constraints":
                        s2_constraints = np.append(s2_constraints,float(current_line[1]))
                    if current_line[0]=="hri replans":
                        s2_replans = np.append(s2_replans,float(current_line[1]))
        except:
            pass
    if s3:
        try:
            with open("../data/"+path+"_12h_sce1_layer123/"+path+"_12h_sce1_layer123_"+str(r)+"_metrics.txt") as file :
                for line in file: 
                    current_line = line.split(':')
                    if current_line[0]=="brakes":
                        s3_brakes = np.append(s3_brakes,float(current_line[1]))
                    if current_line[0]=="vsmu constraints":
                        s3_vsmu = np.append(s3_vsmu,float(current_line[1]))
                    if current_line[0]=="hri constraints":
                        s3_constraints = np.append(s3_constraints,float(current_line[1]))
                    if current_line[0]=="hri replans":
                        s3_replans = np.append(s3_replans,float(current_line[1]))
        except:
            pass
    if s4:
        try:
            with open("../data/"+path+"_12h_sce1_layer1235/"+path+"_12h_sce1_layer1235_"+str(r)+"_metrics.txt") as file :
                for line in file: 
                    current_line = line.split(':')
                    if current_line[0]=="brakes":
                        s4_brakes = np.append(s4_brakes,float(current_line[1]))
                    if current_line[0]=="vsmu constraints":
                        s4_vsmu = np.append(s4_vsmu,float(current_line[1]))
                    if current_line[0]=="hri constraints":
                        s4_constraints = np.append(s4_constraints,float(current_line[1]))
                    if current_line[0]=="hri replans":
                        s4_replans = np.append(s4_replans,float(current_line[1]))
        except:
            pass
            
if s1:
    print "S1"
    print "   breaks:",     min(s1_brakes),     "-", np.median(s1_brakes),      "-",max(s1_brakes)

if s2:
    print "S2"
    print "   breaks:",     min(s2_brakes),     "-", np.median(s2_brakes),      "-",max(s2_brakes)
    print "   vsmu:",       min(s2_vsmu),       "-", np.median(s2_vsmu),        "-",max(s2_vsmu)

if s3:
    print "S3"
    print "   breaks:",     min(s3_brakes),     "-", np.median(s3_brakes),      "-",max(s3_brakes)
    print "   vsmu:",       min(s3_vsmu),       "-", np.median(s3_vsmu),        "-",max(s3_vsmu)
    print "   constraints:",min(s3_constraints),"-", np.median(s3_constraints), "-",max(s3_constraints)
    print "   replans:",    min(s3_replans),    "-", np.median(s3_replans),     "-",max(s3_replans)

if s4:
    print "S4"
    print "   breaks:",     min(s4_brakes),     "-", np.median(s4_brakes),      "-",max(s4_brakes)
    print "   vsmu:",       min(s4_vsmu),       "-", np.median(s4_vsmu),        "-",max(s4_vsmu)
    print "   constraints:",min(s4_constraints),"-", np.median(s4_constraints), "-",max(s4_constraints)
    print "   replans:",    min(s4_replans),    "-", np.median(s4_replans),     "-",max(s4_replans)