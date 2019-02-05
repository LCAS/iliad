import pandas as pd
import numpy as np

loadFile = '/home/manolofc/workspace/TAROS19/src/iliad/taros19_experiments/bags/results.csv'
saveFolder = '/home/manolofc/workspace/TAROS19/src/iliad/taros19_experiments/bags/'
latexFile = saveFolder+"table.tex"

df =  pd.read_csv(loadFile,index_col=0)

# navigation ids as Strings
navs={1:"DWA",2:"TEB",3:"ILIAD"}
df['navigation_ids']=df['navigation_ids'].astype('category')
df['navigation_ids']=df['navigation_ids'].values.rename_categories(navs)

# collisions as numbers, for averages ...
df['collisions']=df['collisions'].astype(int)

# remove unused data
df.drop(df[df['navigation_ids']==0].index, inplace=True)
df.drop(columns=['recording_dates'],inplace=True)
df.drop(columns=['bagfilenames'],inplace=True)

# changing index cols with rename()
df.rename(index=str, columns= {"completion_times": "Time to completion",
                   "path_lengths":"Length of path",
                   "robot_speeds": "Av. robot speed",
                   "human_speeds": "Av. human speed",
                   "min_human_robot_distances": "Min. human-robot dist.",
                   "collisions": "Collisions"},
                                 inplace = True)

startLine = ['}} &',
             '                                 &',
             '                                 &',
             '                                 &',
             '                                 &',
             '                                 &']
endLine = ['\\\\', '\\\\', '\\\\', '\\\\', '\\\\', '\\\\  \\bottomrule']


latexOutput = open(latexFile, 'w')

for i in range(1,6):
    #count executions of each scenario situation
    dfi = df[df['scenario_ids'] == i]
    dfi['ones'] = 1
    totalRuns = dfi.groupby(['navigation_ids']).count()
    totalRuns = totalRuns['ones']

    # get average values of all but collisions
    dfmi = df[df['scenario_ids']==i].groupby(['navigation_ids']).mean()
    collisions=dfmi['Collisions']*totalRuns


    # dont use collision to stats:
    indx = np.logical_and( (df['scenario_ids'] == i) , (df['Collisions']==0 ) )
    dfmi = df[indx].groupby(['navigation_ids']).mean()
    dfmi['Collisions'] = collisions

    # round decimals
    dfmi = dfmi.round(2)

    # drop scenario no navigation
    dfmi.drop(labels=[0],inplace=True)

    # temp csv file without latex stuff
    saveFile=saveFolder+'results_scenario_'+str(i)+'.csv'
    dfmi=dfmi.T
    dfmi.drop(labels=['scenario_ids'], inplace=True)
    dfmi.to_csv(saveFile, index=True,sep='&',na_rep='-')

    # and we get the data separated by & simbols ...
    with open(saveFile) as inputData:
        fileLines = inputData.readlines()

    # now add stuff to the file to create latex table content
    code='unknown'
    if i==5:
        code='base'
    elif i==1:
        code='cross L-R'
    elif i==2:
        code='cross R-L'
    elif i==3:
        code='overtake'
    elif i==4:
        code='pass-by'


    for j in range(0,len(startLine)):
        newLine=''
        if j==0:
            newLine='\\multirow{6}{*}{\\rot{'+code
        dataLine=fileLines[j+1]
        dataLine=dataLine[0:-1]
        newLine=newLine+startLine[j]+dataLine+endLine[j]
        latexOutput.write(newLine+'\n')
latexOutput.close()

