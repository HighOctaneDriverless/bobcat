import numpy as np
para_matches = 2
#averaging
step_size = 5
#data = np.arange(15)
data = np.array([4,4,2,2,2,1,1,8,1,1,4,4,4,4,4,4,4,4,5,5,4,4,2,2,2,1,1,8,1,1,4,4,4,4,4,4,4,4,5,5,4,4,2,2,2,1,1,8,1,1,4,4,4,4,4,4,4,4,5,5,4,4,2,2,2,1,1,8,1,1,4,4,4,4,4,4,4,4,5,5])
data = np.repeat(data,8)
print(data.shape)
#for over array
clusters = []
matches = []

def appendCluster(m):
    #print(m)
    middle = sum(m)/len(m)
    distance = 0
    for j in range(m[0],m[len(m)-1]+1):
        distance = distance + data[j]
        #print(j)
    #print(m)
    avgDist = distance/len(m)
    vec = np.array([avgDist,middle,len(m)])
    return vec
    

data_av = []
#average over avg values
for i in range(0,data.size-1,step_size):
    sum_ = np.sum(data[i:i+step_size])
    sum_avg = sum_/step_size
    #print(sum_avg)
    data_av.append(sum_avg)

data_avg = np.array(data_av)
#print(data_avg)
#data to data_avg
for i in range(0,data_avg.size-1):
    #print("i",i)
    if(np.power(data_avg[i+1]-data_avg[i],2) < 3 and data_avg[i] < 6):
        matches.append(i)
        #print('length matches',len(matches))
        if(i==data_avg.size-2 and len(matches) > para_matches):
            #print('len>para_matches')
            matches.append(i+1)     
            print('cluster recognized')
            vec = appendCluster(matches)
            clusters.append(vec)
            matches = []
    elif(len(matches) > para_matches):
        matches.append(i)
        print('cluster recognized')
        vec = appendCluster(matches)
        clusters.append(vec)
        matches = []
    else:
        matches = []
    
print('cluster',clusters)


#shape:
#t[0,:,0] ist die eine dimension