import numpy as np

def CL1(RunTime, state, weight):
	CL_Class=np.zeros(60, dtype=np.int8)
	CL_Cluster=np.zeros((2,3,6), dtype=np.int16)
	CL_Clusternum=np.zeros((2,3), dtype=np.int8)
	CL_F=0
	CL_Fnew=0
	CL_Max=0
	if state==0:
		weight=np.array([1.0,0.0])
	else:
		weight[0]=weight[0]/24

	for i in range(3):
		CL_Class[i]=1
		CL_Class[i+3]=2
		CL_Class[i+12]=4
		CL_Class[i+15]=5
	for i in range(6):
		CL_Class[i+6]=3
		CL_Class[i+18]=6
	for i in range(60):
		k=CL_Class[i]
		if k>0:
			k1=int((k-1)/3)
			k2=(k-1)%3
			CL_F+=RunTime[i+8][k1][k2]*weight[0]
			if RunTime[i+8][k1][k2]>CL_Max:
				CL_Max=RunTime[i+8][k1][k2]
	CL_F+=CL_Max*weight[1]

	TorF=True
	Time=0
	while TorF:
		TorF=False
		Time+=1
		for i in range(60):
			for j in range(60):
				if (CL_Class[i]+CL_Class[j])==0:
					continue
				CL_Fnew=CL_F-CL_Max*weight[1]
				if CL_Class[i]>0:
					k=CL_Class[i]
					k1=int((k-1)/3)
					k2=(k-1)%3
					CL_Fnew-=RunTime[i+8][k1][k2]*weight[0]
					CL_Fnew+=RunTime[j+8][k1][k2]*weight[0]
				if CL_Class[j]>0:
					k=CL_Class[j]
					k1=int((k-1)/3)
					k2=(k-1)%3
					CL_Fnew-=RunTime[j+8][k1][k2]*weight[0]
					CL_Fnew+=RunTime[i+8][k1][k2]*weight[0]
				if state==1:
					CL_Max_new=0
					for m in range(60):
						k=CL_Class[m]
						if m==i: 
							k=CL_Class[j]
						if m==j: 
							k=CL_Class[i]
						if k>0:
							k1=int((k-1)/3)
							k2=(k-1)%3
							if RunTime[m+8][k1][k2]>CL_Max_new: 
								CL_Max_new=RunTime[m+8][k1][k2]
					CL_Fnew+=CL_Max_new*weight[1]
					#if CL_Max_new<CL_Max:

				if CL_Fnew<CL_F:
					CL_F=CL_Fnew
					CL_Class[i], CL_Class[j]= CL_Class[j], CL_Class[i]
					if state==1:
						CL_Max=CL_Max_new
					TorF=True
	CL_F_NoMax = 0
	for i in range(60):
		k=CL_Class[i]
		if k>0:
			k1=int((k-1)/3)
			k2=(k-1)%3
			CL_Cluster[k1][k2][CL_Clusternum[k1][k2]]=i+8
			CL_Clusternum[k1][k2]+=1
			CL_F_NoMax += RunTime[k1][i+8][k2]


	return CL_F, CL_F_NoMax, CL_Max, CL_Class, CL_Cluster



