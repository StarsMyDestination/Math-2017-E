import numpy as np


def CL1(RunTime, state=None, weight=None):
    CL_Class = np.zeros(60, dtype=np.int8)
    CL_Cluster = np.zeros((2, 3, 6), dtype=np.int16)
    CL_Clusternum = np.zeros((2, 3), dtype=np.int8)
    CL_F = 0
    CL_Fnew = 0

    # init status of 60 node F (make first 24 nodes to be 1 1 1 2 2 2 3 3 3 3 3 3 4 4 4 5 5 5 6 6 6 6 6 6)
    for i in range(3):
        CL_Class[i] = 1
        CL_Class[i + 3] = 2
        CL_Class[i + 12] = 4
        CL_Class[i + 15] = 5
    for i in range(6):
        CL_Class[i + 6] = 3
        CL_Class[i + 18] = 6

    #init time cost, for initial status above, compute initial overall time cost
    for i in range(60):
        k = CL_Class[i]
        if k > 0:
            k1 = int((k - 1) / 3)  # 0 or 1 represents D1 or D2
            k2 = (k - 1) % 3  # 0, 1 or 2 represents vehicle A, B or C
            CL_F += RunTime[i + 8][k1][k2]

    TorF = True
    while TorF:
        TorF = False
        for i in range(60):
            for j in range(60):
                if (CL_Class[i] + CL_Class[j]) == 0:
                    continue
                CL_Fnew = CL_F
                if CL_Class[i] > 0:
                    k = CL_Class[i]
                    k1 = int((k - 1) / 3)
                    k2 = (k - 1) % 3
                    CL_Fnew -= RunTime[i + 8][k1][k2]
                    CL_Fnew += RunTime[j + 8][k1][k2]
                if CL_Class[j] > 0:
                    k = CL_Class[j]
                    k1 = int((k - 1) / 3)
                    k2 = (k - 1) % 3
                    CL_Fnew -= RunTime[j + 8][k1][k2]
                    CL_Fnew += RunTime[i + 8][k1][k2]
                if CL_Fnew < CL_F:
                    CL_F = CL_Fnew
                    CL_Class[i], CL_Class[j] = CL_Class[j], CL_Class[i]
                    TorF = True
    for i in range(60):
        k = CL_Class[i]
        if k > 0:
            k1 = int((k - 1) / 3)
            k2 = (k - 1) % 3
            CL_Cluster[k1][k2][CL_Clusternum[k1][k2]] = i + 8
            CL_Clusternum[k1][k2] += 1

    return CL_F, CL_Class, CL_Cluster
