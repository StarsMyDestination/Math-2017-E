import numpy as np
import utils


def CL2(RunTime, state, weight, Start, End):
    CL_Class = -np.ones((36, 6), dtype=np.int8)
    CL_Cluster = -np.ones((24, 2), dtype=np.int16)
    CL_F = 0
    CL_Fnew = 0
    CL_Max = 0
    if state==0:
        weight=np.array([1.0,0.0])
    else:
        weight[0]=weight[0]/24

    for i in range(4):
        for j in range(6):
            CL_Class[i*6+j][j] = i*6+j
    for i in range(36):
        for j in range(6):
            k = CL_Class[i][j]
            if k > -1:
                k2, _ = utils.get_vehicle_info(k)
                CL_F += RunTime[j + 2][Start[k]][k2]*weight[0]
                CL_F += RunTime[End[i]][j + 2][k2]*weight[0]
                if (RunTime[j + 2][Start[k]][k2]+RunTime[End[i]][j + 2][k2])>CL_Max:
                    CL_Max = RunTime[j + 2][Start[k]][k2]+RunTime[End[i]][j + 2][k2]
    CL_F += CL_Max*weight[1]

    TorF = True
    while TorF:
        TorF = False
        for i in range(36):
            for ik in range(6):
                for j in range(36):
                    for jk in range(6):
                        if (CL_Class[i][ik] + CL_Class[j][jk]) == -2:
                            continue
                        if (i == j)&(ik == jk):
                            continue
                        if CL_Class[i][ik] == -1:
                            if np.sum(CL_Class[i]) - CL_Class[i][ik] > -5:
                                continue
                        else:
                            if CL_Class[j][jk] == -1:
                                if np.sum(CL_Class[j]) - CL_Class[j][jk] > -5:
                                    continue
                        if CL_Class[i][ik] == -1:
                            tmp = 0
                            for jkk in range(36):
                                if CL_Class[jkk][jk] > -1:
                                    tmp += 1
                            if tmp <= 3:
                                continue
                            tmp = 0
                            for ikk in range(36):
                                if CL_Class[ikk][ik] > -1:
                                    tmp += 1
                            if tmp >= 5:
                                continue
                        else:
                            if CL_Class[j][jk] == -1:
                                tmp = 0
                                for ikk in range(36):
                                    if CL_Class[ikk][ik] > -1:
                                        tmp += 1
                                if tmp <= 3:
                                    continue
                                tmp = 0
                                for jkk in range(36):
                                    if CL_Class[jkk][jk] > -1:
                                        tmp += 1
                                if tmp >= 5:
                                    continue


                        CL_Fnew = CL_F - CL_Max*weight[1]
                        if CL_Class[i][ik] > -1:
                            k = CL_Class[i][ik]
                            k2, _ = utils.get_vehicle_info(k)
                            CL_Fnew -= RunTime[ik + 2][Start[k]][k2]*weight[0]
                            CL_Fnew -= RunTime[End[i]][ik + 2][k2]*weight[0]
                            CL_Fnew += RunTime[jk + 2][Start[k]][k2]*weight[0]
                            CL_Fnew += RunTime[End[j]][jk + 2][k2]*weight[0]
                        if CL_Class[j][jk] > -1:
                            k = CL_Class[j][jk]
                            k2, _ = utils.get_vehicle_info(k)
                            CL_Fnew -= RunTime[jk + 2][Start[k]][k2]*weight[0]
                            CL_Fnew -= RunTime[End[j]][jk + 2][k2]*weight[0]
                            CL_Fnew += RunTime[ik + 2][Start[k]][k2]*weight[0]
                            CL_Fnew += RunTime[End[i]][ik + 2][k2]*weight[0]
                        if state == 1:
                            CL_Max_new = 0
                            for m in range(36):
                                 for n in range(6):
                                    k = CL_Class[m][n]
                                    if (m==i)&(n==ik):
                                        k = CL_Class[j][jk]
                                    if (m==j)&(n==jk):
                                        k = CL_Class[i][ik]
                                    if k > -1:
                                        k2, _ = utils.get_vehicle_info(k)
                                        if (RunTime[n+2][Start[k]][k2]+RunTime[End[m]][n+2][k2]) > CL_Max_new:
                                            CL_Max_new = RunTime[n+2][Start[k]][k2]+RunTime[End[m]][n+2][k2]
                            CL_Fnew += CL_Max_new*weight[1]
                        #print (CL_F, CL_Fnew, CL_Max, CL_Max_new)
                        if CL_Fnew < CL_F - 1e-9:
                            CL_F = CL_Fnew
                            CL_Class[i][ik], CL_Class[j][jk] = CL_Class[j][jk], CL_Class[i][ik]
                            if state == 1:
                                CL_Max = CL_Max_new
                            TorF = True
    CL_F_NoMax = 0
    CL_Max = 0
    for i in range(36):
        for j in range(6):
            k = CL_Class[i][j]
            if k > -1:
                k2, _ = utils.get_vehicle_info(k)
                CL_Cluster[k][0] = End[i]
                CL_Cluster[k][1] = j + 2
                CL_F_NoMax += RunTime[j+2][Start[k]][k2]+RunTime[End[i]][j+2][k2]
                if (RunTime[j+2][Start[k]][k2]+RunTime[End[i]][j+2][k2]) > CL_Max:
                    CL_Max = RunTime[j+2][Start[k]][k2]+RunTime[End[i]][j+2][k2]

    return CL_F, CL_F_NoMax, CL_Max, CL_Class, CL_Cluster
