import numpy as np
import utils


def CL2(RunTime, state, weight, Start, End):
    CL_Class = -np.ones((36, 6), dtype=np.int8)
    CL_Cluster = -np.ones((24, 2), dtype=np.int16)
    CL_F = 0
    CL_Fnew = 0

    for i in range(24):
        CL_Class[i][0] = i
    for i in range(36):
        for j in range(6):
            k = CL_Class[i][j]
            if k > -1:
                k2, _ = utils.get_vehicle_info(k)
                CL_F += RunTime[j + 2][Start[k]][k2]
                CL_F += RunTime[End[i]][j + 2][k2]

    TorF = True
    while TorF:
        TorF = False
        for i in range(36):
            for ik in range(6):
                for j in range(36):
                    for jk in range(6):
                        if (CL_Class[i][ik] + CL_Class[j][jk]) == -2:
                            continue
                        if CL_Class[i][ik] == -1:
                            if np.sum(CL_Class[i]) - CL_Class[i][ik] > -5:
                                continue
                        else:
                            if CL_Class[j][jk] == -1:
                                if np.sum(CL_Class[j]) - CL_Class[j][jk] > -5:
                                    continue

                        CL_Fnew = CL_F
                        if CL_Class[i][ik] > -1:
                            k = CL_Class[i][ik]
                            k2, _ = utils.get_vehicle_info(k)
                            CL_Fnew -= RunTime[ik + 2][Start[k]][k2]
                            CL_Fnew -= RunTime[End[i]][ik + 2][k2]
                            CL_Fnew += RunTime[jk + 2][Start[k]][k2]
                            CL_Fnew += RunTime[End[j]][jk + 2][k2]
                        if CL_Class[j][jk] > -1:
                            k = CL_Class[j][jk]
                            k2, _ = utils.get_vehicle_info(k)
                            CL_Fnew -= RunTime[jk + 2][Start[k]][k2]
                            CL_Fnew -= RunTime[End[j]][jk + 2][k2]
                            CL_Fnew += RunTime[ik + 2][Start[k]][k2]
                            CL_Fnew += RunTime[End[i]][ik + 2][k2]
                        if CL_Fnew < CL_F:
                            CL_F = CL_Fnew
                            CL_Class[i][ik], CL_Class[j][jk] = CL_Class[j][jk], CL_Class[i][ik]
                            TorF = True
    for i in range(36):
        for j in range(6):
            k = CL_Class[i][j]
            if k > -1:
                CL_Cluster[k][0] = End[i]
                CL_Cluster[k][1] = j + 2

    return CL_F, CL_Class, CL_Cluster
