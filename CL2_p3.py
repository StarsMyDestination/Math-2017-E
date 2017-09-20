import numpy as np
import utils


def CL2(RunTime, state, weight, Start, End, Znum, ReplaceCar):
    CL_Class = -np.ones((36, 9), dtype=np.int8)
    CL_Cluster = -np.ones((24, 2), dtype=np.int16)
    CL_F = 0
    CL_Fnew = 0

    for i in range(24):
        CL_Class[i][0] = i
    for i in range(3):
        CL_Class[ReplaceCar[i]][0] = -1
        CL_Class[ReplaceCar[i]][6+i] = ReplaceCar[i]
    for i in range(3):
        Start[ReplaceCar[i]] = Znum[6+i]
    for i in range(36):
        for j in range(9):
            k = CL_Class[i][j]
            if k > -1:
                k2, _ = utils.get_vehicle_info(k)
                CL_F += RunTime[Znum[j]][Start[k]][k2]
                CL_F += RunTime[End[i]][Znum[j]][k2]

    TorF = True
    while TorF:
        TorF = False
        for i in range(36):
            for ik in range(9):
                for j in range(36):
                    for jk in range(9):
                        if (ik<6)&(jk>=6):
                            continue
                        if (ik>=6)&(jk<6):
                            continue
                        if (ik>=6)&(ik!=jk):
                            continue
                        if (jk>=6)&(ik!=jk):
                            continue
                        if (CL_Class[i][ik] + CL_Class[j][jk]) == -2:
                            continue
                        if CL_Class[i][ik] == -1:
                            if np.sum(CL_Class[i]) - CL_Class[i][ik] > -8:
                                continue
                        else:
                            if CL_Class[j][jk] == -1:
                                if np.sum(CL_Class[j]) - CL_Class[j][jk] > -8:
                                    continue

                        CL_Fnew = CL_F
                        if CL_Class[i][ik] > -1:
                            k = CL_Class[i][ik]
                            k2, _ = utils.get_vehicle_info(k)
                            CL_Fnew -= RunTime[Znum[ik]][Start[k]][k2]
                            CL_Fnew -= RunTime[End[i]][Znum[ik]][k2]
                            CL_Fnew += RunTime[Znum[jk]][Start[k]][k2]
                            CL_Fnew += RunTime[End[j]][Znum[jk]][k2]
                        if CL_Class[j][jk] > -1:
                            k = CL_Class[j][jk]
                            k2, _ = utils.get_vehicle_info(k)
                            CL_Fnew -= RunTime[Znum[jk]][Start[k]][k2]
                            CL_Fnew -= RunTime[End[j]][Znum[jk]][k2]
                            CL_Fnew += RunTime[Znum[ik]][Start[k]][k2]
                            CL_Fnew += RunTime[End[i]][Znum[ik]][k2]
                        if CL_Fnew < CL_F - 1e-6:
                            CL_F = CL_Fnew
                            CL_Class[i][ik], CL_Class[j][jk] = CL_Class[j][jk], CL_Class[i][ik]
                            TorF = True
        # print (CL_F, CL_Fnew)

    for i in range(36):
        for j in range(9):
            k = CL_Class[i][j]
            if k > -1:
                CL_Cluster[k][0] = End[i]
                CL_Cluster[k][1] = Znum[j]

    return CL_F, CL_Class, CL_Cluster
