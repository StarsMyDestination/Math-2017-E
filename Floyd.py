import numpy as np
def Floyd (Dist):
    Path=np.zeros((Nnode,Nnode), dtype=np.int16)
    for i in range(Nnode):
        for j in range(Nnode):
            Path[i][j]=i
    for k in range(Nnode):
        for i in range(Nnode):
            for j in range(Nnode):
                if (Dist[i][k]+Dist[k][j]<Dist[i][j]):
                    Dist[i][j]=Dist[i][k]+Dist[k][j]
                    Path[i][j]=Path[k][j]
    return Dist, Path
