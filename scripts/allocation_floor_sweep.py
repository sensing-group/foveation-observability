"""Floor-constrained allocation, threshold robustness, detectability field. Same model as Fig. 4."""
import json, os
from pathlib import Path
import numpy as np
from scipy.optimize import linprog
import model as crlb
macs,nodes=crlb.load_positions(Path(__file__).resolve().parent.parent/"data"/"positions.json")
n=len(nodes); lam=crlb.C_LIGHT/crlb.CARRIER_HZ; links=[(i,j) for i in range(n) for j in range(i+1,n)]
B=200.0; RATE_WALK=46.0; ROOM_W,ROOM_H=5.0,4.0
def sens(x):
    out=[]
    for (i,j) in links:
        ni,nj=nodes[i],nodes[j]; d=np.linalg.norm(ni-nj); sf=crlb.FRESNEL_SIGMA_SCALE*np.sqrt(lam*d)
        rho=np.linalg.norm(x-ni)+np.linalg.norm(x-nj)-d; out.append(np.exp(-rho**2/(2*sf**2)))
    return np.array(out)
def req(x,frac):
    s=sens(x); o=np.argsort(-s); c=np.cumsum(s[o])/s.sum(); k=int(np.searchsorted(c,frac)+1); return [links[t] for t in o[:k]]
def lp(reqset,beta):
    # b_i >= beta*B/n for every node (periphery floor as a fraction of the uniform share)
    c=np.zeros(n+1); c[-1]=-1; A=[];bub=[]
    for (i,j) in reqset:
        r=np.zeros(n+1); r[i]=-1; r[j]=-1; r[-1]=1; A.append(r); bub.append(0)
    Aeq=np.zeros((1,n+1)); Aeq[0,:n]=1
    res=linprog(c,A_ub=np.array(A),b_ub=bub,A_eq=Aeq,b_eq=[B],bounds=[(beta*B/n,None)]*n+[(0,None)],method="highs")
    return res.x[-1]
grid=[(x,y) for x in np.arange(0.25,ROOM_W,0.25) for y in np.arange(0.25,ROOM_H,0.25)]
print("== threshold robustness (beta=0, B=200): fraction of room points where LP optimum >= 46 Hz ==")
for frac in (0.6,0.7,0.8,0.9):
    r=np.array([lp(req(np.array(g),frac),0.0) for g in grid]); print(f"frac {frac}: pass {100*(r>=RATE_WALK).mean():.0f}%  median {np.median(r):.0f} Hz  min {r.min():.0f}")
print("== floor sweep (frac=0.8, B=200): beta = every node's share as a fraction of uniform ==")
for beta in (0,0.1,0.2,0.3,0.4,0.5,0.6,0.8,1.0):
    r=np.array([lp(req(np.array(g),0.8),beta) for g in grid]); print(f"beta {beta:.1f}: pass {100*(r>=RATE_WALK).mean():3.0f}%  median {np.median(r):5.1f} Hz  min {r.min():5.1f}  dark links (two unfunded) at beta=0 n/a")
print("== budget needed at the room centre and worst-case point for 46 Hz, beta in {0,0.25,0.5} ==")
for beta in (0,0.25,0.5):
    Bs=[]
    for g in grid:
        rs=req(np.array(g),0.8); r1=lp(rs,beta)  # linear in B
        Bs.append(RATE_WALK/r1*B)
    Bs=np.array(Bs); c=np.array([ROOM_W/2,ROOM_H/2]); rc=lp(req(c,0.8),beta)
    print(f"beta {beta}: B* centre {RATE_WALK/rc*B:.0f}/s  median {np.median(Bs):.0f}/s  p90 {np.percentile(Bs,90):.0f}/s  (uniform needs 230/s)")
print("== detectability field vs localizability (10 cm grid, full room) ==")
xs=np.arange(0,ROOM_W+1e-9,crlb.GRID_RES_M); ys=np.arange(0,ROOM_H+1e-9,crlb.GRID_RES_M)
X,Y=np.meshgrid(xs,ys,indexing="ij"); G=np.column_stack([X.ravel(),Y.ravel()])
R=crlb.crlb(crlb.fim(G,nodes)); snr=crlb.SIGNAL_REL/crlb.NOISE_REL
W=np.array([sens(g) for g in G]); defl=np.sqrt((W**2).sum(1))*snr   # deflection of a presence test, one sample per link
notloc=R>crlb.BLIND_THRESHOLD_M
for thr in (2.0,3.0,5.0):
    notdet=defl<thr
    print(f"deflection thr {thr}: not detectable {100*notdet.mean():.0f}% | not localizable {100*notloc.mean():.0f}% | detectable but not localizable {100*(notloc&~notdet).mean():.0f}% | neither {100*(notloc&notdet).mean():.0f}%")
print(f"deflection median {np.median(defl):.1f}, p10 {np.percentile(defl,10):.1f}, min {defl.min():.1f}")
