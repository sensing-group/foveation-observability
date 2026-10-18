"""Required set + max-min allocation LP over the room, from the deployment geometry.
Definition (paper, Sec. III-C): smallest link set carrying 80% of Fresnel sensitivity at x.
Broadcast model: link (i,j) rate = b_i + b_j, sum b = B.  Uniform: 2B/T.
LP: max r s.t. b_i+b_j >= r on required links, sum b_i = B, b >= 0.
"""
import json, os, sys
from pathlib import Path
import numpy as np
from scipy.optimize import linprog

import model as crlb
POS = Path(__file__).resolve().parent.parent / "data" / "positions.json"
FRACTION = 0.80
B = 200.0
ROOM_W, ROOM_H, RES = 5.0, 4.0, 0.25

macs, nodes = crlb.load_positions(POS)
n = len(nodes); lam = crlb.C_LIGHT/crlb.CARRIER_HZ
links = [(i,j) for i in range(n) for j in range(i+1,n)]

def link_sens(x, metric):
    out = []
    for (i,j) in links:
        ni,nj = nodes[i],nodes[j]; d=np.linalg.norm(ni-nj); sf=crlb.FRESNEL_SIGMA_SCALE*np.sqrt(lam*d)
        di,dj=np.linalg.norm(x-ni),np.linalg.norm(x-nj); rho=di+dj-d
        w=np.exp(-rho**2/(2*sf**2))
        if metric=="w": out.append(w)
        else:
            g=(x-ni)/di+(x-nj)/dj; g=g*w*(-rho/sf**2); out.append(float(g@g))  # FIM trace contribution
    return np.array(out)

def required_set(x, metric):
    s=link_sens(x,metric); order=np.argsort(-s); cum=np.cumsum(s[order])/s.sum()
    k=int(np.searchsorted(cum, FRACTION)+1); return [links[o] for o in order[:k]]

def lp_maxmin(req):
    # variables: b_0..b_{n-1}, r ; maximize r
    c=np.zeros(n+1); c[-1]=-1
    A=[]; bub=[]
    for (i,j) in req:
        row=np.zeros(n+1); row[i]=-1; row[j]=-1; row[-1]=1; A.append(row); bub.append(0)
    Aeq=np.zeros((1,n+1)); Aeq[0,:n]=1
    res=linprog(c,A_ub=np.array(A),b_ub=bub,A_eq=Aeq,b_eq=[B],bounds=[(0,None)]*(n+1),method="highs")
    return res.x[-1], res.x[:n]

for metric in ("w","fim"):
    print(f"\n=== sensitivity metric: {metric} ===")
    rows=[]
    for x in np.arange(0.25,ROOM_W,RES):
        for y in np.arange(0.25,ROOM_H,RES):
            req=required_set(np.array([x,y]),metric); span=len({v for e in req for v in e})
            r,_=lp_maxmin(req); rows.append((x,y,len(req),span,r,2*B/span))
    rows=np.array(rows)
    k,span,r,uni=rows[:,2],rows[:,3],rows[:,4],rows[:,5]
    print(f"grid points {len(rows)}; |req| median {np.median(k):.0f} range {k.min():.0f}-{k.max():.0f}; span median {np.median(span):.0f}")
    print(f"LP max-min rate @B=200: median {np.median(r):.1f} Hz, min {r.min():.1f}, max {r.max():.1f}")
    print(f"uniform 2B/T_span:      median {np.median(uni):.1f} Hz")
    gain=r/ (2*B/n)
    print(f"LP gain over uniform-over-all-10 (40 Hz): median {np.median(gain):.2f}x, >=1.15x at {(gain>=1.15).mean()*100:.0f}% of points, >=46Hz (walking) at {(r>=46).mean()*100:.0f}% of points")
    m=(k==13)&(span==10)
    print(f"points with |req|=13 and span=10: {m.sum()}")
    if m.sum():
        print("  at those points LP max-min:", np.round(r[m],1))
    # room centre
    c=np.array([ROOM_W/2,ROOM_H/2]); req=required_set(c,metric); rr,b=lp_maxmin(req)
    print(f"room centre: |req|={len(req)} span={len({v for e in req for v in e})} LP={rr:.1f} Hz; alloc={np.round(b,0)}")
    deg=np.bincount([v for e in req for v in e],minlength=n); print("  degrees:",deg)
