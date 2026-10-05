import csv, numpy as np, sys
sys.path.insert(0, r"C:\Users\18257\Desktop\Mocap\Xunbu_caphost_v6\_osmo_glove_research\labs\glove2robot")
import magcal as mc
rng = np.random.default_rng(9)
P=rng.normal(size=(3,3)); P=P.T@P+np.eye(3)*1.5
S=mc.sym_sqrt(P); W=S/np.cbrt(np.linalg.det(S))
b=np.array([80.,-260.,40.]); r=2100.
N=3000
u=rng.normal(size=(N,3)); u/=np.linalg.norm(u,axis=1,keepdims=True)
m=(np.linalg.solve(W,u.T).T)*r+b+rng.normal(scale=5,size=(N,3))
k=int(N*0.08); i=rng.choice(N,k,replace=False); m[i]+=rng.normal(scale=300,size=(k,3))
q=rng.normal(size=(N,4)); q/=np.linalg.norm(q,axis=1,keepdims=True)

# A) 轻量格式（上位机【标定采集】按钮产出的就是这个）
with open("cal_demo.csv","w",newline="",encoding="utf-8") as fh:
    w=csv.writer(fh); w.writerow(["kind","time","x","y","z","w"])
    for n in range(N):
        t=n*0.01
        w.writerow(["M", f"{t:.7f}", m[n,0], m[n,1], m[n,2], ""])
        w.writerow(["Q", f"{t+0.001:.7f}", q[n,1], q[n,2], q[n,3], q[n,0]])
print("cal_demo.csv 已生成:", N*2, "行")
