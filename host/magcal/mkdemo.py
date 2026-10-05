import numpy as np
import magcal as mc
rng = np.random.default_rng(3)
N=3000
P=rng.normal(size=(3,3)); P=P.T@P+np.eye(3)*1.5
S=mc.sym_sqrt(P); W=S/np.cbrt(np.linalg.det(S))
b=np.array([80.,-260.,40.]); r=2100.
u=rng.normal(size=(N,3)); u/=np.linalg.norm(u,axis=1,keepdims=True)
m=(np.linalg.solve(W,u.T).T)*r+b+rng.normal(scale=5,size=(N,3))
k=int(N*0.08); i=rng.choice(N,k,replace=False); m[i]+=rng.normal(scale=300,size=(k,3))
q=rng.normal(size=(N,4)); q/=np.linalg.norm(q,axis=1,keepdims=True)
np.savez_compressed("demo_sweep.npz", mag=m, quat=q, t_mag=np.arange(N)*0.01, t_quat=np.arange(N)*0.01, dt=np.zeros(N))
print("demo_sweep.npz 生成:", N, "样本")
