import csv, numpy as np, sys
sys.path.insert(0, r"C:\Users\18257\Desktop\Mocap\Xunbu_caphost_v6\_osmo_glove_research\labs\glove2robot")
import magcal as mc
rng = np.random.default_rng(5)

P=rng.normal(size=(3,3)); P=P.T@P+np.eye(3)*1.5
S=mc.sym_sqrt(P); W=S/np.cbrt(np.linalg.det(S))
b=np.array([80.,-260.,40.]); r=2100.
N=2500
u=rng.normal(size=(N,3)); u/=np.linalg.norm(u,axis=1,keepdims=True)
m=(np.linalg.solve(W,u.T).T)*r+b+rng.normal(scale=5,size=(N,3))
k=int(N*0.08); i=rng.choice(N,k,replace=False); m[i]+=rng.normal(scale=300,size=(k,3))
q=rng.normal(size=(N,4)); q/=np.linalg.norm(q,axis=1,keepdims=True)

cols = ["record_kind","mag_device_time_s","mag_x_raw","mag_y_raw","mag_z_raw",
        "quat_device_time_s","quat_w_raw","quat_x_raw","quat_y_raw","quat_z_raw",
        "quat_w_norm","quat_x_norm","quat_y_norm","quat_z_norm"]
with open("fake_analysis.csv","w",newline="",encoding="utf-8") as fh:
    w=csv.DictWriter(fh,fieldnames=cols); w.writeheader()
    for n in range(N):
        t=n*0.01
        w.writerow({"record_kind":"mag","mag_device_time_s":t,
                    "mag_x_raw":m[n,0],"mag_y_raw":m[n,1],"mag_z_raw":m[n,2]})
        w.writerow({"record_kind":"quat","quat_device_time_s":t+0.001,
                    "quat_w_raw":q[n,0]*16384,"quat_x_raw":q[n,1]*16384,
                    "quat_y_raw":q[n,2]*16384,"quat_z_raw":q[n,3]*16384,
                    "quat_w_norm":q[n,0],"quat_x_norm":q[n,1],
                    "quat_y_norm":q[n,2],"quat_z_norm":q[n,3]})
print("fake_analysis.csv 已生成:", N*2, "行")
