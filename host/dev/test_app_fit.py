import importlib.util, math, numpy as np
path=r'C:\Users\18257\Desktop\Mocap\Xunbu_caphost_v6\THost\3D力测试上位机.pyw'
spec=importlib.util.spec_from_file_location('force_ui',path); m=importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
true_offset=np.array([12.,-7.,4.]); true_soft=np.array([[.8,.08,0],[.03,.92,.02],[0,.04,1.1]])
samples=[]
for theta in np.linspace(0.,math.pi,14):
 for phi in np.linspace(0.,2.*math.pi,18,endpoint=False):
  u=np.array([math.sin(theta)*math.cos(phi),math.sin(theta)*math.sin(phi),math.cos(theta)])
  samples.append(tuple(np.linalg.solve(true_soft,u)+true_offset))
off,soft,res=m.fit_ellipsoid(samples)
r=[m.vnorm(m.matrix_vector(soft,m.vsub(s,off))) for s in samples]
print('off',off,'soft',soft,'res',res,'radii',min(r),max(r),np.std(r))
