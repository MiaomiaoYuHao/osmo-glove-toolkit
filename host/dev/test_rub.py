import importlib.util,numpy as np
path=r'C:\Users\18257\Desktop\Mocap\Xunbu_caphost_v6\THost\3D力测试上位机.pyw';spec=importlib.util.spec_from_file_location('m',path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
axis=np.array([.8,-.2,.3]);axis/=np.linalg.norm(axis);base=np.array([100.,200.,300.]);s=[]
for i in range(60):
 amp=12. if i<8 else (12. if i%2==0 else -12.);noise=np.random.default_rng(i).normal(0,.02,3);s.append(tuple(base+axis*amp+noise))
learn,r,g=m.extract_rubbing_direction(s);u=np.asarray(learn)/np.linalg.norm(learn);print('dot',u@axis,'learn',u,'axis',axis)
