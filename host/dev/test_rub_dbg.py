import importlib.util,numpy as np
path=r'C:\Users\18257\Desktop\Mocap\Xunbu_caphost_v6\THost\3D力测试上位机.pyw';spec=importlib.util.spec_from_file_location('m',path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
axis=np.array([.8,-.2,.3]);axis/=np.linalg.norm(axis);base=np.array([100.,200.,300.]);s=[]
for i in range(60):
 amp=12. if i<8 else (12. if i%2==0 else -12.);noise=np.random.default_rng(i).normal(0,.02,3);s.append(tuple(base+axis*amp+noise))
data=np.asarray(s);base=np.median(data,axis=0);centered=data-base;norms=np.linalg.norm(centered,axis=1);active=norms>max(np.percentile(norms,20),1e-12);cov=np.cov(centered[active],rowvar=False);ev,evec=np.linalg.eigh(cov);axis2=evec[:,np.argsort(ev)[-1]];onset=max(.5*float(np.percentile(norms,95)),1e-12);high=np.where(norms>onset)[0];first=int(high[0]);es=centered[first:min(first+8,len(centered))];ew=norms[first:min(first+8,len(centered))];early=np.average(es,axis=0,weights=np.maximum(ew,1e-12));print('first',first,'dot before',early@axis2,'early',early,'axis2',axis2);learn,r,g=m.extract_rubbing_direction(s);u=np.asarray(learn)/np.linalg.norm(learn);print('dot',u@axis,'learn',u,'axis',axis)

