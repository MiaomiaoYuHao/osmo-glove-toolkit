import math, numpy as np
true_offset=np.array([12.,-7.,4.]); true_soft=np.array([[.8,.08,0],[.03,.92,.02],[0,.04,1.1]])
samples=[]
for theta in np.linspace(0.,math.pi,14):
 for phi in np.linspace(0.,2.*math.pi,18,endpoint=False):
  unit=np.array([math.sin(theta)*math.cos(phi),math.sin(theta)*math.sin(phi),math.cos(theta)])
  samples.append(np.linalg.solve(true_soft,unit)+true_offset)
x=np.asarray(samples)
center0=np.median(x,axis=0); centered=x-center0; scale0=float(np.median(np.linalg.norm(centered,axis=1))); print('center0',center0,'scale0',scale0,'span',np.percentile(x,95,axis=0)-np.percentile(x,5,axis=0))
mask=np.ones(len(x),bool)
for k in range(3):
 points=centered[mask]/scale0; px,py,pz=points[:,0],points[:,1],points[:,2]
 D=np.column_stack((px*px,py*py,pz*pz,2*px*py,2*px*pz,2*py*pz,px,py,pz))
 coeff,*_=np.linalg.lstsq(D,np.ones(len(points)),rcond=None); print('coeff',coeff)
 A=np.array([[coeff[0],coeff[3],coeff[4]],[coeff[3],coeff[1],coeff[5]],[coeff[4],coeff[5],coeff[2]]]); b=coeff[6:9]
 cp=-.5*np.linalg.solve(A,b); constant=float(cp@A@cp-1); ev,evec=np.linalg.eigh(A); print('A',A,'b',b,'cp',cp,'constant',constant,'ev',ev)

invsqrt=evec@np.diag(1/np.sqrt(ev))@evec.T
for const in (1.0, 1.0/max(ev), 1.0/min(ev)):
 W=invsqrt/np.sqrt(const); radii=np.linalg.norm((x-(center0+scale0*cp))@W.T,axis=1); print('const',const,'radii minmax std',radii.min(),radii.max(),radii.std())
