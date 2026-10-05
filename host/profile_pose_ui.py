from __future__ import annotations
import cProfile, importlib.util, io, pstats, sys, tempfile, time, tkinter as tk
from pathlib import Path
APP=Path(__file__).resolve().parent
sys.path.insert(0,str(APP))
spec=importlib.util.spec_from_file_location('pose_app',APP/'姿态测试上位机.pyw')
pose=importlib.util.module_from_spec(spec); spec.loader.exec_module(pose)

def row(i,kind,ns):
 r={'host_time':time.time(),'index':i,'finger':'index','finger_id':4,'link':1,'sensor_id':1,'kind':kind,
 'mag_x':None,'mag_y':None,'mag_z':None,'quat_x':None,'quat_y':None,'quat_z':None,'quat_w':None,'quat_accuracy':None,
 '_trace':{'trace_id':i,'connection_id':1,'sequence':i,'rx_wall_time_ns':time.time_ns(),'rx_monotonic_ns':ns,'packet_len':32}}
 if kind=='QUAT': r.update({'quat_x':0.0,'quat_y':0.0,'quat_z':0.0,'quat_w':1.0,'quat_accuracy':13.9801,'packet':{'quat':{'x':0.0,'y':0.0,'z':0.0,'w':1.0,'accuracy':13.9801}}})
 else: r.update({'mag_x':-50.0,'mag_y':10.0,'mag_z':20.0,'packet':{'mag':{'x':-50.0,'y':10.0,'z':20.0}}})
 return r

def main():
 root=tk.Tk(); root.geometry('1100x740'); app=pose.PoseMonitor(root,auto_connect=False,auto_trace=False)
 temp=tempfile.TemporaryDirectory(prefix='pose_profile_',dir=APP/'diagnostics')
 app.trace.start_session(temp.name,metadata={'profile':True})
 total=2400; base=1_000_000_000
 for i in range(total):
  kind='QUAT' if i%2==0 else 'MAG'
  app.events.put((kind.lower(),row(i+1,kind,base+i*5_000_000)))
 state={'done':0}
 def pump():
  batch=min(60,total-state['done']); app._drain_events(batch); state['done']+=batch
  if state['done']<total: root.after(5,pump)
  else: root.after(500,root.quit)
 root.after(10,pump)
 prof=cProfile.Profile(); prof.enable(); root.mainloop(); prof.disable()
 app.trace.stop_session('profile'); print('PROCESSED',state['done'],'QUEUE',app.events.qsize(),'TRACE_Q',app.trace.stats()['queue_depth'])
 s=io.StringIO(); pstats.Stats(prof,stream=s).sort_stats('cumulative').print_stats(35); print(s.getvalue())
 s=io.StringIO(); pstats.Stats(prof,stream=s).sort_stats('tottime').print_stats(25); print(s.getvalue())
 temp.cleanup(); app.on_close(); return 0
if __name__=='__main__': raise SystemExit(main())
