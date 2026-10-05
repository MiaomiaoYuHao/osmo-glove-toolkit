#!/usr/bin/env python3
from __future__ import annotations
import importlib.util, sys, tempfile, time, tkinter as tk
from pathlib import Path
APP=Path(__file__).resolve().parent
sys.path.insert(0,str(APP))
spec=importlib.util.spec_from_file_location('pose_app',APP/'姿态测试上位机.pyw')
pose=importlib.util.module_from_spec(spec); spec.loader.exec_module(pose)

def row(i, kind, ns):
    r={'host_time':time.time(),'index':i,'finger':'index','finger_id':4,'link':1,'sensor_id':1,'kind':kind,
       'mag_x':None,'mag_y':None,'mag_z':None,'quat_x':None,'quat_y':None,'quat_z':None,'quat_w':None,'quat_accuracy':None,
       '_trace':{'trace_id':i,'connection_id':1,'sequence':i,'rx_wall_time_ns':time.time_ns(),'rx_monotonic_ns':ns,'packet_len':32}}
    if kind=='QUAT':
        r.update({'quat_x':0.0,'quat_y':0.0,'quat_z':0.0,'quat_w':1.0,'quat_accuracy':13.9801,
                  'packet':{'quat':{'x':0.0,'y':0.0,'z':0.0,'w':1.0,'accuracy':13.9801}}})
    else:
        r.update({'mag_x':-50.0,'mag_y':10.0,'mag_z':20.0,'packet':{'mag':{'x':-50.0,'y':10.0,'z':20.0}}})
    return r

def main():
    root=tk.Tk(); root.geometry("1100x740"); app=pose.PoseMonitor(root,auto_connect=False,auto_trace=False)
    test_root=APP/'diagnostics'
    with tempfile.TemporaryDirectory(prefix='ui_throughput_test_',dir=test_root) as temp:
        app.trace.start_session(temp,metadata={'synthetic':True})
        total=3000; base=1_000_000_000
        for i in range(total):
            kind='QUAT' if i%2==0 else 'MAG'
            app.events.put((kind.lower(),row(i+1,kind,base+i*10_000_000)))
        start=time.perf_counter(); done=0
        while done<total:
            batch=min(60,total-done); app._drain_events(batch); done+=batch; root.update(); time.sleep(.002)
        elapsed=time.perf_counter()-start
        app.trace.stop_session('throughput_test')
        assert app.events.empty(), app.events.qsize()
        assert elapsed<8.0, elapsed
        print('UI_THROUGHPUT_PASS=True'); print(f'ELAPSED_S={elapsed:.3f}'); print(f'EVENTS_PER_SECOND={total/elapsed:.1f}')
    app.on_close(); return 0
if __name__=='__main__': raise SystemExit(main())
