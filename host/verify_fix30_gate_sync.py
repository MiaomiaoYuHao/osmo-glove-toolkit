import importlib.util, tkinter as tk, sys
from pathlib import Path
APP=Path(r'THost').resolve(); sys.path.insert(0,str(APP))
spec=importlib.util.spec_from_file_location('pose_app',APP/'姿态测试上位机.pyw')
pose=importlib.util.module_from_spec(spec); spec.loader.exec_module(pose)
root=tk.Tk(); root.withdraw(); app=pose.PoseMonitor(root,auto_connect=False,auto_trace=False)
fields={'e_norm_x1000':1250,'e_dip_x1000':200,'e_step_x1000':300,'e_drift_x1000':0,'e_grad_x1000':100,'e_fleet_x1000':0,'weight_pct':100,'rejecting':0,'hard_event':0,'candidate_valid':0,'dip_valid':1,'radius1':34,'revision':19}
app._handle_firmware_line({'line':'synthetic P','parsed':{'type':'mag_status_row','fields':fields},'rx_monotonic_ns':1_000_000_000})
app._merge_mag_values({'clean':0,'trust':1.0,'hold_s':0.0,'dir_res_deg':1.0,'yaw_valid':1.0})
app._refresh_mag_live_gate_rows()
app.open_mag_diag_window(); root.update()
app._refresh_mag_correction_state(3, 1.0, 0, 1_000_000_000)
assert '已生效' not in app.mag_correction_var.get(), app.mag_correction_var.get()
assert '暂停' in app.mag_correction_var.get(), app.mag_correction_var.get()
text=app.mag_feature_text.get('1.0',tk.END)
assert '磁场强度超范围' in text, text
assert '当前:' not in text, text
assert '门限:' not in text, text
app._merge_mag_values({'p_e_norm_x1000':500})
app.mag_live_last_update = 0.0
app._refresh_mag_live_gate_rows()
root.update()
text_after = app.mag_feature_text.get('1.0', tk.END)
assert '磁场强度超范围' not in text_after, text_after
app._refresh_mag_correction_state(3, 1.0, 0, 2_000_000_000)
assert app.mag_correction_var.get() == '磁修正：已生效', app.mag_correction_var.get()
print('FIX30_GATE_UI_PASS=True')
print('ACTIVE_ONLY_AND_DISAPPEARS=True')
app.on_close()
