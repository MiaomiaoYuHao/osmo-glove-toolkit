import numpy as np

def rate_report(name, t):
    t = np.sort(np.asarray(t)); d = np.diff(t); d = d[d > 0]
    med = float(np.median(d)); span = float(t[-1] - t[0]); rate = (len(t)-1)/span
    gaps = d > med*1.5
    lost = int(np.sum(np.round(d[gaps]/med)-1)) if gaps.any() else 0
    print("%-10s %7.1f Hz  med %6.2fms  p95 %6.2fms  max %8.2fms" % (name, rate, med*1000, np.percentile(d,95)*1000, d.max()*1000))
    print("%-10s 丢帧 %d 处  约 %d 样本  (%.2f%%)" % ("", int(gaps.sum()), lost, lost/len(t)*100))

rate_report("干净", np.arange(600)*0.01)
t = np.arange(600)*0.01; t[300:] += 0.5
rate_report("丢50个", t)
np.random.seed(1); t=[]; x=0.0
for i in range(600):
    if np.random.rand() > 0.03: t.append(x)
    x += 0.01
rate_report("随机丢3%", np.array(t))
