/* 剧烈甩动仿真。真实姿态做 3 Hz、峰值 900 deg/s 的往复甩动（斜轴），
 * 六轴 yaw 误差按 |omega| 的 1.2% 累积（饱和/标度因子类误差），
 * 磁力计 0.5% 噪声，mag 与 quat 配对错一拍（固件的 fresh 配对方式）。
 * 输出：运动中误差峰值、停止时误差、停止后回到 2 度以内所需时间。 */
#ifndef MF_SRC
#define MF_SRC "../Src/glove/mag_fusion.c"
#endif
#include MF_SRC
#include <stdio.h>
#include <stdlib.h>
#define D2R (MF_PI/180.0f)
#define R2D (180.0f/MF_PI)
#ifndef SKEW
#define SKEW 1          /* mag 比 quat 晚几拍配对 */
#endif
#ifndef TS_ERR_MS
#define TS_ERR_MS 0.0f
#endif
#ifndef TS_JIT_MS
#define TS_JIT_MS 0.0f
#endif
#ifndef USE_TS
#define USE_TS 1        /* 1 = 调用带时间戳的新接口 */
#endif
static uint32_t rs=12345u;
static float gauss(void){ float u1,u2; rs=rs*1664525u+1013904223u; u1=((rs>>8)+1)/16777217.0f;
  rs=rs*1664525u+1013904223u; u2=((rs>>8)+1)/16777217.0f; return sqrtf(-2*logf(u1))*cosf(2*MF_PI*u2); }
static void qmul(const float a[4],const float b[4],float o[4]){
  o[0]=a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3]; o[1]=a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2];
  o[2]=a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1]; o[3]=a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]; }
static void qnorm(float q[4]){ float n=sqrtf(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]); for(int i=0;i<4;i++) q[i]/=n; }
static float yaw_err(const float qo[4],const float qt[4]){ /* 世界系绕 Z 的误差 */
  float qi[4]={qt[0],-qt[1],-qt[2],-qt[3]}, e[4]; qmul(qo,qi,e);
  return 2.0f*atan2f(e[3],e[0]); }
#define N 1400
int main(int argc,char**argv){
  float PEAK = (argc>1)? atof(argv[1]) : 900.0f;
  static float hist_q[64][4]; int hi=0;
  mag_fusion_t f; uint64_t tick=64000; float qt[4]={1,0,0,0}, err=0;
  float ax[3]={0.45f,0.35f,0.82f}; { float n=sqrtf(ax[0]*ax[0]+ax[1]*ax[1]+ax[2]*ax[2]); for(int i=0;i<3;i++) ax[i]/=n; }
  float dipt=-55.0f*D2R, w_e[3]={cosf(dipt),0,sinf(dipt)};
  mag_fusion_init(&f);
  mf_m3_identity(f.cal[0].W); f.cal[0].radius=1; f.cal[0].valid=1u; f.cal[0].active=0u;
  f.cal[0].revision=1u; f.cal[0].model=2u; f.fe.align_valid=1u;
  f.ref_dip=f.ref_dip0=dipt; f.ref_dip_valid=1u; f.ref_norm=f.ref_norm0=1.0f;
  f.learn_state=MAG_LEARN_LOCKED;
  float T_on=5.0f, T_off=8.0f, peak_err=0, err_at_stop=0, t_ok=-1, maxw0=0;
  int w_zero_in_motion=0, n_motion=0;
  for(int k=0;k<N*1;k++){
    float t=k*0.01f, rate=0;
    if(t>=T_on && t<T_off) rate = PEAK*D2R*sinf(2*MF_PI*3.0f*(t-T_on));
#ifdef WARMUP
    if(t>=1.0f && t<3.0f) rate = 400.0f*D2R*sinf(2*MF_PI*2.0f*(t-1.0f));
#endif
    /* 真实姿态推进 */
    float dq[4]={cosf(0.5f*rate*0.01f), ax[0]*sinf(0.5f*rate*0.01f), ax[1]*sinf(0.5f*rate*0.01f), ax[2]*sinf(0.5f*rate*0.01f)}, tmp[4];
    qmul(qt,dq,tmp); memcpy(qt,tmp,sizeof(tmp)); qnorm(qt);
    memcpy(hist_q[hi&63],qt,sizeof(qt)); hi++;
#ifndef DRIFT_K
#define DRIFT_K 0.012f
#endif
    err += DRIFT_K*fabsf(rate)*0.01f;          /* 六轴 yaw 误差累积 */
    /* 磁力计：在 SKEW 拍之前的真实姿态下采样 */
    float *qm = hist_q[(hi-1-SKEW)&63], qmi[4]={qm[0],-qm[1],-qm[2],-qm[3]}, b[3], m[3], o[3];
    mf_q_rotate(qmi,w_e,b);
#ifndef NOISE
#define NOISE 0.005f
#endif
    for(int i=0;i<3;i++) m[i]=b[i]+NOISE*gauss();
#if USE_TS
    { float jit = TS_JIT_MS*(2.0f*(((rs=rs*1664525u+1013904223u)>>8)/16777216.0f)-1.0f);
      int64_t err_t = (int64_t)((TS_ERR_MS + jit)*0.001f/MF_TICK_S);
      mag_fusion_on_mag_ts(&f,m,o,(uint64_t)((int64_t)tick+640-(int64_t)(SKEW*640)+err_t)); }
#else
    mag_fusion_on_mag(&f,m,o);
#endif
    /* 六轴上报 = 真实姿态叠加世界系 yaw 误差 */
    float qe[4]={cosf(0.5f*err),0,0,sinf(0.5f*err)}, qr[4]; qmul(qe,qt,qr);
    tick += 640u;
    mag_fusion_on_quat(&f,tick,qr);
    float e = yaw_err(qr,qt)*R2D;
    if(t>=T_on && t<T_off){ n_motion++; if(fabsf(e)>peak_err) peak_err=fabsf(e); if(f.weight<=0.0f) w_zero_in_motion++; }
    if(fabsf(t-T_off)<0.005f) err_at_stop=e;
    if(t>=T_off && t_ok<0 && fabsf(e)<2.0f) t_ok=t-T_off;
    if(t>=T_off && t_ok>=0 && fabsf(e)>=2.0f) t_ok=-1;   /* 必须保持在 2 度内 */
  }
#if USE_TS
  printf("[学到延迟 %+5.2fms] ", mag_fusion_ts_offset_ms(&f));
#ifdef DBG_EXITS
  { extern unsigned dbg_ret[16]; printf("\n  更新=%u 退出:", dbg_ret[0]); for(int i=1;i<12;i++) printf(" [%d]=%u", i, dbg_ret[i]); printf("\n"); }
#endif
#endif
  printf("峰值%4.0fdeg/s | 六轴累积误差 %5.1f | 运动中输出误差峰值 %5.1f | 停止瞬间 %6.1f | 停后回到2度内 %s%.2fs | 运动中weight=0占比 %3.0f%%\n",
     PEAK, err*R2D, peak_err, err_at_stop, t_ok<0?">":"", t_ok<0?(N*0.01f-T_off):t_ok, 100.0f*w_zero_in_motion/n_motion);
  return 0;
}
