/* 静止噪声：板子不动，0.5% 磁噪声，看输出 yaw 的抖动 */
#ifndef MF_SRC
#define MF_SRC "../Src/glove/mag_fusion.c"
#endif
#ifndef USE_TS
#define USE_TS 1
#endif
#if !USE_TS
#undef USE_TS
#endif
#include MF_SRC
#include <stdio.h>
static uint32_t rs=777u;
static float gauss(void){ float u1,u2; rs=rs*1664525u+1013904223u; u1=((rs>>8)+1)/16777217.0f;
  rs=rs*1664525u+1013904223u; u2=((rs>>8)+1)/16777217.0f; return sqrtf(-2*logf(u1))*cosf(2*MF_PI*u2); }
int main(void){
  mag_fusion_t f; uint64_t tick=64000; float dip=-55.0f*MF_PI/180.0f, s=0, s2=0, mn=1e9, mx=-1e9; int n=0;
  mag_fusion_init(&f); mf_m3_identity(f.cal[0].W); f.cal[0].radius=1; f.cal[0].valid=1u; f.cal[0].active=0u;
  f.cal[0].revision=1u; f.cal[0].model=2u; f.fe.align_valid=1u;
  f.ref_dip=f.ref_dip0=dip; f.ref_dip_valid=1u; f.ref_norm=f.ref_norm0=1.0f; f.learn_state=MAG_LEARN_LOCKED;
  for(int k=0;k<2000;k++){ float m[3]={cosf(dip)+0.005f*gauss(), 0.005f*gauss(), sinf(dip)+0.005f*gauss()}, o[3], q[4]={1,0,0,0};
#ifdef USE_TS
    mag_fusion_on_mag_ts(&f,m,o,tick+640);
#else
    mag_fusion_on_mag(&f,m,o);
#endif
    tick+=640; mag_fusion_on_quat(&f,tick,q);
    if(k>=1000){ float y=2.0f*atan2f(q[3],q[0])*180.0f/MF_PI; s+=y; s2+=y*y; n++; if(y<mn)mn=y; if(y>mx)mx=y; } }
  printf("静止输出 yaw 标准差 %.3f 度，峰峰值 %.3f 度\n", sqrtf(s2/n-(s/n)*(s/n)), mx-mn); return 0; }
