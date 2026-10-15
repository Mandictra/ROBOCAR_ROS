#include <cstdio>
#include <cmath>
#include <random>
#include <vector>
#include <algorithm>
#include "fusion.h"

static const float W_GEO = 0.20f, TICK = 2.0f * 3.14159265f * 0.0325f / 1000.0f;
static const float DT = 0.05f;
static double G0 = 0.004;

// ---------- replica of the existing 4-state EKF (main.cpp as uploaded) ----------
struct OldEKF {
  double X[4] = {0,0,0,0}, P[4][4], Q[4] = {0.001,0.001,0.005,0.05}, R[4] = {0.05,0.05,0.1,0.02};
  double gyro_bias_adj = 0; double still_t = 0;
  OldEKF(){ for(int i=0;i<4;i++) for(int j=0;j<4;j++) P[i][j]=(i==j); }
  static void inv4(double A[4][4], double Ai[4][4]) {
    double M[4][8]; for(int i=0;i<4;i++){ for(int j=0;j<4;j++){M[i][j]=A[i][j]; M[i][4+j]=(i==j);} }
    for(int c=0;c<4;c++){ int p=c; for(int r=c+1;r<4;r++) if(fabs(M[r][c])>fabs(M[p][c])) p=r;
      for(int k=0;k<8;k++) std::swap(M[c][k],M[p][k]); double d=M[c][c]; for(int k=0;k<8;k++) M[c][k]/=d;
      for(int r=0;r<4;r++) if(r!=c){ double f=M[r][c]; for(int k=0;k<8;k++) M[r][k]-=f*M[c][k]; } }
    for(int i=0;i<4;i++) for(int j=0;j<4;j++) Ai[i][j]=M[i][4+j]; }
  void step(double dt, double gz, double af, double dl, double dr, bool cmd_still) {
    gz -= gyro_bias_adj;                       // driver-corrected rate
    if (cmd_still) { still_t += dt; if (still_t > 1.0 && fabs(gz) < 0.05) { gyro_bias_adj += 0.02*gz; gz -= 0.02*gz; } } else still_t = 0;
    if (fabs(gz) < 0.005) gz = 0;
    double v_enc = (dl+dr)/2/dt, dth = (dr-dl)/W_GEO;
    double vp = X[3] + af*dt, tp = X[2] + gz*dt; if (cmd_still) vp = 0;
    X[0]+=vp*cos(tp)*dt; X[1]+=vp*sin(tp)*dt; X[2]=tp; X[3]=vp;
    double F[4][4]={{1,0,-vp*sin(tp)*dt,cos(tp)*dt},{0,1,vp*cos(tp)*dt,sin(tp)*dt},{0,0,1,0},{0,0,0,1}};
    double FP[4][4],Pn[4][4]; for(int i=0;i<4;i++)for(int j=0;j<4;j++){double s=0;for(int k=0;k<4;k++)s+=F[i][k]*P[k][j];FP[i][j]=s;}
    for(int i=0;i<4;i++)for(int j=0;j<4;j++){double s=0;for(int k=0;k<4;k++)s+=FP[i][k]*F[j][k];Pn[i][j]=s+(i==j?Q[i]:0);}
    double Z[4]={X[0],X[1],X[2]-gz*dt+dth,v_enc}, Y[4]; for(int i=0;i<4;i++) Y[i]=Z[i]-X[i];
    while(Y[2]>M_PI)Y[2]-=2*M_PI; while(Y[2]<-M_PI)Y[2]+=2*M_PI;
    double S[4][4],Si[4][4]; for(int i=0;i<4;i++)for(int j=0;j<4;j++) S[i][j]=Pn[i][j]+(i==j?R[i]:0);
    inv4(S,Si); double K[4][4]; for(int i=0;i<4;i++)for(int j=0;j<4;j++){double s=0;for(int k=0;k<4;k++)s+=Pn[i][k]*Si[k][j];K[i][j]=s;}
    for(int i=0;i<4;i++){double s=0;for(int j=0;j<4;j++)s+=K[i][j]*Y[j];X[i]+=s;}
    while(X[2]>M_PI)X[2]-=2*M_PI; while(X[2]<-M_PI)X[2]+=2*M_PI;
    for(int i=0;i<4;i++)for(int j=0;j<4;j++){double s=0;for(int k=0;k<4;k++)s+=((i==k)-K[i][k])*Pn[k][j];P[i][j]=s;}
  }
};

static double wrapd(double a){ while(a>M_PI)a-=2*M_PI; while(a<-M_PI)a+=2*M_PI; return a; }

struct Result { double pos_err, yaw_rms, yaw_end; };

// scenario: 0 = nominal, 1 = heavy slip (extra wheel spin events + strong skid scale), 2 = IMU dropouts
Result run(int scen, unsigned seed, int which, bool print=false) {
  std::mt19937 rng(seed); std::normal_distribution<double> N(0,1);
  double kappa = (scen==1) ? 0.55 : 0.72;                 // encoders under-read yaw rate on skid steer
  double gbias = G0 + 0.3*G0*N(rng);                     // rad/s residual gyro bias (~0.7 deg/s)
  double abias = 0.06*N(rng);
  double tx=0,ty=0,tth=0, tv=0;
  FusionEKF f; f.p.track_width_eff = W_GEO;                // deliberately NOT calibrated for the skid scale
  OldEKF o;
  double gx=0,gy=0,gth=0;                                  // gyro-only heading + enc speed
  double ex=0,ey=0,eth=0;                                  // encoder-only
  double enc_acc[4]={0,0,0,0}; long cnt[4]={0,0,0,0}, prev[4]={0,0,0,0};
  double sumsq=0; int n=0; double T=180;
  for (int k=0;k<(int)(T/DT);k++) {
    double t=k*DT;
    // command profile: straight, turns (both ways), stops, reverse
    double vc, wc;
    double ph = fmod(t,36.0);
    if (ph<8){vc=0.35;wc=0;} else if(ph<12){vc=0.25;wc=0.9;} else if(ph<14){vc=0;wc=0;}
    else if(ph<22){vc=0.35;wc=0;} else if(ph<26){vc=0.25;wc=-0.8;} else if(ph<30){vc=-0.30;wc=0;} else {vc=0;wc=0;}
    double a_true=(vc-tv)/0.15; double v_old=tv; tv+=a_true*DT*1.0; if (fabs(vc-tv)<1e-3) tv=vc; (void)v_old;
    double w_true = wc;
    tth+=w_true*DT; tx+=tv*cos(tth)*DT; ty+=tv*sin(tth)*DT;
    // wheel travel with skid-steer yaw under-read, slip, quantization
    double vl=tv - 0.5*W_GEO*w_true*kappa, vr=tv + 0.5*W_GEO*w_true*kappa;
    double dwl[2]={vl*DT,vl*DT}, dwr[2]={vr*DT,vr*DT};     // fl,rl / fr,rr
    bool spin = (scen==1) ? (fmod(t,17.0)>8 && fmod(t,17.0)<10.5) : (fmod(t,53.0)>20 && fmod(t,53.0)<21.0);
    if (spin) dwl[0]*=2.2;                                   // front-left wheel spinning
    double d[4]={dwl[0],dwl[1],dwr[0],dwr[1]};
    double dq[4];
    for(int i=0;i<4;i++){ enc_acc[i]+=d[i]/TICK; long c=(long)floor(enc_acc[i]); dq[i]=(c-cnt[i])*TICK; cnt[i]=c; }
    double dfl=dq[0],drl=dq[1],dfr=dq[2],drr=dq[3];
    // IMU
    double gz = w_true + gbias + 0.006*N(rng);
    double af = a_true + abias + 0.15*N(rng);
    bool imu_ok = true;
    if (scen==2 && fmod(t,40.0)>25 && fmod(t,40.0)<30) imu_ok=false;
    bool cmd_still = (vc==0 && wc==0);
    // old
    o.step(DT, gz, af, (dfl+drl)/2, (dfr+drr)/2, cmd_still);
    // fusion
    FusionInput in{(float)DT, imu_ok, (float)gz, (float)af, (float)dfl,(float)drl,(float)dfr,(float)drr};
    f.step(in);
    // baselines
    double dl=(dfl+drl)/2, dr=(dfr+drr)/2, dc=(dl+dr)/2;
    gth+=gz*DT; gx+=dc*cos(gth); gy+=dc*sin(gth);
    eth+=(dr-dl)/W_GEO; ex+=dc*cos(eth); ey+=dc*sin(eth);
    double est_th = (which==0)? o.X[2] : (which==1)? f.theta() : (which==2)? gth : eth;
    double e=wrapd(est_th-tth); sumsq+=e*e; n++;
    if (k==(int)(T/DT)-1){
      double estx=(which==0)?o.X[0]:(which==1)?f.px():(which==2)?gx:ex, esty=(which==0)?o.X[1]:(which==1)?f.py():(which==2)?gy:ey;
      return {hypot(estx-tx,esty-ty), sqrt(sumsq/n), e};
    }
  }
  return {0,0,0};
}

int main(int argc,char**argv){ if(argc>1) G0=atof(argv[1]); printf("residual gyro bias ~ %.3f rad/s (%.2f deg/s)\n",G0,G0*180/M_PI);
  const char* names[4]={"old EKF","new fusion","gyro+enc speed","encoder only"};
  const char* sc[3]={"nominal","heavy slip + skid","IMU dropouts"};
  for(int s=0;s<3;s++){
    printf("\n=== scenario: %s (180 s, 20 seeds, mean) ===\n",sc[s]);
    printf("%-16s %12s %12s %14s\n","method","pos err [m]","yaw rms [deg]","final yaw [deg]");
    for(int w=0;w<4;w++){
      double pe=0,yr=0,ye=0; int R=20;
      for(int r=0;r<R;r++){ auto x=run(s,100+r,w); pe+=x.pos_err; yr+=x.yaw_rms; ye+=fabs(x.yaw_end); }
      printf("%-16s %12.2f %12.2f %14.2f\n",names[w],pe/R,yr/R*180/M_PI,ye/R*180/M_PI);
    }
  }
}
