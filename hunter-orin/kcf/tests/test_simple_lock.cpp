#include "simple_lock.hpp"
#include <cstdio>
#include <cmath>
using namespace slock;
static int P=0,F=0;
static void ck(bool c,const char*w){ if(c)++P; else {++F; printf("  [FAIL] %s\n",w);} }
static Detection D(double x,double y,double w,double h,double s){ return Detection{hybrid::BoxF::fromXYWH(x,y,w,h),s,2}; }
int main(){
    Config c; c.lockWindow=30; c.lockHits=20;
    // --- 1) 40 karenin 30'unda -> kilit; her karede kacirma sayaci SIFIRLAMAZ
    { SimpleLock L(c); hybrid::BoxF ib; int n=0;
      for(int i=0;i<30;i++){ std::vector<Detection> d;
        if(i%3!=2) d.push_back(D(100,100,50,50,0.9));   // 20/30 isabet
        L.onDetections(d); if(L.pendingInit(ib)){n=i+1;break;} }
      ck(n>0,"20/30 isabette kilit kurulmadi");
      ck(L.state()==State::SEARCH,"initDone oncesi durum SEARCH olmali");
      L.initDone(true);
      ck(L.state()==State::LOCK,"initDone sonrasi LOCK olmali");
      ck(L.hasBox(),"kilit kutusu yok"); }
    // --- 2) 20/40 isabet -> kilit YOK
    { SimpleLock L(c); hybrid::BoxF ib; bool got=false;
      for(int i=0;i<200;i++){ std::vector<Detection> d;
        if(i%2==0) d.push_back(D(100,100,50,50,0.9));
        L.onDetections(d); if(L.pendingInit(ib)){got=true;break;} }
      ck(!got,"15/30 isabette kilit KURULMAMALI"); }
    // --- 3) Kilitliyken tespit ISTENMEZ (varlik kontrolu kapali)
    { SimpleLock L(c); hybrid::BoxF ib;
      for(int i=0;i<40;i++){ L.onDetections({D(100,100,50,50,0.9)}); if(L.pendingInit(ib)) break; }
      L.initDone(true);
      ck(!L.detectionWanted(),"kilitte tespit istenmemeli");
      for(int i=0;i<500;i++) L.onTrackerResult(true,hybrid::BoxF::fromXYWH(100,100,50,50));
      ck(!L.detectionWanted(),"varlik kapaliyken hic tespit istenmemeli");
      ck(L.state()==State::LOCK,"kilit 500 kare durmali"); }
    // --- 4) BUYUME: 50x50 -> 125x125 tetikler -> 75x75'e doner
    { SimpleLock L(c); hybrid::BoxF ib;
      for(int i=0;i<40;i++){ L.onDetections({D(100,100,50,50,0.9)}); if(L.pendingInit(ib)) break; }
      L.initDone(true);
      L.onTrackerResult(true,hybrid::BoxF::fromXYWH(100,100,124,124));
      ck(!L.pendingInit(ib),"x2.48'de tetiklememeli");
      L.onTrackerResult(true,hybrid::BoxF::fromXYWH(100,100,126,126));
      ck(L.pendingInit(ib),"x2.52'de tetiklemeli");
      ck(std::fabs(ib.w()-75.0)<0.6,"75x75'e cekilmeli, geldi %.1f");
      L.initDone(true);
      ck(L.clamps()==1,"clamp sayaci"); }
    // --- 5) [B] Referans GUNCELLENIR: ikinci tetik 187'de olmali
    { SimpleLock L(c); hybrid::BoxF ib;
      for(int i=0;i<40;i++){ L.onDetections({D(100,100,50,50,0.9)}); if(L.pendingInit(ib)) break; }
      L.initDone(true);
      L.onTrackerResult(true,hybrid::BoxF::fromXYWH(100,100,126,126)); L.pendingInit(ib); L.initDone(true);
      L.onTrackerResult(true,hybrid::BoxF::fromXYWH(100,100,180,180));
      ck(!L.pendingInit(ib),"yeni ref 75 -> 180 (x2.4) tetiklememeli");
      L.onTrackerResult(true,hybrid::BoxF::fromXYWH(100,100,190,190));
      ck(L.pendingInit(ib),"yeni ref 75 -> 190 (x2.53) tetiklemeli");
      ck(std::fabs(ib.w()-112.5)<0.8,"112.5'e cekilmeli"); }
    // --- 6) Kucuk kutuda buyume kurali UYGULANMAZ (growthMinPx=50)
    { SimpleLock L(c); hybrid::BoxF ib;
      for(int i=0;i<40;i++){ L.onDetections({D(100,100,16,16,0.9)}); if(L.pendingInit(ib)) break; }
      L.initDone(true);
      L.onTrackerResult(true,hybrid::BoxF::fromXYWH(100,100,45,45));  // x2.8 ama <50px
      ck(!L.pendingInit(ib),"50px altinda tetiklememeli");
      L.onTrackerResult(true,hybrid::BoxF::fromXYWH(100,100,55,55));  // x3.4 ve >=50px
      ck(L.pendingInit(ib),"50px ustunde tetiklemeli"); }
    // --- 7) VARLIK kontrolu: karede drone varsa kilit DURUR
    { Config cc=c; cc.presencePeriod=100; cc.presenceMisses=3; SimpleLock L(cc); hybrid::BoxF ib;
      for(int i=0;i<40;i++){ L.onDetections({D(100,100,50,50,0.9)}); if(L.pendingInit(ib)) break; }
      L.initDone(true);
      for(int r=0;r<10;r++){
        for(int i=0;i<100;i++) L.onTrackerResult(true,hybrid::BoxF::fromXYWH(100,100,50,50));
        ck(L.detectionWanted(),"varlik penceresi acilmali");
        // BASKA yerde drone var -> kutumuzla ortusmese bile kilit DURUR
        L.onDetections({D(900,700,60,60,0.9)}); }
      ck(L.state()==State::LOCK,"karede drone varken kilit DUSMEMELI"); }
    // --- 8) VARLIK kontrolu: 3 kez bos -> kilit duser
    { Config cc=c; cc.presencePeriod=100; cc.presenceMisses=3; SimpleLock L(cc); hybrid::BoxF ib;
      for(int i=0;i<40;i++){ L.onDetections({D(100,100,50,50,0.9)}); if(L.pendingInit(ib)) break; }
      L.initDone(true);
      for(int r=0;r<2;r++){
        for(int i=0;i<100;i++) L.onTrackerResult(true,hybrid::BoxF::fromXYWH(100,100,50,50));
        L.onDetections({}); }
      ck(L.state()==State::LOCK,"2 bos kontrolde dusmemeli");
      for(int i=0;i<100;i++) L.onTrackerResult(true,hybrid::BoxF::fromXYWH(100,100,50,50));
      L.onDetections({});
      ck(L.state()==State::SEARCH,"3 bos kontrolde dusmeli"); }
    // --- 9) KCF surekli gecersiz -> guvenlik agi
    { SimpleLock L(c); hybrid::BoxF ib;
      for(int i=0;i<30;i++){ L.onDetections({D(100,100,50,50,0.9)}); if(L.pendingInit(ib)) break; }
      L.initDone(true);
      for(int i=0;i<c.invalidLimit;i++) L.onTrackerResult(false,hybrid::BoxF{});
      ck(L.state()==State::SEARCH,"invalidLimit sonrasi aramaya donmeli"); }
    // --- 10) Guvenli alan zaman asimi
    { Config cc=c; cc.outsideLimit=60; SimpleLock L(cc); hybrid::BoxF ib;
      for(int i=0;i<30;i++){ L.onDetections({D(100,100,50,50,0.9)}); if(L.pendingInit(ib)) break; }
      L.initDone(true);
      for(int i=0;i<59;i++){ L.onTrackerResult(true,hybrid::BoxF::fromXYWH(100,100,50,50)); L.onUsable(false); }
      ck(L.state()==State::LOCK,"59 karede henuz dusmemeli");
      L.onTrackerResult(true,hybrid::BoxF::fromXYWH(100,100,50,50)); L.onUsable(false);
      ck(L.state()==State::SEARCH,"60 karede aramaya donmeli"); }
    // --- 11) Arada iceri girerse sayac sifirlanir
    { Config cc=c; cc.outsideLimit=60; SimpleLock L(cc); hybrid::BoxF ib;
      for(int i=0;i<30;i++){ L.onDetections({D(100,100,50,50,0.9)}); if(L.pendingInit(ib)) break; }
      L.initDone(true);
      for(int r=0;r<10;r++){
        for(int i=0;i<50;i++){ L.onTrackerResult(true,hybrid::BoxF::fromXYWH(100,100,50,50)); L.onUsable(false); }
        L.onTrackerResult(true,hybrid::BoxF::fromXYWH(100,100,50,50)); L.onUsable(true); }
      ck(L.state()==State::LOCK,"arada iceri girince dusmemeli"); }
    // --- 12) Kilit dustukten sonra YENIDEN yakalayabilmeli
    { SimpleLock L(c); hybrid::BoxF ib;
      for(int i=0;i<30;i++){ L.onDetections({D(100,100,50,50,0.9)}); if(L.pendingInit(ib)) break; }
      L.initDone(true);
      for(int i=0;i<c.invalidLimit;i++) L.onTrackerResult(false,hybrid::BoxF{});
      ck(L.state()==State::SEARCH,"once dusmeli");
      bool again=false;
      for(int i=0;i<30;i++){ L.onDetections({D(700,500,60,60,0.9)}); if(L.pendingInit(ib)){again=true;break;} }
      ck(again,"dustukten sonra yeniden kilitlenebilmeli");
      ck(ib.x1>600,"yeni kutu dogru yerde"); }
    printf("\n%s  %d/%d kontrol\n", F?"KALDI":"GECTI", P, P+F);
    return F?1:0;
}
