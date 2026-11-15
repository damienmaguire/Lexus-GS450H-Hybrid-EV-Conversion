/* test_deadband.c -- sweep -300..+300 A in 0.01 A steps for each engine
 * state through bth5_applyCurrentPolicy(): never a charge-side value before
 * RUN, 0x1FF inside the 0.5 A deadband, correct state outside it. */
#include <stdio.h>
#include "bth_frames.h"
int main(void){ int bad=0; for (int st=0; st<4; st++) for (int ca=-30000; ca<=30000; ca++){ float a=ca*0.01f; bth5_eng_t e={st,0,0x1FF}; int sl; uint16_t r=bth5_encodeCurrent(a), r13=bth5_encodeCurrentFloor(a); int s=bth5_applyCurrentPolicy(&r,&r13,a,&e,&sl);
 if (st!=3 && (r>0x1FF||r13>0x1FF)) bad++;            /* never charge-side before RUN */
 if (a>-0.5f && a<0.5f && r!=0x1FF) bad++;              /* deadband */
 if (st==3 && a<=-0.5f && s!=BTH5_IST_CHG) bad++;
 if (a>=0.5f && s!=BTH5_IST_DIS) bad++; }
 printf("pre-RUN charge-side / deadband violations over -300..+300 A x 4 eng states: %d\n",bad); return bad!=0; }
