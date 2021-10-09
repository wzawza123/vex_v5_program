/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\DELL                                             */
/*    Created:      Fri Nov 06 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
//motor define
#define LF1 4
#define LF0 9
#define LB1 3
#define LB0 1
#define RF1 19
#define RF0 17
#define RB1 18
#define RB0 20

//roll ball define
#define HL 5
#define HR 15
#define BU 6
#define BD 16
#define FDL 7
#define FUL 8
#define FDR 14
#define FUR 2

#define MAXV 12000;

void VRUN(double l,double r){
    //more left
    VVST(LF0,LF1,l)
        //less left
    VVST(LB0,LB1,l)//FAV(3)+FAV(1)
        //more right
    VVST(RF0,RF1,r)
        //less right
    VVST(RB0,RB1,r)//FAV(3)-FAV(1)
}

void GetBall(int VTG)
{
    VSTT(BD,VTG);
    VSTT(BU,VTG);
    VSTT(FDL,VTG);    
    VSTT(FUL,VTG);    
    VSTT(FDR,VTG);    
    VSTT(FUR,VTG);    

}

void OutBall_F(int VTG)
{
    VSTT(HL,-VTG);
    VSTT(HR,-VTG);
    VSTT(BD,-VTG);
    VSTT(BU,-VTG);
    VSTT(FDL,-VTG);    
    VSTT(FUL,-VTG);    
    VSTT(FDR,-VTG);    
    VSTT(FUR,-VTG);    
}

void OutBall_B(int VTG)
{
    VSTT(HL,VTG);
    VSTT(HR,VTG);
    VSTT(BD,VTG);
    VSTT(BU,-VTG);
    VSTT(FDL,VTG);    
    VSTT(FUL,VTG);    
    VSTT(FDR,VTG);    
    VSTT(FUR,VTG);    

}

void StrBall(int VTG)
{
    VSTT(HL,VTG);
    VSTT(HR,VTG);
    VSTT(FUL,VTG); 
    VSTT(FDL,VTG);    
    VSTT(FDR,VTG);    
    VSTT(FUR,VTG);    
}
void set_zero()
{
    VSTT(HL,0);
    VSTT(HR,0);
    VSTT(BD,0);
    VSTT(BU,0);
    VSTT(FDR,0);    
    VSTT(FUR,0);    
    VSTT(FDL,0);    
    VSTT(FUL,0);    
}



int main() 
{
    while(true)
    {
        if(std::abs(FAV(3))!=0 || std::abs(FAV(1))!=0)
            VRUN(FAV(3)+0.5*FAV(1),FAV(3)-0.5*FAV(1));
        else
            VRUN(0,0);
            
        if(BP(L2))
            GetBall(12000);
        else if(BP(R1))
            OutBall_F(12000);
        else if(BP(L1))
            OutBall_B(12000);
        else if(BP(R2))
            StrBall(12000);
        else
            set_zero();
        SLEEP(7);
    }
    return 0;
}
   
    
    
    
    
