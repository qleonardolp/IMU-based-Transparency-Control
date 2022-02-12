#include<iostream>
#include<windows.h>
#include<Definitions.h>
#include<conio.h>

#include <mutex>

#include<ReabRob_EPOS.h>


using namespace std;



void exampleMove(CAN_Network* CAN1,int speed){

    CAN1->configure();

    CAN1->setState(ST_ENABLED);
    CAN1->setVelocity(speed);
                
    system("Pause");

    CAN1->setState(ST_DISABLED);

}

void exampleMoveAsync1(CAN_Network* CAN1,int speed){

    CAN1->configure();

    CAN1->setState(ST_ENABLED);
    CAN1->setVelocity(speed);
                

}

void exampleMoveAsync2(CAN_Network* CAN1){

       CAN1->setState(ST_DISABLED);

}

#define _epos1 CAN1->nodeId = 1;
#define _epos2 CAN1->nodeId = 2;
#define _epos3 CAN1->nodeId = 3;
#define _epos32 CAN1->nodeId = 32;
int main(){

    cout<<"EPOS Test in C++\n"; 
    
    CAN_Network * CAN1 = new CAN_Network("EPOS2","MAXON SERIAL V2","USB","USB0");
    if(!CAN1->connect()) return 0;
    

    WORD nodeID = 32;



    VCS_AnalogInputConfiguration(CAN1->epos,nodeID,1,AIC_GENERAL_PURPOSE_A,true,&CAN1->errorCode);
    VCS_AnalogInputConfiguration(CAN1->epos,nodeID,2,AIC_GENERAL_PURPOSE_B,true,&CAN1->errorCode);

    // while (true)
    // {
    //     long pos = 0;
    //     VCS_GetPositionIs(CAN1->epos,nodeID,&pos,&CAN1->errorCode);
    //     cout<<pos<<" ";
    //     long vol = 0;
    //     WORD vol1 = 0;
    //     VCS_GetAnalogInputVoltage(CAN1->epos,nodeID,1,&vol,&CAN1->errorCode);
        
    //     cout<<vol<<endl;


    //     if(kbhit()) {
    //         getch();
    //         break;
    //     }
    // }
    
    // _epos1 exampleMoveAsync2(CAN1);

    // exampleMove(CAN1,100); 
    _epos32 exampleMoveAsync1(CAN1,1000);

    Sleep(2000);

    _epos32 exampleMoveAsync1(CAN1,0);

    CAN1->setState(ST_DISABLED);



    // _epos2 exampleMoveAsync1(CAN1,200);

    // _epos3 exampleMoveAsync1(CAN1,50);

    // system("Pause");

    // _epos1 exampleMoveAsync2(CAN1);

    // _epos2 exampleMoveAsync2(CAN1);

    // _epos3 exampleMoveAsync2(CAN1);

    
    // _epos3
    // for(int i = 0 ; i < 100 ; i++){
    //     cout<<"I1 = "<<CAN1->analogRead(1)<<"\n";
    //     Sleep(50);
    // }
   
    
    CAN1->close();

    cout<<"Saliendo\n"; 

    return 0;
}