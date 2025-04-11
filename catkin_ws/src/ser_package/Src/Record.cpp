#include "AutoControl.h"
#include "Record.h"
void Record(AutoCtrl* pCtrl,Lazer* lazerparam)
{
    
    AutoCtrl::Action act;
    act.action = AutoCtrl::straight1;
    float init = pCtrl->GetIMUSensor()->GetIMUData().yaw;
    //double init1 =0.0;
    //inityaw_dog = 0.0001;
    //act.inityaw=inityaw_dog;
    act.r =init;
    act.wanty=460.0;//445//450
    act.wantx=-40.0;
    act.lasty=0.0;
    act.lastx=0.0;
    //printf("\nx:%.3f,y:%.3f\n",lazerparam->lazer_param.lazerx,lazerparam->lazer_param.lazery);
    act.actionCnt = 550;
    pCtrl->AddAction(act);

    act.action = AutoCtrl::hold;
    act.actionCnt = 1;
    pCtrl->AddAction(act);

    init -= 135;
    //init1 -= 135/180*3.1415926535;
    //inityaw_dog -= 135/180*3.1415926535;
    act.action = AutoCtrl::autoRotateTo;
    act.r = init;
    //act.inityaw=inityaw_dog;
    pCtrl->AddAction(act);

    act.action = AutoCtrl::straight2;
    act.r = init;
    //act.inityaw=inityaw_dog;
    act.wanty=670.0;//320//600
    act.wantx=-20.0;
    act.lasty=0.0;
    act.lastx=0.0;
    act.actionCnt = 525;
    pCtrl->AddAction(act);

    act.action = AutoCtrl::hold;
    act.actionCnt = 1;
    pCtrl->AddAction(act);

    init += 225;
    //init1+=225/180*3.1415926535;
    //inityaw_dog += 225.180*3.1415926535;
    act.action = AutoCtrl::autoRotateTo;
    act.r = init;
    //act.inityaw=inityaw_dog;
    pCtrl->AddAction(act);

    act.action = AutoCtrl::straight3;
    act.r = init;
    //act.inityaw=inityaw_dog;
    act.wantx=0.0;
    act.wanty=930;//500//1000
    act.lasty=-20.0;
    act.lastx=0.0;
    act.actionCnt = 530;
    pCtrl->AddAction(act);

    act.action = AutoCtrl::hold;
    act.actionCnt = 1;
    pCtrl->AddAction(act);

    init += 135;
    //init1 +=135/180*3.1415926535;
    //inityaw_dog +=135/180*3.1415926535;
    act.action = AutoCtrl::autoRotateTo;
    act.r = init;
    //act.inityaw=inityaw_dog;
    pCtrl->AddAction(act);


    act.action = AutoCtrl::straight4;
    act.r = init;
    //act.inityaw=inityaw_dog;
    act.wantx=20.0;
    act.wanty=680;//了211//710
    act.lasty=0.0;
    act.lastx=0.0;
    act.actionCnt = 525;
    pCtrl->AddAction(act);
    
    /*
    AutoCtrl::Action act;
    act.action = AutoCtrl::straight;
    float init = pCtrl->GetIMUSensor()->GetIMUData().yaw;
    act.r = init;
    act.actionCnt = 12;
    pCtrl->AddAction(act);

    act.action = AutoCtrl::hold;
    act.actionCnt = 1;
    pCtrl->AddAction(act);

    init -= 135;
    act.action = AutoCtrl::autoRotateTo;
    act.r = init;
    pCtrl->AddAction(act);

    act.action = AutoCtrl::straight;
    act.r = init;
    act.actionCnt = 17;
    pCtrl->AddAction(act);

    act.action = AutoCtrl::hold;
    act.actionCnt = 1;
    pCtrl->AddAction(act);

    init += 225;
    act.action = AutoCtrl::autoRotateTo;
    act.r = init;
    pCtrl->AddAction(act);

    act.action = AutoCtrl::straight;
    act.r = init;
    act.actionCnt = 25;
    pCtrl->AddAction(act);

    act.action = AutoCtrl::hold;
    act.actionCnt = 1;
    pCtrl->AddAction(act);

    init += 135;
    act.action = AutoCtrl::autoRotateTo;
    act.r = init;
    pCtrl->AddAction(act);

    act.action = AutoCtrl::straight;
    act.r = init;
    act.actionCnt = 18;
    pCtrl->AddAction(act);
    */
    /*
    act.action = AutoCtrl::hold;
    act.actionCnt = 3;
    pCtrl->AddAction(act);

    init -= 45;
    act.action = AutoCtrl::autoRotateTo;
    act.r = init;
    pCtrl->AddAction(act);

    act.action = AutoCtrl::straight;
    act.r = init;
    act.actionCnt = 20;
    pCtrl->AddAction(act);

    act.action = AutoCtrl::hold;
    act.actionCnt = 3;
    pCtrl->AddAction(act);

    init += 90;
    act.action = AutoCtrl::autoRotateTo;
    act.r = init;
    pCtrl->AddAction(act);

    //return ;
    act.action = AutoCtrl::straight;
    act.r = init;
    act.actionCnt = 20;
    pCtrl->AddAction(act);

    act.action = AutoCtrl::hold;
    act.actionCnt = 3;
    pCtrl->AddAction(act);

    init += 90;
    act.action = AutoCtrl::autoRotateTo;
    act.r = init;
    pCtrl->AddAction(act);

    act.action = AutoCtrl::straight;
    act.r = init;
    act.actionCnt = 20;
    pCtrl->AddAction(act);
    */
}
