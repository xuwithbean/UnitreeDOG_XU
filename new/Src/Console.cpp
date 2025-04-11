#include "Console.h"
#include <stdio.h>
#include <math.h>
#include <linux/input.h>
#include <unistd.h>
#include <fcntl.h>
#include "Debug.h"
#include "Record.h"
#include "PacePlanner.h"
////////////key masks///////////////
#define MASK_A          0x01
#define MASK_S          0x02
#define MASK_D          0x04
#define MASK_Q          0x08
#define MASK_W          0x10
#define MASK_E          0x20
#define MASK_QUIT       0x40
#define MASK_STAND      0x80
#define MASK_RECORD     0x100
#define MASK_SHOWRECORD 0x200

#define MASK_HOP            0x800
#define MASK_HOPFRONT       0x1000
#define MASK_STEPANDSPAN    0x2000
#define MASK_CLAW           0x4000
#define MASK_CLAWLEFT       0x8000
#define MASK_CLAWRIGHT      0x10000
#define MASK_RESTORE        0x20000
#define MASK_LERPRES        0x40000



void Console::_updateKeyEvents()
{
    input_event key_info;
    unsigned int mask = 0;
    //m_event.read((char*)&key_info,sizeof(key_info));
    if(read(m_eventFd,&key_info,sizeof(input_event)) > 0)
    {
        //printf("event\n");
        if(key_info.type != EV_KEY)return;

        switch(key_info.code)
        {
            case KEY_A:mask = MASK_A;break;
            case KEY_S:mask = MASK_S;break;
            case KEY_D:mask = MASK_D;break;
            case KEY_Q:mask = MASK_Q;break;
            case KEY_W:mask = MASK_W;break;
            case KEY_E:mask = MASK_E;break;
            case KEY_SPACE:mask = MASK_HOP|MASK_HOPFRONT;break;
            case KEY_P:mask = MASK_QUIT;break;
            case KEY_F:mask = MASK_STAND;break;
            case KEY_2:mask = MASK_HOP|MASK_CLAW;break;
            case KEY_1:mask = MASK_HOP|MASK_CLAWLEFT;break;
            case KEY_3:mask = MASK_HOP|MASK_CLAWRIGHT;break;
            case KEY_9:mask = MASK_HOP|MASK_STEPANDSPAN;break;
            case KEY_0:mask = MASK_HOP|MASK_RESTORE;break;
            case KEY_4:mask = MASK_HOP|MASK_LERPRES;break;
            case KEY_U:mask = MASK_RECORD;break;
            case KEY_I:mask = MASK_SHOWRECORD;break;

        }
        if(!mask)return;
        if(key_info.value == 0)
            m_keyMask &= ~mask;
        else if(key_info.value == 1 || key_info.value == 2)
            m_keyMask |= mask;
    }
}

//void Console::outstatus()
//{
    //printf("n_mannual:%d,mannual:%d\nn_auto:%d,auto:%d\nn_stop:%d,stop:%d\nn_quit:%d,quit:%d\nctrl[%.3f,%.3f,%.3f]\nout[%.3f%.3f%.3f]\nerr_his[%.3f,%.3f,%.3f]\n\
    inc[%.3f,%.3f,%.3f]"
    //,m_need_mannual,m_mannual,m_need_auto,m_auto,m_need_stop,m_stop,m_need_quit,m_quit,m_ctrl_x,m_ctrl_y,m_ctrl_r,m_out_x,m_out_y,m_out_r,
    //m_his_x,m_his_y,m_his_r,m_inc_x,m_inc_y,m_inc_r);
//}

void* Console::consoleFunc(void* arg)
{
    Console* pThis = (Console*)arg;
    printf("[Console]:start up!\n");
    pThis->_console();
}

int __parseNum(char* &ptr)
{
    int num = 0;
    for(;*ptr >= '0' && *ptr <= '9';++ptr)
    {
        num *= 10;
        num += *ptr-'0';
    }
    return num;
}

float __parseFloat(char* &ptr)
{
    int num = 0,dot = -1,sign = 1;
    float result = 0;
    for(;(*ptr >= '0' && *ptr <= '9') || *ptr == '-' || *ptr == '.';++ptr)
    {
        if(*ptr == '-')
        {
            if(num)return 0;
            sign = -1;
        }else if(*ptr == '.')
        {
            if(dot != -1)return 0;
            dot = 0;
        }else
        {
            if(dot != -1)dot++;
            num *= 10;
            num += *ptr -'0';
        }
    }
    result = num;
    for(int i = 0;i<dot;++i)result /= 10.0;
    return result*sign;
}

char *__doParse(char* ptr,AutoCtrl::Action &action)
{
    if(ptr == NULL)return NULL;
    if(*ptr == '\0')return NULL;
    char cmd = *ptr;
    if(cmd == 't')
    {
        ptr++;
        action.r = __parseFloat(ptr);
        action.action = AutoCtrl::autoRotateTo;
        return ptr+1;
    }else if(cmd == 'r')
    {
        ptr++;
        action.r = __parseFloat(ptr);
        action.action = AutoCtrl::autoRotateWith;
        return ptr+1;
    }else if(cmd == 'l')
    {
        ptr++;
        action.action = AutoCtrl::straight;
        action.actionCnt = __parseNum(ptr);
        ptr++;
        action.r = __parseFloat(ptr);
        return ptr+1;
    }else if(cmd == 'k')
    {
        ptr++;
        action.action = AutoCtrl::sback;
        action.actionCnt = __parseNum(ptr);
        ptr++;
        action.r = __parseFloat(ptr);
        return ptr+1;
    }
    else if(cmd == 'b')
    {
        ptr++;
        action.action = AutoCtrl::climbstraight;
        action.actionCnt = __parseNum(ptr);
        ptr++;
        action.r = __parseFloat(ptr);
        return ptr+1;
    }
    else if(cmd == 'h')
    {
        ptr++;
        action.action = AutoCtrl::hop;
        action.actionCnt = __parseNum(ptr);
        ptr++;
        action.r = __parseFloat(ptr);
        return ptr+1;
    }
    else if(cmd == 'i')
    {
        ptr++;
        action.action = AutoCtrl::hoponstair;
        action.actionCnt = __parseNum(ptr);
        ptr++;
        action.r = __parseFloat(ptr);
        return ptr+1;
    }
    else if(cmd == 'f')
    {
        ptr++;
        action.action = AutoCtrl::hopdownstair;
        action.actionCnt = __parseNum(ptr);
        ptr++;
        action.r = __parseFloat(ptr);
        return ptr+1;
    }
    else if(cmd == 'v')
    {
        ptr++;
        action.action = AutoCtrl::hopair;
        action.actionCnt = __parseNum(ptr);
        ptr++;
        action.r = __parseFloat(ptr);
        return ptr+1;
    }
    else if(cmd == 'u')
    {
        ptr++;
        action.action = AutoCtrl::getbalance;
        action.actionCnt = __parseNum(ptr);
        ptr++;
        action.r = __parseFloat(ptr);
        return ptr+1;
    }
    else if(cmd == 'o')
    {
        ptr++;
        action.action = AutoCtrl::hopforce;
        action.actionCnt = __parseNum(ptr);
        ptr++;
        action.r = __parseFloat(ptr);
        return ptr+1;
    }
    else if(cmd == 'g')
    {
        ptr++;
        action.action = AutoCtrl::clawForward;
        action.actionCnt = __parseNum(ptr);
        ptr++;
        action.r = __parseFloat(ptr);
        return ptr+1;
    }
    else if(cmd == 'n')
    {
        ptr++;
        action.action = AutoCtrl::clawLeft;
        action.actionCnt = __parseNum(ptr);
        ptr++;
        action.r = __parseFloat(ptr);
        return ptr+1;
    }
    else if(cmd == 'm')
    {
        ptr++;
        action.action = AutoCtrl::clawRight;
        action.actionCnt = __parseNum(ptr);
        ptr++;
        action.r = __parseFloat(ptr);
        return ptr+1;
    }
    else if(cmd == 'a')
    {
        ptr++;
        action.action = AutoCtrl::moveL;
        action.actionCnt = __parseNum(ptr);
        ptr++;
        action.r = __parseFloat(ptr);
        return ptr+1;
    }
    else if(cmd == 'd')
    {
        ptr++;
        action.action = AutoCtrl::moveR;
        action.actionCnt = __parseNum(ptr);
        ptr++;
        action.r = __parseFloat(ptr);
        return ptr+1;
    }
    else if(cmd == 'w')
    {
        ptr++;
        action.action = AutoCtrl::run;
        action.actionCnt = __parseNum(ptr);
        ptr++;
        action.r = __parseFloat(ptr);
        return ptr+1;
    }
    else if(cmd == 's')
    {
        ptr++;
        action.action = AutoCtrl::back;
        action.actionCnt = __parseNum(ptr);
        ptr++;
        action.r = __parseFloat(ptr);
        return ptr+1;
    }
    ptr = ptr+1;
    int num = __parseNum(ptr);
    if(*ptr != ';')return NULL;
    ptr++;
    action.actionCnt = num;
    switch(cmd)
    {
        case 'w':action.action = AutoCtrl::run;break;
        case 'a':action.action = AutoCtrl::moveL;break;
        case 's':action.action = AutoCtrl::back;break;
        case 'd':action.action = AutoCtrl::moveR;break;
        case 'q':action.action = AutoCtrl::rotateL;break;
        case 'e':action.action = AutoCtrl::rotateR;break;
        case 'z':action.action = AutoCtrl::turnL;break;
        case 'c':action.action = AutoCtrl::turnR;break;
        case 'f':action.action = AutoCtrl::hold;break;
        case 'l':action.action = AutoCtrl::straight;break;
        case 'k':action.action = AutoCtrl::sback;break;
        case 'b':action.action = AutoCtrl::climbstraight;break;
        case 'h':action.action = AutoCtrl::hop;break;
        case 'o':action.action = AutoCtrl::hopforce;break;
        case 'j':action.action = AutoCtrl::clawForward;break;
        case 'n':action.action = AutoCtrl::clawLeft;break;
        case 'm':action.action = AutoCtrl::clawRight;break;
        default:ptr = (char*)-1;break;
    }
    return ptr;
}

char buffer[1024];


void Console::_outRecord()
{
    AutoCtrl::Action lastAction;
    int cnt = 0;
    for(int i = 0;i<m_record.size();++i)
    {
        if(cnt == 0)
        {
            cnt = 1;
            lastAction = m_record[i];
        }else if(lastAction.x == m_record[i].x &&
                lastAction.y == m_record[i].y &&
                lastAction.r == m_record[i].r)
                cnt++;
        else
        {
            int code;
            code |= lastAction.x == 1?0x1:0;
            code |= lastAction.x == -1?0x2:0;
            code |= lastAction.y == 1?0x4:0;
            code |= lastAction.y == -1?0x8:0;
            code |= lastAction.r == 1?0x10:0;
            code |= lastAction.r == -1?0x20:0;
            char cmd = 0;
            switch(code)
            {
                case 0:cmd = 'f';break;
                case 0x1:cmd = 'd';break;
                case 0x2:cmd = 'a';break;
                case 0x4:cmd = 'w';break;
                case 0x8:cmd = 's';break;
                case 0x10:cmd = 'q';break;
                case 0x20:cmd = 'e';break;
                case 0x14:cmd = 'z';break;
                case 0x24:cmd = 'c';break;
            }
            if(cmd != 0)
                printf("%c%d;\n",cmd,cnt);
            cnt = 1;
            lastAction = m_record[i];
        }
    }
}

void Console::_console()
{
    char cmd;
    while(!m_threadQuit)
    {
        usleep(1000);
        if(m_status.mannaul)
        {
            //printf("mannual\n");
            if(!m_expStatus.mannaul)
            {
                if(!m_prop)
                printf("waiting to exit mannual mode...\n"),m_prop = true;
                continue;
            }
            _updateKeyEvents();
            if(m_keyMask & MASK_A && m_keyMask & MASK_D)
                m_request.x = 0;
            else if(m_keyMask & MASK_D) m_request.x = 1;
            else if(m_keyMask & MASK_A) m_request.x = -1;
            else m_request.x = 0;

            if(m_keyMask & MASK_W && m_keyMask & MASK_S)
                m_request.y = 0;
            else if(m_keyMask & MASK_W) m_request.y = 1;
            else if(m_keyMask & MASK_S) m_request.y = -1;
            else m_request.y = 0;

            if(m_keyMask & MASK_E && m_keyMask & MASK_Q)
                m_request.r = 0;
            else if(m_keyMask & MASK_E) m_request.r = -1;
            else if(m_keyMask & MASK_Q) m_request.r = 1;
            else m_request.r = 0;

            if(m_keyMask & MASK_HOP)
            {
                if(!m_hopBanned)
                {
                    m_request.reqHop = true;
                    if(m_keyMask & MASK_HOPFRONT)
                        m_request.hopType = Controller::HopForward;
                    else if(m_keyMask & MASK_STEPANDSPAN)
                        m_request.hopType = Controller::StepAndSpan;
                    else if(m_keyMask & MASK_CLAW)
                        m_request.hopType = Controller::Claw;
                    else if(m_keyMask & MASK_CLAWLEFT)
                        m_request.hopType = Controller::ClawLeft;
                    else if(m_keyMask & MASK_CLAWRIGHT)
                        m_request.hopType = Controller::ClawRight;
                    else if(m_keyMask & MASK_RESTORE)
                        m_request.hopType = Controller::StepToRestore;
                    else if(m_keyMask & MASK_LERPRES)
                        m_request.hopType = Controller::LerpToRestore;
                    else m_request.reqHop = false;
                }
            }

            if(m_keyMask & MASK_STAND)
                m_request.reqStop = true;

            if(m_keyMask & MASK_QUIT)
            {
                m_request.reqStop = true;
                m_expStatus.mannaul = false;
                m_expStatus.auto_ = false;
                m_expStatus.quit = false;
                m_keyMask = 0;
            }
            if(m_keyMask & MASK_RECORD)
            {
                if(!m_recording)
                {
                    printf("\n[Console] start recording...\n");
                    m_recording = true;
                    m_record.clear();
                }
            }
            if(m_keyMask & MASK_SHOWRECORD)
            {
                if(m_recording)
                {
                    m_recording = false;
                    _outRecord();
                    /*OUT("\n[Console]: showing the record result:\n");
                    for(int i = 0;i<m_record.size();++i)
                        OUT("pCtrl->AddRecord(%.3f,%.3f,%.3f);\n",m_record[i].x,m_record[i].y,m_record[i].r);
                    OUT("\n[Console]: end of the record\n");
                    */
                    m_request.reqStop = true;
                }
            }
            //printf("\nmask:%d\n",m_keyMask);

        }else if(m_status.auto_)
        {
            if(!m_expStatus.auto_)
            {
                if(!m_prop)
                printf("waiting to exit auto mode...\n"),m_prop = true;
                continue;
            }
            scanf("%c",&cmd);
            switch(cmd)
            {
                case 'r':
                case 'R':
                {
                    printf("[Console] using records ...\n");
                    Record(m_pCtrl);

                }break;
                case 'x':
                case 'X':
                {
                    printf("[Console] Abort!\n");
                    m_pCtrl->ClearActions();
                    m_request.reqStop = true;
                }break;
                case 'Q':
                case 'q':
                {
                    m_pCtrl->ClearActions();
                    m_request.reqStop = true;
                    m_expStatus.auto_ = false;

                }break;

            }
        }else if(m_status.test)
        {
            if(!m_expStatus.test)
            {
                if(!m_prop)
                printf("waiting to exit test mode...\n"),m_prop = true;
                continue;
            }
            scanf("%s",buffer);
            printf("test recv : %s\n",buffer);
            if(buffer[0] == 'x' || buffer[0] == 'X')
            {
                m_pCtrl->ClearActions();
                m_request.reqStop = true;
                m_expStatus.test = false;
                continue;
            }
            if(buffer[0] == 'p' || buffer[0] == 'P')
            {
                m_request.reqStop = true;
                continue;
            }
            if(buffer[0] == '/')
            {
                m_request.reqStop = false;
                continue;
            }
            char* ptr = buffer;
            std::vector<AutoCtrl::Action> actions;
            AutoCtrl::Action action;
            ptr = __doParse(ptr,action);
            while(ptr)
            {
                actions.push_back(action);
                ptr = __doParse(ptr,action);
                if(ptr == (char*)-1)
                {
                    actions.clear();
                    printf("unknow command!\n");
                    continue;
                }
            }
            printf("command check: (total command %d)\n",actions.size());
            for(int i = 0;i<actions.size();++i)
            {
                if(actions[i].action == AutoCtrl::autoRotateTo || actions[i].action == AutoCtrl::autoRotateWith
                ||actions[i].action == AutoCtrl::straight || actions[i].action == AutoCtrl::climbstraight
                ||actions[i].action == AutoCtrl::hop || actions[i].action == AutoCtrl::hoponstair
                ||actions[i].action == AutoCtrl::hopdownstair || actions[i].action == AutoCtrl::hopforce
                ||actions[i].action == AutoCtrl::clawForward || actions[i].action == AutoCtrl::clawLeft
                ||actions[i].action == AutoCtrl::clawRight || actions[i].action == AutoCtrl::moveL
                ||actions[i].action == AutoCtrl::moveR  || actions[i].action == AutoCtrl::run
                ||actions[i].action == AutoCtrl::back || actions[i].action == AutoCtrl::hopair
                ||actions[i].action == AutoCtrl::getbalance || actions[i].action == AutoCtrl::sback)
                {
                    if(actions[i].action == AutoCtrl::autoRotateTo)printf("rotate to %.3f deg\n",actions[i].r);
                    else if(actions[i].action == AutoCtrl::autoRotateWith)printf("rotate with %.3f deg\n",actions[i].r);
                    else if(actions[i].action == AutoCtrl::straight)printf("move %d cnt,with %.3f deg\n",actions[i].actionCnt,actions[i].r);
                    else if(actions[i].action == AutoCtrl::sback)printf("sback %d cnt,%.3f deg\n",actions[i].actionCnt,actions[i].r);
                    else if(actions[i].action == AutoCtrl::climbstraight)printf("climb %d cnt,%.3f deg\n",actions[i].actionCnt,actions[i].r);
                    else if(actions[i].action == AutoCtrl::hop)printf("hop %d cnt,%.3f deg\n",actions[i].actionCnt,actions[i].r);
                    else if(actions[i].action == AutoCtrl::hoponstair)printf("hoponstair %d cnt,%.3f deg\n",actions[i].actionCnt,actions[i].r);
                    else if(actions[i].action == AutoCtrl::hopdownstair)printf("hopdownstair %d cnt,%.3f deg\n",actions[i].actionCnt,actions[i].r);
                    else if(actions[i].action == AutoCtrl::hopair)printf("hopair %d cnt,%.3f deg\n",actions[i].actionCnt,actions[i].r);
                    else if(actions[i].action == AutoCtrl::getbalance)printf("getbalance %d cnt,%.3f deg\n",actions[i].actionCnt,actions[i].r);
                    else if(actions[i].action == AutoCtrl::hopforce)printf("hopforce %d cnt,%.3f deg\n",actions[i].actionCnt,actions[i].r);
                    else if(actions[i].action == AutoCtrl::clawForward)printf("clawforward %d cnt,%.3f deg\n",actions[i].actionCnt,actions[i].r);
                    else if(actions[i].action == AutoCtrl::clawRight)printf("clawright %d cnt,%.3f deg\n",actions[i].actionCnt,actions[i].r);
                    else if(actions[i].action == AutoCtrl::clawLeft)printf("clawleft %d cnt,%.3f deg\n",actions[i].actionCnt,actions[i].r);
                    else if(actions[i].action == AutoCtrl::moveL)printf("moveL %d cnt,%.3f deg\n",actions[i].actionCnt,actions[i].r);
                    else if(actions[i].action == AutoCtrl::moveR)printf("moveL %d cnt,%.3f deg\n",actions[i].actionCnt,actions[i].r);
                    else if(actions[i].action == AutoCtrl::run)printf("run %d cnt,%.3f deg\n",actions[i].actionCnt,actions[i].r);
                    else if(actions[i].action == AutoCtrl::back)printf("back %d cnt,%.3f deg\n",actions[i].actionCnt,actions[i].r);
                }else
                {
                switch(actions[i].action)
                {
                    case AutoCtrl::rotateL:printf("rotateL ");break;
                    case AutoCtrl::rotateR:printf("rotateR ");break;
                    case AutoCtrl::turnL:printf("turnL ");break;
                    case AutoCtrl::turnR:printf("turnR ");break;
                    case AutoCtrl::hold:printf("hold ");break;
                }
                printf("%d times;\n",actions[i].actionCnt);
                }
            }
            bool ok = false;
            while(true)
            {
                printf("continue? y/n:\n");
                char cmd;
                scanf("%c",&cmd);
                if(cmd == 'n')
                {
                    printf("aborted!\n");
                    break;
                }else if(cmd == 'y')
                {
                    printf("executing...\n");
                    ok = true;
                    break;
                }
            }
            if(ok)
                m_pCtrl->AddActions(actions);
        }else
        {
            if(m_expStatus.auto_ || m_expStatus.mannaul || m_expStatus.test)
            {
                if(!m_prop)
                    printf("waiting to change mode...\n"),m_prop = true;
                continue;
            }
            if(m_expStatus.quit)
            {
                //printf("waiting to quit...\n");
                continue;
            }
            scanf("%c",&cmd);
            printf("recv:%c\n",cmd);
            switch(cmd)
            {
                case 'H':
                case 'h':
                {
                    printf("**********************************\n");
                    printf("*           help page            *\n");
                    printf("**********************************\n");
                    printf("[h/H] help page\n");
                    printf("[g/G] gyro value\n");
                    printf("[m/M] mannually control\n");
                    printf("[v/V] automatically control\n");
                    printf("[t/T] automatic test\n");
                    printf("[x/X] quit system\n");
                    printf("when system is mannually controlling:\n");
                    printf("\t[w/a/s/d] moving control\n");
                    printf("\t[q/e] rotating control\n");
                    printf("\t[Space] hop forward\n");

                    printf("\t[1] claw left\n");
                    printf("\t[2] claw forward\n");
                    printf("\t[3] claw right\n");
                    printf("\t[4] lerp to restore posture\n");

                    printf("\t[9] span the legs\n");
                    printf("\t[0] step to restore posture\n");


                    printf("\t[p] quit control\n");
                    printf("when system is automatically controlling:\n");
                    printf("\t[q] quit\n");
                    printf("when system is in automatic test:\n");
                    printf("\t[x/X] quit\n");
                    printf("\t[p/P] stop\n");
                    printf("\t[{cmd<number>;}+] make test command\n");
                    printf("\tbasic command:\n");
                    printf("\t[w<num>] move forward <num> steps\n");
                    printf("\t[a<num>] move left <num> steps\n");
                    printf("\t[s<num>] move backward <num> steps\n");
                    printf("\t[d<num>] move right <num> steps\n");
                    printf("\t[q<num>] rotate anticlockwise <num> steps\n");
                    printf("\t[e<num>] rotate clockwise <num> steps\n");
                    printf("\t[z<num>] turn left <num> steps\n");
                    printf("\t[c<num>] turn right <num> steps\n");
                    printf("\t[t<num>] rotate to <num> degree\n");
                    printf("\t[r<num>] rotate with <num> degree\n");
                }break;
                case 'g':
                case 'G':
                {
                    printf("[Gyro Value]=%.3f\n",m_pCtrl->GetIMUSensor()->GetIMUData().yaw);
                }break;
                case 'm':
                case 'M':
                {
                    m_request.reqStop = true;
                    m_expStatus.mannaul = true;
                }break;
                case 'v':
                case 'V':
                {
                    m_expStatus.auto_ = true;
                    m_request.reqStop = true;
                }break;
                case 'x':
                case 'X':
                {
                    m_request.reqStop = true;
                    m_expStatus.quit = true;
                }break;
                case 't':
                case 'T':
                {
                    m_request.reqStop = true;
                    m_expStatus.test = true;
                }
            }
        }
    }
    printf("[Console]:system quit!\n");
}

Console::Console(const char* strKeyEvent,AutoCtrl* pCtrl,Controller* pCtrller)
{

    m_pCtrl = pCtrl;
    m_pCtrller = pCtrller;
    //m_event.open(strKeyEvent);


    m_threadQuit = true;
    m_threadDesc = 0;
    m_request.r = m_request.x = m_request.y = 0;
    m_request.reqHop = m_request.reqStop = false;
    m_status.auto_ = m_status.mannaul =m_status.quit = m_status.test= false;
    m_expStatus.auto_ = m_expStatus.mannaul =m_expStatus.quit= m_expStatus.test= false;
    m_keyMask = 0;
    m_prop = false;
    m_hopBanned = false;
    m_eventFd = open(strKeyEvent,O_RDONLY,0777);
    if(m_eventFd < 0)
        printf("[Console]:cannot open event device\n");
}

void Console::Start()
{
    if(!m_threadQuit)return;
    m_threadQuit = false;
    thread_create(Console::consoleFunc,this,m_threadDesc);
}

void Console::Exit()
{
    if(m_threadQuit)return;
    m_threadQuit = true;
    thread_join(m_threadDesc);
    thread_destroy(m_threadDesc);
}

Console::~Console()
{
}

void Console::UpdateEvent(bool ctrlStop,bool stepOver)
{
    if(ctrlStop)
    {
        //printf("stop\n");
        if(m_status.auto_ != m_expStatus.auto_ ||
         m_status.mannaul != m_expStatus.mannaul ||
          m_status.quit != m_expStatus.quit ||
          m_status.test != m_expStatus.test)
        {
            m_status = m_expStatus;
            printf("[Console]:mode changed!\n");
            printf("mannaul:%d,auto:%d,test:%d,quit:%d\n",m_status.mannaul,m_status.auto_,m_status.test,m_status.quit);
            m_prop = false;
            m_keyMask = 0;
            fflush(stdin);
        }
    }
    if(stepOver)
    {
        if(m_recording && !ctrlStop)
        {
            AutoCtrl::Action action;
            action.x = m_request.x;
            action.y = m_request.y;
            action.r = m_request.r;
            //m_pCtrller->GetCurrentVelocity(action.x,action.y,action.r);
            m_record.push_back(action);
        }

    }
}

void Console::GetConsoleRequest(Console::ConsoleRequest &request)
{
    m_hopBanned = false;
    if(m_request.reqHop)m_hopBanned = true;
    request = m_request;
    m_request.reqHop = m_request.reqStop = false;
}
