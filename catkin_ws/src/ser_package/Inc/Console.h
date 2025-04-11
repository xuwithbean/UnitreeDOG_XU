#include "MultiThread.h"
#include <fstream>
#include "AutoControl.h"
#include "Controller.h"

class Console
{
    public:
    struct ConsoleStatus
    {
        bool auto_;
        bool mannaul;
        bool quit;
        bool test;
    };
    struct ConsoleRequest
    {
        bool reqStop;
        bool reqHop;
        Controller::HopType hopType;
        float x,y,r;
        float leanAngle;
    };
    protected:
    void _updateKeyEvents();

    static void* consoleFunc(void* arg);

    void _console();
    void _outstatus();
    void _outRecord();
    public:
    Console(const char* strKeyEvent,AutoCtrl* pCtrl,Controller* pCtrller,Lazer* pLazer);
    ~Console();
    void Start();
    void Exit();

    void UpdateEvent(bool ctrlStop,bool stepOver);

    void GetConsoleRequest(ConsoleRequest& request);
    void GetConsoleStatus(ConsoleStatus& status){status = m_status;}

    protected:
    unsigned int m_keyMask;
    int m_eventFd;
    THREAD m_threadDesc;
    ConsoleStatus m_status;
    ConsoleStatus m_expStatus;
    ConsoleRequest m_request;
    bool m_prop;
    bool m_threadQuit;
    bool m_recording;
    std::ifstream m_event;
    AutoCtrl* m_pCtrl;
    Controller* m_pCtrller;
    std::vector<AutoCtrl::Action> m_record;
    Lazer* m_Lazer;
    bool m_hopBanned;
    float m_leanAngle;
};

