#include "AutoControl.h"
#include "MultiThread.h"


class Connector
{
public:
    enum GameType
    {
        speed,
        obstacle
    };
protected:
    static void* _threadFunc(void* p);

    void _speedCmp(int msgId);
    void _obstCmp(int msgId);
    void _sendMsg(int msgId);
public:

    Connector(GameType type,AutoCtrl* pCtrl);
    void ActionOver();
    void Start();
    void Exit();

protected:
    bool m_sysQuit;
    GameType m_gameType;
    THREAD m_threadDesc;
    AutoCtrl* m_pCtrl;
    int m_state;

};
