#include "MultiThread.h"
#include <fstream>

#define OUT     Debug::Output
#define RECORD  Debug::Record

namespace Debug
{
    void Initialize(const char* recordFile,const char* outputFile,const char* rtRecordFile);
    void Exit();
    void Record(int leg,float shoulder,float arm,float feet);
    void Output(const char* fmt,...);

};
