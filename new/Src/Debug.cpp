#include "Debug.h"
#include <stdarg.h>
#include <fcntl.h>
#include <unistd.h>

MUTEX mutexOutput;
MUTEX mutexRecord;
std::ofstream rcFile,outFile;
int rcQueue;
bool enableRecord = false,enableOutput = false,enableRealtime = false;


void Debug::Initialize(const char* recordFile,const char* outputFile,const char* rtRecordFile)
{
    mutex_create(mutexOutput);
    mutex_create(mutexRecord);
    if(recordFile)
    {
        enableRecord = true;
        rcFile.open(recordFile);
        if(!rcFile.is_open())
        {
            printf("[Debug]:record file open failed!");
            exit(-1);
        }
    }else enableRecord = false;

    if(outputFile)
    {
        enableOutput = true;
        outFile.open(outputFile);
        if(!outFile.is_open())
        {
            printf("[Debug]:output file open failed!");
            exit(-1);
        }
    }else enableOutput = false;

    if(rtRecordFile)
    {
    enableRealtime = true;
        rcQueue = open(rtRecordFile,O_WRONLY);
        if(rcQueue < 0)
        {
            printf("[Debug]:pipe file open failed!");
            exit(-1);
        }
    }else enableRealtime = false;

}

void Debug::Exit()
{
    mutex_destroy(mutexOutput);
    mutex_destroy(mutexRecord);

    if(rcFile.is_open())rcFile.close();
    if(outFile.is_open())outFile.close();
    if(rcQueue > 0)close(rcQueue);
}

char outputBuffer[1024];
char recordBuffer[1024];
void Debug::Output(const char* fmt,...)
{
    mutex_lock(mutexOutput);
    va_list args;
    va_start(args, fmt);
    int n = vsprintf(outputBuffer,fmt,args);
    if(enableOutput)
    {
        outFile.write(outputBuffer,n);
        outFile.flush();
    }
    printf("%s",outputBuffer);
    va_end(args);
    mutex_unlock(mutexOutput);
}

void Debug::Record(int leg,float shoulder,float arm,float feet)
{
    if(enableRealtime || enableRecord)
    {
        mutex_lock(mutexRecord);
        int n = sprintf(recordBuffer,"%d,%f,%f,%f\n",leg,shoulder,arm,feet);
        if(enableRecord)
            rcFile.write(recordBuffer,n);
        if(enableRealtime)
            write(rcQueue,recordBuffer,n);
        mutex_unlock(mutexRecord);
    }else{
        printf("%d,%f,%f,%f\n",leg,shoulder,arm,feet);
        usleep(10000);
    }
}
