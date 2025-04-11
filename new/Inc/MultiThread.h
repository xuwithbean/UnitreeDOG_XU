#pragma once
//#define SIMULATE

#ifdef SIMULATE
#include <Windows.h>
#define THREAD	HANDLE
#define MUTEX	HANDLE
#define thread_create(proc,args)	CreateThread(NULL, NULL, proc, args, NULL, NULL)
#define mutex_create()				CreateMutex(NULL, FALSE, NULL)
#define mutex_lock(mutex)			WaitForSingleObject(mutex,INFINITE)
#define mutex_unlock(mutex)			ReleaseMutex(mutex)
#define mutex_destroy(mutex)		CloseHandle(mutex)
#define thread_join(thread)			WaitForSingleObject(thread,INFINITE)
#define thread_destroy(thread)		CloseHandle(thread)
#else
#include <pthread.h>
#define THREAD	pthread_t
#define MUTEX   pthread_mutex_t
#define thread_create(proc,args,thread)		pthread_create(& thread,NULL,proc,args)
#define mutex_create(mutex)				    pthread_mutex_init(& mutex,NULL)
#define mutex_lock(mutex)				    pthread_mutex_lock(& mutex)
#define mutex_unlock(mutex)				    pthread_mutex_unlock(& mutex)
#define mutex_destroy(mutex)				pthread_mutex_destroy(& mutex)
#define thread_join(thread)					pthread_join(thread,NULL)
#define thread_destroy(thread)              thread
#endif
