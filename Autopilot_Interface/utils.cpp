/**
* @file utils.cpp
*
* @author Spencer Watza, <s.g.watza@gmail.com>
* @author James Jenkins, <james.p.jenkins@wmich.edu>
*/
#include <stdio.h>
#include <stdint.h>
#include <Windows.h>

#include "utils.h"


void usleep(__int64 usec)
{
	HANDLE timer;
	LARGE_INTEGER ft;

	// Convert to 100 nanosecond interval, negative value indicates relative time
	ft.QuadPart = -(10 * usec);

	timer = CreateWaitableTimer(NULL, TRUE, NULL);
	SetWaitableTimer(timer, &ft, 0, NULL, NULL, 0);
	WaitForSingleObject(timer, INFINITE);
	CloseHandle(timer);
}


uint64_t get_time_usec()
{
	SYSTEMTIME st;
	FILETIME ft;
	//static struct timeval _time_stamp;
	GetSystemTime(&st);
	SystemTimeToFileTime(&st, &ft);
	_ULARGE_INTEGER li;
	li.HighPart = ft.dwHighDateTime;
	li.LowPart = ft.dwLowDateTime;
	return li.QuadPart;
}