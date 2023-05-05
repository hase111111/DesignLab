// ************************************
// 
// ASURA I
// 
// TimeManager.c
// 
// 
// 時間管理
// 
// ************************************

#include "TimeManager.h"

/// 初期化
void time_initialize(TimeInfo* time)
{
	QueryPerformanceFrequency( &(time->nFreq) );
	memset(&(time->nStart), 0, sizeof(LARGE_INTEGER));
	memset(&(time->nNow), 0, sizeof(LARGE_INTEGER));
	time->deltaCount = 0;
	time->isCounting = FALSE;

	return;
}

/// 計測スタート
void time_start(TimeInfo* time)
{
	QueryPerformanceCounter( &(time->nStart) );
	time->isCounting = TRUE;

	return;
}

/// 経過時間取得
DWORD time_getCount(TimeInfo* time)
{
	QueryPerformanceCounter( &(time->nNow) );
	time->deltaCount = (DWORD)( ((time->nNow.QuadPart - time->nStart.QuadPart)*1000)/time->nFreq.QuadPart );

	if (time->isCounting)
		time->isCounting = FALSE;
	else
		time->deltaCount = 0;

	return time->deltaCount;
}

/// 待機
void time_wait(TimeInfo* time, DWORD waitMSEC)
{
	 //time_start(time);

	do
	{
		QueryPerformanceCounter( &(time->nNow) );
		time->deltaCount = (DWORD)( ((time->nNow.QuadPart - time->nStart.QuadPart)*1000)/time->nFreq.QuadPart );

	} while (time->deltaCount < waitMSEC);

	return;
}
