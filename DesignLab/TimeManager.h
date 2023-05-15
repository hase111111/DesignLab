// ************************************
// 
// ASURA I
// 
// TimeManager.h
// 
// 
// 時間管理
// 
// ************************************

#pragma once
#include <windows.h>
#include <stdio.h>

/// 構造体宣言
typedef struct
{
	LARGE_INTEGER nFreq;
	LARGE_INTEGER nStart;
	LARGE_INTEGER nNow;

	DWORD deltaCount;

	BOOL isCounting;

} TimeInfo;	/// 通信時間

/// 初期化
void time_initialize(TimeInfo* time);

/// 計測スタート
void time_start(TimeInfo* time);

/// 経過時間取得
DWORD time_getCount(TimeInfo* time);

void time_wait(TimeInfo* time, DWORD waitMSEC);
