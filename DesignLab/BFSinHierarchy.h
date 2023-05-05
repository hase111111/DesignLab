#pragma once
#include "pch.h"

extern const int edge_index[37];
extern const char edge_list[298];
extern const char Raised_RF[23];//右前脚
extern const char Raised_RM[23];
extern const char Raised_RR[23];	//右後脚
extern const char Raised_LR[23];	//左後脚
extern const char Raised_LM[23];	//左中脚
extern const char Raised_LF[23];		//左前脚

extern const char comType1[9]; //パターン6
extern const char comType2[9]; //パターン1
extern const char comType3[9]; //パターン2
extern const char comType4[9]; //パターン3
extern const char comType5[9]; //パターン4
extern const char comType6[9]; //パターン5
	   
extern const char comType7[9]; //パターン7
extern const char comType8[9]; //パターン8
extern const char comType0[18]; //パターン0 どの隣りあった足も上げることができない


void clear_node_bfs(const char* clearnum, char n, bool* visited);
void bfs_in_hierarchy(char start, char* prev, char* cost, bool* visited);