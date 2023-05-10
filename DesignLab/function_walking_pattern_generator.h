#pragma once

#include "listFunc.h"
#include "vectorFunc.h"
#include "hexapod.h"


//ファイルに書き込んである物の説明
void Description_txt( std::ofstream& foutKaisetu);

//現在時刻をコンソールとファイルに出力
int now_time(std::ofstream& fout_log);
int now_time();

//ログを出力する
void output_log(const char *str, std::ofstream& fout_log);

//進行方向を取得する
void getWalkingDirection(myvector::SVector* walkingDirection);

//マップを取得する
void getMap(myvector::SVector *&p_mapData3D, int* mapData3D_MAX, myvector::SVector legPosi[6], myvector::SVector center_of_mass);

//座法変換と回転
myvector::SVector VCangeBodyToLeg(myvector::SVector& Vin, float thP, float thR, float thY);