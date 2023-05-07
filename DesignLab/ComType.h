#pragma once
#include "Define.h"

//
//leg_stateの上位bitにて表されているもの．詳細は波東さんの修論で．
//BFSinHierarchy・CreateComCandidate・PassFindingと様々なファイルに跨る処理をまとめたくて作ったもの.
//
// 重心パターン … 10通りある．leg_state の上位bitにて表現されるもの
//  重心タイプ  … 35通りある．ground_leg から可能なものに数字が割り振ってある．
//		と，長谷川は定義した．波東先輩はこれをこれらを全てタイプ(COMType)として読んでいるので区別するためにそう定義した.
//

namespace ComType 
{
	//接地している脚をtrueとしたbool型の配列と，重心パターンから，可能なものかを出力する
	bool isAbleCoM(const int _com_pattern, const bool _ground_leg[Define::LEG_NUM]);

	//接地している脚をtrueした配列から，重心タイプを出力する関数．該当しないならば負の値を返す
	char getComTypeFromGroundLeg(const bool _ground_leg[Define::LEG_NUM]);
}
