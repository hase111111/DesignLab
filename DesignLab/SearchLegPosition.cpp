#include "SearchLegPosition.h"
#include <iostream>
#include "MyMath.h"

//脚の半径　2つの脚を　LEGR * 2　以上近づけることはできない
constexpr auto LEGR = 20;

SearchPossibleLegPosition::SearchPossibleLegPosition()
{
	for (int i = 0; i < HexapodConst::LEG_NUM; ++i)
	{
		m_is_ground[i] = false;
		m_leg_state[i] = 0;
	}
}

int SearchPossibleLegPosition::PossibleLegPoint_Rotation()
{
	if (mp_MapState == nullptr)return 0;

	my_vec::SVector diff_l2[HexapodConst::LEG_NUM];	//回転中心からleg2までのベクトル
	float Leg_th[HexapodConst::LEG_NUM];//回転中心からleg2までのベクトルがx軸となす角

	//4つの変数の初期化
	for (int i = 0; i < HexapodConst::LEG_NUM; ++i)
	{
		//-pi～pi
		diff_l2[i] = phantomX.getGlobalLeg2Pos(i) - phantomX.getRotaionCenter();

		Leg_th[i] = atan2(diff_l2[i].y, diff_l2[i].x);
	}

	//メンバ変数　LegGroundablePointNum[i][j]の初期化
	for (int i = 0; i < HexapodConst::LEG_NUM; ++i)
	{
		for (int j = 0; j < leg_state::DISCRETE_NUM; ++j)
		{
			LegGroundablePointNum[i][j] = 0;
		}
	}

	//可動範囲内の脚接地候補点を、それぞれの脚のそれぞれの離散化位置ごとに格納


	my_vec::SVector leg1_candidate;//coxaから脚接地可能点へのベクトル（グローバル）
	my_vec::SVector diff_point;	//回転中心から候補点までの差分ベクトル
	float th;//回転中心から候補点までの差分ベクトルのx軸とのなす角
	float dth;//回転中心から候補点までベクトルがx軸となす角　と　回転中心からleg2までのベクトルがx軸となす角　の　差
	my_vec::SVector diff;//leg2と脚接地候補点の差分ベクトル
	float sort_p_LegGroundableCandidatePoint[HexapodConst::LEG_NUM][leg_state::DISCRETE_NUM][1000];

	for (int i = 0; i < HexapodConst::LEG_NUM; ++i)
	{
		if (m_is_ground[i] == true) { continue; }

		my_vec::SVector point1, point2;//i番目の脚の可動範囲が内接する正方形の左下と右上の角の座標
		phantomX.calculateRangeOfMovement(i, point1, point2);

		int x1 = MapState::getDevideMapNumX(std::min(point1.x, point2.x));
		int x2 = MapState::getDevideMapNumX(std::max(point1.x, point2.x));
		int y1 = MapState::getDevideMapNumY(std::min(point1.y, point2.y));
		int y2 = MapState::getDevideMapNumY(std::max(point1.y, point2.y));

		//x1からx2，y1からy2までのブロック
		for (int ix = x1; ix <= x2; ix++)
		{
			for (int iy = y1; iy <= y2; iy++)
			{
				for (int n = 0; n < mp_MapState->getPointNumFromDevideMap(ix,iy); ++n)
				{
					//あるブロックに存在する脚設置可能点の数
					leg1_candidate = mp_MapState->getPosFromDevideMap(ix, iy, n) - phantomX.getGlobalCoxaJointPos(i);	//マップの座標はグローバル

					//これで可動範囲内の点だけ取れると思うが、逆にいまの重心高さを変えれば届くような点はここでは除外される。くやしぃ～、（だが、それでいいっ！！）おそらく
					if (phantomX.check_touchdown_point2(i, VCangeBodyToLeg(leg1_candidate), phantomX.getGlobalMyPosition().z - mp_MapState->getPosFromDevideMap(ix, iy, n).z))
					{
						//VCangeは謎、変更20200612 VCangeはhexapod.hの座標系に合わせてる
						diff_point = mp_MapState->getPosFromDevideMap(ix, iy, n) - phantomX.getRotaionCenter();
						th = atan2(diff_point.y, diff_point.x);
						dth = (th - Leg_th[i]) * phantomX.getTargetRotation().z;

						//外積が+のとき左回り,-のとき右回り(回転中心から現在の脚へのベクトル×回転中心からマップへのベクトル)
						//外積が+で角度の差が-,外積が-で差が+はatan2の区間をまたいでいる
						diff = mp_MapState->getPosFromDevideMap(ix, iy, n) - phantomX.getGlobalLeg2Pos(i);

						//離散化位置4
						if (my_vec::VMag(diff) < 50)
						{
							//現在の脚先と距離が近い場合は同じ脚接地点（計算誤差）	
							if (m_leg_state[i] == 4)
							{
								p_LegGroundableCandidatePoint[i][3][LegGroundablePointNum[i][3]] = mp_MapState->getPosFromDevideMap(ix, iy, n);	//離散化位置4 - 1で配列のindexは3
								sort_p_LegGroundableCandidatePoint[i][3][LegGroundablePointNum[i][3]] = 50.0f - my_vec::VMag(diff);//現在の位置に近いほど値が大きい
								LegGroundablePointNum[i][3]++;

								if (LegGroundablePointNum[i][3] > LegGroundableCandidatePoint_MAX - 1) { return -1; }
							}
						}
						//離散化位置567
						else if ((diff_l2[i].x * diff_point.y - diff_l2[i].y * diff_point.x) * phantomX.getTargetRotation().z > 0.01f)
						{
							//目標方向 //角度差の±でもいいけど20200612
							if (dth < -my_math::MY_FLT_PI)
							{//差の修正
								dth = dth + 2.0f * my_math::MY_FLT_PI;
							}
							else if (dth > my_math::MY_FLT_PI)
							{
								dth = dth - 2.0f * my_math::MY_FLT_PI;
							}

							if (diff.z > 5.0f)
							{	
								//候補点がleg2より高いなら 何ミリ高いかはロボットとカメラの精度によるけどどうやって決めよう　とりあえず10mmで
								if (m_leg_state[i] == 7)
								{
									p_LegGroundableCandidatePoint[i][6][LegGroundablePointNum[i][6]] = mp_MapState->getPosFromDevideMap(ix, iy, n);	//離散化位置7  -1で配列のindexは6
									sort_p_LegGroundableCandidatePoint[i][6][LegGroundablePointNum[i][6]] = dth;
									LegGroundablePointNum[i][6]++;
									if (LegGroundablePointNum[i][6] > LegGroundableCandidatePoint_MAX - 1) { return -1; }
								}
							}
							else if (diff.z < -5.0)
							{	
								//候補点がleg2より低い
								if (m_leg_state[i] == 5)
								{
									p_LegGroundableCandidatePoint[i][4][LegGroundablePointNum[i][4]] = mp_MapState->getPosFromDevideMap(ix, iy, n);	//離散化位置5  -1で配列のindexは4
									sort_p_LegGroundableCandidatePoint[i][4][LegGroundablePointNum[i][4]] = dth;
									LegGroundablePointNum[i][4]++;
									if (LegGroundablePointNum[i][4] > LegGroundableCandidatePoint_MAX - 1) { return -1; }
								}
							}
							else
							{	
								//±10mmなら同じ高さとして扱う　っていう風にしてる190609
								if (m_leg_state[i] == 6)
								{
									p_LegGroundableCandidatePoint[i][5][LegGroundablePointNum[i][5]] = mp_MapState->getPosFromDevideMap(ix, iy, n);	//離散化位置6  -1で配列のindexは5
									sort_p_LegGroundableCandidatePoint[i][5][LegGroundablePointNum[i][5]] = dth;
									LegGroundablePointNum[i][5]++;
									if (LegGroundablePointNum[i][5] > LegGroundableCandidatePoint_MAX - 1) { return -1; }
								}
							}
						}

						//離散化位置123
						else if ((diff_l2[i].x * diff_point.y - diff_l2[i].y * diff_point.x) * phantomX.getTargetRotation().z < -0.01f)
						{
							//目標と逆方向
							if (dth < -my_math::MY_FLT_PI)
							{//差の修正
								dth = dth + 2.0f * my_math::MY_FLT_PI;
							}
							else if (dth > my_math::MY_FLT_PI)
							{
								dth = dth - 2.0f * my_math::MY_FLT_PI;
							}

							if (diff.z > 5.0f)
							{	//候補点がleg2より高いなら 何ミリ高いかはロボットとカメラの精度によるけどどうやって決めよう　とりあえず10mmで
								if (m_leg_state[i] == 3)
								{
									p_LegGroundableCandidatePoint[i][2][LegGroundablePointNum[i][2]] = mp_MapState->getPosFromDevideMap(ix, iy, n);	//離散化位置3  -1で配列のindexは2
									sort_p_LegGroundableCandidatePoint[i][2][LegGroundablePointNum[i][2]] = dth;
									LegGroundablePointNum[i][2]++;
									if (LegGroundablePointNum[i][2] > LegGroundableCandidatePoint_MAX - 1) { return -1; }
								}
							}
							else if (diff.z < -5.0f)
							{	//候補点がleg2より低い
								if (m_leg_state[i] == 1)
								{
									p_LegGroundableCandidatePoint[i][0][LegGroundablePointNum[i][0]] = mp_MapState->getPosFromDevideMap(ix, iy, n);	//離散化位置1  -1で配列のindexは0
									sort_p_LegGroundableCandidatePoint[i][0][LegGroundablePointNum[i][0]] = dth;
									LegGroundablePointNum[i][0]++;
									if (LegGroundablePointNum[i][0] > LegGroundableCandidatePoint_MAX - 1) { return -1; }
								}
							}
							else
							{	//±10mmなら同じ高さとして扱う　っていう風にしてる190609
								if (m_leg_state[i] == 2)
								{
									p_LegGroundableCandidatePoint[i][1][LegGroundablePointNum[i][1]] = mp_MapState->getPosFromDevideMap(ix, iy, n);	//離散化位置2  -1で配列のindexは1
									sort_p_LegGroundableCandidatePoint[i][1][LegGroundablePointNum[i][1]] = dth;
									LegGroundablePointNum[i][1]++;
									if (LegGroundablePointNum[i][1] > LegGroundableCandidatePoint_MAX - 1) { return -1; }
								}
							}
						}
						//離散化位置4
						else
						{
							//角度差は小さいが遠い場所
							if (m_leg_state[i] == 4)
							{
								if (dth < -my_math::MY_FLT_PI)
								{
									dth = dth + 2.0f * my_math::MY_FLT_PI;
								}
								else if (dth > my_math::MY_FLT_PI)
								{
									dth = dth - 2.0f * my_math::MY_FLT_PI;
								}

								p_LegGroundableCandidatePoint[i][3][LegGroundablePointNum[i][3]] = mp_MapState->getPosFromDevideMap(ix, iy, n);	//この場合も離散化位置4扱いで どうしよ190609
								sort_p_LegGroundableCandidatePoint[i][3][LegGroundablePointNum[i][3]] = dth;
								LegGroundablePointNum[i][3]++;
								if (LegGroundablePointNum[i][3] > LegGroundableCandidatePoint_MAX - 1) { return -1; }
							}
						}
					}
				}
			}
		}
	}

	//ここで脚接地可能点のソート
	//p_LegGroundableCandidatePoint[][]の中のLegGroundablePointNum[][]個のデータをソートsort_p_LegGroundableCandidatePointの大きい順
	LegGroundableCandidatePointsort_Th(p_LegGroundableCandidatePoint, sort_p_LegGroundableCandidatePoint, LegGroundablePointNum);

	return 0;
}

int SearchPossibleLegPosition::Target_delta_comz() const
{
	if (mp_MapState == nullptr)return 0;

	//可動範囲内の脚接地候補点を、それぞれの脚のそれぞれの離散化位置ごとに格納

	float max_map_height = -1000.0f;
	my_vec::SVector point1, point2;	//i番目の脚の可動範囲が内接する正方形の左下と右上の角の座標

	{
		phantomX.calculateRangeOfMovement(0, point1, point2);//ROMが内接する長方形

		int x1 = MapState::getDevideMapNumX(std::min(point1.x, point2.x));
		int x2 = MapState::getDevideMapNumX(std::max(point1.x, point2.x));
		int y1 = MapState::getDevideMapNumY(std::min(point1.y, point2.y));
		int y2 = MapState::getDevideMapNumY(std::max(point1.y, point2.y));

		//x1からx2までのブロック，y1からy2までのブロックを総当たりする
		for (int ix = x1; ix <= x2; ++ix)
		{
			for (int iy = y1; iy <= y2; ++iy)
			{
				for (int n = 0; n < mp_MapState->getPointNumFromDevideMap(ix, iy); ++n)
				{
					if (max_map_height < mp_MapState->getPosFromDevideMap(ix, iy, n).z) { max_map_height = mp_MapState->getPosFromDevideMap(ix, iy, n).z; }
				}
			}
		}
	}

	{
		phantomX.calculateRangeOfMovement(HexapodConst::LEG_NUM - 1, point1, point2);

		int x1 = MapState::getDevideMapNumX(std::min(point1.x, point2.x));
		int x2 = MapState::getDevideMapNumX(std::max(point1.x, point2.x));
		int y1 = MapState::getDevideMapNumY(std::min(point1.y, point2.y));
		int y2 = MapState::getDevideMapNumY(std::max(point1.y, point2.y));

		//x1からx2，y1からy2までのブロック
		for (int ix = x1; ix <= x2; ++ix)
		{
			for (int iy = y1; iy <= y2; ++iy)
			{
				for (int n = 0; n < mp_MapState->getPointNumFromDevideMap(ix, iy); ++n)
				{
					if (max_map_height < mp_MapState->getPosFromDevideMap(ix, iy, n).z) { max_map_height = mp_MapState->getPosFromDevideMap(ix, iy, n).z; }
				}
			}
		}
	}

	float target_delta_comz = max_map_height + HexapodConst::VERTICAL_MIN_RANGE - phantomX.getGlobalMyPosition().z;

	return (int)target_delta_comz;
}

float SearchPossibleLegPosition::calculateCollisionAvoidanceMovement() const
{
	if (mp_MapState == nullptr)return 0;

	//1.胴体が内接する長方形の領域を計算
	//2.長方形の領域を含む、それよりも大きい範囲内にある脚接地可能点を抽出する。
	//3.各脚接地可能点に対して、長方形の領域内かどうかを調べる。
	//4.領域内の脚設置可能点の中の最大高さを求める。
	//5.その点と胴体が一定値以上離れているかを判定する。
	//6.接触する場合、その点から一定値はなれるように重心高さの変更量を決定する。

	//1.胴体が内接する長方形の領域を計算
	float minx = MapConst::MAP_MAX_HORIZONTAL;
	float miny = MapConst::MAP_MAX_FORWARD;
	float maxx = MapConst::MAP_MIN_HORIZONTAL;
	float maxy = MapConst::MAP_MIN_FORWARD;

	//coxaを含む胴体が内接する、xy平面に平行な長方形の左下の頂点座標（グローバル）
	for (int i = 0; i < HexapodConst::LEG_NUM; ++i)
	{
		my_vec::SVector Fem_buf = phantomX.getGlobalFemurJointPos(i);
		if (Fem_buf.x < minx) { minx = Fem_buf.x; }
		if (Fem_buf.y < miny) { miny = Fem_buf.y; }
		if (maxx < Fem_buf.x) { maxx = Fem_buf.x; }
		if (maxy < Fem_buf.y) { maxy = Fem_buf.y; }
	}

	const float fem_margin = 10.0f; //第2関節が若干ぶつかってそうだったから、実際より大きめに設定する。直線の場合x軸はなくてもいいが一応。
	minx -= fem_margin;
	miny -= fem_margin;
	maxx += fem_margin;
	maxy += fem_margin;

	//2.長方形の領域を含む、それよりも大きい範囲内にある脚接地可能点を抽出する。
	//3.各脚接地可能点に対して、長方形の領域内かどうかを調べる。
	//4.領域内の脚設置可能点の中の最大高さを求める。回転させるときは平面と点の距離で導出する。（いまはしてない）
	float max_map_height = -100000.0f;

	//x1からx2，y1からy2までのブロック
	for (int ix = MapState::getDevideMapNumX(minx); ix <= MapState::getDevideMapNumX(maxx); ++ix)
	{	
		for (int iy = MapState::getDevideMapNumY(miny); iy <= MapState::getDevideMapNumY(maxy); ++iy)
		{	
			for (int n = 0; n < mp_MapState->getPointNumFromDevideMap(ix, iy); ++n)
			{	
				//xが範囲内か，yが範囲内か，
				if (minx <= mp_MapState->getPosFromDevideMap(ix, iy, n).x && mp_MapState->getPosFromDevideMap(ix, iy, n).x <= maxx)
				{
					if (miny <= mp_MapState->getPosFromDevideMap(ix, iy, n).y && mp_MapState->getPosFromDevideMap(ix, iy, n).y <= maxy)
					{
						//あるブロックに存在する脚設置可能点の最大高さ
						if (max_map_height < mp_MapState->getPosFromDevideMap(ix, iy, n).z) { max_map_height = mp_MapState->getPosFromDevideMap(ix, iy, n).z; }
					}
				}
			}
		}
	}

	//5.その点と胴体が一定値以上離れているかを判定する。
	//6.離れていない場合、その点から一定値はなれるように重心高さの変更量を決定する。
	if (max_map_height + HexapodConst::VERTICAL_MIN_RANGE <= phantomX.getGlobalMyPosition().z)
	{
		//十分離れているときは移動する必要はないので0を返す．
		return 0; 
	}
	else
	{
		//離れていない場合は衝突しているので，衝突しないために必要な移動量を返す
		return (max_map_height - phantomX.getGlobalMyPosition().z) + HexapodConst::VERTICAL_MIN_RANGE; 
	}
}

void SearchPossibleLegPosition::checkLegCross(my_vec::SVector p_LegGroundableCandidatePoint[6][leg_state::DISCRETE_NUM][1000], int LegGroundablePointNum[6][leg_state::DISCRETE_NUM])
{
	//脚先が近すぎたらダメ　他の脚と交差したらダメ
	//考えるのは接地している脚のみ

	//0番脚　衝突可能性があるのは1番，5番脚
	for (int i = 0; i < leg_state::DISCRETE_NUM; ++i) {
		individualcCheckLegCross(p_LegGroundableCandidatePoint[0][i], &LegGroundablePointNum[0][i], 0, 1, 5);
	}
	/*individualcCheckLegCross(p_LegGroundableCandidatePoint[0][1], &LegGroundablePointNum[0][1], 0, 1, 5);
	individualcCheckLegCross(p_LegGroundableCandidatePoint[0][2], &LegGroundablePointNum[0][2], 0, 1, 5);
	individualcCheckLegCross(p_LegGroundableCandidatePoint[0][3], &LegGroundablePointNum[0][3], 0, 1, 5);*/

	//1番脚　衝突可能性があるのは0番，2番脚
	for (int i = 0; i < leg_state::DISCRETE_NUM; ++i) {
		individualcCheckLegCross(p_LegGroundableCandidatePoint[1][i], &LegGroundablePointNum[1][i], 1, 2, 0);
	}
	/*individualcCheckLegCross(p_LegGroundableCandidatePoint[1][1], &LegGroundablePointNum[1][1], 1, 2, 0);
	individualcCheckLegCross(p_LegGroundableCandidatePoint[1][2], &LegGroundablePointNum[1][2], 1, 2, 0);
	individualcCheckLegCross(p_LegGroundableCandidatePoint[1][3], &LegGroundablePointNum[1][3], 1, 2, 0);*/

	//2番脚　衝突可能性があるのは1番，3番脚
	for (int i = 0; i < leg_state::DISCRETE_NUM; ++i) {
		individualcCheckLegCross(p_LegGroundableCandidatePoint[2][i], &LegGroundablePointNum[2][i], 2, 3, 1);
	}
	/*individualcCheckLegCross(p_LegGroundableCandidatePoint[2][1], &LegGroundablePointNum[2][1], 2, 3, 1);
	individualcCheckLegCross(p_LegGroundableCandidatePoint[2][2], &LegGroundablePointNum[2][2], 2, 3, 1);
	individualcCheckLegCross(p_LegGroundableCandidatePoint[2][3], &LegGroundablePointNum[2][3], 2, 3, 1);*/

	//3番脚　衝突可能性があるのは2番，4番脚
	for (int i = 0; i < leg_state::DISCRETE_NUM; ++i) {
		individualcCheckLegCross(p_LegGroundableCandidatePoint[3][i], &LegGroundablePointNum[3][i], 3, 4, 2);
	}
	/*individualcCheckLegCross(p_LegGroundableCandidatePoint[3][1], &LegGroundablePointNum[3][1], 3, 4, 2);
	individualcCheckLegCross(p_LegGroundableCandidatePoint[3][2], &LegGroundablePointNum[3][2], 3, 4, 2);
	individualcCheckLegCross(p_LegGroundableCandidatePoint[3][3], &LegGroundablePointNum[3][3], 3, 4, 2);*/

	//4番脚　衝突可能性があるのは3番，5番脚
	for (int i = 0; i < leg_state::DISCRETE_NUM; ++i) {
		individualcCheckLegCross(p_LegGroundableCandidatePoint[4][i], &LegGroundablePointNum[4][i], 4, 5, 3);
	}
	/*individualcCheckLegCross(p_LegGroundableCandidatePoint[4][1], &LegGroundablePointNum[4][1], 4, 5, 3);
	individualcCheckLegCross(p_LegGroundableCandidatePoint[4][2], &LegGroundablePointNum[4][2], 4, 5, 3);
	individualcCheckLegCross(p_LegGroundableCandidatePoint[4][3], &LegGroundablePointNum[4][3], 4, 5, 3);*/

	//5番脚　衝突可能性があるのは4番，0番脚
	for (int i = 0; i < leg_state::DISCRETE_NUM; ++i) {
		individualcCheckLegCross(p_LegGroundableCandidatePoint[5][i], &LegGroundablePointNum[5][i], 5, 0, 4);
	}
	/*individualcCheckLegCross(p_LegGroundableCandidatePoint[5][1], &LegGroundablePointNum[5][1], 5, 0, 4);
	individualcCheckLegCross(p_LegGroundableCandidatePoint[5][2], &LegGroundablePointNum[5][2], 5, 0, 4);
	individualcCheckLegCross(p_LegGroundableCandidatePoint[5][3], &LegGroundablePointNum[5][3], 5, 0, 4);*/
}

bool SearchPossibleLegPosition::individualcCheckLegCross(my_vec::SVector* LegGroundableCandidatePoint, int* LegGroundablePointNum, int checkedLegNum, int checkLegNum1, int checkLegNum2) 
{
	for (int i = 0; i < *LegGroundablePointNum; i++) 
	{
		//脚が接近しすぎていないかどうか
		if (VMag2(phantomX.getGlobalLegPos(checkLegNum1), LegGroundableCandidatePoint[i]) > LEGR) 
		{	
			//脚が接近しすぎていないかどうか
			if (VMag2(phantomX.getGlobalLegPos(checkLegNum2), LegGroundableCandidatePoint[i]) > LEGR) 
			{	
				if (!isCross(phantomX.getGlobalCoxaJointPos(checkedLegNum), LegGroundableCandidatePoint[i], phantomX.getGlobalCoxaJointPos(checkLegNum1), phantomX.getGlobalLegPos(checkLegNum1)))
				{
					if (!isCross(phantomX.getGlobalCoxaJointPos(checkedLegNum), LegGroundableCandidatePoint[i], phantomX.getGlobalCoxaJointPos(checkLegNum2), phantomX.getGlobalLegPos(checkLegNum2)))
					{
						//[0]が脚接地候補
						LegGroundableCandidatePoint[0] = LegGroundableCandidatePoint[i];
						return false;
					}
				}
			}
		}
	}

	*LegGroundablePointNum = 0;
	return true;	//干渉しない脚位置が見当たらない
}

bool SearchPossibleLegPosition::isCross(const my_vec::SVector& s1, const my_vec::SVector& e1, const  my_vec::SVector& s2, const my_vec::SVector& e2) const
{
	my_vec::SVector xy_s1, xy_s2, xy_e1, xy_e2;

	xy_s1 = s1;
	xy_s2 = s2;
	xy_e1 = e1;
	xy_e2 = e2;

	xy_s1.z = 0;
	xy_s2.z = 0;
	xy_e1.z = 0;
	xy_e2.z = 0;

	my_vec::SVector v, v1, v2;
	float t1, t2;
	v1 = xy_e1 - xy_s1;
	v2 = xy_e2 - xy_s2;
	v  = xy_s1 - xy_s2;

	//2つのベクトルが平行
	if ((my_vec::VCross(v1, v2)).lengthSquare() < 0.01)
	{
		return false;	//2つのアークの始点，終点のどれも一致していない
	}

	t1 = -my_vec::VCross(v, v2).z / my_vec::VCross(v1, v2).z;
	t2 = -my_vec::VCross(v, v1).z / my_vec::VCross(v1, v2).z;

	const float _err = 0.01f;

	if (-_err < t1 && t1 < 1.0f + _err && -_err < t2 && t2 < 1.0f + _err)
	{
		const float _err = 0.01f;

		//線分同士が交差しているならば
		if (my_vec::isEqualVector(xy_s1, xy_s2, _err) || my_vec::isEqualVector(xy_s1, xy_e2, _err) || my_vec::isEqualVector(xy_s2, xy_e1, _err) || my_vec::isEqualVector(xy_e1, xy_e2, _err))
		{
			return true;
		}

		return true;
	}
	return false;
}

my_vec::SVector SearchPossibleLegPosition::VCangeBodyToLeg(const my_vec::SVector& Vin) const
{
	//グローバル座標→ロボット脚座標(hexapodクラス)
	return my_vec::VRot(my_vec::VGet(Vin.x, -Vin.y, -Vin.z), phantomX.getGlobalMyDirectionthP(), phantomX.getGlobalMyDirectionthR(), -phantomX.getGlobalMyDirectionthY() - my_math::MY_FLT_PI / 2.0f);
}

void SearchPossibleLegPosition::LegGroundableCandidatePointsort_Th(my_vec::SVector p_LegGroundableCandidatePoint[HexapodConst::LEG_NUM][leg_state::DISCRETE_NUM][1000], float sort_p_LGCP[HexapodConst::LEG_NUM][leg_state::DISCRETE_NUM][1000], int LegGroundablePointNum[HexapodConst::LEG_NUM][leg_state::DISCRETE_NUM])
{
	//sort_～～は回転角度,p_leg～～は脚接地可能点
	for (int i = 0; i < HexapodConst::LEG_NUM; i++) 
	{
		for (int ii = 0; ii < leg_state::DISCRETE_NUM; ii++)
{
			for (int j = 0; j < LegGroundablePointNum[i][ii]; j++) 
			{
				for (int jj = LegGroundablePointNum[i][ii] - 1; jj > j; jj--) 
				{
					//進行方向への距離が大きいほどjjの小さい位置へ行く,バブルソート
					if (sort_p_LGCP[i][ii][jj] > sort_p_LGCP[i][ii][jj - 1]) 
					{
						float ft = sort_p_LGCP[i][ii][jj];
						my_vec::SVector Vt = p_LegGroundableCandidatePoint[i][ii][jj];

						sort_p_LGCP[i][ii][jj] = sort_p_LGCP[i][ii][jj - 1];
						p_LegGroundableCandidatePoint[i][ii][jj] = p_LegGroundableCandidatePoint[i][ii][jj - 1];

						sort_p_LGCP[i][ii][jj - 1] = ft;
						p_LegGroundableCandidatePoint[i][ii][jj - 1] = Vt;
					}
				}
			}
		}
	}

	//脚位置からほかの脚に当たらないように脚位置を選択
	checkLegCross(p_LegGroundableCandidatePoint, LegGroundablePointNum);
}

void SearchPossibleLegPosition::SetLegGroundableCandidatePointMAX(const int LGCP_MAX)
{
	LegGroundableCandidatePoint_MAX = LGCP_MAX;
}

void SearchPossibleLegPosition::setLegState(const int _leg_state)
{
	for (int i = 0; i < HexapodConst::LEG_NUM; ++i)
	{
		if (leg_state::isGrounded(_leg_state, i) == true)
		{
			m_is_ground[i] = true;
		}
		else 
		{
			m_is_ground[i] = false;
		}

		m_leg_state[i] = leg_state::getLegState(_leg_state, i);
	}
}
