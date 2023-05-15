#include "SearchLegPosition.h"
#include <iostream>

#define _USE_MATH_DEFINES


SearchPossibleLegPosition::SearchPossibleLegPosition()
{
}

//初期値を入力後実行　エラーは -1を返す（大木さんのやつ）
int SearchPossibleLegPosition::calculateLegGroundablePoint() 
{
	myvector::SVector ll1;
for (int ii = 0; ii < 6; ii++)for (int i = 0; i < 7; i++)LegGroundablePointNum[ii][i] = 0;			//LegGroundablePointNum[ii][i]の初期化

	for (int ix = 0;ix < mapData3D_MAX; ix++)
	{	//マップ総当たり
		for(int i = 0; i < HexapodConst::LEG_NUM; i++)
		{
			ll1 = myvector::subVec(p_mapData3D[ix], phantomX.getGlobalCoxaJointPos(i));

			if( phantomX.isLegWithinRange(i, VCangeBodyToLeg(ll1)))
			{	//可動範囲内にあるかどうか
				myvector::SVector diff = myvector::subVec(p_mapData3D[ix], phantomX.getGlobalLeg2Pos(i));
				//if(myvector::VDot(phantomX.getTargetDirection(), myvector::subVec(p_mapData3D[ix], phantomX.getGlobalLeg2Pos(i))) > 1){			//進行方向側
				if(myvector::VDot(phantomX.getTargetDirection(), diff) > 1)
				{			//進行方向側
					if (diff.z > 5.0) 
					{	//基準位置より高い　何mmより高いとするかはロボットとカメラの精度しだいだけどどうやって決めよう　とりあえず一旦10mmで
						p_LegGroundableCandidatePoint[i][6][LegGroundablePointNum[i][6]] = p_mapData3D[ix];	//離散化位置7　-1で6
						LegGroundablePointNum[i][6]++;
						if (LegGroundablePointNum[i][6] > LegGroundableCandidatePoint_MAX - 1) { return -1; }
					} 
					else if (diff.z <= 5.0) 
					{	//基準位置より低い
						p_LegGroundableCandidatePoint[i][4][LegGroundablePointNum[i][4]] = p_mapData3D[ix];	//離散化位置5
						LegGroundablePointNum[i][4]++;
						if (LegGroundablePointNum[i][4] > LegGroundableCandidatePoint_MAX - 1) { return -1; }
					}
					else 
					{
						p_LegGroundableCandidatePoint[i][5][LegGroundablePointNum[i][5]] = p_mapData3D[ix];	//離散化位置6
						LegGroundablePointNum[i][5]++;
						if (LegGroundablePointNum[i][5] > LegGroundableCandidatePoint_MAX - 1) { return -1; }
					}

				}
				else if(myvector::VDot(phantomX.getTargetDirection(), diff) < -1)
				{	//進行方向と逆
					if (diff.z > 5.0) 
					{	//基準位置より高い　
						p_LegGroundableCandidatePoint[i][2][LegGroundablePointNum[i][2]] = p_mapData3D[ix];	//離散化位置3　-1で2
						LegGroundablePointNum[i][2]++;
						if (LegGroundablePointNum[i][2] > LegGroundableCandidatePoint_MAX - 1) { return -1; }
					}
					else if (diff.z <= 5.0) 
					{	//基準位置より低い
						p_LegGroundableCandidatePoint[i][0][LegGroundablePointNum[i][0]] = p_mapData3D[ix];	//離散化位置1
						LegGroundablePointNum[i][0]++;
						if (LegGroundablePointNum[i][0] > LegGroundableCandidatePoint_MAX - 1) { return -1; }
					} else 
					{
						p_LegGroundableCandidatePoint[i][1][LegGroundablePointNum[i][1]] = p_mapData3D[ix];	//離散化位置2
						LegGroundablePointNum[i][1]++;
						if (LegGroundablePointNum[i][1] > LegGroundableCandidatePoint_MAX - 1) { return -1; }
					}
				/*}else if(myvector::VDot(phantomX.getTargetDirection(), myvector::subVec(p_mapData3D[ix], phantomX.getGlobalLeg2Pos(i))) < -1){	//進行方向と逆

					p_LegGroundableCandidatePoint[i][1][ LegGroundablePointNum[i][1] ] = p_mapData3D[ix];
					LegGroundablePointNum[i][1]++;
					if(LegGroundablePointNum[i][1] > LegGroundableCandidatePoint_MAX - 1)return -1;*/

				}
				else
				{		//進行方向に寄与なし	離散化位置4
					p_LegGroundableCandidatePoint[i][3][ LegGroundablePointNum[i][3] ] = p_mapData3D[ix];
					LegGroundablePointNum[i][3]++;
					if (LegGroundablePointNum[i][3] > LegGroundableCandidatePoint_MAX - 1) { return -1; }
				}
			}
		}
	}

	//ここで脚接地可能点のソート
	//p_LegGroundableCandidatePoint[][]の中のLegGroundablePointNum[][]個のデータをソート
	LegGroundableCandidatePointsort(p_LegGroundableCandidatePoint, LegGroundablePointNum);

	//他の脚に当たるものを削除
	checkLegCross(p_LegGroundableCandidatePoint, LegGroundablePointNum);

	for(int iii = 0; iii < HexapodConst::LEG_NUM; iii++)
	{
		for(int ii = 0; ii < HexapodConst::LEG_NUM; ii++)
		{
			//for(int i = 1; i < 4; i ++){
			if(ii != iii)
			{
				for (int i = 1; i < DISCRETE_NUM; i++) 
				{
					if (LegGroundablePointNum[ii][i] != 0) 
					{
						if (myvector::VMag2(phantomX.getGlobalLegPos(iii), p_LegGroundableCandidatePoint[ii][i][0]) < 20) 
						{	//他の脚の接地点と近すぎないこと
							std::cout << "LegGroundablePointNum[" << ii << "][" << i << "]\n";
							std::cout << LegGroundablePointNum[ii][i] << "\n";

							std::cout << "_SearchPossibleLegPosition.phantomX.getGlobalLegPos(" << iii << ")\n";
							myvector::VectorOutPut(phantomX.getGlobalLegPos(iii));

							std::cout << "_SearchPossibleLegPosition.p_LegGroundableCandidatePoint[" << ii << "][" << i << "][0]\n";
							myvector::VectorOutPut(p_LegGroundableCandidatePoint[ii][i][0]);

							LegGroundablePointNum[ii][i] = 0;
						}
					}
				}
			}
		}
	}

	return 0;
}

//void SearchPossibleLegPosition::checkLegCross(myvector::VECTOR p_LegGroundableCandidatePoint[6][4][1000], int LegGroundablePointNum[6][4]){
void SearchPossibleLegPosition::checkLegCross(myvector::SVector p_LegGroundableCandidatePoint[6][DISCRETE_NUM][1000], int LegGroundablePointNum[6][DISCRETE_NUM]){
	//脚先が近すぎたらダメ　他の脚と交差したらダメ
	//考えるのは接地している脚のみ

	//0番脚　衝突可能性があるのは1番，5番脚
	for (int i = 0; i < DISCRETE_NUM; ++i) {
		individualcCheckLegCross(p_LegGroundableCandidatePoint[0][i], &LegGroundablePointNum[0][i], 0, 1, 5);
	}
	/*individualcCheckLegCross(p_LegGroundableCandidatePoint[0][1], &LegGroundablePointNum[0][1], 0, 1, 5);
	individualcCheckLegCross(p_LegGroundableCandidatePoint[0][2], &LegGroundablePointNum[0][2], 0, 1, 5);
	individualcCheckLegCross(p_LegGroundableCandidatePoint[0][3], &LegGroundablePointNum[0][3], 0, 1, 5);*/

	//1番脚　衝突可能性があるのは0番，2番脚
	for (int i = 0; i < DISCRETE_NUM; ++i) {
		individualcCheckLegCross(p_LegGroundableCandidatePoint[1][i], &LegGroundablePointNum[1][i], 1, 2, 0);
	}
	/*individualcCheckLegCross(p_LegGroundableCandidatePoint[1][1], &LegGroundablePointNum[1][1], 1, 2, 0);
	individualcCheckLegCross(p_LegGroundableCandidatePoint[1][2], &LegGroundablePointNum[1][2], 1, 2, 0);
	individualcCheckLegCross(p_LegGroundableCandidatePoint[1][3], &LegGroundablePointNum[1][3], 1, 2, 0);*/

	//2番脚　衝突可能性があるのは1番，3番脚
	for (int i = 0; i < DISCRETE_NUM; ++i) {
		individualcCheckLegCross(p_LegGroundableCandidatePoint[2][i], &LegGroundablePointNum[2][i], 2, 3, 1);
	}
	/*individualcCheckLegCross(p_LegGroundableCandidatePoint[2][1], &LegGroundablePointNum[2][1], 2, 3, 1);
	individualcCheckLegCross(p_LegGroundableCandidatePoint[2][2], &LegGroundablePointNum[2][2], 2, 3, 1);
	individualcCheckLegCross(p_LegGroundableCandidatePoint[2][3], &LegGroundablePointNum[2][3], 2, 3, 1);*/

	//3番脚　衝突可能性があるのは2番，4番脚
	for (int i = 0; i < DISCRETE_NUM; ++i) {
		individualcCheckLegCross(p_LegGroundableCandidatePoint[3][i], &LegGroundablePointNum[3][i], 3, 4, 2);
	}
	/*individualcCheckLegCross(p_LegGroundableCandidatePoint[3][1], &LegGroundablePointNum[3][1], 3, 4, 2);
	individualcCheckLegCross(p_LegGroundableCandidatePoint[3][2], &LegGroundablePointNum[3][2], 3, 4, 2);
	individualcCheckLegCross(p_LegGroundableCandidatePoint[3][3], &LegGroundablePointNum[3][3], 3, 4, 2);*/

	//4番脚　衝突可能性があるのは3番，5番脚
	for (int i = 0; i < DISCRETE_NUM; ++i) {
		individualcCheckLegCross(p_LegGroundableCandidatePoint[4][i], &LegGroundablePointNum[4][i], 4, 5, 3);
	}
	/*individualcCheckLegCross(p_LegGroundableCandidatePoint[4][1], &LegGroundablePointNum[4][1], 4, 5, 3);
	individualcCheckLegCross(p_LegGroundableCandidatePoint[4][2], &LegGroundablePointNum[4][2], 4, 5, 3);
	individualcCheckLegCross(p_LegGroundableCandidatePoint[4][3], &LegGroundablePointNum[4][3], 4, 5, 3);*/

	//5番脚　衝突可能性があるのは4番，0番脚
	for (int i = 0; i < DISCRETE_NUM; ++i) {
		individualcCheckLegCross(p_LegGroundableCandidatePoint[5][i], &LegGroundablePointNum[5][i], 5, 0, 4);
	}
	/*individualcCheckLegCross(p_LegGroundableCandidatePoint[5][1], &LegGroundablePointNum[5][1], 5, 0, 4);
	individualcCheckLegCross(p_LegGroundableCandidatePoint[5][2], &LegGroundablePointNum[5][2], 5, 0, 4);
	individualcCheckLegCross(p_LegGroundableCandidatePoint[5][3], &LegGroundablePointNum[5][3], 5, 0, 4);*/
}

bool SearchPossibleLegPosition::individualcCheckLegCross(myvector::SVector* LegGroundableCandidatePoint, int* LegGroundablePointNum, int checkedLegNum, int checkLegNum1, int checkLegNum2) {
	for (int i = 0; i < *LegGroundablePointNum; i++) {

		if (VMag2(phantomX.getGlobalLegPos(checkLegNum1), LegGroundableCandidatePoint[i]) > LEGR) {	//脚が接近しすぎていないかどうか
			if (VMag2(phantomX.getGlobalLegPos(checkLegNum2), LegGroundableCandidatePoint[i]) > LEGR) {	//脚が接近しすぎていないかどうか
				if (!isCross(phantomX.getGlobalCoxaJointPos(checkedLegNum), LegGroundableCandidatePoint[i], phantomX.getGlobalCoxaJointPos(checkLegNum1), phantomX.getGlobalLegPos(checkLegNum1))) {
					if (!isCross(phantomX.getGlobalCoxaJointPos(checkedLegNum), LegGroundableCandidatePoint[i], phantomX.getGlobalCoxaJointPos(checkLegNum2), phantomX.getGlobalLegPos(checkLegNum2))) {

						LegGroundableCandidatePoint[0] = LegGroundableCandidatePoint[i];//[0]が脚接地候補
						return 0;
					}
				}
			}
		}
	}
	*LegGroundablePointNum = 0;
	return 1;	//干渉しない脚位置が見当たらない
}

//線分が交差しているか判定
bool SearchPossibleLegPosition::isCross(myvector::SVector s1, myvector::SVector e1, myvector::SVector s2, myvector::SVector e2){
	myvector::SVector xy_s1, xy_s2, xy_e1, xy_e2;

	xy_s1 = s1;
	xy_s2 = s2;
	xy_e1 = e1;
	xy_e2 = e2;

	xy_s1.z = 0;
	xy_s2.z = 0;
	xy_e1.z = 0;
	xy_e2.z = 0;

	myvector::SVector v,v1,v2;
	double t1,t2;
	v1 = myvector::subVec(xy_e1,xy_s1);
	v2 = myvector::subVec(xy_e2,xy_s2);
	v  = myvector::subVec(xy_s1,xy_s2);

	//2つのベクトルが平行
	if(myvector::VSquareSize(myvector::VCross(v1, v2)) < 0.01){
		return 0;	//2つのアークの始点，終点のどれも一致していない
	}

	t1 = - myvector::VCross(v, v2).z / myvector::VCross(v1, v2).z;
	t2 = - myvector::VCross(v, v1).z / myvector::VCross(v1, v2).z;


	if (ContactJudgment(t1, t2)) {		//線分同士が交差しているならば
		if(VectorEqual(xy_s1, xy_s2) || VectorEqual(xy_s1, xy_e2) || VectorEqual(xy_s2, xy_e1) || VectorEqual(xy_e1, xy_e2)){
			return 1;
		}
		return 1;
	}
	return 0;
}

bool SearchPossibleLegPosition::ContactJudgment(double t1, double t2){
	return (-HITDETECTIONMARGIN < t1 && t1 < 1.0 + HITDETECTIONMARGIN && -HITDETECTIONMARGIN < t2 && t2 < 1.0 + HITDETECTIONMARGIN) ;
}

//1 2つのベクトルの全ての成分が等しい
//0 2つのベクトルが異なる
bool SearchPossibleLegPosition::VectorEqual(myvector::SVector v1, myvector::SVector v2){
	return (abs(v1.x - v2.x) + abs(v1.y - v2.y) + abs(v1.z - v2.z)) < 0.01;
}

//void SearchPossibleLegPosition::LegGroundableCandidatePointsort(myvector::VECTOR p_LegGroundableCandidatePoint[6][4][1000], int LegGroundablePointNum[6][4]){
void SearchPossibleLegPosition::LegGroundableCandidatePointsort(myvector::SVector p_LegGroundableCandidatePoint[6][DISCRETE_NUM][1000], int LegGroundablePointNum[6][DISCRETE_NUM]){
	//double sort_p_LegGroundableCandidatePoint[6][4][1000];
	double sort_p_LegGroundableCandidatePoint[6][DISCRETE_NUM][1000];

	for(int i = 0; i < HexapodConst::LEG_NUM; i++){
		for(int ii = 1; ii < DISCRETE_NUM; ii++){
			for (int iii = 0; iii < LegGroundablePointNum[i][ii]; iii++){
				sort_p_LegGroundableCandidatePoint[i][ii][iii] = myvector::VDot(p_LegGroundableCandidatePoint[i][ii][iii], phantomX.getTargetDirection() );//脚位置と接地可能点の進行方向に対する距離
			}
		}
	}

	for(int i = 0; i < HexapodConst::LEG_NUM; i++){
		for(int ii = 0; ii < DISCRETE_NUM; ii++){
			for(int j = 0; j < LegGroundablePointNum[i][ii]; j++){
				for(int jj = LegGroundablePointNum[i][ii] - 1; jj > j;jj--){
					if(sort_p_LegGroundableCandidatePoint[i][ii][jj] > sort_p_LegGroundableCandidatePoint[i][ii][jj - 1]){//進行方向への距離が大きいほどjjの小さい位置へ行く,バブルソート
						double ft = sort_p_LegGroundableCandidatePoint[i][ii][jj];
						myvector::SVector Vt = p_LegGroundableCandidatePoint[i][ii][jj];

						sort_p_LegGroundableCandidatePoint[i][ii][jj] = sort_p_LegGroundableCandidatePoint[i][ii][jj - 1];
						p_LegGroundableCandidatePoint[i][ii][jj] = p_LegGroundableCandidatePoint[i][ii][jj - 1];

						sort_p_LegGroundableCandidatePoint[i][ii][jj - 1] = ft;
						p_LegGroundableCandidatePoint[i][ii][jj - 1] = Vt;
					}
				}
			}
		}
	}
}

myvector::SVector SearchPossibleLegPosition::VCangeBodyToLeg(myvector::SVector& Vin){//グローバル座標→ロボット脚座標(hexapodクラス)
	return myvector::VRot(myvector::VGet(Vin.x, -Vin.y, -Vin.z), phantomX.getGlobalMyDirectionthP(), phantomX.getGlobalMyDirectionthR(), -phantomX.getGlobalMyDirectionthY() - Define::MY_PI/2.0);  // - 3.14/2はどうにかならないかな
}


//指定した点PがベクトルSMとベクトルMEの張る面に対して表にあるか裏にあるか判定SM×MEのベクトルの指す方向が表
bool SearchPossibleLegPosition::Check_front_and_back(const myvector::SVector& S, const myvector::SVector& M, const myvector::SVector& E, const myvector::SVector& P){
	myvector::SVector SM, ME, MP;
	//ローカルベクトルに変換
	SM = myvector::subVec(M, S);
	ME = myvector::subVec(E, M);
	MP = myvector::subVec(P, M);
	if( myvector::VDot( myvector::VCross( SM, ME),MP) >= 0) return 1;//表にある。
	return 0;//裏にある。
}

//三角形ポリゴンと線分の衝突判定 接する場合衝突としない
bool SearchPossibleLegPosition::CollisionDetection_PL3(const myvector::SVector &In1, const myvector::SVector &In2, const myvector::SVector &In3, const myvector::SVector &LineStart, const myvector::SVector &LineEnd)
{	
	//ポリゴン3点,線始点,終点
	myvector::SVector Vn;		//ポリゴンの法線ベクトル
	myvector::SVector Vp;		//平面と線分の交差する点
	double lengS, lengE;		//平面と線分の2点の

	Vn = myvector::VCross(myvector::subVec(In2, In1), myvector::subVec(In3,In2));

	//線分と，ポリゴンが含まれる平面が交わるかどうか判断 接触は交わる判定しない
	if (myvector::VDot(Vn, myvector::subVec(In1, LineStart)) * myvector::VDot(Vn, myvector::subVec(In1, LineEnd)) >= 0) { return false; }//0なら衝突してない。		

	//平面と交差していなければ必ずポリゴンと干渉していない。
	//しかし、交差していても、交差している点がポリゴンの内か外かで干渉しているかわかれる。
	//交差する点の位置推定
	lengS = abs(myvector::VDot(Vn, myvector::subVec(LineStart, In1)) / myvector::VMag(Vn));
	lengE = abs(myvector::VDot(Vn, myvector::subVec(LineEnd, In1)) / myvector::VMag(Vn));
	Vp = myvector::VScale(myvector::subVec(LineEnd, LineStart), (lengS / (lengS + lengE)));		//LineStartからVpのベクトル　(lengS + lengE)は0にならない
	Vp = myvector::addVec(LineStart, Vp);														//グローバル座標でのVp

	//Vpがポリゴンの領域内にあるかどうか
	if (myvector::VDot(myvector::VCross(myvector::subVec(In2, In1), myvector::subVec(Vp, In2)), Vn) <= 0) { return false; }//ポリゴンの外だから干渉していない
	if (myvector::VDot(myvector::VCross(myvector::subVec(In3, In2), myvector::subVec(Vp, In3)), Vn) <= 0) { return false; }
	if (myvector::VDot(myvector::VCross(myvector::subVec(In1, In3), myvector::subVec(Vp, In1)), Vn) <= 0) { return false; }

	return true;//平面と交差かつ、交差点がポリゴン内だから干渉してる。
}


//旋回時の脚位置の候補を求める
int SearchPossibleLegPosition::PossibleLegPoint_Rotation() 
{
	myvector::SVector Rotation_Center = phantomX.getRotaionCenter();
	myvector::SVector targetD = phantomX.getTargetRotation();
	myvector::SVector com = phantomX.getGlobalMyPosition();

	myvector::SVector G_CoxaJointPosi[HexapodConst::LEG_NUM];
	myvector::SVector leg2[HexapodConst::LEG_NUM]; //基準位置のグローバル座標
	myvector::SVector diff_l2[HexapodConst::LEG_NUM];	//回転中心からleg2までのベクトル
	double Leg_th[HexapodConst::LEG_NUM];//回転中心からleg2までのベクトルがx軸となす角


	//4つの変数の初期化
	for (int i = 0; i < HexapodConst::LEG_NUM; ++i) 
	{
		//-pi～pi
		G_CoxaJointPosi[i] = phantomX.getGlobalCoxaJointPos(i);
		leg2[i] = phantomX.getGlobalLeg2Pos(i);

		//sx[i] = leg2[i].x - Rotation_Center.x;
		//sy[i] = leg2[i].y - Rotation_Center.y;
		diff_l2[i] = myvector::subVec(leg2[i], Rotation_Center);
		//Leg_th[i] = atan2(sy[i], sx[i]);
		Leg_th[i] = atan2(diff_l2[i].y, diff_l2[i].x);
		//global_coxa_joint_posi
	}
	//メンバ変数　LegGroundablePointNum[i][j]の初期化
	for (int i = 0; i < HexapodConst::LEG_NUM; ++i) 
	{
		for (int j = 0; j < DISCRETE_NUM; ++j) 
		{
			LegGroundablePointNum[i][j] = 0;			
		}
	}

	//可動範囲内の脚接地候補点を、それぞれの脚のそれぞれの離散化位置ごとに格納
	myvector::SVector point1, point2;//i番目の脚の可動範囲が内接する正方形の左下と右上の角の座標
	int x1, x2, y1, y2;	//分割したmapエリアのブロック番号
	myvector::SVector leg1_candidate;//coxaから脚接地可能点へのベクトル（グローバル）
	myvector::SVector diff_point;	//回転中心から候補点までの差分ベクトル
	double th;//回転中心から候補点までの差分ベクトルのx軸とのなす角
	double dth;//回転中心から候補点までベクトルがx軸となす角　と　回転中心からleg2までのベクトルがx軸となす角　の　差
	myvector::SVector diff;//leg2と脚接地候補点の差分ベクトル
	double sort_p_LegGroundableCandidatePoint[HexapodConst::LEG_NUM][DISCRETE_NUM][1000];

	for (int i = 0; i < HexapodConst::LEG_NUM; ++i) 
	{
		if (v[i]) { continue; }
		phantomX.calculateRangeOfMovement(i, point1, point2);
		AreaDivide(point1, point2, x1, x2, y1, y2);//ここのマップの大きさはもっと範囲狭くして小さくしてもいい。

		for (int ix = x1; ix <= x2; ++ix) 
		{	//x1からx2までのブロック
			for (int iy = y1; iy <= y2; ++iy) 
			{	//y1からy2までのブロック
				for (int n = 0; n < spointDataNum[ix][iy]; ++n) 
				{//あるブロックに存在する脚設置可能点の数
					leg1_candidate = myvector::subVec( (*sMapData)[ix][iy][n], G_CoxaJointPosi[i] );//マップの座標はグローバル
					//if (phantomX.isLegWithinRange(i, VCangeBodyToLeg(leg1_candidate))) {	//可動範囲内にあるかどうか
					//これで可動範囲内の点だけ取れると思うが、逆にいまの重心高さを変えれば届くような点はここでは除外される。くやしぃ～、（だが、それでいいっ！！）おそらく
					if (phantomX.check_touchdown_point2(i, VCangeBodyToLeg(leg1_candidate),com.z - (*sMapData)[ix][iy][n].z)) {	//VCangeは謎、変更20200612 VCangeはhexapod.hの座標系に合わせてる
						diff_point = myvector::subVec((*sMapData)[ix][iy][n], Rotation_Center);
						th = atan2(diff_point.y, diff_point.x);
						dth = (th - Leg_th[i]) * targetD.z;

						//外積が+のとき左回り,-のとき右回り(回転中心から現在の脚へのベクトル×回転中心からマップへのベクトル)
						//外積が+で角度の差が-,外積が-で差が+はatan2の区間をまたいでいる
						diff = myvector::subVec((*sMapData)[ix][iy][n], leg2[i]);	
						//離散化位置4
						if (myvector::VMag(diff) < 50) 
						{//現在の脚先と距離が近い場合は同じ脚接地点（計算誤差）	
								if (kaisou[i] == 4) 
								{
									p_LegGroundableCandidatePoint[i][3][LegGroundablePointNum[i][3]] = (*sMapData)[ix][iy][n];	//離散化位置4 - 1で配列のindexは3
									sort_p_LegGroundableCandidatePoint[i][3][LegGroundablePointNum[i][3]] = 50.0 - myvector::VMag(diff);//現在の位置に近いほど値が大きい
									LegGroundablePointNum[i][3]++;

									if (LegGroundablePointNum[i][3] > LegGroundableCandidatePoint_MAX - 1) { return -1; }
								}
							}
						//離散化位置567
						else if ( (diff_l2[i].x * diff_point.y - diff_l2[i].y * diff_point.x) * targetD.z > 0.01)
						{//目標方向 //角度差の±でもいいけど20200612
							if (dth < -Define::MY_PI) 
							{//差の修正
								dth = dth + 2.0*Define::MY_PI;
							}
							else if (dth > Define::MY_PI) 
							{
								dth = dth - 2.0*Define::MY_PI;
							}

							if (diff.z > 5.0) 
							{	//候補点がleg2より高いなら 何ミリ高いかはロボットとカメラの精度によるけどどうやって決めよう　とりあえず10mmで
								if (kaisou[i] == 7) 
								{
									p_LegGroundableCandidatePoint[i][6][LegGroundablePointNum[i][6]] = (*sMapData)[ix][iy][n];	//離散化位置7  -1で配列のindexは6
									sort_p_LegGroundableCandidatePoint[i][6][LegGroundablePointNum[i][6]] = dth;
									LegGroundablePointNum[i][6]++;
									if (LegGroundablePointNum[i][6] > LegGroundableCandidatePoint_MAX - 1) { return -1; }
								}
							}
							else if (diff.z < -5.0) 
							{	//候補点がleg2より低い
								if (kaisou[i] == 5) 
								{
									p_LegGroundableCandidatePoint[i][4][LegGroundablePointNum[i][4]] = (*sMapData)[ix][iy][n];	//離散化位置5  -1で配列のindexは4
									sort_p_LegGroundableCandidatePoint[i][4][LegGroundablePointNum[i][4]] = dth;
									LegGroundablePointNum[i][4]++;
									if (LegGroundablePointNum[i][4] > LegGroundableCandidatePoint_MAX - 1) { return -1; }
								}
							}
							else 
							{	//±10mmなら同じ高さとして扱う　っていう風にしてる190609
								if (kaisou[i] == 6) 
								{
									p_LegGroundableCandidatePoint[i][5][LegGroundablePointNum[i][5]] = (*sMapData)[ix][iy][n];	//離散化位置6  -1で配列のindexは5
									sort_p_LegGroundableCandidatePoint[i][5][LegGroundablePointNum[i][5]] = dth;
									LegGroundablePointNum[i][5]++;
									if (LegGroundablePointNum[i][5] > LegGroundableCandidatePoint_MAX - 1) { return -1; }
								}
							}

							/*//if(i >= 4 && targetD.z == 1 && myvector::V2Mag2(Rotation_Center,(*sMapData)[ix][iy][n]) > (phantomX.getTurningRadius() - 150.0) && phantomX.getTurningRadius() > 200)continue;
							p_LegGroundableCandidatePoint[i][3][LegGroundablePointNum[i][3]] = (*sMapData)[ix][iy][n];
							sort_p_LegGroundableCandidatePoint[i][3][LegGroundablePointNum[i][3]] = dth;
							LegGroundablePointNum[i][3]++;
							//左前脚が旋回円より内側にある(左回り）右回りでは右前脚(i = 0)を選択しやすくする→脚接地可能点が1つの場合意味がない
							//if(i >= 4 && targetD.z == 1 && myvector::V2Mag2(Rotation_Center,(*sMapData)[ix][iy][n]) > (phantomX.getTurningRadius() - 100.0) && phantomX.getTurningRadius() > 200) {sort_p_LegGroundableCandidatePoint[i][3][LegGroundablePointNum[i][3]] = 100.0 - phantomX.getTurningRadius();LegGroundablePointNum[i][3]--;}//値は適当
							//else if(i < 3 && targetD.z == -1 && myvector::V2Mag2(Rotation_Center,(*sMapData)[ix][iy][n]) < (phantomX.getTurningRadius() - 100.0) && phantomX.getTurningRadius() > 200)  sort_p_LegGroundableCandidatePoint[i][3][LegGroundablePointNum[i][3]] += phantomX.getTurningRadius() * 2500.0;
							if (LegGroundablePointNum[i][3] > LegGroundableCandidatePoint_MAX - 1)return -1;*/

						//}else if ((sx[i] * ey - sy[i] * ex) * targetD.z < -0.01) {//目標と逆方向
						}

						//離散化位置123
						else if ( (diff_l2[i].x * diff_point.y - diff_l2[i].y * diff_point.x) * targetD.z < -0.01) 
						{//目標と逆方向
							if (dth < -Define::MY_PI) 
							{//差の修正
								dth = dth + 2.0*Define::MY_PI;
							}
							else if (dth > Define::MY_PI) 
							{
								dth = dth - 2.0*Define::MY_PI;
							}

							if (diff.z > 5.0) 
							{	//候補点がleg2より高いなら 何ミリ高いかはロボットとカメラの精度によるけどどうやって決めよう　とりあえず10mmで
								if (kaisou[i] == 3) 
								{
									p_LegGroundableCandidatePoint[i][2][LegGroundablePointNum[i][2]] = (*sMapData)[ix][iy][n];	//離散化位置3  -1で配列のindexは2
									sort_p_LegGroundableCandidatePoint[i][2][LegGroundablePointNum[i][2]] = dth;
									LegGroundablePointNum[i][2]++;
									if (LegGroundablePointNum[i][2] > LegGroundableCandidatePoint_MAX - 1) { return -1; }
								}
							}
							else if (diff.z < -5.0) 
							{	//候補点がleg2より低い
								if (kaisou[i] == 1) 
								{
									p_LegGroundableCandidatePoint[i][0][LegGroundablePointNum[i][0]] = (*sMapData)[ix][iy][n];	//離散化位置1  -1で配列のindexは0
									sort_p_LegGroundableCandidatePoint[i][0][LegGroundablePointNum[i][0]] = dth;
									LegGroundablePointNum[i][0]++;
									if (LegGroundablePointNum[i][0] > LegGroundableCandidatePoint_MAX - 1) { return -1; }
								}
							}
							else 
							{	//±10mmなら同じ高さとして扱う　っていう風にしてる190609
								if (kaisou[i] == 2) 
								{
									p_LegGroundableCandidatePoint[i][1][LegGroundablePointNum[i][1]] = (*sMapData)[ix][iy][n];	//離散化位置2  -1で配列のindexは1
									sort_p_LegGroundableCandidatePoint[i][1][LegGroundablePointNum[i][1]] = dth;
									LegGroundablePointNum[i][1]++;
									if (LegGroundablePointNum[i][1] > LegGroundableCandidatePoint_MAX - 1) { return -1; }
								}
							}

							/*//if(i >= 4 && targetD.z == 1 && myvector::V2Mag2(Rotation_Center,(*sMapData)[ix][iy][n]) > (phantomX.getTurningRadius() - 150.0) && phantomX.getTurningRadius() > 200)continue;
							p_LegGroundableCandidatePoint[i][1][LegGroundablePointNum[i][1]] = (*sMapData)[ix][iy][n];
							sort_p_LegGroundableCandidatePoint[i][1][LegGroundablePointNum[i][1]] = dth;
							LegGroundablePointNum[i][1]++;
							if (LegGroundablePointNum[i][1] > LegGroundableCandidatePoint_MAX - 1)return -1;*/

						}
						//離散化位置4
						else {//角度差は小さいが遠い場所
								if (kaisou[i] == 4) {
									if (dth < -Define::MY_PI) {
										dth = dth + 2.0*Define::MY_PI;
									}
									else if (dth > Define::MY_PI) {
										dth = dth - 2.0*Define::MY_PI;
									}
									p_LegGroundableCandidatePoint[i][3][LegGroundablePointNum[i][3]] = (*sMapData)[ix][iy][n];	//この場合も離散化位置4扱いで どうしよ190609
									sort_p_LegGroundableCandidatePoint[i][3][LegGroundablePointNum[i][3]] = dth;
									LegGroundablePointNum[i][3]++;
									if (LegGroundablePointNum[i][3] > LegGroundableCandidatePoint_MAX - 1)return -1;
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


//ソートというか大きいやつ１つわかればいい
void SearchPossibleLegPosition::LegGroundableCandidatePointsort_Th(myvector::SVector p_LegGroundableCandidatePoint[HexapodConst::LEG_NUM][DISCRETE_NUM][1000], double sort_p_LGCP[HexapodConst::LEG_NUM][DISCRETE_NUM][1000], int LegGroundablePointNum[HexapodConst::LEG_NUM][DISCRETE_NUM]) 
{
	//sort_～～は回転角度,p_leg～～は脚接地可能点
	for (int i = 0; i < HexapodConst::LEG_NUM; i++) 
	{
		for (int ii = 0; ii < DISCRETE_NUM; ii++) 
{
			for (int j = 0; j < LegGroundablePointNum[i][ii]; j++) 
			{
				for (int jj = LegGroundablePointNum[i][ii] - 1; jj > j; jj--) 
				{
					//進行方向への距離が大きいほどjjの小さい位置へ行く,バブルソート
					if (sort_p_LGCP[i][ii][jj] > sort_p_LGCP[i][ii][jj - 1]) 
					{
						double ft = sort_p_LGCP[i][ii][jj];
						myvector::SVector Vt = p_LegGroundableCandidatePoint[i][ii][jj];

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

//旋回時の脚位置の候補を求める一脚版 //一脚の場合脚接地点選択がいらない。
int SearchPossibleLegPosition::PossibleLegPoint_Rotation(int legnum) 
{
	myvector::SVector Rotation_Center = phantomX.getRotaionCenter();
	myvector::SVector G_CoxaJointPosi, leg2;
	myvector::SVector com = phantomX.getGlobalMyPosition();
	double Leg_th;//回転中心からの角度
	myvector::SVector diff_l2;	//回転中心からleg2までのベクトル
	double th, dth;
	myvector::SVector targetD = phantomX.getTargetRotation();

	//回転中心から現在の脚接地点の角度計算//-pi～pi
	leg2 = phantomX.getGlobalLeg2Pos(legnum);//グローバル
	diff_l2 = myvector::subVec(leg2, Rotation_Center);
	Leg_th = atan2(diff_l2.y, diff_l2.x);
	G_CoxaJointPosi = phantomX.getGlobalCoxaJointPos(legnum);
	
	std::string wait;

	for (int i = 0; i < DISCRETE_NUM; i++) { LegGroundablePointNum[legnum][i] = 0; }		//LegGroundablePointNum[ii][i]の初期化
																								//可動範囲内の脚接地候補点を格納

	myvector::SVector point1, point2;
	int x1, x2, y1, y2;

	int ok = 0;
	phantomX.calculateRangeOfMovement(legnum, point1, point2);	//i番目の脚の可動範囲が内接する正方形の左下と右上の角の座標
	AreaDivide(point1, point2, x1, x2, y1, y2);	//分割したmapエリアのブロック番号

	for (int ix = x1; ix <= x2; ++ix) 
	{	//x1からx2までのブロック
		if (ok == 1111111) { break; }

		for (int iy = y1; iy <= y2; ++iy) 
		{	//y1からy2までのブロック
			if (ok == 1111111) { break; }

			for (int n = 0; n < spointDataNum[ix][iy]; ++n) 
			{
				if (ok == 1111111) { break; }


				myvector::SVector ll1 = myvector::subVec((*sMapData)[ix][iy][n], G_CoxaJointPosi);

				//可動範囲内にあるかどうか　後々は第三引数を、接地面座標系で見た重心と脚先高さ（脚接地候補点高さ）の差にしなければならない。
				if (phantomX.check_touchdown_point2(legnum, VCangeBodyToLeg(ll1),com.z - (*sMapData)[ix][iy][n].z)) 
				{	
					//回転中心から脚接候補点の角度計算
					//ex = (*sMapData)[ix][iy][n].x - Rotation_Center.x;
					//ey = (*sMapData)[ix][iy][n].y - Rotation_Center.y;
					//th = atan2(ey, ex);
					myvector::SVector diff_point = myvector::subVec((*sMapData)[ix][iy][n], Rotation_Center);	//回転中心から候補点までの差分ベクトル
					th = atan2(diff_point.y, diff_point.x);
					dth = (th - Leg_th) * targetD.z;

					//外積が+のとき左回り,-のとき右回り(回転中心から現在の脚へのベクトル×回転中心からマップへのベクトル)
					//外積が+で角度の差が-,外積が-で差が+はatan2の区間をまたいでいる
					//if (sx[i] - ex<50 && sx[i] - ex>-50 && sy[i] - ey<50 && sy[i] - ey>-50) {//現在の脚先と距離が近い場合は同じ脚接地点（計算誤差）
					myvector::SVector diff = myvector::subVec((*sMapData)[ix][iy][n], leg2);	//leg2と脚接地候補点の差分ベクトル
					if (myvector::VMag(diff) < 50) {//現在の脚先と距離が近い場合は同じ脚接地点（計算誤差）	
						//if(i >= 4 && targetD.z == 1 && myvector::V2Mag2(Rotation_Center,(*sMapData)[ix][iy][n]) > (phantomX.getTurningRadius() - 150.0) && phantomX.getTurningRadius() > 200)continue;
						//p_LegGroundableCandidatePoint[legnum][3][LegGroundablePointNum[legnum][3]] = (*sMapData)[ix][iy][n];	//離散化位置4 - 1で配列のindexは3
						//sort_p_LegGroundableCandidatePoint[i][3][LegGroundablePointNum[i][3]] = 50.0 - myvector::V2Mag2(leg2[i], (*sMapData)[ix][iy][n]);//現在の位置に近いほど値が大きい
						//sort_p_LegGroundableCandidatePoint[3][LegGroundablePointNum[legnum][3]] = 50.0 - myvector::VMag(diff);//現在の位置に近いほど値が大きい
						if (LegGroundablePointNum[legnum][3] == 0) 
						{
							LegGroundablePointNum[legnum][3]++;
							ok += 1;
						}

					//	if (LegGroundablePointNum[legnum][3] > LegGroundableCandidatePoint_MAX - 1)return -1;

						//}else if ((sx[i] * ey - sy[i] * ex) * targetD.z > 0.01) {//目標方向
					} 
					else if ((diff_l2.x * diff_point.y - diff_l2.y * diff_point.x) * targetD.z > 0.01) 
					{//目標方向
						//if (dth < -Define::MY_PI) {//差の修正
						//	dth = dth + 2.0*Define::MY_PI;
						//} else if (dth > Define::MY_PI) {
						//	dth = dth - 2.0*Define::MY_PI;
						//}

						if (diff.z > 5.0) 
						{	//候補点がleg2より高いなら 何ミリ高いかはロボットとカメラの精度によるけどどうやって決めよう　とりあえず10mmで
							//p_LegGroundableCandidatePoint[legnum][6][LegGroundablePointNum[legnum][6]] = (*sMapData)[ix][iy][n];	//離散化位置7  -1で配列のindexは6
							//sort_p_LegGroundableCandidatePoint[6][LegGroundablePointNum[legnum][6]] = dth;
							if (LegGroundablePointNum[legnum][6] == 0) 
							{
								LegGroundablePointNum[legnum][6]++;
								ok += 10;
							}
							//if (LegGroundablePointNum[legnum][6] > LegGroundableCandidatePoint_MAX - 1)return -1;

						} 
						else if (diff.z < -5.0) {	//候補点がleg2より低い
							//p_LegGroundableCandidatePoint[legnum][4][LegGroundablePointNum[legnum][4]] = (*sMapData)[ix][iy][n];	//離散化位置5  -1で配列のindexは4
							//sort_p_LegGroundableCandidatePoint[4][LegGroundablePointNum[legnum][4]] = dth;
							if (LegGroundablePointNum[legnum][4] == 0) {
								LegGroundablePointNum[legnum][4]++;
								ok += 100;
							}
							//if (LegGroundablePointNum[legnum][4] > LegGroundableCandidatePoint_MAX - 1)return -1;

						} 
						else {	//±5mmなら同じ高さとして扱う　っていう風にしてる190609
							//p_LegGroundableCandidatePoint[legnum][5][LegGroundablePointNum[legnum][5]] = (*sMapData)[ix][iy][n];	//離散化位置6  -1で配列のindexは5
							//sort_p_LegGroundableCandidatePoint[5][LegGroundablePointNum[legnum][5]] = dth;
							if (LegGroundablePointNum[legnum][5] == 0) {
								LegGroundablePointNum[legnum][5]++;
								ok += 1000;
							}
							//if (LegGroundablePointNum[legnum][5] > LegGroundableCandidatePoint_MAX - 1)return -1;
						}

						/*//if(i >= 4 && targetD.z == 1 && myvector::V2Mag2(Rotation_Center,(*sMapData)[ix][iy][n]) > (phantomX.getTurningRadius() - 150.0) && phantomX.getTurningRadius() > 200)continue;
						p_LegGroundableCandidatePoint[i][3][LegGroundablePointNum[i][3]] = (*sMapData)[ix][iy][n];
						sort_p_LegGroundableCandidatePoint[i][3][LegGroundablePointNum[i][3]] = dth;
						LegGroundablePointNum[i][3]++;
						//左前脚が旋回円より内側にある(左回り）右回りでは右前脚(i = 0)を選択しやすくする→脚接地可能点が1つの場合意味がない
						//if(i >= 4 && targetD.z == 1 && myvector::V2Mag2(Rotation_Center,(*sMapData)[ix][iy][n]) > (phantomX.getTurningRadius() - 100.0) && phantomX.getTurningRadius() > 200) {sort_p_LegGroundableCandidatePoint[i][3][LegGroundablePointNum[i][3]] = 100.0 - phantomX.getTurningRadius();LegGroundablePointNum[i][3]--;}//値は適当
						//else if(i < 3 && targetD.z == -1 && myvector::V2Mag2(Rotation_Center,(*sMapData)[ix][iy][n]) < (phantomX.getTurningRadius() - 100.0) && phantomX.getTurningRadius() > 200)  sort_p_LegGroundableCandidatePoint[i][3][LegGroundablePointNum[i][3]] += phantomX.getTurningRadius() * 2500.0;
						if (LegGroundablePointNum[i][3] > LegGroundableCandidatePoint_MAX - 1)return -1;*/

						//}else if ((sx[i] * ey - sy[i] * ex) * targetD.z < -0.01) {//目標と逆方向
					} 
					else if ((diff_l2.x * diff_point.y - diff_l2.y * diff_point.x) * targetD.z < -0.01) {//目標と逆方向
						//if (dth < -Define::MY_PI) {//差の修正
						//	dth = dth + 2.0*Define::MY_PI;
						//} else if (dth > Define::MY_PI) {
						//	dth = dth - 2.0*Define::MY_PI;
						//}

						if (diff.z > 5.0) {	//候補点がleg2より高いなら 何ミリ高いかはロボットとカメラの精度によるけどどうやって決めよう　とりあえず10mmで
							//p_LegGroundableCandidatePoint[legnum][2][LegGroundablePointNum[legnum][2]] = (*sMapData)[ix][iy][n];	//離散化位置3  -1で配列のindexは2
							//sort_p_LegGroundableCandidatePoint[2][LegGroundablePointNum[legnum][2]] = dth;
							if (LegGroundablePointNum[legnum][2] == 0) {
								LegGroundablePointNum[legnum][2]++;
								ok += 10000;
							}
							//if (LegGroundablePointNum[legnum][2] > LegGroundableCandidatePoint_MAX - 1)return -1;

						} 
						else if (diff.z < -5.0) {	//候補点がleg2より低い
							//p_LegGroundableCandidatePoint[legnum][0][LegGroundablePointNum[legnum][0]] = (*sMapData)[ix][iy][n];	//離散化位置1  -1で配列のindexは0
							//sort_p_LegGroundableCandidatePoint[0][LegGroundablePointNum[legnum][0]] = dth;
							if (LegGroundablePointNum[legnum][0] == 0) {
								LegGroundablePointNum[legnum][0]++;
								ok += 100000;
							}
							//if (LegGroundablePointNum[legnum][0] > LegGroundableCandidatePoint_MAX - 1)return -1;

						} 
						else {	//±5mmなら同じ高さとして扱う　っていう風にしてる190609
						//	p_LegGroundableCandidatePoint[legnum][1][LegGroundablePointNum[legnum][1]] = (*sMapData)[ix][iy][n];	//離散化位置2  -1で配列のindexは1
							//sort_p_LegGroundableCandidatePoint[1][LegGroundablePointNum[legnum][1]] = dth;
							if (LegGroundablePointNum[legnum][1] == 0) {
								LegGroundablePointNum[legnum][1]++;
								ok += 1000000;
							}
							//if (LegGroundablePointNum[legnum][1] > LegGroundableCandidatePoint_MAX - 1)return -1;
						}

						/*//if(i >= 4 && targetD.z == 1 && myvector::V2Mag2(Rotation_Center,(*sMapData)[ix][iy][n]) > (phantomX.getTurningRadius() - 150.0) && phantomX.getTurningRadius() > 200)continue;
						p_LegGroundableCandidatePoint[i][1][LegGroundablePointNum[i][1]] = (*sMapData)[ix][iy][n];
						sort_p_LegGroundableCandidatePoint[i][1][LegGroundablePointNum[i][1]] = dth;
						LegGroundablePointNum[i][1]++;
						if (LegGroundablePointNum[i][1] > LegGroundableCandidatePoint_MAX - 1)return -1;*/

					} 
					else {//角度差は小さいが遠い場所
						 //if(i >= 4 && targetD.z == 1 && myvector::V2Mag2(Rotation_Center,(*sMapData)[ix][iy][n]) > (phantomX.getTurningRadius() - 150.0) && phantomX.getTurningRadius() > 200)continue;
						//if (dth < -Define::MY_PI) {
						//	dth = dth + 2.0*Define::MY_PI;
						//} else if (dth > Define::MY_PI) {
						//	dth = dth - 2.0*Define::MY_PI;
						//}
						//p_LegGroundableCandidatePoint[legnum][3][LegGroundablePointNum[legnum][3]] = (*sMapData)[ix][iy][n];	//この場合も離散化位置4扱いで どうしよ190609
						//sort_p_LegGroundableCandidatePoint[3][LegGroundablePointNum[legnum][3]] = dth;
							if (LegGroundablePointNum[legnum][3] == 0) {
								LegGroundablePointNum[legnum][3]++;
								ok += 1;
							}
						//if (LegGroundablePointNum[legnum][3] > LegGroundableCandidatePoint_MAX - 1)return -1;
					}
				}
			}
		}
	}

	//ここで脚接地可能点のソート
	//p_LegGroundableCandidatePoint[][]の中のLegGroundablePointNum[][]個のデータをソートsort_p_LegGroundableCandidatePointの大きい順
	//LegGroundableCandidatePointsort_Th(legnum, p_LegGroundableCandidatePoint, sort_p_LegGroundableCandidatePoint, LegGroundablePointNum);

	//for (int iii = 0; iii < 6; iii++) {
	//	for (int i = 0; i < 7; i++) {
	//		if (legnum != iii) {
	//			if (LegGroundablePointNum[legnum][i] != 0 && myvector::VMag2(phantomX.getGlobalLegPos(iii), p_LegGroundableCandidatePoint[legnum][i][0]) < 20) {//脚同士が近すぎる
	//				//if( ii == 3 || iii == 3 )continue;
	//				std::cout << "LegGroundablePointNum[" << legnum << "][" << i << "]\n";
	//				std::cout << LegGroundablePointNum[legnum][i] << "\n";
	//				std::cout << "_SearchPossibleLegPosition.phantomX.getGlobalLegPos(" << iii << ")\n";
	//				myvector::VectorOutPut(phantomX.getGlobalLegPos(iii));
	//				std::cout << "_SearchPossibleLegPosition.p_LegGroundableCandidatePoint[" << legnum << "][" << i << "][0]\n";
	//				myvector::VectorOutPut(p_LegGroundableCandidatePoint[legnum][i][0]);
	//				LegGroundablePointNum[legnum][i] = 0;
	//				//std::string stop;
	//				//std::cin>>stop;
	//			}
	//		}
	//	}
	//}

	return 0;
}

//ソート1脚版
void SearchPossibleLegPosition::LegGroundableCandidatePointsort_Th(int legnum, myvector::SVector p_LegGroundableCandidatePoint[HexapodConst::LEG_NUM][DISCRETE_NUM][1000], double sort_p_LGCP[DISCRETE_NUM][1000], int LegGroundablePointNum[HexapodConst::LEG_NUM][DISCRETE_NUM]) 
{
	//sort_～～は回転角度,p_leg～～は脚接地可能点
	for (int ii = 0; ii < DISCRETE_NUM; ii++) 
	{
		for (int j = 0; j < LegGroundablePointNum[legnum][ii]; j++) 
		{
			for (int jj = LegGroundablePointNum[legnum][ii] - 1; jj > j; jj--) 
			{
				//進行方向への距離が大きいほどjjの小さい位置へ行く,バブルソート
				if (sort_p_LGCP[ii][jj] > sort_p_LGCP[ii][jj - 1]) 
				{
					double ft = sort_p_LGCP[ii][jj];
					myvector::SVector Vt = p_LegGroundableCandidatePoint[legnum][ii][jj];

					sort_p_LGCP[ii][jj] = sort_p_LGCP[ii][jj - 1];
					p_LegGroundableCandidatePoint[legnum][ii][jj] = p_LegGroundableCandidatePoint[legnum][ii][jj - 1];

					sort_p_LGCP[ii][jj - 1] = ft;
					p_LegGroundableCandidatePoint[legnum][ii][jj - 1] = Vt;
				}
			}
		}
	}

	//脚位置からほかの脚に当たらないように脚位置を選択
	checkLegCross(p_LegGroundableCandidatePoint, LegGroundablePointNum);
}

//将来的に必要になりそうな重心高さの変更量を返す。
int SearchPossibleLegPosition::Target_delta_comz() 
{
	//可動範囲内の脚接地候補点を、それぞれの脚のそれぞれの離散化位置ごとに格納
	myvector::SVector point1, point2;	//i番目の脚の可動範囲が内接する正方形の左下と右上の角の座標
	int x1, x2, y1, y2;					//分割したmapエリアのブロック番号
	myvector::SVector G_CoxaJointPosi;
	double max_map_height = -1000;
	double target_delta_comz = 0;
	int i = 0;
	G_CoxaJointPosi = phantomX.getGlobalCoxaJointPos(i);
	phantomX.calculateRangeOfMovement(i, point1, point2);//ROMが内接する長方形
	AreaDivide(point1, point2, x1, x2, y1, y2);//ここのマップの大きさはもっと範囲狭くして小さくしてもいい。

	//x1からx2までのブロック，y1からy2までのブロックを総当たりする
	for (int ix = x1; ix <= x2; ++ix) 
	{	
		for (int iy = y1; iy <= y2; ++iy) 
		{	
			for (int n = 0; n < spointDataNum[ix][iy]; ++n) 
			{
				//あるブロックに存在する脚設置可能点の最大高さ
				//ここで可動範囲内かどうか判定120<r<R[0]をしてから、下のif文
				//高さの判定はしない、外積取って、距離が定義域内かだけ判定する。
				myvector::SVector ll1 = myvector::subVec((*sMapData)[ix][iy][n], G_CoxaJointPosi);
				//if (phantomX.check_touchdown_point4(i, VCangeBodyToLeg(ll1))) {
				if (max_map_height < (*sMapData)[ix][iy][n].z) { max_map_height = (*sMapData)[ix][iy][n].z; }
				//}
			}
		}
	}

	i = 5;
	G_CoxaJointPosi = phantomX.getGlobalCoxaJointPos(i);
	phantomX.calculateRangeOfMovement(i, point1, point2);
	AreaDivide(point1, point2, x1, x2, y1, y2);	//ここのマップの大きさはもっと範囲狭くして小さくしてもいい。

	for (int ix = x1; ix <= x2; ++ix) 
	{	//x1からx2までのブロック
		for (int iy = y1; iy <= y2; ++iy) 
		{	//y1からy2までのブロック
			for (int n = 0; n < spointDataNum[ix][iy]; ++n) 
			{//あるブロックに存在する脚設置可能点の最大高さ
				myvector::SVector ll1 = myvector::subVec((*sMapData)[ix][iy][n], G_CoxaJointPosi);
				//if (phantomX.check_touchdown_point4(i, VCangeBodyToLeg(ll1))) {
					if (max_map_height < (*sMapData)[ix][iy][n].z) max_map_height = (*sMapData)[ix][iy][n].z;
				//}
			}
		}
	}

	//target_delta_comz5 = max_map_height;// -phantomX.getGlobalLeg2Pos(i).z;
	//if (target_delta_comz0 > target_delta_comz5) target_delta_comz = target_delta_comz0;
	//else target_delta_comz = target_delta_comz5;
	target_delta_comz = max_map_height + MIN_DELTAZ - phantomX.getGlobalMyPosition().z ;
	//if (target_delta_comz < 0 && target_delta_comz >= -20) target_delta_comz = 0;
	//if (0 > target_delta_comz - (int)target_delta_comz && target_delta_comz - (int)target_delta_comz >= -0.1) return (int)target_delta_comz + 1;
	return (int)target_delta_comz;
}

//胴体と地形（脚接地可能点）との衝突判定。高さ方向に設定したクリアランス分だけ余裕分がないときは、衝突とみなす。
int SearchPossibleLegPosition::Collision_judgment_with_the_body() 
{
	//1.胴体が内接する長方形の領域を計算
	//2.長方形の領域を含む、それよりも大きい範囲内にある脚接地可能点を抽出する。
	//3.各脚接地可能点に対して、長方形の領域内かどうかを調べる。
	//4.領域内の脚設置可能点の中の最大高さを求める。
	//5.その点と胴体が一定値以上離れているかを判定する。
	//6.接触する場合、その点から一定値はなれるように重心高さの変更量を決定する。

	//1.胴体が内接する長方形の領域を計算
	double minx = 1000000;
	double miny = 1000000;
	double maxx = -1000000;
	double maxy = -1000000;
	myvector::SVector Fem_buf;

	for (int i = 0; i < HexapodConst::LEG_NUM; ++i) 
	{
		Fem_buf = phantomX.getGlobalFemurJointPos(i);
		if (Fem_buf.x < minx) minx = Fem_buf.x;
		if (Fem_buf.y < miny) miny = Fem_buf.y;
		if (maxx < Fem_buf.x) maxx = Fem_buf.x;
		if (maxy < Fem_buf.y) maxy = Fem_buf.y;
	}
	int fem_margin = 10; //第2関節が若干ぶつかってそうだったから、実際より大きめに設定する。直線の場合x軸はなくてもいいが一応。
	minx -= fem_margin;
	miny -= fem_margin;
	maxx += fem_margin;
	maxy += fem_margin;
	myvector::SVector p[4];
	p[0] = myvector::VGet(minx, miny, 0);//coxaを含む胴体が内接する、xy平面に平行な長方形の左下の頂点座標（グローバル）
	p[1] = myvector::VGet(maxx, miny, 0);//coxaを含む胴体が内接する、xy平面に平行な長方形の右下の頂点座標（グローバル）
	p[2] = myvector::VGet(maxx, maxy, 0);//coxaを含む胴体が内接する、xy平面に平行な長方形の右上の頂点座標（グローバル）
	p[3] = myvector::VGet(minx, maxy, 0);//coxaを含む胴体が内接する、xy平面に平行な長方形の左上の頂点座標（グローバル）


	//myvector::VECTOR n = phantomX.getNormalVector();//胴体からなる平面の法線ベクトル
	//double pz[4];
	myvector::SVector com = phantomX.getGlobalMyPosition();
	////胴体が回転してるとき、各点の高さ情報も一応加えとく。
	//for (int i = 0; i < 4; ++i) {
	//	n=(a,b,c)とすると、平面の方程式は、a(Px-Gx)+b(Py-Gy)+c(Pz-Gz) = 0;今回はPzだけ未知数
	//	pz[i] = (n.x*(p[i].x - com.x) + n.y*(p[i].y - com.y))/n.z + com.z;//平面の方程式より導出https://mathtrain.jp/heimen
	//	p[i] = myvector::addVec(p[i], myvector::VGet(0, 0, pz[i]));
	//}

	//2.長方形の領域を含む、それよりも大きい範囲内にある脚接地可能点を抽出する。
	int x1, x2, y1, y2;
	AreaDivide(p[0], p[2], x1, x2, y1, y2);	//分割したmapエリアのブロック番号


	//3.各脚接地可能点に対して、長方形の領域内かどうかを調べる。
	//4.領域内の脚設置可能点の中の最大高さを求める。
	//回転させるときは平面と点の距離で導出する。（いまはしてない）
	double max_map_height = -100000;
	for (int ix = x1; ix <= x2; ++ix) 
	{	//x1からx2までのブロック
		for (int iy = y1; iy <= y2; ++iy) 
		{	//y1からy2までのブロック
			for (int n = 0; n < spointDataNum[ix][iy]; ++n) 
			{//あるブロックに存在する脚設置可能点の最大高さ
				if (minx <= (*sMapData)[ix][iy][n].x && (*sMapData)[ix][iy][n].x <= maxx) 
				{//xが範囲内か
					if (miny <= (*sMapData)[ix][iy][n].y && (*sMapData)[ix][iy][n].y <= maxy) 
					{//yが範囲内か
						if (max_map_height < (*sMapData)[ix][iy][n].z) { max_map_height = (*sMapData)[ix][iy][n].z; }
					}
				}
			}
		}
	}

	//5.その点と胴体が一定値以上離れているかを判定する。
	//6.離れていない場合、その点から一定値はなれるように重心高さの変更量を決定する。
	if (max_map_height + BODY_MARGIN <= com.z) 
	{
		return 0; //十分離れているときは、衝突していない判定 = 0を返す。
	}
	else
	{
		double add_comz = (max_map_height-com.z) + BODY_MARGIN;
		return (int)add_comz; //離れていない場合は、衝突しているので衝突しないようにする正の値を返す。
	}

}

//4角形ポリゴンと線分の衝突判定 干渉しているとき1,干渉していないとき0 (ただしポリゴンの4頂点同一平面上にある場合)	//ポリゴン3点,線始点,終点
bool SearchPossibleLegPosition::CollisionDetection_PL4(const myvector::SVector &In1, const myvector::SVector &In2, const myvector::SVector &In3, const myvector::SVector &In4, const myvector::SVector &LineStart, const myvector::SVector &LineEnd) 
{	
	myvector::SVector Vn;														//ポリゴンの法線ベクトル
	myvector::SVector Vp;														//平面と線分の交差する点
	double lengS, lengE;												//平面と線分の2点の
	Vn = myvector::VCross(myvector::subVec(In2, In1), myvector::subVec(In3, In2));

	//交差してない（接触まではセーフ）	//線分と，ポリゴンが含まれる平面が交わるかどうか判断 接触は交わる判定しない
	if (myvector::VDot(Vn, myvector::subVec(In1, LineStart)) * myvector::VDot(Vn, myvector::subVec(In1, LineEnd)) >= 0) { return 0; }

	//平面と交差していなければ必ずポリゴンと干渉していない。
	//しかし、交差していても、交差している点がポリゴンの内か外かで干渉しているかわかれる。
	//交差する点の位置推定
	lengS = abs(myvector::VDot(Vn, myvector::subVec(LineStart, In1)) / myvector::VMag(Vn));
	lengE = abs(myvector::VDot(Vn, myvector::subVec(LineEnd, In1)) / myvector::VMag(Vn));
	Vp = myvector::VScale(myvector::subVec(LineEnd, LineStart), (lengS / (lengS + lengE)));		//LineStartからVpのベクトル　(lengS + lengE)は0にならない
	Vp = myvector::addVec(LineStart, Vp);												//グローバル座標でのVp

	//Vpがポリゴンの領域内にあるかどうか
	if (myvector::VDot(myvector::VCross(myvector::subVec(In2, In1), myvector::subVec(Vp, In2)), Vn) <= 0) { return false; }//ポリゴンの外だから干渉していない
	if (myvector::VDot(myvector::VCross(myvector::subVec(In3, In2), myvector::subVec(Vp, In3)), Vn) <= 0) { return false; }
	if (myvector::VDot(myvector::VCross(myvector::subVec(In4, In3), myvector::subVec(Vp, In4)), Vn) <= 0) { return false; }
	if (myvector::VDot(myvector::VCross(myvector::subVec(In1, In4), myvector::subVec(Vp, In1)), Vn) <= 0) { return false; }
	//double length_VpP = myvector::V2Mag(myvector::subVec(Vp, LineEnd));

	return true;//平面と交差かつ、交差点がポリゴン内だから干渉してる。
}


//点と直線の最短距離、p:点、In1:直線の始点, V1:直線の方向ベクトル，h:点から直線に下ろした垂線の足, t:媒介変数
double SearchPossibleLegPosition::calcPointLineDist(const myvector::SVector &p, const myvector::SVector &In1, const myvector::SVector &V1, myvector::SVector &h, double &t) 
{
	double lenSqV1 = myvector::VDot(V1, V1);
	t = 0;
	if (lenSqV1 > 0) { t = myvector::VDot(V1, myvector::subVec(p, In1)) / lenSqV1; }

	h = myvector::addVec(In1, myvector::VScale(V1, t));
	return myvector::VMag(myvector::subVec(h, p));
}

//2つのベクトルのなす角が鋭角かどうかを返す、鋭角なら1、そうでなければ0     p1:点p2:ベクトルの始点p3:点
bool SearchPossibleLegPosition::isSharpAngle(const myvector::SVector &p1, const myvector::SVector &p2, const myvector::SVector &p3) 
{
	myvector::SVector v1 = myvector::subVec(p1, p2);
	myvector::SVector v2 = myvector::subVec(p3, p2);
	if (myvector::VDot(v1, v2) < 0) return false;
	return true;
}

//点と線分の最短距離　p:点 s:線分の始点 e:線分の終点 h:線分上の最短距離になる点 t:媒介変数
double SearchPossibleLegPosition::calcPointSegmentDist(const myvector::SVector &p, const myvector::SVector &s, const myvector::SVector &e, myvector::SVector &h, double t) 
{
	double len = calcPointLineDist(p, s, myvector::subVec(e, s), h, t);

	if (isSharpAngle(p, s, e) == false) 
	{
		h = s;
		return myvector::VMag(myvector::subVec(s, p));
	}
	else if (isSharpAngle(p, e, s) == false) 
	{
		h = e;
		return myvector::VMag(myvector::subVec(e, p));
	}

	return len;
}

//直線と直線の最短距離(ねじれの関係) In1:直線1の始点 LineStart:直線2の始点 V1,V2:直線1,2の方向ベクトル p1,p2::それぞれの直線上の最短距離となる点 t1,t2:それぞれの直線の媒介変数
double SearchPossibleLegPosition::calcLineLineDist(const myvector::SVector &In1, const myvector::SVector &LineStart, const myvector::SVector &V1, const myvector::SVector &V2, myvector::SVector &p1, myvector::SVector &p2, double &t1, double &t2) 
{
	//2直線がねじれの関係のとき
	double DV1V2 = myvector::VDot(V1, V2);
	double DV1V1 = myvector::VDot(V1, V1);
	double DV2V2 = myvector::VDot(V2, V2);
	myvector::SVector P21P11 = myvector::subVec(In1, LineStart);
	t1 = (DV1V2 * myvector::VDot(V2, P21P11) - DV2V2 * myvector::VDot(V1, P21P11)) / (DV1V1*DV2V2 - DV1V2 * DV1V2);
	p1 = myvector::addVec(In1, myvector::VScale(V1, t1));
	t2 = myvector::VDot(V2, myvector::subVec(p1, LineStart)) / DV2V2;
	p2 = myvector::addVec(LineStart, myvector::VScale(V2, t1));

	return myvector::VMag(myvector::subVec(p2, p1));
}

//線分と線分の最短距離を求める一応前二つが地形ポリゴンの座標、あと二つが膝と足先の座標で使う予定。返り値は2線分間の最短距離[mm]
double SearchPossibleLegPosition::calcSegmentSegmentDist(const myvector::SVector &In1, const myvector::SVector &In2, const myvector::SVector &LineStart, const myvector::SVector &LineEnd) {
	//ねじれの関係のとき、2直線間の最短距離を求めて、仮のt1,t2を求める。
	double t1;
	double t2;
	myvector::SVector p1;
	myvector::SVector p2;
	myvector::SVector V1 = myvector::subVec(In2, In1);//方向ベクトル
	myvector::SVector V2 = myvector::subVec(LineEnd, LineStart);//方向ベクトル
	double len = calcLineLineDist(In1, LineStart, V1, V2, p1, p2, t1, t2);
	if (0 <= t1 && t1 <= 1 && 0 <= t2 && t2 <= 1) return len; //最短距離となる点がどちらも線分内

	if (t1 < 0) t1 = 0;
	else if (1 < t1) t1 = 1;
	p1 = myvector::addVec(In1, myvector::VScale(V1, t1));
	len = calcPointSegmentDist(p1, LineStart, LineEnd, p2, t2);
	if (0 <= t2 && t2 <= 1) return len;//線分1上に点がなく、線分2上には点があるとき

	if (t2 < 0) t2 = 0;
	else if (1 < t2) t2 = 1;
	p2 = myvector::addVec(LineStart, myvector::VScale(V2, t2));
	len = calcPointSegmentDist(p2, In1, In2, p1, t1);
	if (0 <= t1 && t1 <= 1) return len;//線分2上に点がなく、線分1上には点があるとき

	if (t1 < 0) t1 = 0;
	else if (1 < t1) t1 = 1;
	p1 = myvector::addVec(In1, myvector::VScale(V1, t1));
	return myvector::VMag(myvector::subVec(p2, p1));//どちらの線分上にも点がないとき
}

//すねと地形の干渉チェック 基本的に重心平行移動時用 hato 実機実験用の簡易的な奴
bool SearchPossibleLegPosition::Collision_judgment_with_shin() 
{
	//(1)線分がポリゴンを貫通しているかチェック 貫通していたらreturn1
	//(2)ポリゴンを貫通していないが、脚の太さ的にポリゴンと干渉しているかチェック、干渉していたらreturn1
	//(1),(2)どちらのフラグも立たなければ、脛と地形は干渉していない。
	myvector::SVector G_tibiaposi[HexapodConst::LEG_NUM];//膝の座標
	myvector::SVector G_legposi[HexapodConst::LEG_NUM];//脚先の座標
	myvector::SVector PLvertex1[4];//四角形ポリゴン1
	myvector::SVector PLvertex2[4];//四角形ポリゴン2
	myvector::SVector PLvertex3[4];//四角形ポリゴン3

	//初期化
	for (int i = 0; i < HexapodConst::LEG_NUM; ++i) 
	{
		G_tibiaposi[i] = phantomX.getGlobalTibiaJointPos(i);
		G_legposi[i] = phantomX.getGlobalLegPos(i);
	}
	for (int i = 0; i < 4; ++i) 
	{
		PLvertex1[i] = p_mapData3D[mapData3D_MAX - 40 + i];
		PLvertex2[i] = p_mapData3D[mapData3D_MAX - 36 + i];
		PLvertex3[i] = p_mapData3D[mapData3D_MAX - 32 + i];
	}


	//(1)各脚ごとに脛がポリゴンを貫通していないかチェック、貫通していたら1を返す。（）
	for (int i = 0; i < HexapodConst::LEG_NUM; ++i) 
	{
#ifdef COLLISION_SHIN1
		if (CollisionDetection_PL4(PLvertex1[0], PLvertex1[1], PLvertex1[2], PLvertex1[3], G_tibiaposi[i], G_legposi[i])) { return true; }
#endif
#ifdef COLLISION_SHIN2
		if (CollisionDetection_PL4(PLvertex2[0], PLvertex2[1], PLvertex2[2], PLvertex2[3], G_tibiaposi[i], G_legposi[i])) { return true; }
#endif
#ifdef COLLISION_SHIN3
		if (CollisionDetection_PL4(PLvertex3[0], PLvertex3[1], PLvertex3[2], PLvertex3[3], G_tibiaposi[i], G_legposi[i])) { return true; }
#endif
	}

	//(2)地形の線分と脛の線分の最短距離をチェックして、最短距離が脛の半径より小さかったら干渉している。(脚の太さを考慮している)
	myvector::SVector shin_start[HexapodConst::LEG_NUM];
	myvector::SVector shin_v;	//膝から脚先への方向ベクトル
	myvector::SVector shin_end[HexapodConst::LEG_NUM];  //脚先だけ干渉チェックしないように線分の長さを調整
	double shin_l = SHIN_L / 100;//膝から足先までの長さ(130mm)の80%(104mm)脛の半径と合わせて130を超えなければおｋ

	for (int i = 0; i < HexapodConst::LEG_NUM; ++i) 
	{
		shin_start[i] = myvector::VGet(0, G_tibiaposi[i].y, G_tibiaposi[i].z);	//線分をyz平面に投影。
		shin_v = myvector::subVec(G_legposi[i], G_tibiaposi[i]);
		shin_end[i] = myvector::addVec(G_tibiaposi[i], myvector::VScale(shin_v, shin_l));//膝から足先までの長さ(130mm)の80%(104mm)脛の半径と合わせて130を超えなければおｋ
		shin_end[i].x = 0;	//線分をyz平面に投影。
	}

	myvector::SVector PLseg1[4];//地形の線分
	myvector::SVector PLseg2[4];
	myvector::SVector PLseg3[4];

	for (int i = 0; i < 4; ++i) 
	{
		PLseg1[i] = myvector::VGet(0, PLvertex1[i].y, PLvertex1[i].z);	//線分をyz平面に投影。
		PLseg2[i] = myvector::VGet(0, PLvertex2[i].y, PLvertex2[i].z);	//線分をyz平面に投影。
		PLseg3[i] = myvector::VGet(0, PLvertex3[i].y, PLvertex3[i].z);	//線分をyz平面に投影。
	}

	for (int i = 0; i < HexapodConst::LEG_NUM; ++i) 
	{
#ifdef COLLISION_SHIN1
		if (calcSegmentSegmentDist(PLseg1[1], PLseg1[2], shin_start[i], shin_end[i]) <= SHIN_R) { return true; }
#endif
#ifdef COLLISION_SHIN2
		if (calcSegmentSegmentDist(PLseg2[1], PLseg2[2], shin_start[i], shin_end[i]) <= SHIN_R) { return true; }
#endif
#ifdef COLLISION_SHIN3
		if (calcSegmentSegmentDist(PLseg3[1], PLseg3[2], shin_start[i], shin_end[i]) <= SHIN_R) { return true; }
#endif
	}

	//脛がポリゴンと干渉していないので0を返す。
	return false;
}

void SearchPossibleLegPosition::SetLegGroundableCandidatePointMAX(const int LGCP_MAX)
{
	LegGroundableCandidatePoint_MAX = LGCP_MAX;
}