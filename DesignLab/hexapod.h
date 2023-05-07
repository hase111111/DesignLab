#pragma once

#include "listFunc.h"
#include "Target.h"
#include "vectorFunc.h"
#include "Define.h"

/*
*	------------------------------------------------------------
*		機体の大きさの定義
*	------------------------------------------------------------
*/
#define FWIDTH				60.0f	//前方の幅[mm]
#define MWIDTH				100.0f	//中心の幅[mm]
#define RWIDTH				60.0f	//後方の幅[mm]
#define FLENGTH				120.0f	//中央から前方までの距離[mm]
#define RLENGTH				120.0f	//中央から後方までの距離[mm]

#define L_COXA				52.0f
#define L_FEMUR				66.0f
#define L_TIBIA				130.0f

//phantomXを用いて矩形脚先軌道で脚を動かすことを想定した場合は,以下の値に設定すること（実機実験のでも使える）
//また、SPLP.hの BODY_MARGINはMIN_DELTAZと値を合わせて、安定余裕は15mmにすること（CreateComCandidate.hのSTABILITYMARGINの値）
//ヨー軸の可動域±40°
//
//#define LEGROM_RMARGIN		10	//脚可動範囲のマージン(外側）
//#define LEG_HIEGHT			-25.0f//脚の振り上げ高さ
//#define MIN_LEG_RADIUS		130
//#define MAX_LEG_RADIUS		200//コメントアウトすれば運動学通り
//#define MIN_DELTAZ			50
//#define MAX_DELTAZ			190



#define LEGROM_RMARGIN		10	//脚可動範囲のマージン(外側）これは一応そのまま
#define LEG_HIEGHT			-20//脚の振り上げ高さ
#define MIN_LEG_RADIUS		120
//#define MAX_LEG_RADIUS		200//脚を伸ばし切らない程度200orコメントアウト
#define MIN_DELTAZ			30		//地面の最高点と胴体下方の隙間
#define MAX_DELTAZ			190		//脚を伸ばし切らない程度

//安定余裕は0mm(CreateComCandidate.hのSTABILITYMARGINの値)
//SPLP.hの BODY_MARGINはMIN_DELTAZと値を合わせる




const myvector::SVector COMPOSI = myvector::VGet(0.0, 0.0, 0.0);	//myvector::VGet(0.0, 0.0, 0.0);//重心位置

//typedef struct TargetSet STarget;

// 自機の状態型
struct STransform
{
	myvector::SVector				com;		//3次元位置 z上 右手座標系 yが前
	double				thR, thP, thY;	//3次元回転	テイトブライアン角
};


//-----------------------------------------------------------------

class Hexapod
{

	/*	脚番号の設定
			\		 /
			 \______/			座標系 座標系混ざりすぎて草　
			  |5   0|			x
			  |		|			⇑
		   ---|4   1|---	   z✖⇒y
			  |		|			(z軸はロボットから見て鉛直下向きが正)
			  |3___2|
			 /		\
			/		 \
	*/
private:
	STransform ziki;								//自機の位置と向き
	STarget Target;
	myvector::SVector	L_Leg_Position[6];		//各脚の位置	ローカル座標 coxaJointが原点
	myvector::SVector	L_Position_of_2[6];		//各脚の基準位置	ローカル座標 coxaJointが原点
	myvector::SVector	L_CoxaJoint_posi[6];			//coxaJointの位置 			ローカル座標 重心が原点
	myvector::SVector	G_CoxaJoint_posi[6];			//coxaJointの位置 			グローバル座標
	myvector::SVector	L_FemurJoint_posi[6];			//FemurJointの位置 		ローカル座標 重心が原点
	myvector::SVector	G_FemurJoint_posi[6];			//FemurJointの位置 		グローバル座標
	myvector::SVector	L_TibiaJoint_posi[6];			//TibiaJointの位置 		ローカル座標 重心が原点
	myvector::SVector	G_TibiaJoint_posi[6];			//TibiaJointの位置 		グローバル座標
	bool setJointPosi();

public:
	Hexapod(void);//脚の付け根座標代入L_CoxaJoint_posi
	~Hexapod(void);//空

	int LegROM_r[200];						//脚可動範囲の半径 胴体高さについてのテーブル 200の意味は胴体中心から鉛直下向きに200mm
	myvector::SVector rotation(myvector::SVector In, myvector::SVector center, double thP, double thR, double thY);
	myvector::SVector show_normal_vector();//胴体からなる平面の法線ベクトルを返すつまり、ロボット座標系のｚ軸方向の単位ベクトル
	bool check_touchdown_point(int legNum, const myvector::SVector& LineEnd);//脚先が可動範囲内かどうか(脚番号,coxaからの位置)
	bool check_touchdown_point2(int legNum, const myvector::SVector& LineEnd, const double delta_z);//追加20200612hato
	bool check_touchdown_point3(int legNum, const myvector::SVector& LineEnd, const double delta_z);//追加20200612hato
	bool check_touchdown_point4(int legNum, const myvector::SVector& LineEnd);//追加20200709hato

	bool isAblePause(myvector::SVector* leg, const bool groundingLeg[Define::LEG_NUM]);
	bool isAbleCOM(myvector::SVector* leg, const bool groundingLeg[Define::LEG_NUM]);
	void makeLegROM_r();
	void calculateRangeOfMovement(int legnum, myvector::SVector& p1, myvector::SVector& p2);

	//ロボットの姿勢を入力
	void setMyDirection(const double _thP, const double _thR, const double _thY);//姿勢
	void setMyPosition(const myvector::SVector com);//重心位置
	void setTarget(const STarget _target);//目標
	void setPosition_of_2(const myvector::SVector posi2[Define::LEG_NUM]);
	void setMyLegPosition(const myvector::SVector posi[Define::LEG_NUM]);

	//ロボットの姿勢を出力
	myvector::SVector getTravelingDirection();
	myvector::SVector getTurningDirection();
	myvector::SVector getTurningAngle();
	myvector::SVector getTurningCenter();
	double getTurningRadius();
	ETargetMode getTargetType();
	myvector::SVector getLocalLegPosition(int legNum);
	myvector::SVector getLocalPosition_of_2(int legNum);
	myvector::SVector getGlobalLegPos(int legNum);
	myvector::SVector getPosition_of_2(int legNum);
	myvector::SVector getGlobalCoxaJointPos(int legNum);
	myvector::SVector getGlobalFemurJointPos(int legNum);
	myvector::SVector getLocalFemurJointPos(int legNum);//追加hato
	myvector::SVector getGlobalTibiaJointPos(int legNum);
	myvector::SVector getLocalCoxaJointPos(int legNum);
	double getGlobalMyDirectionthP();
	double getGlobalMyDirectionthR();
	double getGlobalMyDirectionthY();
	myvector::SVector getGlobalMyPosition();
};

