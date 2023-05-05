#pragma once

#include "listFunc.h"
#include "Target.h"
#include "vectorFunc.h"
#include "Define.h"

//		機体の大きさの定義

//前方の幅[mm]
#define FWIDTH				60.0	
//中心の幅[mm]
#define MWIDTH				100.0	
//後方の幅[mm]
#define RWIDTH				60.0	
//中央から前方までの距離[mm]
#define FLENGTH				120.0	
//中央から後方までの距離[mm]
#define RLENGTH				120.0	

#define L_COXA				52.0
#define L_FEMUR				66.0
#define L_TIBIA				130.0

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



//myvector::VGet(0.0, 0.0, 0.0);//重心位置
const myvector::SVector COMPOSI = myvector::VGet(0.0, 0.0, 0.0);	

// 自機の状態型
struct STransform
{
	myvector::SVector	com ;			//3次元位置 z上 右手座標系 yが前
	double				thR, thP, thY;	//3次元回転	テイトブライアン角
};



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
	myvector::SVector	L_Leg_Position[Define::LEG_NUM];	//各脚の位置		ローカル座標 coxaJointが原点
	myvector::SVector	L_Position_of_2[Define::LEG_NUM];	//各脚の基準位置	ローカル座標 coxaJointが原点
	myvector::SVector	m_local_coxajoint_pos[Define::LEG_NUM];		//coxaJointの位置 	ローカル座標 重心が原点
	myvector::SVector	m_global_coxajoint_pos[Define::LEG_NUM];	//coxaJointの位置 	グローバル座標
	myvector::SVector	m_local_femurjoint_pos[Define::LEG_NUM];	//FemurJointの位置 	ローカル座標 重心が原点
	myvector::SVector	m_global_femurjoint_pos[Define::LEG_NUM];	//FemurJointの位置 	グローバル座標
	myvector::SVector	L_TibiaJoint_posi[Define::LEG_NUM];	//TibiaJointの位置 	ローカル座標 重心が原点
	myvector::SVector	G_TibiaJoint_posi[Define::LEG_NUM];	//TibiaJointの位置 	グローバル座標

	//可動範囲と脚の接地点から  脚の接地点が可動範囲内ならばtrue 外だったらfalseを返す．逆運動学，運動学の計算から算術する方式 ロボット固有
	bool setJointPos();
	
public:
	Hexapod();	//脚の付け根座標代入L_CoxaJoint_posi
	~Hexapod() = default;

	int LegROM_r[200];						//脚可動範囲の半径 胴体高さについてのテーブル 200の意味は胴体中心から鉛直下向きに200mm
	myvector::SVector rotation(const myvector::SVector _in, const myvector::SVector _center, const double _thP, const double _thR, const double _thY) const;
	bool check_touchdown_point(int legNum, const myvector::SVector &LineEnd);							//脚先が可動範囲内かどうか(脚番号,coxaからの位置)
	bool check_touchdown_point2(int legNum, const myvector::SVector &LineEnd,const double delta_z);		//追加20200612hato
	bool check_touchdown_point3(int legNum, const myvector::SVector &LineEnd, const double delta_z);	//追加20200612hato
	bool check_touchdown_point4(int legNum, const myvector::SVector &LineEnd);							//追加20200709hato

	bool isAblePause(myvector::SVector* leg, int groundingLeg[6]);
	bool isAbleCOM(myvector::SVector* leg, int groundingLeg[6]);

	//初期姿勢。下向きにZの正、胴体前方にX、右手座標系でY　正直、体勢によらずロボット固有のものだから、計算結果を定数でコピーでもいい。。可動域を変えないのであれば、
	void makeLegROM_r();
	void calculateRangeOfMovement(int legnum, myvector::SVector& p1, myvector::SVector& p2);

	// setter
	 
	//重心位置を設定する．
	void setMyPosition(const myvector::SVector _com) { ziki.com = _com; }
	//目標を設定する．
	void setTarget(const STarget _target) { Target = _target; }
	//回転の姿勢を設定する
	void setMyDirection(const double _thP, const double _thR, const double _thY);
	void setPosition_of_2(const myvector::SVector _pos[Define::LEG_NUM]);
	void setMyLegPosition(const myvector::SVector _pos[Define::LEG_NUM]);

	// getter 数が多すぎるのであまりよくない
	myvector::SVector getTravelingDirection() const { return Target.TargetDirection; }
	myvector::SVector getTurningDirection() const { return Target.TargetRotation; }
	myvector::SVector getTurningAngle() const { return Target.TargetAngle; }
	myvector::SVector getTurningCenter() const { return Target.RotationCenter; }
	double getTurningRadius() const { return Target.TurningRadius; }
	ETargetMode getTargetType() const { return Target.TargetMode; };
	myvector::SVector getLocalLegPosition(const int _leg_num) const { return L_Leg_Position[_leg_num]; }
	myvector::SVector getLocalPosition_of_2(const int _leg_num) const { return L_Position_of_2[_leg_num]; }
	myvector::SVector getLocalFemurJointPos(const int _leg_num) const { return m_local_femurjoint_pos[_leg_num]; }	//追加hato
	myvector::SVector getLocalCoxaJointPos(const int _leg_num) const { return m_local_coxajoint_pos[_leg_num]; };
	double getGlobalMyDirectionthP() const { return ziki.thP; }
	double getGlobalMyDirectionthR() const { return ziki.thR; }
	double getGlobalMyDirectionthY() const { return ziki.thY; }
	//胴体からなる平面の法線ベクトルを返すつまり、ロボット座標系のｚ軸方向の単位ベクトル
	myvector::SVector getNormalVector() const;
	myvector::SVector getPosition_of_2(const int _leg_num) const;
	myvector::SVector getGlobalLegPos(const int _leg_num) const;
	myvector::SVector getGlobalMyPosition() const { return ziki.com; }
	myvector::SVector getGlobalCoxaJointPos(const int _leg_num) const;
	myvector::SVector getGlobalFemurJointPos(const int _leg_num) const;
	myvector::SVector getGlobalTibiaJointPos(const int _leg_num) const;
};

