#pragma once

#include "pch.h"
#include "vectorFunc.h"
#include "SearchLegPosition.h"
#include "hexapod.h"

//脚の数
#define LEGCOUNT				6	

//ノードの個数
#define NODEMAX					100	

//アークの個数
#define ARCMAX					100	

//グラフが作る多角形領域の個数
#define POLYGONNUM				26

//無視するアークの長さ
#define ELLENGTH				30

//無視するアークの角度
#define ELANGLE					0.05

//衝突判定のマージン
#define HITDETECTIONMARGIN		0.01

//重心位置を求めるためにポリゴン内を分割する数
#define DIVIDE_NUM				10

//許容する旋回半径からの重心のずれ追加hato、旋回半径で枝刈りしたいなら小さくする。
#define ALLOW_RADIUS_DIFF		1000

//安定余裕追加hato　一応40mmくらいまで行けなくはないけど、探索空間が小さくなるから、安定余裕を大きくするなら探索ルールや探索深さを変えて調整した方がいい。
//実機は安定余裕15mmが無難、10mmだと脚配置によってはたまに転倒する場合がある。15は今のところ大丈夫そう。
#define STABILITYMARGIN			10

//胴体の平行移動による脛と地形（侵入できない領域）の干渉をチェックするときはコメントアウトを外す。
//すねの干渉チェックをする関数は、SPLP.hのCollision_judgment_with_shin()
//#define COLLISION_CHECK_SHIN


//重心位置を求める際に使うグラフのノード
typedef struct _tagInterNode
{
	myvector::SVector Point;		//3D位置
} IntersectionNode;

//多角形の座標を格納
typedef struct _intersectionpolygon
{
	int nOfVertex;									//頂点の数
	struct _tagInterNode*  VertexNode[LEGCOUNT];	//ポリゴンを形成する頂点の座標ロボット座標系　6角形の場合内部の領域は最大6角形のはず…　確証がない	
	myvector::SVector COMPoint;						//重心位置 重心の移動量
	int COMtype;									//重心タイプ
	myvector::SVector Leg[LEGCOUNT];				//重心移動後の脚座標 ローカル座標系(coxa)
} IntersectionPolygon;


class CreateComCandidate
{
public:
	CreateComCandidate();
	~CreateComCandidate()= default;

	//メンバ変数
	Hexapod phantomX;
	SearchPossibleLegPosition S_P_L_P2;//胴体と地面の衝突判定用　独立性低くなるけど、
	myvector::SVector p_candidatePoint[POLYGONNUM];		//重心移動可能点群
	int candidatePointNum;
	int p_candidatePointType[POLYGONNUM];
	myvector::SVector p_candidatePointLeg[LEGCOUNT][20];
	IntersectionPolygon IPolygon[POLYGONNUM];	

	//重心移動が可能な点を探索　もろもろデータを入力後実行
	int getMovementPossibilityArea();

private:
	//メンバ変数
	myvector::SVector m_leg_pos[LEGCOUNT];	//ロボット座標系　20200619
	double m_stability_margin;
	myvector::SVector m_global_com;					//お試し．190419で検索すれば
	IntersectionNode INode[NODEMAX];
	int groundingLeg[6];//遊脚は０接地脚は１

	//メンバ関数
	int getInsidePolygon();
	int getComInPolygon(IntersectionPolygon* polygon);
	int getComInPolygon_circlingTarget(IntersectionPolygon* polygon);

	bool lineSegmentHitDetection(myvector::SVector s1, myvector::SVector e1, myvector::SVector s2, myvector::SVector e2, myvector::SVector *crossPoint);
	double VCrossXZ(myvector::SVector In1, myvector::SVector In2);
	bool ContactJudgment(double t1, double t2);
	bool VectorEqual(myvector::SVector v1, myvector::SVector v2);

public:
	//getterとsetter

	//安定余裕を設定する
	void setStabilityMargin(const double stabilityMarginIn) { m_stability_margin = stabilityMarginIn; }

	//脚の座標を設定する
	void setLegPosition(myvector::SVector legIn[LEGCOUNT]);

	//重心を設定する
	void setGlobalCenterOfMass(const myvector::SVector _global_com) { m_global_com = _global_com; }

	void setGroundingLeg(int In_groundingLeg[6]);
};