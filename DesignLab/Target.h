#pragma once
#include "vectorFunc.h"

//●列挙子について
//	以下で定義されているETargetModeが列挙子に当たります
//	イメージとしては以下に宣言された値しかとることのできない
//	新しい変数を定義するような感じです．
//	こうすることで関数を特定のモードで動作させたいときなどに，
//	入力されたくない数字が入力されずらくなります．
//	また値に名前を付けることで何を想定してこの値が代入されたのかが分かりやすくなるという利点があります．
//	C++ には enum と enum class という2通りの列挙子がありますが，安全のため enum class の仕様がおすすめです．
//	よくわからんっていうのであれば以下の記述を真似して書いてみてください．


//Targetへの移動をどのように評価するか表す列挙子
enum class ETargetMode : int
{
	NONE = 0,						//0:評価しない
	STRAIGHT_VECOTR = 1,			//1:直線移動（ベクトル）
	STRAIGHT_POSITION = 2,			//2:直線移動（座標）
	TURN_ON_SPOT_DIRECTION = 3,		//3:その場旋回（回転方向）
	TURN_ON_SPOT_ANGLE = 4,			//4:その場旋回（回転角度）
	TURN_DIRECTION = 5,				//5:旋回（方向）
	TURN_ANGLE = 6,					//6:旋回（角度）
};

//探索において目標となる座標や角度，評価する値についてまとめた構造体
typedef struct TargetSet 
{
	myvector::SVector TargetDirection;		//目標方向x,y,z
	myvector::SVector TargetPosition;		//目標位置
	myvector::SVector TargetRotation;		//目標角度P,R,Y
	myvector::SVector TargetAngle;			//目標旋回方向（反時計回り→正）
	myvector::SVector RotationCenter;		//回転中心x,y,z
	double TurningRadius;
	ETargetMode TargetMode;					//1:直線移動（ベクトル）,2:直線移動（座標）,3:その場旋回（回転方向）,4:その場旋回（回転角度）,5:旋回（方向）,6:旋回（角度）

} STarget;