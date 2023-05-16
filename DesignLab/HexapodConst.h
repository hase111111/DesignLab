#pragma once

// Hexapod，phantomXのパラメータを定数で表現したもの．

class HexapodConst final
{
public:
	//コンストラクタとコピーコンストラクタを削除し，実体を生成できないようにする．
	HexapodConst() = delete;
	HexapodConst(const HexapodConst& _other) = delete;

	//Hexapodの脚の本数を表す．これを変更しても脚の本数が変更できるわけではない．マジックナンバーをなくすことが目的．
	constexpr static int LEG_NUM = 6;

	//ロボットの寸法
	const static double COXA_LENGTH;		//第1関節部の長さ．詳しくはreferenceをフォルダ参照．
	const static double FEMUR_LENGTH;		//第2関節部の長さ．詳しくはreferenceをフォルダ参照．
	const static double TIBIA_LENGTH;		//第3関節部の長さ．詳しくはreferenceをフォルダ参照．
	const static double BODY_FRONT_WIDTH;	//前方の幅[mm]．phantomXの横幅.
	const static double BODY_CENTER_WIDTH;	//中心の幅[mm]．phantomXの横幅.
	const static double BODY_REAR_WIDTH;	//後方の幅[mm]．phantomXの横幅.
	const static double BODY_FRONT_LENGTH;	//中央から前方までの距離[mm]．phantomXの縦幅.
	const static double BODY_REAR_LENGTH;	//中央から前方までの距離[mm]．phantomXの縦幅.

	//ロボットの可動範囲
	const static double VERTICAL_MAX_RANGE;	//地面の最高点と胴体下方の隙間の最大値を示す．脚を伸ばし切らない程度に設定する．旧名 MAX_DELTAZ
	const static double VERTICAL_MIN_RANGE;	//地面の最高点と胴体下方の隙間の最小値を示す．旧名 MIN_DELTAZ
};

