//#pragma once
//#include "Target.h"
//#include "designlab_vector3.h"
//#include "hexapod_const.h"
//
////phantomXを用いて矩形脚先軌道で脚を動かすことを想定した場合は,以下の値に設定すること（実機実験のでも使える）
////また、SPLP.hの BODY_MARGINはMIN_DELTAZと値を合わせて、安定余裕は15mmにすること（CreateComCandidate.hのSTABILITYMARGINの値）
////ヨー軸の可動域±40°
////
////#define LEGROM_RMARGIN		10	//脚可動範囲のマージン(外側）
////#define LEG_HIEGHT			-25.0f//脚の振り上げ高さ
////#define MIN_LEG_RADIUS		130
////#define MAX_LEG_RADIUS		200//コメントアウトすれば運動学通り
////#define MIN_DELTAZ			50
////#define MAX_DELTAZ			190
//
//#define LEGROM_RMARGIN		10		//脚可動範囲のマージン(外側）これは一応そのまま
//#define LEG_HIEGHT			-20		//脚の振り上げ高さ
//#define MIN_LEG_RADIUS		120
////#define MAX_LEG_RADIUS	200		//脚を伸ばし切らない程度200orコメントアウト
//
////安定余裕は0mm(CreateComCandidate.hのSTABILITYMARGINの値)
////SPLP.hの BODY_MARGINはMIN_DELTAZと値を合わせる
//
//const designlab::Vector3 COMPOSI = designlab::Vector3(0.0f, 0.0f, 0.0f);	//designlab::VGet(0.0, 0.0, 0.0);//重心位置
//
//// 自機の状態型
//struct STransform
//{
//	designlab::Vector3	com;		//3次元位置 z上 右手座標系 yが前
//	float				thR, thP, thY;	//3次元回転	テイトブライアン角
//};
//
//class Hexapod
//{
//
//	/*	脚番号の設定
//			\		 /
//			 \______/			座標系 座標系混ざりすぎて草　
//			  |5   0|			x
//			  |		|			⇑
//		   ---|4   1|---	   z✖⇒y
//			  |		|			(z軸はロボットから見て鉛直下向きが正)
//			  |3___2|
//			 /		\
//			/		 \
//	*/
//private:
//	STransform ziki;								//自機の位置と向き
//	STarget m_target;
//	designlab::Vector3	m_local_leg_pos[HexapodConst::LEG_NUM];			//各脚の位置		ローカル座標 coxaJointが原点
//	designlab::Vector3	m_local_leg2_pos[HexapodConst::LEG_NUM];		//各脚の基準位置	ローカル座標 coxaJointが原点 （旧名 L_Position_of_2）
//	designlab::Vector3	m_local_coxajoint_pos[HexapodConst::LEG_NUM];	//coxaJointの位置 	ローカル座標 重心が原点 x横 y前方 z縦
//	designlab::Vector3	m_local_femurjoint_pos[HexapodConst::LEG_NUM];	//FemurJointの位置 	ローカル座標 重心が原点
//	designlab::Vector3	m_local_tibiajoint_pos[HexapodConst::LEG_NUM];	//TibiaJointの位置 	ローカル座標 重心が原点
//
//	//脚可動範囲の半径 胴体高さについてのテーブル 200の意味は胴体中心から鉛直下向きに200mm
//	static int m_leg_max_r[200];
//
//	//可動範囲と脚の接地点から  脚の接地点が可動範囲内ならば1 外だったら0を返す．全然setJointPosじゃない．
//	bool setJointPosi();
//
//public:
//	Hexapod();
//	~Hexapod() = default;
//
//	// PFで使用
//	designlab::Vector3 rotation(const designlab::Vector3& In, const designlab::Vector3& center, const float thP, const float thR, const  float thY) const;
//
//	// 可動範囲と脚の接地点から，脚の接地点が可動範囲内ならばtrue 外だったらfalseを返す．
//	// 扇形であるという仮定から算術する方式 ロボット固有 LineEndは多分普通に回転させてないからグローバル(coxa座標系で見たとき)のcoxaから接地点までのベクトル
//	// 脚先が可動範囲内かどうか(脚番号,coxaからの位置)．大木さんのやつでしか使ってない．（旧名 check_touchdown_point）
//	// SPBR と SPLPで使用．間接的にCCCでも使用．
//	bool isLegWithinRange(const int legNum, const designlab::Vector3& LineEnd) const;
//
//	// 脚先、接地可能点、重心高さが一定でないときの可動範囲を修正したもの今のところSPLPのpossibleLegPointRotationで使う予定．
//	// delta_zは重心高さ（グローバル）と足先の高さ（グローバル）の差で、大きくとっても0~200mmの間に収まらなきゃダメ。LegROM_r[]のインデックス
//	// 多分↑の関数とやってること変わらなげ。。後々delta_zは接地面座標系で見た重心と脚先高さの差にしなければならない。
//	// SPLPで使用
//	bool check_touchdown_point2(int legNum, const designlab::Vector3& LineEnd, const float delta_z) const;
//
//	//実行可能な体勢ならばtrue．legはCoxaJointを(0,0,0)としたときの位置．CCCで使用．
//	bool isAblePause(const designlab::Vector3 _leg[HexapodConst::LEG_NUM], const bool _is_leg_grounded[HexapodConst::LEG_NUM]);
//
//	//脚位置と重心位置から転倒しないか判断し，転倒しないならばtrueを返す．CCCで使用．
//	bool isAbleCOM(const designlab::Vector3 _leg[HexapodConst::LEG_NUM], const bool _is_leg_grounded[HexapodConst::LEG_NUM]);
//
//	//初期姿勢。下向きにZの正、胴体前方にX、右手座標系でY　正直、体勢によらずロボット固有のものだから、計算結果を定数でコピーでもいい。。可動域を変えないのであれば、
//	// PFで使用
//	static void makeLegROM_r();
//
//	//扇形が内接するxy軸に平行な長方形の左下p1と右上p2の頂点を導出している。//hato20200710
//	//現在の位置と向きにおける脚到達範囲を返す p1:最小角度の時の座標 p2:最大角度の時の座標 ピッチ・ロール回転は考慮していない 扇形であること前提
//	// SPLPで使用
//	void calculateRangeOfMovement(const int legnum, designlab::Vector3& p1, designlab::Vector3& p2) const;
//
//	//setter と getter．数が多すぎる．減らしたい
//
//	void setMyDirection(const float _thP, const float _thR, const float _thY);	//姿勢
//	void setMyPosition(const designlab::Vector3 _com) { this->ziki.com = _com; }		//重心位置
//	void setTarget(const STarget _target) { this->m_target = _target; }
//	void setLocalLeg2Pos(const designlab::Vector3 posi2[HexapodConst::LEG_NUM]);			//旧名 setPosition_of_2
//	void setLocalLegPos(const designlab::Vector3 posi[HexapodConst::LEG_NUM]);
//
//	//ロボットの姿勢を出力
//	inline designlab::Vector3 getTargetDirection() const { return this->m_target.TargetDirection; }	//Targetの値を受け取る
//	inline designlab::Vector3 getTargetRotation() const { return { 0,0,0 }/*this->m_target.TargetRotation*/; }	//Targetの値を受け取る
//	inline designlab::Vector3 getTargetAngle() const { return { 0,0,0 }/*this->m_target.TargetAngle*/; }			//Targetの値を受け取る
//	inline designlab::Vector3 getRotaionCenter()const { return this->m_target.RotationCenter; }		//Targetの値を受け取る
//	inline float getTurningRadius() const { return this->m_target.TurningRadius; }					//Targetの値を受け取る
//	inline ETargetMode getTargetMode() const { return this->m_target.TargetMode; }					//Targetの値を受け取る
//
//	inline designlab::Vector3 getLocalLegPos(const int _leg_num) const { return this->m_local_leg_pos[_leg_num]; }
//	inline designlab::Vector3 getLocalLeg2Pos(const int _leg_num) const { return this->m_local_leg2_pos[_leg_num]; }	//旧名 getLocalPosition_of_2
//	inline designlab::Vector3 getLocalCoxaJointPos(const int _leg_num) const { return m_local_coxajoint_pos[_leg_num]; }
//
//	//旧名 getGlobalLeg2Pos
//	designlab::Vector3 getGlobalLegPos(const int _leg_num) const;
//	designlab::Vector3 getGlobalLeg2Pos(const int _leg_num) const;
//	designlab::Vector3 getGlobalCoxaJointPos(const int _leg_num) const;
//	designlab::Vector3 getGlobalFemurJointPos(const int _leg_num) const;
//	designlab::Vector3 getGlobalTibiaJointPos(const int _leg_num) const;
//	inline float getGlobalMyDirectionthP() const { return this->ziki.thP; }
//	inline float getGlobalMyDirectionthR() const { return this->ziki.thR; }
//	inline float getGlobalMyDirectionthY() const { return this->ziki.thY; }
//	designlab::Vector3 getGlobalMyPosition() const { return ziki.com; }
//};
//
