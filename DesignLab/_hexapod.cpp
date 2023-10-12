//#include "hexapod.h"
//#include <iostream>
//#include <vector>
//#include "designlab_math.h"
//
////脚可動範囲の半径 胴体高さについてのテーブル 200の意味は胴体中心から鉛直下向きに200mm
//int Hexapod::m_leg_max_r[200] = { 0 };
//
//Hexapod::Hexapod()
//{
//	//脚の付け根座標代入L_CoxaJoint_posi m_local_coxajoint_pos
//	m_local_coxajoint_pos[0] = designlab::Vector3(HexapodConst::BODY_FRONT_WIDTH, HexapodConst::BODY_FRONT_LENGTH, 0.0f);
//	m_local_coxajoint_pos[1] = designlab::Vector3(HexapodConst::BODY_CENTER_WIDTH, 0.0f, 0.0f);
//	m_local_coxajoint_pos[2] = designlab::Vector3(HexapodConst::BODY_REAR_WIDTH, -HexapodConst::BODY_REAR_LENGTH, 0.0f);
//	m_local_coxajoint_pos[3] = designlab::Vector3(-HexapodConst::BODY_REAR_WIDTH, -HexapodConst::BODY_REAR_LENGTH, 0.0f);
//	m_local_coxajoint_pos[4] = designlab::Vector3(-HexapodConst::BODY_CENTER_WIDTH, 0.0f, 0.0f);
//	m_local_coxajoint_pos[5] = designlab::Vector3(-HexapodConst::BODY_FRONT_WIDTH, HexapodConst::BODY_FRONT_LENGTH, 0.0f);
//}
//
////ベクトルの3Dローテーション オイラー角（テイト・ブライアン角）Y-X-Z
//// 2018.12.27 コメントのY-X-Zって間違いっぽい．英語版wikipediaに詳しいが，多分Y-X-Yあたりのオイラー角の定義だと思うんですが……（テイト・ブライアンも違うのでは）
////2020.05.15.hato
////　これは、RPYが基本的に胴体の傾き（別に胴体じゃなくてもいいけど）だから、胴体座標系やcoxa座標系から座標ΣP（例えばcoxaからみた脚先座標など）を
////グローバル座標で見た座標Σ0Pにへんかんするための、
////	Σ0_P = （Σ0_R_Σ)ΣP + (グローバル座標系で見たその座標系の原点座標）
////っていう式の（Σ0_R_Σ)ΣPの計算をしてる関数。
////
////（Σ0RΣ)は回転行列で（Σ0RΣ)=Ry*Rx*Rz
////Ry,Rx,Rzは添え字の軸回りに回転させたときの回転行列をあらわす。
////一般的には、進行方向にｘ軸をとるから、
////Rxがロールの関数＝Rx(thR), Ryがピッチの関数=Ry(thP),Pz がヨーの関数=Rz(thY)
////になる。
////ただ、このシミュレーションでは、進行方向がy軸だから、
////Rxがピッチの関数＝Rx(thR), Ryがロールの関数=Ry(thP),Pz がヨーの関数=Rz(thY)
////になっている。
////また、Σ0RΣ　＝　R＊P＊Yの順に回転行列を計算するのが一般的？だから、それにあわせると、
////普通は、Σ0RΣ　＝　Rx(R) * Ry(P) * Rz(Y)　の順番で計算するところ
////ここでは、Σ0RΣ　＝　Ry(R) * Rx(P) * Rz(Y)　の順番で計算している。
////そこがちょっとややこしいところだけど、計算自体は間違ってない。
//designlab::Vector3 Hexapod::rotation(const designlab::Vector3& In, const designlab::Vector3& center, const float thP, const float thR, const float thY) const
//{
//	designlab::Vector3 ans, buf;
//	buf = In - center;
//	ans.x = (cos(thR) * cos(thY) + sin(thR) * sin(thP) * sin(thY)) * buf.x +
//		(cos(thY) * sin(thR) * sin(thP) - cos(thR) * sin(thY)) * buf.y +
//		(cos(thP) * sin(thR)) * buf.z;
//
//	ans.y = cos(thP) * sin(thY) * buf.x +
//		cos(thP) * cos(thY) * buf.y +
//		(-sin(thP)) * buf.z;
//
//	ans.z = (cos(thR) * sin(thP) * sin(thY) - cos(thY) * sin(thR)) * buf.x +
//		(cos(thR) * cos(thY) * sin(thP) + sin(thR) * sin(thY)) * buf.y +
//		(cos(thR) * cos(thP)) * buf.z;
//
//	return ans + center;
//}
//
//bool Hexapod::setJointPosi()
//{
//	// 逆運動学の計算結果を用いて準運動学を計算する．逆運動学，運動学の計算から算術する方式 ロボット固有
//	using namespace dl_math;
//
//	//逆運動学と運動学を行った結果が半径Permission^0.5の円の中なら等しいと考える
//	const float PERMISSION = 1;
//
//	const float mins[3] = { -1.428f, -1.780f, -1.194f };  //脚可動範囲
//	const float maxs[3] = { 1.402f,  1.744f,  1.769f };	//左からcoxa,femur,tibia
//	const float coxaMins[6] = { -0.610f, -1.428f, -2.213f, -0.960f, -1.745f, -2.531f };
//	const float coxaMaxs[6] = { 2.187f,  1.402f,  0.617f,  2.531f,  1.745f,  0.960f };
//
//	for (int legNum = 0; legNum < HexapodConst::kLegNum; legNum++)
//	{
//		const designlab::Vector3 legposi(m_local_leg_pos[legNum].x, m_local_leg_pos[legNum].y, -m_local_leg_pos[legNum].z);
//
//		//逆運動学
//		// first, make this a 2DOF problem... by solving coxa
//		const float coxa = atan2(legposi.y, legposi.x);//coxa角度
//
//		const float IK_trueX = sqrt(squared(legposi.x) + squared(legposi.y)) - HexapodConst::PHANTOMX_COXA_LENGTH;//xy平面におけるfemur脚先までの距離
//		const float im = sqrt(squared(IK_trueX) + squared(legposi.z));								//femurから脚先までの距離
//
//		// get femur angle above horizon...
//		float q1 = -atan2(legposi.z, IK_trueX);//femurから脚先へのベクトルとxy平面との角度
//		float d1 = squared(HexapodConst::PHANTOMX_FEMUR_LENGTH) - squared(HexapodConst::PHANTOMX_TIBIA_LENGTH) + squared(im);
//		float d2 = 2.0f * HexapodConst::PHANTOMX_FEMUR_LENGTH * im;
//		float q2 = acos(d1 / d2);
//		const float femur = q1 + q2;
//
//		// and tibia angle from femur...
//		d1 = pow(HexapodConst::PHANTOMX_FEMUR_LENGTH, 2.0f) - squared(im) + squared(HexapodConst::PHANTOMX_TIBIA_LENGTH);
//		d2 = 2.0f * HexapodConst::PHANTOMX_TIBIA_LENGTH * HexapodConst::PHANTOMX_FEMUR_LENGTH;
//		const float tibia = acos(d1 / d2) - dl_math::MY_FLT_PI / 2.0f;
//
//		//運動学
//		const float K_trueX = HexapodConst::PHANTOMX_FEMUR_LENGTH * cos(femur) + HexapodConst::PHANTOMX_TIBIA_LENGTH * cos(femur + tibia - dl_math::MY_FLT_PI / 2.0f);
//
//		designlab::Vector3 kinematics;					//ans of kinematics use sorution of i_kinematics
//		kinematics.x = cos(coxa) * (K_trueX + HexapodConst::PHANTOMX_COXA_LENGTH);
//		kinematics.y = sin(coxa) * (K_trueX + HexapodConst::PHANTOMX_COXA_LENGTH);
//		kinematics.z = -(HexapodConst::PHANTOMX_FEMUR_LENGTH * sin(femur) + HexapodConst::PHANTOMX_TIBIA_LENGTH * sin(femur + tibia - dl_math::MY_FLT_PI / 2.0f));
//
//		m_local_femurjoint_pos[legNum] = m_local_coxajoint_pos[legNum] + designlab::Vector3(HexapodConst::PHANTOMX_COXA_LENGTH * cos(coxa), HexapodConst::PHANTOMX_COXA_LENGTH * sin(coxa), 0.0f);
//		m_local_tibiajoint_pos[legNum] = m_local_femurjoint_pos[legNum] + designlab::Vector3(HexapodConst::PHANTOMX_FEMUR_LENGTH * cos(femur) * cos(coxa), HexapodConst::PHANTOMX_FEMUR_LENGTH * cos(femur) * sin(coxa), HexapodConst::PHANTOMX_FEMUR_LENGTH * sin(femur));
//
//		const float Permission = (kinematics - legposi).GetSquaredLength();
//
//		if (PERMISSION < Permission) { std::cout << "Error PERMISSION OVER \n"; return true; };
//	}
//
//	return false;
//}
//
//bool Hexapod::isLegWithinRange(const int legNum, const designlab::Vector3& LineEnd) const
//{
//	//可動域が80,60,55,50,45,35,30degのパターンもかつてはあり，コメントアウトされていたが，全て削除して整理した．
//
//	//可動域40			
//	//const float _movable_angle = 40.0f;
//
//	// 5 , 40 , 85,
//	// 85, 40 ,  5,
//	// 5 , -40,
//	// 85, 40 ,
//	const float LCoxaJointMinsCos[HexapodConst::kLegNum] = { 0.996194698f,	0.766044443f,	0.087155743f,	-0.996194698f,	-0.766044443f,	-0.087155743f };
//	const float LCoxaJointMaxsCos[HexapodConst::kLegNum] = { 0.087155743f,	0.766044443f,	0.996194698f,	-0.087155743f,	-0.766044443f,	-0.996194698f };
//	const float LCoxaJointMinsSin[HexapodConst::kLegNum] = { 0.087155743f,	-0.64278761f,	-0.996194698f,	-0.087155743f,	0.64278761f,	0.996194698f };
//	const float LCoxaJointMaxsSin[HexapodConst::kLegNum] = { 0.996194698f,	0.64278761f,	-0.087155743f,	-0.996194698f,	-0.64278761f,	0.087155743f };
//
//	//外積,脚位置が扇形のスタート位置より<180°なら+,>180°なら-
//	float crossMinIn = LCoxaJointMinsCos[legNum] * LineEnd.x - LCoxaJointMinsSin[legNum] * LineEnd.y;
//	float crossMaxIn = LCoxaJointMaxsCos[legNum] * LineEnd.x - LCoxaJointMaxsSin[legNum] * LineEnd.y;
//
//	//扇形の角度が180以上なら符号が逆になる
//	if (crossMinIn < 0 || crossMaxIn > 0) { return false; }
//
//	//X0Y0平面に投影した脚の根元から脚先までの半径を求める
//	designlab::Vector3 xy_LineEnd = LineEnd;
//	xy_LineEnd.z = 0;					//引数のZの値を0にしたもの
//	float delta = xy_LineEnd.GetLength();	// m_leg_max_r その胴体高さにおける脚の到達半径の長さ
//
//	//付け根からxy平面の距離<胴体高さに対する可動範囲の半径．胴体から近い場合は体勢がきついので使用しない20180312
//	if (MIN_LEG_RADIUS < delta && delta < m_leg_max_r[int(LineEnd.z)])
//	{
//		return true;
//	}
//
//	return false;
//}
//
//bool Hexapod::check_touchdown_point2(int legNum, const designlab::Vector3& LineEnd, const float delta_z) const
//{
//	//if (int(delta_z) < MIN_DELTAZ || MAX_DELTAZ < int(delta_z)) { return 0; }
//	if (int(delta_z) < HexapodConst::VERTICAL_MIN_RANGE || HexapodConst::VERTICAL_MAX_RANGE < int(delta_z)) { return 0; }
//
//	//可動域が80,60,55,50,45,35,30degのパターンもかつてはあり，コメントアウトされていたが，全て削除して整理した．
//
//	//可動域40													
//	const	float	LCoxaJointMinsCos[6] = { 0.996194698f	,	0.766044443f,	0.087155743f ,	-0.996194698f,	-0.766044443f,	-0.087155743f };
//	const	float	LCoxaJointMaxsCos[6] = { 0.087155743f	,	0.766044443f,	0.996194698f ,	-0.087155743f,	-0.766044443f,	-0.996194698f };
//	const	float	LCoxaJointMinsSin[6] = { 0.087155743f	,	-0.64278761f,	-0.996194698f,	-0.087155743f,	0.64278761f	 ,	0.996194698f };
//	const	float	LCoxaJointMaxsSin[6] = { 0.996194698f	,	0.64278761f	,	-0.087155743f,	-0.996194698f,	-0.64278761f ,	0.087155743f };
//
//
//	//外積,脚位置が扇形のスタート位置より<180°なら+,>180°なら-
//	float crossMinIn = LCoxaJointMinsCos[legNum] * LineEnd.x - LCoxaJointMinsSin[legNum] * LineEnd.y;
//	float crossMaxIn = LCoxaJointMaxsCos[legNum] * LineEnd.x - LCoxaJointMaxsSin[legNum] * LineEnd.y;
//
//	//扇形の角度が180以上なら符号が逆になる
//	if (crossMinIn < 0 || crossMaxIn > 0) { return false; }
//
//	designlab::Vector3 xy_LineEnd;
//	xy_LineEnd = LineEnd;
//	xy_LineEnd.z = 0;
//
//	// m_leg_max_r その胴体高さにおける脚の到達半径の長さ
//	float delta = xy_LineEnd.GetLength();//X0Y0平面に投影した脚の根元から脚先までの半径
//
//	//付け根からxy平面の距離<胴体高さに対する可動範囲の半径//胴体から近い場合は体勢がきついので使用しない20180312
//	if (MIN_LEG_RADIUS < delta && delta < m_leg_max_r[int(delta_z)])
//	{
//		//重心高さとある脚設置可能点の高さの差に応じて、許容する半径が変わる。
//		//今のLegROM_rの計算じゃ73mmより上にあげると探索不可能　後々はインデックスは、接地面座標系で見た重心と脚先高さの差にしなければならない。
//		return true;
//	}
//
//	return false;
//}
//
//bool Hexapod::isAblePause(const designlab::Vector3 _leg[HexapodConst::kLegNum], const bool _is_leg_grounded[HexapodConst::kLegNum])
//{
//	for (int i = 0; i < HexapodConst::kLegNum; i++)
//	{
//		//接地している脚のみ判定する
//		if (_is_leg_grounded[i] == true)
//		{
//			//接地可能地点がないのならば false
//			if (isLegWithinRange(i, _leg[i]) == false)
//			{
//				return false;
//			}
//		}
//	}
//
//	return true;
//}
//
//bool Hexapod::isAbleCOM(const designlab::Vector3 L_leg[HexapodConst::kLegNum], const bool _is_leg_grounded[HexapodConst::kLegNum])
//{
//	//接地している脚のみこのvectorにいれる
//	std::vector<designlab::Vector3> _grounded_leg_pos;
//
//	for (int i = 0; i < HexapodConst::kLegNum; i++)
//	{
//		if (_is_leg_grounded[i] == true)
//		{
//			// 何故 xとyを入れ替えているのか ? → プログラムによって座標系が違う恐ろしい状態になっているため
//			auto temp_x_y_change = designlab::Vector3(L_leg[i].y, L_leg[i].x, L_leg[i].z);
//			_grounded_leg_pos.push_back(temp_x_y_change + getGlobalCoxaJointPos(i));
//		}
//	}
//
//	//i番目の脚位置を基としてi+1番脚までのベクトルと重心(0,0,0)の位置を比べて常に右にあったらok
//	for (size_t i = 0; i < _grounded_leg_pos.size(); i++)
//	{
//		//i番目の脚先からi+1番目の脚先へ向かうベクトル．余りをとっているのは，i = 最大値, i+1 = 0となるときのため.
//		designlab::Vector3 i_to_i_puls_1 = _grounded_leg_pos.at((i + 1) % _grounded_leg_pos.size()) - _grounded_leg_pos.at(i);
//		i_to_i_puls_1.z = 0.0f;
//
//		//i番目の脚先からロボットの重心へ向かうベクトル．
//		designlab::Vector3 i_to_com = COMPOSI - _grounded_leg_pos.at(i);
//		i_to_com.z = 0.0f;
//
//		//脚がクロスする場合
//		if (i_to_i_puls_1.Cross(i_to_com).z > 0.0f)
//		{
//			return false;
//		}
//	}
//
//	return true;
//}
//
//void Hexapod::makeLegROM_r()
//{
//	// 逆運動学coxaなしの計算結果を用いて準運動学を計算する
//	using namespace dl_math;
//
//	for (int i = 0; i < 200; i++) { m_leg_max_r[i] = 0; }
//
//	const float PERMISSION = 0.5f;			//逆運動学と運動学を行った結果が半径Permission^0.5の円の中なら等しいと考える
//
//	const float mins[3] = { -1.428f, -1.780f, -1.194f };	//脚可動範囲 おそらくrad 変換したやつ(-81.8° -101.98° -68.41°)  190527
//	const float maxs[3] = { 1.402f,  1.744f,  1.769f };	//左からcoxa,femur,tibia (80.32° 99.92° 101.36°)
//
//	//ans of kinematics use sorution of i_kinematics 
//
//	//zは最大196．ixは最大248
//	for (int iz = 0; iz < 200; iz++)
//	{
//		for (int ix = 53; ix < 248; ix++)
//		{
//			const designlab::Vector3 _LineEnd((float)ix, 0.0f, (float)iz);		//脚先座標（ローカル）
//
//			//逆運動学coxaなし
//
//			//const float _coxa_angle = atan2(_LineEnd.x, _LineEnd.y);	//coxa角度
//
//			const float _IK_trueX = sqrt(squared(_LineEnd.x) + squared(_LineEnd.y)) - HexapodConst::PHANTOMX_COXA_LENGTH;	//femurから足先までを結ぶベクトルをxy平面に投影したときのベクトルの大きさ
//			float _im = sqrt(squared(_IK_trueX) + squared(_LineEnd.z));					//絶対に正
//			if (_im == 0.0f) _im += 0.01f;	//0割り対策
//
//			const float _q1 = -atan2(_LineEnd.z, _IK_trueX);													//マイナスでおｋ座標系的にq1自体は常に負//xがゼロだと定義域エラー
//			const float _q2 = acos((squared(HexapodConst::PHANTOMX_FEMUR_LENGTH) + squared(_im) - squared(HexapodConst::PHANTOMX_TIBIA_LENGTH)) / (2.0f * HexapodConst::PHANTOMX_FEMUR_LENGTH * _im));	//im=0だと定義域エラー
//
//			const float _femur_angle = _q1 + _q2;
//			const float _tibia_angle = acos((squared(HexapodConst::PHANTOMX_FEMUR_LENGTH) + squared(HexapodConst::PHANTOMX_TIBIA_LENGTH) - squared(_im)) / (2.0f * HexapodConst::PHANTOMX_FEMUR_LENGTH * HexapodConst::PHANTOMX_TIBIA_LENGTH)) - dl_math::MY_FLT_PI / 2.0f;
//
//			//float im = sqrt(pow(fabs(IK_trueX), 2.0) + pow(fabs(LineEnd.z), 2.0));//femurから足先の距離
//			//float d1 = pow((float)L_FEMUR, 2.0) - pow((float)L_TIBIA, 2.0) + pow(fabs((float)im), 2.0);
//			//float d2 = 2 * L_FEMUR*im;
//			//float q2 = acos((float)d1 / (float)d2);	//余弦定理
//			//d1 = pow((float)L_FEMUR, 2.0) - pow(fabs((float)im), 2.0) + pow((float)L_TIBIA, 2.0);
//			//d2 = 2 * L_TIBIA*L_FEMUR;
//			//tibia = acos((float)d1 / (float)d2) - 1.570796326795;
//
//			//lange of motion
//			//実機はわからんが、シミュレーションだと、これがいらない。
//			//if文入れると、重心と足先高さの差が、73mm以下は取れない。hato
//			//if ( femur < femurMins)break;
//			//if (femurMaxs < femur)break;
//			//if (tibia < tibiaMins)break;
//			//if(tibiaMaxs < tibia )break;
//
//			//運動学
//			const float _K_trueX = HexapodConst::PHANTOMX_FEMUR_LENGTH * cos(_femur_angle) + HexapodConst::PHANTOMX_TIBIA_LENGTH * cos(_femur_angle + _tibia_angle - dl_math::MY_FLT_PI / 2.0f);
//
//			designlab::Vector3 _kinematics;
//			_kinematics.x = _K_trueX + HexapodConst::PHANTOMX_COXA_LENGTH;
//			_kinematics.y = 0.0f;
//			_kinematics.z = -(HexapodConst::PHANTOMX_FEMUR_LENGTH * sin(_femur_angle) + HexapodConst::PHANTOMX_TIBIA_LENGTH * sin(_femur_angle + _tibia_angle - dl_math::MY_FLT_PI / 2.0f));
//
//			const float _Permission = (_kinematics - _LineEnd).GetSquaredLength();
//
//			if (PERMISSION > _Permission)
//			{
//				m_leg_max_r[iz] = ix - LEGROM_RMARGIN;//y=0のとき，脚高さzのときのx方向の最大の範囲
//
//#ifdef  MAX_LEG_RADIUS
//				if (iz <= 115) { m_leg_max_r[iz] = MAX_LEG_RADIUS; }//脚を置く位置が遠すぎるとトルクが足りなくて沈み込みが激しいから200までにした2020/11/09hato
//#endif
//			}
//		}
//	}
//}
//
//void Hexapod::calculateRangeOfMovement(const int _legnum, designlab::Vector3& p1, designlab::Vector3& p2) const
//{
//	//脚関節の基準角度（ロボット座標系） -180~180度内であること
//	float coxaDefoAngle[HexapodConst::kLegNum] = { 45.0f, 0.0f, -45.0f, -135.0f, 180.0f, 135.0f };
//
//	for (int i = 0; i < HexapodConst::kLegNum; ++i)
//	{
//		//[rad]に変換
//		coxaDefoAngle[i] = coxaDefoAngle[i] * dl_math::MY_FLT_PI / 180.0f;
//	}
//
//	const float rangeOfAngle = 40.0f * dl_math::MY_FLT_PI / 180.0f;	//[rad] 脚の可動範囲 今は基準角度より±40度に動くと仮定
//	//float r = m_leg_max_r[int(this->ziki.com.z)];	//現在の胴体高さにおける脚到達距離　これは、地面がz=0にあることを前提としているからダメ。20200618
//	//↑グローバルで胴体高さ196mm以上とかになったら、バグる原因
//	//とりあえず網羅することが大事。実際にとれるかどうかは、checktouchdownpointで脚設置可能点の高さを考慮して判定しているから。
//	float r = 240.0f;// m_leg_max_r[0]; //0だと237indexを73くらいにすると、r=223くらい。後々はインデックスは、接地面座標系で見た重心と脚先高さの差にしなければならない。
//	//rは脚設置可能点を参照する領域の大きさの要素。rの値によって挙動は変化する。
//
//	//t1:その脚における最小角度 t2:最大角度
//	float t1 = coxaDefoAngle[_legnum] - rangeOfAngle + this->ziki.thY;	//[rad]
//	float t2 = coxaDefoAngle[_legnum] + rangeOfAngle + this->ziki.thY;
//	if ((t1 < -2.0f * dl_math::MY_FLT_PI) || (t1 > 2.0f * dl_math::MY_FLT_PI)) { t1 = fmod(t1, 2.0f * dl_math::MY_FLT_PI); }	//-2pi~2pに
//
//	if (t1 < -dl_math::MY_FLT_PI) { t1 = t1 + 2.0f * dl_math::MY_FLT_PI; }	//-piより小さければ
//
//	if (t2 > dl_math::MY_FLT_PI) { t2 = t2 - 2.0f * dl_math::MY_FLT_PI; }
//
//	//初期化
//	p1 = designlab::Vector3(0.0f, 0.0f, 0.0f);
//	p2 = designlab::Vector3(0.0f, 0.0f, 0.0f);
//
//	//p1p2の座標決定
//	if ((r * cos(t1)) > p2.x)
//	{
//		p2.x = r * cos(t1);
//	}
//	else if ((r * cos(t1)) < p1.x)
//	{
//		p1.x = r * cos(t1);
//	}
//
//	if ((r * sin(t1)) > p2.y)
//	{
//		p2.y = r * sin(t1);
//	}
//	else if ((r * sin(t1)) < p1.y)
//	{
//		p1.y = r * sin(t1);
//	}
//
//	if ((r * cos(t2)) > p2.x)
//	{
//		p2.x = r * cos(t2);
//	}
//	else if ((r * cos(t2)) < p1.x)
//	{
//		p1.x = r * cos(t2);
//	}
//
//	if ((r * sin(t2)) > p2.y)
//	{
//		p2.y = r * sin(t2);
//	}
//	else if ((r * sin(t2)) < p1.y)
//	{
//		p1.y = r * sin(t2);
//	}
//
//	//90 180 -90 -180度の時の処理
//	if ((t2 > dl_math::MY_FLT_PI / 4.0f && t1 < dl_math::MY_FLT_PI / 4.0f) || (t1 > 0.0f && t1 < dl_math::MY_FLT_PI / 4.0f && t2 < -dl_math::MY_FLT_PI / 4.0f)) { p2.y = r; }//r*sin(90)
//	if (t2 > 0.0f && t1 < 0.0f) { p2.x = r; }															//r*con(0)
//	if ((t2 > -dl_math::MY_FLT_PI / 4.0f && t1 < -dl_math::MY_FLT_PI / 4.0f) || (t1 > dl_math::MY_FLT_PI / 4.0f && t2 < 0.0f && t2 > -dl_math::MY_FLT_PI / 4.0f)) { p1.y = -r; }//r*sin(-90)
//	if (t2 < t1) { p1.x = -r; }																			//r*cos(180)
//
//	//グローバル座標に変換
//	p1 = p1 + getGlobalCoxaJointPos(_legnum);
//	p2 = p2 + getGlobalCoxaJointPos(_legnum);
//}
//
//void Hexapod::setMyDirection(const float _thP, const float _thR, const float _thY)
//{
//	this->ziki.thP = _thP;
//	this->ziki.thR = _thR;
//	this->ziki.thY = _thY;
//}
//
//void Hexapod::setLocalLeg2Pos(const designlab::Vector3 posi2[HexapodConst::kLegNum])
//{
//	for (int i = 0; i < HexapodConst::kLegNum; i++)
//	{
//		this->m_local_leg2_pos[i] = posi2[i];
//	}
//}
//
//void Hexapod::setLocalLegPos(const designlab::Vector3 posi[HexapodConst::kLegNum])
//{
//	for (int i = 0; i < HexapodConst::kLegNum; i++)
//	{
//		this->m_local_leg_pos[i] = posi[i];
//	}
//
//	if (setJointPosi())
//	{
//		/*std::cout<<"Error anable leg_pos posi"<<std::endl;*//*std::cout<<hexapod::showGlobalMyDirectionthY()<<std::endl;std::string stop; std::cin>>stop;*/
//	}
//}
//
//designlab::Vector3 Hexapod::getGlobalLeg2Pos(const int _leg_num) const
//{
//	//グローバル座標を返す
//	designlab::Vector3 rotatePosition_of_2 = rotation(m_local_leg2_pos[_leg_num], designlab::Vector3(0, 0, 0), this->ziki.thP, this->ziki.thR, this->ziki.thY);
//	designlab::Vector3 ans = rotatePosition_of_2 + getGlobalCoxaJointPos(_leg_num);
//	return ans;
//}
//
//designlab::Vector3 Hexapod::getGlobalLegPos(const int _leg_num) const
//{
//	//グローバル座標を返す
//	designlab::Vector3 rotateLegPosition = rotation(m_local_leg_pos[_leg_num], designlab::Vector3(0, 0, 0), this->ziki.thP, this->ziki.thR, this->ziki.thY);
//	//重心から脚の付け根+付け根から脚先
//	designlab::Vector3 ans = rotateLegPosition + getGlobalCoxaJointPos(_leg_num);
//	return ans;
//}
//
//designlab::Vector3 Hexapod::getGlobalCoxaJointPos(const int _leg_num) const
//{
//	//グローバル座標を返す(重心位置から脚の付け根の計算)
//	designlab::Vector3 rotateCoxaJointPosi = rotation(this->m_local_coxajoint_pos[_leg_num], designlab::Vector3(0, 0, 0), this->ziki.thP, this->ziki.thR, this->ziki.thY);
//	designlab::Vector3 ans = rotateCoxaJointPosi + this->ziki.com;
//	return ans;
//}
//
//designlab::Vector3 Hexapod::getGlobalFemurJointPos(const int _leg_num) const
//{
//	//グローバル座標を返す
//	designlab::Vector3 rotateFemurJointPosi = rotation(this->m_local_femurjoint_pos[_leg_num], designlab::Vector3(0, 0, 0), this->ziki.thP, this->ziki.thR, this->ziki.thY);
//	designlab::Vector3 ans = rotateFemurJointPosi + this->ziki.com;
//	return ans;
//}
//
//designlab::Vector3 Hexapod::getGlobalTibiaJointPos(const int _leg_num) const
//{
//	//グローバル座標を返す
//	designlab::Vector3 rotateTibiaJointPosi = rotation(this->m_local_tibiajoint_pos[_leg_num], designlab::Vector3(0, 0, 0), this->ziki.thP, this->ziki.thR, this->ziki.thY);
//	designlab::Vector3 ans = rotateTibiaJointPosi + this->ziki.com;
//	return ans;
//}