#include "hexapod.h"
#include <iostream>
#include "Define.h"

Hexapod::Hexapod()
{
	//脚の付け根座標代入L_CoxaJoint_posi m_local_coxajoint_pos
	m_local_coxajoint_pos[0] = myvector::VGet(FWIDTH, FLENGTH, 0);
	m_local_coxajoint_pos[1] = myvector::VGet(MWIDTH, 0, 0);
	m_local_coxajoint_pos[2] = myvector::VGet(RWIDTH, -RLENGTH, 0);
	m_local_coxajoint_pos[3] = myvector::VGet(-RWIDTH, -RLENGTH, 0);
	m_local_coxajoint_pos[4] = myvector::VGet(-MWIDTH, 0, 0);
	m_local_coxajoint_pos[5] = myvector::VGet(-FWIDTH, FLENGTH, 0);
}

//ベクトルの3Dローテーション オイラー角（テイト・ブライアン角）Y-X-Z
// 2018.12.27 コメントのY-X-Zって間違いっぽい．英語版wikipediaに詳しいが，多分Y-X-Yあたりのオイラー角の定義だと思うんですが……（テイト・ブライアンも違うのでは）
//2020.05.15.hato
//　これは、RPYが基本的に胴体の傾き（別に胴体じゃなくてもいいけど）だから、胴体座標系やcoxa座標系から座標ΣP（例えばcoxaからみた脚先座標など）を
//グローバル座標で見た座標Σ0Pにへんかんするための、
//	Σ0_P = （Σ0_R_Σ)ΣP + (グローバル座標系で見たその座標系の原点座標）
//っていう式の（Σ0_R_Σ)ΣPの計算をしてる関数。
//
//（Σ0RΣ)は回転行列で（Σ0RΣ)=Ry*Rx*Rz
//Ry,Rx,Rzは添え字の軸回りに回転させたときの回転行列をあらわす。
//一般的には、進行方向にｘ軸をとるから、
//Rxがロールの関数＝Rx(thR), Ryがピッチの関数=Ry(thP),Pz がヨーの関数=Rz(thY)
//になる。
//ただ、このシミュレーションでは、進行方向がy軸だから、
//Rxがピッチの関数＝Rx(thR), Ryがロールの関数=Ry(thP),Pz がヨーの関数=Rz(thY)
//になっている。
//また、Σ0RΣ　＝　R＊P＊Yの順に回転行列を計算するのが一般的？だから、それにあわせると、
//普通は、Σ0RΣ　＝　Rx(R) * Ry(P) * Rz(Y)　の順番で計算するところ
//ここでは、Σ0RΣ　＝　Ry(R) * Rx(P) * Rz(Y)　の順番で計算している。
//そこがちょっとややこしいところだけど、計算自体は間違ってない。
myvector::SVector Hexapod::rotation(myvector::SVector In, myvector::SVector center, double thP, double thR, double thY) 
{
	myvector::SVector ans, buf;
	buf = subVec(In, center);
	ans.x = (cos(thR) * cos(thY) + sin(thR) * sin(thP) * sin(thY)) * buf.x +
		(cos(thY) * sin(thR) * sin(thP) - cos(thR) * sin(thY)) * buf.y +
		(cos(thP) * sin(thR)) * buf.z;

	ans.y = cos(thP) * sin(thY) * buf.x +
		cos(thP) * cos(thY) * buf.y +
		(-sin(thP)) * buf.z;

	ans.z = (cos(thR) * sin(thP) * sin(thY) - cos(thY) * sin(thR)) * buf.x +
		(cos(thR) * cos(thY) * sin(thP) + sin(thR) * sin(thY)) * buf.y +
		(cos(thR) * cos(thP)) * buf.z;

	return myvector::addVec(ans, center);
}

void Hexapod::setMyDirection(const double _thP, const double _thR, const double _thY) 
{
	this->ziki.thP = _thP;
	this->ziki.thR = _thR;
	this->ziki.thY = _thY;
}



void Hexapod::setTarget(const STarget _target) 
{
	this->m_target = _target;
}

void Hexapod::setPosition_of_2(const myvector::SVector posi2[Define::LEG_NUM])
{
	for (int i = 0; i < Define::LEG_NUM; i++)
	{
		this->m_local_leg2_pos[i] = posi2[i];
	}
}

void Hexapod::setMyLegPosition(const myvector::SVector posi[Define::LEG_NUM])
{
	for (int i = 0; i < Define::LEG_NUM; i++)
	{
		this->m_local_leg_pos[i] = posi[i];
	}
	
	if (setJointPosi()) 
	{
		/*std::cout<<"Error anable Leg posi"<<std::endl;*//*std::cout<<hexapod::showGlobalMyDirectionthY()<<std::endl;std::string stop; std::cin>>stop;*/ 
	}
}

myvector::SVector Hexapod::getGlobalLeg2Pos(const int _leg_num)
{	
	//グローバル座標を返す
	myvector::SVector ans, rotatePosition_of_2;
	rotatePosition_of_2 = rotation(m_local_leg2_pos[_leg_num], myvector::VGet(0, 0, 0), this->ziki.thP, this->ziki.thR, this->ziki.thY);
	ans = myvector::addVec(rotatePosition_of_2, getGlobalCoxaJointPos(_leg_num));
	return ans;
}

myvector::SVector Hexapod::getGlobalLegPos(const int _leg_num)
{	
	//グローバル座標を返す
	myvector::SVector ans, rotateLegPosition;
	rotateLegPosition = rotation(m_local_leg_pos[_leg_num], myvector::VGet(0, 0, 0), this->ziki.thP, this->ziki.thR, this->ziki.thY);
	//重心から脚の付け根+付け根から脚先
	ans = myvector::addVec(rotateLegPosition, getGlobalCoxaJointPos(_leg_num));
	return ans;
}

myvector::SVector Hexapod::getGlobalCoxaJointPos(const int _leg_num)
{
	//グローバル座標を返す(重心位置から脚の付け根の計算)
	myvector::SVector ans, rotateCoxaJointPosi;
	rotateCoxaJointPosi = rotation(this->m_local_coxajoint_pos[_leg_num], myvector::VGet(0, 0, 0), this->ziki.thP, this->ziki.thR, this->ziki.thY);
	ans = myvector::addVec(rotateCoxaJointPosi, this->ziki.com);
	return ans;
}

myvector::SVector Hexapod::getGlobalFemurJointPos(const int _leg_num)
{
	//グローバル座標を返す
	myvector::SVector ans, rotateFemurJointPosi;
	rotateFemurJointPosi = rotation(this->m_local_femurjoint_pos[_leg_num], myvector::VGet(0, 0, 0), this->ziki.thP, this->ziki.thR, this->ziki.thY);
	ans = myvector::addVec(rotateFemurJointPosi, this->ziki.com);
	return ans;
}

myvector::SVector Hexapod::getGlobalTibiaJointPos(const int _leg_num)
{
	//グローバル座標を返す
	myvector::SVector ans, rotateTibiaJointPosi;
	rotateTibiaJointPosi = rotation(this->m_local_tibiajoint_pos[_leg_num], myvector::VGet(0, 0, 0), this->ziki.thP, this->ziki.thR, this->ziki.thY);
	ans = myvector::addVec(rotateTibiaJointPosi, this->ziki.com);
	return ans;
}

bool Hexapod::setJointPosi() 
{
	// 逆運動学の計算結果を用いて準運動学を計算する．逆運動学，運動学の計算から算術する方式 ロボット固有

	//逆運動学と運動学を行った結果が半径Permission^0.5の円の中なら等しいと考える
	const double PERMISSION = 1;			

	const double mins[3] = { -1.428, -1.780, -1.194 };  //脚可動範囲
	const double maxs[3] = { 1.402,  1.744,  1.769 };  //左からcoxa,femur,tibia
	const double coxaMins[6] = { -0.610, -1.428, -2.213, -0.960, -1.745, -2.531 };
	const double coxaMaxs[6] = { 2.187,   1.402,  0.617,  2.531,  1.745,  0.960 };

	const double femurMins = -1.780;
	const double femurMaxs = 1.744;
	const double tibiaMins = -1.194;
	const double tibiaMaxs = 1.769;

	double coxa, femur, tibia;
	myvector::SVector kinematics;					//ans of kinematics use sorution of i_kinematics
	myvector::SVector legposi;

	for (int legNum = 0; legNum < Define::LEG_NUM; legNum++) 
	{
		legposi = myvector::VGet(m_local_leg_pos[legNum].x, m_local_leg_pos[legNum].y, -m_local_leg_pos[legNum].z);

		//逆運動学
		// first, make this a 2DOF problem... by solving coxa
		coxa = atan2(legposi.y, legposi.x);//coxa角度
		double IK_trueX = sqrt(pow(fabs(legposi.x), 2.0) + pow(fabs(legposi.y), 2.0)) - L_COXA;//xy平面におけるfemur脚先までの距離
		double im = sqrt(pow(fabs(IK_trueX), 2.0) + pow(fabs(legposi.z), 2.0));//femurから脚先までの距離

		// get femur angle above horizon...
		double q1 = -atan2(legposi.z, IK_trueX);//femurから脚先へのベクトルとxy平面との角度
		double d1 = pow((double)L_FEMUR, 2.0) - pow((double)L_TIBIA, 2.0) + pow(fabs((double)im), 2.0);
		double d2 = 2 * L_FEMUR * im;
		double q2 = acos((double)d1 / (double)d2);
		femur = q1 + q2;

		// and tibia angle from femur...
		d1 = pow((double)L_FEMUR, 2.0) - pow(fabs((double)im), 2.0) + pow((double)L_TIBIA, 2.0);
		d2 = 2 * L_TIBIA * L_FEMUR;
		tibia = acos((double)d1 / (double)d2) - 1.57;

		//運動学
		double K_trueX;

		K_trueX = L_FEMUR * cos(femur) + L_TIBIA * cos(femur + tibia - 1.57);
		kinematics.x = cos(coxa) * (K_trueX + L_COXA);
		kinematics.y = sin(coxa) * (K_trueX + L_COXA);
		kinematics.z = -(L_FEMUR * sin(femur) + L_TIBIA * sin(femur + tibia - 1.57));


		m_local_femurjoint_pos[legNum] = myvector::addVec(m_local_coxajoint_pos[legNum], myvector::VGet(L_COXA * cos(coxa), L_COXA * sin(coxa), 0));
		m_local_tibiajoint_pos[legNum] = myvector::addVec(m_local_femurjoint_pos[legNum], myvector::VGet(L_FEMUR * cos(femur) * cos(coxa), L_FEMUR * cos(femur) * sin(coxa), L_FEMUR * sin(femur)));

		double Permission = VSquareSize(subVec(kinematics, legposi));

		if (PERMISSION < Permission) { std::cout << "Error PERMISSION OVER \n"; return 1; };
	}

	return 0;
}

bool Hexapod::check_touchdown_point(int legNum, const myvector::SVector& LineEnd)
{
	//可動域が80,60,55,50,45,35,30degのパターンもかつてはあり，コメントアウトされていたが，全て削除して整理した．

	//可動域40													
	const	double	LCoxaJointMinsCos[6] = { 0.996194698f,	0.766044443f,	0.087155743f,	-0.996194698f,	-0.766044443f,	-0.087155743f };
	const	double	LCoxaJointMaxsCos[6] = { 0.087155743f,	0.766044443f,	0.996194698f,	-0.087155743f,	-0.766044443f,	-0.996194698f };
	const	double	LCoxaJointMinsSin[6] = { 0.087155743f,	-0.64278761f,	-0.996194698f,	-0.087155743f,	0.64278761f,	0.996194698f };
	const	double	LCoxaJointMaxsSin[6] = { 0.996194698f,	0.64278761f,	-0.087155743f,	-0.996194698f,	-0.64278761f,	0.087155743f };

	double crossMinIn;
	double crossMaxIn;

	crossMinIn = LCoxaJointMinsCos[legNum] * LineEnd.x - LCoxaJointMinsSin[legNum] * LineEnd.y; //外積,脚位置が扇形のスタート位置より<180°なら+,>180°なら-
	crossMaxIn = LCoxaJointMaxsCos[legNum] * LineEnd.x - LCoxaJointMaxsSin[legNum] * LineEnd.y;

	//扇形の角度が180以上なら符号が逆になる
	if (crossMinIn < 0 || crossMaxIn > 0) { return false; }

	myvector::SVector xy_LineEnd;
	xy_LineEnd = LineEnd;
	xy_LineEnd.z = 0;

	// LegROM_r その胴体高さにおける脚の到達半径の長さ
	double delta = VMag(xy_LineEnd);//X0Y0平面に投影した脚の根元から脚先までの半径

	//付け根からxy平面の距離<胴体高さに対する可動範囲の半径//胴体から近い場合は体勢がきついので使用しない20180312
	if (MIN_LEG_RADIUS < delta && delta < LegROM_r[int(LineEnd.z)])
	{
		return true;
	}

	return false;
}

bool Hexapod::check_touchdown_point2(int legNum, const myvector::SVector& LineEnd, const double delta_z) 
{
	if (int(delta_z) < MIN_DELTAZ || MAX_DELTAZ < int(delta_z)) { return 0; }

	//可動域が80,60,55,50,45,35,30degのパターンもかつてはあり，コメントアウトされていたが，全て削除して整理した．

	//可動域40													
	const	double	LCoxaJointMinsCos[6] = { 0.996194698	,	0.766044443	,	0.087155743	,	-0.996194698	,	-0.766044443	,	-0.087155743 };
	const	double	LCoxaJointMaxsCos[6] = { 0.087155743	,	0.766044443	,	0.996194698	,	-0.087155743	,	-0.766044443	,	-0.996194698 };
	const	double	LCoxaJointMinsSin[6] = { 0.087155743	,	-0.64278761	,	-0.996194698	,	-0.087155743	,	0.64278761	,	0.996194698 };
	const	double	LCoxaJointMaxsSin[6] = { 0.996194698	,	0.64278761	,	-0.087155743	,	-0.996194698	,	-0.64278761	,	0.087155743 };

	double crossMinIn;
	double crossMaxIn;

	//外積,脚位置が扇形のスタート位置より<180°なら+,>180°なら-
	crossMinIn = LCoxaJointMinsCos[legNum] * LineEnd.x - LCoxaJointMinsSin[legNum] * LineEnd.y; 
	crossMaxIn = LCoxaJointMaxsCos[legNum] * LineEnd.x - LCoxaJointMaxsSin[legNum] * LineEnd.y;

	//扇形の角度が180以上なら符号が逆になる
	if (crossMinIn < 0 || crossMaxIn > 0) { return false; }

	myvector::SVector xy_LineEnd;
	xy_LineEnd = LineEnd;
	xy_LineEnd.z = 0;
	
	// LegROM_r その胴体高さにおける脚の到達半径の長さ
	double delta = VMag(xy_LineEnd);//X0Y0平面に投影した脚の根元から脚先までの半径

	//付け根からxy平面の距離<胴体高さに対する可動範囲の半径//胴体から近い場合は体勢がきついので使用しない20180312
	if (MIN_LEG_RADIUS < delta && delta < LegROM_r[int(delta_z)])
	{
		//重心高さとある脚設置可能点の高さの差に応じて、許容する半径が変わる。
		//今のLegROM_rの計算じゃ73mmより上にあげると探索不可能　後々はインデックスは、接地面座標系で見た重心と脚先高さの差にしなければならない。
		return true;
	}

	return false;
}

bool Hexapod::isAblePause(myvector::SVector* leg, const bool groundingLeg[Define::LEG_NUM])
{
	for (int i = 0; i < Define::LEG_NUM; i++)
	{
		if (groundingLeg[i] == true)
		{
			if (check_touchdown_point(i, leg[i]) == 0)
			{
				return false;
			}
		}
	}

	return true;
}

bool Hexapod::isAbleCOM(myvector::SVector* L_leg, const bool groundingLeg[Define::LEG_NUM])
{
	int landingLegInd[7] = { 0 };
	int landingLegCnt = 0;

	for (int i = 0; i < Define::LEG_NUM; i++)
	{
		if (groundingLeg[i] == true) { landingLegInd[landingLegCnt++] = i; }
	}

	landingLegInd[landingLegCnt++] = landingLegInd[0];

	myvector::SVector G_CoxaJointPosi[6];

	for (int i = 0; i < 6; i++) 
	{
		G_CoxaJointPosi[i] = getGlobalCoxaJointPos(i);
	}

	for (int i = 0; i < landingLegCnt - 1; i++) 
	{
		//n番目の脚位置を基としてn+1番脚までのベクトルと重心(0,0,0)の位置を比べて常に右にあったらok
		myvector::SVector n_np1, n_g;//landingLegInd[n]番脚からlandingLegInd[n+1]番脚へのヴェクタ―，landingLegInd[n]番脚から重心へのヴェクタ―

		//x→y,y→x
		myvector::SVector x_y_changedL_leg[6];
		for (int iLeg = 0; iLeg < Define::LEG_NUM; iLeg++) 
		{
			x_y_changedL_leg[iLeg] = myvector::VGet(L_leg[iLeg].y, L_leg[iLeg].x, L_leg[iLeg].z); 
		}

		//n番目の脚先からn+1へのベクトル
		n_np1 = myvector::subVec(myvector::addVec(G_CoxaJointPosi[landingLegInd[i + 1]], x_y_changedL_leg[landingLegInd[i + 1]]), myvector::addVec(G_CoxaJointPosi[landingLegInd[i]], x_y_changedL_leg[landingLegInd[i]]));
		n_np1.z = 0;

		//n番目の脚先からロボットの重心方向のベクトル
		n_g = myvector::subVec(COMPOSI, myvector::addVec(G_CoxaJointPosi[landingLegInd[i]], x_y_changedL_leg[landingLegInd[i]]));
		n_g.z = 0;

		if (myvector::VCross(n_np1, n_g).z > 0) 
		{
			return false;//脚がクロスする場合
		}
	}

	return true;
}

void Hexapod::makeLegROM_r()
{
	// 逆運動学coxaなしの計算結果を用いて準運動学を計算する

	for (int i = 0; i < 200; i++) { LegROM_r[i] = 0; }

	const double PERMISSION = 0.5;			//逆運動学と運動学を行った結果が半径Permission^0.5の円の中なら等しいと考える

	const double mins[3] = { -1.428, -1.780, -1.194 };  //脚可動範囲 おそらくrad 変換したやつ(-81.8° -101.98° -68.41°)  190527
	const double maxs[3] = { 1.402,  1.744,  1.769 };  //左からcoxa,femur,tibia (80.32° 99.92° 101.36°)

	const double femurMins = -1.780;
	const double femurMaxs = 1.744;
	const double tibiaMins = -1.194;
	const double tibiaMaxs = 1.769;

	double coxa, femur, tibia;
	myvector::SVector kinematics;
	myvector::SVector LineEnd;//脚先座標（ローカル）

	//ans of kinematics use sorution of i_kinematics 

	//zは最大196．ixは最大248．
	for (int iz = 0; iz < 200; iz++) 
	{
		for (int ix = 53; ix < 248; ix++) 
		{
			LineEnd.x = ix;
			LineEnd.y = 0;
			LineEnd.z = iz;

			//逆運動学coxaなし
			// first, make this a 2DOF problem... by solving coxa

			coxa = atan2(LineEnd.x, LineEnd.y);	//coxa角度
			double IK_trueX = sqrt(pow(fabs(LineEnd.x), 2.0) + pow(fabs(LineEnd.y), 2.0)) - L_COXA;//femurから足先までを結ぶベクトルをxy平面に投影したときのベクトルの大きさ
			double im = sqrt(IK_trueX * IK_trueX + LineEnd.z * LineEnd.z);//絶対に正//femurから足先の距離

			// get femur angle above horizon...
			double q1 = -atan2(LineEnd.z, IK_trueX);//マイナスでおｋ座標系的にq1自体は常に負//xがゼロだと定義域エラー
			//double d1 = pow((double)L_FEMUR, 2.0) - pow((double)L_TIBIA, 2.0) + pow(fabs((double)im), 2.0);
			double q2 = acos((L_FEMUR * L_FEMUR + im * im - L_TIBIA * L_TIBIA) / (2 * L_FEMUR * im));//im=0だと定義域エラー
			//double d2 = 2 * L_FEMUR*im;
			//double q2 = acos((double)d1 / (double)d2);	//余弦定理
			femur = q1 + q2;//ok

			// and tibia angle from femur...
			//d1 = pow((double)L_FEMUR, 2.0) - pow(fabs((double)im), 2.0) + pow((double)L_TIBIA, 2.0);
			//d2 = 2 * L_TIBIA*L_FEMUR;

			//tibia = acos((double)d1 / (double)d2) - 1.570796326795;//ok
			tibia = acos((L_FEMUR * L_FEMUR + L_TIBIA * L_TIBIA - im * im) / (2 * L_FEMUR * L_TIBIA)) - 1.570796326795;//ok
			//lange of motion
			//実機はわからんが、シミュレーションだと、これがいらない。
			//if文入れると、重心と足先高さの差が、73mm以下は取れない。hato
			//if ( femur < femurMins)break;
			//if (femurMaxs < femur)break;
			//if (tibia < tibiaMins)break;
			//if(tibiaMaxs < tibia )break;

			//運動学
			double K_trueX;

			K_trueX = L_FEMUR * cos(femur) + L_TIBIA * cos(femur + tibia - 1.570796326795);
			//kinematics.x = sin(coxa) * (K_trueX + L_COXA);	//coxaの求めかたが変なため，ここでsinになってる
			kinematics.x = K_trueX + L_COXA;
			//kinematics.y = cos(coxa) * (K_trueX + L_COXA); //要はゼロ
			kinematics.y = 0;
			kinematics.z = -(L_FEMUR * sin(femur) + L_TIBIA * sin(femur + tibia - 1.570796326795));//おｋ

			double Permission = VSquareSize(subVec(kinematics, LineEnd));

			if (PERMISSION > Permission) 
			{
				LegROM_r[iz] = ix - LEGROM_RMARGIN;//y=0のとき，脚高さzのときのx方向の最大の範囲
#ifdef  MAX_LEG_RADIUS
				if (iz <= 115) LegROM_r[iz] = MAX_LEG_RADIUS;//脚を置く位置が遠すぎるとトルクが足りなくて沈み込みが激しいから200までにした2020/11/09hato
				   //if (ix < LegROM_rmin[iz]) LegROM_rmin[iz] = ix;	//rminを求めたかったけどいらないかな190606
#endif
			}
		}
	}
}

void Hexapod::calculateRangeOfMovement(int legnum, myvector::SVector& p1, myvector::SVector& p2) 
{
	//現在の位置と向きにおける脚到達範囲を返す p1:最小角度の時の座標 p2:最大角度の時の座標 ピッチ・ロール回転は考慮していない 扇形であること前提
	double coxaDefoAngle[6] = { 45.0, 0.0, -45.0, -135.0, 180, 135.0 };	//脚関節の基準角度（ロボット座標系） -180~180度内であること
	for (int i = 0; i < 6; ++i)coxaDefoAngle[i] = coxaDefoAngle[i] * Define::MY_PI / 180.0;	//[rad]に変換
	double rangeOfAngle = 40.0 * Define::MY_PI / 180.0;	//[rad] 脚の可動範囲 今は基準角度より±40度に動くと仮定
	//double r = LegROM_r[int(this->ziki.com.z)];	//現在の胴体高さにおける脚到達距離　これは、地面がz=0にあることを前提としているからダメ。20200618
	//↑グローバルで胴体高さ196mm以上とかになったら、バグる原因
	//とりあえず網羅することが大事。実際にとれるかどうかは、checktouchdownpointで脚設置可能点の高さを考慮して判定しているから。
	double r = 240;// LegROM_r[0]; //0だと237indexを73くらいにすると、r=223くらい。後々はインデックスは、接地面座標系で見た重心と脚先高さの差にしなければならない。
	//rは脚設置可能点を参照する領域の大きさの要素。rの値によって挙動は変化する。
	double t1, t2;	//t1:その脚における最小角度 t2:最大角度

	t1 = coxaDefoAngle[legnum] - rangeOfAngle + this->ziki.thY;	//[rad]
	t2 = coxaDefoAngle[legnum] + rangeOfAngle + this->ziki.thY;
	if ((t1 < -2 * Define::MY_PI) || (t1 > 2 * Define::MY_PI)) t1 = fmod(t1, 2 * Define::MY_PI);	//-2pi~2pに
	if (t1 < -Define::MY_PI) t1 = t1 + 2 * Define::MY_PI;	//-piより小さければ
	if (t2 > Define::MY_PI) t2 = t2 - 2 * Define::MY_PI;

	p1 = myvector::VGet(0, 0, 0); //初期化
	p2 = myvector::VGet(0, 0, 0);

	//p1p2の座標決定
	if ((r * cos(t1)) > p2.x) 
	{
		p2.x = r * cos(t1);
	}
	else if ((r * cos(t1)) < p1.x) 
	{
		p1.x = r * cos(t1);
	}

	if ((r * sin(t1)) > p2.y) 
	{
		p2.y = r * sin(t1);
	}
	else if ((r * sin(t1)) < p1.y) 
	{
		p1.y = r * sin(t1);
	}

	if ((r * cos(t2)) > p2.x) 
	{
		p2.x = r * cos(t2);
	}
	else if ((r * cos(t2)) < p1.x) 
	{
		p1.x = r * cos(t2);
	}

	if ((r * sin(t2)) > p2.y) 
	{
		p2.y = r * sin(t2);
	}
	else if ((r * sin(t2)) < p1.y) 
	{
		p1.y = r * sin(t2);
	}

	//90 180 -90 -180度の時の処理
	if ((t2 > Define::MY_PI / 4 && t1 < Define::MY_PI / 4) || (t1 > 0 && t1 < Define::MY_PI / 4 && t2 < -Define::MY_PI / 4)) { p2.y = r; }//r*sin(90)
	if (t2 > 0 && t1 < 0) { p2.x = r; }																	//r*con(0)
	if ((t2 > -Define::MY_PI / 4 && t1 < -Define::MY_PI / 4) || (t1 > Define::MY_PI / 4 && t2 < 0 && t2 > -Define::MY_PI / 4)) { p1.y = -r; }//r*sin(-90)
	if (t2 < t1) { p1.x = -r; }																			//r*cos(180)

	//グローバル座標に変換
	p1 = myvector::addVec(p1, this->getGlobalCoxaJointPos(legnum));
	p2 = myvector::addVec(p2, this->getGlobalCoxaJointPos(legnum));
}

