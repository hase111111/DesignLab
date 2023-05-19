#include "PassFinding.h"
#include "function_walking_pattern_generator.h"
#include "vectorFunc.h"
#include "Define.h"
#include "LegState.h"
#include <bitset>
#include <iostream>
#include <string>

using namespace LegState;

static const double  F_PI = 3.14159265358979323846264338327950288f;

extern int no_use_kaisou[DISCRETE_NUM][DISCRETE_NUM][DISCRETE_NUM][DISCRETE_NUM][DISCRETE_NUM][DISCRETE_NUM];

PassFinding::PassFinding(const int _route_num, const MapState* _p_map_state)
{
	m_route_max = _route_num;
	LastNodeNum = 0;				//route[m_route_max]に格納されているノードの数
	RET_DIST = 4;					//初回以降の探索で初期値にするノード
	DIST_MAX = 18;					//一回の探索で記録するノードの個数　出力にのみ使用
	initiHX2();
	initHX2();

	//重心位置候補を初期化する
	_CreateComCandidate.setStabilityMargin(STABILITYMARGIN);//安定余裕

	//SPLPを初期化する.着地可能地点のセットを行う
	_SearchPossibleLegPosition.SetLegGroundableCandidatePointMAX(1000);
	_SearchPossibleLegPosition.mp_MapState = _p_map_state;

	LegHeight = LEG_HIEGHT;		//脚を上げる高さ
}

PassFinding::~PassFinding(void)
{
	//動的配列の解放
	delete[] route;
}

void PassFinding::operator()()
{	
	route = new LNODE[m_route_max];	//動的配列の確保	デストラクタでdeleteしてる

	getOptimalRoute(m_search_mode,m_search_depth);	//幅優先探索

	if(m_evaluation_mode == ETargetMode::NONE)
	{
		//探索をresultRouteにコピーして，評価は行わないパターン．
		//P_F0で行われる　結果は全てresultRouteに保存されて，後でP_F1~P_F6に分配される

		//オーバーフローする前に緊急停止
		if(LastNodeNum > m_result_route_limit)
		{
			std::cout << "Error Overflow m_result_route_limit = " << LastNodeNum; 
			std::string r;
			std::cin >> r;
		}			

		std::cout<<"探索深さ1のノード数\t"<<LastNodeNum<<"\n";

		m_node_num = LastNodeNum;
		
		for(int i = 0; i <= LastNodeNum  ; i++)
		{
			resultRoute[i] = route[i];
		}

	}
	else if(m_evaluation_mode == ETargetMode::STRAIGHT_VECOTR || m_evaluation_mode == ETargetMode::STRAIGHT_POSITION)
	{
		//直進移動で評価
		std::cout<<"評価1\n";
		m_node_num = LastNodeNum;
		chooseMostSuitableSolution(resultRoute);

	}
	else if(m_evaluation_mode == ETargetMode::TURN_ON_SPOT_DIRECTION || m_evaluation_mode == ETargetMode::TURN_ON_SPOT_ANGLE)
	{
		//その場旋回で評価
		std::cout<<"評価2\n";
		std::cout<<"LastNodeNum = "<<LastNodeNum<<"\n";
		m_node_num = LastNodeNum;
		chooseMostSuitableSolutionSpin(*resultOfstream, resultRoute);

	}
	else if(m_evaluation_mode == ETargetMode::TURN_DIRECTION || m_evaluation_mode == ETargetMode::TURN_ANGLE)
	{
		//旋回方向で評価．これが現在使用されている関数である2023/04/28

		//注意事項：評価は末端ノード（探索深さが一番深いところにあるノード．現状(2019年)だと最大で5）のノードのみが評価される．
		//旋回動作の研究（並木さん）では，目標とする旋回軌道から一定距離離れたノードは除外する，というふうにしているがその判定は末端ノードでしか行われない．
		//そのため，末端ノードでは一定距離内に入っていたとしても，根ノードからその末端ノードに至る経路に含まれるノードが一定距離内に入っているか保証はない．
		//CCCの重心タイプごとの代表点をきめる部分でその除外をすれば解決するが、挙動は不明。20200606hato
		//そもそもgetOptimalLouteと
		m_node_num = LastNodeNum;
		chooseMostSuitableSolutionRotation(*resultOfstream, resultRoute);	
	}
	else
	{
		std::cout<<"TargetMode Error 評価できません\nプログラムを終了します\n";
	}
}

//ルート探索 //幅優先探索を行う
void PassFinding::getOptimalRoute(ESearchMode _search_type,int m_search_depth)
{	
	//根ノードは親へのポインタを持たないため，NULLならば根ノードとなる
	if(CurrentCondition[0].parent != NULL)
	{
		//CurrentCondition[0]が葉ノードならば
		std::cout << "PassFinding クラスの getOptimalRoute 関数でエラーが起きました" << std::endl;
		std::cout << "ERROR CurrentCondition[0].parent = " << CurrentCondition[0].parent << std::endl;
		std::cout << "この関数を終了します" << std::endl;
		return;
	}

	for (int i = 0; i < CurrentConditionNum; ++i)
	{
		//自身に割り当てられている初期ノードをrouteに保存　
		//マルチスレッドだとP_F0で探索深さ1まで探索を行い，その結果を各スレッドに分配しているため
		route[i] = CurrentCondition[i];

		if (route[i].parent != NULL) 
		{
			route[i].parent = &(route[0]);
		}
	}

	//vectorのsize()関数を使いたい，せめてsizeofを使ってくれ!
	LastNodeNum = CurrentConditionNum - 1;

	if(m_search_mode == ESearchMode::EACH_OPERATION)
	{
		//1動作ごとの探索
		for(int i = 0; i < m_search_depth; i++)
		{	
			//探索深さ毎に実行する
			searchGaitPattern();
		}
	}
	else
	{
		std::cout << "古い探索モードが指定されました．これらを使うことは現在推奨されておりません" <<  std::endl;
		std::cout << "この関数を終了します" << std::endl;
		return;
	}
}

//次の動作を探索
void PassFinding::searchGaitPattern()
{
	//探索ルール
	// A1: 先行研究のやり方
	// A2: 先行研究のやり方で，重心の上下動変更したやつ
	// B : 脚↑-重心↑-重心→-脚→- 脚↓のループ
	// C : 重心↑↓-脚↑↓-重心→←-脚→←-脚↑↓のループ
	// D : 重心上下なし-脚↑↓-重心→←-脚→←-脚↑↓のループ
	
	for (int i = LastNodeNum; i >= 0; i--)
	{
		//LastNodeNum=親ノードの数→to=親ノード
		if (route[i].node_height <= 0)
		{
			// node_height = 1 のときのみ子ノードを探索
			continue;
		}
		if (route[i].node_height < 2)
		{	
			//ノード高さ1なら子ノードを接続可
			
			//脚の平行移動探索 (→次のノードは脚の上下移動へ遷移する)
			if (route[i].debug % 100 == 32) 
			{
				//ルールA1　前回が同じ動作なら探索しない	//if (route[to].debug %100/ 10 != 4){
				//ルールA2									//if (route[to].debug %100 / 10 != 4 && route[to].debug % 100 != 24){
				////ルールB									//if (route[to].debug % 100 == 36){
				//ルールC,D									//if (route[to].debug % 100 == 32){

				//脚の移動パターンを全探索する
				std::vector<int> _res_transition_hierarchy;	//階層の遷移の結果を収納する配列．かつてはint型の352のデータをもつ配列だった．変数3^3+8

				//遊脚した足の移動パターン全探索
				searchTransitionHierarchy(route[i], _res_transition_hierarchy);	

				//遷移可能な階層を木構造に挿入する
				for (int j = 0; j < _res_transition_hierarchy.size(); j++) 
				{
					//参照するノードを一つ進める
					LastNodeNum++;

					//オーバーフローする前に緊急停止する
					if (LastNodeNum >= m_route_max) { std::cout << "Error Overflow LastNodeNum = " << LastNodeNum; std::string r; std::cin >> r; }			

					//各パラメータを代入する
					route[LastNodeNum].leg_state = _res_transition_hierarchy[j];						//探索した階層
					route[LastNodeNum].center_of_mass = route[i].center_of_mass;					//重心移動を足す
					route[LastNodeNum].global_center_of_mass = route[i].global_center_of_mass;		//重心移動距離を更新
					route[LastNodeNum].pitch = route[i].pitch;										//旋回は行わない
					route[LastNodeNum].roll = route[i].roll;
					route[LastNodeNum].yaw = route[i].yaw;
					route[LastNodeNum].delta_comz = route[i].delta_comz;
					route[LastNodeNum].target_delta_comz = route[i].target_delta_comz;

					for (int k = 0; k < HexapodConst::LEG_NUM; k++) 
					{
						//脚位置2は親ノードのまま  脚位置はまだ未定義
						route[LastNodeNum].Leg2[k] = route[i].Leg2[k];		
						route[LastNodeNum].Leg[k] = route[i].Leg[k];
					}

					route[LastNodeNum].parent = &route[i];								//子ノードを親ノードに接続
					route[LastNodeNum].node_height = 1;									//新しくできたノードのタイプを1にする
					route[LastNodeNum].debug = 40 + (route[i].debug % 100 / 10);		//この書き方だと43へ遷移する
				}
			}

			//重心の平行移動探索 (→次のノードは脚の平行移動へ遷移する)
			if (route[i].debug % 100 == 26) 
			{
				//ルールA1	前回動作が同じ動作，または，脚の水平移動なら探索しない		//if (route[to].debug % 100 / 10 != 3 && route[to].debug%100 / 10 != 4)
				//ルールA2	//if (route[to].debug % 100 / 10 != 3 && route[to].debug % 100 / 10 != 4 && route[to].debug % 100 != 24) 
				//ルールB	//if ( route[to].debug %100 == 62 ) 
				//ルールC	これ
				//ルールD	//if(route[to].debug%100 ==22)

				//4次方向移動の結果の個数を収納する変数 重心移動
				std::vector<myvector::SLegVector> _res_leg_pos;
				std::vector<myvector::SVector> _res_com_pos;
				std::vector<int> _res_state;									

				//重心の平行移動を全探索する．（4次方向の移動）　
				searchTransitionCoM(route[i], _res_leg_pos, _res_com_pos, _res_state);

				//SearchPossibleLegPositionは胴体と脚設置可能点の接触判定のみに用いる
				_SearchPossibleLegPosition.phantomX.setTarget(m_target);//目標
				_SearchPossibleLegPosition.phantomX.setMyDirection(route[i].pitch, route[i].roll, route[i].yaw);	//自機の角度グローバル

				for (int j = 0; j < (int)_res_leg_pos.size(); j++) 
				{	
					//安定余裕の計算．上の関数で正しく計算されているなら、この部分は必要ない
					int _is_ground[6];
					myvector::SVector leg_buf[6];

					for (int k = 0; k < HexapodConst::LEG_NUM; ++k) 
					{
						_is_ground[k] = isGrounded(_res_state.at(j), k);
						leg_buf[k] = _res_leg_pos.at(j).leg[k];
					}

					//安定余裕一定値以下のとき0となる
					if (Stability_Margin(leg_buf, _is_ground) == 0)
					{	
						//そもそもグラフに接続しない
						continue;
					}

					LastNodeNum++;
					//オーバーフローする前に緊急停止
					if (LastNodeNum >= m_route_max) { std::cout << "Error Overflow LastNodeNum = " << LastNodeNum; }			
					route[LastNodeNum].leg_state = _res_state.at(j);		//重心タイプを変更　脚位置は全て基準位置へ
					route[LastNodeNum].center_of_mass = route[i].center_of_mass;							//重心移動を足す
					route[LastNodeNum].global_center_of_mass = myvector::addVec(route[i].global_center_of_mass, _res_com_pos.at(j));			//重心移動距離を更新
					route[LastNodeNum].pitch = route[i].pitch;
					route[LastNodeNum].roll = route[i].roll;
					route[LastNodeNum].yaw = route[i].yaw;

					for (int k = 0; k < HexapodConst::LEG_NUM; k++)
					{
						route[LastNodeNum].Leg[k] = _res_leg_pos.at(j).leg[k];	//脚位置を更新
						route[LastNodeNum].Leg2[k] = _res_leg_pos.at(j).leg[k];	//2の脚位置を更新

						if (_is_ground[k] == 0)
						{
							route[LastNodeNum].Leg2[k].z = route[i].Leg2[k].z;	//2の脚位置は必ず接地した位置　この部分は、地面が平らなら成り立つ。
						}
					}

					route[LastNodeNum].parent = &route[i];												//子ノードを親ノードに接続
					route[LastNodeNum].node_height = 1;													//新しくできたノードのタイプを1にする
					route[LastNodeNum].debug = route[i].debug % 100 / 10 + 30;							// 32へ遷移する
					route[LastNodeNum].delta_comz = route[i].delta_comz;
					route[LastNodeNum].target_delta_comz = route[i].target_delta_comz;

					//SearchPossibleLegPosition系は胴体と脚設置可能点の接触判定に用いる
					_SearchPossibleLegPosition.phantomX.setMyPosition(route[LastNodeNum].global_center_of_mass);//重心位置グローバル
					_SearchPossibleLegPosition.phantomX.setLocalLeg2Pos(route[LastNodeNum].Leg2);//足先位置の基準ローカル
					_SearchPossibleLegPosition.phantomX.setLocalLegPos(route[LastNodeNum].Leg);//脚先位置ローカル

					//十分に胴体の高さがあるとき0,一定値以下なら正の値を返し、接続不能な子ノードとする、また、上書きする。
					if (_SearchPossibleLegPosition.calculateCollisionAvoidanceMovement() > Define::ALLOWABLE_ERROR) 
					{
						route[LastNodeNum].node_height = -100;
						LastNodeNum--;
					}
				}
			}

			//重心の上下移動探索 (→次のノードは脚の上下移動へ遷移する)
			if (route[i].debug % 100 == 24) 
			{
				//ルールA1　親ノードが同じ動作，または，脚の水平移動なら探索しない	//if (route[to].debug &100/ 10 != 6 && route[to].debug &100/ 10 != 4 ){//重いしあんまうまくいかない
				//ルールA2,C　脚を下したときだけ重心の探索	これ
				//ルールB	//if (route[to].debug % 100 == 22) {
				//ルールD	//if(route[to].debug == 1231323){

				//重心の上下移動用の変数
				const int Edge_num = 5;			//重心の候補地点の分割数を決定する
				myvector::SVector ret_2G_leg_add[Edge_num];
				myvector::SVector ret_2G_GCOM_add[Edge_num];

				//現在の重心位置からの可動範囲を求める
				int _target_delta_comz = searchTransitionComVertical(route[i], Edge_num, ret_2G_leg_add, ret_2G_GCOM_add);

				// 新しいノードを生成して，グラフに追加する
				for (int j = 0; j < Edge_num; ++j)
				{
					LastNodeNum++;

					//オーバーフローする前に緊急停止
					if (LastNodeNum >= m_route_max) { std::cout << "Error Overflow LastNodeNum = " << LastNodeNum; std::string r; std::cin >> r; }
				
					route[LastNodeNum].leg_state = route[i].leg_state;
					route[LastNodeNum].center_of_mass = route[i].center_of_mass;	//使ってないからなくてもいい。
					route[LastNodeNum].global_center_of_mass = route[i].global_center_of_mass + ret_2G_GCOM_add[j];	//重心高さを変更
					route[LastNodeNum].pitch = route[i].pitch;	//回転角は変わらず
					route[LastNodeNum].roll = route[i].roll;
					route[LastNodeNum].yaw = route[i].yaw;

					//脚位置と基準脚位置を変更
					for (int k = 0; k < HexapodConst::LEG_NUM; ++k) 
					{
						if (isGrounded(route[i].leg_state, k) == true) 
						{
							route[LastNodeNum].Leg[k] = route[i].Leg[k] + ret_2G_leg_add[j];
						}
						else 
						{
							//遊脚の重心からの相対位置は変わらず。
							route[LastNodeNum].Leg[k] = route[i].Leg[k]; 
						}

						//基準位置は接地面が基準なので、遊脚か否かにかかわらず、支持脚の移動量だけｚ更新。
						route[LastNodeNum].Leg2[k] = route[i].Leg2[k] + ret_2G_leg_add[j];
					}

					route[LastNodeNum].parent = &route[i];								//子ノードを親ノードに接続
					route[LastNodeNum].node_height = 1;									//新しくできたノードのタイプを1にする
					route[LastNodeNum].debug = route[i].debug % 100 / 10 + 60;			//重心上下移動は6．62が代入される.
					route[LastNodeNum].delta_comz = route[LastNodeNum].global_center_of_mass.z - route[i].global_center_of_mass.z;
					route[LastNodeNum].target_delta_comz = _target_delta_comz;
				}
			}

			//脚の上下移動探索 (→次のノードは重心の平行・上下移動へ遷移する)
			if (route[i].debug % 100 == 62 || route[i].debug % 100 == 43) 
			{
				//ルールA1　2回連続脚の上下のときは探索しない。それ以外は探索する．	if (route[to].debug % 100 != 22){	//すでに2回以上連続で脚を上下運動させている//2次方向の探索 
				//ルールA2		if (route[to].debug % 100 != 22 && route[to].debug % 100 != 24 && route[to].debug % 100 != 32) 	
				//ルールB.D		if (route[to].debug % 100 == 24 || route[to].debug % 100 == 43) 
				//ルールC		これ

				//脚の上下移動探索
				int LegGroundablePointNum[HexapodConst::LEG_NUM][DISCRETE_NUM] = {};						//脚の前後の脚接地可能点 LegGroundablePointNum[HexapodConst::LEG_NUM][0] が１の位置を表す。
				myvector::SVector LegGroundablePoint[HexapodConst::LEG_NUM][DISCRETE_NUM][100] = {};		//複数ある脚位置1，2，3,4,5,6,7に位置する点群の中の代表する1点

				if (m_target.TargetMode == ETargetMode::STRAIGHT_VECOTR || m_target.TargetMode == ETargetMode::STRAIGHT_POSITION)
				{
					//脚位置算出 直線移動	Target.TargetMode <= 2
					getLegGroundablePoint(&route[i], LegGroundablePointNum, LegGroundablePoint);
				}
				else
				{
					//脚位置算出 旋回移動
					SelectLegPoint_Rotation(route[i], LegGroundablePointNum, LegGroundablePoint);
				}

				
				bool visited[ComType::COM_TYPE_NUM] = { 0 };	//探索済みもしくは探索できないノードは1になる
				bool i_F[HexapodConst::LEG_NUM] = {};			//i_F[6]は足をつけるかどうか

				//脚の接地パターンと階層、重心位置からノード削除
				getGraph(route[i].leg_state, i_F, LegGroundablePointNum, visited);	//マップのデータからグラフを得る//遷移不可能なノードはvisited[i]が1になる

				//重心の上下移動で頻発するからcoutをCOしてる(ただちゃんと動けてはいる)
				for (int j = 0; j < HexapodConst::LEG_NUM; ++j) 
				{
					if (isGrounded(route[i].leg_state, j) == true && i_F[j] == false)
					{
						//現在接地脚の脚接地点数が0//2回連続で脚の上下移動を探索するときによくなる。
						std::cout << "Error i_F が不可解な値 \n" << "leg" << j << std::endl;
						route[i].node_height = -100;
						std::cout << "v = ";
						for (int k = 0; k < HexapodConst::LEG_NUM; k++) { std::cout << isGrounded(route[i].leg_state, k); }
						std::cout << ", if=";
						for (int k = 0; k < HexapodConst::LEG_NUM; k++) { std::cout << i_F[k]; }
						std::cout << std::endl;
						break;
					}
				}

				int ret_2_transition_v[ComType::COM_TYPE_NUM][6] = {};	//関数に渡す2次方向移動を保存する配列　36通りある　6は適当にきめた ret_2_transition_v[][0]は格納されているルートの長さ
				int passnum_2zi;										//関数から帰ってきた2次方向移動の数

				//前回上下運動の場合,前々回のノードから1回で移動できるノードを削除したい（今はやっていない）
				pass_transitions_2zi(route[i].leg_state, visited, ret_2_transition_v, &passnum_2zi);	//i_F[6]は足をつけるかどうか　ret_2_transition_v[36][6]に結果//階層内最短経路探索
				//passnum_2ziは移動可能なノードの数
				//ret_2_transition_v[a][b],aは移動可能なノードのノード番号の小さい順,bはルートb=0ルート長さb=1今のノードb=2ひとつ先のノードb=3二つ先のノード…

				for (int j = 0; j < passnum_2zi; j++) 
				{
					//null動作を追加するときは，上の条件式に=を追加．←必要性を感じないので削除．必要ならば過去logから復活させる．
					if (j == passnum_2zi) 
					{
						(LastNodeNum)++;//子ノード
						route[LastNodeNum].leg_state = route[i].leg_state;						//探索した階層
						route[LastNodeNum].center_of_mass = route[i].center_of_mass;							//重心移動を足す
						route[LastNodeNum].global_center_of_mass = route[i].global_center_of_mass;			//重心移動距離を更新
						route[LastNodeNum].pitch = route[i].pitch;											//旋回は行わない
						route[LastNodeNum].roll = route[i].roll;
						route[LastNodeNum].yaw = route[i].yaw;

						for (int le = 0; le < HexapodConst::LEG_NUM; le++) 
						{
							route[LastNodeNum].Leg2[le] = route[i].Leg2[le];		//脚位置2は親ノードのまま  脚位置はまだ未定義
							route[LastNodeNum].Leg[le] = route[i].Leg[le];
						}

						route[LastNodeNum].parent = &route[i];												//子ノードを親ノードに接続
						route[LastNodeNum].node_height = 1;													//新しくできたノードのタイプを1にする
						route[LastNodeNum].debug = route[i].debug % 100 / 10 + 20;
						route[LastNodeNum].delta_comz = route[i].delta_comz;
						route[LastNodeNum].target_delta_comz = route[i].target_delta_comz;
						break;
					}

					//前回上下運動のときはルート長さ1のみ,その他は格納されているルートの長さだけ繰り返し
					if (ret_2_transition_v[j][0] > 1) { continue; }	//1回で移動できるところのみ

					for (int k = 0; k < ret_2_transition_v[j][0]; k++)
					{
						//格納されているルートの長さだけ繰り返し

						int leg_con_buf = ret_2_transition_v[j][k + 2];//子ノードの脚状態
						int v[HexapodConst::LEG_NUM];
						int kaisou[HexapodConst::LEG_NUM];
						myvector::SVector leg_buf[HexapodConst::LEG_NUM];

						for (int l = 0; l < HexapodConst::LEG_NUM; ++l)
						{
							v[l] = isGrounded(leg_con_buf, l);
							kaisou[l] = LegState::getLegState(leg_con_buf, l);
							kaisou[l] -= 1;

							if (v[l]) 
							{
								if (isGrounded(route[i].leg_state, l)) 
								{
									//親ノードでも接地していたら
									leg_buf[l] = route[i].Leg[l];
								}
								else 
								{
									//新たに接地した場合
									leg_buf[l] = LegGroundablePoint[l][kaisou[l]][0];
								}
							}
						}

						//	安定余裕が10mm以下はゼロ
						if (Stability_Margin(leg_buf, v) == 0) 
						{
							continue;	//そもそも接続しない
						}

						LastNodeNum++;	//安定余裕がある時だけグラフに接続

						//オーバーフローする前に緊急停止
						if (LastNodeNum >= m_route_max) { std::cout << "Error Overflow LastNodeNum = " << LastNodeNum; std::string r; std::cin >> r; }	

						route[LastNodeNum].leg_state = leg_con_buf;										//vの値を更新
						route[LastNodeNum].center_of_mass = route[i].center_of_mass;					//重心位置は親ノードと同じ
						route[LastNodeNum].global_center_of_mass = route[i].global_center_of_mass;		//重心位置は親ノードと同じ
						route[LastNodeNum].pitch = route[i].pitch;
						route[LastNodeNum].roll = route[i].roll;
						route[LastNodeNum].yaw = route[i].yaw;
						bool _is_grouned_leg[HexapodConst::LEG_NUM];
						for (int m = 0; m < HexapodConst::LEG_NUM; m++)
						{
							_is_grouned_leg[m] = isGrounded(route[i].leg_state, m);
						}
						//route[LastNodeNum].debug = route[i].debug % 100 / 10 + 20 + iHX2[isGrounded(route[i].leg_state, 0)][isGrounded(route[i].leg_state, 1)][isGrounded(route[i].leg_state, 2)][isGrounded(route[i].leg_state, 3)][isGrounded(route[i].leg_state, 4)][isGrounded(route[i].leg_state, 5)] * 100;	//1回で移動できるときのみに限定しているとき
						route[LastNodeNum].debug = route[i].debug % 100 / 10 + 20 + ComType::getComTypeFromGroundLeg(_is_grouned_leg) * 100;
						route[LastNodeNum].delta_comz = route[i].delta_comz;
						route[LastNodeNum].target_delta_comz = route[i].target_delta_comz;

						for (int l = 0; l < HexapodConst::LEG_NUM; ++l)
						{
							//0は遊脚
							if (v[l] == 0) 
							{								
								route[LastNodeNum].Leg[l] = route[i].Leg[l];
								//route[LastNodeNum].Leg[iLeg].z = route[to].Leg[iLeg].z + LegHeight;
								route[LastNodeNum].Leg[l].z = LegHeight;
								//ここで可動範囲内かどうか調べなきゃいけない20200615
							}
							else 
							{
								route[LastNodeNum].Leg[l] = leg_buf[l];//iLeg番目の脚位置（VECTOR）を代入
								if (LegGroundablePointNum[l][kaisou[l]] == 0) { std::cout << "えっ!!\n"; }
							}

							route[LastNodeNum].Leg2[l] = route[i].Leg2[l];						//脚位置２は変更しない

						}
						//脚位置代入終了

						if (k == 0) 
						{
							route[LastNodeNum].parent = &route[i];								//親ノードと子ノードを接続

							if (route[LastNodeNum].parent->node_height <= 0) 
							{
								//親のノード高さが0以下はない
								route[LastNodeNum].node_height = route[LastNodeNum].parent->node_height - 1;
								route[LastNodeNum].debug = -1;
								std::cout << "ここには来ないでください\n";
							}
							else 
							{
								route[LastNodeNum].node_height = 1;								//とりあえず更新可に,1回目の移動
							}

						}
						else 
						{
							std::cout << "ここには来ないでください\n";
							route[LastNodeNum].parent = &route[LastNodeNum - 1];				//親ノードと子ノードを接続

							if (route[LastNodeNum].parent->node_height <= 0) 
							{
								//親のノード高さが0以下はない
								route[LastNodeNum].node_height = route[LastNodeNum].parent->node_height - 1;
								route[LastNodeNum].debug = -1;
							}
							else 
							{
								//2回移動の場合
								route[LastNodeNum].node_height = 1;								//とりあえず更新可に
								route[LastNodeNum - 1].node_height = 3;							//2次移動の途中のノードなので更新不可
							}
						}


					}
				}
			}
		}

		route[i].node_height++;
	}
}

void PassFinding::searchTransitionHierarchy(const LNODE &node, std::vector<int>& _res_transition_hierarchy)
{
	//遊脚の水平移動後の階層をすべて調べる	

	//結果を入れる配列を空にしておく
	_res_transition_hierarchy.clear();	

	//現在，接地している脚の本数を数える	(//v_bitが1かどうか)
	int _touch_ground_leg_num = getGroundedLegNum(node.leg_state);

	//地面についている脚の本数によって処理をする
	if (_touch_ground_leg_num == 5)
	{
		//遊脚している脚を探す．遊脚数は1なので1つの数字が帰るはず
		std::vector<int> _lifted_leg;
		getLiftedLegNumWithVector(node.leg_state, _lifted_leg);

		//可動範囲内の点を計算する処理は中身がなかったので全削除した．

		//脚状態 0001(1) から 0111(7)まで 次のパターンを生成する．なお上位bitは遊脚を表す．(0なら遊脚)
		for (int i = 1; i <= DISCRETE_NUM; i++)
		{	
			////新しい脚状態を生成する
			int _new_leg_state = node.leg_state;
			changeLegState(_new_leg_state, _lifted_leg.at(0), i);

			if (Define::FLAG_DO_PRUNING == true)
			{
				if (no_use_kaisou[LegState::getLegState(_new_leg_state, 0) - 1][LegState::getLegState(_new_leg_state, 1) - 1][LegState::getLegState(_new_leg_state, 2) - 1][LegState::getLegState(_new_leg_state, 3) - 1][LegState::getLegState(_new_leg_state, 4) - 1][LegState::getLegState(_new_leg_state, 5) - 1] == 0) continue;	/*int版*/
			}

			//結果を代入する．移動後の階層がどこなのかを記録する
			_res_transition_hierarchy.push_back(_new_leg_state);	
		}	
	}
	else if (_touch_ground_leg_num == 4) 
	{
		//遊脚している脚を探す．遊脚数は2なので2つの数字が帰るはず
		std::vector<int> _lifted_leg;
		getLiftedLegNumWithVector(node.leg_state, _lifted_leg);

		//可動範囲内の点を計算する処理は中身がなかったので全削除した．

		//脚状態 0001(1) から 0111(7)まで 次のパターンを生成する．なお上位bitは遊脚を表す．(0なら遊脚)
		for (int i = 1; i <= DISCRETE_NUM; i++)
		{						
			for (int j = 1; j <= DISCRETE_NUM; j++)
			{		
				//新しい脚状態を生成する
				int _new_leg_state = node.leg_state;
				changeLegState(_new_leg_state, _lifted_leg.at(0), i);
				changeLegState(_new_leg_state, _lifted_leg.at(1), j);

				if (Define::FLAG_DO_PRUNING)
				{
					if (no_use_kaisou[LegState::getLegState(_new_leg_state, 0) - 1][LegState::getLegState(_new_leg_state, 1) - 1][LegState::getLegState(_new_leg_state, 2) - 1][LegState::getLegState(_new_leg_state, 3) - 1][LegState::getLegState(_new_leg_state, 4) - 1][LegState::getLegState(_new_leg_state, 5) - 1] == 0) continue;
				}

				//結果を代入する
				_res_transition_hierarchy.push_back(_new_leg_state);	// 移動後の階層がどこなのかを記録する
			}
		}
	}
	else if (_touch_ground_leg_num == 3) 
	{
		//遊脚している脚を探す．遊脚数は3なので3つの数字が帰るはず
		std::vector<int> _lifted_leg;
		getLiftedLegNumWithVector(node.leg_state, _lifted_leg);

		//可動範囲内の点を計算する処理は中身がなかったので全削除した．

		//脚状態 0001(1) から 0111(7)まで 次のパターンを生成する．なお上位bitは遊脚を表す．(0なら遊脚)
		for (int i = 1; i <= DISCRETE_NUM; i++)
		{						
			for (int j = 1; j <= DISCRETE_NUM; j++)
			{
				for (int k = 1; k <= DISCRETE_NUM; k++)
				{
					//新しい脚状態を生成する
					int _new_leg_state = node.leg_state;
					changeLegState(_new_leg_state, _lifted_leg.at(0), i);
					changeLegState(_new_leg_state, _lifted_leg.at(1), j);
					changeLegState(_new_leg_state, _lifted_leg.at(2), k);

					if (Define::FLAG_DO_PRUNING == true)
					{
						if (no_use_kaisou[LegState::getLegState(_new_leg_state, 0) - 1][LegState::getLegState(_new_leg_state, 1) - 1][LegState::getLegState(_new_leg_state, 2) - 1][LegState::getLegState(_new_leg_state, 3) - 1][LegState::getLegState(_new_leg_state, 4) - 1][LegState::getLegState(_new_leg_state, 5) - 1] == 0) continue;

					}

					//結果を代入する
					_res_transition_hierarchy.push_back(_new_leg_state);	// 移動後の階層がどこなのかを記録する
				}
			}
		}
	}
	else 
	{
		//ここに来るのは接地している脚の数が6本 or 1本 or 2本
		//地面についている脚が3本を切ることはない，何故ならロボットが倒れてしまうため．
		//また6本接地しているならば脚を動かせない．(遊脚する必要がある)
		//よって処理を行わない．(_res_transition_hierarchyを空にして終わる)
	}
}

void PassFinding::searchTransitionCoM(const LNODE &node, std::vector<myvector::SLegVector>& _res_leg_pos, std::vector<myvector::SVector>& _res_com_pos, std::vector<int>& _res_state)
{
//重心の平行移動探索　重心移動可能な位置を求めてそれを遷移可能なノードとする 

	//CreateComCandidate の初期化をする
	_CreateComCandidate.initHexapodJustBeforeSearch(node, m_target);

	//vectorの中身を空にする
	_res_leg_pos.clear();
	_res_com_pos.clear();
	_res_state.clear();

	//移動できる重心タイプの数が戻る、その時の移動する重心位置
	_CreateComCandidate.getComMovableArea(node, _res_leg_pos, _res_com_pos, _res_state);
}

int PassFinding::searchTransitionComVertical(const LNODE& _node, const int _devide_num, myvector::SVector ret_2G_leg_add[5], myvector::SVector ret_2G_GCOM_add[5])
{
	_SearchPossibleLegPosition.phantomX.setMyPosition(_node.global_center_of_mass);			//重心位置グローバル
	_SearchPossibleLegPosition.phantomX.setTarget(m_target);								//目標
	_SearchPossibleLegPosition.phantomX.setMyDirection(_node.pitch, _node.roll, _node.yaw);	//自機の角度グローバル
	_SearchPossibleLegPosition.phantomX.setLocalLeg2Pos(_node.Leg2);						//足先位置の基準ローカル
	_SearchPossibleLegPosition.phantomX.setLocalLegPos(_node.Leg);						//脚先位置ローカル

	int _target_z = _SearchPossibleLegPosition.Target_delta_comz();

	double Gz_plus = 1000;	//グローバルで見て重心を上げられる限界高さ
	double Gz_minus = 1000;	//グローバルで見て重心を下げられる限界高さ
	double l = 186.0;		// MAX_DELTAZ;//脚を伸ばしたときのももとすねの長さの和(FEMURから脚先までの長さ(196mm-マージン取って186mm)
	double r = 0;			//Σ(ローカル座標系）xy平面上でのFEMURの座標と脚先の座標の距離（例えばr=0なら、z=lとなる）
	double MAX_z;			//脚を伸ばしたときの胴体と脚先の高さの差、つまりある脚の重心との最大距離。

	//接地している脚について
	for (int i = 0; i < HexapodConst::LEG_NUM; ++i)
	{
		if (isGrounded(_node.leg_state, i) == true)
		{
			//myvector::VECTOR L_femur = _SearchPossibleLegPosition.phantomX.getLocalFemurJointPos(i);//FEMURのローカル座標を取得
			//r = sqrt(pow((Leg.x - L_femur.x), 2) + pow((Leg.y - L_femur.y), 2));
			r = sqrt(_node.Leg[i].x * _node.Leg[i].x + _node.Leg[i].y * _node.Leg[i].y) - HexapodConst::COXA_LENGTH;

			//それぞれの接地脚で現在の脚先位置（x,y)で最大まで足を延ばしたときの胴体と客先の高さの差(z)を計算する
			MAX_z = sqrt(l * l - r * r);//ももの付け根から脚先の距離があるほど、脚を伸ばせる範囲は狭い
			//座標は付け根のままで座標系の回転だけグローバルに合わせたやつ
			//double Lz_now = phantomX2.rotation(node->Leg[i], myvector::VGet(0, 0, 0), node->pitch, node->roll, node->yaw).z;
			double Now_Lz = -_node.Leg[i].z;	//正の値にしとく
			// >=0 胴体回転がないなら、z + node->Leg[i].z に等しい  現在の脚位置から足を下げられる限界
			//わかりにくいから、zを下向きに正で計算する。
			//重心を上げていって一番最初に脚が伸びきったときが、
			if (MAX_z - Now_Lz < Gz_plus)
			{
				Gz_plus = MAX_z - Now_Lz;
			}

			// >=0 胴体回転がないなら、LegHeight(MIN_DELTAZ) - node->Leg[i].z に等しい  現在の脚位置から脚を上げられる限界
			if (Now_Lz - HexapodConst::VERTICAL_MIN_RANGE < Gz_minus)
			{
				Gz_minus = Now_Lz - HexapodConst::VERTICAL_MIN_RANGE;
			}
			//一番最初に支持脚が遊脚とと同じ高さになったときが下げられる限界。
		}
	}

	//上限の調整 本当に可動範囲内か，コメントアウトされていたので全削除．
	

	//下限の調整をする．グローバルで見て遊脚高さが脚の基準位置の高さ(＋margin)より下がらないようにする．
	for (int i = 0; i < HexapodConst::LEG_NUM; ++i)
	{
		if (isGrounded(_node.leg_state, i) == false)
		{
			myvector::SVector Legbuf = _node.Leg[i] - myvector::VGet(0, 0, Gz_minus);

			if (Legbuf.z < _node.Leg2[i].z)
			{
				Gz_minus = _node.Leg[i].z - _node.Leg2[i].z - (HexapodConst::VERTICAL_MIN_RANGE + LEG_HIEGHT);
			}
		}
	}

	//胴体と脚設置可能点の高さの差が一定値以下だったら、下限を修正する。
	_SearchPossibleLegPosition.phantomX.setMyPosition(_node.global_center_of_mass - myvector::VGet(0, 0, Gz_minus));	//一番低くしたときの重心位置
	const double _fix = _SearchPossibleLegPosition.calculateCollisionAvoidanceMovement();//十分に距離が離れているとき0,一定値以下なら、ぶつからない正の重心変更量が入る。

	if (_fix > Define::ALLOWABLE_ERROR)
	{
		//親ノードではぶつかってないはずだから、fix<=Gz_minus;つまり、Gz_minus - fix >= 0;
		Gz_minus -= _fix;

		//Gz_minusが目標高さになるが、そこまで上げられないとき、目標高さをGz_plusにあわせる。
		//下のif文はそもそも、胴体が脚設置可能点に接触している場合しかならないから、基本的にはあり得ない。
		if (Gz_minus + Gz_plus < 0) { Gz_minus = -Gz_plus; }
	}

	double delta_Gz = abs((Gz_minus + Gz_plus) / ((double)_devide_num - 1));//abs()をつけないとバグになることがわかった。
	std::vector<double> add_Gz;

	for (int i = 0; i < _devide_num; ++i)
	{
		add_Gz.push_back(pow(-Gz_minus + delta_Gz * i, 2));
		ret_2G_GCOM_add[i] = myvector::VGet(0, 0, -Gz_minus + delta_Gz * i);
		ret_2G_leg_add[i] = myvector::VGet(0, 0, -(-Gz_minus + delta_Gz * i));
	}

	//変化量が最も少ないものの変化量をゼロにする。
	bool change = false;

	for (int i = 0; i < _devide_num - 1; ++i)
	{
		if (add_Gz[i] - add_Gz[i + 1] <= 0)
		{
			ret_2G_GCOM_add[i] = myvector::VGet(0, 0, 0);
			ret_2G_leg_add[i] = myvector::VGet(0, 0, 0);
			change = true;
			break;
		}
	}

	//一番重心を上げるときをゼロして丸めると、移動できなくなる。
	if (change == false)
	{
		//ret_2G_GCOM_add[EDGE_NUM_2G - 1] = myvector::VGet(0, 0, 0);
		//ret_2G_leg_add[EDGE_NUM_2G - 1] = myvector::VGet(0, 0, 0);
	}
 
	return _target_z;
}

//探索した中から適切なルートを検出	大木さんの時のやつ　直線のみ
int PassFinding::chooseMostSuitableSolution(LNODE ret_pass[20]) {

	LNODE* ret_to = NULL;					//返す値
	//int MAX_kaisou = 0;
	double MAX_Velocity = -1000;
	double MAX_com = -1000;
	double MAX_legPosiAve = -1000;
	double MAX_legPosiAve_and_Velocity = -1000;
	double MAX_legPosiAve_and_zyuusin = -1000;
	int to;
	double legPosiAve, legPosiAveVelocity;
	double zyuusin;							//重心位置移動量
	double distance;
	double MIN_DISTANCE = -1;
	double kyakuitisihyou = 0;				//脚位置指標　接地している脚が3や2である割合が高ければ大きくなる
	double MAX_kyakuitisihyou = 0;			//地面についている脚の前進寄与度の最大
	double NumOfTouchEarth = 0;				//地面についている脚の本数
	static int countNotZyuusinidou = 0;		//12ステップですべての足の移動を行える、また1回の実行で4ステップ以上進むことは保障されるので3回実行して1回も重心移動がなかったら前進できないことになる　関数を出ても消えない
	int countPathnum = 0;						//検討したルートの個数

	for (to = LastNodeNum; to >= 0; to--) 
	{
		if (route[to].node_height < 0)continue;//不正なノードは評価しない
		if (route[to].node_height >= 3)continue;//葉ノード以外は評価しない
		int dist = GetRouteDepth(&route[to]);								//ルートの長さをdistに代入
		zyuusin = myvector::VDot(myvector::subVec(route[to].global_center_of_mass, route[0].global_center_of_mass), m_target.TargetDirection);							//重心位置移動量を計算
		distance = (m_target.TargetPosition.x - route[to].global_center_of_mass.x) * (m_target.TargetPosition.x - route[to].global_center_of_mass.x) + (m_target.TargetPosition.y - route[to].global_center_of_mass.y) * (m_target.TargetPosition.y - route[to].global_center_of_mass.y);
		double Velocity = (double)zyuusin / (double)dist;  //速度のようなものを定義(重心移動量を探索深さで割っている)

		kyakuitisihyou = 0;
		NumOfTouchEarth = 0;
		legPosiAve = 0;

		for (int i = 0; i < 6; i++)
		{
			int legv = isGrounded(route[to].leg_state, i);
			int legkaisou = LegState::getLegState(route[to].leg_state, i);
			//kyakuitisihyou += HX4[route[to].kaisou][i] * HX2[route[to].v][i];
			//NumOfTouchEarth += HX2[route[to].v][i];
			kyakuitisihyou += legkaisou * legv;//各脚の位置3,2,1×状態1接地,0遊脚
			NumOfTouchEarth += legv;
			//legPosiAve += (myvector::VDot(myvector::subVec(route[to].Leg[i], route[0].Leg[i]), Target.TargetDirection) + zyuusin)  *   HX2[route[to].v][i];
			legPosiAve += (myvector::VDot(myvector::subVec(route[to].Leg[i], route[0].Leg[i]), m_target.TargetDirection) + zyuusin) * isGrounded(route[to].leg_state, i);
		}
		kyakuitisihyou /= NumOfTouchEarth;									//地面についている脚の前進寄与度を計算

		legPosiAve /= 6.0;
		legPosiAveVelocity = legPosiAve / (double)dist;

		//ノードの移動が意味を成さないほど少ないものは省く。distが3以下のルートは他に自身より発展したルートが存在するので省く
		if (GetRouteDepth(&route[to]) >= 3) 
		{			
			//ルートが長く、葉ノードか1つ手前ならば判断に入れる

			if (m_target.TargetMode == ETargetMode::STRAIGHT_VECOTR) {
				//legPosiAveが大きいならば無条件で更新
				if (legPosiAve > MAX_legPosiAve) {
					MAX_kyakuitisihyou = kyakuitisihyou;
					MAX_legPosiAve = legPosiAve;
					MAX_com = zyuusin;
					//MAX_kaisou = route[to].kaisou;
					ret_to = &route[to];
				}

				//legPosiAveが同じならば速度が大きいほうを選ぶ
				else if (legPosiAve == MAX_legPosiAve) {

					if (Velocity > MAX_Velocity) {

						MAX_Velocity = Velocity;
						ret_to = &route[to];
					}

				}
			}
			//////////////////////////////////////////////////////////
			if (m_target.TargetMode == ETargetMode::STRAIGHT_POSITION) {
				if (distance > 100 && legPosiAve > MAX_legPosiAve) {//目標から離れているとき移動量が大きい
					MAX_kyakuitisihyou = kyakuitisihyou;
					MIN_DISTANCE = distance;
					MAX_legPosiAve = legPosiAve;
					MAX_com = zyuusin;
					//MAX_kaisou = route[to].kaisou;
					ret_to = &route[to];
				}
				else if (MIN_DISTANCE == -1) {//目標から近いときで1回目
					MAX_kyakuitisihyou = kyakuitisihyou;
					MIN_DISTANCE = distance;
					MAX_legPosiAve = legPosiAve;
					MAX_com = zyuusin;
					//MAX_kaisou = route[to].kaisou;
					ret_to = &route[to];
				}
				else if (MIN_DISTANCE > distance && distance <= 100) {//重心位置が目標に近ければ更新
					MAX_kyakuitisihyou = kyakuitisihyou;
					MIN_DISTANCE = distance;
					MAX_legPosiAve = legPosiAve;
					MAX_com = zyuusin;
					//MAX_kaisou = route[to].kaisou;
					ret_to = &route[to];
				}			////legPosiAveが大きいならば更新
				else if (MIN_DISTANCE == distance && legPosiAve > MAX_legPosiAve && distance <= 100) {
					MAX_kyakuitisihyou = kyakuitisihyou;
					MAX_legPosiAve = legPosiAve;
					MAX_com = zyuusin;
					//MAX_kaisou = route[to].kaisou;
					ret_to = &route[to];
				}
				////legPosiAveが同じならば速度が大きいほうを選ぶ
				else if (MIN_DISTANCE == distance && legPosiAve == MAX_legPosiAve && distance <= 100) {
					if (Velocity > MAX_Velocity) {
						MAX_Velocity = Velocity;
						ret_to = &route[to];
					}
				}
			}
			countPathnum++;
		}//END if(route_A[to][0]>=RET_DIST)
	}
	std::cout << "MIN_DISTANCE=" << MIN_DISTANCE << std::endl;
	*retPriority = MAX_legPosiAve;
	*retSecond = MAX_Velocity;
	//-------------------------------------------------------------------

	std::cout << "saiyuuroutetansaku\n";
	LNODE* Root2My[20] = { NULL };																//ルートを出力するために葉ノードから根ノードのノードを記録する配列
	std::cout << "Root2My[20]\n";
	if (ret_to == NULL) {
		std::cout << "chooseMostSuitableSolution のret_to == NULLで歩けないと判断\n";
		return -1;
	}
	GetRoute(ret_to, Root2My);																//入力されたノードから根ノードまでのノードのアドレスをとってくる関数、得られたノードのアドレスは第2引数に入れられる

	//-------------------------------------------------------------------
	if (MAX_Velocity == 0.0)countNotZyuusinidou++;					//重心移動がなかったらインクリメント
	if (MAX_Velocity != 0.0)countNotZyuusinidou = 0;					//重心移動があった時点で0に戻す
	if (countNotZyuusinidou > 4) {
		ret_pass[0].node_height = -1;
		route[0].node_height = -1;				//4に到達つまり重心移動が連続4回なかったらroute[0].NoodeHeightを-1にする。これは強制終了される
		std::cout << "chooseMostSuitableSolution のcountNotZyuusinidouで歩けないと判断\n";
		return -1;
	}

	if (countPathnum == 0) {//ルートの数なし
		ret_pass[0].node_height = -1;
		route[0].node_height = -1;				//これは強制終了される
		std::cout << "chooseMostSuitableSolution のcountPathnumで歩けないと判断\n";
		return -1;
	}

	for (int dist = 0; dist < DIST_MAX; dist++) {
		if (Root2My[dist] == NULL)break;
		ret_pass[dist] = *Root2My[dist];
	}

	return 0;
}

//旋回半径=0(その場旋回)の評価
int PassFinding::chooseMostSuitableSolutionSpin(std::ofstream& fout1, LNODE ret_pass[20]) {

	int debug_to = 0;
	LNODE* ret_to = NULL;					//返す値
	int MAX_kaisou = 0;
	double MAX_legPosiAve_and_Velocity = -1000;
	double MAX_legPosiAve_and_zyuusin = -1000;
	int MIN_dist = 1000;
	int to;
	double kyakuitisihyou;				//脚位置指標　接地している脚が3や2である割合が高ければ大きくなる
	double MAX_kyakuitisihyou = 0;			//地面についている脚の前進寄与度の最大
	double NumOfTouchEarth;				//地面についている脚の本数
	double rotation;//胴体回転量
	double rotationLegAve = 0;//接地脚の回転量の平均
	double rotation_leg = 0;
	double rotationLegAveVelocity;
	double MAX_rotation = 100;
	double MAX_rotationLegAve = -100;
	double MIN_rotationTarget;
	double StabilityMargin = 0;
	myvector::SVector Rotation_Center = m_target.RotationCenter;//回転中心
	static int countNotZyuusinidou = 0;		//12ステップですべての足の移動を行える、また1回の実行で4ステップ以上進むことは保障されるので3回実行して1回も重心移動がなかったら前進できないことになる　関数を出ても消えない
	int countPathnum = 0;						//検討したルートの個数
	double leg_reference_th[6] = { 45,0,-45,-135,180,135 };	//脚の基準角度[deg]

	for (int i = 0; i < 6; i++) {
		leg_reference_th[i] = leg_reference_th[i] * F_PI / 180.0;	//deg→rad
	}

	for (to = LastNodeNum; to >= 1; to--) {//to=0はロボットの状態が変わらないのでto=1まで
		//std::cout<<"route["<<to<<"].node_height"<<route[to].node_height<<std::endl;
		if (route[to].node_height < 0)continue;//不正なノードは評価しない
		if (route[to].node_height >= 3)continue;//葉ノード以外は評価しない
		int dist = GetRouteDepth(&route[to]);								//ルートの長さをdistに代入
		rotation = route[to].yaw - route[0].yaw;							//胴体z軸回転量
		double Velocity = (double)rotation / (double)dist;						//速度のようなものを定義(回転量/探索長さ)

		kyakuitisihyou = 0;
		NumOfTouchEarth = 0;
		rotationLegAve = 0;

		myvector::SVector legPos;
		myvector::SVector legPosFirst;

		for (int i = 0; i < 6; i++)
		{
			int legv = isGrounded(route[to].leg_state, i);
			int legkaisou = LegState::getLegState(route[to].leg_state, i);
			//kyakuitisihyou += HX4[route[to].kaisou][i] * HX2[route[to].v][i];//各脚の位置3,2,1×状態1接地,0遊脚
			//NumOfTouchEarth += HX2[route[to].v][i];
			kyakuitisihyou += legkaisou * legv;//各脚の位置3,2,1×状態1接地,0遊脚
			NumOfTouchEarth += legv;

			//脚の胴体中心に対する回転量
			legPos = myvector::VRot(myvector::addVec(phantomX.getLocalCoxaJointPos(i), route[to].Leg[i]), route[to].pitch, route[to].roll, -route[to].yaw); //ローカル座標系における現在のノードでの脚位置
			legPosFirst = myvector::VRot(myvector::addVec(phantomX.getLocalCoxaJointPos(i), route[0].Leg[i]), route[0].pitch, route[0].roll, -route[0].yaw);  //ローカル座標系における始点のノードでの脚位置
			rotation_leg = (atan2(legPos.y, legPos.x) - atan2(legPosFirst.y, legPosFirst.x)) * legv;//傾かない前提
			//rotation_leg = (atan2(myvector::VRot(myvector::addVec(phantomX.getLocalCoxaJointPos(i), route[to].Leg[i]), route[to].pitch, route[to].roll, -route[to].yaw).y, myvector::VRot(myvector::addVec(phantomX.getLocalCoxaJointPos(i), route[to].Leg[i]), route[to].pitch, route[to].roll, -route[to].yaw).x) - atan2(myvector::VRot(myvector::addVec(phantomX.getLocalCoxaJointPos(i), route[0].Leg[i]), route[0].pitch, route[0].roll, -route[0].yaw).y, myvector::VRot(myvector::addVec(phantomX.getLocalCoxaJointPos(i), route[0].Leg[i]), route[0].pitch, route[0].roll, -route[0].yaw).x)) * HX2[route[to].v][i];//傾かない前提

			if (rotation_leg < -F_PI) {//180°以上回転しないとする.atan2の範囲を超えているため修正
				rotation_leg += F_PI * 2.0;
			}
			else if (rotation_leg > F_PI) {
				rotation_leg -= F_PI * 2.0;
			}
			//回転中心に対する最終的に接地している脚の回転量atan2回転後（-回転中心＋（重心＋脚位置））-atan2回転前（-回転中心＋（重心＋脚位置））
			//rotation_leg = atan2(myvector::subVec(myvector::VRot(myvector::addVec(phantomX.getLocalCoxaJointPos(i),route[to].Leg[i]),route[to].pitch,route[to].roll,-route[to].yaw),Rotation_Center).y,myvector::subVec(myvector::VRot(myvector::addVec(phantomX.getLocalCoxaJointPos(i),route[to].Leg[i]),route[to].pitch,route[to].roll,-route[to].yaw),Rotation_Center).x)-atan2(myvector::subVec(myvector::VRot(myvector::addVec(phantomX.getLocalCoxaJointPos(i),route[0].Leg[i]),route[0].pitch,route[0].roll,-route[0].yaw),Rotation_Center).y,myvector::subVec(myvector::VRot(myvector::addVec(phantomX.getLocalCoxaJointPos(i),route[0].Leg[i]),route[0].pitch,route[0].roll,-route[0].yaw),Rotation_Center).x) * HX2[route[to].v][i];
			//if(rotation_leg < -F_PI){//180°以上回転しないとする.atan2の範囲を超えているため修正
			//	rotation_leg += F_PI*2.0;
			//}else if(rotation_leg > F_PI){
			//	rotation_leg -= F_PI*2.0;
			//}
			//接地脚の脚の基準位置からの角度
			//rotation_leg = (atan2(route[to].Leg[i].y,route[to].Leg[i].x) - leg_reference_th[i])* HX2[route[to].v][i];
			//if(rotation_leg < -F_PI){//180°以上回転しないとする.atan2の範囲を超えているため修正
			//	rotation_leg += F_PI*2.0;
			//}else if(rotation_leg > F_PI){
			//	rotation_leg -= F_PI*2.0;
			//}
			rotationLegAve += rotation_leg;
		}
		kyakuitisihyou /= NumOfTouchEarth;									//地面についている脚の前進寄与度を計算

		rotationLegAve /= NumOfTouchEarth;
		rotationLegAveVelocity = rotationLegAve / (double)dist;
		double rotationTarget = m_target.TargetAngle.z - route[to].yaw; //目標角度と現在角度の差

		//////////////////////////////////////////////////////////
		if (MAX_rotation == 100) {//1回目
			MAX_kyakuitisihyou = kyakuitisihyou;
			//MAX_kaisou = route[to].kaisou;
			MAX_rotation = rotation * m_target.TargetRotation.z;
			MAX_rotationLegAve = rotationLegAve;
			MIN_rotationTarget = rotationTarget * rotationTarget;
			ret_to = &route[to];
			MIN_dist = dist;
		}
		//回転量が目標と近い更新
		else if (m_target.TargetMode == ETargetMode::TURN_ON_SPOT_ANGLE)
		{
			//角度目標
			if (fabs(rotationTarget) < MIN_rotationTarget)
			{
				//目標角度に近い
				MAX_kyakuitisihyou = kyakuitisihyou;
				//MAX_kaisou = route[to].kaisou;
				MAX_rotation = rotation * m_target.TargetRotation.z;
				MAX_rotationLegAve = rotationLegAve;
				MIN_rotationTarget = fabs(rotationTarget);
				ret_to = &route[to];
				MIN_dist = dist;
			}
			else if (fabs(rotationTarget) == MIN_rotationTarget && rotationLegAve > MAX_rotationLegAve) {//脚の移動量が大きい
				MAX_kyakuitisihyou = kyakuitisihyou;
				//MAX_kaisou = route[to].kaisou;
				MAX_rotation = rotation * m_target.TargetRotation.z;
				MAX_rotationLegAve = rotationLegAve;
				ret_to = &route[to];
				MIN_dist = dist;
			}
			else if (fabs(rotationTarget) == MIN_rotationTarget && rotationLegAve == MAX_rotationLegAve && MIN_dist > dist) {//探索深さが浅い
				MAX_kyakuitisihyou = kyakuitisihyou;
				//MAX_kaisou = route[to].kaisou;
				MAX_rotation = m_target.TargetRotation.z;
				ret_to = &route[to];
				MIN_dist = dist;
			}
		}
		//回転量が大きいと更新
		else if (m_target.TargetMode == ETargetMode::TURN_ON_SPOT_DIRECTION) {//方向目標
			if (rotation * m_target.TargetRotation.z > MAX_rotation) {//回転量が多い
				MAX_kyakuitisihyou = kyakuitisihyou;
				//MAX_kaisou = route[to].kaisou;
				MAX_rotation = rotation * m_target.TargetRotation.z;
				MAX_rotationLegAve = rotationLegAve;
				ret_to = &route[to];
				MIN_dist = dist;
			}
			else if (rotation * m_target.TargetRotation.z == MAX_rotation && rotationLegAve > MAX_rotationLegAve) {//脚の移動量が大きい
				MAX_kyakuitisihyou = kyakuitisihyou;
				//MAX_kaisou = route[to].kaisou;
				MAX_rotationLegAve = rotationLegAve;
				ret_to = &route[to];
				MIN_dist = dist;
			}
			else if (rotation * m_target.TargetRotation.z == MAX_rotation && rotationLegAve == MAX_rotationLegAve && MIN_dist > dist) {//探索深さが浅い
				MAX_kyakuitisihyou = kyakuitisihyou;
				//MAX_kaisou = route[to].kaisou;
				ret_to = &route[to];
				MIN_dist = dist;
			}
		}

		countPathnum++;
		//}//END if(route_A[to][0]>=RET_DIST)
	}
	if (m_target.TargetMode == ETargetMode::TURN_ON_SPOT_ANGLE) {
		if (MAX_rotation != 100) {
			*retPriority = MIN_rotationTarget;
			*retSecond = MAX_rotationLegAve;
		}
		else {
			*retPriority = 100;
			*retSecond = -100;
		}
	}
	else if (m_target.TargetMode == ETargetMode::TURN_ON_SPOT_DIRECTION) {
		if (MAX_rotation != 100) {
			*retPriority = MAX_rotation;
			*retSecond = MAX_rotationLegAve;
			*retThird = MIN_dist;
			//*retPriority = MAX_rotationLegAve;
			//*retSecond = MAX_rotation;
		}
		else {
			*retPriority = -100;
			*retSecond = -100;
			*retThird = 100;
		}
	}
	//-------------------------------------------------------------------
	//std::cout<<"saiyuuroutetansaku\n";
	LNODE* Root2My[20] = { NULL };																//ルートを出力するために葉ノードから根ノードのノードを記録する配列
	//std::cout<<"Root2My[20]\n";
	if (ret_to == NULL) {
		//std::cout<<"chooseMostSuitableSolution のret_to == NULLで歩けないと判断\n"<<"Hello";
		return -1;
	}
	GetRoute(ret_to, Root2My);																//入力されたノードから根ノードまでのノードのアドレスをとってくる関数、得られたノードのアドレスは第2引数に入れられる
	//-------------------------------------------------------------------


	if (MAX_rotation == 0.0)countNotZyuusinidou++;					//旋回移動がなかったらインクリメント
	if (MAX_rotation != 0.0)countNotZyuusinidou = 0;					//旋回移動があった時点で0に戻す

	if (countPathnum == 0) {//ルートの数なし
		ret_pass[0].node_height = -1;
		route[0].node_height = -1;				//これは強制終了される
		std::cout << "chooseMostSuitableSolution のcountPathnumで歩けないと判断\n";
		return -1;
	}

	for (int dist = 0; dist < DIST_MAX; dist++) {
		if (Root2My[dist] == NULL)break;
		ret_pass[dist] = *Root2My[dist];
	}

	//ファイルに出力確認用
	fout1 << "///////////////////////////////////////////////\n                ルート決定\n///////////////////////////////////////////////\n";
	fout1 << "LastNodeNum = " << LastNodeNum << "\n";
	for (int dist = 0; dist < DIST_MAX; dist++) {
		int v[6];
		for (int i = 0; i < HexapodConst::LEG_NUM; ++i) v[i] = isGrounded(Root2My[dist]->leg_state, i);
		if (Root2My[dist] == NULL)break;														//全て出力されたら出力をやめる
		fout1 << "debug = " << Root2My[dist]->debug << "\n";
		fout1 << "node_height = " << Root2My[dist]->node_height << "\n";
		//fout1<<"COMType = "<<Root2My[dist]->COM_type<<"\n";
		fout1 << "COMType = " << LegState::getComPatternState(Root2My[dist]->leg_state) << "\n";
		//fout1<<"v = "<<Root2My[dist]->v<<"\n";
		//fout1<<"kaisou = "<<Root2My[dist]->kaisou<<"\n";
		fout1 << "leg_condition = " << std::bitset<24>(Root2My[dist]->leg_state) << "\n";
		fout1 << "center_of_mass[" << dist << "] = \t";
		myvector::VectorOutPut(Root2My[dist]->global_center_of_mass, fout1);
		fout1 << "Leg\n";
		for (int i = 0; i < 6; i++) {
			myvector::VectorOutPut(Root2My[dist]->Leg[i], fout1);			//脚位置
		}
		fout1 << "thY = " << Root2My[dist]->yaw << "\n";
		//fout1<<"stabilitymargin = "<<Stability_Margin(Root2My[dist]->Leg, HX2[Root2My[dist]->v])<<"\n";
		fout1 << "stabilitymargin = " << Stability_Margin(Root2My[dist]->Leg, v) << "\n";

		fout1 << "\n";
	}

	return 0;
}

//旋回半径>0の時の評価　現在は直進でも旋回半径を十分大きいものとして扱ってこの関数で評価している190613
int PassFinding::chooseMostSuitableSolutionRotation(std::ofstream& fout1, LNODE ret_pass[20]) 
{
	myvector::SVector Vtemp;
	//double temp;
	//double temp2;
	LNODE* ret_to = NULL;					//返す値

	//int MAX_kaisou = 0;
	//double MAX_Velocity = -1000;
	//double MAX_legPosiAve = -1000;
	//double MAX_legPosiAve_and_Velocity = -1000;
	//double MAX_legPosiAve_and_zyuusin = -1000;

	int MIN_dist = 1000;
	int to;
	double targetDistance = 0;						//目標との距離
	double MIN_targetDistance;
	double kyakuitisihyou;				//脚位置指標　接地している脚が3や2である割合が高ければ大きくなる
	//double MAX_kyakuitisihyou = 0;			//地面についている脚の前進寄与度の最大
	double NumOfTouchEarth;				//地面についている脚の本数
	double rotation;//胴体回転量
	double rotation_leg_ave = 0;//接地脚の回転量の平均
	double rotation_leg = 0;
	//double rotation_leg_diff = 0;
	//double rotation_leg_diff2 = 0;
	//double MIN_rotation_leg_diff = 100;
	double MAX_rotation = 100;
	double MAX_rotation_leg = -100;
	//double MAX_rotation_leg_ave = -100;
	double CtoGdist;		//回転中心との距離
	double tangentAngle;	//接線角度
	double CircumferenceAngle;	//円周上の角度
	double transitionAngle;	//重心の移動角度
	double MAX_transitionAngle = -100;
	double robotAngle;
	double radiusDiff;
	double AllowradiusDiff = 100000;
	double AllowrobotAngle = 100000;
	double MIN_robotAngle;	//ロボットの角度と目標角度の差の最小
	static int countNotZyuusinidou = 0;		//12ステップですべての足の移動を行える、また1回の実行で4ステップ以上進むことは保障されるので3回実行して1回も重心移動がなかったら前進できないことになる　関数を出ても消えない
	int countPathnum = 0;						//検討したルートの個数	
	double comz = 100;
	//int count_else = 0;

	//---重心高さ評価用変数---//20200721追加hato（ targettype == 5だけ）
	int diff_delta_comz = 10000;
	int MIN_ddc = 10000;//MINというより、ゼロ以下がよくて、その中でもゼロに近いものが一番良い、ゼロ以下場合、正の値でゼロに近いものが最もいい

	//-------------------------//


	//--------全ての評価対象ノードを複数の評価値を用いて比較し、最適なノードを1つ選択する（for文開始）-------------//
	for (to = LastNodeNum; to >= 1; to--) {//to=0はロボットの状態が変わらないのでto=1まで
		 //std::cout<<"route["<<to<<"].node_height"<<route[to].node_height<<std::endl;
		if (route[to].node_height < 0)continue;	//不正なノードは評価しない
		if (route[to].node_height >= 3)continue;	//葉ノード以外は評価しない
		//----------------------------------------各ノードの評価値を計算------------------------------------------//
		//--------------1つ目の評価値(transitionAngle)重心の移動角度を計算----------------------------------------------------------------------//
		Vtemp = myvector::subVec(route[to].global_center_of_mass, m_target.RotationCenter);//旋回中心から重心へのベクトル
		CtoGdist = myvector::V2Mag(Vtemp);							//重心位置と回転中心との距離
		radiusDiff = fabs(CtoGdist - m_target.TurningRadius);			//目標とする旋回半径と現在の重心位置との差(絶対値)
		int dist = GetRouteDepth(&route[to]);						//ルートの長さ

		//目標位置との距離    Target.TargetMode % 2 == 0
		if (m_target.TargetMode == ETargetMode::STRAIGHT_POSITION || m_target.TargetMode == ETargetMode::TURN_ANGLE || m_target.TargetMode == ETargetMode::TURN_ON_SPOT_ANGLE)
		{
			targetDistance = myvector::V2Mag(myvector::subVec(m_target.TargetPosition, route[to].global_center_of_mass));
		}

		rotation = (route[to].yaw - route[0].yaw) * m_target.TargetRotation.z;							//胴体ヨー軸回転量
		CircumferenceAngle = atan2(Vtemp.y, Vtemp.x);					//重心の旋回中心に対する角度(x軸の正方向が0°)
		Vtemp = myvector::subVec(route[0].global_center_of_mass, m_target.RotationCenter);
		transitionAngle = limitRangeAngle(CircumferenceAngle - atan2(Vtemp.y, Vtemp.x));	//旋回角度の変化(±180°以内)
		transitionAngle *= m_target.TargetRotation.z;					//目標方向が正
		//-----------------------------------------------------------------------------------------------------------//
		//-----------------2つ目の評価値（robot_angle）おそらく目標軌道と、ロボット座標系y軸のなす角を計算-------------------------//
		//目標値なしのとき//円より重心が内側→中心から引いた線に垂直方向、円より外側→円への接線方向を胴体角度の目標とする
		if (CtoGdist <= m_target.TurningRadius) {//旋回半径より内側
			tangentAngle = CircumferenceAngle + F_PI / 2.0 * m_target.TargetRotation.z - F_PI / 2.0;//旋回円の重心位置での接線方向
		}
		else {
			//外側
			myvector::SVector a;
			Vtemp.x = route[to].global_center_of_mass.x - m_target.RotationCenter.x;
			Vtemp.y = route[to].global_center_of_mass.y - m_target.RotationCenter.y;

			if (m_target.TargetRotation.z > 0)
			{
				//左回転方向接点
				a.x = (Vtemp.x * m_target.TurningRadius - Vtemp.y * sqrt(pow(Vtemp.x, 2.0) + pow(Vtemp.y, 2.0) - pow(m_target.TurningRadius, 2.0))) * m_target.TurningRadius / (pow(Vtemp.x, 2.0) + pow(Vtemp.y, 2.0)) + m_target.RotationCenter.x;
				a.y = (Vtemp.y * m_target.TurningRadius + Vtemp.x * sqrt(pow(Vtemp.x, 2.0) + pow(Vtemp.y, 2.0) - pow(m_target.TurningRadius, 2.0))) * m_target.TurningRadius / (pow(Vtemp.x, 2.0) + pow(Vtemp.y, 2.0)) + m_target.RotationCenter.y;
				tangentAngle = atan2(a.y - route[to].global_center_of_mass.y, a.x - route[to].global_center_of_mass.x) - F_PI / 2.0;
			}
			else if (m_target.TargetRotation.z < 0)
			{
				//右回転方向接点
				a.x = (Vtemp.x * m_target.TurningRadius + Vtemp.y * sqrt(pow(Vtemp.x, 2.0) + pow(Vtemp.y, 2.0) - pow(m_target.TurningRadius, 2.0))) * m_target.TurningRadius / (pow(Vtemp.x, 2.0) + pow(Vtemp.y, 2.0)) + m_target.RotationCenter.x;
				a.y = (Vtemp.y * m_target.TurningRadius - Vtemp.x * sqrt(pow(Vtemp.x, 2.0) + pow(Vtemp.y, 2.0) - pow(m_target.TurningRadius, 2.0))) * m_target.TurningRadius / (pow(Vtemp.x, 2.0) + pow(Vtemp.y, 2.0)) + m_target.RotationCenter.y;
				tangentAngle = atan2(a.y - route[to].global_center_of_mass.y, a.x - route[to].global_center_of_mass.x) - F_PI / 2.0;
			}
		}

		robotAngle = limitRangeAngle(tangentAngle - route[to].yaw);
		robotAngle = fabs(robotAngle);
		kyakuitisihyou = 0;
		NumOfTouchEarth = 0;
		rotation_leg_ave = 0;

		//--------------------------------------------------------------------------------------------------------------------------//
		//-------------------------------3つ目の評価値（rotation_leg_ave）地面についている脚の平均回転量（とは？）を計算-------------//
		for (int i = 0; i < 6; i++)
		{
			int legv = isGrounded(route[to].leg_state, i);
			int legkaisou = LegState::getLegState(route[to].leg_state, i);
			//kyakuitisihyou += HX4[route[to].kaisou][i] * HX2[route[to].v][i];//各脚の位置3,2,1×状態1接地,0遊脚
			//NumOfTouchEarth += HX2[route[to].v][i];							//接地脚の数
			kyakuitisihyou += legkaisou * legv;//各脚の位置3,2,1×状態1接地,0遊脚
			NumOfTouchEarth += legv;							//接地脚の数
			//legPosiAve += (myvector::VDot(myvector::subVec(route[to].Leg[i], route[0].Leg[i]), Target.TargetDirection) + myvector::VDot(myvector::subVec(route[to].global_center_of_mass,route[0].global_center_of_mass),Target.TargetDirection)) * HX2[route[to].v][i];//接地脚の移動量
			////脚の胴体中心に対する回転量(atan2(回転後(重心から脚先).y,回転後(重心から脚先).x)-回転前
			//rotation_leg = (atan2(myvector::VRot(myvector::addVec(phantomX.getLocalCoxaJointPos(i),route[to].Leg[i]),route[to].pitch,route[to].roll,-route[to].yaw).y,myvector::VRot(myvector::addVec(phantomX.getLocalCoxaJointPos(i),route[to].Leg[i]),route[to].pitch,route[to].roll,-route[to].yaw).x) - atan2(myvector::VRot(myvector::addVec(phantomX.getLocalCoxaJointPos(i),route[0].Leg[i]),route[0].pitch,route[0].roll,-route[0].yaw).y, myvector::VRot(myvector::addVec(phantomX.getLocalCoxaJointPos(i),route[0].Leg[i]),route[0].pitch,route[0].roll,-route[0].yaw).x)) * HX2[route[to].v][i];//傾かない前提
			//if(rotation_leg < -F_PI){//180°以上回転しないとする.atan2の範囲を超えているため修正
			//	rotation_leg += F_PI*2.0;
			//}else if(rotation_leg > F_PI){
			//	rotation_leg -= F_PI*2.0;
			//}

			//回転中心に対する脚の回転量(atan2回転後（ー回転中心＋（重心＋脚位置））-atan2回転前（ー回転中心＋（重心＋脚位置））×目標方向)
			rotation_leg = (atan2(myvector::subVec(myvector::VRot(myvector::addVec(phantomX.getLocalCoxaJointPos(i), route[to].Leg[i]), route[to].pitch, route[to].roll, -route[to].yaw), m_target.RotationCenter).y, myvector::subVec(myvector::VRot(myvector::addVec(phantomX.getLocalCoxaJointPos(i), route[to].Leg[i]), route[to].pitch, route[to].roll, -route[to].yaw), m_target.RotationCenter).x) - atan2(myvector::subVec(myvector::VRot(myvector::addVec(phantomX.getLocalCoxaJointPos(i), route[0].Leg[i]), route[0].pitch, route[0].roll, -route[0].yaw), m_target.RotationCenter).y, myvector::subVec(myvector::VRot(myvector::addVec(phantomX.getLocalCoxaJointPos(i), route[0].Leg[i]), route[0].pitch, route[0].roll, -route[0].yaw), m_target.RotationCenter).x));
			rotation_leg = limitRangeAngle(rotation_leg);
			rotation_leg = rotation_leg * m_target.TargetRotation.z * legv;
			rotation_leg_ave += rotation_leg;
		}
		kyakuitisihyou /= NumOfTouchEarth;									//地面についている脚の前進寄与度を計算

		rotation_leg = rotation_leg_ave;
		rotation_leg_ave /= NumOfTouchEarth;//地面についている脚の平均回転量
		//-------------------------------------------------------------------------------------------------------------------------------//


		//---------------------------------4つ目の評価値（diff_delta_comz)目標とする重心高さ変更量と、実際の変更量の差---------------------//
		diff_delta_comz = (int)(route[to].target_delta_comz - route[to].delta_comz);

		/*これは、	int delta_comz;//重心の上下動作で実際に動いた量
		int target_delta_comz;//重心の上下動作で動いてほしい量（ゼロ以下にならないと段差を上れない）
		だから、優先順位としては、
		１．ゼロ以下である（段差を越せる）
		２．ゼロ以下のものの中では、もっともゼロに近い方がよい（段差を越せるもののうち、重心高さが最も低くなるもの）
		３．ゼロ以下となるノードがない場合は、ゼロより大きいもののうち、もっともゼロに近い方がよい（段差はまだ越せないけど、越せる方向に最も動いてることを示す）
		で評価するようにする。20200721 hato
		*/
		//------------------------------------------------------------------------------------------------------------------//

		//---------------------各ノードの評価値の計算終了-------------------------------------//




		//計算した評価値をもとにノードを比較して、最適ノードを選択する

		//回転量が目標と近い更新（ある意味初期化）
		if (MAX_rotation_leg == -100) {//1回目または重心位置が一定範囲外
			MAX_rotation_leg = rotation_leg_ave;
			MIN_targetDistance = targetDistance;
			MIN_robotAngle = robotAngle;
			MAX_transitionAngle = transitionAngle;
			ret_to = &route[to];
			//MIN_rotation_leg_diff = rotation_leg_diff;
			MIN_dist = dist;
			AllowradiusDiff = radiusDiff;
			AllowrobotAngle = robotAngle;
			MAX_rotation = rotation;
			comz = fabs(route[to].delta_comz);
			MIN_ddc = diff_delta_comz;//20200721
		}



		if (m_target.TargetMode == ETargetMode::TURN_DIRECTION)
		{
			//方向

				//現状最適ノードと比較して、高さ変更量の差がゼロ以下で、最適ノードよりもゼロに近い場合
			if (MIN_ddc <= 0 && diff_delta_comz <= 0 && MIN_ddc < diff_delta_comz) {
				MAX_rotation_leg = rotation_leg_ave;
				MAX_transitionAngle = transitionAngle;
				MIN_robotAngle = robotAngle;
				MIN_dist = dist;
				//route[to].global_center_of_mass = myvector::addVec(route[to].global_center_of_mass, myvector::VGet(0,0,(double)diff_delta_comz));
				ret_to = &route[to];
				MIN_ddc = diff_delta_comz;
			}
			//ゼロ以下かつ値が一緒
			else if (MIN_ddc <= 0 && diff_delta_comz <= 0 && diff_delta_comz == MIN_ddc) {//ゼロ以下で値が同じ場合、重心の移動角度が大きいほうがいい
				if (transitionAngle > MAX_transitionAngle) {
					//std::cout<<rotation_leg_ave<<",";
					MAX_rotation_leg = rotation_leg_ave;
					MAX_transitionAngle = transitionAngle;
					MIN_robotAngle = robotAngle;
					MIN_dist = dist;
					ret_to = &route[to];
					MIN_ddc = diff_delta_comz;
				}
				//胴体の回転が接線方向
				else if (MIN_robotAngle > robotAngle && transitionAngle == MAX_transitionAngle) {//移動角度も同じ場合、robotangleが小さい方がいい
					MAX_rotation_leg = rotation_leg_ave;
					MIN_robotAngle = robotAngle;
					MIN_dist = dist;
					ret_to = &route[to];
					MIN_ddc = diff_delta_comz;
				}
				//接地脚の移動角度の平均
				else if (transitionAngle == MAX_transitionAngle && MAX_rotation_leg < rotation_leg_ave && MIN_robotAngle == robotAngle) {//robotangleも同じ場合、脚の移動角度が大きい方がいい
					MAX_rotation_leg = rotation_leg_ave;
					MIN_robotAngle = robotAngle;
					MIN_dist = dist;
					ret_to = &route[to];
					MIN_ddc = diff_delta_comz;
				}
			}
			//基準が正の値だが、ゼロ以下
			else if (MIN_ddc > 0 && diff_delta_comz <= 0) {
				MAX_rotation_leg = rotation_leg_ave;
				MAX_transitionAngle = transitionAngle;
				MIN_robotAngle = robotAngle;
				MIN_dist = dist;
				//route[to].global_center_of_mass.z += diff_delta_comz;
				ret_to = &route[to];
				MIN_ddc = diff_delta_comz;
			}
			//ゼロより大きくて、ゼロに近い
			else if (MIN_ddc > 0 && diff_delta_comz > 0 && MIN_ddc > diff_delta_comz) {
				MAX_rotation_leg = rotation_leg_ave;
				MAX_transitionAngle = transitionAngle;
				MIN_robotAngle = robotAngle;
				MIN_dist = dist;
				ret_to = &route[to];
				MIN_ddc = diff_delta_comz;
			}
			//ゼロより大きくてかつ値が一緒
			else if (MIN_ddc > 0 && diff_delta_comz > 0 && MIN_ddc == diff_delta_comz) {
				if (transitionAngle > MAX_transitionAngle) {
					//std::cout<<rotation_leg_ave<<",";
					MAX_rotation_leg = rotation_leg_ave;
					MAX_transitionAngle = transitionAngle;
					MIN_robotAngle = robotAngle;
					MIN_dist = dist;
					ret_to = &route[to];
					MIN_ddc = diff_delta_comz;
				}
				//胴体の回転が接線方向
				else if (MIN_robotAngle > robotAngle && transitionAngle == MAX_transitionAngle) {//移動角度も同じ場合、robotangleが小さい方がいい
					MAX_rotation_leg = rotation_leg_ave;
					MIN_robotAngle = robotAngle;

					MIN_dist = dist;
					ret_to = &route[to];
					MIN_ddc = diff_delta_comz;
				}
				//接地脚の移動角度の平均
				else if (transitionAngle == MAX_transitionAngle && MAX_rotation_leg < rotation_leg_ave && MIN_robotAngle == robotAngle) {//robotangleも同じ場合、脚の移動角度が大きい方がいい
					MAX_rotation_leg = rotation_leg_ave;
					MIN_robotAngle = robotAngle;
					MIN_dist = dist;
					ret_to = &route[to];
					MIN_ddc = diff_delta_comz;
				}
			}


			//---------------------------------------------------------------------------------//



		}
		else if (m_target.TargetMode == ETargetMode::TURN_ANGLE) {
			//位置
										  //目標位置に近い
			if (targetDistance < MIN_targetDistance) {
				MAX_transitionAngle = transitionAngle;
				MIN_targetDistance = targetDistance;
				MIN_robotAngle = robotAngle;
				MIN_dist = dist;
				ret_to = &route[to];
			}
			//胴体の回転が接線方向
			else if (MIN_robotAngle > robotAngle && targetDistance == MIN_targetDistance) {
				MAX_transitionAngle = transitionAngle;
				MIN_robotAngle = robotAngle;
				MIN_dist = dist;
				ret_to = &route[to];
			}
			//探索深さが浅い
			else if (MIN_dist > dist && MIN_robotAngle == robotAngle && targetDistance == MIN_targetDistance) {
				MAX_transitionAngle = transitionAngle;
				MIN_dist = dist;
				ret_to = &route[to];
			}

		}
		else if (false/*Target.TargetMode == 7*/)
		{
			//方向
				//if(transitionAngle < F_PI*2.0/3.0){
			if (MAX_rotation < rotation) {//胴体の角度量が多い
				MAX_rotation = rotation;
				MAX_transitionAngle = transitionAngle;
				MIN_robotAngle = robotAngle;
				MAX_rotation_leg = rotation_leg_ave;
				MIN_dist = dist;
				ret_to = &route[to];
			}
			//旋回中心を中心とした旋回角度が大きい
			else if (transitionAngle > MAX_transitionAngle && MAX_rotation == rotation) {
				MAX_transitionAngle = transitionAngle;
				MIN_robotAngle = robotAngle;
				MAX_rotation_leg = rotation_leg_ave;
				MIN_dist = dist;
				ret_to = &route[to];
			}
			//胴体の回転が接線方向
			else if (MIN_robotAngle > robotAngle && transitionAngle == MAX_transitionAngle && MAX_rotation == rotation) {
				MIN_robotAngle = robotAngle;
				MAX_rotation_leg = rotation_leg_ave;
				MIN_dist = dist;
				ret_to = &route[to];
			}
			//接地脚の移動角度の平均
			else if (MIN_robotAngle == robotAngle && transitionAngle == MAX_transitionAngle && MAX_rotation == rotation && MAX_rotation_leg < rotation_leg_ave) {
				MAX_rotation_leg = rotation_leg_ave;
				MIN_dist = dist;
				ret_to = &route[to];
			}
		}
		else if (false/*Target.TargetMode == 8*/)
		{
			//位置

			//目標位置に近い
			if (targetDistance < MIN_targetDistance) {
				MAX_transitionAngle = transitionAngle;
				MIN_targetDistance = targetDistance;
				MIN_robotAngle = robotAngle;
				MIN_dist = dist;
				ret_to = &route[to];
			}
			//胴体の回転が接線方向
			else if (MIN_robotAngle > robotAngle && targetDistance == MIN_targetDistance) {
				MAX_transitionAngle = transitionAngle;
				MIN_robotAngle = robotAngle;
				MIN_dist = dist;
				ret_to = &route[to];
			}
			//探索深さが浅い
			else if (MIN_dist > dist && MIN_robotAngle == robotAngle && targetDistance == MIN_targetDistance) {
				MAX_transitionAngle = transitionAngle;
				MIN_dist = dist;
				ret_to = &route[to];
			}

		}

		//同じなら安定余裕の大きいもの
		//if(StabilityMargin > Stability_Margin(route[to].Leg, HX2[route[to].v]);

		countPathnum++;
	}

	//最適なノードが１つ選択された（for文終了）


	//メンバ変数に最適ノードの評価値を代入(functionpassfindingで他スレッドの最適ノードと比較するため)

	//あとはここの5を変更（した）（functionpassfinding も忘れずに）
	if (m_target.TargetMode == ETargetMode::TURN_DIRECTION)
	{
		if (MAX_rotation_leg != -100) {
			*retPriority = MIN_ddc;
			*retSecond = MAX_transitionAngle;
			*retThird = MIN_robotAngle;
			*retFourth = MAX_rotation_leg;
		}
		else {
			*retPriority = 10000;
			*retSecond = -100;
			*retThird = 100;
			*retFourth = -100;
		}
	}
	else if (m_target.TargetMode == ETargetMode::TURN_ANGLE) {
		if (MAX_rotation_leg != -100) {
			*retPriority = MAX_rotation;
			*retSecond = MAX_transitionAngle;
		}
		else {
			*retPriority = -100;
			*retSecond = -100;
		}
	}
	else if (/*Target.TargetMode == 7*/false)
	{
		if (MAX_rotation_leg != -100) {
			*retPriority = MAX_rotation;
			*retSecond = MAX_transitionAngle;
			*retThird = MIN_robotAngle;
			*retFourth = MAX_rotation_leg;
			std::cout << "Priority=" << MAX_rotation << std::endl;
			std::cout << "Second=" << MAX_transitionAngle << std::endl;
			std::cout << "Third=" << MIN_robotAngle << std::endl;
		}
		else {
			*retPriority = -100;
			*retSecond = -100;
			*retThird = 100;
			*retFourth = -100;
		}
	}
	else if (/*Target.TargetMode == 8*/false)
	{
		if (MAX_rotation_leg != -100)
		{
			*retPriority = MAX_rotation;
			*retSecond = MIN_robotAngle;
		}
		else
		{
			*retPriority = -100;
			*retSecond = 100;
		}
	}
	//---------------------------------------------------------------------------------------------------------------------------------------//


	//----------------------------探索で得られたノードに対して、評価に値するノードが1つもなかったとき -1を返す------------------//
	if (countPathnum == 0) {//for文の末尾までたどり着いたノードがない
		ret_pass[0].node_height = -1;
		route[0].node_height = -1;				//これは強制終了される
		std::cout << "chooseMostSuitableSolution のcountPathnumで歩けないと判断\n";
		return -1;
	}
	//-----------------------------------------------------------------------------------------------------------------------//


	//-------------------探索を始めたノードから最適ノードに至るまでの経路を得る-------------------//
	//std::cout<<"saiyuuroutetansaku\n";
	LNODE* Root2My[20] = { NULL };																//ルートを出力するために葉ノードから根ノードのノードを記録する配列
	//std::cout<<"Root2My[20]\n";
	//最適ノードがそもそもないときの処理
	if (ret_to == NULL) {
		std::cout << "chooseMostSuitableSolution のret_to == NULLで歩けないと判断\n" << "Hello";
		return -1;
	}
	GetRoute(ret_to, Root2My);																//入力されたノードから根ノードまでのノードのアドレスをとってくる関数、得られたノードのアドレスは第2引数に入れられる
	//---------------------------------------------------------------------------------------------//


	//----------------------よくわからんし、使われてない-----------------------//
	if (MAX_transitionAngle == 0.0 && MAX_rotation == 0.0)countNotZyuusinidou++;					//重心移動がなかったらインクリメント
	if (MAX_transitionAngle != 0.0 || MAX_rotation != 0.0)countNotZyuusinidou = 0;					//重心移動があった時点で0に戻す
	//-------------------------------------------------------------------------//


	//-------------------探索を始めたノードから最適ノードに至るまでの経路をコピー-------------------//

	for (int dist = 0; dist < DIST_MAX; dist++) {
		if (Root2My[dist] == NULL)break;
		ret_pass[dist] = *Root2My[dist];
	}
	//---------------------------------------------------------------------------------------------//

	return 0;
}

//routeにあるノード全てで重複していないか確認している	脚状態・胴体姿勢・重心位置が同じなら無効化	親ノードが無効になっていたら子の無効	//脚座標は見なくていいのか？
int PassFinding::deleteNode(LNODE* deletCondition) 
{
	//同じノードの場合NoodeHeight=-10000
	double diff = 0;
	int cntDeleteNode = 0;

	for (int to = LastNodeNum; to >= 0; to--) {

		//	for(int iLeg = 0; iLeg < 6; iLeg++){
		//		diff += myvector::VMag2(route[to].Leg[iLeg], deletCondition->Leg[iLeg]);
		//	}

		//	diff += myvector::VMag2(route[to].global_center_of_mass, deletCondition->global_center_of_mass);
		//
		//		ロボットの状態が一致				　　　　　　かつ					ヨー軸回転角が一致	 
		if ((route[to].leg_state == deletCondition->leg_state) && (route[to].yaw * route[to].yaw - deletCondition->yaw * deletCondition->yaw < 0.0001)) {
			myvector::SVector nowG = myvector::VGet(route[to].global_center_of_mass.x, route[to].global_center_of_mass.y, route[to].global_center_of_mass.z);
			myvector::SVector deletG = myvector::VGet(deletCondition->global_center_of_mass.x, deletCondition->global_center_of_mass.z, deletCondition->global_center_of_mass.z);
			if (myvector::VMag(myvector::subVec(nowG, deletG)) < 0.001) {	//z座標も確認するべき？←確認するべきというか消さなくてよさげ20200626hato いまは、getoptimalloute(,)でCOしてる
				route[to].node_height = -10000;
				cntDeleteNode++;
			}
		}

	}

	for (int to = LastNodeNum; to >= 0; to--) {
		if (route[to].parent == NULL)continue;
		if (route[to].parent->node_height == -10000 && route[to].node_height != -10000) {
			route[to].node_height = -10000;
			to = LastNodeNum + 1;
		}
	}

	return cntDeleteNode;
}

//④脚の上下移動　2次方向の移動の探索
//旋回動作時の脚位置選択
int PassFinding::SelectLegPoint_Rotation(const LNODE& node, int LegGroundablePointNum[HexapodConst::LEG_NUM][DISCRETE_NUM], myvector::SVector LegGroundablePoint[HexapodConst::LEG_NUM][DISCRETE_NUM][100]) 
{
	myvector::SVector Rotation_Center = m_target.RotationCenter;//回転中心．回転中心が脚位置と同じときは外積が0となる

	_SearchPossibleLegPosition.phantomX.setMyPosition(node.global_center_of_mass);
	_SearchPossibleLegPosition.phantomX.setTarget(m_target);
	_SearchPossibleLegPosition.phantomX.setMyDirection(node.pitch, node.roll, node.yaw);
	_SearchPossibleLegPosition.phantomX.setLocalLeg2Pos(node.Leg2);
	_SearchPossibleLegPosition.phantomX.setLocalLegPos(node.Leg);

	int ground_num = LegState::getGroundedLegNum(node.leg_state);
	
	_SearchPossibleLegPosition.setLegState(node.leg_state);
	
	if (ground_num != HexapodConst::LEG_NUM) 
	{
		//全脚接地なら探索候補点を探さなくていい
		if (_SearchPossibleLegPosition.PossibleLegPoint_Rotation() == -1) { std::cout << "PossibleLegPoint_Rotation_Error\n"; }//旋回時の脚位置を求める 回転方向への移動量（角度）が大きいものが選ばれる
	}

	for (int i = 0; i < HexapodConst::LEG_NUM; ++i)
	{
		const int _leg_state_num = _SearchPossibleLegPosition.getLegState(i) - 1;	//配列には脚状態-1の値を代入する
	
		if (_SearchPossibleLegPosition.isGround(i) == true) 
		{
			LegGroundablePointNum[i][_leg_state_num] += 1;
			LegGroundablePoint[i][_leg_state_num][0] = node.Leg[i];
		}
		else 
		{
			if (_SearchPossibleLegPosition.LegGroundablePointNum[i][_leg_state_num])
			{
				LegGroundablePointNum[i][_leg_state_num] += 1;
			}
			else { continue; }

			//0番目の点を代入
			const double _r = _SearchPossibleLegPosition.phantomX.getGlobalMyDirectionthR();
			const double _p = _SearchPossibleLegPosition.phantomX.getGlobalMyDirectionthP();
			const double _y = _SearchPossibleLegPosition.phantomX.getGlobalMyDirectionthY();
			LegGroundablePoint[i][_leg_state_num][0] = myvector::VRot(_SearchPossibleLegPosition.p_LegGroundableCandidatePoint[i][_leg_state_num][0] - _SearchPossibleLegPosition.phantomX.getGlobalCoxaJointPos(i), _p, _r, _y);			
		}
	}

	return 0;
}

//pass_2ziより前に呼び出す　階層内のグラフはあらかじめ作成してあり，現在の脚接地可能点より実現できないものは選択肢に入らない(配列visitedが最初から1になる)
void PassFinding::getGraph(const int leg_state, bool i_F[HexapodConst::LEG_NUM], int LegGroundablePointNum[HexapodConst::LEG_NUM][DISCRETE_NUM], bool visited[ComType::COM_TYPE_NUM])
{
	//マップのデータからグラフを作る
	//ノードの削除

	const char* Raise_p[HexapodConst::LEG_NUM] = { Raised_RF, Raised_RM, Raised_RR, Raised_LR, Raised_LM, Raised_LF };
	const char* comType_p[9] = { comType0, comType1, comType2, comType3, comType4, comType5, comType6, comType7, comType8 };

	for (int i = 0; i < HexapodConst::LEG_NUM; ++i) 
	{	
		//脚を置くことのできる点が存在するか確認
		if (isGrounded(leg_state, i) == true)  
		{
			//接地してる脚については確実に足場がある。
			i_F[i] = true; 
		}
		else 
		{
			int kaisou = LegState::getLegState(leg_state, i);
			kaisou -= 1;
			i_F[i] = (LegGroundablePointNum[i][kaisou] > 0);
		}
	}

	//脚が下せないならば，その脚を接地させないと実現できないノードを削除
	for (int i = 0; i < HexapodConst::LEG_NUM; ++i) 
	{	
		if (i_F[i] == false) 
		{
			// 23は BFSinHierarchyのRaise配列の大さ．つまり，全ての移動可能パターンを移動不能に設定している．
			clear_node_bfs(Raise_p[i], 23, visited);
		}
	}
	
	int COMtype = LegState::getComPatternState(leg_state);
	int n = 9;
	if (COMtype == 0) { n = 18; }	//重心位置（使うのは重心位置タイプ）によっても実現できないノードがあるため，それを削除
	clear_node_bfs(comType_p[COMtype], n, visited);
}

void PassFinding::pass_transitions_2zi(int leg_state, bool* visited, int ret_2_transition_v[ComType::COM_TYPE_NUM][6], int* passnum)
{
	//ret_2_transition_v[i][0]　は現在の2次位置vからiノードまでのコスト
	//ret_2_transition_v[i][1]　がv，　ret_2_transition_v[i][2]以降が経路

	//なぜか i < 35 おそらくComType::COM_TYPE_NUMと同じで36だと思うのだが
	for (int i = 0; i < 35; i++) 
	{
		for (int ii = 0; ii < 6; ii++) 
		{
			ret_2_transition_v[i][ii] = 0; 
		}
	}

	char cost[36] = { 0 };	//vからiまでのコストを保存
	char prev[36] = { 0 };	//iの一個前のノード番号を保存

	int v = iHX2[isGrounded(leg_state, 0)][isGrounded(leg_state, 1)][isGrounded(leg_state, 2)][isGrounded(leg_state, 3)][isGrounded(leg_state, 4)][isGrounded(leg_state, 5)];
	bfs_in_hierarchy(v, prev, cost, visited);

	int count_ret_2 = 0;	//現在のノードから遷移できるノード数
	int new_v;
	const int shift_leg_bit[HexapodConst::LEG_NUM] = { 0, 4, 8, 12, 16, 20 };	//4bitずつ
	const int v_bit = 0b1000;	//vは遊脚の組み合わせ(2次位置)のこと　なぜvかは知らん ←　知っとけ

	for (int i = 0; i < ComType::COM_TYPE_NUM; ++i)
	{
		if (cost[i] > 0)
		{
			//costが0はスタート，-1は遷移できないノード

			ret_2_transition_v[count_ret_2][0] = cost[i];	//ここでcostを代入
			new_v = i;

			for (int j = 0; j <= cost[i]; ++j)
			{	
				//探索した経路を保存
				int cond = leg_state;

				for (int k = 0; k < HexapodConst::LEG_NUM; ++k)
				{
					if (HX2[new_v][k])
					{
						cond |= (v_bit << shift_leg_bit[k]);	//接地脚ならばその脚の4bit目を1にする
					}
					else
					{
						cond &= ~(v_bit << shift_leg_bit[k]);	//遊脚ならばその脚の4bit目を0
					}
				}

				//ret_2_transition_v[count_ret_2][ cost[i] - j + 1 ] = new_v;	//途中の経路で出てくるノードを保存
				ret_2_transition_v[count_ret_2][cost[i] - j + 1] = cond;	//途中の経路で出てくるノードを保存
				new_v = prev[new_v];

				if (new_v < 0)
				{ 
					//スタートまで遡ったらbreak
					break;
				}
			}

			++count_ret_2;
		}
	}
	*passnum = count_ret_2;
}

//特に変更する必要もない関数
void init_no_use(std::string filename) 
{
	std::ifstream ff(filename);

	if (ff.fail()) 
	{
		std::cerr << "Failed to open file." << filename <<std::endl;
	}

	std::string str;

	for (int i = 0; i < 7; ++i) {
		for (int j = 0; j < 7; ++j) {
			for (int k = 0; k < 7; ++k) {
				for (int l = 0; l < 7; ++l) {
					for (int m = 0; m < 7; ++m) {
						for (int n = 0; n < 7; ++n) {
							if (getline(ff, str)) {
								no_use_kaisou[i][j][k][l][m][n] = atoi(str.c_str());
								//no_use_kaisou[i][j][k][l][m][n] = atof(str.c_str());
							} else {
								std::cout << "k_no_use 失敗";
								std::cin >> str;
							}
						}
					}
				}
			}
		}
	}
}

void PassFinding::initHX2() 
{
	int D_HX2[36][6] = 
	{
	{1,1,1,1,1,1},		//HX[0]
	{1,0,1,1,1,1},		//HX[1]
	{1,1,0,1,1,1},		//HX[2]
	{1,1,1,0,1,1},		//HX[3]
	{1,1,1,1,0,1},		//HX[4]
	{1,1,1,1,1,0},		//HX[5]
	{0,1,1,1,1,1},		//HX[6]
	{0,1,1,0,1,1},		//HX[7]
	{1,1,0,1,1,0},		//HX[8]
	{1,0,1,1,0,1},		//HX[9]
	{1,0,1,0,1,1},		//HX[10]
	{1,1,0,1,0,1},		//HX[11]
	{1,1,1,0,1,0},		//HX[12]
	{0,1,1,1,0,1},		//HX[13]
	{1,0,1,1,1,0},		//HX[14]
	{0,1,0,1,1,1},		//HX[15]
	{1,0,1,0,1,0},		//HX[16]
	{0,1,0,1,0,1},		//HX[17]
	{0,0,1,1,1,1},		//HX[18]
	{1,0,0,1,1,1},		//HX[19]
	{1,1,0,0,1,1},		//HX[20]
	{1,1,1,0,0,1},		//HX[21]
	{1,1,1,1,0,0},		//HX[22]
	{0,1,1,1,1,0},		//HX[23]
	{0,0,1,1,0,1},		//HX[24]
	{1,0,0,1,1,0},		//HX[25]
	{0,1,0,0,1,1},		//HX[26]
	{1,0,1,0,0,1},		//HX[27]
	{1,1,0,1,0,0},		//HX[28]
	{0,1,1,0,1,0},		//HX[29]
	{0,0,1,0,1,1},		//HX[30]
	{1,0,0,1,0,1},		//HX[31]
	{1,1,0,0,1,0},		//HX[32]
	{0,1,1,0,0,1},		//HX[33]
	{1,0,1,1,0,0},		//HX[34]
	{0,1,0,1,1,0}		//HX[35]
	};
	memcpy(HX2, D_HX2, sizeof(int) * 36 * 6);
}

void PassFinding::initiHX2() 
{
	iHX2[1][1][1][1][1][1] = 0;		//HX[0]
	iHX2[1][0][1][1][1][1] = 1;		//HX[1]
	iHX2[1][1][0][1][1][1] = 2;		//HX[2]
	iHX2[1][1][1][0][1][1] = 3;		//HX[3]
	iHX2[1][1][1][1][0][1] = 4;		//HX[4]
	iHX2[1][1][1][1][1][0] = 5;		//HX[5]
	iHX2[0][1][1][1][1][1] = 6;		//HX[6]
	iHX2[0][1][1][0][1][1] = 7;		//HX[7]
	iHX2[1][1][0][1][1][0] = 8;		//HX[8]
	iHX2[1][0][1][1][0][1] = 9;		//HX[9]
	iHX2[1][0][1][0][1][1] = 10;	//HX[10]
	iHX2[1][1][0][1][0][1] = 11;	//HX[11]
	iHX2[1][1][1][0][1][0] = 12;	//HX[12]
	iHX2[0][1][1][1][0][1] = 13;	//HX[13]
	iHX2[1][0][1][1][1][0] = 14;	//HX[14]
	iHX2[0][1][0][1][1][1] = 15;	//HX[15]
	iHX2[1][0][1][0][1][0] = 16;	//HX[16]
	iHX2[0][1][0][1][0][1] = 17;	//HX[17]
	iHX2[0][0][1][1][1][1] = 18;	//HX[18]
	iHX2[1][0][0][1][1][1] = 19;	//HX[19]
	iHX2[1][1][0][0][1][1] = 20;	//HX[20]
	iHX2[1][1][1][0][0][1] = 21;	//HX[21]
	iHX2[1][1][1][1][0][0] = 22;	//HX[22]
	iHX2[0][1][1][1][1][0] = 23;	//HX[23]
	iHX2[0][0][1][1][0][1] = 24;	//HX[24]
	iHX2[1][0][0][1][1][0] = 25;	//HX[25]
	iHX2[0][1][0][0][1][1] = 26;	//HX[26]
	iHX2[1][0][1][0][0][1] = 27;	//HX[27]
	iHX2[1][1][0][1][0][0] = 28;	//HX[28]
	iHX2[0][1][1][0][1][0] = 29;	//HX[29]
	iHX2[0][0][1][0][1][1] = 30;	//HX[30]
	iHX2[1][0][0][1][0][1] = 31;	//HX[31]
	iHX2[1][1][0][0][1][0] = 32;	//HX[32]
	iHX2[0][1][1][0][0][1] = 33;	//HX[33]
	iHX2[1][0][1][1][0][0] = 34;	//HX[34]
	iHX2[0][1][0][1][1][0] = 35;	//HX[35]
}

double PassFinding::limitRangeAngle(const double angle) const
{
	//atan2に合わせるため，角度を -π < angle < π にする (例えば 340°→-20°,-340°→20°)
	double _res = angle + Define::MY_PI;
	_res = fmod(_res, 2.0 * Define::MY_PI);

	if (_res < 0) 
	{
		_res += Define::MY_PI;
	}
	else 
	{
		_res -= Define::MY_PI;
	}

	return _res;
}

//安定余裕を計算
double PassFinding::Stability_Margin(myvector::SVector leg[6], int v[6]) 
{
	myvector::SVector LtoL[6], LtoG[6], COG, n, Leg[6];
	double margin = 0;
	double minmargin = 1000;
	int groundLeg;

	COG = COMPOSI;
	groundLeg = 0;

	for (int i = 0; i < 6; i++) 
	{
		if (v[i] == 1) 
		{
			Leg[groundLeg] = myvector::addVec(phantomX.getLocalCoxaJointPos(i), leg[i]);//脚先位置(ロボット座標系)
			LtoG[groundLeg] = myvector::subVec(COG, Leg[groundLeg]);//脚先→重心ベクトル

			if (groundLeg > 0) 
			{
				LtoL[groundLeg] = myvector::subVec(Leg[groundLeg], Leg[groundLeg - 1]);//脚i-1→脚iベクトル
				n = myvector::VUnit(LtoL[groundLeg]);//単位ベクトル
				margin = myvector::VCross(LtoG[groundLeg - 1],n).z;//脚i-1→脚iベクトルと重心との距離 外積計算→　http://www.sousakuba.com/Programming/gs_dot_line_distance.html
				
				if (margin < STABILITYMARGIN) 
				{
					//安定余裕が10mm以下
					return 0;
				}
				else if (margin <  minmargin ) 
				{
					minmargin = margin;
				}
			}

			groundLeg++;
		}
	}
	groundLeg--;
	LtoL[0] = myvector::subVec(Leg[0], Leg[groundLeg]);
	n = myvector::VUnit(LtoL[0]);
	margin = myvector::VCross( LtoG[groundLeg], n).z;
	
	if (margin < STABILITYMARGIN) 
	{
		return 0;
	}
	else if (margin < minmargin) 
	{
		minmargin = margin;
	}
	
	return minmargin; 
}

//波東が使ってない関数、よって三浦さんのプログラムのやつから何も変更してない

//pass_2ziで使用　　全部の脚で離散化位置に接地可能点が存在するか計算	離散化位置に存在する点の数がLGPNum その座標がLGP	
//LGPは進行方向への寄与が大きい順に並べられているが，現在はLGP[i][j][0]しか使っていない　何か有効に使えないかな190613//大木さんのやつ。。。
void PassFinding::getLegGroundablePoint(LNODE* nuwLoute, int LegGroundablePointNum[HexapodConst::LEG_NUM][DISCRETE_NUM], myvector::SVector LegGroundablePoint[HexapodConst::LEG_NUM][DISCRETE_NUM][100]) 
{
	//_SearchPossibleLegPosition.phantomX.setMyPosition(nuwLoute->global_center_of_mass);
	//_SearchPossibleLegPosition.phantomX.setTarget(m_target);
	//_SearchPossibleLegPosition.phantomX.setMyDirection(nuwLoute->pitch, nuwLoute->roll, nuwLoute->yaw);
	//_SearchPossibleLegPosition.phantomX.setLocalLeg2Pos(nuwLoute->Leg2);
	//_SearchPossibleLegPosition.phantomX.setLocalLegPos(nuwLoute->Leg);

	//_SearchPossibleLegPosition.mapData3D_MAX = mapData3D_MAX;
	//_SearchPossibleLegPosition.p_mapData3D = mapData3D;

	//if (_SearchPossibleLegPosition.calculateLegGroundablePoint() == -1)std::cout << "overFlow__calculateLegGroundablePoint\n";//脚可動範囲内の可能点を調査、移動できるところが多すぎるとエラー

	//for (int ii = 0; ii < HexapodConst::LEG_NUM; ii++) 
	//{
	//	for (int i = 0; i < DISCRETE_NUM; i++) 
	//	{
	//		//LegGroundablePointNum[ii][i] = _SearchPossibleLegPosition.LegGroundablePointNum[ii][i+1];
	//		LegGroundablePointNum[ii][i] = _SearchPossibleLegPosition.LegGroundablePointNum[ii][i];
	//		for (int iLegPosi = 0; iLegPosi < LegGroundablePointNum[ii][i]; iLegPosi++) {
	//			if (iLegPosi >= 100)break;

	//			//LegGroundablePoint[ii][i][iLegPosi] = myvector::subVec(_SearchPossibleLegPosition.p_LegGroundableCandidatePoint[ii][i + 1][iLegPosi], _SearchPossibleLegPosition.phantomX.getGlobalCoxaJointPos(ii));	//0番目にソートされた脚接地点を代入
	//			LegGroundablePoint[ii][i][iLegPosi] = myvector::subVec(_SearchPossibleLegPosition.p_LegGroundableCandidatePoint[ii][i][iLegPosi], _SearchPossibleLegPosition.phantomX.getGlobalCoxaJointPos(ii));	//0番目にソートされた脚接地点を代入

	//			if (LegGroundablePoint[ii][i][iLegPosi].z > -100) {
	//				std::cout << "ERROR10 LegGroundablePoint[ii][j].z > -100\n";
	//				LegGroundablePoint[ii][i][iLegPosi].z = -100;
	//			}
	//		}
	//	}
	//}
}
