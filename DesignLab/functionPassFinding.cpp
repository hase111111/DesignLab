#include "functionPassFinding.h"
#include <iostream>
#include <boost/thread.hpp>
#include "MapConst.h"

std::ofstream fout_log,fout1,fout1_d,fout2,fout2_d,fout3,fout3_d,fout4,fout4_d;	//ファイル出力用
std::ofstream foutA1,foutA2,foutA3,foutA4,foutA5,foutA6,foutKaisetu;	//ファイル出力用
std::ofstream Result;	//ファイル出力用 脚位置の3Dデータ

 SNode backupNode[1000];//過去の体勢
 int backupNodeNum = 0;//過去の体勢の数

 SNode functionPassFinding(const SNode& CurrentCondition, const SNode& PastCondition, const STarget _target, const MapState* _p_map_state, SNode pass_root[100], int& m_node_num)
 {
	 //マジックナンバーをまとめたもの
	 const int _loute_num_0 = 500;
	 const int _loute_num_other = 3000000;

	 const int _result_route_first = -171;

	 PassFinding _PassFindingFirst(_loute_num_0, _p_map_state);
	 PassFinding P_F1(_loute_num_other, _p_map_state);
	 PassFinding P_F2(_loute_num_other, _p_map_state);
	 PassFinding P_F3(_loute_num_other, _p_map_state);
	 PassFinding P_F4(_loute_num_other, _p_map_state);
	 PassFinding P_F5(_loute_num_other, _p_map_state);
	 PassFinding P_F6(_loute_num_other, _p_map_state);

	 // PassFindingクラスの配列． 
	 // 動的配列 vector にスマートポインタに入れたクラスを渡している．
	 // vectorにクラスやそのポインタを生で突っ込むと面倒くさいエラーが出やすいのでスマートポインタで管理
	 //std::vector<std::shared_ptr<PassFinding>> _PassFindingArray;

	 //const int _pass_finding_array_num = 6;

	 //for (int i = 0; i < _pass_finding_array_num; i++)
	 //{
		// //通常通り push_back を使えるが，スマートポインタに入れた場合はmake_??? 関数を使う必要がある
		// _PassFindingArray.push_back(std::make_shared<PassFinding>(_loute_num_other, &divideMapData));
	 //}

	 SNode _result_route[7][500];

	 //初期化
	 for (int i = 0; i < 500; i++)
	 {
		 for (int j = 0; j < 7; j++)
		 {
			 _result_route[j][i].node_height = _result_route_first;
		 }
	 }


	 //CurrentConditionを用いてルート探索を行い最適なルートを返す
	 _PassFindingFirst.CurrentConditionNum = 1;					//探索する初めの状態の数
	 _PassFindingFirst.CurrentCondition[0] = CurrentCondition;	//ルート探索の初期値(現在のノード)
	 _PassFindingFirst.PastCondition = PastCondition;			//ひとつ前のノード
	 _PassFindingFirst.setTarget(_target);						//目標

	 //深さ1,4次方向に探索
	 _PassFindingFirst.setSearchDepth(1);			//探索深さ
	 _PassFindingFirst.setSearchMode(ESearchMode::EACH_OPERATION);	//探索タイプ0:2次方向から直線移動探索,1:4次方向から,2:シングルスレッド用,ここまで大木さん,3:2次方向からその場旋回,4:4次方向から,5:1動作ごと探索,6:水平移動と上下移動を一度に探索
	 _PassFindingFirst.setResultRouteLimit(500);					//探索するノード数の上限,一度に探索する数が大きくなると変える必要ありCurrentCondition[??]も変える
	 _PassFindingFirst.setEvaluationMode(ETargetMode::NONE);		//探索したノードを評価しない
	 _PassFindingFirst.resultRoute = _result_route[0];				//探索したノードを格納するポインタ
	 _PassFindingFirst.resultOfstream = &fout1;						//出力ファイルのポインタ

	 //探索開始
	 _PassFindingFirst();
	 m_node_num = _PassFindingFirst.getNodeNum();	//生成したノードの数を記録する

	 // 1～6を初期化する
	 {
		 P_F1.CurrentCondition[0] = _result_route[0][0];	//ルート探索の初期値
		 P_F2.CurrentCondition[0] = _result_route[0][0];
		 P_F3.CurrentCondition[0] = _result_route[0][0];
		 P_F4.CurrentCondition[0] = _result_route[0][0];
		 P_F5.CurrentCondition[0] = _result_route[0][0];
		 P_F6.CurrentCondition[0] = _result_route[0][0];

		 P_F1.PastCondition = PastCondition;				//ひとつ前のノード
		 P_F2.PastCondition = PastCondition;
		 P_F3.PastCondition = PastCondition;
		 P_F4.PastCondition = PastCondition;
		 P_F5.PastCondition = PastCondition;
		 P_F6.PastCondition = PastCondition;

		 P_F1.CurrentConditionNum = 1;					//探索する状態の数
		 P_F2.CurrentConditionNum = 1;
		 P_F3.CurrentConditionNum = 1;
		 P_F4.CurrentConditionNum = 1;
		 P_F5.CurrentConditionNum = 1;
		 P_F6.CurrentConditionNum = 1;
	 }

	 //// auto &i : vector で全要素にアクセスできる．覚えとくと便利 
	 //for (auto &i : _PassFindingArray)
	 //{
		// //各ステータスを初期化する
		// i->CurrentCondition[0] = _result_route[0][0];
		// i->PastCondition = PastCondition;
		// i->CurrentConditionNum = 1;

		// //

		// //ターゲットを指定する
		// i->setTarget(target);
	 //}

		//P_F0で探索したものをP_F1~6に分けてさらに探索
		{
			int i;
			int initialNum[7] = {1,1,1,1,1,1,1};

			//output_log("初期値代入.", fout_log);

			for(i = 1; i < 500; i ++)
			{
				if (_result_route[0][i].node_height == -171) { break; }

				if(i % 6 == 0)
				{		
					P_F1.CurrentCondition[initialNum[1]] = _result_route[0][i];						//ルート探索の初期値
					P_F1.CurrentCondition[initialNum[1]].parent = &(P_F1.CurrentCondition[0]);		//親ノードへのポンインタ
					P_F1.CurrentConditionNum = ++initialNum[1];										//探索する初めの状態の数
				}
				else if(i % 6 == 1)
				{
					P_F2.CurrentCondition[initialNum[2]] = _result_route[0][i];
					P_F2.CurrentCondition[initialNum[2]].parent = &(P_F2.CurrentCondition[0]);
					P_F2.CurrentConditionNum = ++initialNum[2];
				}
				else if(i % 6 == 2)
				{
					P_F3.CurrentCondition[initialNum[3]] = _result_route[0][i];
					P_F3.CurrentCondition[initialNum[3]].parent = &(P_F3.CurrentCondition[0]);
					P_F3.CurrentConditionNum = ++initialNum[3];
				}
				else if(i % 6 == 3)
				{
					P_F4.CurrentCondition[initialNum[4]] = _result_route[0][i];
					P_F4.CurrentCondition[initialNum[4]].parent = &(P_F4.CurrentCondition[0]);
					P_F4.CurrentConditionNum = ++initialNum[4];
				}
				else if(i % 6 == 4)
				{
					P_F5.CurrentCondition[initialNum[5]] = _result_route[0][i];
					P_F5.CurrentCondition[initialNum[5]].parent = &(P_F5.CurrentCondition[0]);
					P_F5.CurrentConditionNum = ++initialNum[5];
				}
				else if(i % 6 == 5)
				{
					P_F6.CurrentCondition[initialNum[6]] = _result_route[0][i];
					P_F6.CurrentCondition[initialNum[6]].parent = &(P_F6.CurrentCondition[0]);
					P_F6.CurrentConditionNum = ++initialNum[6];
				}
			}

			if(i % 6 == 0)
			{
				P_F1.CurrentCondition[initialNum[1]] = CurrentCondition;	//ルート探索の初期値
				P_F1.CurrentCondition[initialNum[1]].node_height = 1;		//葉ノードならば1
				P_F1.CurrentConditionNum = ++initialNum[1];					//探索する初めの状態の数
			}
			else if(i % 6 == 1)
			{
				P_F2.CurrentCondition[initialNum[2]] = CurrentCondition;
				P_F2.CurrentCondition[initialNum[2]].node_height = 1;
				P_F2.CurrentConditionNum = ++initialNum[2];
			}
			else if(i % 6 == 2)
			{
				P_F3.CurrentCondition[initialNum[3]] = CurrentCondition;
				P_F3.CurrentCondition[initialNum[3]].node_height = 1;
				P_F3.CurrentConditionNum = ++initialNum[3];
			}
			else if(i % 6 == 3)
			{
				P_F4.CurrentCondition[initialNum[4]] = CurrentCondition;
				P_F4.CurrentCondition[initialNum[4]].node_height = 1;
				P_F4.CurrentConditionNum = ++initialNum[4];
			}
			else if(i % 6 == 4)
			{
				P_F5.CurrentCondition[initialNum[5]] = CurrentCondition;
				P_F5.CurrentCondition[initialNum[5]].node_height = 1;
				P_F5.CurrentConditionNum = ++initialNum[5];
			}
			else if(i % 6 == 5)
			{
				P_F6.CurrentCondition[initialNum[6]] = CurrentCondition;
				P_F6.CurrentCondition[initialNum[6]].node_height = 1;
				P_F6.CurrentConditionNum = ++initialNum[6];
			}
		}

		{
			//ターゲットをセットする
			P_F1.setTarget(_target);
			P_F2.setTarget(_target);
			P_F3.setTarget(_target);
			P_F4.setTarget(_target);
			P_F5.setTarget(_target);
			P_F6.setTarget(_target);
		}


		//2次,4次移動のルート全探索
			
		//評価値
		double Priority[6] = {0};
		double Second[6] = { 0 };
		double Third[6] = { 0 };
		double Fourth[6] = { 0 };
		double MAX_Priority = 0;
		double MAX_Second = 0;
		double MIN_Priority = 0;
		double MIN_Second = 0;
		double MAX_Third = 0;
		double MIN_Third = 0;
		double MAX_Fourth = 0;
		double MIN_Fourth = 0;

		int _search_depth = 4;

		P_F1.setSearchDepth(_search_depth);
		P_F1.setSearchMode(ESearchMode::EACH_OPERATION);
		P_F1.resultRoute = _result_route[1];
		P_F1.resultOfstream = &foutA1;
		P_F1.setEvaluationMode(_target.TargetMode);
		P_F1.retPriority = &(Priority[0]);
		P_F1.retSecond = &(Second[0]);
		P_F1.retThird = &(Third[0]);
		P_F1.retFourth = &(Fourth[0]);

		P_F2.setSearchDepth(_search_depth);
		P_F2.setSearchMode(ESearchMode::EACH_OPERATION);
		P_F2.resultRoute = _result_route[2];
		P_F2.resultOfstream = &foutA2;
		P_F2.setEvaluationMode(_target.TargetMode);
		P_F2.retPriority = &(Priority[1]);
		P_F2.retSecond = &(Second[1]);
		P_F2.retThird = &(Third[1]);
		P_F2.retFourth = &(Fourth[1]);

		P_F3.setSearchDepth(_search_depth);
		P_F3.setSearchMode(ESearchMode::EACH_OPERATION);
		P_F3.resultRoute = _result_route[3];
		P_F3.resultOfstream = &foutA3;
		P_F3.setEvaluationMode(_target.TargetMode);
		P_F3.retPriority = &(Priority[2]);
		P_F3.retSecond = &(Second[2]);
		P_F3.retThird = &(Third[2]);
		P_F3.retFourth = &(Fourth[2]);

		P_F4.setSearchDepth(_search_depth);
		P_F4.setSearchMode(ESearchMode::EACH_OPERATION);
		P_F4.resultRoute = _result_route[4];
		P_F4.resultOfstream = &foutA4;
		P_F4.setEvaluationMode(_target.TargetMode);
		P_F4.retPriority = &(Priority[3]);
		P_F4.retSecond = &(Second[3]);
		P_F4.retThird = &(Third[3]);
		P_F4.retFourth = &(Fourth[3]);

		P_F5.setSearchDepth(_search_depth);
		P_F5.setSearchMode(ESearchMode::EACH_OPERATION);
		P_F5.resultRoute = _result_route[5];
		P_F5.resultOfstream = &foutA5;
		P_F5.setEvaluationMode(_target.TargetMode);
		P_F5.retPriority = &(Priority[4]);
		P_F5.retSecond = &(Second[4]);
		P_F5.retThird = &(Third[4]);
		P_F5.retFourth = &(Fourth[4]);

		P_F6.setSearchDepth(_search_depth);
		P_F6.setSearchMode(ESearchMode::EACH_OPERATION);
		P_F6.resultRoute = _result_route[6];
		P_F6.resultOfstream = &foutA6;
		P_F6.setEvaluationMode(_target.TargetMode);
		P_F6.retPriority = &(Priority[5]);
		P_F6.retSecond = &(Second[5]);
		P_F6.retThird = &(Third[5]);
		P_F6.retFourth = &(Fourth[5]);

		boost::thread Thread_P_F1(P_F1);	//スレッドによる探索
		boost::thread Thread_P_F2(P_F2);
		boost::thread Thread_P_F3(P_F3);
		boost::thread Thread_P_F4(P_F4);
		boost::thread Thread_P_F5(P_F5);
		boost::thread Thread_P_F6(P_F6);

		Thread_P_F1.join();
		Thread_P_F2.join();
		Thread_P_F3.join();
		Thread_P_F4.join();
		Thread_P_F5.join();
		Thread_P_F6.join();

		m_node_num += P_F1.getNodeNum();
		m_node_num += P_F2.getNodeNum();
		m_node_num += P_F3.getNodeNum();
		m_node_num += P_F4.getNodeNum();
		m_node_num += P_F5.getNodeNum();
		m_node_num += P_F6.getNodeNum();

		int optimalRouteInd = 1;

		//1スレッド目の評価値
		switch(_target.TargetMode)
		{
		case ETargetMode::STRAIGHT_VECOTR:
		case ETargetMode::STRAIGHT_POSITION:
		case ETargetMode::TURN_ON_SPOT_DIRECTION:
		/*case 7:*/

			//max,max
			MAX_Priority = Priority[0];
			MAX_Second = Second[0];
			if(/*target.TargetMode == 7 || */_target.TargetMode == ETargetMode::TURN_ON_SPOT_DIRECTION)
			{
				MIN_Third = Third[0];
				if(false/*target.TargetMode == 7*/) 
				{
					MAX_Fourth = Fourth[0];
				}
			}
			break;

		case ETargetMode::TURN_ON_SPOT_ANGLE:
			//min,max
			MIN_Priority = Priority[0];
			MAX_Second = Second[0];
			break;

		case ETargetMode::TURN_DIRECTION:
			MIN_Priority = Priority[0];
			MAX_Second = Second[0];
			MIN_Third = Third[0];
			MAX_Fourth = Fourth[0];
			break;

		case ETargetMode::TURN_ANGLE:
		/*case 8:*/
			//max,min
			MAX_Priority = Priority[0];
			MIN_Second = Second[0];
			break;

		default:
			break;
		}

		//他のスレッドと評価を比較
		for(int i = 1; i < 6; i++)
		{
			switch(_target.TargetMode)
			{
			case ETargetMode::STRAIGHT_VECOTR:
			case ETargetMode::STRAIGHT_POSITION:
				//max,max
				if(Priority[i] > MAX_Priority)
				{
					MAX_Priority = Priority[i];
					MAX_Second = Second[i];
					MIN_Third = Third[i];
					optimalRouteInd = i+1;
				}
				else if(Priority[i] == MAX_Priority)
				{
					if(Second[i] > MAX_Second)
					{
						MAX_Second = Second[i];;
						MIN_Third = Third[i];
						optimalRouteInd = i+1;
					}
				}
				break;
			case ETargetMode::TURN_ON_SPOT_ANGLE:
				//min,max
				if(Priority[i] < MIN_Priority)
				{
					MIN_Priority = Priority[i];
					MAX_Second = Second[i];
					optimalRouteInd = i+1;
				}
				else if(Priority[i] == MIN_Priority)
				{
					if(Second[i] > MAX_Second)
					{
						MAX_Second = Second[i];
						optimalRouteInd = i+1;
					}
				}
				break;
			case ETargetMode::TURN_DIRECTION:
				//max,min
				if(MIN_Priority <= 0 && Priority[i] <= 0 && MIN_Priority < Priority[i])
				{
					MIN_Priority = Priority[i];
					MAX_Second = Second[i];
					MIN_Third = Third[i];
					MAX_Fourth = Fourth[i];
					optimalRouteInd = i+1;

				}
				else if (MIN_Priority <= 0 && Priority[i] <= 0 && MIN_Priority == Priority[i]) 
				{
					if (Second[i] > MAX_Second) 
					{
						MAX_Second = Second[i];
						MIN_Third = Third[i];
						MAX_Fourth = Fourth[i];
						optimalRouteInd = i + 1;
					}
					else if (Second[i] == MIN_Second && MIN_Third > Third[i])
					{
							MIN_Third = Third[i];
							MAX_Fourth = Fourth[i];
							optimalRouteInd = i + 1;
					}
					else if (Second[i] == MIN_Second && MIN_Third == Third[i] && MAX_Fourth < Fourth[i]) 
					{
						MAX_Fourth = Fourth[i];
						optimalRouteInd = i + 1;
					}
				}
				else if (MIN_Priority > 0 && Priority[i] <= 0) 
				{
					MIN_Priority = Priority[i];
					MAX_Second = Second[i];
					MIN_Third = Third[i];
					MAX_Fourth = Fourth[i];
					optimalRouteInd = i + 1;
				}
				else if (MIN_Priority > 0 && Priority[i] > 0 && MIN_Priority > Priority[i])
				{
					MIN_Priority = Priority[i];
					MAX_Second = Second[i];
					MIN_Third = Third[i];
					MAX_Fourth = Fourth[i];
					optimalRouteInd = i + 1;
				}
				else if (MIN_Priority > 0 && Priority[i] > 0 && MIN_Priority == Priority[i])
				{
					if (Second[i] > MAX_Second) 
					{
						MAX_Second = Second[i];
						MIN_Third = Third[i];
						MAX_Fourth = Fourth[i];
						optimalRouteInd = i + 1;
					}
					else if (Second[i] == MIN_Second && MIN_Third > Third[i]) 
					{
						MIN_Third = Third[i];
						MAX_Fourth = Fourth[i];
						optimalRouteInd = i + 1;
					}
					else if (Second[i] == MIN_Second && MIN_Third == Third[i] && MAX_Fourth < Fourth[i]) 
					{
						MAX_Fourth = Fourth[i];
						optimalRouteInd = i + 1;
					}
				}
				break;

			case ETargetMode::TURN_ON_SPOT_DIRECTION:
				//max,max
				if(Priority[i] > MAX_Priority)
				{
					MAX_Priority = Priority[i];
					MAX_Second = Second[i];
					MIN_Third = Third[i];
					optimalRouteInd = i+1;
				}
				else if(Priority[i] == MAX_Priority)
				{
					if(Second[i] > MAX_Second)
					{
						MAX_Second = Second[i];;
						MIN_Third = Third[i];
						optimalRouteInd = i+1;
					}
					else if(Second[i] == MAX_Second && Third[i] < MIN_Third)
					{
						MIN_Third = Third[i];
						optimalRouteInd = i+1;
					}
				}
				break;

			//case 7:
			//	//max,max
			//	if(Priority[i] > MAX_Priority){
			//		MAX_Priority = Priority[i];
			//		MAX_Second = Second[i];
			//		MIN_Third = Third[i];
			//		MAX_Fourth = Fourth[i];
			//		optimalRouteInd = i+1;
			//	}else if(Priority[i] == MAX_Priority){
			//		if(Second[i] > MAX_Second){
			//			MAX_Second = Second[i];;
			//			MIN_Third = Third[i];
			//			MAX_Fourth = Fourth[i];
			//			optimalRouteInd = i+1;
			//		}else if(Second[i] == MAX_Second && Third[i] < MIN_Third){
			//			MIN_Third = Third[i];
			//			MAX_Fourth = Fourth[i];
			//			optimalRouteInd = i+1;
			//			if(Third[i] == MIN_Third && Fourth[i] > MAX_Fourth){
			//				Fourth[i] = MAX_Fourth;
			//				optimalRouteInd = i+1;
			//			}
			//		}
			//	}
			//	break;

			case ETargetMode::TURN_ANGLE:
			/*case 8:*/
				
			//max,min
				if(Priority[i] > MAX_Priority)
				{
					MAX_Priority = Priority[i];
					MIN_Second = Second[i];
					optimalRouteInd = i+1;
				}
				else if(Priority[i] == MAX_Priority)
				{
					if(Second[i] < MIN_Second)
					{
						MIN_Second = Second[i];
						optimalRouteInd = i+1;
					}
				}
				break;
			default:
				break;
			}
		}

		SNode pass_kekka[20];

		//最も評価の高いルートをコピー
		int count_pass = 0;

		for (int i = 0; i < 20; i++) 
		{
			pass_root[i] = pass_kekka[i] = _result_route[optimalRouteInd][i];

			if (i != 0) 
			{
				if (_result_route[optimalRouteInd][i].node_height == -171)
				{
					break;
				}
				else
				{
					++count_pass;
				}
			}
		}

		//PastCondition = pass_kekka[0];

		myvector::SVector trueCOM;		//補間のための重心位置
		myvector::SVector trueLeg[6], trueLeg2[6];		//補間のための脚位置

		myvector::SVector retCOM;		//重心位置　返り値


		trueCOM = myvector::addVec(pass_kekka[0].global_center_of_mass, retCOM);

		for(int iLeg = 0; iLeg < 6; iLeg++)
		{
			trueLeg[iLeg]  = myvector::subVec(pass_kekka[1].Leg[iLeg]  ,retCOM);
			trueLeg2[iLeg] = myvector::subVec(pass_kekka[1].Leg2[iLeg] ,retCOM);
		}



		int retDepth = 1;
		static int pass_kekka_cnt = 1;

		//自分の現在の状態をCurrentConditionに代入
		if(pass_kekka[retDepth].node_height <= 0)
		{
			//output_log("ルート短すぎ.", fout_log);
		}

		//----------------null動作を飛ばす----------------//
		//現在の状態とi動作後を比較、同じ場合はさらにi+1,i+2と調べて、異なるノードになるまで深くする。
		for (int i = 0; i < count_pass; ++i) 
		{
			if (isNodeEqual(pass_kekka[i], pass_kekka[i + 1])) 
			{
				++retDepth;
			}
			else 
			{
				break;
			}
		}

		//脚の水平移動は移動を伴わないから飛ばす。
		//if (pass_kekka[retDepth].debug % 100/10 == 4) ++retDepth;

		if (retDepth > 5) { retDepth = 5; }
		//--------------------------------------------------//

		backupNode[backupNodeNum++] = pass_kekka[4/*retDepth*/];

		return pass_kekka[retDepth];
}


bool isEqualNode(SNode node1,SNode node2)
{
	//脚状態と重心位置がほぼ同じなら，同じノードとして扱う

	if (node1.leg_state == node2.leg_state) 
	{
		if (myvector::VMag(myvector::subVec(node1.global_center_of_mass, node2.global_center_of_mass)) < 0.000001) 
		{
			return true;
		}
	}

	return false;

	/*
	if(node1.COM_type == node2.COM_type){
		if(myvector::VMag(myvector::subVec(node1.global_center_of_mass, node2.global_center_of_mass)) < 0.000001){
			if(node1.kaisou == node2.kaisou){
				if(node1.v == node2.v){
					return 1;
				}
			}
		}
	}
	return 0;
	*/
}
