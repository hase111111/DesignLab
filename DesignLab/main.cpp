#include "SystemMain.h"
#include "LegState.h"

int no_use_kaisou[leg_state::DISCRETE_NUM][leg_state::DISCRETE_NUM][leg_state::DISCRETE_NUM][leg_state::DISCRETE_NUM][leg_state::DISCRETE_NUM][leg_state::DISCRETE_NUM];

int main(void)
{
	//メインシステムクラスを作成する．ここでコンストラクタが呼ばれる．
	SystemMain _sys;

	//メインの処理を行う．
	_sys.main();

	// main() 関数が終了したら，プログラムを終える．
	return 0;
}

//! @file main.cpp
//! @brief プログラムのエントリポイント
//! @author 長谷川
//! @date 2023/06/30
//! @details　今後このソースファイルを使う人のためにC++で大きなプロジェクトを作成するときのルールをここに定義しておく．<br>
//! <br>
//!●命名規則<br>
//!	　・スネークケース snake_case<br>
//!	　・キャメルケース camelCase<br>
//!	　・パスカルケース PascalCase<br>
//!		変数やクラスの名前を決めるときは上記のどれかを使って決めるのが基本です．<br>
//!		例えば int NumBer; とか class Sample_Class;だとかは避けましょう<br>
//! <br>
//!		私は<br>
//!			・変数			スネークケース (my_var)<br>
//!			・ローカル変数	先頭に _ (アンダーバー) + スネークケース (_local_var)<br>
//!			・クラス		パスカルケース (MyClass)<br>
//!			・メンバ変数	先頭に m_ + スネークケース (m_my_var)<br>
//!			・関数名		キャメルケース (myFunction)<br>
//!			・列挙子		先頭にE + パスカルケース(EMyEnum)<br>
//!			・構造体		先頭にS + パスカルケース(SMyStruct)<br>
//! <br>
//!		のようにしています．<br>
//!		また，変数名をローマ字で書いたり，過度に省略したりするのはやめましょう(例えば int getsuyoubi; とか std::string tkg;とか)<br>
//!		固有名詞は全部書く，逆にinit(初期化)やnum(ナンバー)などよく使うものは省略すべきです．func(関数)等は見ればわかるので書かないようにしましょう<br>
//!		多少長くなっても何しているか分かりやすいほうが，見やすくなります(特に関数において)<br>
//! <br>
//!		よく使う省略後<br>
//!			・res (result 結果)					・num (number 数)<br>
//!			・cnt (count カウント)				・pos (position 位置)<br>
//!			・rot (rotation 回転)				etc...<br>
//! <br>
//!		関数においては動詞→目的語となるように付けると混乱を防げると思います．(例えば getData() とか makeMap() とか)<br>
//!		bool型の場合は質問形式で命名しましょう．bool _is_empty = false; とか bool isOddNumber(const int _num); のようにです<br>
//! <br>
//!		なぜこのような命名規則を導入するのかというと，自分の書いたプログラムを他人に読んでもらうとき，命名方法が適当だと非常に読みづらくなるためです．<br>
//!		自分なりのルールを決めて分かりやすく整理しておきましょう<br>
//! <br>
//!●クラスについて<br>
//!		基本的にメンバ変数は全て private: に入れておくのが普通だと思いまず．<br>
//!		面倒ですが値をとったり設定したりしたいなら，get??? や set??? 関数を使いましょう<br>
//!		生のメンバ変数の値を変更するのはやめるべきです<br>
//! <br>
//!		また一つのクラスに大量の機能を追加しないでください．かつてのPassFindingクラスはグラフ探索＋ファイル入出力＋コマンド送信等<br>
//!		多量の機能を持っていましたが，可読性が下がるのであまりよくないのではないかと感じます．<br>
//!		機能を細かく分けてプログラムを書くべきでしょう<br>
//! <br>
//!●関数について<br>
//!		関数を宣言した場合，その関数の引数や機能についてコメントを付けましょう．<br>
//!		後から見た時に何をすればよいか分かりやすくて助かります．<br>
//!		注釈はヘッダーに書いておけば，他のファイルでも確認できるようになります<br>
//! <br>
//!		値を返したいときは基本は戻り値を使いましょう(いわゆる return ???;ってやつです)<br>
//!		不必要なポイント渡しはプログラムを無駄に煩雑にします． <br>
//!		また，C++においては変数はポインタ渡し(*)ではなくて参照渡し(&)を使ったほうが良い気がします(要審議)<br>
//!		参照渡しは安全で制約の厳しいポインタ渡しのような物です．使いやすく見やすいのでおすすめです．<br>
//!		このプログラムでは複数の戻り値を必要とする関数においてポインタ渡しが多用されていましたが<br>
//!		C++ なら tuple を使うか，せめて引数の参照渡しにしてほしい感じがあります．(個人の感想です)<br>
//! <br>
//!	また変更しない引数はconstを付けておきましょう．<br>
//!	同様にメンバ変数を変更しないメンバ関数にもconstを付けましょう<br>
//! <br>
//!　関数の引数においてマジックナンバーは避けましょう．<br>
//!	マジックナンバーとは直に数値を使うことで，本人以外にその値が何を表しているのか理解できないものを指します．<br>
//!　変数や列挙子やコメントなどを用いて変数の意味を説明していただけると助かります<br>
//! <br>
//!●定数について<br>
//!	#define を使うのはやめましょう C++においては const static な変数(静的で定数のメンバ変数)を使うほうがよいと思います．(たぶん)<br>
//!	詳しくは Common / Define.hに書いておきます．<br>
//!	どうしても書きたいのならばcppファイルに書きましょう．hファイルに書き込むと他のファイルのコンパイルにも多大な影響を及ぼして危険です<br>
//!	C++には constexprという機能があるのでこれを使うのもありだと思います<br>
//! <br>
//!●配列について<br>
//!	C言語において通常使われる int _array[10]; のような配列の使用は最小限にしたほうが良いと思います．<br>
//!	なぜなら，このプログラムでも過去にそれによるスタックオーバーフローが起きていたためです．<br>
//!	time[1000]という配列に時間を記録していたため， 1000回以上のループができなくなっていました．<br>
//!	とりあえず大きな配列を作っておいて，その中にデータを記録するのは拡張性が低いし，見つけづらいバグの温床になります．<br>
//!	std::vector (動的な配列) を積極的に使いたいと思っています．<br>
//!	new/delete は個人的には絶対に使いたくありません．(エラーが怖いので)<br>
//!	また，変数を int x1,x2,x3,x4;のように大量に宣言するのはやめましょう．その場合は普通に配列でいいです int x[4];  <br>
//! <br>
//!●for文について<br>
//!	for文に使用する変数は i → j → k としましょう．通例そのような命名をすることが多いです．<br>
//!	また，for文を多重に使用するととても読みづらくなります．4重5重にfor文が重なる場合は設計を見直してください<br>
//! <br>
//!●ヘッダー(.h)について<br>
//!	ヘッダーにはむやみに #include を書かないようにしましょう．<br>
//!	コンパイルがクソ時間かかります．.cppのほうでインクルードしましょう<br>
//!	またグローバル変数を使ったり，externを使ったりするのは避けてください．多くの場合スパゲッティコードの原因になります．<br>
//!	設計をよく考えれば使わずに済むはずです．どうしても必要な場合は一つのヘッダにまとめておいてください．<br>
//!	散らされると探すのが本当に大変です．<br>
//!	リンケージ(externして他のファイルでも関数を使うこと)したい場合は名前空間を使うのはどうでしょうか?(要検討)<br>
//! <br>
//!●小数について<br>
//!	小数をあらわす事のできる変数の形は float と double の二つがあります．<br>
//!	float のほうがサイズが小さいので探索に向いているハズ，floatで統一して書いていきます．(要検討)<br>
//! <br>
//!●列挙体について<br>
//!	整数型の引数を使って関数の機能を分けたいとき，例えば以下のような関数を使いたいとき，<br>
//!	int calculateNumber(const int _num1, const int _num2, const int mode)()<br>
//!	{<br>
//!		if(mode == 1){～何かしらの処理～}<br>
//!		else if(mode == 2){～何かしらの処理～}<br>
//!		else if(mode == 3){～何かしらの処理～}<br>
//!	}<br>
//!	このような場合は列挙体を使うことをお勧めします．何故ならば，非常に読みやすくなるからです．<br>
//!	詳しくは Target.hを確認して下さい<br>
//! <br>
//!●構造体について<br>
//!	C++で書かれているのでわざわざtypedefしなくてもOKです<br>



////時間計測用の定義と関数
//#if defined(_WIN32) || (_MSC_VER)
//#define VC_MODE
//#endif
//
//#ifdef VC_MODE
//#include <sys/types.h>
//#include <sys/timeb.h>
//#else
//#include <sys/time.h>
//#endif
//
//#ifdef VC_MODE
//inline double seconds() {
//	_timeb tp;
//	_ftime_s(&tp);
//	return ((double)tp.time + (double)tp.millitm / 1000.0);
//}
//#else
//inline double seconds() {
//	struct timeval tp;
//	struct timezone tzp;
//	int i = gettimeofday(&tp, &tzp);
//	return ((double)tp.tv_sec + (double)tp.tv_usec * 1.e-6);
//}
//#endif
//
//#include "MapCreator.h"
//#include "functionPassFinding.h"
//#include "phantomxCommander.h"
//#include "S_NE.h"
//#include "Define.h"
//#include "LogFileIO.h"
//#include "TimeFileIO.h"
//#include "NodeFileIO.h"
//#include "CmdIO.h"
//#include "SimulateResult.h"
//#include "GraphicDataBroker.h"
//#include "GraphicSystem.h"
//#include "NodeEdit.h"
//#include "SystemMain.h"
//#include "MyMath.h"
//
//
////#define COMMUNICATION	//実機実験（phantomXを動かす）場合define
//
//int main(void)
//{	
//	//メインシステムクラスを作成する．ここでコンストラクタが呼ばれる．
//	SystemMain _sys;
//
//	//メインの処理を行う．
//	_sys.main();
//
//	// main() 関数が終了したら，プログラムを終える．
//	return 0;
//
//	// Hexapodの初期化を行う．
//	Hexapod::makeLegROM_r();
//
//	//マップを生成する．
//	MapState _MapState;
//	_MapState.init(EMapCreateMode::Flat, MapCreator::OPTION_PERFORATED | MapCreator::OPTION_STEP, false);
//
//	//仲介人作成
//	GraphicDataBroker _Broker;
//	_Broker.setMapState(_MapState);
//	std::vector<SNode> answer; //結果を格納するベクタ
//
//	//画像表示
//	GraphicSystem _Graphic;
//	_Graphic.init(&_Broker);
//	boost::thread _Th_Graphic(&GraphicSystem::main, &_Graphic);	//画像表示処理を別スレッドで立ち上げる．
//
//	// 移動方向の設定
//	STarget Target;
//	Target.TargetDirection = my_vec::SVector(0, 1, 0);			//目標方向x,y,z(直進移動)
//	Target.TargetPosition = my_vec::SVector(0, 10000, 0);		//目標位置
//	Target.TargetRotation = my_vec::SVector(0, 0, 1);			//目標旋回方向P,R,Y(現在Y方向しか考えていない値は1が左回転、-1が右回転)
//	Target.TargetAngle = my_vec::SVector(0, 0, my_math::MY_FLT_PI / 2.0f);		//目標旋回角度(胴体の角度)
//	Target.RotationCenter = my_vec::SVector(-10000, 0, 0);		//回転中心x,y,z
//	Target.TargetMode = ETargetMode::TURN_DIRECTION;			//1:直線移動（ベクトル）,2:直線移動（座標）,3:その場旋回（回転方向）,4:その場旋回（回転角度）,5:旋回大（方向）,6:旋回大（角度）,7:旋回小（方向）,8:旋回小（角度）
//	Target.TurningRadius = abs(Target.RotationCenter.x);
//	Target.TurningRadius = Target.RotationCenter.length();	//旋回半径 これだと、原点と旋回中心との距離,いや更新してないから現状では一応よさげ、y軸上を直進させるなら、ｘの距離つまり、旋回中心とy軸との距離
//
//	//過去と未来の体勢の初期化
//	SNode CurrentCondition;
//	node_edit::initNode(CurrentCondition, false);
//	SNode NewCondition;							//現在のロボットの状態
//	SNode PastCondition = CurrentCondition;		//1つ前のロボットの状態
//	PastCondition.leg_state = 0;
//	std::vector<SNode> PreviousNode;			//前のノードを何個かログをとっておく．同じ動作をしていないかのチェックに使う
//	SNode pass_root[100];						//ルート
//
//	//ログファイルの作成
//	LogFileIO _LogFileIO;
//	if (_LogFileIO.openLogFile() == false) { return -1; }
//
//	//シミュレーションの結果を入れるクラス
//	SimulateResult _SimuRes;
//	bool continue_simulation = true;
//
//	//コマンドラインに書き込むためのクラス
//	CmdIO _CmdIO;
//
//#ifdef COMMUNICATION
//	//phantomXへの通信クラス
//	phantomxCommander commander;
//
//	//現在の体勢を送信
//	//stampLegNum = 0;
//	for (int i = 0; i < 6; i++) {
//		commander.endpoints[i] = my_vec::VGet(CurrentCondition.leg_pos[i].y, CurrentCondition.leg_pos[i].x, -CurrentCondition.leg_pos[i].z);
//		/*if (CurrentCondition.leg_pos[i].z + CurrentCondition.global_center_of_mass.z == 0) {
//			stampLegNum++;
//		}*/
//	}
//	commander.SecurelySendEndPoints(3000, 500);
//
//#endif
//
//	std::cout << "文字入力でシミュレーションを開始します。\n\n";
//	std::cout << "phantomXの電源を切らずに[SHIFT]+[CAPSLOCK]長押し（終了するまで）で安全に終了できます。\n\n";
//
//	int m_loop_counter = 0;//実際に行ったシミュレーション回数
//
//	for (m_loop_counter; m_loop_counter < Define::SIMURATE_NUM; m_loop_counter++) 
//	{
//		std::cout << "シミュレーション　" << (m_loop_counter + 1) << "回目" << std::endl;
//		std::cout << "現在の歩行成功回数\t" << _SimuRes.getClearNum() << std::endl;
//		_LogFileIO.addLogStringWithInt(m_loop_counter + 1);
//
//		//1シミュレーション終わるごとに、ロボットの位置を変更する。
//		if (m_loop_counter != 0) 
//		{
//			node_edit::initNode(CurrentCondition, true);
//			PastCondition = CurrentCondition;
//			PastCondition.leg_state = 0;
//		}
//		
//		int _distance_move_y = (int)CurrentCondition.global_center_of_mass.y;	//ファイル出力用　1シミュレーションでの移動距離の最大最小を記録する
//
//		if(Define::FLAG_GRAPHIC_AVAILABLE == true)
//		{
//			_Broker.pushNode(CurrentCondition);
//		}
//
//		//時間とノードの出力のためのvector
//		std::vector<double> _time;
//		std::vector<int> _node_num;
//
//		for (int m_simu_counter = 0; m_simu_counter < Define::GATE_PATTERN_GENERATE_NUM; m_simu_counter++) 
//		{
//			//1度のシミュレーションでの最大動作数
//			_LogFileIO.addLogStringWithNode(m_simu_counter, CurrentCondition);
//
//			if (Define::FLAG_GRAPHIC_AVAILABLE == true)
//			{
//				_Broker.pushNode(CurrentCondition);
//			}
//
//			//直進移動のシミュレーション用
//			Target.RotationCenter.y = CurrentCondition.global_center_of_mass.y; 
//			Target.RotationCenter.z = CurrentCondition.global_center_of_mass.z;
//
//			CurrentCondition.delta_comz = 0;
//			CurrentCondition.target_delta_comz = 0;
//
//			//1動作ごとに現状をコマンドラインに表示
//			std::cout << "シミュレーション" << (m_loop_counter+1) <<"回目" <<  std::endl;
//			std::cout << "成功回数" << _SimuRes.getClearNum() << "回" << std::endl;
//			std::cout << "総動作数" << _SimuRes.getGatePatternGenerateSum() << std::endl;
//			std::cout << m_simu_counter << "動作目" << std::endl;
//			std::cout << "シミュレーション実行時間" << _SimuRes.getGatePatternGenerateTimeSum() << "秒" << std::endl;
//
//			//時間計測用
//			double _elapse;
//			_elapse = seconds();	
//
//			//グラフ探索で歩容パターン生成
//			_node_num.push_back(0);
//			NewCondition = functionPassFinding(CurrentCondition, Target, &_MapState, pass_root, _node_num.back());
//			
//			//グラフ探索にかかった時間を記録する
//			_time.push_back(seconds() - _elapse);
//			
//			// NewConditionに情報を記録する
//			NewCondition.last_node_num = _node_num.back();//探索で得られたノード数
//			NewCondition.time = (float)_time.back();//探索でかかった時間
//
//			//コマンドラインに結果を表示する
//			_CmdIO.outputNode(NewCondition);
//
//			//シミュレーション結果を更新する
//			_SimuRes.countupGatePatternGenerateSum();
//			_SimuRes.updateGatePatternGenerateTime(_time.back());
//			_distance_move_y += (int)(NewCondition.global_center_of_mass.y - CurrentCondition.global_center_of_mass.y);
//
//
//			//遷移可能なノードが存在しない場合，処理をやめループを一つ抜ける
//			if (NewCondition.node_height < 0) 
//			{
//				std::cout << std::endl << "グラフ探索で，歩容パターンが得られなかったため，探索を終了します.\n\n" << std::endl;
//				
//				//結果を記録する
//				_SimuRes.countupFailedByNoGatePattern();
//				_LogFileIO.addLogStringWithNode(m_simu_counter, NewCondition);
//				_LogFileIO.addLogString("failed_by_no_gate_pattern");
//				break;
//			}
//
//			//ノードを後ろに追加して，30個以上溜まったら前の物を消す．ここマジックナンバーなので後でDefineに追加したほうがいい
//			PreviousNode.push_back(PastCondition);
//			if (PreviousNode.size() > 30) { PreviousNode.erase(PreviousNode.begin()); }
//
//			// Conditionを更新する
//			PastCondition = CurrentCondition;
//			CurrentCondition = NewCondition;
//
//			CurrentCondition.node_height = 1;
//			CurrentCondition.parent = NULL;
//
//			//同じ動作を繰り返すと，処理をやめループを一つ抜ける
//			bool _loop_out_flag = false;
//
//			if (m_simu_counter > 30) 
//			{
//				for (auto &i : PreviousNode) 
//				{
//					if (isNodeEqual(i, CurrentCondition) && (abs(i.yaw - CurrentCondition.yaw) < 0.01f) && i.debug == CurrentCondition.debug) 
//					{
//						std::cout << "同じ動作を繰り返したため停止します" << std::endl;
//						_SimuRes.countupFailedByGatePatternLoop();
//						_LogFileIO.addLogString("failed_by_gate_pattern_loop");
//						_loop_out_flag = true;
//					}
//				}
//			}
//
//			if (_loop_out_flag == true) { break; }
//
//			//目的の距離あるくことができたなら，処理をやめループを抜ける
//			if (CurrentCondition.global_center_of_mass.y >= Define::GOAL_TAPE) 
//			{
//				std::cout << "一定距離の歩行に成功しました．" << std::endl;
//				_SimuRes.countupClearNum();
//				_LogFileIO.addLogString("clear");
//				break;
//			}
//
//			//SHIFT+CAPSLOCKキーが押されているとき 歩行終了　これをしないとファイル書き込みが完了しない
//			if (GetAsyncKeyState(VK_CAPITAL) & 1/*0x8000*/) 
//			{
//				std::cout << "一時的にbreakしないようにしてる(main.cpp 528あたり)2021/01/22" << std::endl;
//				if(m_loop_counter!=1) --m_loop_counter;//シミュレーション回数にカウントしない
//				continue_simulation = false;
//				break;
//			}
//
//		//データ送信部
//		#ifdef COMMUNICATION	
//			{ 
//				boost::mutex::scoped_lock lkm(mtxMapData);			//マップへのアクセス権
//				boost::mutex::scoped_lock lk(mtxHexapodGraphic); //ロボット体勢へのアクセス権
//
//				//現在の体勢を送信
//				//stampLegNum = 0;
//				for (int i = 0; i < 6; i++) {
//					commander.endpoints[i] = my_vec::VGet(CurrentCondition.leg_pos[i].y, CurrentCondition.leg_pos[i].x, -CurrentCondition.leg_pos[i].z);
//					/*if (CurrentCondition.leg_pos[i].z + CurrentCondition.global_center_of_mass.z == 0) {
//						stampLegNum++;
//					}*/
//				}
//				commander.SecurelySendEndPoints(3000, 500);
//			}
//		#endif
//
//		}
//
//		//シミュレーションの時間とノードの数のリストをファイルに出力する
//		TimeFileIO _TimeFileIO;
//		_TimeFileIO.outputTimeFile(_time);
//
//		NodeFileIO _NodeFileIO;
//		_NodeFileIO.outputNodeFile(_node_num);
//
//
//		_SimuRes.updateDistanceMoveY(_distance_move_y);
//
//		//if (_SimuRes.getGatePatternGenerateSum() > 1000) 
//		//{
//		//	//1000動作超えるとバグるからループを抜ける　原因不明のまま ←おそらくオーバーフロー 解決済み (長谷川)
//		//	break;
//		//}
//
//		if (continue_simulation == false) 
//		{
//			//std::cout << "シミュレーションを続行する 1 ,続行しない 0 を入力" << std::endl;
//			//std::cin >> continue_simulation;
//			//if (continue_simulation == false) break;
//		}
//	}
//
//	//ループここまで，以下結果を出力
//	
//	//シミュレーションの結果をLogファイルへ出力
//	_LogFileIO.addLogStringSimulation(m_loop_counter, _SimuRes);
//	_LogFileIO.closeLogFile();
//
//	//シミュレーションの結果をコマンドラインへ出力
//	_CmdIO.outputSimulateResult(m_loop_counter, _SimuRes);
//	
//	std::cout << m_loop_counter << "回分の歩容パターン生成シミュレーションが終了しました。\nプログラムを終了します。";
//
//	//画像表示している方の終了を待つ．
//	_Th_Graphic.join();
//
//	return 0;
//}