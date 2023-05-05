#include "pch.h"
#include "hexapodGraphic.h"
#include "mainfunction.h"
#include "functionPassFinding.h"
#include "phantomxCommander.h"
#include "S_NE.h"
#include "Define.h"
#include "LogFileIO.h"
#include "TimeFileIO.h"
#include "NodeFileIO.h"
#include "CmdIO.h"
#include "SimulateResult.h"
//#define COMMUNICATION	//実機実験（phantomXを動かす）場合define		

//	
//　今後このソースファイルを使う人のためにC++で大きなプロジェクトを作成するときのルールをここに定義しておく
//
//●命名規則
//	　スネークケース snake_case
//	　キャメルケース camelCase
//	　パスカルケース PascalCase
//	
//	変数やクラスの名前を決めるときは上記のどれかを使って決めるのが基本です．
//	例えば int NumBer; とか class Sample_Class;だとかは避けましょう
//
//	私は
//	・変数			スネークケース (my_var)
//	・ローカル変数	先頭に _ (アンダーバー) + スネークケース (_local_var)
//	・クラス		パスカルケース (MyClass)
//	・メンバ変数	先頭に m_ + スネークケース (m_my_var)
//	・関数名		キャメルケース (myFunction)
//	・列挙子		先頭にE + パスカルケース(EMyEnum)
//	・構造体		先頭にS + パスカルケース(SMyStruct)
//
//	のようにしています．
//	また，変数名をローマ字で書いたり，過度に省略したりするのはやめましょう(例えば int getsuyoubi; とか std::string tkg;とか)
//	固有名詞は全部書く，逆にinit(初期化)やnum(ナンバー)などよく使うものは省略すべきです．func(関数)等は見ればわかるので書かないようにしましょう
//	多少長くなっても何しているか分かりやすいほうが，見やすくなります(特に関数において)
// 
//	よく使う省略後
//	・res (result 結果)					・num (number 数)
//	・cnt (count カウント)				・pos (position 位置)
//	・rot (rotation 回転)				・
// 
//	関数においては動詞→目的語となるように付けると混乱を防げると思います．(例えば getData() とか makeMap() とか)
//	bool型の場合は質問形式で命名しましょう．bool _is_empty = false; とか bool isOddNumber(const int _num); のようにです
//
//	なぜこのような命名規則を導入するのかというと，自分の書いたプログラムを他人に読んでもらうとき，命名方法が適当だと非常に読みづらくなるためです．
//	自分なりのルールを決めて分かりやすく整理しておきましょう
//
//●クラスについて
//	基本的にメンバ変数は全て private: に入れておくのが普通だと思いまず．
//	面倒ですが値をとったり設定したりしたいなら，get??? や set??? 関数を使いましょう
//	生のメンバ変数の値を変更するのはやめるべきです
//
//	また一つのクラスに大量の機能を追加しないでください．かつてのPassFindingクラスはグラフ探索＋ファイル入出力＋コマンド送信等
//	多量の機能を持っていましたが，可読性が下がるのであまりよくないのではないかと感じます．
//	機能を細かく分けてプログラムを書くべきでしょう
//
//●関数について
//	関数を宣言した場合，その関数の引数や機能についてコメントを付けましょう．
//	後から見た時に何をすればよいか分かりやすくて助かります．
//	注釈はヘッダーに書いておけば，他のファイルでも確認できるようになります
//
//	値を返したいときは基本は戻り値を使いましょう(いわゆる return ???;ってやつです)
//	不必要なポイント渡しはプログラムを無駄に煩雑にします．
//	また，C++においては変数はポインタ渡し(*)ではなくて参照渡し(&)を使ったほうが良い気がします(要審議)
//	参照渡しは安全で制約の厳しいポインタ渡しのような物です．使いやすく見やすいのでおすすめです．
//	このプログラムでは複数の戻り値を必要とする関数においてポインタ渡しが多用されていましたが
//	C++ なら tuple を使うか，せめて引数の参照渡しにしてほしい感じがあります．(個人の感想です)
//
//	また変更しない引数はconstを付けておきましょう．
//	同様にメンバ変数を変更しないメンバ関数にもconstを付けましょう
//
//　関数の引数においてマジックナンバーは避けましょう．
//	マジックナンバーとは直に数値を使うことで，本人以外にその値が何を表しているのか理解できないものを指します．
//　変数や列挙子やコメントなどを用いて変数の意味を説明していただけると助かります
//
//●定数について
//	#define を使うのはやめましょう C++においては const static な変数(静的で定数のメンバ変数)を使うほうがよいと思います．(たぶん)
//	詳しくは Common / Define.hに書いておきます．
//	どうしても書きたいのならばcppファイルに書きましょう．hファイルに書き込むと他のファイルのコンパイルにも多大な影響を及ぼして危険です
//	C++には constexprという機能があるのでこれを使うのもありだと思います
//
//●配列について
//	C言語において通常使われる int _array[10]; のような配列の使用は最小限にしたほうが良いと思います．
//	なぜなら，このプログラムでも過去にそれによるスタックオーバーフローが起きていたためです．
//	time[1000]という配列に時間を記録していたため， 1000回以上のループができなくなっていました．
//	とりあえず大きな配列を作っておいて，その中にデータを記録するのは拡張性が低いし，見つけづらいバグの温床になります．
//	std::vector (動的な配列) を積極的に使いたいと思っています．
//	new/delete は個人的には絶対に使いたくありません．(エラーが怖いので)
//	また，変数を int x1,x2,x3,x4;のように大量に宣言するのはやめましょう．配列でいいです int x[4]; ． 
//
//●for文について
//	for文に使用する変数は i → j → k としましょう．通例そのような命名をすることが多いです．
//	また，for文を多重に使用するととても読みづらくなります．4重5重にfor文が重なる場合は設計を見直してください
// 
//●ヘッダー(.h)について
//	ヘッダーにはむやみに #include を書かないようにしましょう．
//	コンパイルがクソ時間かかります．.cppのほうでインクルードしましょう
//	またグローバル変数を使ったり，externを使ったりするのは避けてください．多くの場合スパゲッティコードの原因になります．
//	設計をよく考えれば使わずに済むはずです．どうしても必要な場合は一つのヘッダにまとめておいてください．
//	散らされると探すのが本当に大変です．
//	リンケージ(externして他のファイルでも関数を使うこと)したい場合は名前空間を使うのはどうでしょうか?(要検討)
//
//●小数について
//	小数をあらわす事のできる変数の形は float と double の二つがあります．
//	float のほうがサイズが小さいので探索に向いているハズ，floatで統一して書いていきます．(要検討)
//
//●列挙子について
//	整数型の引数を使って関数の機能を分けたいとき，例えば以下のような関数を使いたいとき，
//	int calculateNumber(const int _num1, const int _num2, const int mode)()
//	{
//		if(mode == 1){～何かしらの処理～}
//		else if(mode == 2){～何かしらの処理～}
//		else if(mode == 3){～何かしらの処理～}
//	}
//	このような場合は列挙子を使うことをお勧めします．何故ならば，非常に読みやすくなるからです．
//	詳しくは Target.hを確認して下さい
//
//●構造体について
//	C++で書かれているのでわざわざtypedefしなくてもOKです
//

//時間計測用の定義と関数
#if defined(_WIN32) || (_MSC_VER)
#define VC_MODE
#endif

#ifdef VC_MODE
#include <sys/types.h>
#include <sys/timeb.h>
#else
#include <sys/time.h>
#endif

#ifdef VC_MODE
inline double seconds() {
	_timeb tp;
	_ftime_s(&tp);
	return ((double)tp.time + (double)tp.millitm / 1000.0);
}
#else
inline double seconds() {
	struct timeval tp;
	struct timezone tzp;
	int i = gettimeofday(&tp, &tzp);
	return ((double)tp.tv_sec + (double)tp.tv_usec * 1.e-6);
}
#endif



void SetDefaultCondition(LNODE &node);
bool WriteNodeToFile(std::ofstream *file, int n, const LNODE& node_log);

int no_use_kaisou[DISCRETE_NUM][DISCRETE_NUM][DISCRETE_NUM][DISCRETE_NUM][DISCRETE_NUM][DISCRETE_NUM];
//float no_use_kaisou[DISCRETE_NUM][DISCRETE_NUM][DISCRETE_NUM][DISCRETE_NUM][DISCRETE_NUM][DISCRETE_NUM];

int main(void)
{
	std::string stop;

	//スレッド間の衝突回避のためのミューテックス
	boost::mutex mtxHexapodGraphic;		//phantomXの体勢データへのアクセス権
	boost::mutex mtxMapData;			//マップデータへアクセス権

	Hexapod phantomX;

	int _map_data_num = Define::MAPDATA3D_MAX;
	myvector::SVector mapData[Define::MAPDATA3D_MAX], allmapData[Define::MAPDATA3D_MAX];//ロボット座標系
	myvector::SVector mapDataBackUp[Define::MAPDATA3D_MAX];				//グローバル座標系

	// 移動方向の設定
	STarget Target;
	Target.TargetDirection = myvector::VGet(0, 1, 0);			//目標方向x,y,z(直進移動)
	Target.TargetPosition = myvector::VGet(0, 10000, 0);		//目標位置
	Target.TargetRotation = myvector::VGet(0, 0, 1);			//目標旋回方向P,R,Y(現在Y方向しか考えていない値は1が左回転、-1が右回転)
	Target.TargetAngle = myvector::VGet(0, 0, M_PI / 2.0);		//目標旋回角度(胴体の角度)
	Target.RotationCenter = myvector::VGet(-10000, 0, 0);		//回転中心x,y,z
	Target.TargetMode = ETargetMode::TURN_DIRECTION;			//1:直線移動（ベクトル）,2:直線移動（座標）,3:その場旋回（回転方向）,4:その場旋回（回転角度）,5:旋回大（方向）,6:旋回大（角度）,7:旋回小（方向）,8:旋回小（角度）
	Target.TurningRadius = abs(Target.RotationCenter.x);
	Target.TurningRadius = myvector::VMag(Target.RotationCenter);	//旋回半径 これだと、原点と旋回中心との距離,いや更新してないから現状では一応よさげ、y軸上を直進させるなら、ｘの距離つまり、旋回中心とy軸との距離

	LNODE CurrentCondition;
	SetDefaultCondition(CurrentCondition);

	//過去と未来の体勢の初期化
	LNODE NewCondition;							//現在のロボットの状態
	LNODE PastCondition;						//1つ前のロボットの状態
	PastCondition = CurrentCondition;
	PastCondition.leg_state = 0;
	std::vector<LNODE> PreviousNode;			//前のノードを何個かログをとっておく．同じ動作をしていないかのチェックに使う
	LNODE pass_root[100];						//ルート

	//ウィンドウ用変数の初期化
	HINSTANCE hInstance = GetModuleHandle(0);//画面

	//脚接地点とphantomXの状態の画像表示クラス
	std::vector<LNODE> answer; //結果を格納するベクタ
	HexapodGraphic H_G(hInstance);
	bool H_G_isCicle = 1;	//0でスレッドの終了処理
	{
		H_G.answer = &answer;
		H_G.mtxHexapodGraphic = &mtxHexapodGraphic;
		H_G.mtxMapData = &mtxMapData;
		H_G.isCicle = &H_G_isCicle;
		H_G.p_map = mapData;
	}
	boost::thread Thread_H_G(H_G);

	//ログファイルの作成
	LogFileIO _LogFileIO;
	if (_LogFileIO.openLogFile() == false) { return -1; }

	//シミュレーションの結果を入れるクラス
	SimulateResult _SimuRes;
	bool continue_simulation = true;

	//コマンドラインに書き込むためのクラス
	CmdIO _CmdIO;

#ifdef COMMUNICATION
	//phantomXへの通信クラス
	phantomxCommander commander;

	//現在の体勢を送信
	//stampLegNum = 0;
	for (int i = 0; i < 6; i++) {
		commander.endpoints[i] = myvector::VGet(CurrentCondition.Leg[i].y, CurrentCondition.Leg[i].x, -CurrentCondition.Leg[i].z);
		/*if (CurrentCondition.Leg[i].z + CurrentCondition.global_center_of_mass.z == 0) {
			stampLegNum++;
		}*/
	}
	commander.SecurelySendEndPoints(3000, 500);

#endif

	std::cout << "文字入力でシミュレーションを開始します。\n\n";
	std::cout << "phantomXの電源を切らずに[SHIFT]+[CAPSLOCK]長押し（終了するまで）で安全に終了できます。\n\n";

	int num = 1;
	int m_loop_counter = 0;//実際に行ったシミュレーション回数

	for (m_loop_counter; m_loop_counter < Define::SIMURATE_NUM; m_loop_counter++) {

		std::cout << "シミュレーション　" << num << "回目" << std::endl;
		std::cout << "現在の歩行成功回数\t" << _SimuRes.getClearNum() << std::endl;
		_LogFileIO.addLogStringWithInt(num);

		//1シミュレーション終わるごとに、ロボットの位置を変更する。
		if (m_loop_counter != 0) 
		{
			SetConditionForStripe(CurrentCondition, num-1);
			PastCondition = CurrentCondition;
			PastCondition.leg_state = 0;
		}

		int _distance_move_y = (int)CurrentCondition.global_center_of_mass.y;	//ファイル出力用　1シミュレーションでの移動距離の最大最小を記録する
		_map_data_num = Define::MAPDATA3D_MAX;									//ダミーマップ　手打ちで脚接地候補点を入れる際はこれを使う
		getMap(allmapData, &_map_data_num, &CurrentCondition, num -1 );			//0～5は脚の初期位置

		////自分の足元の脚接地点のみを既知とする(他の脚接地可能点を未知とする)
		for (int i = 0; i < _map_data_num; i++)
		{
			mapData[i].x = allmapData[i].x - CurrentCondition.global_center_of_mass.x;
			mapData[i].y = allmapData[i].y - CurrentCondition.global_center_of_mass.y;
			mapData[i].z = allmapData[i].z - CurrentCondition.global_center_of_mass.z;
		}

		//脚接地点を将棋のマス目のようにブロックごとに分割	ここどうにかしないとカメラ使えない
		std::vector< std::vector< std::vector<myvector::SVector> > > divideMapData(LP_DIVIDE_NUM, std::vector< std::vector<myvector::SVector> >(LP_DIVIDE_NUM, std::vector<myvector::SVector>(5)));
		int pointNum[LP_DIVIDE_NUM][LP_DIVIDE_NUM] = { 0 };
		MapSqrtDivide(allmapData, _map_data_num, divideMapData, pointNum);

		if(Define::FLAG_GRAPHIC_AVAILABLE == true)
		{
			boost::mutex::scoped_lock lk(mtxHexapodGraphic); //ロボット体勢へのアクセス権
			answer.push_back(CurrentCondition);
		}

		//時間とノードの出力のためのvector
		std::vector<double> _time;
		std::vector<int> _node_num;

		for (int m_simu_counter = 0; m_simu_counter < Define::GATE_PATTERN_GENERATE_NUM; m_simu_counter++) 
		{
			//1度のシミュレーションでの最大動作数
			_LogFileIO.addLogStringWithNode(m_simu_counter, CurrentCondition);

			if(Define::FLAG_GRAPHIC_AVAILABLE)
			{
				boost::mutex::scoped_lock lk(mtxHexapodGraphic); //ロボット体勢へのアクセス権
				answer.push_back(CurrentCondition);
			}

			//ダミーマップ
			//重心移動分マップを移動
			recalMap(mapData, _map_data_num, &CurrentCondition, &PastCondition);

			{
				boost::mutex::scoped_lock lk(mtxMapData);		//マップへのアクセス権
				for (int i = 0; i < Define::MAPDATA3D_MAX; i++)mapDataBackUp[i] = myvector::VGet(10000, 10000, 0);
				for (int i = 0; i < _map_data_num; i++) {//mapDataBackUp=グローバル座標,mapData=原点ロボットの中心、方向グローバルと同じ

					mapDataBackUp[i] = myvector::VRot(mapData[i], CurrentCondition.global_center_of_mass, CurrentCondition.pitch, CurrentCondition.roll, CurrentCondition.yaw);
					mapDataBackUp[i] = myvector::VAdd(mapDataBackUp[i], CurrentCondition.global_center_of_mass);
					//mapDataBackUp[i] =mapData[i];
					//mapDataBackUp[i].z = 0;
				}
			}

			/*
			for (int i = 0; i < MAPDATA3D_MAX; ++i) {
				//std::cout << "(" << allmapData[i].x << "," << allmapData[i].y << "," << allmapData[i].z << ")" << std::endl;
				std::cout << "(" << mapDataBackUp[i].x << "," << mapDataBackUp[i].y << "," << mapDataBackUp[i].z << ")" << std::endl;
			}
			//どこかで目標位置の更新(必要なら)
			/////////////////脚先位置決定部/////////////////
			探索した動作を全て行う
			if(&pass_root[passdepth] != NULL && pass_root[passdepth].node_height > 0){
				NewCondition = pass_root[passdepth];
				passdepth++;
			}else{
				std::cout<<"探索深さ = "<<passdepth<<std::endl;
				//start=clock();
				NewCondition = functionPassFinding(CurrentCondition, PastCondition, _map_data_num, mapDataBackUp, Target ,pass_root);
				//end=clock();
				//printf("%f\n",(double)(end-start)/CLOCKS_PER_SEC);
				passdepth = 2;
			}

			double t1 = seconds();
			double t2;
			for (;;) {
				t2 = seconds();
				if (t2 - t1 > 0.3)break;
			}
			---------------------------------------------------//
			std::string stooop;//初期体勢のスナップショット取るよう
			if (gnum == 1) std::cin >> stooop;
			---------------------------------------------------//
			//
			脚の上下運動または左右運動の動作のみをまとめて行う
			if(&pass_root[passdepth] != NULL && pass_root[passdepth].node_height > 0 && ((pass_root[1].debug >= 30 && pass_root[passdepth].debug >= 30) || ((pass_root[1].debug < 30 && pass_root[passdepth].debug < 30)))){
				NewCondition = pass_root[passdepth];
				passdepth++;
			}else{
				NewCondition = functionPassFinding(CurrentCondition, PastCondition, _map_data_num, mapDataBackUp, Target ,pass_root);
				passdepth = 2;
			}
			if (CurrentCondition.global_center_of_mass.y >= 700.0 * (remapcnt + 1) ) {
				dis_over = true;
				++remapcnt;
				break;
			}
			*/

			//直進移動のシミュレーション用
			Target.RotationCenter.y = CurrentCondition.global_center_of_mass.y; 
			Target.RotationCenter.z = CurrentCondition.global_center_of_mass.z;

			CurrentCondition.delta_comz = 0;
			CurrentCondition.target_delta_comz = 0;

			//1動作ごとに現状をコマンドラインに表示
			std::cout << "シミュレーション" << (m_loop_counter+1) <<"回目" <<  std::endl;
			std::cout << "成功回数" << _SimuRes.getClearNum() << "回" << std::endl;
			std::cout << "総動作数" << _SimuRes.getGatePatternGenerateSum() << std::endl;
			std::cout << m_simu_counter << "動作目" << std::endl;
			std::cout << "シミュレーション実行時間" << _SimuRes.getGatePatternGenerateTimeSum() << "秒" << std::endl;

			//時間計測用
			double _elapse;
			_elapse = seconds();	

			//グラフ探索で歩容パターン生成
			_node_num.push_back(0);
			NewCondition = functionPassFinding(CurrentCondition, PastCondition, _map_data_num, mapDataBackUp, Target, pass_root, divideMapData, pointNum, _node_num.back());	
			
			//グラフ探索にかかった時間を記録する
			_time.push_back(seconds() - _elapse);	
			
			// NewConditionに情報を記録する
			NewCondition.last_node_num = _node_num.back();//探索で得られたノード数
			NewCondition.time = _time.back();//探索でかかった時間

			//コマンドラインに結果を表示する
			_CmdIO.outputLNODE(NewCondition);

			//シミュレーション結果を更新する
			_SimuRes.countupGatePatternGenerateSum();
			_SimuRes.updateGatePatternGenerateTime(_time.back());
			_distance_move_y += (int)(NewCondition.global_center_of_mass.y - CurrentCondition.global_center_of_mass.y);


			//遷移可能なノードが存在しない場合，処理をやめループを一つ抜ける
			if (NewCondition.node_height < 0) 
			{
				std::cout << std::endl << "グラフ探索で，歩容パターンが得られなかったため，探索を終了します.\n\n" << std::endl;
				
				//結果を記録する
				_SimuRes.countupFailedByNoGatePattern();
				_LogFileIO.addLogStringWithNode(m_simu_counter, NewCondition);
				_LogFileIO.addLogString("failed_by_no_gate_pattern");
				break;
			}

			//ノードを後ろに追加して，30個以上溜まったら前の物を消す．ここマジックナンバーなので後でDefineに追加したほうがいい
			PreviousNode.push_back(PastCondition);
			if (PreviousNode.size() > 30) { PreviousNode.erase(PreviousNode.begin()); }

			// Conditionを更新する
			PastCondition = CurrentCondition;
			CurrentCondition = NewCondition;

			CurrentCondition.node_height = 1;
			CurrentCondition.parent = NULL;

			//同じ動作を繰り返すと，処理をやめループを一つ抜ける
			bool _loop_out_flag = false;

			if (m_simu_counter > 30) 
			{
				for (auto &i : PreviousNode) 
				{
					if (LNODEEqual(i, CurrentCondition) && (abs(i.yaw - CurrentCondition.yaw) < 0.01) && i.debug == CurrentCondition.debug) 
					{
						std::cout << "同じ動作を繰り返したため停止します" << std::endl;
						_SimuRes.countupFailedByGatePatternLoop();
						_LogFileIO.addLogString("failed_by_gate_pattern_loop");
						_loop_out_flag = true;
					}
				}
			}

			if (_loop_out_flag == true) { break; }

			//目的の距離あるくことができたなら，処理をやめループを抜ける
			if (CurrentCondition.global_center_of_mass.y >= Define::GOAL_TAPE) 
			{
				std::cout << "一定距離の歩行に成功しました．" << std::endl;
				_SimuRes.countupClearNum();
				_LogFileIO.addLogString("clear");
				break;
			}

			//SHIFT+CAPSLOCKキーが押されているとき 歩行終了　これをしないとファイル書き込みが完了しない
			if (GetAsyncKeyState(VK_CAPITAL) & 1/*0x8000*/) 
			{
				std::cout << "一時的にbreakしないようにしてる(main.cpp 528あたり)2021/01/22" << std::endl;
				if(m_loop_counter!=1) --m_loop_counter;//シミュレーション回数にカウントしない
				continue_simulation = false;
				break;
			}

		//データ送信部
		#ifdef COMMUNICATION	
			{ 
				boost::mutex::scoped_lock lkm(mtxMapData);			//マップへのアクセス権
				boost::mutex::scoped_lock lk(mtxHexapodGraphic); //ロボット体勢へのアクセス権

				//現在の体勢を送信
				//stampLegNum = 0;
				for (int i = 0; i < 6; i++) {
					commander.endpoints[i] = myvector::VGet(CurrentCondition.Leg[i].y, CurrentCondition.Leg[i].x, -CurrentCondition.Leg[i].z);
					/*if (CurrentCondition.Leg[i].z + CurrentCondition.global_center_of_mass.z == 0) {
						stampLegNum++;
					}*/
				}
				commander.SecurelySendEndPoints(3000, 500);
			}
		#endif

		}
		
		++num;

		//シミュレーションの時間とノードの数のリストをファイルに出力する
		TimeFileIO _TimeFileIO;
		_TimeFileIO.outputTimeFile(_time);

		NodeFileIO _NodeFileIO;
		_NodeFileIO.outputNodeFile(_node_num);


		_SimuRes.updateDistanceMoveY(_distance_move_y);

		if (_SimuRes.getGatePatternGenerateSum() > 1000) 
		{
			//1000動作超えるとバグるからループを抜ける　原因不明のまま ←おそらくオーバーフロー 解決済み (長谷川)
			break;
		}

		if (continue_simulation == false) 
		{
			//std::cout << "シミュレーションを続行する 1 ,続行しない 0 を入力" << std::endl;
			//std::cin >> continue_simulation;
			//if (continue_simulation == false) break;
		}
	}

	//ループここまで，以下結果を出力

	H_G_isCicle = 0;	//0でスレッドの終了処理
	
	//シミュレーションの結果をLogファイルへ出力
	_LogFileIO.addLogStringSimulation(m_loop_counter, _SimuRes);
	_LogFileIO.closeLogFile();

	//シミュレーションの結果をコマンドラインへ出力
	_CmdIO.outputSimulateResult(m_loop_counter, _SimuRes);
	
	std::cout << m_loop_counter << "回分の歩容パターン生成シミュレーションが終了しました。\n文字入力でプログラムを終了します。";
	std::cin >> stop;

	return 0;
}


void SetDefaultCondition(LNODE &node) 
{	
	//初期姿勢の代入　地形変えるときは初期姿勢も変える
	double COM_Z = 130;
	
	node.global_center_of_mass = myvector::VGet(0,0, COM_Z);//重心位置グローバル
	//node.global_center_of_mass = myvector::VGet(0, 0, 160);//重心位置グローバル

	node.Leg[0] = myvector::VGet(120, 100, -COM_Z);//付け根から脚先の位置
	node.Leg[1] = myvector::VGet(130, 0, -COM_Z);
	node.Leg[2] = myvector::VGet(120, -100, -COM_Z);
	node.Leg[3] = myvector::VGet(-120, -100, -COM_Z);
	node.Leg[4] = myvector::VGet(-130, 0, -COM_Z);
	node.Leg[5] = myvector::VGet(-120, 100, -COM_Z);

	//脚の位置(z方向固定)
	node.Leg2[0] = myvector::VGet(120, 100, -COM_Z);
	node.Leg2[1] = myvector::VGet(130, 0, -COM_Z);
	node.Leg2[2] = myvector::VGet(120, -100, -COM_Z);
	node.Leg2[3] = myvector::VGet(-120, -100, -COM_Z);
	node.Leg2[4] = myvector::VGet(-130, 0, -COM_Z);
	node.Leg2[5] = myvector::VGet(-120, 100, -COM_Z);

	//姿勢テイトブライアン角グローバル
	node.pitch = 0.0;//x軸回転
	node.roll = 0.0;//y軸回転
	node.yaw = 0.0;//z軸回転
	node.center_of_mass = 0;//重心位置int
	node.leg_state = 0b00000000110011001100110011001100;
	/*
	std::cout << "leg_conもともと" << node.leg_state << std::endl;
	std::string s = std::to_string(0b00000000110011001100110011001100);
	node.leg_state = std::stoi(s);
	std::cout <<"leg_conもともとと一緒になればおｋ" <<  node.leg_state << std::endl;
	std::cout << "leg_state" << std::bitset<32>(node.leg_state) << std::endl;
	std::cin >> node.pitch;
	*/
	node.parent = NULL;		//親ノードのポインタ
	node.node_height = 1;	//ノード高さ
	node.debug = 24;		//現在運動履歴として使用,前回の脚上下ノード(上下運動をした場合)2桁,前回の動作1桁,前々回の動作1桁
	node.delta_comz = 0;

	//node.Leg[0] = myvector::VGet(90, 80, -160);//付け根から脚先の位置
	//node.Leg[1] = myvector::VGet(100, 0, -160);
	//node.Leg[2] = myvector::VGet(90, -80, -160);
	//node.Leg[3] = myvector::VGet(-90, -80, -160);
	//node.Leg[4] = myvector::VGet(-100, 0, 160);
	//node.Leg[5] = myvector::VGet(-90, 80, -160);
	//////脚の位置(z方向固定)
	//node.Leg2[0] = myvector::VGet(90, 80, -160);
	//node.Leg2[1] = myvector::VGet(100, 0, -160);
	//node.Leg2[2] = myvector::VGet(90, -80, -160);
	//node.Leg2[3] = myvector::VGet(-90, -80, -160);
	//node.Leg2[4] = myvector::VGet(-100, 0, -160);
	//node.Leg2[5] = myvector::VGet(-90, 80, -160);
	//
	//node.global_center_of_mass = myvector::VGet(11.7557, 1800, 110);//重心位置グローバル
	//node.Leg[0] = myvector::VGet(108.244, 60, -110.39);//付け根から脚先の位置
	//node.Leg[1] = myvector::VGet(148.244, 7.39098e-06, -65);
	//node.Leg[2] = myvector::VGet(28.2443, -60, -65);
	//node.Leg[3] = myvector::VGet(-51.7557, -105, -149.61);
	//node.Leg[4] = myvector::VGet(-131.756, 90, -130);
	//node.Leg[5] = myvector::VGet(-11.7557, 105, -65);
	//node.Leg2[0] = myvector::VGet(118.182, 62.5714, -130);
	//node.Leg2[1] = myvector::VGet(148.244, 7.39098e-06, -130);
	//node.Leg2[2] = myvector::VGet(38.1818, -57.4286, -149.61);
	//node.Leg2[3] = myvector::VGet(-51.7557, -105, -149.61);
	//node.Leg2[4] = myvector::VGet(-161.818, 92.5714, -130);
	//node.Leg2[5] = myvector::VGet(-11.7557, 105, -110.39);
	//node.pitch = 0;//x軸回転
	//node.roll = 0;//y軸回転
	//node.roll = 0;//z軸回転
	//node.center_of_mass = 0;//重心位置int
	//node.leg_state = 0b00000110010011001100010001001100;
	//node.parent = NULL;//親ノードのポインタ
	//node.node_height = 1;//ノード高さ
	//node.debug = 22;//現在運動履歴として使用,前回の脚上下ノード(上下運動をした場合)2桁,前回の動作1桁,前々回の動作1桁
}