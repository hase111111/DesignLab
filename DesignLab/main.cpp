//! @file main.cpp
//! @brief プログラムのエントリポイント．

#include <memory>

#include <boost/thread.hpp>

#include "test_registration_form.h"

#if ! defined DESIGNLAB_USE_TEST

#include "application_setting_recorder.h"
#include "application_setting_reader_toml.h"
#include "boot_mode_selecter.h"
#include "cmdio_util.h"
#include "gait_pattern_generator_switch_move.h"
#include "gait_pattern_generator_thread.h"
#include "graph_tree_creator.h"
#include "graph_searcher_straight_move.h"
#include "graph_searcher_spot_turn.h"
#include "graphic_main_basic.h"
#include "graphic_main_graph_viewer.h"
#include "graphic_main_test.h"
#include "graphic_system.h"
#include "map_creator_by_csv.h"
#include "node_creator_builder_hato.h"
#include "node_creator_builder_rot_test.h"
#include "node_creator_builder_turn_spot.h"
#include "system_main_graph_viewer.h"
#include "system_main_result_viewer.h"
#include "system_main_simulation.h"
#include "phantomx_mk2.h"

enum class Color
{
	kRed,
	kGreen,
	kBlue,
};

#include "toml_serialize_macro.h"
struct Temp final
{
	int a{ 0 };
	int b{ 0 };
	Color color{ Color::kRed };
	std::string str{ "abc" };
};

struct TestDescription final
{
	const std::string file_description{ "いい感じのファイルです．" };

	const Toml11Description a{ "x","aは万病に聞きます" };
	const Toml11Description b{ "x","bは以外に細かいことを気にします．" };
	const Toml11Description color{ "x","colorは色です．" };
	const Toml11Description str{ "y","strは文字列です．" };
};


DESIGNLAB_DEFINE_CONVERSION_NON_INTRUSIVE(Temp, TestDescription, a, b, color, str);


// このプロジェクトがコンパイルされない場合はソリューションエクスプローラーから
// DesignLabを右クリック →「スタートアッププロジェクトに設定」を選択．

namespace dlio = ::designlab::cmdio;


int main()
{
	//コンソールが表示されるまで待つ．これをやらないとcoutの表示がおかしくなる．
	//たぶん一度に大量にcoutするとおかしくなるのだろうが，なんか変なつくりだからどーにかできないかね？
	Sleep(100);

	Temp t;
	//toml形式でcout 
	toml::basic_value<toml::preserve_comments, std::map> v(t);
	std::cout << toml::format(v) << std::endl;

	//まずは，設定ファイルを読み込む
	const std::unique_ptr<IApplicationSettingReader> setting_reader = std::make_unique<ApplicationSettingReaderToml>();		// 設定ファイルを読み込むクラス

	const std::shared_ptr<const ApplicationSettingRecorder> setting_recorder = setting_reader->ReadFileOrUseAndOutputDefault();	// 読み込んだ設定ファイルを記録するクラスに記録する


	//次に，コマンドラインの出力を設定する
	dlio::SetDoOutput(setting_recorder->do_cmd_output);
	dlio::SetOutputLimit(setting_recorder->cmd_output_detail);

	dlio::OutputTitle("グラフ探索による6脚歩行ロボットの自由歩容計画", true);	//タイトルを表示する


	//GUIを別のスレッドで実行する．このスレッドへはGraphicDataBrokerを通してデータを渡す．
	GraphicSystem graphic_system(setting_recorder);

	boost::thread graphic_thread(&GraphicSystem::Main, &graphic_system);	//グラフィックシステムを別スレッドで実行する．


	//処理を実行する
	while (true)
	{
		//起動モードを選択する
		BootMode boot_mode = setting_recorder->default_mode;

		if (setting_recorder->ask_about_modes)
		{
			BootModeSelecter boot_mode_selecter;

			boot_mode_selecter.SetDefaultBootMode(setting_recorder->default_mode);	//デフォルトの起動モードを設定する
			boot_mode = boot_mode_selecter.SelectBootMode();		//起動モードを選択する
		}


		//選択が終わったら，選択されたモードに応じてシステムを作成する
		auto graphic_data_broker = std::make_shared<GraphicDataBroker>();
		auto phantomx_mk2 = std::make_shared<PhantomXMkII>();

		auto node_creator_builder_straight = std::make_unique<NodeCreatorBuilderHato>(phantomx_mk2, phantomx_mk2, phantomx_mk2);
		auto node_creator_builder_turn_spot = std::make_unique<NodeCreatorBuilderTurnSpot>(phantomx_mk2, phantomx_mk2, phantomx_mk2);

		auto graph_tree_creator_straight = std::make_unique<GraphTreeCreator>(std::move(node_creator_builder_straight));
		auto graph_tree_creator_turn_spot = std::make_unique<GraphTreeCreator>(std::move(node_creator_builder_turn_spot));

		auto graph_searcher_straight = std::make_unique<GraphSearcherStraightMove>(phantomx_mk2);
		auto graph_searcher_turn_spot = std::make_unique<GraphSearcherSpotTurn>();

		std::unique_ptr<ISystemMain> system_main;


		if (boot_mode == BootMode::kSimulation)
		{

			//シミュレーションシステムクラスを作成する．

			auto pass_finder_straight = std::make_unique<GaitPatternGeneratorThread>(std::move(graph_tree_creator_straight), std::move(graph_searcher_straight), 5, 10000000);

			auto pass_finder_turn_spot = std::make_unique<GaitPatternGeneratorThread>(std::move(graph_tree_creator_turn_spot), std::move(graph_searcher_turn_spot), 5, 10000000);

			auto pass_finder = std::make_unique<GaitPatternGeneratorSwitchMove>(std::move(pass_finder_straight), std::move(pass_finder_turn_spot));

			auto map_creator = std::make_unique<MapCreatorByCsv>("./simulation_data/map_state1.csv");

			system_main = std::make_unique<SystemMainSimulation>(
				std::move(pass_finder),
				std::move(map_creator),
				graphic_data_broker,
				setting_recorder
			);

			auto graphic_main = std::make_unique<GraphicMainBasic>(
				graphic_data_broker,
				phantomx_mk2,
				phantomx_mk2,
				phantomx_mk2,
				setting_recorder
			);

			graphic_system.ChangeGraphicMain(std::move(graphic_main));
		}
		else if (boot_mode == BootMode::kViewer)
		{
			//グラフビューアシステムクラスを作成する．

			system_main = std::make_unique<SystemMainGraphViewer>(
				std::move(graph_tree_creator_straight),
				graphic_data_broker,
				setting_recorder
			);

			std::unique_ptr<IGraphicMain> graphic_main_viewer = std::make_unique<GraphicMainGraphViewer>(
				graphic_data_broker,
				phantomx_mk2,
				phantomx_mk2,
				phantomx_mk2,
				setting_recorder
			);

			graphic_system.ChangeGraphicMain(std::move(graphic_main_viewer));
		}
		else if (boot_mode == BootMode::kDisplayTest)
		{
			std::unique_ptr<IGraphicMain> graphic_main_test = std::make_unique<GraphicMainTest>(
				phantomx_mk2,
				phantomx_mk2,
				phantomx_mk2,
				setting_recorder
			);

			graphic_system.ChangeGraphicMain(std::move(graphic_main_test));
		}
		else if (boot_mode == BootMode::kResultViewer)
		{
			//結果表示システムクラスを作成する．
			system_main = std::make_unique<SystemMainResultViewer>(graphic_data_broker, setting_recorder);

			std::unique_ptr<IGraphicMain> graphic_main = std::make_unique<GraphicMainBasic>(
				graphic_data_broker,
				phantomx_mk2,
				phantomx_mk2,
				phantomx_mk2,
				setting_recorder
			);

			graphic_system.ChangeGraphicMain(std::move(graphic_main));
		}
		else
		{
			assert(true);	//無効なモードが指定された．
		}


		//システムを実行する
		if (system_main)
		{
			system_main->Main();
		}
		else
		{
			dlio::Output("SystemMainクラスがありません．"
				"(GraphicSystemしか使用しない場合はこのメッセージが表示されることがあります．)", OutputDetail::kSystem);
		}


		//もう一度実行するかどうかを選択する
		dlio::OutputHorizontalLine("=", OutputDetail::kSystem);
		dlio::OutputNewLine(1, OutputDetail::kSystem);

		if (!dlio::InputYesNo("アプリケーションを続行しますか？"))
		{
			break;
		}

		dlio::OutputNewLine(1, OutputDetail::kSystem);
		dlio::OutputHorizontalLine("=", OutputDetail::kSystem);
		dlio::OutputNewLine(1, OutputDetail::kSystem);
	}

	dlio::Output("Dxlibの終了を待っています．GUIの×ボタンを押してください．", OutputDetail::kSystem);
	graphic_thread.join();

	return 0;
}

#endif 

//! @mainpage
//! @details 今後このソースファイルを使う人のためにC++で大きなプロジェクトを作成するときのルールをここに定義しておく．
//! @n 基本的には google c++ style guide に従っているが，一部独自のルールを追加している．
//! @n
//! @n 独自ルールについて，
//! @n ・インデント…スペース2つではなくタブを使う．
//! @n ・名前空間…名前の衝突が起こりそうな一部のクラス・構造体のみ名前空間を使う．関数は必ず名前空間にいれる．
//! @n ・boostについて…標準・非標準にかかわらず使用する．
//! @n ・cpplintについて…使用しない．ただし，short，longなどは使用せず，int型を使用する．
//! @n ・その他の機能について…filesystemは使用する．
//! @n
//! @n 文字コードはutf-8を使用する．
//! @n
//! @n ●命名規則
//! @n ・スネークケース snake_case
//! @n ・キャメルケース camelCase
//! @n ・パスカルケース PascalCase
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
//!	詳しくは Common / define.hに書いておきます．<br>
//!	どうしても書きたいのならばcppファイルに書きましょう．hファイルに書き込むと他のファイルのコンパイルにも多大な影響を及ぼして危険です<br>
//!	C++には constexprという機能があるのでこれを使うのもありだと思います<br>
//! <br>
//!●配列について<br>
//!	C言語において通常使われる int _array[10]; のような配列の使用は最小限にしたほうが良いと思います．<br>
//!	なぜなら，このプログラムでも過去にそれによるスタックオーバーフローが起きていたためです．<br>
//!	time[1000]という配列に時間を記録していたため， 1000回以上のループができなくなっていました．<br>
//!	とりあえず大きな配列を作っておいて，その中にデータを記録するのは拡張性が低いし，見つけづらいバグの温床になります．<br>
//! @n std::vector (動的な配列) を積極的に使いたいと思っています．<br>
//! @n new/delete は個人的には絶対に使いたくありません．(エラーが怖いので)<br>
//! @n また，変数を int x1,x2,x3,x4;のように大量に宣言するのはやめましょう．その場合は普通に配列でいいです int x[4];  <br>
//! @n
//! @n ●for文について
//! @n  for文に使用する変数は i → j → k としましょう．通例そのような命名をすることが多いです．
//! @n  また，for文の中でサイズの大きなクラスや構造体を宣言するのはやめましょう．
//! @n
//! @n ●ヘッダー(.h)について
//! @n   必ず，インクルードガードを書きましょう．#ifndef ~ #endifのやつです
//! @n   #pragma once と書くと，インクルードガードを書かなくてもインクルードガードの機能が働きます．
//! @n   しかし，これはC++の機能ではないので非推奨です．(ぶっちゃけ，どっちでもいいと思います．どっちもちゃんと動くし，)
//! @n
//! @n   ヘッダーにはむやみに #include を書かないようにしましょう．
//! @n   コンパイルがクソ時間かかります．.cppのほうでインクルードできるならしましょう
//! @n   またグローバル変数を使ったり，externを使ったりするのは避けてください．多くの場合スパゲッティコードの原因になります．
//! @n   設計をよく考えれば使わずに済むはずです．どうしても必要な場合は一つのヘッダにまとめておいてください．
//! @n   散らされると探すのが本当に大変です．
//! @n
//! @n ●小数について
//! @n   小数をあらわす事のできる変数の形は float と double の二つがあります．
//! @n   float のほうがサイズが小さいので探索に向いているハズ，floatで統一して書いていきます．(要検討)
//! @n 
//! @n ●列挙体について
//! @n   整数型の引数を使って関数の機能を分けたいとき，例えば以下のような関数を使いたいとき，
//! @n
//! @n ~~~ enum sample1
//! @n   int CalculateNumber(const int num1, const int num2, const int mode)()
//! @n   {
//! @n     if(mode == 1){ return num1 + num2;}
//! @n     else if(mode == 2){ return num1 - num2;}
//! @n     else if(mode == 3){ return num1 * num2;}
//! @n     else if(mode == 4){ return num1 / num2;}
//! @n   }
//! @n ~~~
//! @n
//! @n   このような場合は列挙体を使うことをお勧めします．何故ならば，非常に読みやすくなるからです．
//! @n   例えば以下のように書き換えることができます．
//! @n
//! @n ~~~ enum sample2
//! @n   enum CalculateMode
//! @n   {
//! @n     kAdd = 1,
//! @n     kSub = 2,
//! @n     kMul = 3,
//! @n     kDiv = 4,
//! @n   };
//! @n
//! @n   int CalculateNumber(const int num1, const int num2, const CalculateMode mode)()
//! @n   {
//! @n     if(mode == CalculateMode::kAdd){ return num1 + num2;}
//! @n     else if(mode == CalculateMode::kSub){ return num1 - num2;}
//! @n     else if(mode == CalculateMode::kMul){ return num1 * num2;}
//! @n     else if(mode == CalculateMode::kDiv){ return num1 / num2;}
//! @n   }
//! @n ~~~
//! @n
//! @n ●構造体について
//! @n   C++で書かれているのでわざわざtypedefしなくてもOKです．
//! @n   むしろ，読みづらいのでtypedefは使わないほうが良いと思います．