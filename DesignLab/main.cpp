#include <memory>
#include <iostream>

#include "application_setting_reader.h"
#include "application_setting_recorder.h"
#include "designlab_cmdio.h"
#include "system_main.h"
#include "pass_finder_hato_thread.h"
#include "pass_finder_factory_hato.h"
#include "basic_graphic_main_builder.h"
#include "other_graphic_main_builder.h"
#include "phantomx_state_calculator.h"
#include "graph_viewer_system_main.h"



int main(void)
{
	//このプロジェクトがコンパイルされない場合はソリューションエクスプローラーからDesignLabを右クリック→「スタートアッププロジェクトに設定」

	//これを記述しておくと実行速度が早くなる．そのかわりprintfを使用できない．
	std::ios_base::sync_with_stdio(false);
	std::cin.tie(nullptr);


	//まずは，設定ファイルを読み込む
	ApplicationSettingReader setting_reader;		//設定ファイルを読み込むクラス
	SApplicationSettingRecorder setting_recorder;	//設定ファイルの内容を記録するクラス

	setting_reader.read(&setting_recorder);			//ファイルを読み込む


	//タイトルを表示する
	dl_cio::outputNewLine(&setting_recorder, 2, EOutputPriority::SYSTEM);
	dl_cio::outputTitle(&setting_recorder);


	//起動モードを選択する
	EBootMode boot_mode = setting_recorder.default_mode;

	if (setting_recorder.ask_about_modes)
	{
		//起動モードを選択する
		boot_mode = dl_cio::selectBootMode(&setting_recorder);

	}



	//プログラムを起動する
	if (boot_mode == EBootMode::SIMULATION)
	{
		//メインシステムクラスを作成する．ここでコンストラクタが呼ばれる．
		SystemMain sys(std::make_unique<PassFinderHatoThread>(), std::make_unique<PassFinderFactoryHato>(), std::make_unique<BasicGraphicMainBuilder>(),
			std::make_unique<PhantomXStateCalclator>(), &setting_recorder);

		//シミュレーションの処理を行う．
		sys.main();

	}
	else if (boot_mode == EBootMode::VIEWER)
	{
		GraphViewerSystemMain viewer(&setting_recorder);	//グラフ木を作成しその内容を表示する．

		viewer.main();					//メインの処理を行う．
	}
	else if (boot_mode == EBootMode::DISPLAY_TEST)
	{
		GraphicSystem gr_sys;

		GraphicDataBroker dummy_broker;

		gr_sys.init(std::make_unique<TestGraphicMainBuilder>(), std::make_shared<PhantomXStateCalclator>(), &dummy_broker, &setting_recorder);

		gr_sys.main();
	}


	return 0;
}


//! @file main.cpp
//! @brief プログラムのエントリポイント
//! @author 長谷川
//! @date 2023/06/30

//! @mainpage
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

