#include "GraphicSystem.h"
#include "DxLib.h"
#include "GraphicConst.h"
#include "GraphicLoop.h"
#include "GraphicMainSample.h"
#include "GraphicMainBasic.h"
#include "Define.h"

//注意として，Dxlib系の関数は 真偽を大文字の TRUEとFALSEを使って表すので，従来のtrue falseを使用しないようにしましょう．
//まぁ，ぶっちゃけ小文字の方でも動くけど，バージョンの更新によって動かなくなる可能性があるのでDxlibに組み込まれているものを使うのが無難です．
//また，Dxlibはエラーが出たときに -1 を返す関数が非常に多いです．そのため例えば if(DxLib_Init() == false) と書いてもエラーを受け取れないことがあります．
//正しくは if(DxLib_Init() < 0) となります．これは bool型 がデフォルトで存在しないC言語でも使用することができるようにするための配慮であり，C++で書かれている本コードにおいては
//混乱の元です(涙)．Dxlibのエラーはboolではなく，int型の負の値ということを覚えておいてください．


bool GraphicSystem::init(const GraphicDataBroker* _p_broker)
{	
	//ブローカーがnull(存在しない)ならfalse
	if (_p_broker == nullptr) { return false; }

	mp_Broker = _p_broker;

	return true;
}

void GraphicSystem::main()
{
	//そもそも描画処理を使わないならば即終了
	if (Define::FLAG_GRAPHIC_AVAILABLE == false) { return; }

	//初期化をしていない or 失敗した場合則終了．
	if (mp_Broker == nullptr) { return; }

	// Dxlibの関数は複数スレッドで呼ぶことを考慮されていないので，複数のスレッドから呼ぶと必ず問題が起きます．
	//そのため，初期化処理，描画，終了処理の全てをこの関数の中で呼ぶ必要があります．
	if (dxlibInit() == false) { return; }
	
	//描画の処理を行うクラスをセットする．
	GraphicLoop _Looper(std::make_unique<GraphicMainBasic>(mp_Broker));

	// ProcessMessage関数はウィンドウの×ボタンがおされると失敗の値を返す．また，ウィンドウを維持するためには定期的に呼び出し続ける必要があるのでループで呼び続けている．
	// ProcessMessageは成功で0(C++におけるfalse)，失敗で-1(C++におけるtrueは0以外の値)を返す，そのため !ProcessMessage はこの関数が成功の時のみループする...頭の痛い処理である．
	while (!ProcessMessage())
	{
		//falseが帰った場合，ループを抜ける．
		if (_Looper.loop() == false) 
		{
			break;
		}
	}

	//終了処理を行う．
	dxlibFinalize();
}

bool GraphicSystem::dxlibInit()
{
	// 1部の初期化用関数はDxlib_Initを呼ぶ前に実行する必要があるのでここで実行します．

	SetMainWindowText(GraphicConst::WIN_NAME.c_str());	//タイトルを変更．ウィンドウの左上に表示されるものです．
	ChangeWindowMode(TRUE);								//ウインドウモードに変更．これをしないとフルスクリーンで表示されます．
	SetWindowSizeChangeEnableFlag(FALSE);               //ウィンドウサイズを自由に変更できないようにする．
	SetOutApplicationLogValidFlag(FALSE);				//ログ出力無しに変更．これをしないとLog.txtという邪魔なファイルが出力されます．
	SetAlwaysRunFlag(TRUE);								//ウインドウがアクティブではない状態でも処理を続行するように変更．

	//ウィンドウの横幅，縦幅，カラーを設定します．
	SetGraphMode(GraphicConst::WIN_X, GraphicConst::WIN_Y, GraphicConst::COLOR_BIT);

	//ＤＸライブラリ初期化処理
	if (DxLib_Init() < 0)
	{
		return false;
	}

	//描画先を裏画面にする．説明が難しいのですが，画面のちらつきを押えてくれる効果があり，Dxlibを使う以上必須の項目です．
	SetDrawScreen(DX_SCREEN_BACK);

	return true;
}

void GraphicSystem::dxlibFinalize() const
{
	// DXライブラリの終了処理を呼ぶ.
	DxLib_End();
}
