﻿#ifndef DESIGNLAB_GRAPHIC_SYSTEM_H_
#define DESIGNLAB_GRAPHIC_SYSTEM_H_

#include <memory>

#include <boost/thread.hpp>

#include "application_setting_record.h"
#include "fps_controller.h"
#include "graphic_data_broker.h"
#include "interface_graphic_main.h"


namespace designlab
{

//! @class GraphicSystem
//! @brief Dxlibの処理を行うクラス．
//! @details 
//! Dxlib(デラックス ライブラリ)とは，C++のiostreamだけではできない処理である，ウィンドウを表示して，
//! 描画を行う処理を行うためのライブラリである．
//! @n 主にゲームプログラミングをする際に，ウィンドウを表示するためのライブラリとして使用される．
//! Dxlib以外にも OpenCVなどにもウィンドウを表示する機能があるが，今回のプログラムではDxlibを用いて結果を表示する．
//! Dxlibは Windows API とかいうWindowsのアプリケーションを作るための機能を，使いやすくしてくれるライブラリである．
//! @n  
//! @subsubsection 参考ページ 
//! @n
//! @li https://dixq.net/rp2/ ←C++用の資料．少々難しい． 
//! @li https://dixq.net/g/   ←C言語用の資料．
//! @li https://dxlib.xsrv.jp/dxfunc.html ←公式の関数のリファレンス(関数の目次)．
//! 
//! Dxlibを非同期処理で動かすことで別スレッドで行っているグラフ探索の処理をもとにロボットの状態を表示する．
//! しかし，Dxlibは非同期処理を考慮して設計されていないので，動かし方によってはうまく動作しない．
//! このプロジェクトではこの関数の中でのみDxlibの処理を動かすことで，エラーを防いでいるが，予期せぬエラーが発生する可能性がある．
//! @n 
//! @subsubsection [Dxlibの注意] 
//! 注意として，Dxlib系の関数は 真偽を大文字の TRUEとFALSEを使って表すので，従来のtrue falseを使用しないようにすること．
//! (実際には小文字の方でも動くけど，バージョンの更新によって動かなくなる可能性があるのでDxlibに組み込まれているものを使うのが無難．)
//! また，Dxlibはエラーが出たときに - 1 を返す関数が非常に多い．そのため例えば if (DxLib_Init() == false) と書いても
//! エラーを正しく受け取れない．正しくは if (DxLib_Init() < 0) となる．
//! @code 
//! //間違った形
//! if (DxLib_Init() == false)
//! {
//! 	//エラー処理
//! }
//! 
//! //正しい形
//! if (DxLib_Init() < 0)
//! {
//! 	//エラー処理
//! }
//! @endcode
//! これは bool型 がデフォルトで存在しないC言語でも使用することができるようにするための配慮であり，
//! C++で書かれている本コードにおいては混乱の元でしかない．
//! Dxlibのエラーはboolではなく，int型の負の値ということを覚えておくこと．
//! @n 
//! @n また，Dxlibは2窓できないので，MyDxlibInit に失敗する場合はバックグラウンドでDxlibが動いている可能性がある．(例えば MyDxlibFinalize が呼ばれず，終了できていないときなど)
//! その場合はタスクマネージャーからdxlibを落とすことで，実行することが可能になる．
class GraphicSystem final
{
public:

	//! @param [in] setting_ptr アプリケーションの設定を記録するクラスのポインタ．
	GraphicSystem(const std::shared_ptr<const ApplicationSettingRecord> setting_ptr);


	//! @brief ウィンドウの表示を行う関数．boost::threadにこの関数を渡して並列処理を行う．
	//! @n メンバ関数のMyDxlibInit関数に失敗した場合，終了する．
	void Main();

	//! @brief グラフィックの表示を行うクラスを変更する．
	//! @param [in] graphic_main_ptr GraphicMainクラスのユニークポインタ．
	void ChangeGraphicMain(std::unique_ptr<IGraphicMain>&& graphic_main_ptr);

private:

	//! @brief Dxlibの初期化処理を行う．
	//! @n 処理をラッパして自作する場合はMyを頭につけると良い．
	//! @return 初期化に成功したか．
	bool MyDxlibInit();

	//! @brief GraphicSystemクラスのwhileループの中で毎フレーム呼ばれる処理．
	//! @return ループを続けるか．falseならばループを抜ける．異常が起きた場合もfalseを返す．
	bool Loop();

	//! @brief Dxlibの終了処理を行う．
	void MyDxlibFinalize() const;


	std::unique_ptr<IGraphicMain> graphic_main_ptr_;	//!< グラフィックの表示を行うクラスのポインタ．

	const std::shared_ptr<const ApplicationSettingRecord> setting_ptr_;	//!< 設定を保存する構造体のポインタ．

	FpsController fps_controller_;		//!< FPSを一定に制御するクラス．

	boost::mutex mutex_;	//!< 複数の関数から非同期的に，同時にアクセスすると危険なので，それを防ぐためにmutexを用いて排他制御を行う．
};

}	// namespace designlab


#endif	// DESIGNLAB_GRAPHIC_SYSTEM_H_