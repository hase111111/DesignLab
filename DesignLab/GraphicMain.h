#pragma once

// GraphicMainは Dxlibの処理を行ってくれるクラスです．
// Dxlib(デラックス ライブラリ)はウィンドウを表示して，所謂コマンドラインに文字を表示するだけの寂しいプログラムに彩りを与えてくれるやつです．
// 主にゲームプログラミングをする際に，ウィンドウを表示するためのライブラリとして使用されます．
// Dxlib以外にも OpenCVなどにもウィンドウを表示する機能がありますが，今回のプログラムではDxlibを用いて結果を表示します．
// Dxlibは WinAPI というWindowsのアプリケーションを作るための機能を，使いやすくしてくれるライブラリです．
// 以下参考ページ
// https://dixq.net/rp2/ ←C++用の資料．少々難しいかも．
// https://dixq.net/g/   ←C言語用の資料．あまり参考にならないかも
// https://dxlib.xsrv.jp/dxfunc.html ←公式の関数のリファレンス(目次的なもの)．

class GraphicMain final
{
public:
	GraphicMain() = default;
	~GraphicMain() = default;

	//Dxlibの初期化を行います．失敗するとfalseを返します．
	bool init();

	// ウィンドウの表示を行ってくれる関数です．boost::threadにこの関数を渡して並列処理を行います．initに失敗している，またはinitを呼ぶ前に実行した時は即座に終了します．
	void main();

private:

	void finalize() const;	//Dxlibの終了処理を行います．

	bool m_is_init_success = false;
};
