//! @file GraphicConst.h
//! @brief 画像表示処理用の定数．
//! @author 長谷川

#pragma once
#include <string>

//! @class GraphicConst
//! @brief 画像表示処理用の定数をまとめたもの．
//! @details 定数クラスの詳細はDefine.hを参照．<br> <br>
//! このクラスの注意点としては，まず，WIN_X, WIN_Yに大きすぎる値を代入しないこと．また，16:9の画面比は崩さないほうがいいと思う．<br>
//! 次にカメラに関する値を変更する時は慎重に行うこと，下げすぎると何も見えなくなるし，上げすぎると尋常じゃなく重くなります．
//! @author 長谷川
class GraphicConst final
{
public:
	const static std::string WIN_NAME;	//!< ウィンドウの名前．
	const static int WIN_X;				//!< ウィンドウの横幅．dxlibではウィンドウの横方向に，右向きを正として X 軸をとります．
	const static int WIN_Y;				//!< ウィンドウの縦幅．dxlibではウィンドウの縦方向に，下向きを正として Y 軸をとります．
	const static int COLOR_BIT;			//!< 色を表現するbit数．通常32で良いが軽くするなら16にする．

	const static int BACK_COLOR_R;		//!< ウィンドウ背景色． 赤色成分
	const static int BACK_COLOR_G;		//!< ウィンドウ背景色． 緑色成分
	const static int BACK_COLOR_B;		//!< ウィンドウ背景色． 青色成分

	const static float CAMERA_FAR;	//!< カメラが表示できる最も遠い座標．
	const static float CAMERA_NEAR;	//!< カメラが表示できる最も近い座標．

	const static float CAMERA_TO_TARGET_MAX;	//!< カメラと注視目標の最大距離．CAMERA_FARとCAMERA_NEARの間の値じゃないとなにも表示されなくなる．

	//! ウィンドウのFPS(フレーム / 秒．秒間画面を何回更新するか．TVは30fps，Nintendo Switchは60fps)．
	//! 60より大きい値にしても意味はない．30とかにしてもいいけどそこまで処理負荷は変わらないと思う．
	constexpr static unsigned int GRAPHIC_FPS = 60;

private:

	//コンストラクタとコピーコンストラクタを削除して実体を生成できないようにする．
	GraphicConst() = delete;
	GraphicConst(GraphicConst& _other) = delete;
};