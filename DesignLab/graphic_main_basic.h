#pragma once

#include <vector>

#include "abstract_graphic_main.h"
#include "map_state.h"
#include "node.h"
#include "graphic_const.h"
#include "camera_gui.h"
#include "gui_controller.h"
#include "hexapod_renderer.h"
#include "movement_locus_renderer.h"


//! @class GraphicMainBasic
//! @date 2023/08/09
//! @author 長谷川
//! @brief このプロジェクトにおける標準的なロボットの描画機能を持つクラス．
//! @details 波東さんのプログラムのロボット表示機能を書き直したもの．
//! 基本的な処理の内容は変化していないが，より表示するデータの内容が詳しくなっている．
//! また，UIによってランタイムで表示方法を制御することができるようになったため，よりロボットの状態を理解しやすくなっている．ﾀﾌﾞﾝﾈ
//! @note 処理を大きく書き換えたい場合はそもそも新しいクラスを書くようにするとよいと思う．
//! @n GraphicSampleを参考にして，作成するようにすると楽．
class GraphicMainBasic final : public AbstractGraphicMain
{
public:
	GraphicMainBasic(const GraphicDataBroker* broker);
	~GraphicMainBasic() = default;

	bool update() override;

	void draw() const override;

private:

	CameraGUI m_camera_gui;				// カメラの位置を制御するGUI

	HexapodRenderer m_hexapod_renderer;

	MovementLocusRenderer m_movement_locus_renderer;

	GUIController m_gui_controller;		// GUI (ロボットの状態とか表示する窓) を制御するクラス．

	std::vector<SNode> m_node;	//ロボットの動きの遷移を記録するvector

	int m_display_node = 0;		//描画しているノード

	MapState m_map_state;				//表示するマップ．

	int m_counter = 0;			//このクラスが実行されてから何回update関数が呼ばれたかカウントする．

	const int kNodeGetCount = 2 * GraphicConst::GRAPHIC_FPS;	//2秒ごとに読み出す．
};


//! @file graphic_main_basic.h
//! @date 2023/08/09
//! @author 長谷川
//! @brief 基本的な描画クラス．
//! @n 行数 : @lineinfo
