#include "graphic_main_basic.h"

#include "DxLib.h"

#include "designlab_dxlib.h"
#include "world_grid_renderer.h"
#include "map_renderer.h"


GraphicMainBasic::GraphicMainBasic(const GraphicDataBroker* const  broker, const SApplicationSettingRecorder* const setting)
	: AbstractGraphicMain(broker, setting), m_map_state(mp_broker->getMapState()), kNodeGetCount(setting->window_fps * 2),
	m_node_display_gui(mp_setting->window_size_x - NodeDisplayGUI::BOX_SIZE_X - 10, 10),
	m_display_node_switch_gui(10, mp_setting->window_size_y - DisplayNodeSwitchGUI::GUI_HEIGHT - 10)
{
	m_node.clear();
}


bool GraphicMainBasic::update()
{

	//ノードを読み出す時間になったら，仲介人からデータを読み出す．
	if (m_counter % kNodeGetCount == 0)
	{
		//仲介人からデータを読み出す
		mp_broker->copyOnlyNewNode(&m_node);

		std::vector<size_t> simu_end_index;

		mp_broker->copySimuEndIndex(&simu_end_index);


		//ノードの情報を表示するGUIに情報を伝達する．
		m_display_node_switch_gui.setGraphData(m_node.size(), simu_end_index);



		//移動軌跡を更新する．
		m_movement_locus_renderer.setMovementLocus(m_node);

		m_movement_locus_renderer.setSimuEndIndex(simu_end_index);


		//ロボットの接地点を更新する．
		m_robot_graund_point_renderer.setNode(m_node, simu_end_index);
	}


	//ノードが存在しているのならば，各クラスに情報を伝達する
	if (!m_node.empty())
	{
		m_display_node = (int)m_display_node_switch_gui.getDisplayNodeNum();	//表示するノードを取得する．

		m_hexapod_renderer.update(m_node.at(m_display_node));					//ロボットの状態を更新する．

		m_camera_gui.setHexapodPos(m_node.at(m_display_node).global_center_of_mass);		//カメラの位置を更新する．
	}


	m_counter++;				//カウンタを進める．

	m_camera_gui.update();      //カメラのGUIを更新する．

	m_display_node_switch_gui.update();	//ノードの情報を表示するGUIを更新する．

	return true;
}


void GraphicMainBasic::draw() const
{
	// 3Dのオブジェクトの描画

	dl_dxlib::setZBufferEnable();		//Zバッファを有効にする．


	WorldGridRenderer grid_renderer;	//インスタンスを生成する．

	grid_renderer.draw();				//グリッドを描画する．


	MapRenderer map_render;				//マップを描画する．

	map_render.draw(m_map_state);


	m_movement_locus_renderer.draw(m_display_node_switch_gui.getSimulationNum());   //移動軌跡を描画する．

	m_robot_graund_point_renderer.draw(m_display_node_switch_gui.getSimulationNum());

	if (!m_node.empty())
	{
		//ノードが存在しているならば，ロボットを描画する．
		m_hexapod_renderer.draw(m_node.at(m_display_node));

		m_stability_margin_renderer.draw(m_node.at(m_display_node));
	}


	// 2DのGUIの描画

	m_camera_gui.draw();        //カメラのGUIを描画する．

	m_node_display_gui.draw();	 //ノードの情報を表示するGUIを描画する．

	m_display_node_switch_gui.draw();	//表示するノードを切り替えるGUIを描画する．
}
