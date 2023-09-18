#include "graphic_main_advance.h"

#include "DxLib.h"

#include "dxlib_util.h"
#include "keyboard.h"
#include "map_renderer.h"
#include "world_grid_renderer.h"


GraphicMainAdvance::GraphicMainAdvance(const GraphicDataBroker* const  broker, std::shared_ptr<AbstractHexapodStateCalculator> calc, const SApplicationSettingRecorder* const setting)
	: IGraphicMain(broker, calc, setting),
	m_map_state(mp_broker->map_state()),
	kNodeGetCount(setting->window_fps * 2),
	m_node_display_gui(mp_setting->window_size_x - NodeDisplayGui::kWidth - 10, 10, calc),
	m_display_node_switch_gui(10, mp_setting->window_size_y - DisplayNodeSwitchGUI::GUI_HEIGHT - 10),
	m_hexapod_renderer(calc)
{
	m_node.clear();
}


bool GraphicMainAdvance::Update()
{

	//ノードを読み出す時間になったら，仲介人からデータを読み出す．
	if (m_counter % kNodeGetCount == 0)
	{
		//仲介人からデータを読み出す
		mp_broker->CopyOnlyNewNode(&m_node);

		std::vector<size_t> simu_end_index;

		mp_broker->CopySimuEndIndex(&simu_end_index);


		//ノードの情報を表示するGUIに情報を伝達する．
		m_display_node_switch_gui.setGraphData(m_node.size(), simu_end_index);



		//移動軌跡を更新する．
		m_movement_locus_renderer.set_move_locus_point(m_node);

		m_movement_locus_renderer.set_simulation_end_indexes(simu_end_index);


		//ロボットの接地点を更新する．
		m_robot_graund_point_renderer.setNode(m_node, simu_end_index);
	}


	//ノードが存在しているのならば，各クラスに情報を伝達する
	if (!m_node.empty())
	{
		// 表示ノードが変更されたら，表示するノードを変更する．
		if (m_display_node_index != (int)m_display_node_switch_gui.getDisplayNodeNum())
		{
			if (m_display_node_index > 0)
			{
				m_interpolated_anime_start_count = m_counter;		//アニメーションを開始した時間を記録する．
				m_interpolated_node_creator.createInterpolatedNode(m_node[m_display_node_index], m_node[m_display_node_switch_gui.getDisplayNodeNum()], &m_interpolated_node);
			}


			m_display_node_index = (int)m_display_node_switch_gui.getDisplayNodeNum();				//表示するノードを取得する．

			m_hexapod_renderer.set_draw_node(m_node.at(m_display_node_index));							//ロボットの状態を更新する．

			m_camera_gui.setHexapodPos(m_node.at(m_display_node_index).global_center_of_mass);		//カメラの位置を更新する．

			m_node_display_gui.SetDisplayNode(m_node.at(m_display_node_index));						//ノードの情報を表示するGUIに情報を伝達する．
		}

		if (m_interpolated_anime_start_count <= m_counter && m_counter < m_interpolated_anime_start_count + kInterpolatedAnimeCount)
		{
			//アニメーション中は m_interpolated_node の補完されたノードを表示する
			int anime_index = static_cast<int>(m_interpolated_node.size()) * (m_counter - m_interpolated_anime_start_count) / kInterpolatedAnimeCount;

			m_hexapod_renderer.set_draw_node(m_interpolated_node[anime_index]);

			m_node_display_gui.SetDisplayNode(m_interpolated_node[anime_index]);
		}
		else if (m_counter == m_interpolated_anime_start_count + kInterpolatedAnimeCount)
		{
			//アニメーションが終了したら，元のノードを表示する
			m_hexapod_renderer.set_draw_node(m_node.at(m_display_node_index));

			m_node_display_gui.SetDisplayNode(m_node.at(m_display_node_index));
		}
	}


	m_counter++;				//カウンタを進める．

	m_camera_gui.Update();      //カメラのGUIを更新する．

	m_node_display_gui.Update();	//ノードの情報を表示するGUIを更新する．

	m_display_node_switch_gui.Update();	//ノードの情報を表示するGUIを更新する．


	//キー入力で表示を切り替える
	if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_L) == 1)
	{
		m_is_display_movement_locus = !m_is_display_movement_locus;
	}
	else if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_G) == 1)
	{
		m_is_display_robot_graund_point = !m_is_display_robot_graund_point;
	}

	return true;
}


void GraphicMainAdvance::Draw() const
{
	// 3Dのオブジェクトの描画

	designlab::dxlib_util::SetZBufferEnable();		//Zバッファを有効にする．


	WorldGridRenderer grid_renderer;	//インスタンスを生成する．

	grid_renderer.Draw();				//グリッドを描画する．


	MapRenderer map_render;				//マップを描画する．

	map_render.Draw(m_map_state);


	if (m_is_display_movement_locus)m_movement_locus_renderer.Draw(m_display_node_switch_gui.getSimulationNum());   //移動軌跡を描画する．

	if (m_is_display_robot_graund_point)m_robot_graund_point_renderer.Draw(m_display_node_switch_gui.getSimulationNum());


	if (!m_node.empty())
	{
		//ノードが存在しているならば，ロボットを描画する．
		m_hexapod_renderer.Draw();

		if (m_counter > m_interpolated_anime_start_count + kInterpolatedAnimeCount)
		{
			m_stability_margin_renderer.Draw(m_node.at(m_display_node_index));
		}
	}


	// 2DのGUIの描画

	m_camera_gui.Draw();        //カメラのGUIを描画する．

	m_node_display_gui.Draw();	 //ノードの情報を表示するGUIを描画する．

	m_display_node_switch_gui.Draw();	//表示するノードを切り替えるGUIを描画する．
}
