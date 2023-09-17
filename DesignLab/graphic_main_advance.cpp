#include "graphic_main_advance.h"

#include "DxLib.h"

#include "designlab_dxlib.h"
#include "world_grid_renderer.h"
#include "map_renderer.h"
#include "keyboard.h"


GraphicMainAdvance::GraphicMainAdvance(const GraphicDataBroker* const  broker, std::shared_ptr<AbstractHexapodStateCalculator> calc, const SApplicationSettingRecorder* const setting)
	: AbstractGraphicMain(broker, calc, setting),
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

	//�m�[�h��ǂݏo�����ԂɂȂ�����C����l����f�[�^��ǂݏo���D
	if (m_counter % kNodeGetCount == 0)
	{
		//����l����f�[�^��ǂݏo��
		mp_broker->CopyOnlyNewNode(&m_node);

		std::vector<size_t> simu_end_index;

		mp_broker->CopySimuEndIndex(&simu_end_index);


		//�m�[�h�̏���\������GUI�ɏ���`�B����D
		m_display_node_switch_gui.setGraphData(m_node.size(), simu_end_index);



		//�ړ��O�Ղ��X�V����D
		m_movement_locus_renderer.setMovementLocus(m_node);

		m_movement_locus_renderer.setSimuEndIndex(simu_end_index);


		//���{�b�g�̐ڒn�_���X�V����D
		m_robot_graund_point_renderer.setNode(m_node, simu_end_index);
	}


	//�m�[�h�����݂��Ă���̂Ȃ�΁C�e�N���X�ɏ���`�B����
	if (!m_node.empty())
	{
		// �\���m�[�h���ύX���ꂽ��C�\������m�[�h��ύX����D
		if (m_display_node_index != (int)m_display_node_switch_gui.getDisplayNodeNum())
		{
			if (m_display_node_index > 0)
			{
				m_interpolated_anime_start_count = m_counter;		//�A�j���[�V�������J�n�������Ԃ��L�^����D
				m_interpolated_node_creator.createInterpolatedNode(m_node[m_display_node_index], m_node[m_display_node_switch_gui.getDisplayNodeNum()], &m_interpolated_node);
			}


			m_display_node_index = (int)m_display_node_switch_gui.getDisplayNodeNum();				//�\������m�[�h���擾����D

			m_hexapod_renderer.setNode(m_node.at(m_display_node_index));							//���{�b�g�̏�Ԃ��X�V����D

			m_camera_gui.setHexapodPos(m_node.at(m_display_node_index).global_center_of_mass);		//�J�����̈ʒu���X�V����D

			m_node_display_gui.SetDisplayNode(m_node.at(m_display_node_index));						//�m�[�h�̏���\������GUI�ɏ���`�B����D
		}

		if (m_interpolated_anime_start_count <= m_counter && m_counter < m_interpolated_anime_start_count + kInterpolatedAnimeCount)
		{
			//�A�j���[�V�������� m_interpolated_node �̕⊮���ꂽ�m�[�h��\������
			int anime_index = static_cast<int>(m_interpolated_node.size()) * (m_counter - m_interpolated_anime_start_count) / kInterpolatedAnimeCount;

			m_hexapod_renderer.setNode(m_interpolated_node[anime_index]);

			m_node_display_gui.SetDisplayNode(m_interpolated_node[anime_index]);
		}
		else if (m_counter == m_interpolated_anime_start_count + kInterpolatedAnimeCount)
		{
			//�A�j���[�V�������I��������C���̃m�[�h��\������
			m_hexapod_renderer.setNode(m_node.at(m_display_node_index));

			m_node_display_gui.SetDisplayNode(m_node.at(m_display_node_index));
		}
	}


	m_counter++;				//�J�E���^��i�߂�D

	m_camera_gui.Update();      //�J������GUI���X�V����D

	m_node_display_gui.Update();	//�m�[�h�̏���\������GUI���X�V����D

	m_display_node_switch_gui.Update();	//�m�[�h�̏���\������GUI���X�V����D


	//�L�[���͂ŕ\����؂�ւ���
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
	// 3D�̃I�u�W�F�N�g�̕`��

	dl_dxlib::setZBufferEnable();		//Z�o�b�t�@��L���ɂ���D


	WorldGridRenderer grid_renderer;	//�C���X�^���X�𐶐�����D

	grid_renderer.Draw();				//�O���b�h��`�悷��D


	MapRenderer map_render;				//�}�b�v��`�悷��D

	map_render.Draw(m_map_state);


	if (m_is_display_movement_locus)m_movement_locus_renderer.Draw(m_display_node_switch_gui.getSimulationNum());   //�ړ��O�Ղ�`�悷��D

	if (m_is_display_robot_graund_point)m_robot_graund_point_renderer.Draw(m_display_node_switch_gui.getSimulationNum());


	if (!m_node.empty())
	{
		//�m�[�h�����݂��Ă���Ȃ�΁C���{�b�g��`�悷��D
		m_hexapod_renderer.Draw();

		if (m_counter > m_interpolated_anime_start_count + kInterpolatedAnimeCount)
		{
			m_stability_margin_renderer.Draw(m_node.at(m_display_node_index));
		}
	}


	// 2D��GUI�̕`��

	m_camera_gui.Draw();        //�J������GUI��`�悷��D

	m_node_display_gui.Draw();	 //�m�[�h�̏���\������GUI��`�悷��D

	m_display_node_switch_gui.Draw();	//�\������m�[�h��؂�ւ���GUI��`�悷��D
}
