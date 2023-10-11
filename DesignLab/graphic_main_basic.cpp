#include "graphic_main_basic.h"

#include <Dxlib.h>

#include "dxlib_util.h"
#include "keyboard.h"
#include "map_renderer.h"
#include "world_grid_renderer.h"


GraphicMainBasic::GraphicMainBasic(const std::shared_ptr<const GraphicDataBroker>& broker_ptr, const std::shared_ptr<const AbstractHexapodStateCalculator>& calculator_ptr,
	const std::shared_ptr<const ApplicationSettingRecorder>& setting_ptr) :
	kNodeGetCount(setting_ptr ? setting_ptr->window_fps * 2 : 60),
	broker_ptr_(broker_ptr),
	node_display_gui_(setting_ptr ? setting_ptr->window_size_x - NodeDisplayGui::kWidth - 10 : 0, 10, calculator_ptr),
	display_node_switch_gui_(10, setting_ptr ? setting_ptr->window_size_y - DisplayNodeSwitchGui::GUI_HEIGHT - 10 : 0),
	hexapod_renderer_(calculator_ptr),
	robot_graund_point_renderer_(calculator_ptr),
	stability_margin_renderer_(calculator_ptr),
	map_state_(broker_ptr_ ? broker_ptr_->map_state.GetData() : MapState()),
	graph_({}),
	display_node_index_(0),
	counter_(0),
	is_displayed_movement_locus_(false),
	is_displayed_robot_graund_point_(false)
{
}


bool GraphicMainBasic::Update()
{
	if (map_update_count != broker_ptr_->map_state.GetUpdateCount())
	{
		map_update_count = broker_ptr_->map_state.GetUpdateCount();
		map_state_ = broker_ptr_->map_state.GetData();
	}

	//�m�[�h��ǂݏo�����ԂɂȂ�����C����l����f�[�^��ǂݏo���D
	if (counter_ % kNodeGetCount == 0 && graph_update_count != broker_ptr_->graph.GetUpdateCount())
	{
		//����l����f�[�^��ǂݏo��
		graph_ = broker_ptr_->graph.GetData();

		std::vector<size_t> simu_end_index;

		simu_end_index = broker_ptr_->simu_end_index.GetData();

		//�m�[�h�̏���\������GUI�ɏ���`�B����D
		display_node_switch_gui_.setGraphData(graph_.size(), simu_end_index);



		//�ړ��O�Ղ��X�V����D
		movement_locus_renderer_.SetMoveLocusPoint(graph_);

		movement_locus_renderer_.SetSimulationEndIndexes(simu_end_index);


		//���{�b�g�̐ڒn�_���X�V����D
		robot_graund_point_renderer_.SetNodeAndSimulationEndNodeIndex(graph_, simu_end_index);


		graph_update_count = broker_ptr_->graph.GetUpdateCount();
	}


	//�m�[�h�����݂��Ă���̂Ȃ�΁C�e�N���X�ɏ���`�B����
	if (!graph_.empty())
	{
		// �\���m�[�h���ύX���ꂽ��C�\������m�[�h��ύX����D
		if (display_node_index_ != display_node_switch_gui_.getDisplayNodeNum())
		{
			display_node_index_ = display_node_switch_gui_.getDisplayNodeNum();	//�\������m�[�h���擾����D

			hexapod_renderer_.SetDrawNode(graph_.at(display_node_index_));					//���{�b�g�̏�Ԃ��X�V����D

			camera_gui_.SetHexapodPos(graph_.at(display_node_index_).global_center_of_mass);		//�J�����̈ʒu���X�V����D

			node_display_gui_.SetDisplayNode(graph_.at(display_node_index_));			//�m�[�h�̏���\������GUI�ɏ���`�B����D
		}
	}


	++counter_;				//�J�E���^��i�߂�D

	camera_gui_.Update();				//�J������GUI���X�V����D

	node_display_gui_.Update();			//�m�[�h�̏���\������GUI���X�V����D

	display_node_switch_gui_.Update();	//�m�[�h�̏���\������GUI���X�V����D


	// �L�[���͂ŕ\����؂�ւ���
	// TODO : ���Ƃ�GUI�Ɉڍs����
	if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_L) == 1)
	{
		is_displayed_movement_locus_ = !is_displayed_movement_locus_;
	}
	else if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_G) == 1)
	{
		is_displayed_robot_graund_point_ = !is_displayed_robot_graund_point_;
	}


	return true;
}


void GraphicMainBasic::Draw() const
{
	// 3D�̃I�u�W�F�N�g�̕`��

	designlab::dxlib_util::SetZBufferEnable();		//Z�o�b�t�@��L���ɂ���D


	WorldGridRenderer grid_renderer;	//�C���X�^���X�𐶐�����D

	grid_renderer.Draw();				//�O���b�h��`�悷��D


	MapRenderer map_render;				//�}�b�v��`�悷��D

	map_render.Draw(map_state_);


	if (is_displayed_movement_locus_)
	{
		movement_locus_renderer_.Draw(display_node_switch_gui_.getSimulationNum());   //�ړ��O�Ղ�`�悷��D
	}

	if (is_displayed_robot_graund_point_)
	{
		robot_graund_point_renderer_.Draw(display_node_switch_gui_.getSimulationNum());
	}


	if (!graph_.empty())
	{
		//�m�[�h�����݂��Ă���Ȃ�΁C���{�b�g��`�悷��D
		hexapod_renderer_.Draw();

		stability_margin_renderer_.Draw(graph_.at(display_node_index_));
	}


	// 2D��GUI�̕`��

	camera_gui_.Draw();        //�J������GUI��`�悷��D

	node_display_gui_.Draw();		//�m�[�h�̏���\������GUI��`�悷��D

	display_node_switch_gui_.Draw();	//�\������m�[�h��؂�ւ���GUI��`�悷��D
}
