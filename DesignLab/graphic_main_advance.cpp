#include "graphic_main_advance.h"

#include <Dxlib.h>

#include "dxlib_util.h"
#include "hexapod_renderer_builder.h"
#include "keyboard.h"
#include "map_renderer.h"
#include "world_grid_renderer.h"


GraphicMainAdvance::GraphicMainAdvance(const std::shared_ptr<const GraphicDataBroker>& broker_ptr, const std::shared_ptr<const AbstractHexapodStateCalculator>& calculator_ptr,
	const std::shared_ptr<const ApplicationSettingRecorder>& setting_ptr) :
	kNodeGetCount(setting_ptr ? setting_ptr->window_fps * 2 : 60),
	kInterpolatedAnimeCount(60),
	broker_ptr_(broker_ptr),
	node_display_gui_(setting_ptr ? setting_ptr->window_size_x - NodeDisplayGui::kWidth - 10 : 10, 10, calculator_ptr),
	display_node_switch_gui_(10, setting_ptr ? setting_ptr->window_size_y - DisplayNodeSwitchGui::kGuiHeight - 10 : 10),
	hexapod_renderer_(HexapodRendererBuilder::Build(calculator_ptr, setting_ptr->gui_display_quality)),
	robot_graund_point_renderer_(calculator_ptr),
	stability_margin_renderer_(calculator_ptr),
	map_state_(broker_ptr ? broker_ptr->map_state.GetData() : MapState()),
	graph_({}),
	display_node_index_(0),
	counter_(0),
	interpolated_anime_start_count_(kInterpolatedAnimeCount * -10),
	is_displayed_movement_locus_(true),
	is_displayed_robot_graund_point_(true)
{
}


bool GraphicMainAdvance::Update()
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
		display_node_switch_gui_.SetGraphData(graph_.size(), simu_end_index);


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
		if (display_node_index_ != display_node_switch_gui_.GetDisplayNodeNum())
		{
			if (display_node_index_ > 0)
			{
				interpolated_anime_start_count_ = counter_;		//�A�j���[�V�������J�n�������Ԃ��L�^����D
				interpolated_node_creator_.createInterpolatedNode(graph_[display_node_index_], graph_[display_node_switch_gui_.GetDisplayNodeNum()], &interpolated_node_);
			}


			display_node_index_ = display_node_switch_gui_.GetDisplayNodeNum();				//�\������m�[�h���擾����D

			hexapod_renderer_->SetDrawNode(graph_.at(display_node_index_));							//���{�b�g�̏�Ԃ��X�V����D

			camera_gui_.SetHexapodPos(graph_.at(display_node_index_).global_center_of_mass);		//�J�����̈ʒu���X�V����D

			node_display_gui_.SetDisplayNode(graph_.at(display_node_index_));						//�m�[�h�̏���\������GUI�ɏ���`�B����D
		}

		if (interpolated_anime_start_count_ <= counter_ && counter_ < interpolated_anime_start_count_ + kInterpolatedAnimeCount)
		{
			//�A�j���[�V�������� interpolated_node_ �̕⊮���ꂽ�m�[�h��\������
			size_t anime_index = interpolated_node_.size() * (static_cast<size_t>(counter_) - static_cast<size_t>(interpolated_anime_start_count_))
				/ static_cast<size_t>(kInterpolatedAnimeCount);

			hexapod_renderer_->SetDrawNode(interpolated_node_[anime_index]);

			node_display_gui_.SetDisplayNode(interpolated_node_[anime_index]);
		}
		else if (counter_ == interpolated_anime_start_count_ + kInterpolatedAnimeCount)
		{
			//�A�j���[�V�������I��������C���̃m�[�h��\������
			hexapod_renderer_->SetDrawNode(graph_.at(display_node_index_));

			node_display_gui_.SetDisplayNode(graph_.at(display_node_index_));
		}
	}


	counter_++;				//�J�E���^��i�߂�D

	camera_gui_.Update();				//�J������GUI���X�V����D

	node_display_gui_.Update();			//�m�[�h�̏���\������GUI���X�V����D

	display_node_switch_gui_.Update();	//�m�[�h�̏���\������GUI���X�V����D


	//�L�[���͂ŕ\����؂�ւ���
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


void GraphicMainAdvance::Draw() const
{
	// 3D�̃I�u�W�F�N�g�̕`��

	designlab::dxlib_util::SetZBufferEnable();		//Z�o�b�t�@��L���ɂ���D


	WorldGridRenderer grid_renderer;	//�C���X�^���X�𐶐�����D

	grid_renderer.Draw();				//�O���b�h��`�悷��D


	MapRenderer map_render;				//�}�b�v��`�悷��D

	map_render.Draw(map_state_);


	if (is_displayed_movement_locus_)movement_locus_renderer_.Draw(display_node_switch_gui_.GetSimulationNum());   //�ړ��O�Ղ�`�悷��D

	if (is_displayed_robot_graund_point_)robot_graund_point_renderer_.Draw(display_node_switch_gui_.GetSimulationNum());


	if (!graph_.empty())
	{
		//�m�[�h�����݂��Ă���Ȃ�΁C���{�b�g��`�悷��D
		hexapod_renderer_->Draw();

		if (counter_ > interpolated_anime_start_count_ + kInterpolatedAnimeCount)
		{
			stability_margin_renderer_.Draw(graph_.at(display_node_index_));
		}
	}


	// 2D��GUI�̕`��

	camera_gui_.Draw();        //�J������GUI��`�悷��D

	node_display_gui_.Draw();	 //�m�[�h�̏���\������GUI��`�悷��D

	display_node_switch_gui_.Draw();	//�\������m�[�h��؂�ւ���GUI��`�悷��D
}