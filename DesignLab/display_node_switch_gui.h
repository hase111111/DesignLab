//! @file display_node_switch_gui.h
//! @brief �m�[�h�̕\���؂�ւ�GUI


#ifndef DESIGNLAB_DISPLAY_NODE_SWITCH_GUI_
#define DESIGNLAB_DISPLAY_NODE_SWITCH_GUI_


#include <vector>
#include <map>
#include <memory>

#include "button_controller.h"


//! @class DisplayNodeSwitchGui
//! @brief �m�[�h�̕\���؂�ւ�GUI
class DisplayNodeSwitchGui final
{
public:
	DisplayNodeSwitchGui(int x, int y);
	DisplayNodeSwitchGui();


	//! @brief GUI�ɕ\������m�[�h�̏���ݒ肷��
	//! @param[in] node_num �S�m�[�h��
	//! @param[in] simu_end_index �V�~�����[�V�����̏I���m�[�h�ԍ�
	void SetGraphData(size_t node_num, const std::vector<size_t>& simu_end_index);


	//! @brief ���ݕ\������m�[�h�̔ԍ����擾����
	//! @return size_t ���ݕ\������m�[�h�̔ԍ�
	size_t GetDisplayNodeNum() const;

	//! @brief ���ݕ\������V�~�����[�V�����̔ԍ����擾����
	//! @return int ���ݕ\������V�~�����[�V�����̔ԍ�
	int GetSimulationNum() const;


	//! @brief GUI�̍X�V�C���t���[�����s���邱��
	void Update();

	//! @brief GUI�̕`��
	void Draw() const;


	const static int kGuiWidth = 275;	//!< GUI�̕�

	const static int kGuiHeight = 250;	//!< GUI�̍���

private:

	//! @brief �{�^���̎��
	enum class ButtonType : int
	{
		kPrevNode,		//!< �O�̃m�[�h
		kNextNode,		//!< ���̃m�[�h
		kMostPrevNode,	//!< �ł��O�̃m�[�h��
		kMostNextNode,	//!< �ł����̃m�[�h��
		kPrevSimu,		//!< �O�̃V�~�����[�V����
		kNextSimu,		//!< ���̃V�~�����[�V����
		kPlayStop,		//!< �Đ�/��~
		kSpeedUp,		//!< ���x�A�b�v
		kSpeedDown,		//!< ���x�_�E��
	};


	//! �V�~�����[�V�����̒��ōŏ��̃m�[�h�Ɉړ�����֐�
	void MoveMostPrevNode();

	//! �V�~�����[�V�����̒��őO�̃m�[�h�Ɉړ�����֐�
	void MovePrevNode();

	//! �V�~�����[�V�����̒��ōŌ�̃m�[�h�Ɉړ�����֐�
	void MoveMostNextNode();

	//! �V�~�����[�V�����̒��Ŏ��̃m�[�h�Ɉړ�����֐�
	void MoveNextNode();

	//! �O�̃V�~�����[�V�����Ɉړ�����֐�
	void MovePrevSimulation();

	//! ���̃V�~�����[�V�����Ɉړ�����֐�
	void MoveNextSimulation();

	//! �S�V�~�����[�V�����������߂�
	int GetAllSimulationNum() const;



	const int kGuiLeftPosX;		//!< GUI�̍����X���W

	const int kGuiTopPosY;		//!< GUI�̍����Y���W

	const int kAnimeSpeedMax;	//!< �A�j���[�V�������x�̍ő�l

	const int kAnimeSpeedMin;	//!< �A�j���[�V�������x�̍ŏ��l


	std::map<ButtonType, std::unique_ptr<ButtomController>> button_;	//!< �{�^��


	size_t all_node_num_;			//!< �S�m�[�h��

	std::vector<size_t> simu_end_index_;	//!< �V�~�����[�V�����̏I���m�[�h�ԍ�

	size_t display_node_num_;		//!< �\������m�[�h�̔ԍ�

	int simulation_num_;			//!< �\������V�~�����[�V�����̔ԍ�


	bool do_auto_animation_;		//!< �����Đ������ǂ���

	int animation_speed_;			//!< �Đ����x

	int counter_;					//!< �J�E���^�[
};


#endif 