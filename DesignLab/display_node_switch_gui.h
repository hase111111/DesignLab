#pragma once

#include <vector>
#include <map>
#include <memory>

#include "button_controller.h"


//! @class DisplayNodeSwitchGUI
//! @date 2023/08/30
//! @author ���J��
//! @brief �m�[�h�̕\���؂�ւ�GUI
class DisplayNodeSwitchGUI final
{
public:
	DisplayNodeSwitchGUI(const int x, const int y);
	DisplayNodeSwitchGUI();


	//! @brief GUI�ɕ\������m�[�h�̏���ݒ肷��
	//! @param[in] node_num �S�m�[�h��
	//! @param[in] simu_end_index �V�~�����[�V�����̏I���m�[�h�ԍ�
	void setGraphData(const size_t node_num, const std::vector<size_t>& simu_end_index);


	//! @brief ���ݕ\������m�[�h�̔ԍ����擾����
	//! @return size_t ���ݕ\������m�[�h�̔ԍ�
	size_t getDisplayNodeNum() const;

	//! @brief ���ݕ\������V�~�����[�V�����̔ԍ����擾����
	//! @return int ���ݕ\������V�~�����[�V�����̔ԍ�
	int getSimulationNum() const;


	//! @brief GUI�̍X�V�C���t���[�����s���邱��
	void Update();

	//! @brief GUI�̕`��
	void Draw() const;


	const static int GUI_WIDTH = 275;	//!< GUI�̕�

	const static int GUI_HEIGHT = 250;	//!< GUI�̍���

private:

	//! @brief �{�^���̎��
	enum class EButtonType : int
	{
		PrevNode,		//!< �O�̃m�[�h
		NextNode,		//!< ���̃m�[�h
		MostPrevNode,	//!< �ł��O�̃m�[�h��
		MostNextNode,	//!< �ł����̃m�[�h��
		PrevSimu,		//!< �O�̃V�~�����[�V����
		NextSimu,		//!< ���̃V�~�����[�V����
		PlayStop,		//!< �Đ�/��~
		SpeedUp,		//!< ���x�A�b�v
		SpeedDown,		//!< ���x�_�E��
	};


	// �V�~�����[�V�����̒��ōŏ��̃m�[�h�Ɉړ�����֐�
	void moveMostPrevNode();

	// �V�~�����[�V�����̒��őO�̃m�[�h�Ɉړ�����֐�
	void movePrevNode();

	// �V�~�����[�V�����̒��ōŌ�̃m�[�h�Ɉړ�����֐�
	void moveMostNextNode();

	// �V�~�����[�V�����̒��Ŏ��̃m�[�h�Ɉړ�����֐�
	void moveNextNode();

	//�O�̃V�~�����[�V�����Ɉړ�����֐�
	void movePrevSimulation();

	//���̃V�~�����[�V�����Ɉړ�����֐�
	void moveNextSimulation();

	//�S�V�~�����[�V�����������߂�
	int getAllSimulationNum() const;



	const int kGUILeftPosX;		//!< GUI�̍����X���W

	const int kGUITopPosY;		//!< GUI�̍����Y���W

	const int kAnimeSpeedMax = 120;	//!< �A�j���[�V�������x�̍ő�l

	const int kAnimeSpeedMin = 1;	//!< �A�j���[�V�������x�̍ŏ��l


	std::map<EButtonType, std::unique_ptr<ButtomController>> m_button;	//!< �{�^��


	size_t m_all_node_num;			//!< �S�m�[�h��

	std::vector<size_t> m_simu_end_index;	//!< �V�~�����[�V�����̏I���m�[�h�ԍ�

	size_t m_display_node_num;		//!< �\������m�[�h�̔ԍ�

	int m_simulation_num;			//!< �\������V�~�����[�V�����̔ԍ�


	bool m_do_auto_animation;		//!< �����Đ������ǂ���

	int m_animation_speed;			//!< �Đ����x

	int m_counter;					//!< �J�E���^�[
};



//! @file display_node_switch_gui.h
//! @date 2023/08/30
//! @author ���J��
//! @brief �m�[�h�̕\���؂�ւ�GUI
//! @n �s�� : @lineinfo
