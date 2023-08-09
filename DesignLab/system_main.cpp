#include "system_main.h"

#include <boost/thread.hpp>

#include "Define.h"
#include "my_math.h"
#include "my_timer.h"
#include "CmdIO.h"
#include "hexapod.h"
#include "HexapodStateCalculator.h"
#include "node_validity_checker.h"
#include "graphic_main_basic.h"
#include "graphic_main_test.h"


SystemMain::SystemMain(std::unique_ptr<IPassFinder>&& graph_search)
{
	//���{�b�g�̃f�[�^������������D
	Hexapod::makeLegROM_r();
	HexapodStateCalclator::initLegR();

	//�}�b�v�𐶐�����D
	m_map_state.init(EMapCreateMode::LATTICE_POINT, MapCreator::OPTION_SLOPE, true);

	//����l�Ƀ}�b�v��n���D
	m_broker.setMapState(m_map_state);

	//�O���t�T���N���X���Z�b�g����
	mp_pass_finder = std::move(graph_search);

	//�摜�E�B���h�E��\������N���X�ɒ���l�̃A�h���X��n���āC����������������D
	m_graphic_system.init(std::make_unique<GraphicMainBasic>(&m_broker));

	//���̒T���ł̖ڕW��ݒ肷��D
	m_target.TargetMode = ETargetMode::StraightPosition;
	m_target.TargetPosition = { 3000,0,0 };
}

void SystemMain::main()
{
	if (!mp_pass_finder)
	{
		//�O���t�T���N���X���Z�b�g����Ă��Ȃ��ꍇ�́C�G���[���o�͂��ďI������D
		std::cout << "GraphSearch is not set." << std::endl;
		return;
	}

	CmdIO _cmd;	//�R�}���h���C���ɕ�����`�悷��N���X��p�ӂ���D

	NodeValidityChecker node_checker;	//�m�[�h�̑Ó������`�F�b�N����N���X��p�ӂ���D

	//�摜�\���E�B���h�E��ʃX���b�h�ŗ����グ��D�������Ɏ��s������C���������摜�\�������Ȃ��ݒ�ɂȂ��Ă���Ɨ����オ��Ȃ��D
	boost::thread graphic_thread(&GraphicSystem::main, &m_graphic_system);

	//�V�~�����[�V�������s���񐔕����[�v����D
	for (int i = 0; i < Define::SIMURATE_NUM; i++)
	{
		SNode _current_node;									//���݂̃m�[�h�̏�Ԃ��i�[����ϐ��D
		const bool _do_random_init = (i == 0) ? false : true;	// i �̒l�� 0 �Ȃ�΃����_���ȏꏊ�ɏ������͂��Ȃ��D(i == 0)��]�����āCtrue�Ȃ�ΑO��(false)�Cfalse�Ȃ�Ό��(true)��������D
		_current_node.init(_do_random_init);

		if (Define::FLAG_GRAPHIC_AVAILABLE == true) { m_broker.pushNode(_current_node); }	//�O���t�B�b�N���L���Ȃ�΁C����l�ɍŏ��̃m�[�h�̏�Ԃ�ʒB����D

		_cmd.outputGraphSearchStaretMessage(i + 1);	//�R�}���h���C���ɊJ�n���̃��b�Z�[�W���o�͂���D
		_cmd.outputNode(_current_node, 0);			//�R�}���h���C���ɍŏ��̃m�[�h�̏�Ԃ��o�͂���D


		//�ő���e�����񐔕��܂Ń��[�v����D
		for (int i = 0; i < Define::GATE_PATTERN_GENERATE_NUM; i++)
		{
			SNode result_node;				//�O���t�T���̌��ʂ��i�[����ϐ��D
			MyTimer timer;					//�^�C�}�[��p�ӂ���D

			timer.start();		//�^�C�}�[�X�^�[�g

			EGraphSearchResult result_state = mp_pass_finder->getNextNodebyGraphSearch(_current_node, &m_map_state, m_target, result_node);		//�O���t�T�����s���D

			timer.end();		//�^�C�}�[�X�g�b�v


			if (graphSeachResultIsSuccessful(result_state) == false)
			{
				//���̕��e�������ł��Ȃ�������C���[�v��������C���̃V�~�����[�V�����֐i�ށD
				_cmd.outputErrorMessageInGraphSearch("Failed to generate the next gait.");
				break;
			}

			_current_node = result_node;		//���̕��e�������ł��Ă���Ȃ�΁C�m�[�h���X�V����D

			if (Define::FLAG_GRAPHIC_AVAILABLE == true) { m_broker.pushNode(_current_node); }	//�O���t�B�b�N���L���Ȃ�Β���l�Ɍ��ʂ�ʒB����D
			_cmd.outputNode(_current_node, i + 1);												//�R�}���h���C���Ɍ��݂̃m�[�h���o�͂���D


			node_checker.setNode(_current_node);		//����`�F�b�J�[�ɂ��m�[�h��ʒB����D

			if (node_checker.isLoopMove() == true)
			{
				//���삪���[�v���Ă��܂��Ă���Ȃ�΁C���[�v��������C���̃V�~�����[�V�����֐i�ށD
				_cmd.outputErrorMessageInGraphSearch("Motion stuck in a loop.");
				break;
			}

			if (_current_node.global_center_of_mass.x > Define::GOAL_TAPE) { break; }
		}


	}

	//�摜�\���E�B���h�E�̏I����҂D
	std::cout << "Waiting for dxlib to finish." << std::endl;
	graphic_thread.join();
}
