#include "system_main.h"

#include <boost/thread.hpp>

#include "Define.h"
#include "designlab_math.h"
#include "CmdIO.h"
#include "hexapod.h"
#include "hexapod_state_calculator.h"
#include "node_validity_checker.h"
#include "graphic_main_basic.h"
#include "graphic_main_test.h"


SystemMain::SystemMain(std::unique_ptr<IPassFinder>&& graph_search)
{
	//���{�b�g�̃f�[�^������������D
	Hexapod::makeLegROM_r();
	HexapodStateCalclator::initLegR();

	//���ʂ��t�@�C���ɏo�͂���N���X������������D
	m_result_exporter.init();

	//�}�b�v�𐶐�����D
	m_map_state.init(EMapCreateMode::FLAT, MapCreator::OPTION_PERFORATED | MapCreator::OPTION_ROUGH, true);

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
		SNode current_node;										//���݂̃m�[�h�̏�Ԃ��i�[����ϐ��D
		const bool do_random_init = true;//(i == 0) ? false : true;	// i �̒l�� 0 �Ȃ�΃����_���ȏꏊ�ɏ������͂��Ȃ��D(i == 0)��]�����āCtrue�Ȃ�ΑO��(false)�Cfalse�Ȃ�Ό��(true)��������D
		current_node.init(do_random_init);

		SSimulationRecord record;	//�V�~�����[�V�����̌��ʂ��i�[����ϐ��D
		record.result_nodes.push_back(current_node);	//�V�~�����[�V�����̌��ʂ��i�[����ϐ��Ɍ��݂̃m�[�h�̏�Ԃ�ǉ�����D
		record.simulation_result = ESimulationResult::FAILURE_BY_NODE_LIMIT_EXCEEDED;	//�V�~�����[�V�����̌��ʂ��i�[����ϐ��𐬌��ɏ���������D


		if (Define::FLAG_GRAPHIC_AVAILABLE) { m_broker.pushNode(current_node); }	//�O���t�B�b�N���L���Ȃ�΁C����l�ɍŏ��̃m�[�h�̏�Ԃ�ʒB����D


		_cmd.outputGraphSearchStaretMessage(i + 1);	//�R�}���h���C���ɊJ�n���̃��b�Z�[�W���o�͂���D
		_cmd.outputNode(current_node, 0);			//�R�}���h���C���ɍŏ��̃m�[�h�̏�Ԃ��o�͂���D


		//�ő���e�����񐔕��܂Ń��[�v����D
		for (int i = 0; i < Define::GATE_PATTERN_GENERATE_NUM; i++)
		{
			m_timer.start();		//�^�C�}�[�X�^�[�g

			SNode result_node;		//�O���t�T���̌��ʂ��i�[����ϐ��D

			EGraphSearchResult result_state = mp_pass_finder->getNextNodebyGraphSearch(current_node, &m_map_state, m_target, result_node);		//�O���t�T�����s���D

			m_timer.end();			//�^�C�}�[�X�g�b�v


			record.computation_time.push_back(m_timer.getMicroSecond() / 1000);	//�v�Z���Ԃ��i�[����D
			record.graph_search_results.push_back(result_state);			//�O���t�T���̌��ʂ��i�[����D
			record.result_nodes.push_back(result_node);	//�V�~�����[�V�����̌��ʂ��i�[����ϐ��Ɍ��݂̃m�[�h�̏�Ԃ�ǉ�����D


			if (!graphSeachResultIsSuccessful(result_state))
			{
				_cmd.outputErrorMessageInGraphSearch("Failed to generate the next gait.");

				record.simulation_result = ESimulationResult::FAILURE_BY_GRAPH_SEARCH;	//�V�~�����[�V�����̌��ʂ��i�[����ϐ������s�ɍX�V����D

				break;	//���̕��e�������ł��Ȃ�������C���̃��[�v�𔲂��C���̃V�~�����[�V�����֐i�ށD
			}


			current_node = result_node;		//���̕��e�������ł��Ă���Ȃ�΁C�m�[�h���X�V����D

			if (Define::FLAG_GRAPHIC_AVAILABLE) { m_broker.pushNode(current_node); }			//�O���t�B�b�N���L���Ȃ�Β���l�Ɍ��ʂ�ʒB����D

			_cmd.outputNode(current_node, i + 1);												//�R�}���h���C���Ɍ��݂̃m�[�h���o�͂���D

			node_checker.setNode(current_node);													//����`�F�b�J�[�ɂ��m�[�h��ʒB����D


			if (node_checker.isLoopMove())
			{
				_cmd.outputErrorMessageInGraphSearch("Motion stuck in a loop.");

				record.simulation_result = ESimulationResult::FAILURE_BY_LOOP_MOTION;	//�V�~�����[�V�����̌��ʂ��i�[����ϐ������s�ɍX�V����D

				break;	//���삪���[�v���Ă��܂��Ă���Ȃ�΁C���[�v��������C���̃V�~�����[�V�����֐i�ށD
			}

			if (current_node.global_center_of_mass.x > Define::GOAL_TAPE)
			{
				record.simulation_result = ESimulationResult::SUCCESS;	//�V�~�����[�V�����̌��ʂ��i�[����ϐ��𐬌��ɍX�V����D

				break;
			}
		}

		m_result_exporter.exportResult(record);	//�V�~�����[�V�����̌��ʂ��t�@�C���ɏo�͂���D

	}

	//�摜�\���E�B���h�E�̏I����҂D
	std::cout << "Waiting for dxlib to finish." << std::endl;
	graphic_thread.join();
}
