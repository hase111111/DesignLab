#include "graph_viewer_system_main.h"

#include <bitset>
#include <iostream>

#include <boost/thread.hpp>
#include <magic_enum.hpp>

#include "cmdio_util.h"
#include "define.h"
#include "graph_search_const.h"
#include "node_initializer.h"
#include "pass_finder_basic.h"
#include "phantomx_mk2.h"


namespace dlio = designlab::cmdio;


GraphViewerSystemMain::GraphViewerSystemMain(
	std::unique_ptr<IPassFinder>&& pass_finder_ptr,
	const std::shared_ptr<GraphicDataBroker>& broker_ptr,
	const std::shared_ptr<const ApplicationSettingRecorder>& setting_ptr
	) :
	pass_finder_ptr_(std::move(pass_finder_ptr)),
	broker_ptr_(broker_ptr),
	setting_ptr_(setting_ptr)
{
	dlio::OutputTitle("�O���t�m�F���[�h");	//�^�C�g����\������

	//�}�b�v�𐶐�����
	dlio::Output("�܂��́C�}�b�v�𐶐�����D�I�v�V�����𐮐��œ��͂��邱�ƁD", OutputDetail::kSystem);
	
	MapCreateMode selected_mode = InputMapCreateMode();
	unsigned int selected_option = InputMapCreateOption();

	SimulationMapCreator map_creator(selected_mode, selected_option);
	map_state_ = map_creator.InitMap();


	broker_ptr_->map_state.SetData(map_state_);	//����l������������
}


void GraphViewerSystemMain::Main()
{
	//�������^�[��
	if (!pass_finder_ptr_) 
	{
		dlio::Output("�O���t�؍쐬�N���X������������Ă��Ȃ��D�I������", OutputDetail::kError);
		return;
	}

	dlio::Output("�ʃX���b�h��GUI���N������D", OutputDetail::kInfo);	

	//�m�[�h������������
	dlio::Output("�m�[�h������������D", OutputDetail::kSystem);

	NodeInitializer node_initializer;
	RobotStateNode first_node = node_initializer.InitNode();
	std::vector<RobotStateNode> graph;


	while (true)
	{
		OutputGraphStatus(graph);

		if (graph.size() == 0)
		{
			// �O���t���Ȃ��ꍇ�C

			dlio::Output("�܂��O���t�𐶐����Ă��Ȃ��D", OutputDetail::kSystem);

			if (dlio::InputYesNo("�O���t���쐬���܂����H"))
			{
				CreateGraph(first_node, &graph);	// �O���t���쐬����D
				
				broker_ptr_->graph.SetData(graph);	// �O���t�؂̒l�𒇉�l�ɃZ�b�g����D�����GUI�ɃO���t���\�������D
			}
			else
			{
				//�I�����邩���₷��
				if (dlio::InputYesNo("�I�����܂����H")) { break; }
			}
		}
		else
		{
			//�O���t������ꍇ

			dlio::Output("�O���t�𑀍삷��", OutputDetail::kSystem);
			dlio::Output("���상�j���[��\�����܂�", OutputDetail::kSystem);

			//���상�j���[��\������

			std::vector<std::function<void()>> func_list;	//����������Ȃ��֐��������_���Ŏ󂯎��vector

			func_list.push_back(
				[&]() 
				{
					RobotStateNode selected = SelectNode(graph);
					CreateGraph(selected, &graph);
					broker_ptr_->graph.SetData(graph);
				}
			);

			func_list.push_back(
				[&]()
				{
					RobotStateNode selected = SelectNode(graph);

					dlio::OutputNewLine(1, OutputDetail::kSystem);
					dlio::OutputHorizontalLine("*", OutputDetail::kSystem);
					dlio::Output(selected.ToString(), OutputDetail::kSystem);
					dlio::OutputHorizontalLine("*", OutputDetail::kSystem);
					dlio::OutputNewLine(1, OutputDetail::kSystem);
				}
			);

			func_list.push_back(
				[&]()
				{
					graph.clear();
					broker_ptr_->graph.Clean();
					dlio::Output("�O���t��S�č폜����", OutputDetail::kSystem);
					dlio::OutputNewLine(1, OutputDetail::kSystem);
				}
			);

			
			dlio::OutputNewLine(1, OutputDetail::kSystem);
			dlio::Output("�����I�����Ă�������", OutputDetail::kSystem);
			dlio::Output("�@0 : �m�[�h�I�����C���̃m�[�h��e�ɂ��ăO���t�𐶐�����", OutputDetail::kSystem);
			dlio::Output("�@1 : �m�[�h�I�����ĕ\������", OutputDetail::kSystem);
			dlio::Output("�@2 : �O���t��S�폜����", OutputDetail::kSystem);
			dlio::Output("�@3 : �I������", OutputDetail::kSystem);

			int selected_index = dlio::InputInt(0, static_cast<int>(func_list.size()), 3, "�����ő����I�����Ă��������D�͈͊O�̒l�̏ꍇ�I�����܂��D");
	
			//�I�����ꂽ��������s����
			if (selected_index < func_list.size()) 
			{
				func_list[selected_index](); 
			}
			else 
			{
				if (dlio::InputYesNo("�I�����܂����H")) { break; }
			}
		}

	}	//while (true)
}


void GraphViewerSystemMain::CreateGraph(const RobotStateNode parent, std::vector<RobotStateNode>* graph)
{
	dlio::OutputNewLine(1, OutputDetail::kSystem);
	dlio::Output("�O���t�؂��쐬����", OutputDetail::kSystem);
	dlio::OutputNewLine(1, OutputDetail::kSystem);

	// �O���t�T��������
	RobotStateNode parent_node = parent;
	parent_node.ChangeParentNode();

	TargetRobotState target;
	target.target_mode = TargetMode::kStraightPosition;
	target.target_position = { 100000,0,0 };
	target.rotation_center = { 0,100000,0 };

	RobotStateNode fake_result_node;

	stopwatch_.Start();

	GraphSearchResult result =
		pass_finder_ptr_->GetNextNodebyGraphSearch(parent_node, map_state_, target, &fake_result_node);

	stopwatch_.End();


	// �O���t�T���̌��ʂ�\������
	dlio::OutputNewLine(1, OutputDetail::kSystem);
	dlio::Output("�O���t�T���I��", OutputDetail::kSystem);
	dlio::Output("�O���t�T���ɂ����������� : " + stopwatch_.GetElapsedMilliSecondString(), OutputDetail::kSystem);

	std::string res_str = magic_enum::enum_name<GraphSearchResult>(result).data();
	res_str.erase(0, 1);	//�擪��k���폜����

	dlio::Output("�O���t�T������ : " + res_str, OutputDetail::kSystem);

	// �l��Ԃ��D
	graph->clear();
	pass_finder_ptr_->GetGraphTree(graph);
}

void GraphViewerSystemMain::OutputGraphStatus(const std::vector<RobotStateNode>& graph) const
{
	dlio::OutputNewLine(1, OutputDetail::kSystem);
	dlio::OutputHorizontalLine("=", OutputDetail::kSystem);
	dlio::OutputNewLine(1, OutputDetail::kSystem);
	dlio::Output("�O���t�̏�Ԃ�\�����܂��D", OutputDetail::kSystem);
	dlio::OutputNewLine(1, OutputDetail::kSystem);
	dlio::Output("�O���t�̃m�[�h�̐� : " + std::to_string(graph.size()), OutputDetail::kSystem);


	if (graph.size() > 0)
	{
		//�[�����Ƃ̃m�[�h�����L�^����
		
		std::vector<int> depth_num(GraphSearchConst::kMaxDepth + 1);	

		dlio::Output("GraphViewerSystemMain : �O���t�T���̍ő�[�� : " + std::to_string(GraphSearchConst::kMaxDepth), OutputDetail::kSystem);

		for (const auto& i : graph)
		{
			if (i.depth < depth_num.size()) 
			{
				depth_num[i.depth]++;
			}
		}

		//�[�����Ƃ̃m�[�h����\������

		int depth_cnt = 0;

		for (const auto& i : depth_num)
		{
			dlio::Output("�E�[��" + std::to_string(depth_cnt) + " : " + std::to_string(i), OutputDetail::kSystem);
			depth_cnt++;
		}
	}
	else 
	{
		dlio::Output("�O���t����Ȃ̂ŁC�[�����Ƃ̃m�[�h����\���ł��܂���D", OutputDetail::kSystem);
	}

	dlio::OutputNewLine(1, OutputDetail::kSystem);
	dlio::OutputHorizontalLine("=", OutputDetail::kSystem);
	dlio::OutputNewLine(1, OutputDetail::kSystem);
}

MapCreateMode GraphViewerSystemMain::InputMapCreateMode() const
{
	const auto kMapCreateModeList = magic_enum::enum_values<MapCreateMode>();	//MapCreateMode�̃��X�g���擾����

	dlio::OutputNewLine(1, OutputDetail::kSystem);
	dlio::Output("MapCreateMode��I��", OutputDetail::kSystem);

	
	//MapCreateMode�̈ꗗ���o�͂���D
	for (int i = 0; i < kMapCreateModeList.size(); i++)
	{
		std::string name = magic_enum::enum_name<MapCreateMode>(kMapCreateModeList[i]).data();	//MapCreateMode�̖��O���擾����

		name.erase(0, 1);	//�擪��k���폜����

		dlio::Output(std::to_string(i) + " : " + name, OutputDetail::kSystem);
	}


	int selected_mode_index = dlio::InputInt(0, static_cast<int>(kMapCreateModeList.size()) - 1, 0);	//MapCreateMode��index����͂�����

	return kMapCreateModeList[selected_mode_index];
}

unsigned int GraphViewerSystemMain::InputMapCreateOption() const
{
	const auto kMapCreateOptionList = magic_enum::enum_values<MapCreateOption>();	//MapCreateOption�̃��X�g���擾����

	//MapCreateOption�̍��v�l���v�Z����
	unsigned int option_sum = 0;

	for (const auto i : kMapCreateOptionList) 
	{
		option_sum += static_cast<unsigned int>(i);
	}


	dlio::OutputNewLine(1, OutputDetail::kSystem);
	dlio::Output("MapCreateOption��I�� (�����w�肵�����ꍇ�͒l�𑫂��Z���邱��)", OutputDetail::kSystem);


	//MapCreateOption�̈ꗗ���o�͂���D
	for (int i = 0; i < kMapCreateOptionList.size(); i++)
	{
		std::string name = magic_enum::enum_name<MapCreateOption>(kMapCreateOptionList[i]).data();	//MapCreateOption�̃��X�g���擾����

		name.erase(0, 1);	//�擪��k���폜����

		unsigned int option_value = static_cast<unsigned int>(kMapCreateOptionList[i]);

		std::bitset<magic_enum::enum_count<MapCreateOption>()> bit(option_value);

		dlio::Output(std::to_string(option_value) + " : " + name + " (" + bit.to_string() + ")", OutputDetail::kSystem);
	}

	int selected_option = dlio::InputInt(0, option_sum, 0);	//MapCreateOption�̍��v�l����͂�����

	return selected_option;
}

RobotStateNode GraphViewerSystemMain::SelectNode(const std::vector<RobotStateNode>& graph) const
{
	dlio::OutputNewLine(1, OutputDetail::kSystem);
	dlio::Output("�m�[�h��I������", OutputDetail::kSystem);

	if (graph.size() == 0)
	{
		dlio::Output("�O���t����Ȃ̂ŁC������Ԃ̃m�[�h��Ԃ�", OutputDetail::kSystem);

		NodeInitializer node_initializer;
		RobotStateNode first_node = node_initializer.InitNode();

		return first_node;
	}
	else
	{
		dlio::Output("�O���t�̒�����1�̃m�[�h��I�����Ă��������D", OutputDetail::kSystem);

		//�m�[�h��I������
		int selected_node_index = dlio::InputInt(0, static_cast<int>(graph.size()) - 1, 0 , "�����Ńm�[�h��I�����Ă��������D");

		dlio::Output("�I�����ꂽ�m�[�h�C" + std::to_string(selected_node_index) + "�Ԃ�e�ɂ���D", OutputDetail::kSystem);

		return graph[selected_node_index];
	}
}