#include "graph_viewer_system_main.h"

#include <iostream>

#include <boost/thread.hpp>

#include "define.h"
#include "cmdio_util.h"
#include "stopwatch.h"
#include "graph_search_const.h"
#include "pass_finder_basic.h"
#include "phantomx_state_calculator.h"
#include "StringToValue.h"


// ��x�ƒǋL���Ȃ����낤�ƑS�Ă��ׂ��������Ă��܂��D
// �߂��Ⴍ����ǂ݂Â炢���낤�Ǝv�����ǁC�����āD

namespace dlio = designlab::cmdio;
using StrtoVal::StrToInt;


GraphViewerSystemMain::GraphViewerSystemMain(
	std::unique_ptr<IPassFinder>&& pass_finder_ptr,
	std::unique_ptr<IGraphicMain>&& graphic_main_ptr,
	const std::shared_ptr<GraphicDataBroker>& broker_ptr,
	const std::shared_ptr<const ApplicationSettingRecorder>& setting_ptr) :
	graphic_system_(std::move(graphic_main_ptr), setting_ptr),
	pass_finder_ptr_(std::move(pass_finder_ptr)),
	broker_ptr_(broker_ptr),
	setting_ptr_(setting_ptr)
{
	dlio::OutputTitle("�O���t�m�F���[�h");	//�^�C�g����\������

	//�}�b�v�𐶐�����
	std::cout << "GraphViewerSystemMain : �}�b�v�𐶐����܂��D" << std::endl;
	std::cout << "GraphViewerSystemMain : �I�v�V��������͂��Ă�������" << std::endl;
	MapCreator::PrintAllMapCreateMode();
	std::string _mode;
	std::cout << std::endl << "input : ";
	std::cin >> _mode;
	std::cout << std::endl;

	MapCreator::printAllMapCreateOption();
	std::string _option;
	std::cout << std::endl << "input : ";
	std::cin >> _option;
	std::cout << std::endl;
	MapCreator map_creator;
	map_state_ = map_creator.Create(static_cast<MapCreateMode>(StrToInt(_mode)), StrToInt(_option));
	std::cout << "MapCreator : �}�b�v�𐶐����܂����D" << std::endl << std::endl;


	//����l������������
	std::cout << "GraphicDataBroker_Old : ����l�����������܂��D" << std::endl << std::endl;
	broker_ptr_->map_state.SetData(map_state_);
}


void GraphViewerSystemMain::Main()
{
	//�O���t�B�b�N�V�X�e�����N������
	dlio::Output("�ʃX���b�h��GUI���N�����܂��D", OutputDetail::kInfo);

	boost::thread graphic_thread(&GraphicSystem::Main, &graphic_system_);

	//�m�[�h������������
	std::cout << "GraphViewerSystemMain : �m�[�h�����������܂��D" << std::endl << std::endl;
	RobotStateNode _node;
	_node.Init(false);
	std::cout << _node.ToString();
	std::cout << std::endl;

	std::vector<RobotStateNode> _graph;

	while (true)
	{
		std::cout << "--------------------------------------------------" << std::endl;
		std::cout << std::endl;
		showGraphStatus(_graph);

		if (_graph.size() == 0)
		{
			std::cout << "GraphViewerSystemMain : �܂��O���t�𐶐����Ă��܂���" << std::endl;

			if (askYesNo("GraphViewerSystemMain : �O���t���쐬���܂����H"))
			{
				if (!pass_finder_ptr_)
				{
					std::cout << "GraphViewerSystemMain : �O���t�؍쐬�N���X������������Ă��܂���" << std::endl;
					std::cout << "GraphViewerSystemMain : �v���O�������I�����܂�" << std::endl;
					break;
				}

				std::cout << "IGraphTreeCreator : �O���t���쐬���܂�" << std::endl;

				Stopwatch _timer;
				_timer.Start();
				CreateGraph(_node, _graph);
				_timer.End();
				SetGraphToBroker(_graph);
				std::cout << "IGraphTreeCreator : �O���t���쐬���܂���" << std::endl;
				std::cout << "IGraphTreeCreator : �O���t�쐬�ɂ����������� : " << _timer.GetElapsedMilliSecond() << " [ms]" << std::endl;
				std::cout << std::endl;
			}
			else
			{
				//�I�����邩���₷��
				if (askYesNo("GraphViewerSystemMain : �I�����܂����H")) { break; }
			}
		}
		else
		{
			std::cout << "GraphViewerSystemMain : �O���t�𑀍삵�܂�" << std::endl;

			//���상�j���[��\������
			std::cout << "GraphViewerSystemMain : ���상�j���[��\�����܂�" << std::endl;
			std::cout << "GraphViewerSystemMain : 1 : �m�[�h�I�����C���̃m�[�h��e�ɂ��ăO���t�𐶐�����" << std::endl;
			std::cout << "GraphViewerSystemMain : 2 : �m�[�h�I�����ĕ\������" << std::endl;
			std::cout << "GraphViewerSystemMain : 3 : �O���t��S�폜����" << std::endl;
			std::cout << "GraphViewerSystemMain : other : �I������" << std::endl;
			std::cout << std::endl;
			std::cout << "input : ";
			std::string _str;
			std::cin >> _str;
			int _menu = StrToInt(_str);
			std::cout << std::endl;

			if (_menu == 1 || _menu == 2)
			{
				std::cout << "GraphViewerSystemMain : �m�[�h��I�����Ă�������" << std::endl;
				std::cout << "GraphViewerSystemMain : 0 �` " << _graph.size() - 1 << " �̐�������͂��Ă�������" << std::endl;
				std::cout << std::endl;
				std::cout << "input : ";

				std::string _str_node;
				std::cin >> _str_node;
				int _node_num = StrToInt(_str_node);
				std::cout << std::endl;

				if (_node_num < 0 || _node_num >= _graph.size())
				{
					std::cout << "GraphViewerSystemMain : �����ȃm�[�h�ԍ��ł�" << std::endl;
					std::cout << std::endl;
					continue;
				}
				else
				{
					if (_menu == 1)
					{
						std::cout << "--------------------------------------------------" << std::endl;
						std::cout << "GraphViewerSystemMain : �m�[�h��I�����C���̃m�[�h��e�ɂ��ăO���t�𐶐����܂�" << std::endl;
						std::cout << std::endl;
						std::cout << _graph[_node_num].ToString();
						std::cout << std::endl;
						std::cout << "IGraphTreeCreator : �O���t���쐬���܂�" << std::endl;

						Stopwatch _timer;
						_timer.Start();
						CreateGraph(_graph[_node_num], _graph);
						_timer.End();
						SetGraphToBroker(_graph);
						std::cout << "IGraphTreeCreator : �O���t���쐬���܂���" << std::endl;
						std::cout << "IGraphTreeCreator : �O���t�쐬�ɂ����������� : " << _timer.GetElapsedMilliSecond() << " [ms]" << std::endl;
						std::cout << std::endl;
					}
					else
					{
						std::cout << "--------------------------------------------------" << std::endl;
						std::cout << "GraphViewerSystemMain : �m�[�h��\�����܂�" << std::endl;
						std::cout << std::endl;
						std::cout << _graph[_node_num].ToString();
						std::cout << std::endl;
					}
				}
			}
			else if (_menu == 3)
			{
				broker_ptr_->graph.Clean();
				_graph.clear();
				std::cout << "GraphViewerSystemMain : �O���t��S�폜���܂���" << std::endl;
				std::cout << std::endl;
			}
			else
			{
				//�I�����邩���₷��
				dlio::Output("�I�����܂��D", OutputDetail::kSystem);

				if (dlio::InputYesNo()) { break; }
			}
		}
	}
}

void GraphViewerSystemMain::CreateGraph(const RobotStateNode parent, std::vector<RobotStateNode>& graph)
{
	RobotStateNode parent_node = parent;
	parent_node.ChangeParentNode();

	STarget target;
	target.TargetMode = ETargetMode::StraightPosition;
	target.TargetPosition = { 100000,0,0 };
	target.RotationCenter = { 0,100000,0 };

	RobotStateNode fake_result_node;

	GraphSearchResult result =
		pass_finder_ptr_->GetNextNodebyGraphSearch(parent_node, map_state_, target, &fake_result_node);

	graph.clear();

	pass_finder_ptr_->GetGraphTree(&graph);

	if(result != GraphSearchResult::kSuccess)std::cout << fake_result_node.ToString();
}

void GraphViewerSystemMain::SetGraphToBroker(const std::vector<RobotStateNode>& _graph)
{
	broker_ptr_->graph.Clean();

	for (auto& i : _graph)
	{
		broker_ptr_->graph.PushBack(i);
	}
}

bool GraphViewerSystemMain::askYesNo(const std::string& question) const
{
	std::cout << question << " ( y / n )" << std::endl;
	std::cout << std::endl;
	std::cout << "input : ";
	std::string _input;
	std::cin >> _input;
	std::cout << std::endl;

	if (_input == "y" || _input == "Y" || _input == "yes" || _input == "Yes") { return true; }
	return false;
}

void GraphViewerSystemMain::showGraphStatus(const std::vector<RobotStateNode>& _graph) const
{
	std::cout << "GraphViewerSystemMain : �O���t�̏�Ԃ�\�����܂��D" << std::endl;
	std::cout << "GraphViewerSystemMain : �O���t�̃m�[�h�� : " << _graph.size() << std::endl;

	if (_graph.size() > 0)
	{
		std::vector<int> _depth_num((size_t)GraphSearchConst::kMaxDepth + 1);

		std::cout << "GraphViewerSystemMain : �O���t�T���̍ő�[�� : " << (int)GraphSearchConst::kMaxDepth << std::endl;

		for (const auto& i : _graph)
		{
			_depth_num.at(static_cast<size_t>(i.depth))++;
		}

		int _cnt = 0;

		for (const auto& i : _depth_num)
		{
			std::cout << "GraphViewerSystemMain : �[��" << _cnt << " : " << i << std::endl;
			_cnt++;
		}
	}

	std::cout << std::endl;
}
