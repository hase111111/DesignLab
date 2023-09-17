#include "graphic_data_broker.h"


void GraphicDataBroker::set_map_state(const MapState& map)
{
	//�������ݗp�̃��b�N��������D�܂��́Cupgrade_lock��p�ӂ��āC�����unique_lock�ɕύX����D
	boost::upgrade_lock<boost::shared_mutex> upgrade_lock(mtx_);

	{
		boost::upgrade_to_unique_lock<boost::shared_mutex> write_lock(upgrade_lock);

		//�l���Z�b�g����D
		map_state_ = map;
	}
}


MapState GraphicDataBroker::map_state() const
{
	//�ǂݎ��p�̃��b�N��������D���̃X�R�[�v { } �𔲂���܂Ń��b�N��������D(�܂肱�̊֐����I���܂�)
	boost::shared_lock<boost::shared_mutex> read_lock(mtx_);

	return map_state_;
}


void GraphicDataBroker::PushNode(const SNode& node)
{
	//�������ݗp�̃��b�N��������D
	boost::upgrade_lock<boost::shared_mutex> upgrade_lock(mtx_);

	{
		boost::upgrade_to_unique_lock<boost::shared_mutex> write_lock(upgrade_lock);

		//�l��push����D
		graph_.push_back(node);
	}
}


void GraphicDataBroker::CopyAllNode(std::vector<SNode>* node_vec) const
{
	//�ǂݎ��p�̃��b�N��������D
	boost::shared_lock<boost::shared_mutex> read_lock(mtx_);

	(*node_vec).clear();

	for (auto& i : graph_)
	{
		(*node_vec).push_back(i);
	}
}


void GraphicDataBroker::CopyOnlyNewNode(std::vector<SNode>* node_vec) const
{
	//�ǂݎ��p�̃��b�N��������D
	boost::shared_lock<boost::shared_mutex> read_lock(mtx_);

	if ((*node_vec).size() < graph_.size())
	{
		const size_t start_num = (*node_vec).size();

		for (size_t i = start_num; i < graph_.size(); ++i)
		{
			(*node_vec).push_back(graph_.at(i));
		}
	}
}


size_t GraphicDataBroker::GetNodeNum() const
{
	//�ǂݎ��p�̃��b�N��������D
	boost::shared_lock<boost::shared_mutex> read_lock(mtx_);

	return graph_.size();
}


void GraphicDataBroker::DeleteAllNode()
{
	//�������ݗp�̃��b�N��������D
	boost::upgrade_lock<boost::shared_mutex> upgrade_lock(mtx_);

	{
		boost::upgrade_to_unique_lock<boost::shared_mutex> write_lock(upgrade_lock);

		//�l��push����D
		graph_.clear();
	}
}


void GraphicDataBroker::SetSimuEnd()
{
	//�������ݗp�̃��b�N��������D
	boost::upgrade_lock<boost::shared_mutex> upgrade_lock(mtx_);

	{
		boost::upgrade_to_unique_lock<boost::shared_mutex> write_lock(upgrade_lock);

		simu_end_index_.push_back(graph_.size() - 1);
	}
}

size_t GraphicDataBroker::GetSimuEndIndex(const int simu_num)
{
	//�ǂݎ��p�̃��b�N��������D
	boost::shared_lock<boost::shared_mutex> read_lock(mtx_);

	if (simu_num < simu_end_index_.size())
	{
		return simu_end_index_.at(simu_num);

	}
	else if (simu_num == simu_end_index_.size())
	{
		return graph_.size() - 1;
	}

	return -1;
}


void GraphicDataBroker::CopySimuEndIndex(std::vector<size_t>* simu_end_index) const
{
	//�ǂݎ��p�̃��b�N��������D
	boost::shared_lock<boost::shared_mutex> read_lock(mtx_);

	(*simu_end_index).clear();

	for (auto& i : simu_end_index_)
	{
		(*simu_end_index).push_back(i);
	}
}

