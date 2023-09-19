#include "graphic_data_broker.h"


int GraphicDataBroker::update_count() const
{
	//読み取り用のロックをかける．このスコープ { } を抜けるまでロックがかかる．(つまりこの関数が終わるまで)
	boost::shared_lock<boost::shared_mutex> read_lock(mtx_);

	return update_count_;
}


void GraphicDataBroker::set_map_state(const MapState_Old& map)
{
	//書き込み用のロックをかける．まずは，upgrade_lockを用意して，それをunique_lockに変更する．
	boost::upgrade_lock<boost::shared_mutex> upgrade_lock(mtx_);

	{
		boost::upgrade_to_unique_lock<boost::shared_mutex> write_lock(upgrade_lock);

		//値をセットする．
		map_state_ = map;

		//更新回数を増やす．
		++update_count_;
	}
}


const MapState_Old& GraphicDataBroker::map_state() const
{
	//読み取り用のロックをかける．このスコープ { } を抜けるまでロックがかかる．(つまりこの関数が終わるまで)
	boost::shared_lock<boost::shared_mutex> read_lock(mtx_);

	return map_state_;
}


void GraphicDataBroker::PushNode(const SNode& node)
{
	//書き込み用のロックをかける．
	boost::upgrade_lock<boost::shared_mutex> upgrade_lock(mtx_);

	{
		boost::upgrade_to_unique_lock<boost::shared_mutex> write_lock(upgrade_lock);

		//値をpushする．
		graph_.push_back(node);

		//更新回数を増やす．
		++update_count_;
	}
}


void GraphicDataBroker::CopyAllNode(std::vector<SNode>* node_vec) const
{
	//読み取り用のロックをかける．
	boost::shared_lock<boost::shared_mutex> read_lock(mtx_);

	(*node_vec).clear();

	for (auto& i : graph_)
	{
		(*node_vec).push_back(i);
	}
}


void GraphicDataBroker::CopyOnlyNewNode(std::vector<SNode>* node_vec) const
{
	//読み取り用のロックをかける．
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
	//読み取り用のロックをかける．
	boost::shared_lock<boost::shared_mutex> read_lock(mtx_);

	return graph_.size();
}


void GraphicDataBroker::DeleteAllNode()
{
	//書き込み用のロックをかける．
	boost::upgrade_lock<boost::shared_mutex> upgrade_lock(mtx_);

	{
		boost::upgrade_to_unique_lock<boost::shared_mutex> write_lock(upgrade_lock);

		//値をpushする．
		graph_.clear();

		//更新回数を増やす．
		++update_count_;
	}
}


void GraphicDataBroker::SetSimuEnd()
{
	//書き込み用のロックをかける．
	boost::upgrade_lock<boost::shared_mutex> upgrade_lock(mtx_);

	{
		boost::upgrade_to_unique_lock<boost::shared_mutex> write_lock(upgrade_lock);

		simu_end_index_.push_back(graph_.size() - 1);

		//更新回数を増やす．
		++update_count_;
	}
}

size_t GraphicDataBroker::GetSimuEndIndex(const int simu_num)
{
	//読み取り用のロックをかける．
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
	//読み取り用のロックをかける．
	boost::shared_lock<boost::shared_mutex> read_lock(mtx_);

	(*simu_end_index).clear();

	for (auto& i : simu_end_index_)
	{
		(*simu_end_index).push_back(i);
	}
}

