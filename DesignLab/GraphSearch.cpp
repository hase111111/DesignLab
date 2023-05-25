#include "GraphSearch.h"
#include "GraphSearchInitializer.h"

bool GraphSearch::getNextNodebyGraphSearch(const SNode& _current_node, const MapState* const _p_map, const STarget& _target, SNode& _output_node)
{
	//イニシャライザークラスに初期化をしてもらう．
	GraphSearchInitializer _initializer;

	_initializer.init(mp_GraphTreeCreator, mp_GraphSearcher);

	//早期リターン．2つのクラスの初期化に失敗したならば，即座に終了する．
	if (!mp_GraphTreeCreator) { return false; }
	if (!mp_GraphSearcher) { return false; }


	std::vector<SNode> _graph_tree;		//グラフ探索に用いるグラフ．

	//まずはグラフを作成する．_graph_tree 変数に結果を参照渡しされる．
	if (mp_GraphTreeCreator->createGraphTree(_current_node, _p_map, _graph_tree) == false) { return false; }
	
	//次にグラフを評価して，次の動作を決定する．put_node 変数に結果を参照渡しされる．
	if (mp_GraphSearcher->searchGraphTree(_graph_tree, _target, _output_node) == false) { return false; }

	return true;
}
