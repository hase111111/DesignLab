#include "ComMoveNodeCreator.h"
#include "ComCandidatePolygonMaker.h"
#include "ComSelecter.h"
#include "LegState.h"

void ComMoveNodeCreator::create(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph)
{
	std::vector<std::pair<my_vec::SPolygon2, ComType::EComPattern>> _candidate_polygons;

	//重心移動先の候補地点の範囲を示す多角形を作成する
	ComCandidatePolygonMaker _maker;
	_maker.makeCandidatePolygon(_current_node, _candidate_polygons);

	//候補範囲から実際に移動する先の座標を選択する
	ComSelecter _selecter;
	_selecter.setCurrentNode(_current_node);

	for (const auto& i : _candidate_polygons)
	{
		my_vec::SVector _res;

		if (_selecter.getComFromPolygon(i.first, i.second, _res) == true)
		{
			SNode _next_node = _current_node;

			_next_node.changeGlobalCenterOfMass(_res, false);					//重心位置を変更し，それに伴い接地脚の位置も変更する

			LegStateEdit::changeComPattern(_next_node.leg_state, i.second);		//leg_stateのcom_patternを変更する

			for (int i = 0; i < HexapodConst::LEG_NUM; i++) { LegStateEdit::changeLegStateKeepTopBit(_next_node.leg_state, i, 4); }

			_next_node.changeNextNode(_current_num, m_next_move);	//深さや親ノードを変更する

			_output_graph.push_back(_next_node);
		}
	}

	if (DO_DEBUG_PRINT)std::cout << "ComMoveNodeCreator::create() : " << _output_graph.size() << std::endl;
}