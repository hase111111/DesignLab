#include "ComUpDownNodeCreator.h"
#include "HexapodConst.h"
#include "HexapodStateCalculator.h"
#include <cfloat>
#include <algorithm>
#include "MyMath.h"
#include "LegState.h"

void ComUpDownNodeCreator::create(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph)
{
	//重心を最も高くあげることのできる位置と，最も低く下げることのできる位置を求める．グローバル座標で Zの位置．
	//マップを確認して地面の最高点を求め，そこからMAX_RANGE，MIN_RANGEの分だけ離す．


	//マップの最大z座標を求める．
	const int _map_x = mp_Map->getDevideMapNumX(_current_node.global_center_of_mass.x);
	const int _map_y = mp_Map->getDevideMapNumY(_current_node.global_center_of_mass.y);
	const float _map_highest_z = mp_Map->getTopZFromDevideMap(_map_x, _map_y);

	//ロボットの重心の最も低く下げることのできるz座標と，高くあげることができるz座標を求める．どちらもグローバル座標．
	float _highest_body_zpos = _map_highest_z + HexapodConst::VERTICAL_MAX_RANGE;
	float _lowest_body_zpos = _map_highest_z + HexapodConst::VERTICAL_MIN_RANGE;


	// 最も高い地点を修正する．
	using my_math::squared;

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		//接地している脚についてのみ考える．
		if (LegStateEdit::isGrounded(_current_node.leg_state, i) == true)
		{
			//三平方の定理を使って，脚接地地点から重心位置をどれだけ上げられるか考える．
			const float _c = HexapodConst::FEMUR_LENGTH + HexapodConst::TIBIA_LENGTH - MARGIN;
			const float _b = _current_node.leg_pos[i].projectedXY().length() - HexapodConst::COXA_LENGTH;

			const float _a = sqrt(squared(_c) - squared(_b));

			//接地脚の最大重心高さの中から一番小さいものを全体の最大重心位置として記録する．_aは脚の接地点からどれだけ上げられるかを表しているので，グローバル座標に変更する．
			_highest_body_zpos = (std::min)(_a + _current_node.global_center_of_mass.z + _current_node.leg_pos[i].z, _highest_body_zpos);
		}
	}


	//ノードを追加する．
	pushNodeByMaxAndMinPosZ(_current_node, _current_num, _highest_body_zpos, _lowest_body_zpos, _output_graph);
}

void ComUpDownNodeCreator::pushNodeByMaxAndMinPosZ(const SNode& _current_node, const int _current_num, const float _high, const float _low, std::vector<SNode>& _output_graph)
{
	//重心を変化させたものを追加する．変化量が一番少ないノードは削除する．
	{
		//最大と最小の間を分割する．
		const float _div_z = (_high - _low) / (float)DISCRETIZATION;

		//現在の重心との差分が一番小さいものを探す．
		float _dif_min = 100000.0f;
		int _dif_min_index = -1;

		//分割した分新しいノードを追加する．
		for (int i = 0; i < DISCRETIZATION + 1; i++)
		{
			SNode _new_node = _current_node;

			//重心の位置を変更する．
			my_vec::SVector _new_com = _current_node.global_center_of_mass;
			_new_com.z = _low + _div_z * i;

			_new_node.changeGlobalCenterOfMass(_new_com, true);

			if (_dif_min > abs(_current_node.global_center_of_mass.z - _new_node.global_center_of_mass.z))
			{
				_dif_min = abs(_current_node.global_center_of_mass.z - _new_node.global_center_of_mass.z);
				_dif_min_index = i;
			}

			//current_numを親とする，新しいノードに変更する
			_new_node.changeNextNode(_current_num, m_next_move);

			//ノードを追加する．
			_output_graph.push_back(_new_node);
		}

		//一番差分が小さくものを消す
		if (_dif_min_index >= 0) { _output_graph.erase(_output_graph.begin() + _dif_min_index); }
	}

	//重心の変化が一切ないものを追加する．
	{
		SNode _same_node = _current_node;
		_same_node.changeNextNode(_current_num, m_next_move);
		_output_graph.push_back(_same_node);
	}
}
