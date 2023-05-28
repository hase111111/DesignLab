#include "ComUpDownNodeCreator.h"
#include "HexapodConst.h"
#include "HexapodStateCalculator.h"
#include <cfloat>
#include <algorithm>
#include "MyMath.h"
#include "NodeEdit.h"
#include "LegState.h"

void ComUpDownNodeCreator::init(const MapState* const _p_Map)
{
	mp_Map = _p_Map;
}

void ComUpDownNodeCreator::create(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph)
{
	//�d�S���ł����������邱�Ƃ̂ł���ʒu�ƁC�ł��Ⴍ�����邱�Ƃ̂ł���ʒu�����߂�D�O���[�o�����W�� Z�̈ʒu�D


	//�}�b�v���m�F���Ēn�ʂ̍ō��_�����߁C��������MAX_RANGE�CMIN_RANGE�̕����������D
	//���̂����݂��� x�̍ő�ŏ��Cy�̍ő�ŏ������߁C���̊Ԃɑ��݂���}�b�v�̍ő�z���W�����߂�D
	float _x_min = m_HexaCalc.getGlobalLegPos(_current_node, 0).x;
	float _y_min = m_HexaCalc.getGlobalLegPos(_current_node, 0).y;
	float _x_max = m_HexaCalc.getGlobalLegPos(_current_node, 0).x;
	float _y_max = m_HexaCalc.getGlobalLegPos(_current_node, 0).y;

	for (int i = 1; i < HexapodConst::LEG_NUM; i++)
	{
		const float _x_pos = m_HexaCalc.getGlobalCoxaJointPos(_current_node, i).x;
		const float _y_pos = m_HexaCalc.getGlobalCoxaJointPos(_current_node, i).y;

		_x_max = std::max(_x_max, _x_pos);
		_x_min = std::min(_x_min, _x_pos);

		_y_max = std::max(_y_max, _y_pos);
		_y_min = std::min(_y_min, _y_pos);
	}

	//�������ꂽ�}�b�v�̂ǂ̃}�X�ɑ��݂��邩�𒲂ׂ�D
	int _devide_x_min = mp_Map->getDevideMapNumX(_x_min);
	int _devide_x_max = mp_Map->getDevideMapNumX(_x_max);
	int _devide_y_min = mp_Map->getDevideMapNumY(_y_min);
	int _devide_y_max = mp_Map->getDevideMapNumY(_y_max);

	//�}�b�v�̍ő�z���W�����߂�D
	float _map_highest_z = 0;
	bool _is_init_map_highest = false;

	for (int x = _devide_x_min; x < _devide_x_max; x++)
	{
		for (int y = _devide_y_min; y < _devide_y_max; y++)
		{
			if (_is_init_map_highest == true) 
			{
				_map_highest_z = std::max(_map_highest_z, mp_Map->getTopZFromDevideMap(x, y));
			}
			else 
			{
				_map_highest_z = mp_Map->getTopZFromDevideMap(x, y);
				_is_init_map_highest = true;
			}
		}
	}


	//���{�b�g�̏d�S�̍ł��Ⴍ�����邱�Ƃ̂ł���z���W�ƁC���������邱�Ƃ��ł���z���W�����߂�D�ǂ�����O���[�o�����W�D
	float _highest_body_zpos = _map_highest_z + HexapodConst::VERTICAL_MAX_RANGE;
	float _lowest_body_zpos  = _map_highest_z + HexapodConst::VERTICAL_MIN_RANGE;


	//// �ł������n�_���C������D
	//for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	//{
	//	//�ڒn���Ă���r�ɂ��Ă̂ݍl����D
	//	if (LegState::isGrounded(_current_node.leg_state, i) == true)
	//	{
	//		//�O�����̒藝���g���āC�r�ڒn�n�_����d�S�ʒu���ǂꂾ���グ���邩�l����D
	//		const float _c = HexapodConst::FEMUR_LENGTH + HexapodConst::TIBIA_LENGTH - MARGIN;
	//		const float _b = my_math::squared(_current_node.Leg[i].x) + my_math::squared(_current_node.Leg[i].y) - HexapodConst::COXA_LENGTH;
	//		const float _a = sqrt(_c * _c - _b * _b);

	//		//�ڒn�r�̍ő�d�S�����̒������ԏ��������̂�S�̂̍ő�d�S�ʒu�Ƃ��ċL�^����D_a�͋r����ǂꂾ���グ���邩��\���Ă���̂ŁC�O���[�o�����W�ɕύX����D
	//		_highest_body_zpos = std::min(_a + _current_node.global_center_of_mass.z + _current_node.Leg[i].z, _highest_body_zpos);
	//	}
	//}


	//�m�[�h��ǉ�����D
	pushNodeByMaxAndMinPosZ(_current_node, _current_num, _highest_body_zpos, _lowest_body_zpos, _output_graph);
}

void ComUpDownNodeCreator::pushNodeByMaxAndMinPosZ(const SNode& _current_node, const int _current_num, const float _high, const float _low, std::vector<SNode>& _output_graph)
{
	//�܂��͏d�S�̕ω�����؂Ȃ����̂�ǉ�����D
	{
		SNode _same_node = _current_node;
		node_edit::changeNextNode(_same_node, _current_num, m_next_move);
		_output_graph.push_back(_same_node);
	}


	//�d�S��ω����������̂�ǉ�����D
	{
		//�ő�ƍŏ��̊Ԃ𕪊�����D
		const float _div_z = (_high - _low) / (float)DISCRETIZATION;

		//�����������V�����m�[�h��ǉ�����D
		for (int i = 0; i < DISCRETIZATION + 1; i++)
		{
			SNode _new_node = _current_node;

			const float _dif = (_low + _div_z * i) - _current_node.global_center_of_mass.z;		//���݂̏d�S��z���W�ƖڕW��Z���W�̍���(difference)�����߂�D

			_new_node.global_center_of_mass.z += _dif;		//�d�S�ʒu���X�V����D

			//�r�ʒu���X�V����D�{���r�͈ړ����Ȃ��̂ōX�V����K�v�͂Ȃ��̂����C�r�ʒu�͋r�̕t��������ʒu�ɂ��Ă���̂ł������������Ȃ��ƌ��̈ʒu�ɂȂ�Ȃ��D
			for (int l = 0; l < HexapodConst::LEG_NUM; l++)
			{
				//�V�r�͓��̂ƈꏏ�Ɉړ����邩��C�ύX���Ȃ��đ��v�D
				if (LegState::isGrounded(_new_node.leg_state, l) == true)
				{
					_new_node.Leg[l].z -= _dif;
				}

				_new_node.Leg2[l].z -= _dif;
			}

			//�m�[�h��ǉ�����D
			node_edit::changeNextNode(_new_node, _current_num, m_next_move);
			_output_graph.push_back(_new_node);
		}
	}

}
