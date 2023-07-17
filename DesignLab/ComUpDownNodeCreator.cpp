#include "ComUpDownNodeCreator.h"
#include "HexapodConst.h"
#include "HexapodStateCalculator.h"
#include <cfloat>
#include <algorithm>
#include "MyMath.h"
#include "LegState.h"

void ComUpDownNodeCreator::init(const MapState* const _p_Map)
{
	mp_Map = _p_Map;
}

void ComUpDownNodeCreator::create(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph)
{
	//�d�S���ł����������邱�Ƃ̂ł���ʒu�ƁC�ł��Ⴍ�����邱�Ƃ̂ł���ʒu�����߂�D�O���[�o�����W�� Z�̈ʒu�D
	//�}�b�v���m�F���Ēn�ʂ̍ō��_�����߁C��������MAX_RANGE�CMIN_RANGE�̕����������D


	//�}�b�v�̍ő�z���W�����߂�D
	const int _map_x = mp_Map->getDevideMapNumX(_current_node.global_center_of_mass.x);
	const int _map_y = mp_Map->getDevideMapNumY(_current_node.global_center_of_mass.y);
	const float _map_highest_z = mp_Map->getTopZFromDevideMap(_map_x, _map_y);

	//���{�b�g�̏d�S�̍ł��Ⴍ�����邱�Ƃ̂ł���z���W�ƁC���������邱�Ƃ��ł���z���W�����߂�D�ǂ�����O���[�o�����W�D
	float _highest_body_zpos = _map_highest_z + HexapodConst::VERTICAL_MAX_RANGE;
	float _lowest_body_zpos = _map_highest_z + HexapodConst::VERTICAL_MIN_RANGE;


	// �ł������n�_���C������D
	using my_math::squared;

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		//�ڒn���Ă���r�ɂ��Ă̂ݍl����D
		if (LegStateEdit::isGrounded(_current_node.leg_state, i) == true)
		{
			//�O�����̒藝���g���āC�r�ڒn�n�_����d�S�ʒu���ǂꂾ���グ���邩�l����D
			const float _c = HexapodConst::FEMUR_LENGTH + HexapodConst::TIBIA_LENGTH - MARGIN;
			const float _b = _current_node.leg_pos[i].projectedXY().length() - HexapodConst::COXA_LENGTH;

			const float _a = sqrt(squared(_c) - squared(_b));

			//�ڒn�r�̍ő�d�S�����̒������ԏ��������̂�S�̂̍ő�d�S�ʒu�Ƃ��ċL�^����D_a�͋r����ǂꂾ���グ���邩��\���Ă���̂ŁC�O���[�o�����W�ɕύX����D
			_highest_body_zpos = std::min(_a + _current_node.global_center_of_mass.z + _current_node.leg_pos[i].z, _highest_body_zpos);
		}
	}


	//�m�[�h��ǉ�����D
	pushNodeByMaxAndMinPosZ(_current_node, _current_num, _highest_body_zpos, _lowest_body_zpos, _output_graph);
}

void ComUpDownNodeCreator::pushNodeByMaxAndMinPosZ(const SNode& _current_node, const int _current_num, const float _high, const float _low, std::vector<SNode>& _output_graph)
{
	//�܂��͏d�S�̕ω�����؂Ȃ����̂�ǉ�����D
	{
		SNode _same_node = _current_node;
		_same_node.changeNextNode(_current_num, m_next_move);
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
				if (LegStateEdit::isGrounded(_new_node.leg_state, l) == true)
				{
					_new_node.leg_pos[l].z -= _dif;
				}

				_new_node.Leg2[l].z -= _dif;
			}

			//�m�[�h��ǉ�����D
			_new_node.changeNextNode(_current_num, m_next_move);
			_output_graph.push_back(_new_node);
		}
	}

}
