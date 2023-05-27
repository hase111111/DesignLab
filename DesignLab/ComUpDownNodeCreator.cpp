#include "ComUpDownNodeCreator.h"
#include "HexapodConst.h"
#include "HexapodStateCalculator.h"
#include <cfloat>
#include <algorithm>
#include "MyMath.h"
#include "LegState.h"

void ComUpDownNodeCreator::create(const SNode& _current_node, const int _current_num, std::vector<SNode>& _output_graph)
{
	//�d�S���ł����������邱�Ƃ̂ł���ʒu�ƁC�ł��Ⴍ�����邱�Ƃ̂ł���ʒu�����߂�D�O���[�o�����W�� Z�̈ʒu�D
	float _highest_body_zpos = FLT_MAX;
	float _lowest_pos = FLT_MAX;


	//�}�b�v���m�F���Ēn�ʂ̍ō��_�����߁C��������MAX_RANGE�CMIN_RANGE�̕����������D




	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		//�ڒn���Ă���r�ɂ��Ă̂ݍl����D
		if (LegState::isGrounded(_current_node.leg_state, i) == true)
		{
			//�O�����̒藝���g���āC�r�ڒn�n�_����d�S�ʒu���ǂꂾ���グ���邩�l����D
			const float _c = HexapodConst::FEMUR_LENGTH + HexapodConst::TIBIA_LENGTH - MARGIN;
			const float _b = my_math::squared(_current_node.Leg[i].x) + my_math::squared(_current_node.Leg[i].y) - HexapodConst::COXA_LENGTH;
			const float _a = sqrt(_c * _c - _b * _b);

			//�ڒn�r�̍ő�d�S�����̒������ԏ��������̂�S�̂̍ő�d�S�ʒu�Ƃ��ċL�^����D_a�͋r����ǂꂾ���グ���邩��\���Ă���̂ŁC�O���[�o�����W�ɕύX����D
			_highest_body_zpos = std::min(_a + _current_node.global_center_of_mass.z + _current_node.Leg[i].z, _highest_body_zpos);



		}
	}
}
