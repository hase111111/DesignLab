#include "ComUpDownNodeCreator.h"

#include <cfloat>
#include <algorithm>

#include "GraphSearchConst.h"
#include "HexapodConst.h"
#include "HexapodStateCalculator.h"
#include "LegState.h"
#include "my_math.h"


ComUpDownNodeCreator::ComUpDownNodeCreator(const MapState* const p_map, const EHexapodMove next_move) : INodeCreator(p_map, next_move), mp_map(p_map)
{
	if (GraphSearchConst::DO_DEBUG_PRINT)
	{
		std::cout << "[NodeCreator] ComUpDownNodeCreator : �R���X�g���N�^���Ă΂ꂽ�D\n";
	}
}

ComUpDownNodeCreator::~ComUpDownNodeCreator()
{
	if (GraphSearchConst::DO_DEBUG_PRINT)
	{
		std::cout << "[NodeCreator] ComUpDownNodeCreator : �f�X�g���N�^���Ă΂ꂽ�D\n";
	}
}

void ComUpDownNodeCreator::create(const SNode& current_node, const int current_num, std::vector<SNode>* output_graph)
{
	//�d�S���ł����������邱�Ƃ̂ł���ʒu�ƁC�ł��Ⴍ�����邱�Ƃ̂ł���ʒu�����߂�D�O���[�o�����W�� Z�̈ʒu�D
	//�}�b�v���m�F���Ēn�ʂ̍ō��_�����߁C��������MAX_RANGE�CMIN_RANGE�̕����������D


	//�}�b�v�̍ő�z���W�����߂�D
	const int map_x = mp_map->getDevideMapNumX(current_node.global_center_of_mass.x);
	const int map_y = mp_map->getDevideMapNumY(current_node.global_center_of_mass.y);
	const float map_highest_z = mp_map->getTopZFromDevideMap(map_x, map_y);

	//���{�b�g�̏d�S�̍ł��Ⴍ�����邱�Ƃ̂ł���z���W�ƁC���������邱�Ƃ��ł���z���W�����߂�D�ǂ�����O���[�o�����W�D
	float highest_body_zpos = map_highest_z + HexapodConst::VERTICAL_MAX_RANGE;
	float lowest_body_zpos = map_highest_z + HexapodConst::VERTICAL_MIN_RANGE;


	// �ł������n�_���C������D

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		//�ڒn���Ă���r�ɂ��Ă̂ݍl����D
		if (LegStateEdit::isGrounded(current_node.leg_state, i) == true)
		{
			//�O�����̒藝���g���āC�r�ڒn�n�_����d�S�ʒu���ǂꂾ���グ���邩�l����D
			const float _c = HexapodConst::FEMUR_LENGTH + HexapodConst::TIBIA_LENGTH - MARGIN;
			const float _b = current_node.leg_pos[i].projectedXY().length() - HexapodConst::COXA_LENGTH;

			const float _a = sqrt(my_math::squared(_c) - my_math::squared(_b));

			//�ڒn�r�̍ő�d�S�����̒������ԏ��������̂�S�̂̍ő�d�S�ʒu�Ƃ��ċL�^����D_a�͋r�̐ڒn�_����ǂꂾ���グ���邩��\���Ă���̂ŁC�O���[�o�����W�ɕύX����D
			highest_body_zpos = (std::min)(_a + current_node.global_center_of_mass.z + current_node.leg_pos[i].z, highest_body_zpos);
		}
	}


	//�m�[�h��ǉ�����D
	pushNodeByMaxAndMinPosZ(current_node, current_num, highest_body_zpos, lowest_body_zpos, output_graph);
}

void ComUpDownNodeCreator::pushNodeByMaxAndMinPosZ(const SNode& current_node, const int current_num, const float high, const float low, std::vector<SNode>* output_graph)
{
	//�d�S��ω����������̂�ǉ�����D�ω��ʂ���ԏ��Ȃ��m�[�h�͍폜����D
	{
		//�ő�ƍŏ��̊Ԃ𕪊�����D
		const float div_z = (high - low) / (float)DISCRETIZATION;

		//���݂̏d�S�Ƃ̍�������ԏ��������̂�T���D
		float dif_min = 100000.0f;
		int dif_min_index = -1;

		//�����������V�����m�[�h��ǉ�����D
		for (int i = 0; i < DISCRETIZATION + 1; i++)
		{
			SNode new_node = current_node;

			//�d�S�̈ʒu��ύX����D
			my_vec::SVector new_com = current_node.global_center_of_mass;
			new_com.z = low + div_z * i;

			new_node.changeGlobalCenterOfMass(new_com, true);

			if (dif_min > abs(current_node.global_center_of_mass.z - new_node.global_center_of_mass.z))
			{
				dif_min = abs(current_node.global_center_of_mass.z - new_node.global_center_of_mass.z);
				dif_min_index = i;
			}

			//current_num��e�Ƃ���C�V�����m�[�h�ɕύX����
			new_node.changeNextNode(current_num, m_next_move);

			//�m�[�h��ǉ�����D
			(*output_graph).emplace_back(new_node);
		}

		//��ԍ��������������̂�����
		if (dif_min_index >= 0) { (*output_graph).erase((*output_graph).begin() + dif_min_index); }
	}

	//�d�S�̕ω�����؂Ȃ����̂�ǉ�����D
	{
		SNode same_node = current_node;
		same_node.changeNextNode(current_num, m_next_move);
		(*output_graph).emplace_back(same_node);
	}
}
