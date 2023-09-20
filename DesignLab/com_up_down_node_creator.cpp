#include "com_up_down_node_creator.h"

#include <algorithm>
#include <cfloat>

#include "designlab_math.h"
#include "graph_search_const.h"
#include "hexapod_const.h"
#include "leg_state.h"


ComUpDownNodeCreator::ComUpDownNodeCreator(const DevideMapState& map, const std::shared_ptr<const AbstractHexapodStateCalculator>& calc, const EHexapodMove next_move) :
	map_(map),
	calclator_(calc),
	next_move_(next_move)
{
}


void ComUpDownNodeCreator::Create(const SNode& current_node, const int current_num, std::vector<SNode>* output_graph)
{
	//�d�S���ł����������邱�Ƃ̂ł���ʒu�ƁC�ł��Ⴍ�����邱�Ƃ̂ł���ʒu�����߂�D�O���[�o�����W�� Z�̈ʒu�D
	//�}�b�v���m�F���Ēn�ʂ̍ō��_�����߁C��������MAX_RANGE�CMIN_RANGE�̕����������D


	//�}�b�v�̍ő�z���W�����߂�D
	float map_highest_z = -100000;

	if (map_.IsInMap(current_node.global_center_of_mass)) 
	{
		const int kMapX = map_.GetDevideMapIndexX(current_node.global_center_of_mass.x);
		const int kMapY = map_.GetDevideMapIndexY(current_node.global_center_of_mass.y);
		map_highest_z = map_.GetTopZ(kMapX, kMapY);
	}

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		const designlab::Vector3 kCoxaVec = calclator_->getGlobalLegBasePosition(i, current_node.global_center_of_mass, current_node.rot, false);

		if (map_.IsInMap(kCoxaVec)) 
		{
			const int kCoxaX = map_.GetDevideMapIndexX(kCoxaVec.x);
			const int kCoxaY = map_.GetDevideMapIndexY(kCoxaVec.y);
			map_highest_z = (std::max)(map_.GetTopZ(kCoxaX, kCoxaY), map_highest_z);
		}
	}


	//���{�b�g�̏d�S�̍ł��Ⴍ�����邱�Ƃ̂ł���z���W�ƁC���������邱�Ƃ��ł���z���W�����߂�D�ǂ�����O���[�o�����W�D
	float highest_body_zpos = map_highest_z + HexapodConst::VERTICAL_MAX_RANGE;
	float lowest_body_zpos = map_highest_z + HexapodConst::VERTICAL_MIN_RANGE;


	// �ł������n�_���C������D

	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		//�ڒn���Ă���r�ɂ��Ă̂ݍl����D
		if (dl_leg::isGrounded(current_node.leg_state, i))
		{
			//�O�����̒藝���g���āC�r�ڒn�n�_����d�S�ʒu���ǂꂾ���グ���邩�l����D
			const float edge_c = HexapodConst::PHANTOMX_FEMUR_LENGTH + HexapodConst::PHANTOMX_TIBIA_LENGTH - MARGIN;
			const float edge_b = current_node.leg_pos[i].ProjectedXY().Length() - HexapodConst::PHANTOMX_COXA_LENGTH;

			const float edge_a = sqrt(dl_math::squared(edge_c) - dl_math::squared(edge_b));

			//�ڒn�r�̍ő�d�S�����̒������ԏ��������̂�S�̂̍ő�d�S�ʒu�Ƃ��ċL�^����D_a�͋r�̐ڒn�_����ǂꂾ���グ���邩��\���Ă���̂ŁC�O���[�o�����W�ɕύX����D
			highest_body_zpos = (std::min)(edge_a + current_node.global_center_of_mass.z + current_node.leg_pos[i].z, highest_body_zpos);
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
		const float kDivZ = (high - low) / (float)DISCRETIZATION;


		//�����������V�����m�[�h��ǉ�����D
		for (int i = 0; i < DISCRETIZATION + 1; i++)
		{
			bool is_vaild = true;

			SNode new_node = current_node;

			//�d�S�̈ʒu��ύX����D
			designlab::Vector3 new_com = current_node.global_center_of_mass;
			new_com.z = low + kDivZ * i;

			new_node.changeGlobalCenterOfMass(new_com, true);


			for (int j = 0; j < HexapodConst::LEG_NUM; j++)
			{
				if (!calclator_->isLegInRange(j, new_node.leg_pos[j])) { is_vaild = false; }
			}

			//current_num��e�Ƃ���C�V�����m�[�h�ɕύX����
			new_node.changeNextNode(current_num, next_move_);

			//�m�[�h��ǉ�����D
			if (is_vaild)
			{
				(*output_graph).emplace_back(new_node);
			}
		}

	}


	//�d�S�̕ω�����؂Ȃ����̂�ǉ�����D
	{
		SNode same_node = current_node;

		same_node.changeNextNode(current_num, next_move_);

		(*output_graph).emplace_back(same_node);
	}
}
