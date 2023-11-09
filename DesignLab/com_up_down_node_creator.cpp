#include "com_up_down_node_creator.h"

#include <algorithm>
#include <cfloat>

#include "designlab_math_util.h"
#include "graph_search_const.h"
#include "hexapod_const.h"
#include "leg_state.h"
#include "phantomx_const.h"


namespace dllf = designlab::leg_func;
namespace dlm = ::designlab::math_util;


ComUpDownNodeCreator::ComUpDownNodeCreator(const DevideMapState& map, const std::shared_ptr<const AbstractHexapodStateCalculator>& calc, const HexapodMove next_move) :
	map_(map),
	calclator_(calc),
	next_move_(next_move)
{
}


void ComUpDownNodeCreator::Create(const RobotStateNode& current_node, const int current_num, std::vector<RobotStateNode>* output_graph) const
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

	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		//�r�̐�[�̍��W�����߂�D
		const designlab::Vector3 kCoxaVec = calclator_->ConvertRobotToGlobalCoordinate(
			calclator_->GetLegBasePosRobotCoodinate(i), current_node.global_center_of_mass, current_node.rot, false
		);

		if (map_.IsInMap(kCoxaVec)) 
		{
			const int kCoxaX = map_.GetDevideMapIndexX(kCoxaVec.x);
			const int kCoxaY = map_.GetDevideMapIndexY(kCoxaVec.y);
			map_highest_z = (std::max)(map_.GetTopZ(kCoxaX, kCoxaY), map_highest_z);
		}
	}


	//���{�b�g�̏d�S�̍ł��Ⴍ�����邱�Ƃ̂ł���z���W�ƁC���������邱�Ƃ��ł���z���W�����߂�D�ǂ�����O���[�o�����W�D
	float highest_body_zpos = map_highest_z + PhantomXConst::kBodyLiftingHeightMax;
	float lowest_body_zpos = map_highest_z + PhantomXConst::kBodyLiftingHeightMin;


	// �ł������n�_���C������D

	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		//�ڒn���Ă���r�ɂ��Ă̂ݍl����D
		if (dllf::IsGrounded(current_node.leg_state, i))
		{
			//�O�����̒藝���g���āC�r�ڒn�n�_����d�S�ʒu���ǂꂾ���グ���邩�l����D
			const float edge_c = PhantomXConst::kFemurLength + PhantomXConst::kTibiaLength - MARGIN;
			const float edge_b = current_node.leg_pos[i].ProjectedXY().GetLength() - PhantomXConst::kCoxaLength;

			const float edge_a = sqrt(dlm::Squared(edge_c) - dlm::Squared(edge_b));

			//�ڒn�r�̍ő�d�S�����̒������ԏ��������̂�S�̂̍ő�d�S�ʒu�Ƃ��ċL�^����D_a�͋r�̐ڒn�_����ǂꂾ���グ���邩��\���Ă���̂ŁC�O���[�o�����W�ɕύX����D
			highest_body_zpos = (std::min)(edge_a + current_node.global_center_of_mass.z + current_node.leg_pos[i].z, highest_body_zpos);
		}
	}


	//�m�[�h��ǉ�����D
	pushNodeByMaxAndMinPosZ(current_node, current_num, highest_body_zpos, lowest_body_zpos, output_graph);
}


void ComUpDownNodeCreator::pushNodeByMaxAndMinPosZ(const RobotStateNode& current_node, const int current_num, const float high, const float low, std::vector<RobotStateNode>* output_graph) const
{
	//�d�S��ω����������̂�ǉ�����D�ω��ʂ���ԏ��Ȃ��m�[�h�͍폜����D
	{
		//�ő�ƍŏ��̊Ԃ𕪊�����D
		const float kDivZ = (high - low) / (float)DISCRETIZATION;


		//�����������V�����m�[�h��ǉ�����D
		for (int i = 0; i < DISCRETIZATION + 1; i++)
		{
			bool is_vaild = true;

			RobotStateNode new_node = current_node;

			//�d�S�̈ʒu��ύX����D
			designlab::Vector3 new_com = current_node.global_center_of_mass;
			new_com.z = low + kDivZ * i;

			new_node.ChangeGlobalCenterOfMass(new_com, true);


			for (int j = 0; j < HexapodConst::kLegNum; j++)
			{
				if (!calclator_->IsLegInRange(j, new_node.leg_pos[j])) { is_vaild = false; }
			}

			//current_num��e�Ƃ���C�V�����m�[�h�ɕύX����
			new_node.ChangeToNextNode(current_num, next_move_);

			//�m�[�h��ǉ�����D
			if (is_vaild)
			{
				(*output_graph).emplace_back(new_node);
			}
		}

	}


	//�d�S�̕ω�����؂Ȃ����̂�ǉ�����D
	{
		RobotStateNode same_node = current_node;

		same_node.ChangeToNextNode(current_num, next_move_);

		(*output_graph).emplace_back(same_node);
	}
}
