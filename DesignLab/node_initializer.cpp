#include "node_initializer.h"

#include "leg_state.h"
#include "phantomx_const.h"

namespace dllf = designlab::leg_func;


RobotStateNode NodeInitializer::InitNode() const
{
	using designlab::Vector3;
	using designlab::EulerXYZ;

	RobotStateNode res;

	//�r���
	res.leg_state = dllf::MakeLegStateBit(
		DiscreteComPos::kCenterBack,
		{ true, true, true, true, true, true },
		{ DiscreteLegPos::kCenter, DiscreteLegPos::kCenter, DiscreteLegPos::kCenter,
		DiscreteLegPos::kCenter, DiscreteLegPos::kCenter, DiscreteLegPos::kCenter }
	);

	//�r�t���������_�Ƃ����C�r��̈ʒu������������D
	const float z_base = 0.0f;
	const float kComZ = PhantomXConst::kBodyLiftingHeightMin + z_base;	// ���{�b�g�̏d�S��Z���W

	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		res.leg_pos[i] = res.leg_reference_pos[i] = {
			170 * cos(PhantomXConst::kCoxaDefaultAngle[i]),
			170 * sin(PhantomXConst::kCoxaDefaultAngle[i]),
			-kComZ
		};
	}

	//�O���[�o�����W�̏d�S�ʒu�D�O���[�o�����W(0,0,0)�𒆐S�Ƃ����C���̕ϐ� _x�C_y�𔼌a�Ƃ���ȉ~�`�̂Ȃ��ɏd�S���ړ�����D
	const float kAngle = 0; // do_random ? dlm::GenerateRandomNumber(0.0f, 2.0f * dlm::kFloatPi) : 0;
	const float kEx = 0; // do_random ? dlm::GenerateRandomNumber(0.0f, 1.0f) : 0;


	const float kX = 3000.f * 0.25f;
	const float kY = 2000.f / 2.0f * 0.8f;

	res.global_center_of_mass = designlab::Vector3(kEx * kX * cos(kAngle), kEx * kY * sin(kAngle), kComZ);

	//���[���s�b�`���[�ŉ�]��\������D���{�b�g�̏d�S�𒆐S�ɂ��ĉ�]����D 
	res.rot = designlab::EulerXYZ(0, 0, 0);

	res.next_move = HexapodMove::kComUpDown;
	res.parent_num = -1;
	res.depth = 0;

	return res;
}
