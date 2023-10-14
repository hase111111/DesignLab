#include "phantomx_state_calculator_hato.h"

#include "cassert_define.h"
#include "designlab_math_util.h"
#include "phantomx_const.h"

namespace dlm = ::designlab::math_util;

std::array <float, PhantomXStateCalclator_Hato::kMaxDifZ> PhantomXStateCalclator_Hato::leg_max_r_ = PhantomXStateCalclator_Hato::initLegMaxR();


PhantomXStateCalclator_Hato::PhantomXStateCalclator_Hato() 
{
    initLegMaxR();
}

bool PhantomXStateCalclator_Hato::IsLegInRange(const int leg_index, const designlab::Vector3& leg_pos) const
{
	const designlab::Vector2 leg_pos_xy = leg_pos.ProjectedXY();
	const designlab::Vector2 min_leg_pos_xy{HexapodConst::MOVABLE_LEG_RANGE_COS_MIN[leg_index], HexapodConst::MOVABLE_LEG_RANGE_SIN_MAX[leg_index]};
	const designlab::Vector2 max_leg_pos_xy{HexapodConst::MOVABLE_LEG_RANGE_COS_MAX[leg_index], HexapodConst::MOVABLE_LEG_RANGE_SIN_MIN[leg_index]};

	//�r�̊p�x���͈͓��ɂ��邩���ׂ�D�O�όv�Z�ŊԂɂ��邩���ׂ�
	if (min_leg_pos_xy.Cross(leg_pos_xy) > 0.0f) { return false; }
	if (max_leg_pos_xy.Cross(leg_pos_xy) < 0.0f) { return false; }


	//�r��L�΂����Ƃ̂ł��Ȃ��͈͂ɐL�΂��Ă��Ȃ������ׂ�D
	
	//assert(0 <= -leg_pos.z && -leg_pos.z <= kMaxDifZ);
	if (leg_pos.z > 0 || -kMaxDifZ > leg_pos.z) { return false; }

	if (dlm::Squared(leg_max_r_[-static_cast<int>(leg_pos.z)]) < leg_pos_xy.GetSquaredLength()) { return false; }

	if (dlm::Squared(120) > leg_pos_xy.GetSquaredLength()) { return false; }

	return true;
}

std::array <float, PhantomXStateCalclator_Hato::kMaxDifZ> PhantomXStateCalclator_Hato::initLegMaxR()
{
    // �t�^���wcoxa�Ȃ��̌v�Z���ʂ�p���ď��^���w���v�Z����
	std::array <float, PhantomXStateCalclator_Hato::kMaxDifZ> leg_max_r;

	for (auto i : leg_max_r) { i = 0; }

	const float PERMISSION = 0.5f;			//�t�^���w�Ɖ^���w���s�������ʂ����aPermission^0.5�̉~�̒��Ȃ瓙�����ƍl����

	const float mins[3] = { -1.428f, -1.780f, -1.194f };	// �r���͈� �����炭rad �ϊ��������(-81.8�� -101.98�� -68.41��)  190527
	const float maxs[3] = { 1.402f,  1.744f,  1.769f };		// ������coxa,femur,tibia (80.32�� 99.92�� 101.36��)

	//ans of kinematics use sorution of i_kinematics 

	//z�͍ő�196�Dix�͍ő�248
	for (int iz = 0; iz < 200; iz++)
	{
		for (int ix = 53; ix < 248; ix++)
		{
			const designlab::Vector3 line_end((float)ix, 0.0f, (float)iz);		//�r����W�i���[�J���j

			//�t�^���wcoxa�Ȃ�

			//const float _coxa_angle = atan2(line_end.x, line_end.y);	//coxa�p�x

			const float _IK_trueX = sqrt(dlm::Squared(line_end.x) + dlm::Squared(line_end.y)) - PhantomXConst::kCoxaLength;	//femur���瑫��܂ł����ԃx�N�g����xy���ʂɓ��e�����Ƃ��̃x�N�g���̑傫��
			float _im = sqrt(dlm::Squared(_IK_trueX) + dlm::Squared(line_end.z));					//��΂ɐ�
			if (_im == 0.0f) _im += 0.01f;	//0����΍�

			const float _q1 = -atan2(line_end.z, _IK_trueX);													//�}�C�i�X�ł������W�n�I��q1���̂͏�ɕ�//x���[�����ƒ�`��G���[
			const float _q2 = acos(
				(dlm::Squared(PhantomXConst::kFemurLength) + dlm::Squared(_im) - dlm::Squared(PhantomXConst::kTibiaLength)) /
				(2.0f * PhantomXConst::kFemurLength * _im)
			);	//im=0���ƒ�`��G���[

			const float _femur_angle = _q1 + _q2;
			const float _tibia_angle = acos(
				(dlm::Squared(PhantomXConst::kFemurLength) + dlm::Squared(PhantomXConst::kTibiaLength) - dlm::Squared(_im)) /
				(2.0f * PhantomXConst::kFemurLength * PhantomXConst::kTibiaLength)
			) - dlm::kFloatPi / 2.0f;

			//float im = sqrt(pow(fabs(IK_trueX), 2.0) + pow(fabs(LineEnd.z), 2.0));//femur���瑫��̋���
			//float d1 = pow((float)L_FEMUR, 2.0) - pow((float)L_TIBIA, 2.0) + pow(fabs((float)im), 2.0);
			//float d2 = 2 * L_FEMUR*im;
			//float q2 = acos((float)d1 / (float)d2);	//�]���藝
			//d1 = pow((float)L_FEMUR, 2.0) - pow(fabs((float)im), 2.0) + pow((float)L_TIBIA, 2.0);
			//d2 = 2 * L_TIBIA*L_FEMUR;
			//tibia = acos((float)d1 / (float)d2) - 1.570796326795;

			//lange of motion
			//���@�͂킩��񂪁A�V�~�����[�V�������ƁA���ꂪ����Ȃ��B
			//if�������ƁA�d�S�Ƒ��捂���̍����A73mm�ȉ��͎��Ȃ��Bhato
			//if ( femur < femurMins)break;
			//if (femurMaxs < femur)break;
			//if (tibia < tibiaMins)break;
			//if(tibiaMaxs < tibia )break;

			//�^���w
			const float _K_trueX = PhantomXConst::kFemurLength * cos(_femur_angle) + PhantomXConst::kTibiaLength * cos(_femur_angle + _tibia_angle - dlm::kFloatPi / 2.0f);

			designlab::Vector3 _kinematics;
			_kinematics.x = _K_trueX + PhantomXConst::kCoxaLength;
			_kinematics.y = 0.0f;
			_kinematics.z = -(PhantomXConst::kFemurLength * sin(_femur_angle) + PhantomXConst::kTibiaLength * sin(_femur_angle + _tibia_angle - dlm::kFloatPi / 2.0f));

			const float _Permission = (_kinematics - line_end).GetSquaredLength();

			if (PERMISSION > _Permission)
			{
				constexpr float kLegRom_RMargin = 10.f;
				leg_max_r[iz] = static_cast<float>(ix) - kLegRom_RMargin;//y=0�̂Ƃ��C�r����z�̂Ƃ���x�����̍ő�͈̔�

#ifdef  MAX_LEG_RADIUS
				if (iz <= 115) { m_leg_max_r[iz] = MAX_LEG_RADIUS; }//�r��u���ʒu����������ƃg���N������Ȃ��Ē��ݍ��݂�����������200�܂łɂ���2020/11/09hato
#endif
			}
		}
	}

	//for (int i = 0; i < leg_max_r.size(); i++) { std::cout << i << "," << leg_max_r[i] << "," << std::endl; }

	return leg_max_r;
}
