#include "phantomx_renderer_simple.h"

#include <array>

#include "dxlib_util.h"
#include "designlab_math_util.h"
#include "leg_state.h"


namespace dldu = designlab::dxlib_util;
namespace dllf = designlab::leg_func;
namespace dlm = designlab::math_util;


PhantomXRendererSimple::PhantomXRendererSimple(const std::shared_ptr<const AbstractHexapodStateCalculator>& calc, DisplayQuality display_quality) :
	kColorBody(GetColor(23, 58, 235)),
	kColorLeg(GetColor(23, 58, 235)),
	kColorLiftedLeg(GetColor(240, 30, 60)),
	kColorJoint(GetColor(100, 100, 200)),
	kColorLiftedJoint(GetColor(200, 100, 100)),
	kColorLegBase(GetColor(100, 200, 100)),
	kColorKineLeg(GetColor(45, 45, 100)),
	kColorKineJoint(GetColor(60, 60, 115)),
	kColorErrorText(GetColor(32, 32, 32)),
	kColorErrorJoint(GetColor(180, 180, 64)),
	kCapsuleDivNum(6),
	kSphereDivNum(16),
	kLegRadius(10.0f),
	kJointRadius(20.0f),
	calculator_ptr_(calc),
	display_quality_(display_quality)
{
	SetDrawNode(RobotStateNode{});
}


void PhantomXRendererSimple::SetDrawNode(const RobotStateNode& node)
{
	draw_node_ = node;

	if (!calculator_ptr_) { return; }	//�v�Z�킪�Ȃ��Ȃ�Ή������Ȃ�

	calculator_ptr_->CalculateAllJointState(node, &draw_joint_state_);

	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		// ��������Ȃ��Ȃ�Ή������Ȃ�
		if (draw_joint_state_[i].global_joint_position.size() != 4) { continue; }
		if (draw_joint_state_[i].local_joint_position.size() != 4) { continue; }
		if (draw_joint_state_[i].joint_angle.size() != 3) { continue; }

		draw_data_[i].leg_reference_pos = dldu::ConvertToDxlibVec(calculator_ptr_->GetGlobalLegPosition(i, draw_node_.leg_reference_pos[i], draw_node_.global_center_of_mass, draw_node_.rot, true));

		// global���W�̊Ԑڂ̈ʒu��Dxlib��VECTOR�ɕϊ�����D
		draw_data_[i].coxa_joint_pos = dldu::ConvertToDxlibVec(draw_joint_state_[i].global_joint_position[0]);
		draw_data_[i].femur_joint_pos = dldu::ConvertToDxlibVec(draw_joint_state_[i].global_joint_position[1]);
		draw_data_[i].tibia_joint_pos = dldu::ConvertToDxlibVec(draw_joint_state_[i].global_joint_position[2]);
		draw_data_[i].leg_end_pos = dldu::ConvertToDxlibVec(draw_joint_state_[i].global_joint_position[3]);

		// �v�Z�̍������̂��߂ɁCsin, cos���v�Z���Ă����D
		draw_data_[i].coxa_cos = std::cos(draw_joint_state_[i].joint_angle[0]);
		draw_data_[i].coxa_sin = std::sin(draw_joint_state_[i].joint_angle[0]);
		draw_data_[i].femur_cos = std::cos(draw_joint_state_[i].joint_angle[1]);
		draw_data_[i].femur_sin = std::sin(draw_joint_state_[i].joint_angle[1]);
		draw_data_[i].tibia_cos = std::cos(draw_joint_state_[i].joint_angle[1] + draw_joint_state_[i].joint_angle[2]);
		draw_data_[i].tibia_sin = std::sin(draw_joint_state_[i].joint_angle[1] + draw_joint_state_[i].joint_angle[2]);

		//�������l�ɂȂ��Ă��邩�`�F�b�N���邽�߂ɁC���^���w�Ōv�Z���Ȃ���
		draw_data_[i].kine_coxa_joint_vec = draw_joint_state_[i].local_joint_position[0];
		draw_data_[i].kine_femur_joint_vec = draw_data_[i].kine_coxa_joint_vec + HexapodConst::PHANTOMX_COXA_LENGTH * designlab::Vector3{ draw_data_[i].coxa_cos, draw_data_[i].coxa_sin, 0};
		draw_data_[i].kine_tibia_joint_vec = draw_data_[i].kine_femur_joint_vec + HexapodConst::PHANTOMX_FEMUR_LENGTH * designlab::Vector3{ draw_data_[i].coxa_cos * draw_data_[i].femur_cos, draw_data_[i].coxa_sin * draw_data_[i].femur_cos, draw_data_[i].femur_sin };
		draw_data_[i].kine_leg_end_vec = draw_data_[i].kine_tibia_joint_vec + HexapodConst::PHANTOMX_TIBIA_LENGTH * designlab::Vector3{ draw_data_[i].coxa_cos * draw_data_[i].tibia_cos, draw_data_[i].coxa_sin * draw_data_[i].tibia_cos, draw_data_[i].tibia_sin};

		draw_data_[i].kine_coxa_joint_pos = dldu::ConvertToDxlibVec(calculator_ptr_->GetGlobalLegPosition(i, draw_data_[i].kine_coxa_joint_vec, draw_node_.global_center_of_mass, draw_node_.rot, true));
		draw_data_[i].kine_femur_joint_pos = dldu::ConvertToDxlibVec(calculator_ptr_->GetGlobalLegPosition(i, draw_data_[i].kine_femur_joint_vec, draw_node_.global_center_of_mass, draw_node_.rot, true));
		draw_data_[i].kine_tibia_joint_pos = dldu::ConvertToDxlibVec(calculator_ptr_->GetGlobalLegPosition(i, draw_data_[i].kine_tibia_joint_vec, draw_node_.global_center_of_mass, draw_node_.rot, true));
		draw_data_[i].kine_leg_end_pos = dldu::ConvertToDxlibVec(calculator_ptr_->GetGlobalLegPosition(i, draw_data_[i].kine_leg_end_vec, draw_node_.global_center_of_mass, draw_node_.rot, true));

		// �t�^���w�Ōv�Z�����r�̒������v�Z����D
		draw_data_[i].coxa_link_length = (draw_joint_state_[i].local_joint_position[0] - draw_joint_state_[i].local_joint_position[1]).GetLength();
		draw_data_[i].femur_link_length = (draw_joint_state_[i].local_joint_position[1] - draw_joint_state_[i].local_joint_position[2]).GetLength();
		draw_data_[i].tibia_link_length = (draw_joint_state_[i].local_joint_position[2] - draw_joint_state_[i].local_joint_position[3]).GetLength();

		// �Ԑڊp�x���͈͓��ɂ��邩�ǂ����𒲂ׂ�D
		draw_data_[i].is_able_coxa_angle = !(draw_joint_state_[i].joint_angle[0] < HexapodConst::PHANTOMX_COXA_DEFAULT_ANGLE[i] + HexapodConst::PHANTOMX_COXA_ANGLE_MIN ||
			HexapodConst::PHANTOMX_COXA_DEFAULT_ANGLE[i] + HexapodConst::PHANTOMX_COXA_ANGLE_MAX < draw_joint_state_[i].joint_angle[0]);
		draw_data_[i].is_able_femur_angle = !(draw_joint_state_[i].joint_angle[1] < HexapodConst::PHANTOMX_FEMUR_ANGLE_MIN || HexapodConst::PHANTOMX_FEMUR_ANGLE_MAX < draw_joint_state_[i].joint_angle[1]);
		draw_data_[i].is_able_tibia_angle = !(draw_joint_state_[i].joint_angle[2] < HexapodConst::PHANTOMX_TIBIA_ANGLE_MIN || HexapodConst::PHANTOMX_TIBIA_ANGLE_MAX < draw_joint_state_[i].joint_angle[2]);
	}
}


void PhantomXRendererSimple::Draw() const
{
	//���̂�`�悷��D
	std::array<VECTOR, HexapodConst::kLegNum> vertex;

	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		vertex[i] = dldu::ConvertToDxlibVec(draw_joint_state_[i].global_joint_position[0]);
	}

	dldu::DrawHexagonalPrism(vertex, HexapodConst::BODY_HEIGHT, kColorBody);

	//�r��`�悷��D
	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		//�r�̐F��V�r�E�ڒn�ŕύX����D
		const unsigned int kLegBaseColor = dllf::IsGrounded(draw_node_.leg_state, i) ? kColorLeg : kColorLiftedLeg;
		const unsigned int kJointColor = dllf::IsGrounded(draw_node_.leg_state, i) ? kColorJoint : kColorLiftedJoint;

		//�e�r�̕`��
		DrawCapsule3D(draw_data_[i].coxa_joint_pos, draw_data_[i].femur_joint_pos, kLegRadius, kCapsuleDivNum, kLegBaseColor, kLegBaseColor, TRUE);		//coxa
		DrawCapsule3D(draw_data_[i].femur_joint_pos, draw_data_[i].tibia_joint_pos, kLegRadius, kCapsuleDivNum, kLegBaseColor, kLegBaseColor, TRUE);	//femur
		DrawCone3D(draw_data_[i].leg_end_pos, draw_data_[i].tibia_joint_pos, kLegRadius, kCapsuleDivNum, kLegBaseColor, kLegBaseColor, TRUE);			//tibia 

		//�Ԑڂ̕`��
		DrawSphere3D(draw_data_[i].coxa_joint_pos, kJointRadius, kSphereDivNum, draw_data_[i].is_able_coxa_angle ? kJointColor : kColorErrorJoint, draw_data_[i].is_able_coxa_angle ? kJointColor : kColorErrorJoint, TRUE);
		DrawSphere3D(draw_data_[i].femur_joint_pos, kJointRadius, kSphereDivNum, draw_data_[i].is_able_femur_angle ? kJointColor : kColorErrorJoint, draw_data_[i].is_able_femur_angle ? kJointColor : kColorErrorJoint, TRUE);
		DrawSphere3D(draw_data_[i].tibia_joint_pos, kJointRadius, kSphereDivNum, draw_data_[i].is_able_tibia_angle ? kJointColor : kColorErrorJoint, draw_data_[i].is_able_tibia_angle ? kJointColor : kColorErrorJoint, TRUE);

		//�r��̕`��
		DrawSphere3D(draw_data_[i].leg_end_pos, kJointRadius / 2, kSphereDivNum, kJointColor, kJointColor, TRUE);

		//�r�̐ڒn�̊�n�_�̕`��
		DrawSphere3D(draw_data_[i].leg_reference_pos, kJointRadius / 3, kSphereDivNum, kColorLegBase, kColorLegBase, TRUE);


		//�^���w�Ōv�Z�����r�̕`��
		DrawCapsule3D(draw_data_[i].kine_coxa_joint_pos, draw_data_[i].kine_femur_joint_pos, kLegRadius - 5, kCapsuleDivNum, kColorKineLeg, kColorKineLeg, TRUE);		//coxa
		DrawCapsule3D(draw_data_[i].kine_femur_joint_pos, draw_data_[i].kine_tibia_joint_pos, kLegRadius - 5, kCapsuleDivNum, kColorKineLeg, kColorKineLeg, TRUE);	//femur
		DrawCone3D(draw_data_[i].kine_leg_end_pos, draw_data_[i].kine_tibia_joint_pos, kLegRadius - 5, kCapsuleDivNum, kColorKineLeg, kColorKineLeg, TRUE);				//tibia

		//�^���w�Ōv�Z�����Ԑڂ̕`��
		DrawSphere3D(draw_data_[i].kine_coxa_joint_pos, kJointRadius - 5, kSphereDivNum, kColorKineJoint, kColorKineJoint, TRUE);
		DrawSphere3D(draw_data_[i].kine_femur_joint_pos, kJointRadius - 5, kSphereDivNum, kColorKineJoint, kColorKineJoint, TRUE);
		DrawSphere3D(draw_data_[i].kine_tibia_joint_pos, kJointRadius - 5, kSphereDivNum, kColorKineJoint, kColorKineJoint, TRUE);

		//�^���w�Ōv�Z�����r��̕`��
		DrawSphere3D(draw_data_[i].kine_leg_end_pos, kJointRadius / 2 - 2, kSphereDivNum, kColorKineLeg, kColorKineLeg, TRUE);


		// �G���[�o�́D 
		if (!dlm::IsEqual(draw_data_[i].coxa_link_length, HexapodConst::PHANTOMX_COXA_LENGTH))
		{
			DrawString(
				static_cast<int>(ConvWorldPosToScreenPos(dldu::ConvertToDxlibVec((draw_joint_state_[i].global_joint_position[0] + draw_joint_state_[i].global_joint_position[1]) / 2)).x),
				static_cast<int>(ConvWorldPosToScreenPos(dldu::ConvertToDxlibVec((draw_joint_state_[i].global_joint_position[0] + draw_joint_state_[i].global_joint_position[1]) / 2)).y),
				"Error : Coxa Length", kColorErrorText
			);
		}

		if (!dlm::IsEqual(draw_data_[i].femur_link_length, HexapodConst::PHANTOMX_FEMUR_LENGTH))
		{
			DrawString(
				static_cast<int>(ConvWorldPosToScreenPos(dldu::ConvertToDxlibVec((draw_joint_state_[i].global_joint_position[1] + draw_joint_state_[i].global_joint_position[2]) / 2)).x),
				static_cast<int>(ConvWorldPosToScreenPos(dldu::ConvertToDxlibVec((draw_joint_state_[i].global_joint_position[1] + draw_joint_state_[i].global_joint_position[2]) / 2)).y),
				"Error : Femur Length", kColorErrorText
			);
		}

		if (!dlm::IsEqual(draw_data_[i].tibia_link_length, HexapodConst::PHANTOMX_TIBIA_LENGTH))
		{
			DrawString(
				static_cast<int>(ConvWorldPosToScreenPos(dldu::ConvertToDxlibVec((draw_joint_state_[i].global_joint_position[2] + draw_joint_state_[i].global_joint_position[3]) / 2)).x),
				static_cast<int>(ConvWorldPosToScreenPos(dldu::ConvertToDxlibVec((draw_joint_state_[i].global_joint_position[2] + draw_joint_state_[i].global_joint_position[3]) / 2)).y),
				"Error : Tibia Length", kColorErrorText
			);
		}

	}
}


bool PhantomXRendererSimple::IsAbleCoxaLeg(const designlab::Vector3& coxa_joint, const designlab::Vector3& femur_joint) const
{
	if (abs((coxa_joint - femur_joint).GetLength() - HexapodConst::PHANTOMX_COXA_LENGTH) < dlm::kAllowableError) { return true; }
	return false;
}


bool PhantomXRendererSimple::IsAbleFemurLeg(const designlab::Vector3& femur_joint, const designlab::Vector3& tibia_joint) const
{
	if (abs((femur_joint - tibia_joint).GetLength() - HexapodConst::PHANTOMX_FEMUR_LENGTH) < dlm::kAllowableError) { return true; }
	return false;
}


bool PhantomXRendererSimple::IsAbleTibiaLeg(const designlab::Vector3& tibia_joint, const designlab::Vector3& leg_joint) const
{
	if (abs((tibia_joint - leg_joint).GetLength() - HexapodConst::PHANTOMX_TIBIA_LENGTH) < 10) { return true; }
	return false;
}
