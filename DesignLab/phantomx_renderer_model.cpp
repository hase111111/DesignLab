#include "phantomx_renderer_model.h"

#ifndef DESIGNLAB_DONOT_USE_DXLIB

#include <DxLib.h>

#include "designlab_math_util.h"
#include "designlab_rot_converter.h"
#include "designlab_rotation_matrix.h"
#include "dxlib_util.h"
#include "model_loader.h"
#include "phantomx_mk2_const.h"


namespace dlm = designlab::math_util;
namespace dldu = designlab::dxlib_util;


PhantomXRendererModel::PhantomXRendererModel(
	const std::shared_ptr<const IHexapodCoordinateConverter>& converter_ptr_,
	const std::shared_ptr<const IHexapodJointCalculator>& calculator_ptr
) :
	converter_ptr_(converter_ptr_),
	calculator_ptr_(calculator_ptr),
	draw_node_{}
{
}

void PhantomXRendererModel::SetDrawNode(const RobotStateNode& node)
{
	draw_node_ = node;

	draw_joint_state_ = calculator_ptr_->CalculateAllJointState(node);
}

void PhantomXRendererModel::Draw() const
{
	dldu::SetZBufferEnable();	// Z�o�b�t�@��L���ɂ���

	DrawBody();		// �{�f�B�̕`��

	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		DrawCoxaLink(i);	// �r�̕`��

		DrawFemurLink(i);	// �r�̕`��

		DrawTibiaLink(i);	// �r�̕`��

		DrawJointAxis(i);	// �֐ߎ��̕`��
	}
}

void PhantomXRendererModel::DrawBody() const
{
	const int body_model_handle = ModelLoader::GetIns()->GetModelHandle("model/body.mv1");

	// ���f���̓ǂݍ��݂�����Ă��Ȃ���Ε`�悵�Ȃ�(�Ƃ������ł��Ȃ�)
	if (body_model_handle == -1) { printfDx("���f���̓ǂݍ��݂Ɏ��s���܂����D(body_model_handle)"); }

	const VECTOR kScale = VGet(10.f, 10.f, 10.f);


	MV1SetScale(body_model_handle, kScale);

	// dxlib�̍��W�n�͍�����W�n�Ȃ̂ŁC�E����W�n�ɕϊ����邽�߂ɋt�]������D
	MV1SetRotationXYZ(body_model_handle, VGet(-draw_node_.rot.x_angle, -draw_node_.rot.y_angle, -draw_node_.rot.z_angle ));

	MV1SetPosition(body_model_handle, dldu::ConvertToDxlibVec(draw_node_.global_center_of_mass));

	MV1DrawModel(body_model_handle);
}

void PhantomXRendererModel::DrawCoxaLink(const int leg_index) const
{
	const int coxa_model_handle = ModelLoader::GetIns()->GetModelHandle("model/coxa_fixed.mv1");

	if (coxa_model_handle == -1) { printfDx("���f���̓ǂݍ��݂Ɏ��s���܂����D(coxa_model_handle)"); }

	if (draw_joint_state_[leg_index].joint_pos_leg_coordinate.size() != 4) { return; }
	if (draw_joint_state_[leg_index].joint_angle.size() != 3) { return; }

	//Coxa Joint��2��Connect Link�ō\������Ă���̂ŁC���ꂼ��`�悷��
	const VECTOR kScale = VGet(10.f, 10.f, 10.f);

	const VECTOR kCoxaJointPos = dldu::ConvertToDxlibVec(
		converter_ptr_->ConvertLegToGlobalCoordinate(
			draw_joint_state_[leg_index].joint_pos_leg_coordinate[0], leg_index, draw_node_.global_center_of_mass, draw_node_.rot, true
		)
	);

	const float kCoxaAngle = draw_joint_state_[leg_index].joint_angle[0];

	const designlab::RotationMatrix3x3 kBodyRotMat = designlab::ToRotationMatrix(draw_node_.rot);

	const float kOffsetLength = 0;	//��]���S�ƌ��_������Ă���̂ŁC���̕���␳����

	{
		const designlab::RotationMatrix3x3 kDefRotMat =
			designlab::RotationMatrix3x3::CreateRotationMatrixZ(kCoxaAngle) *
			designlab::RotationMatrix3x3::CreateRotationMatrixY(dlm::ConvertDegToRad(-90.0f));

		const VECTOR kOffsetPos = dldu::ConvertToDxlibVec(
			designlab::RotateVector3
			(
				designlab::Vector3::GetFrontVec() * kOffsetLength,
				designlab::RotationMatrix3x3::CreateRotationMatrixZ(kCoxaAngle) * kBodyRotMat
			)
		);

		MV1SetScale(coxa_model_handle, kScale);

		// dxlib�̍��W�n�͍�����W�n�Ȃ̂ŁC�E����W�n�ɕϊ����邽�߂ɋt�]������D
		designlab::EulerXYZ rot = (kBodyRotMat * kDefRotMat).ToEulerXYZ() * -1.f;
		MV1SetRotationXYZ(coxa_model_handle, VGet(rot.x_angle, rot.y_angle, rot.z_angle));

		MV1SetPosition(coxa_model_handle, kCoxaJointPos + kOffsetPos);

		MV1DrawModel(coxa_model_handle);
	}
}

void PhantomXRendererModel::DrawFemurLink(const int leg_index) const
{
	const int thign_model_handle = ModelLoader::GetIns()->GetModelHandle("model/thign_l.mv1");

	if (thign_model_handle == -1) { printfDx("���f���̓ǂݍ��݂Ɏ��s���܂����D(thign_model_handle)"); }

	if (draw_joint_state_[leg_index].joint_pos_leg_coordinate.size() != 4) { return; }
	if (draw_joint_state_[leg_index].joint_angle.size() != 3) { return; }

	//�p�����[�^�̌v�Z
	const VECTOR kScale = VGet(10.f, 10.f, 10.f);

	const VECTOR kFemurJointPos = dldu::ConvertToDxlibVec(
		converter_ptr_->ConvertLegToGlobalCoordinate(
			draw_joint_state_[leg_index].joint_pos_leg_coordinate[1], leg_index, draw_node_.global_center_of_mass, draw_node_.rot, true
		)
	);

	const float kCoxaAngle = draw_joint_state_[leg_index].joint_angle[0];

	const float kFemurAngle = draw_joint_state_[leg_index].joint_angle[1];

	const designlab::RotationMatrix3x3 kBodyRotMat = designlab::ToRotationMatrix(draw_node_.rot);

	// �����N���Ȃ����Ă��邽�߁C�ȒP�̂��߂ɉ�]�������Ԃ悤�ɉ��z�I�ȃ����N���g���Ă���D
	// ���̂��߁C���z�I�ȃ����N�̊p�x��␳����K�v������D
	const float virtual_link_offset_angle = dlm::ConvertDegToRad(-13.5f);	
	
	const designlab::RotationMatrix3x3 kDefRotMat =
		designlab::RotationMatrix3x3::CreateRotationMatrixZ(kCoxaAngle) *
		designlab::RotationMatrix3x3::CreateRotationMatrixY(-kFemurAngle) *
		designlab::RotationMatrix3x3::CreateRotationMatrixX(dlm::ConvertDegToRad(-90.0f)) *
		designlab::RotationMatrix3x3::CreateRotationMatrixY(dlm::ConvertDegToRad(-90.f)) *
		designlab::RotationMatrix3x3::CreateRotationMatrixX(virtual_link_offset_angle);

	const VECTOR kOffsetPos = dldu::ConvertToDxlibVec(
		designlab::RotateVector3
		(
			designlab::Vector3::GetFrontVec(),
			designlab::RotationMatrix3x3::CreateRotationMatrixZ(kCoxaAngle) * kBodyRotMat
		)
	);

	//�`�悷��D
	MV1SetScale(thign_model_handle, kScale);

	// dxlib�̍��W�n�͍�����W�n�Ȃ̂ŁC�E����W�n�ɕϊ����邽�߂ɋt�]������D
	designlab::EulerXYZ rot = (kBodyRotMat * kDefRotMat).ToEulerXYZ() * -1.f;
	MV1SetRotationXYZ(thign_model_handle, VGet(rot.x_angle, rot.y_angle, rot.z_angle));

	MV1SetPosition(thign_model_handle, kFemurJointPos + kOffsetPos);

	MV1DrawModel(thign_model_handle);
}

void PhantomXRendererModel::DrawTibiaLink(int leg_index) const
{
	// ���f���̓ǂݍ��݂��s���D�����ŌĂяo���Ɩ��t���[���ǂݍ��ނ��ƂɂȂ肻�������C���ۂ͊��ɓǍ��ς݂Ȃ�΂��̃n���h�����Ԃ��Ă��邾���Ȃ̂Ŗ��Ȃ��D
	// ����ȂƂ���ł��̏����������Ă���̂́C�R���X�g���N�^�ŌĂяo���ƁCDxlib�̏��������I����Ă��Ȃ��̂ŁC�G���[���o�邩��ł���D
	int tibia_model_handle = ModelLoader::GetIns()->GetModelHandle("model/tibia_l_fixed.mv1");

	if (tibia_model_handle == -1) { printfDx("���f���̓ǂݍ��݂Ɏ��s���܂����D(tibia_model_handle)"); }

	if (draw_joint_state_[leg_index].joint_pos_leg_coordinate.size() != 4) { return; }
	if (draw_joint_state_[leg_index].joint_angle.size() != 3) { return; }

	//�p�����[�^�̌v�Z
	const VECTOR kScale = VGet(0.01f, 0.01f, 0.01f);

	const VECTOR kTibiaJointPos = dldu::ConvertToDxlibVec(
		converter_ptr_->ConvertLegToGlobalCoordinate(
			draw_joint_state_[leg_index].joint_pos_leg_coordinate[2], leg_index, draw_node_.global_center_of_mass, draw_node_.rot, true
		)
	);

	const float kCoxaAngle = draw_joint_state_[leg_index].joint_angle[0];

	const float kFemurAngle = draw_joint_state_[leg_index].joint_angle[1];

	const float kTibiaAngle = draw_joint_state_[leg_index].joint_angle[2];

	const designlab::RotationMatrix3x3 kBodyRotMat = designlab::ToRotationMatrix(draw_node_.rot);

	const designlab::RotationMatrix3x3 kDefRotMat =
		designlab::RotationMatrix3x3::CreateRotationMatrixZ(kCoxaAngle) *
		designlab::RotationMatrix3x3::CreateRotationMatrixY(-kFemurAngle) *
		designlab::RotationMatrix3x3::CreateRotationMatrixY(-kTibiaAngle) *
		designlab::RotationMatrix3x3::CreateRotationMatrixX(dlm::ConvertDegToRad(-90.0f)) *
		designlab::RotationMatrix3x3::CreateRotationMatrixY(dlm::ConvertDegToRad(90.f));

	const VECTOR kOffsetPos = dldu::ConvertToDxlibVec(
	designlab::RotateVector3
		(
			designlab::Vector3::GetFrontVec(),
			designlab::RotationMatrix3x3::CreateRotationMatrixZ(kCoxaAngle) * kBodyRotMat
		)
	);

	//�`�悷��D
	MV1SetScale(tibia_model_handle, kScale);

	// dxlib�̍��W�n�͍�����W�n�Ȃ̂ŁC�E����W�n�ɕϊ����邽�߂ɋt�]������D
	designlab::EulerXYZ rot = (kBodyRotMat * kDefRotMat).ToEulerXYZ() * -1.f;
	MV1SetRotationXYZ(tibia_model_handle, VGet(rot.x_angle, rot.y_angle, rot.z_angle));

	MV1SetPosition(tibia_model_handle, kTibiaJointPos + kOffsetPos);

	MV1DrawModel(tibia_model_handle);
}

void PhantomXRendererModel::DrawJointAxis(int leg_index) const
{
	if (draw_joint_state_[leg_index].joint_pos_leg_coordinate.size() != 4) { return; }
	if (draw_joint_state_[leg_index].joint_angle.size() != 3) { return; }


	const float kAxisLength = 100.f;
	const float kAxisRadius = 2.f;
	const int kAxisDivNum = 16;

	const unsigned int kCoxaAxisColor = GetColor(0, 0, 255);
	const unsigned int kFemurAxisColor = GetColor(0, 255, 0);
	const unsigned int kTibiaAxisColor = kFemurAxisColor;
	const unsigned int kSpecColor = GetColor(255, 255, 255);
	[[maybe_unused]]const unsigned int kJointColor = GetColor(64, 64, 64);

	const float kCoxaAngle = draw_joint_state_[leg_index].joint_angle[0];

	const designlab::RotationMatrix3x3 kBodyRotMat = designlab::ToRotationMatrix(draw_node_.rot);

	//Coxa�̉�]��
	{
		const VECTOR kCoxaJointPos = dldu::ConvertToDxlibVec(
			converter_ptr_->ConvertLegToGlobalCoordinate(
				draw_joint_state_[leg_index].joint_pos_leg_coordinate[0], leg_index, draw_node_.global_center_of_mass, draw_node_.rot, true
			)
		);

		const VECTOR kAxisVec = dldu::ConvertToDxlibVec(
			designlab::RotateVector3(designlab::Vector3::GetUpVec() * kAxisLength / 2, kBodyRotMat)
		);

		DrawCapsule3D(kCoxaJointPos - kAxisVec, kCoxaJointPos + kAxisVec, kAxisRadius, kAxisDivNum, kCoxaAxisColor, kSpecColor, TRUE);

		//�Ԑڂɓ_��`�悷��
		//DrawSphere3D(kCoxaJointPos, kAxisRadius * 2, kAxisDivNum, kJointColor, kSpecColor, TRUE);
	}

	//Femur�̉�]��
	{
		const VECTOR kFemurJointPos = dldu::ConvertToDxlibVec(
			converter_ptr_->ConvertLegToGlobalCoordinate(
				draw_joint_state_[leg_index].joint_pos_leg_coordinate[1], leg_index, draw_node_.global_center_of_mass, draw_node_.rot, true
			)
		);

		const designlab::RotationMatrix3x3 kDefRotMat =
			designlab::RotationMatrix3x3::CreateRotationMatrixZ(kCoxaAngle);

		const VECTOR kAxisVec = dldu::ConvertToDxlibVec(
			designlab::RotateVector3
			(
				designlab::Vector3::GetLeftVec() * kAxisLength / 2,
				kDefRotMat * kBodyRotMat
			)
		);

		DrawCapsule3D(kFemurJointPos - kAxisVec, kFemurJointPos + kAxisVec, kAxisRadius, kAxisDivNum, kFemurAxisColor, kSpecColor, TRUE);

		//�Ԑڂɓ_��`�悷��
		//DrawSphere3D(kFemurJointPos, kAxisRadius * 2, kAxisDivNum, kJointColor, kSpecColor, TRUE);
	}

	//Tibia�̉�]��
	{
		const VECTOR kTibiaJointPos = dldu::ConvertToDxlibVec(
			converter_ptr_->ConvertLegToGlobalCoordinate(
				draw_joint_state_[leg_index].joint_pos_leg_coordinate[2], leg_index, draw_node_.global_center_of_mass, draw_node_.rot, true
			)
		);

		const designlab::RotationMatrix3x3 kDefRotMat =
			designlab::RotationMatrix3x3::CreateRotationMatrixZ(kCoxaAngle);

		const VECTOR kAxisVec = dldu::ConvertToDxlibVec(
			designlab::RotateVector3
			(
				designlab::Vector3::GetLeftVec() * kAxisLength / 2,
				kDefRotMat * kBodyRotMat
			)
		);

		DrawCapsule3D(kTibiaJointPos - kAxisVec, kTibiaJointPos + kAxisVec, kAxisRadius, kAxisDivNum, kTibiaAxisColor, kSpecColor, TRUE);

		//�Ԑڂɓ_��`�悷��
		//DrawSphere3D(kTibiaJointPos, kAxisRadius * 2, kAxisDivNum, kJointColor, kSpecColor, TRUE);
	}
}


#endif	// #ifndef DESIGNLAB_DONOT_USE_DXLIB