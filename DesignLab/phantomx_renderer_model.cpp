#include "phantomx_renderer_model.h"

#ifndef DESIGNLAB_DONOT_USE_DXLIB

#include <DxLib.h>

#include "designlab_math_util.h"
#include "designlab_rot_converter.h"
#include "designlab_rotation_matrix.h"
#include "dxlib_util.h"
#include "model_loader.h"
#include "phantomx_const.h"


namespace dlm = designlab::math_util;
namespace dldu = designlab::dxlib_util;


PhantomXRendererModel::PhantomXRendererModel(const std::shared_ptr<const AbstractHexapodStateCalculator>& calculator_ptr) :
	calculator_ptr_(calculator_ptr),
	draw_node_{}
{
}

void PhantomXRendererModel::SetDrawNode(const RobotStateNode& node)
{
	draw_node_ = node;

	// �t�^���w�ŋ��߂��r�̈ʒu���v�Z����
	calculator_ptr_->CalculateAllJointState(draw_node_ , &draw_joint_state_);
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

	//{
	//	unsigned int color = GetColor(0, 0, 0);

	//	DrawSphere3D(dldu::ConvertToDxlibVec(draw_node_.global_center_of_mass), 10, 16, color, color, TRUE);
	//
	//	for (int i = 0; i < 6; i++)
	//	{
	//		designlab::Vector3 leg_joint = calculator_ptr_->GetGlobalLegPosition(i, draw_node_.leg_pos[i], draw_node_.global_center_of_mass, draw_node_.rot, true);

	//		DrawSphere3D(dldu::ConvertToDxlibVec(leg_joint), 5, 16, color, color, TRUE);

	//		designlab::Vector3 coxa_joint = calculator_ptr_->GetGlobalLegBasePosition(i, draw_node_.global_center_of_mass, draw_node_.rot, true);

	//		DrawSphere3D(dldu::ConvertToDxlibVec(coxa_joint), 5, 16, color, color, TRUE);
	//	}

	//}
}

bool PhantomXRendererModel::IsAbleCoxaLeg(const designlab::Vector3& coxa_joint, const designlab::Vector3& femur_joint) const
{
	if (dlm::IsEqual((coxa_joint - femur_joint).GetLength(), PhantomXConst::kCoxaLength)) { return true; }
	return false;
}


bool PhantomXRendererModel::IsAbleFemurLeg(const designlab::Vector3& femur_joint, const designlab::Vector3& tibia_joint) const
{
	if (dlm::IsEqual((femur_joint - tibia_joint).GetLength(), PhantomXConst::kFemurLength)) { return true; }
	return false;
}


bool PhantomXRendererModel::IsAbleTibiaLeg(const designlab::Vector3& tibia_joint, const designlab::Vector3& leg_joint) const
{
	if (dlm::IsEqual((tibia_joint - leg_joint).GetLength(), PhantomXConst::kTibiaLength) ) { return true; }
	return false;
}

void PhantomXRendererModel::DrawBody() const
{
	// ���f���̓ǂݍ��݂��s���D�����ŌĂяo���Ɩ��t���[���ǂݍ��ނ��ƂɂȂ肻�������C���ۂ͊��ɓǍ��ς݂Ȃ�΂��̃n���h�����Ԃ��Ă��邾���Ȃ̂Ŗ��Ȃ��D
	// ����ȂƂ���ł��̏����������Ă���̂́C�R���X�g���N�^�ŌĂяo���ƁCDxlib�̏��������I����Ă��Ȃ��̂ŁC�G���[���o�邩��ł���D
	int body_model_handle = ModelLoader::GetIns()->LoadModel("model/body.mv1");

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
	// ���f���̓ǂݍ��݂��s���D�����ŌĂяo���Ɩ��t���[���ǂݍ��ނ��ƂɂȂ肻�������C���ۂ͊��ɓǍ��ς݂Ȃ�΂��̃n���h�����Ԃ��Ă��邾���Ȃ̂Ŗ��Ȃ��D
	// ����ȂƂ���ł��̏����������Ă���̂́C�R���X�g���N�^�ŌĂяo���ƁCDxlib�̏��������I����Ă��Ȃ��̂ŁC�G���[���o�邩��ł���D
	int coxa_model_handle = ModelLoader::GetIns()->LoadModel("model/coxa_fixed.mv1");

	if (coxa_model_handle == -1) { printfDx("���f���̓ǂݍ��݂Ɏ��s���܂����D(coxa_model_handle)"); }

	if (draw_joint_state_[leg_index].global_joint_position.size() != 4) { return; }
	if (draw_joint_state_[leg_index].joint_angle.size() != 3) { return; }

	//Coxa Joint��2��Connect Link�ō\������Ă���̂ŁC���ꂼ��`�悷��
	const VECTOR kScale = VGet(10.f, 10.f, 10.f);

	const VECTOR kCoxaJointPos = dldu::ConvertToDxlibVec(draw_joint_state_[leg_index].global_joint_position[0]);

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

	//{
	//	const designlab::RotationMatrix3x3 kDefRotMat =
	//		designlab::RotationMatrix3x3::CreateRotationMatrixZ(kCoxaAngle) *
	//		designlab::RotationMatrix3x3::CreateRotationMatrixX(dlm::ConvertDegToRad(-90.0f)) *
	//		designlab::RotationMatrix3x3::CreateRotationMatrixY(dlm::ConvertDegToRad(90.f));
	//
	//	const float kOffsetLength2 = kOffsetLength + 5.0f;	//��]���S�ƌ��_������Ă���̂ŁC���̕���␳����
	//
	//	const VECTOR kOffsetPos = dldu::ConvertToDxlibVec(
	//		designlab::RotateVector3
	//		(
	//			designlab::Vector3::GetFrontVec() * kOffsetLength2,
	//			designlab::RotationMatrix3x3::CreateRotationMatrixZ(kCoxaAngle) * kBodyRotMat
	//		)
	//	);
	//
	//	MV1SetScale(connect_model_handle, kScale);
	//
	//	// dxlib�̍��W�n�͍�����W�n�Ȃ̂ŁC�E����W�n�ɕϊ����邽�߂ɋt�]������D
	//	designlab::EulerXYZ rot = (kBodyRotMat * kDefRotMat).ToEulerXYZ() * -1.f;
	//	MV1SetRotationXYZ(connect_model_handle, VGet(rot.x_angle, rot.y_angle, rot.z_angle));
	//
	//	MV1SetPosition(connect_model_handle, kCoxaJointPos + kOffsetPos);
	//
	//	MV1DrawModel(connect_model_handle);
	//}
}

void PhantomXRendererModel::DrawFemurLink(int leg_index) const
{
	// ���f���̓ǂݍ��݂��s���D�����ŌĂяo���Ɩ��t���[���ǂݍ��ނ��ƂɂȂ肻�������C���ۂ͊��ɓǍ��ς݂Ȃ�΂��̃n���h�����Ԃ��Ă��邾���Ȃ̂Ŗ��Ȃ��D
	// ����ȂƂ���ł��̏����������Ă���̂́C�R���X�g���N�^�ŌĂяo���ƁCDxlib�̏��������I����Ă��Ȃ��̂ŁC�G���[���o�邩��ł���D
	int thign_model_handle = ModelLoader::GetIns()->LoadModel("model/thign_l.mv1");

	if (thign_model_handle == -1) { printfDx("���f���̓ǂݍ��݂Ɏ��s���܂����D(thign_model_handle)"); }

	if (draw_joint_state_[leg_index].global_joint_position.size() != 4) { return; }
	if (draw_joint_state_[leg_index].joint_angle.size() != 3) { return; }

	//�p�����[�^�̌v�Z
	const VECTOR kScale = VGet(10.f, 10.f, 10.f);

	const VECTOR kFemurJointPos = dldu::ConvertToDxlibVec(draw_joint_state_[leg_index].global_joint_position[1]);

	const float kCoxaAngle = draw_joint_state_[leg_index].joint_angle[0];

	const float kFemurAngle = draw_joint_state_[leg_index].joint_angle[1];

	const designlab::RotationMatrix3x3 kBodyRotMat = designlab::ToRotationMatrix(draw_node_.rot);

	const designlab::RotationMatrix3x3 kDefRotMat =
		designlab::RotationMatrix3x3::CreateRotationMatrixZ(kCoxaAngle) *
		designlab::RotationMatrix3x3::CreateRotationMatrixY(-kFemurAngle) *
		designlab::RotationMatrix3x3::CreateRotationMatrixX(dlm::ConvertDegToRad(-90.0f)) *
		designlab::RotationMatrix3x3::CreateRotationMatrixY(dlm::ConvertDegToRad(-90.f)) *
		designlab::RotationMatrix3x3::CreateRotationMatrixX(PhantomXConst::kFemurVirtualLinkOffsetAngle);

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
	int tibia_model_handle = ModelLoader::GetIns()->LoadModel("model/tibia_l_fixed.mv1");

	if (tibia_model_handle == -1) { printfDx("���f���̓ǂݍ��݂Ɏ��s���܂����D(tibia_model_handle)"); }

	if (draw_joint_state_[leg_index].global_joint_position.size() != 4) { return; }
	if (draw_joint_state_[leg_index].joint_angle.size() != 3) { return; }

	//�p�����[�^�̌v�Z
	const VECTOR kScale = VGet(0.01f, 0.01f, 0.01f);

	const VECTOR kTibiaJointPos = dldu::ConvertToDxlibVec(draw_joint_state_[leg_index].global_joint_position[2]);

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
	if (draw_joint_state_[leg_index].global_joint_position.size() != 4) { return; }
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
		const VECTOR kCoxaJointPos = dldu::ConvertToDxlibVec(draw_joint_state_[leg_index].global_joint_position[0]);

		const VECTOR kAxisVec = dldu::ConvertToDxlibVec(
			designlab::RotateVector3(designlab::Vector3::GetUpVec() * kAxisLength / 2, kBodyRotMat)
		);

		DrawCapsule3D(kCoxaJointPos - kAxisVec, kCoxaJointPos + kAxisVec, kAxisRadius, kAxisDivNum, kCoxaAxisColor, kSpecColor, TRUE);

		//�Ԑڂɓ_��`�悷��
		//DrawSphere3D(kCoxaJointPos, kAxisRadius * 2, kAxisDivNum, kJointColor, kSpecColor, TRUE);
	}

	//Femur�̉�]��
	{
		const VECTOR kFemurJointPos = dldu::ConvertToDxlibVec(draw_joint_state_[leg_index].global_joint_position[1]);

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
		const VECTOR kTibiaJointPos = dldu::ConvertToDxlibVec(draw_joint_state_[leg_index].global_joint_position[2]);

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



#endif