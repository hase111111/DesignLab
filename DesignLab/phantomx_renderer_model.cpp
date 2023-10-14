#include "phantomx_renderer_model.h"

#ifndef DESIGNLAB_DONOT_USE_DXLIB

#include <DxLib.h>

#include "designlab_math_util.h"
#include "designlab_rotation_matrix.h"
#include "dxlib_util.h"
#include "model_loader.h"
#include "phantomx_const.h"


namespace dlm = designlab::math_util;
namespace dldu = designlab::dxlib_util;


PhantomXRendererModel::PhantomXRendererModel(const std::shared_ptr<const AbstractHexapodStateCalculator>& calculator_ptr) :
	calculator_ptr_(calculator_ptr)
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
	}


	{
		unsigned int color = GetColor(0, 0, 0);

		DrawSphere3D(dldu::ConvertToDxlibVec(draw_node_.global_center_of_mass), 10, 16, color, color, TRUE);
	
		for (int i = 0; i < 6; i++)
		{
			designlab::Vector3 leg_joint = calculator_ptr_->GetGlobalLegPosition(i, draw_node_.leg_pos[i], draw_node_.global_center_of_mass, draw_node_.rot, true);

			DrawSphere3D(dldu::ConvertToDxlibVec(leg_joint), 5, 16, color, color, TRUE);

			designlab::Vector3 coxa_joint = calculator_ptr_->GetGlobalLegBasePosition(i, draw_node_.global_center_of_mass, draw_node_.rot, true);

			DrawSphere3D(dldu::ConvertToDxlibVec(coxa_joint), 5, 16, color, color, TRUE);
		}

	}
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
	int connect_model_handle = ModelLoader::GetIns()->LoadModel("model/connect.mv1");

	if (connect_model_handle == -1) { printfDx("���f���̓ǂݍ��݂Ɏ��s���܂����D(connect_model_handle)"); }

	if (draw_joint_state_[leg_index].global_joint_position.size() != 4) { return; }
	if (draw_joint_state_[leg_index].joint_angle.size() != 3) { return; }


	const VECTOR kScale = VGet(10.f, 10.f, 10.f);

	const VECTOR kCoxaJointPos = dldu::ConvertToDxlibVec(draw_joint_state_[leg_index].global_joint_position[0]);

	const float kCoxaAngle = draw_joint_state_[leg_index].joint_angle[0];

	const designlab::RotationMatrix3x3 kDefRotMat = 
		designlab::RotationMatrix3x3::CreateRotationMatrixZ(kCoxaAngle) *
		designlab::RotationMatrix3x3::CreateRotationMatrixY(designlab::math_util::ConvertDegToRad(90.0f));

	const designlab::RotationMatrix3x3 kBodyRotMat(draw_node_.rot);

	const float kOffsetLength = 26.33f;

	const VECTOR kOffsetPos = dldu::ConvertToDxlibVec(
		designlab::rotVector
		(
			designlab::Vector3::GetFrontVec() * kOffsetLength, 
			designlab::RotationMatrix3x3::CreateRotationMatrixZ(kCoxaAngle) * kBodyRotMat
		)
	);

	


	MV1SetScale(connect_model_handle, kScale);

	// dxlib�̍��W�n�͍�����W�n�Ȃ̂ŁC�E����W�n�ɕϊ����邽�߂ɋt�]������D
	designlab::EulerXYZ rot = (kBodyRotMat * kDefRotMat).ToEulerXYZ() * -1;
	MV1SetRotationXYZ(connect_model_handle, VGet(rot.x_angle, rot.y_angle, rot.z_angle));

	MV1SetPosition(connect_model_handle, VAdd(kCoxaJointPos, kOffsetPos));

	MV1DrawModel(connect_model_handle);
}



#endif