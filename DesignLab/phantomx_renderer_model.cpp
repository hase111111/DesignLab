﻿#include "phantomx_renderer_model.h"

#ifndef DESIGNLAB_DONOT_USE_DXLIB

#include <DxLib.h>

#include "designlab_math_util.h"
#include "designlab_rot_converter.h"
#include "designlab_rotation_matrix.h"
#include "dxlib_util.h"
#include "model_loader.h"
#include "phantomx_mk2_const.h"


namespace dl = ::designlab;
namespace dlm = ::designlab::math_util;
namespace dldu = ::designlab::dxlib_util;


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
	dldu::SetZBufferEnable();	// Zバッファを有効にする

	DrawBody();		// ボディの描画

	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		DrawCoxaLink(i);	// 脚の描画

		DrawFemurLink(i);	// 脚の描画

		DrawTibiaLink(i);	// 脚の描画

		DrawJointAxis(i);	// 関節軸の描画
	}
}

void PhantomXRendererModel::DrawBody() const
{
	const int body_model_handle = ModelLoader::GetIns()->GetModelHandle("model/body.mv1");

	// モデルの読み込みがされていなければ描画しない(というかできない)
	if (body_model_handle == -1) { printfDx("モデルの読み込みに失敗しました．(body_model_handle)"); }

	const VECTOR scale = VGet(10.f, 10.f, 10.f);	//モデルの寸法を調整するためのスケール


	MV1SetScale(body_model_handle, scale);

	// dxlibの座標系は左手座標系なので，右手座標系に変換するために逆転させる．
	MV1SetRotationMatrix(body_model_handle,
		dldu::ConvertToDxlibMat(dl::ToRotationMatrix(draw_node_.quat.ToLeftHandCoordinate()))
	);

	MV1SetPosition(body_model_handle, dldu::ConvertToDxlibVec(draw_node_.global_center_of_mass));

	MV1DrawModel(body_model_handle);
}

void PhantomXRendererModel::DrawCoxaLink(const int leg_index) const
{
	const int coxa_model_handle = ModelLoader::GetIns()->GetModelHandle("model/coxa_fixed.mv1");

	if (coxa_model_handle == -1) { printfDx("モデルの読み込みに失敗しました．(coxa_model_handle)"); }

	if (draw_joint_state_[leg_index].joint_pos_leg_coordinate.size() != 4) { return; }
	if (draw_joint_state_[leg_index].joint_angle.size() != 3) { return; }

	//Coxa Jointは2つのConnect Linkで構成されているので，それぞれ描画する
	const VECTOR scale = VGet(10.f, 10.f, 10.f);

	const VECTOR coxa_joint_pos_global = dldu::ConvertToDxlibVec(
		converter_ptr_->ConvertLegToGlobalCoordinate(
			draw_joint_state_[leg_index].joint_pos_leg_coordinate[0], leg_index, draw_node_.global_center_of_mass, draw_node_.quat, true
		)
	);

	const dl::Quaternion coxa_quat = dl::Quaternion::MakeByAngleAxis(dlm::ConvertDegToRad(-90.0f), dl::Vector3::GetLeftVec()) *
		dl::Quaternion::MakeByAngleAxis(draw_joint_state_[leg_index].joint_angle[0], dl::Vector3::GetUpVec()) *
		draw_node_.quat.ToLeftHandCoordinate();

	MV1SetScale(coxa_model_handle, scale);

	// dxlibの座標系は左手座標系なので，右手座標系に変換するために逆転させる．
	MV1SetRotationMatrix(coxa_model_handle, dldu::ConvertToDxlibMat(dl::ToRotationMatrix(coxa_quat)));

	MV1SetPosition(coxa_model_handle, coxa_joint_pos_global);

	MV1DrawModel(coxa_model_handle);
}

void PhantomXRendererModel::DrawFemurLink(const int leg_index) const
{
	const int thign_model_handle = ModelLoader::GetIns()->GetModelHandle("model/thign_l.mv1");

	if (thign_model_handle == -1) { printfDx("モデルの読み込みに失敗しました．(thign_model_handle)"); }

	if (draw_joint_state_[leg_index].joint_pos_leg_coordinate.size() != 4) { return; }
	if (draw_joint_state_[leg_index].joint_angle.size() != 3) { return; }

	//パラメータの計算
	const VECTOR scale = VGet(10.f, 10.f, 10.f);

	const VECTOR femur_joint_pos_global_coord = dldu::ConvertToDxlibVec(
		converter_ptr_->ConvertLegToGlobalCoordinate(
			draw_joint_state_[leg_index].joint_pos_leg_coordinate[1], leg_index, draw_node_.global_center_of_mass, draw_node_.quat, true
		)
	);

	const float coxa_angle = draw_joint_state_[leg_index].joint_angle[0];
	const float femur_angle = draw_joint_state_[leg_index].joint_angle[1];

	// リンクが曲がっているため，簡単のために回転軸を結ぶように仮想的なリンクを使っている．
	// そのため，仮想的なリンクの角度を補正する必要がある．
	const float virtual_link_offset_angle = dlm::ConvertDegToRad(12.5f);	
	
	const dl::Quaternion thign_def_quat =
		dl::Quaternion::MakeByAngleAxis(dlm::ConvertDegToRad(90.0f), dl::Vector3::GetLeftVec()) *
		dl::Quaternion::MakeByAngleAxis(dlm::ConvertDegToRad(-90.0f), dl::Vector3::GetFrontVec()) *
		dl::Quaternion::MakeByAngleAxis(femur_angle, dl::Vector3::GetLeftVec()) *
		dl::Quaternion::MakeByAngleAxis(virtual_link_offset_angle, dl::Vector3::GetLeftVec()) *
		dl::Quaternion::MakeByAngleAxis(coxa_angle, dl::Vector3::GetUpVec());

	//原点の位置が少しズレているので，補正する．
	const VECTOR offset_pos = dldu::ConvertToDxlibVec(
		dl::RotateVector3
		(
			dl::Vector3::GetFrontVec(),
			dl::Quaternion::MakeByAngleAxis(coxa_angle, dl::Vector3::GetUpVec()) * draw_node_.quat.ToLeftHandCoordinate()
		)
	);

	//描画する．
	MV1SetScale(thign_model_handle, scale);

	// dxlibの座標系は左手座標系なので，右手座標系に変換するために逆転させる．
	dl::Quaternion thign_quat = thign_def_quat * draw_node_.quat.ToLeftHandCoordinate();
	MV1SetRotationMatrix(thign_model_handle, dldu::ConvertToDxlibMat(dl::ToRotationMatrix(thign_quat)));

	MV1SetPosition(thign_model_handle, femur_joint_pos_global_coord + offset_pos);

	MV1DrawModel(thign_model_handle);
}

void PhantomXRendererModel::DrawTibiaLink(const int leg_index) const
{
	// モデルの読み込みを行う．ここで呼び出すと毎フレーム読み込むことになりそうだが，実際は既に読込済みならばそのハンドルが返ってくるだけなので問題ない．
	// こんなところでこの処理を書いているのは，コンストラクタで呼び出すと，Dxlibの初期化が終わっていないので，エラーが出るからである．
	int tibia_model_handle = ModelLoader::GetIns()->GetModelHandle("model/tibia_l_fixed.mv1");

	if (tibia_model_handle == -1) { printfDx("モデルの読み込みに失敗しました．(tibia_model_handle)"); }

	if (draw_joint_state_[leg_index].joint_pos_leg_coordinate.size() != 4) { return; }
	if (draw_joint_state_[leg_index].joint_angle.size() != 3) { return; }

	//パラメータの計算
	const VECTOR scale = VGet(0.01f, 0.01f, 0.01f);

	const VECTOR tibia_joint_pos_global_coord = dldu::ConvertToDxlibVec(
		converter_ptr_->ConvertLegToGlobalCoordinate(
			draw_joint_state_[leg_index].joint_pos_leg_coordinate[2], leg_index, draw_node_.global_center_of_mass, draw_node_.quat, true
		)
	);

	const float coxa_angle = draw_joint_state_[leg_index].joint_angle[0];

	const float femur_angle = draw_joint_state_[leg_index].joint_angle[1];

	const float tibia_angle = draw_joint_state_[leg_index].joint_angle[2];

	const dl::Quaternion default_quat = 
		dl::Quaternion::MakeByAngleAxis(dlm::ConvertDegToRad(90.0f), dl::Vector3::GetLeftVec()) *
		dl::Quaternion::MakeByAngleAxis(dlm::ConvertDegToRad(-90.0f), dl::Vector3::GetFrontVec()) *
		dl::Quaternion::MakeByAngleAxis(dlm::ConvertDegToRad(90.0f), dl::Vector3::GetLeftVec()) *
		dl::Quaternion::MakeByAngleAxis(-femur_angle - tibia_angle, dl::Vector3::GetLeftVec()) *
		dl::Quaternion::MakeByAngleAxis(dlm::ConvertDegToRad(-90.0f), dl::Vector3::GetLeftVec()) *
		dl::Quaternion::MakeByAngleAxis(coxa_angle, dl::Vector3::GetUpVec()) * 
		dl::Quaternion::MakeByAngleAxis(dlm::ConvertDegToRad(180.0f), dl::Vector3::GetUpVec());

	const VECTOR offset_pos = dldu::ConvertToDxlibVec(
	dl::RotateVector3
		(
			dl::Vector3::GetFrontVec(),
			dl::Quaternion::MakeByAngleAxis(coxa_angle, dl::Vector3::GetUpVec()) * draw_node_.quat.ToLeftHandCoordinate()
		)
	);

	//描画する．
	MV1SetScale(tibia_model_handle, scale);

	// dxlibの座標系は左手座標系なので，右手座標系に変換するために逆転させる．
	const dl::Quaternion tibia_quat = default_quat * draw_node_.quat.ToLeftHandCoordinate();
	MV1SetRotationMatrix(tibia_model_handle, dldu::ConvertToDxlibMat(dl::ToRotationMatrix(tibia_quat)));

	MV1SetPosition(tibia_model_handle, tibia_joint_pos_global_coord + offset_pos);

	MV1DrawModel(tibia_model_handle);
}

void PhantomXRendererModel::DrawJointAxis(const int leg_index) const
{
	if (draw_joint_state_[leg_index].joint_pos_leg_coordinate.size() != 4) { return; }
	if (draw_joint_state_[leg_index].joint_angle.size() != 3) { return; }

	const float axis_length = 100.f;
	const float axis_radius = 2.f;
	const int axis_div_num = 16;

	const unsigned int coxa_axis_color = GetColor(0, 0, 255);
	const unsigned int femur_axis_color = GetColor(0, 255, 0);
	const unsigned int tibia_axis_color = femur_axis_color;
	const unsigned int spec_color = GetColor(255, 255, 255);

	const float coxa_angle = draw_joint_state_[leg_index].joint_angle[0];

	//Coxaの回転軸
	{
		const VECTOR coxa_joint_pos_global_coord = dldu::ConvertToDxlibVec(
			converter_ptr_->ConvertLegToGlobalCoordinate(
				draw_joint_state_[leg_index].joint_pos_leg_coordinate[0], leg_index, draw_node_.global_center_of_mass, draw_node_.quat, true
			)
		);

		const VECTOR axis_vec = dldu::ConvertToDxlibVec(
			dl::RotateVector3(dl::Vector3::GetUpVec() * axis_length / 2, draw_node_.quat)
		);

		DrawCapsule3D(coxa_joint_pos_global_coord - axis_vec, coxa_joint_pos_global_coord + axis_vec, axis_radius, axis_div_num, coxa_axis_color, spec_color, TRUE);
	}

	//Femurの回転軸
	{
		const VECTOR femur_joint_pos_global_coord = dldu::ConvertToDxlibVec(
			converter_ptr_->ConvertLegToGlobalCoordinate(
				draw_joint_state_[leg_index].joint_pos_leg_coordinate[1], leg_index, draw_node_.global_center_of_mass, draw_node_.quat, true
			)
		);

		const dl::Quaternion default_quat = dl::Quaternion::MakeByAngleAxis(coxa_angle, dl::Vector3::GetUpVec());

		const VECTOR axis_vec = dldu::ConvertToDxlibVec(
			dl::RotateVector3(dl::Vector3::GetLeftVec() * axis_length / 2, draw_node_.quat * default_quat)
		);

		DrawCapsule3D(femur_joint_pos_global_coord - axis_vec, femur_joint_pos_global_coord + axis_vec, axis_radius, axis_div_num, femur_axis_color, spec_color, TRUE);
	}

	//Tibiaの回転軸
	{
		const VECTOR tibia_joint_pos_global_coord = dldu::ConvertToDxlibVec(
			converter_ptr_->ConvertLegToGlobalCoordinate(
				draw_joint_state_[leg_index].joint_pos_leg_coordinate[2], leg_index, draw_node_.global_center_of_mass, draw_node_.quat, true
			)
		);

		const dl::Quaternion default_quat = dl::Quaternion::MakeByAngleAxis(coxa_angle, dl::Vector3::GetUpVec());

		const VECTOR axis_vec = dldu::ConvertToDxlibVec(
			dl::RotateVector3(dl::Vector3::GetLeftVec() * axis_length / 2, draw_node_.quat * default_quat)
		);

		DrawCapsule3D(tibia_joint_pos_global_coord - axis_vec, tibia_joint_pos_global_coord + axis_vec, axis_radius, axis_div_num, tibia_axis_color, spec_color, TRUE);
	}
}


#endif	// DESIGNLAB_DONOT_USE_DXLIB