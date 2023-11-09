#include "graphic_main_test.h"

#include "designlab_math_util.h"
#include "dxlib_util.h"
#include "hexapod_renderer_builder.h"
#include "keyboard.h"
#include "map_renderer.h"
#include "node_initializer.h"
#include "phantomx_const.h"
#include "simulation_map_creator.h"


namespace
{
	constexpr int temp_size = 50;
	constexpr auto temp_ex = 10.8;
	bool temp[HexapodConst::kLegNum][temp_size][temp_size][temp_size] = {};
}

namespace dldu = designlab::dxlib_util;
namespace dlm = designlab::math_util;


GraphicMainTest::GraphicMainTest(const std::shared_ptr<const AbstractHexapodStateCalculator>& calculator_ptr,
	const std::shared_ptr<const ApplicationSettingRecorder>& setting_ptr) :
	calculator_ptr_(calculator_ptr),
	node_display_gui_(setting_ptr ? setting_ptr->window_size_x - NodeDisplayGui::kWidth - 10 : 0, 10, calculator_ptr),
	hexapod_renderer_(HexapodRendererBuilder::Build(calculator_ptr, setting_ptr->gui_display_quality))
{
	NodeInitializer node_initializer;
	RobotStateNode init_node = node_initializer.InitNode();

	SimulationMapCreator map_creator(MapCreateMode::kFlat, static_cast<unsigned int>(MapCreateOption::kNone));
	map_state_ = map_creator.InitMap();
	devide_map_state_.Init(map_state_, {});


	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		for (int x = 0; x < temp_size; x += 1)
		{
			for (int y = 0; y < temp_size; y += 1)
			{
				for (int z = 0; z < temp_size; z += 1)
				{
					designlab::Vector3 pos((float)x - temp_size / 2, (float)y - temp_size / 2, (float)z - temp_size / 2);
					pos *= (float)temp_ex;

					if (calculator_ptr->IsLegInRange(i, pos))
					{
						temp[i][x][y][z] = true;
					}
					else
					{
						temp[i][x][y][z] = false;
					}
				}
			}
		}
	}

}


bool GraphicMainTest::Update()
{
	MoveLeg();

	if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_LSHIFT) > 0 || Keyboard::GetIns()->GetPressingCount(KEY_INPUT_RSHIFT) > 0)
	{
		MoveBody();
	}

	hexapod_renderer_->SetDrawNode(node_);

	node_display_gui_.SetDisplayNode(node_);

	node_display_gui_.Update();

	camera_gui_.SetHexapodPos(node_.global_center_of_mass);  //カメラの位置を更新する．

	camera_gui_.Update();      //カメラのGUIを更新する．

	return true;
}


void GraphicMainTest::Draw() const
{
	dldu::SetZBufferEnable();


	MapRenderer map_render;

	//map_render.Draw(map_state_);


	hexapod_renderer_->Draw();


	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		for (int x = 0; x < temp_size; x += 1)
		{
			for (int y = 0; y < temp_size; y += 1)
			{
				for (int z = 0; z < temp_size; z += 1)
				{
					if (temp[i][x][y][z])
					{
						designlab::Vector3 pos(x - (float)temp_size / 2, y - (float)temp_size / 2, z - (float)temp_size / 2);
						pos *= (float)temp_ex;

						SetDrawBlendMode(DX_BLENDMODE_PMA_ALPHA, 255);
						unsigned int  all_color = GetColor(170, 64, 64);
						unsigned int color[6] = {
							all_color,all_color,all_color,
							all_color,all_color,all_color
							//GetColor(255, 128, 255),GetColor(255, 128, 128),GetColor(255, 255, 128),
							//GetColor(128, 255, 128),GetColor(128, 255, 255),GetColor(128, 128, 255)
						};

						//dldu::DrawCube3D(dldu::ConvertToDxlibVec(calculator_ptr_->GetGlobalLegPosition(i, pos, node_.global_center_of_mass, node_.rot, true)), 10, color[i]);
						SetDrawBlendMode(DX_BLENDMODE_NOBLEND, 0);
					}
					else
					{
						//
					}
				}
			}
		}
	}


	camera_gui_.Draw();        //カメラのGUIを描画する．

	node_display_gui_.Draw();	 //ノードの情報を表示するGUIを描画する．

}

void GraphicMainTest::MoveBody()
{
	const float kComSpeed = 1.1f;

	if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_Q) > 0)
	{
		designlab::Vector3 com = 
			node_.global_center_of_mass + designlab::RotateVector3(designlab::Vector3::GetUpVec() * kComSpeed,node_.rot);
		
		node_.ChangeGlobalCenterOfMass(com, false);
	}
	else if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_E) > 0)
	{
		designlab::Vector3 com =
			node_.global_center_of_mass + designlab::RotateVector3(designlab::Vector3::GetUpVec() * -kComSpeed, node_.rot);

		node_.ChangeGlobalCenterOfMass(com, false);
	}
	else if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_A) > 0)
	{
		designlab::Vector3 com =
			node_.global_center_of_mass + designlab::RotateVector3(designlab::Vector3::GetLeftVec() * kComSpeed, node_.rot);

		node_.ChangeGlobalCenterOfMass(com, false);
	}
	else if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_D) > 0)
	{
		designlab::Vector3 com =
			node_.global_center_of_mass + designlab::RotateVector3(designlab::Vector3::GetLeftVec() * -kComSpeed, node_.rot);

		node_.ChangeGlobalCenterOfMass(com, false);
	}
	else if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_W) > 0)
	{
		designlab::Vector3 com =
			node_.global_center_of_mass + designlab::RotateVector3(designlab::Vector3::GetFrontVec() * kComSpeed, node_.rot);

		node_.ChangeGlobalCenterOfMass(com, false);
	}
	else if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_S) > 0)
	{
		designlab::Vector3 com =
			node_.global_center_of_mass + designlab::RotateVector3(designlab::Vector3::GetFrontVec() * -kComSpeed, node_.rot);

		node_.ChangeGlobalCenterOfMass(com, false);
	}
	else if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_R) > 0)
	{
		designlab::EulerXYZ rot = node_.rot;
		float angle_speed = kComSpeed / 360.0f * 2.f * dlm::kFloatPi;

		if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_I) > 0) 
		{
			angle_speed *= -1.f;
		}

		rot.z_angle += angle_speed;
		node_.rot = rot;
	}
}

void GraphicMainTest::MoveLeg()
{
	const float kSpeed = 1;
	const float kAngleSpeed = 0.01f;

	for (int i = 0; i < HexapodConst::kLegNum; i++)
	{
		if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_1 + i) > 0)
		{
			if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_Q) > 0) { node_.leg_pos[i].z += kSpeed; }
			else if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_E) > 0) { node_.leg_pos[i].z -= kSpeed; }
			else if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_A) > 0) { node_.leg_pos[i].y += kSpeed; }
			else if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_D) > 0) { node_.leg_pos[i].y -= kSpeed; }
			else if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_W) > 0) { node_.leg_pos[i].x += kSpeed; }
			else if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_S) > 0) { node_.leg_pos[i].x -= kSpeed; }
			else if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_M) == 1)
			{
				designlab::Vector3 global = calculator_ptr_->ConvertLegToGlobalCoordinate(
					node_.leg_reference_pos[i], i, node_.global_center_of_mass, node_.rot, true
				);

				int map_x = devide_map_state_.GetDevideMapIndexX(global.x);
				int map_y = devide_map_state_.GetDevideMapIndexY(global.y);

				if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_UP) > 0) { map_x++; }
				else if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_DOWN) > 0) { map_x--; }
				if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_LEFT) > 0) { map_y++; }
				else if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_RIGHT) > 0) { map_y--; }

				designlab::Vector3 map_pos = devide_map_state_.GetPointPos(map_x, map_y, map_index_ % devide_map_state_.GetPointNum(map_x, map_y));
				map_index_++;

				node_.leg_pos[i] = calculator_ptr_->ConvertGlobalToLegCoordinate(
					map_pos, i, node_.global_center_of_mass, node_.rot, true
				);
			}
			
			if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_C) > 0 || Keyboard::GetIns()->GetPressingCount(KEY_INPUT_F) > 0 || Keyboard::GetIns()->GetPressingCount(KEY_INPUT_T) > 0)
			{
				std::array<HexapodJointState, HexapodConst::kLegNum> res;
				calculator_ptr_->CalculateAllJointState(node_, &res);

				float coxa = res[i].joint_angle[0];
				float femur = res[i].joint_angle[1];
				float tibia = res[i].joint_angle[2];

				if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_C) > 0) 
				{
					float spped = Keyboard::GetIns()->GetPressingCount(KEY_INPUT_I) > 0 ? kAngleSpeed : kAngleSpeed * -1.f;
					
					coxa += spped;

					coxa = PhantomXConst::kCoxaDefaultAngle[i] + PhantomXConst::kCoxaAngleMax <= coxa ?
						PhantomXConst::kCoxaDefaultAngle[i] + PhantomXConst::kCoxaAngleMax : coxa;

					coxa = PhantomXConst::kCoxaDefaultAngle[i] + PhantomXConst::kCoxaAngleMin >= coxa ?
						PhantomXConst::kCoxaDefaultAngle[i] + PhantomXConst::kCoxaAngleMin : coxa;
				}
				else if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_F) > 0) 
				{
					float spped = Keyboard::GetIns()->GetPressingCount(KEY_INPUT_I) > 0 ? kAngleSpeed : kAngleSpeed * -1.f;
					
					femur += spped;

					femur = (femur + tibia - dlm::kFloatPi) > 0 ? femur - spped : femur;
					femur = PhantomXConst::kFemurAngleMax <= femur ? PhantomXConst::kFemurAngleMax : femur;
					femur = PhantomXConst::kFemurAngleMin >= femur ? PhantomXConst::kFemurAngleMin : femur;

				}
				else if (Keyboard::GetIns()->GetPressingCount(KEY_INPUT_T) > 0) 
				{
					float spped = Keyboard::GetIns()->GetPressingCount(KEY_INPUT_I) > 0 ? kAngleSpeed : kAngleSpeed * -1.f;
					tibia += spped;
					tibia = (femur + tibia - dlm::kFloatPi) > 0 ? tibia - spped : tibia;
					tibia = PhantomXConst::kTibiaAngleMax <= tibia ? PhantomXConst::kTibiaAngleMax : tibia;
					tibia = PhantomXConst::kTibiaAngleMin >= tibia ? PhantomXConst::kTibiaAngleMin : tibia;
				}

				designlab::Vector3 leg_pos;
				leg_pos += designlab::Vector3{ PhantomXConst::kCoxaLength * cos(coxa),PhantomXConst::kCoxaLength * sin(coxa),0 };

				leg_pos += designlab::Vector3{
					PhantomXConst::kFemurLength * cos(coxa) * cos(femur),
					PhantomXConst::kFemurLength * sin(coxa) * cos(femur),
					PhantomXConst::kFemurLength * sin(femur) 
				};
				leg_pos += designlab::Vector3{
					PhantomXConst::kTibiaLength * cos(coxa) * cos(femur + tibia),
					PhantomXConst::kTibiaLength * sin(coxa) * cos(femur + tibia),
					PhantomXConst::kTibiaLength * sin(femur + tibia)
				};

				node_.leg_pos[i] = leg_pos;
			}
		}
	}
}