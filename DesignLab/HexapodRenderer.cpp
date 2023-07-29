#include "HexapodRenderer.h"
#include "DxLib.h"
#include "Dxlib3DFunction.h"
#include "LegState.h"
#include "MyMath.h"


bool HexapodRenderer::isAbleCoxaLeg(const my_vec::SVector _coxa_joint, const my_vec::SVector _femur_joint) const
{
	if (abs((_coxa_joint - _femur_joint).length() - HexapodConst::COXA_LENGTH) < my_math::ALLOWABLE_ERROR) { return true; }
	return false;
}

bool HexapodRenderer::isAbleFemurLeg(const my_vec::SVector _femur_joint, const my_vec::SVector _tibia_joint) const
{
	if (abs((_femur_joint - _tibia_joint).length() - HexapodConst::FEMUR_LENGTH) < my_math::ALLOWABLE_ERROR) { return true; }
	return false;
}

bool HexapodRenderer::isAbleTibiaLeg(const my_vec::SVector _tibia_joint, const my_vec::SVector _leg_joint) const
{
	if (abs((_tibia_joint - _leg_joint).length() - HexapodConst::TIBIA_LENGTH) < 10) { return true; }
	return false;
}

HexapodRenderer::HexapodRenderer() :
	COLOR_BODY(GetColor(23, 58, 235)), COLOR_LEG(GetColor(23, 58, 235)), COLOR_LIFTED_LEG(GetColor(240, 30, 60)),
	COLOR_JOINT(GetColor(100, 100, 200)), COLOR_LIFTED_JOINT(GetColor(200, 100, 100)), CAPSULE_DIV_NUM(6), SPHERE_DIV_NUM(16),
	COLOR_LEG_BASE(GetColor(100, 200, 100))
{
}

void HexapodRenderer::update(const SNode& _node)
{
	m_HexaCalc.calclateJointPos(_node);
}

void HexapodRenderer::draw(const SNode& _node) const
{
	//��ver��hexapodGraphic�̋L�q�ɂ��������ĕ`�悷��D
	using namespace myDxlib3DFunc;

	//���̂�`�悷��D
	VECTOR vertex[6];
	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		//vertex[i] = convertToDxVec(_hexapod.getGlobalCoxaJointPos(i));
		vertex[i] = convertToDxVec(m_HexaCalc.getGlobalCoxaJointPos(_node, i, true));
	}

	drawHexagonalPrism(vertex, HexapodConst::BODY_HEIGHT, COLOR_BODY);

	//�r��`�悷��D
	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		const VECTOR kLegEndPos = convertToDxVec(m_HexaCalc.getGlobalLegPos(_node, i, true));
		const VECTOR kLegBasePos = convertToDxVec(m_HexaCalc.getGlobalLegBasePos(_node, i, true));
		const VECTOR kCoxaJointPos = convertToDxVec(m_HexaCalc.getGlobalCoxaJointPos(_node, i, true));
		const VECTOR kFemurJointPos = convertToDxVec(m_HexaCalc.getGlobalFemurJointPos(_node, i));
		const VECTOR kTibiaJointPos = convertToDxVec(m_HexaCalc.getGlobalTibiaJointPos(_node, i));

		//�r�̐F��V�r�E�ڒn�ŕύX����D
		const unsigned int kLegBaseColor = LegStateEdit::isGrounded(_node.leg_state, i) ? COLOR_LEG : COLOR_LIFTED_LEG;
		const unsigned int kJointColor = LegStateEdit::isGrounded(_node.leg_state, i) ? COLOR_JOINT : COLOR_LIFTED_JOINT;

		//�e�r�̕`��
		DrawCapsule3D(kCoxaJointPos, kFemurJointPos, LEG_R, CAPSULE_DIV_NUM, kLegBaseColor, kLegBaseColor, TRUE);	//coxa
		DrawCapsule3D(kFemurJointPos, kTibiaJointPos, LEG_R, CAPSULE_DIV_NUM, kLegBaseColor, kLegBaseColor, TRUE);	//femur
		DrawCone3D(kLegEndPos, kTibiaJointPos, LEG_R, CAPSULE_DIV_NUM, kLegBaseColor, kLegBaseColor, TRUE);	//tibia 

		//�Ԑڂ̕`��
		DrawSphere3D(kCoxaJointPos, JOINT_R, SPHERE_DIV_NUM, kJointColor, kJointColor, TRUE);
		DrawSphere3D(kFemurJointPos, JOINT_R, SPHERE_DIV_NUM, kJointColor, kJointColor, TRUE);
		DrawSphere3D(kTibiaJointPos, JOINT_R, SPHERE_DIV_NUM, kJointColor, COLOR_LIFTED_JOINT, TRUE);

		//�r��̕`��
		DrawSphere3D(kLegEndPos, JOINT_R / 2, SPHERE_DIV_NUM, kJointColor, kJointColor, TRUE);

		//�r�̃x�[�X���W�̕`��
		DrawSphere3D(kLegBasePos, JOINT_R / 3, SPHERE_DIV_NUM, COLOR_LEG_BASE, COLOR_LEG_BASE, TRUE);


		//�G���[�o��
		if (isAbleCoxaLeg(m_HexaCalc.getGlobalCoxaJointPos(_node, i, true), m_HexaCalc.getGlobalFemurJointPos(_node, i)) == false)
		{
			DrawString((int)ConvWorldPosToScreenPos(kCoxaJointPos).x, (int)ConvWorldPosToScreenPos(kCoxaJointPos).y, "Error:Coxa", GetColor(255, 64, 64));
		}

		if (isAbleFemurLeg(m_HexaCalc.getGlobalFemurJointPos(_node, i), m_HexaCalc.getGlobalTibiaJointPos(_node, i)) == false)
		{
			DrawString((int)ConvWorldPosToScreenPos(kFemurJointPos).x, (int)ConvWorldPosToScreenPos(kFemurJointPos).y, "Error:Femur", GetColor(64, 255, 64));
		}

		if (isAbleTibiaLeg(m_HexaCalc.getGlobalTibiaJointPos(_node, i), m_HexaCalc.getGlobalLegPos(_node, i, true)) == false)
		{
			DrawString((int)ConvWorldPosToScreenPos(kFemurJointPos).x, (int)ConvWorldPosToScreenPos(kFemurJointPos).y, "Error:Tibia", GetColor(64, 64, 255));
		}

		if (DO_OUTPUT_DEBUG_LOG)
		{
			if (m_HexaCalc.isLegInRange(_node, i))printfDx("�Z");
			else printfDx("�~");

			printfDx(" LegNum: %d \t", i);
			printfDx("Max : %.3f, min : %.3f\t", m_HexaCalc.getMaxLegR(_node.leg_pos[i].z), m_HexaCalc.getMinLegR(_node.leg_pos[i].z));
			printfDx("%.3f\t", _node.leg_pos[i].length());

			if (m_HexaCalc.isLegInRange(_node, i))printfDx("is in range   ");
			else printfDx("isnot in range");

			if (_node.leg_base_pos[i].projectedXY().cross(_node.leg_pos[i].projectedXY()) * _node.leg_pos[i].projectedXY().cross({ 1,0 }) > 0)
			{
				printfDx("front - 567\n");
			}
			else
			{
				printfDx("back - 123\n");
			}
		}
	}

	if (DO_OUTPUT_DEBUG_LOG)
	{
		//is Able Pause
		if (m_HexaCalc.isAblePause(_node)) { printfDx("�Z is Able Pause\n"); }
		else { printfDx("�~ isnot Able Pause\n"); }

		// leg Interfering
		if (m_HexaCalc.isLegInterfering(_node)) { printfDx("�~ is Leg Interfering\n"); }
		else { printfDx("�Z isnot Leg Interfering\n"); }
	}
}
