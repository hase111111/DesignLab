#include "HexapodRenderer.h"
#include "DxLib.h"
#include "Dxlib3DFunction.h"
#include "LegState.h"

HexapodRenderer::HexapodRenderer() :
	COLOR_BODY(GetColor(23, 58, 235)), COLOR_LEG(GetColor(23, 58, 235)), COLOR_LIFTED_LEG(GetColor(240, 30, 60)), 
	COLOR_JOINT(GetColor(100, 100, 200)), COLOR_LIFTED_JOINT(GetColor(200, 100, 100)), CAPSULE_DIV_NUM(6), SPHERE_DIV_NUM(16)
{
}

void HexapodRenderer::update(const SNode& _node)
{
	m_HexaCalc.calclateJointPos(_node);
}

void HexapodRenderer::draw(const SNode& _node) const
{
	//旧verのhexapodGraphicの記述にしたがって描画する．
	using namespace myDxlib3DFunc;
	using namespace LegState;

	//胴体を描画する．
	VECTOR _vertex[6];
	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		//_vertex[i] = convertToDxVec(_hexapod.getGlobalCoxaJointPos(i));
		_vertex[i] = convertToDxVec( m_HexaCalc.getGlobalCoxaJointPos(_node, i) );
	}

	drawHexagonalPrism(_vertex, HexapodConst::BODY_HEIGHT, COLOR_BODY);

	//脚を描画する．
	for (int i = 0; i < HexapodConst::LEG_NUM; i++)
	{
		const VECTOR _leg_end = convertToDxVec( m_HexaCalc.getGlobalLegPos(_node, i) );
		const VECTOR _coxa = convertToDxVec( m_HexaCalc.getGlobalCoxaJointPos(_node, i) );
		const VECTOR _femur = convertToDxVec( m_HexaCalc.getGlobalFemurJointPos(_node, i) );
		const VECTOR _tibia = convertToDxVec( m_HexaCalc.getGlobalTibiaJointPos(_node, i) );

		const unsigned int _color = isGrounded(_node.leg_state, i) ? COLOR_LEG : COLOR_LIFTED_LEG;
		const unsigned int _joint_color = isGrounded(_node.leg_state, i) ? COLOR_JOINT : COLOR_LIFTED_JOINT;

		//各脚の描画
		DrawCapsule3D(_coxa,	_femur, LEG_R, CAPSULE_DIV_NUM, _color, _color, TRUE);	//coxa
		DrawCapsule3D(_femur,	_tibia, LEG_R, CAPSULE_DIV_NUM, _color, _color, TRUE);	//femur
		DrawCone3D(_leg_end,	_tibia, LEG_R, CAPSULE_DIV_NUM, _color, _color, TRUE);	//tibia 

		//間接の描画
		DrawSphere3D(_coxa,	 JOINT_R, SPHERE_DIV_NUM, _joint_color, _joint_color, TRUE);
		DrawSphere3D(_femur, JOINT_R, SPHERE_DIV_NUM, _joint_color, _joint_color, TRUE);
		DrawSphere3D(_tibia, JOINT_R, SPHERE_DIV_NUM, _joint_color, COLOR_LIFTED_JOINT, TRUE);
	}
}
