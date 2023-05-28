#pragma once
#include "Node.h"
#include "HexapodStateCalculator.h"

// ���{�b�g�̕`����s���N���X�D
class HexapodRenderer
{
private:
	const unsigned int COLOR_BODY;			//���̂̐F
	const unsigned int COLOR_LEG;			//�r�̐F
	const unsigned int COLOR_LIFTED_LEG;	//�V�r���Ă���r�̐F
	const unsigned int COLOR_JOINT;			//�W���C���g�̐F
	const unsigned int COLOR_LIFTED_JOINT;	//�V�r���Ă���W���C���g�̐F

	const int CAPSULE_DIV_NUM;				//���{�b�g�̃��f���̉~�����ǂꂾ���ׂ����`�悷�邩�D4 �` 20���炢�����傤�ǂ悢�Ǝv���D
	const int SPHERE_DIV_NUM;				//���{�b�g�̃��f���̋����ǂꂾ���ׂ����`�悷�邩�D16 �` 32���炢�����傤�ǂ悢�Ǝv���D
	const float LEG_R = 10.0f;				//�r�̔��a�D���̃N���X�ł͋r���~���ɋߎ����ĕ`�悵�Ă���D�`�掞�̃f�[�^�̂��߁C�����ω������Ă��V�~�����[�V�����ɉe���͂Ȃ��D
	const float JOINT_R = 20.0f;			//�W���C���g�̔��a�D�`�掞�̃f�[�^�̂��߁C�����ω������Ă��V�~�����[�V�����ɉe���͂Ȃ��D

	HexapodStateCalclator m_HexaCalc;		//���{�b�g�̎p������W���v�Z����D

public:
	HexapodRenderer();
	~HexapodRenderer() = default;

	//���{�b�g�̏�Ԃ��X�V����D
	void update(const SNode& _node);

	//���{�b�g��3D��Ԃɕ`�悷��D
	void draw(const SNode& _node) const;

};
