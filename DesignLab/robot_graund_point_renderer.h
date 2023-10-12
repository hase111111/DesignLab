//! @file robot_graund_point_renderer.h
//! @brief ���{�b�g�̋r�ڒn�_�̍��W��`�悷��N���X

#ifndef DESIGNLAB_ROBOT_GRAUND_POINT_RENDERER_H
#define DESIGNLAB_ROBOT_GRAUND_POINT_RENDERER_H


#include <array>
#include <vector>

#include "abstract_hexapod_state_calculator.h"
#include "designlab_vector3.h"
#include "hexapod_const.h"
#include "robot_state_node.h"


//! @class RobotGraundPointRenderer
//! @brief ���{�b�g���ڒn�_�����n�_�̗�����`�悷��N���X
class RobotGraundPointRenderer final
{
public:

	RobotGraundPointRenderer(const std::shared_ptr<const AbstractHexapodStateCalculator> calclator_ptr);


	//! ���{�b�g���ڒn�_�����n�_�̗������Z�b�g����D
	//! @n �܂��C�V�~�����[�V�������I�������m�[�h�̃C���f�b�N�X���Z�b�g����D
	//! @param [in] result_node ���{�b�g���ڒn�������W��vector
	//! @param [in] simu_end_node_index �V�~�����[�V�������I�������m�[�h�̃C���f�b�N�X��vector
	void SetNodeAndSimulationEndNodeIndex(const std::vector<RobotStateNode>& result_node, const std::vector<size_t>& simu_end_node_index);


	//! ���{�b�g���ڒn�_�����n�_�̗����̕`����s���D
	//! @param [in] draw_simu_num �`����s���V�~�����[�V�����̔ԍ�( 0, 1, 2, ...)
	//! @param [in] draw_all_simulation ��̃p�����[�^�𖳎����āC���ׂẴV�~�����[�V�����ɂ��ĕ`�悷��
	void Draw(size_t draw_simu_num, bool draw_all_simulation = false) const;

private:

	struct VectorAndIsGround
	{
		designlab::Vector3 vec;	//!< ���W

		bool is_ground;			//!< �ڒn���Ă��邩�ǂ���
	};


	const unsigned int kRightLegGraundPointColor;		//!< �r�ڒn�_�̐F (�E��)

	const unsigned int kLeftLegGraundPointColor;		//!< �r�ڒn�_�̐F (����)

	const unsigned int kRightLegGraundPointDarkColor;	//!< ���݂̃V�~�����[�V�����ȊO�̐F (�E��)

	const unsigned int kLeftLegGraundPointDarkColor;	//!< ���݂̃V�~�����[�V�����ȊO�̐F (����)


	const std::shared_ptr<const AbstractHexapodStateCalculator> calclator_ptr_;	//!< ���{�b�g�̍��W�v�Z�N���X�D

	size_t loaded_node_num_;	//!< �ǂݍ��񂾃m�[�h�̐��C�f�[�^���X�V����邽�і���S���Ǎ��Ȃ����Ȃ��悤�ɁC�ǂݍ��񂾃m�[�h�̐����L�����Ă���

	//!< ���{�b�g�̋r�ڒn�_�̍��W�Cgraund_point[�V�~�����[�V�����ԍ�][�m�[�h�ԍ�][�r�ԍ�]�̏��ŃA�N�Z�X����
	std::vector<std::vector<std::array<VectorAndIsGround, HexapodConst::kLegNum>>> graund_point_;
};


#endif // !DESIGNLAB_ROBOT_GRAUND_POINT_RENDERER_H