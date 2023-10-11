//! @file com_selecter_hato.h
//! @brief �d�S�����߂�N���X�D�g������̃v���O�����ɂ�����CCC�̏����Ɠ��l�̏������s���D


#ifndef DESIGNLAB_COM_SELECTER_HATO_H_
#define DESIGNLAB_COM_SELECTER_HATO_H_

#include <vector>
#include <memory>

#include "abstract_hexapod_state_calculator.h"
#include "com_type.h"
#include "designlab_vector3.h"
#include "designlab_polygon2.h"
#include "robot_state_node.h"


//! @class ComSelecterHato
//! @brief �d�S�����߂�N���X�D�g������̃v���O�����ɂ�����CCC�̏����Ɠ��l�̏������s���D
//! @details �d�S�ʒu�̌��ߕ��͔g������̃v���O�����ɏ������Ă���D
//! @n �܂��́C���n�_�̑��p�`���͂ގl�p�`�𐶐��C���̒��ɓ��Ԋu�Ō��_��łD 
//! @n ���ɑ��p�`�̒��ɓ����Ă��Ȃ��_�����O����D 
//! @n �����āC��Έ��S�]�T���v�Z���C�}�[�W�����O�ꂽ�_�����O����D
//! @n �܂��C�ړ���̍��W�ɂ����āC�r�����̋r�Ɗ�����ꍇ�͏��O����D
//! @n �ȏ�̏������s������C�c�����_�̏d�S�����߂�D 
//! @note CCC�ł�Target�̒l�������Ă���̂ŁC���̒l�𗘗p���ďd�S�ʒu�I�����邪�C���̎����ł͂��̃N���X�ɂ��̑I����C�������Ȃ��D
//! ���l�̏������s�����߂ɁCTarget�̒l��K���Ɍ��߂Ă���D
class ComSelecterHato final
{
public:

	ComSelecterHato(const std::shared_ptr<const AbstractHexapodStateCalculator>& calc) : calculator_ptr_(calc) {};


	//! @brief ���݂̃m�[�h��ݒ肷��
	//! @param [in] current_node ���݂̃m�[�h
	inline void SetCurrentNode(const RobotStateNode& current_node) { current_node_ = current_node; } //!< ���݂̃m�[�h��ݒ肷��

	//! @brief �d�S�����߂�
	//! @param [in] polygon �d�S�����߂�Ώۂ̃|���S���D���̒��ɓ���_���o�͂���D
	//! @param [out] output_com �d�S
	//! @return �d�S�����߂邱�Ƃ��ł������ǂ���
	bool GetComFromPolygon(const designlab::Polygon2& polygon, designlab::Vector3* output_com) const;

private:

	static constexpr int kDiscretizationNum = 10; // �d�S�����߂�ۂ̕�����

	const float kStabilityMargin = 10.0f; // ��Έ��S�]�T


	RobotStateNode GetCurrentNode() const { return current_node_; } //!< ���݂̃m�[�h���擾����

	//! @brief ���n�_�𐶐�����
	bool MakeComCandidatePoint(const designlab::Polygon2& polygon, std::pair<bool, designlab::Vector2> output_coms[kDiscretizationNum * kDiscretizationNum]) const;

	//! @brief ��Έ��S�]�T���v�Z���C�}�[�W�����O��Ă��Ȃ������ׂ�
	bool IsInMargin(const designlab::Polygon2& polygon, const std::vector<designlab::Vector2>& edge_vec, const designlab::Vector2& candidate_point) const;



	RobotStateNode current_node_; //!< ���݂̃m�[�h

	const std::shared_ptr<const AbstractHexapodStateCalculator> calculator_ptr_;	//!< ���{�b�g�̏�Ԃ��v�Z����N���X
};


#endif