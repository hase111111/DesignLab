#pragma once

#include <vector>
#include <memory>

#include "designlab_vector3.h"
#include "designlab_polygon2.h"
#include "com_type.h"
#include "node.h"
#include "abstract_hexapod_state_calculator.h"



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

	ComSelecterHato(const std::shared_ptr<const AbstractHexapodStateCalculator>& calc) : mp_calculator(calc) {};


	//! @brief ���݂̃m�[�h��ݒ肷��
	//! @param [in] current_node ���݂̃m�[�h
	inline void setCurrentNode(const SNode& current_node) { m_current_node = current_node; } //!< ���݂̃m�[�h��ݒ肷��

	//! @brief �d�S�����߂�
	//! @param [in] polygon �d�S�����߂�Ώۂ̃|���S���D���̒��ɓ���_���o�͂���D
	//! @param [in] com_pattren �d�S�̋��ߕ�
	//! @param [out] output_com �d�S
	//! @return �d�S�����߂邱�Ƃ��ł������ǂ���
	bool getComFromPolygon(const designlab::Polygon2& polygon, designlab::Vector3* output_com) const;

private:

	static constexpr int DISCRETIZATION_NUM = 10; // �d�S�����߂�ۂ̕�����

	const float STABILITY_MARGIN = 10.0f; // ��Έ��S�]�T

	static constexpr bool DO_DEBUG_PRINT = false; // �f�o�b�O�p�̏o�͂��s�����ǂ����D�e�X�g�R�[�h���������������ۉ��ł��Ă��Ȃ�...



	SNode getCurrentNode() const { return m_current_node; } //!< ���݂̃m�[�h���擾����

	//! @brief ���n�_�𐶐�����
	bool makeComCandidatePoint(const designlab::Polygon2& polygon, std::pair<bool, designlab::SVector2> output_coms[DISCRETIZATION_NUM * DISCRETIZATION_NUM]) const;

	//! @brief ��Έ��S�]�T���v�Z���C�}�[�W�����O��Ă��Ȃ������ׂ�
	bool isInMargin(const designlab::Polygon2& polygon, const std::vector<designlab::SVector2>& edge_vec, const designlab::SVector2& candidate_point) const;



	SNode m_current_node; //!< ���݂̃m�[�h

	const std::shared_ptr<const AbstractHexapodStateCalculator> mp_calculator;	//!< ���{�b�g�̏�Ԃ��v�Z����N���X
};



//! @file com_selecter_hato.h
//! @date 2023/08/12
//! @author ���J��
//! @brief �d�S�����߂�N���X�D�g������̃v���O�����ɂ�����CCC�̏����Ɠ��l�̏������s���D
//! @n �s�� : @lineinfo

