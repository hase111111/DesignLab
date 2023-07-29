#pragma once
#include <vector>
#include <memory>
#include "MyVector.h"
#include "MyPolygon.h"
#include "ComType.h"
#include "Node.h"
#include "HexapodStateCalculator.h"

//! @brief �d�S�����߂�N���X�D�g������̃v���O�����ɂ�����CCC�̏����Ɠ��l�̏������s���D
//! @note CCC�ł�Target�̒l�������Ă���̂ŁC���̒l�𗘗p���ďd�S�ʒu�I�����邪�C���̎����ł͂��̃N���X�ɂ��̑I����C�������Ȃ��D
//! ���l�̏������s�����߂ɁCTarget�̒l��K���Ɍ��߂Ă���D
class ComSelecterHato final
{
private:

	static constexpr int DISCRETIZATION_NUM = 10; // �d�S�����߂�ۂ̕�����

	const float STABILITY_MARGIN = 10.0f; // ��Έ��S�]�T

	static constexpr bool DO_DEBUG_PRINT = false; // �f�o�b�O�p�̏o�͂��s�����ǂ����D�e�X�g�R�[�h���������������ۉ��ł��Ă��Ȃ�...

	SNode m_current_node; //!< ���݂̃m�[�h

	const HexapodStateCalclator m_calclator; //!< ��Ԍv�Z�N���X

public:

	//! @brief ���݂̃m�[�h��ݒ肷��
	//! @param [in] _current_node ���݂̃m�[�h
	inline void setCurrentNode(const SNode& _current_node) { m_current_node = _current_node; } //!< ���݂̃m�[�h��ݒ肷��

	//! @brief �d�S�����߂�
	//! @param [in] polygon �d�S�����߂�Ώۂ̃|���S���D���̒��ɓ���_���o�͂���D
	//! @param [in] _com_pattren �d�S�̋��ߕ�
	//! @param [out] _output_com �d�S
	//! @return �d�S�����߂邱�Ƃ��ł������ǂ���
	//! @details �d�S�ʒu�̌��ߕ��͔g������̃v���O�����ɏ������Ă���D <br>
	//! �܂��́C���n�_�̑��p�`���͂ގl�p�`�𐶐��C���̒��ɓ��Ԋu�Ō��_��łD <br>
	//! ���ɑ��p�`�̒��ɓ����Ă��Ȃ��_�����O����D <br>
	//! �����āC��Έ��S�]�T���v�Z���C�}�[�W�����O�ꂽ�_�����O����D <br>
	//! �܂��C�ړ���̍��W�ɂ����āC�r�����̋r�Ɗ�����ꍇ�͏��O����D <br>
	//! �ȏ�̏������s������C�c�����_�̏d�S�����߂�D <br>
	bool getComFromPolygon(const my_vec::SPolygon2& polygon, const ComType::EComPattern _com_pattren, my_vec::SVector* output_com) const;

private:

	SNode getCurrentNode() const { return m_current_node; } //!< ���݂̃m�[�h���擾����

	//! @brief ���n�_�𐶐�����
	bool makeComCandidatePoint(const my_vec::SPolygon2& polygon, std::pair<bool, my_vec::SVector2> output_coms[DISCRETIZATION_NUM * DISCRETIZATION_NUM]) const;

	//! @brief ��Έ��S�]�T���v�Z���C�}�[�W�����O��Ă��Ȃ������ׂ�
	bool isInMargin(const my_vec::SPolygon2& polygon, const std::vector<my_vec::SVector2>& edge_vec, const my_vec::SVector2& candidate_point) const;
};
