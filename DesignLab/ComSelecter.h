#pragma once
#include "MyVector.h"
#include "MyPolygon.h"
#include "ComType.h"
#include "Node.h"
#include "HexapodStateCalculator.h"

//! @brief �d�S�����߂�N���X�D�g������̃v���O�����ɂ�����CCC�̏������s���Ă���D
class ComSelecter final
{
private:

	const int DISCRETIZATION_NUM = 10; // �d�S�����߂�ۂ̕�����

	const float STABILITY_MARGIN = 10.0f; // ��Έ��S�]�T

	const bool DO_DEBUG_PRINT = false; // �f�o�b�O�p�̏o�͂��s�����ǂ����D�e�X�g�R�[�h���������������ۉ��ł��Ă��Ȃ�...

	SNode m_current_node; //!< ���݂̃m�[�h

	HexapodStateCalclator m_calclator; //!< ��Ԍv�Z�N���X

public:

	//! @brief ���݂̃m�[�h��ݒ肷��
	//! @param [in] _current_node ���݂̃m�[�h
	void setCurrentNode(const SNode& _current_node) { m_current_node = _current_node; } //!< ���݂̃m�[�h��ݒ肷��

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
	bool getComFromPolygon(const my_vec::SPolygon2& polygon, const ComType::EComPattern _com_pattren, my_vec::SVector& _output_com) const;

private:

	//! @brief ���n�_�𐶐�����
	void makeComCandidatePoint(const my_vec::SPolygon2& polygon, std::vector<my_vec::SVector2>& _output_coms) const;

	//! @brief ��Έ��S�]�T���v�Z���C�}�[�W�����O��Ă��Ȃ������ׂ�
	bool isInMargin(const my_vec::SPolygon2& polygon, const my_vec::SVector2& _com) const;
};
