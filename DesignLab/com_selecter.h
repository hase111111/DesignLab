//#pragma once
//
//#include "designlab_vector3.h"
//#include "designlab_polygon2.h"
//#include "com_type.h"
//#include "robot_state_node.h"
//#include "hexapod_state_calculator.h"
//
//
////! @class ComSelecter
////! @brief �d�S�����߂�N���X�D�g������̃v���O�����ɂ�����CCC�̏������s���Ă���D
//class ComSelecter final
//{
//
//public:
//
//	//! @brief ���݂̃m�[�h��ݒ肷��
//	//! @param [in] _current_node ���݂̃m�[�h
//	void SetCurrentNode(const RobotStateNode& _current_node) { m_current_node = _current_node; } //!< ���݂̃m�[�h��ݒ肷��
//
//	//! @brief �d�S�����߂�
//	//! @param [in] polygon �d�S�����߂�Ώۂ̃|���S���D���̒��ɓ���_���o�͂���D
//	//! @param [in] _com_pattren �d�S�̋��ߕ�
//	//! @param [out] _output_com �d�S
//	//! @return �d�S�����߂邱�Ƃ��ł������ǂ���
//	//! @details �d�S�ʒu�̌��ߕ��͔g������̃v���O�����ɏ������Ă���D <br>
//	//! �܂��́C���n�_�̑��p�`���͂ގl�p�`�𐶐��C���̒��ɓ��Ԋu�Ō��_��łD <br>
//	//! ���ɑ��p�`�̒��ɓ����Ă��Ȃ��_�����O����D <br>
//	//! �����āC��Έ��S�]�T���v�Z���C�}�[�W�����O�ꂽ�_�����O����D <br>
//	//! �܂��C�ړ���̍��W�ɂ����āC�r�����̋r�Ɗ�����ꍇ�͏��O����D <br>
//	//! �ȏ�̏������s������C�c�����_�̏d�S�����߂�D <br>
//	bool GetComFromPolygon(const designlab::Polygon2& polygon, const DiscreteComPos _com_pattren, designlab::Vector3& _output_com) const;
//
//private:
//
//	//! @brief ���n�_�𐶐�����
//	void MakeComCandidatePoint(const designlab::Polygon2& polygon, std::vector<designlab::Vector2>& _output_coms) const;
//
//	//! @brief ��Έ��S�]�T���v�Z���C�}�[�W�����O��Ă��Ȃ������ׂ�
//	bool IsInMargin(const designlab::Polygon2& polygon, const designlab::Vector2& _com) const;
//
//
//	const int kDiscretizationNum = 10; // �d�S�����߂�ۂ̕�����
//
//	const float kStabilityMargin = 10.0f; // ��Έ��S�]�T
//
//	const bool DO_DEBUG_PRINT = false; // �f�o�b�O�p�̏o�͂��s�����ǂ����D�e�X�g�R�[�h���������������ۉ��ł��Ă��Ȃ�...
//
//
//	RobotStateNode m_current_node; //!< ���݂̃m�[�h
//
//	HexapodStateCalclator_Old m_calclator; //!< ��Ԍv�Z�N���X
//};
//
//
////! @file com_selecter.h
////! @brief �d�S�����߂�N���X�D�g������̃v���O�����ɂ�����CCC�̏������s���Ă���D