#pragma once

#include <vector>
#include <memory>

#include "node.h"
#include "designlab_polygon.h"
#include "com_type.h"
#include "abstract_hexapod_state_calculator.h"


//! @class ComCandidatePolygonMaker
//! @date 2023/08/12
//! @author ���J��
//! @brief �d�S�ʒu�̌��n�_���������p�`���쐬����N���X
//! @details ���݂̃��{�b�g�̏�Ԃ�\���m�[�h����C�d�S�ʒu�̌��n�_���������p�`���쐬����
//! @n ��@�͔g������̑��ƌ������Q�l�ɂ��Ă��邽�߁C�ڍׂ͂�������Q�Ƃ̂���
//! @n ���R�����C���̎�@�ł̓��{�b�g�̎p���ύX���l�����Ă��Ȃ��̂ł��̃N���X���g�p����ꍇ�́C
//! @n ���{�b�g�̉�]�E����͍s�����Ƃ��ł��Ȃ�
class ComCandidatePolygonMaker final
{
public:

	ComCandidatePolygonMaker(std::shared_ptr<AbstractHexapodStateCalculator> calc) : mp_calculator(calc) {};


	static constexpr int MAKE_POLYGON_NUM = 7;	//!< �쐬���鑽�p�`�̐�


	//! @brief ���݂̃��{�b�g�̏�Ԃ�\���m�[�h����C�d�S�ʒu�̌��n�_���������p�`���쐬����
	//! @param [in] node ���݂̃��{�b�g�̏�Ԃ�\���m�[�h
	//! @param [out] output_poly �d�S�ʒu�̌��n�_���������p�`
	void makeCandidatePolygon(const SNode& node, std::pair<dl_vec::SPolygon2, ComType::EComPattern> output_poly[MAKE_POLYGON_NUM]) const;


private:

	//! @brief �d�S�ʒu�̌��n�_���������p�`���쐬����D���S����̐}�`��4�p�`��5�p�`��p���ĕ\������D
	void makeCandidateBox(const dl_vec::SVector2 leg_pos[HexapodConst::LEG_NUM], const int start_leg_num, dl_vec::SPolygon2* output_poly) const;


	//! @brief �d�S�ʒu�̌��n�_���������p�`���쐬����D���S����̐}�`��3�p�`��p���ĕ\������D
	void makeCandidateTriangle(const dl_vec::SVector2 leg_pos[HexapodConst::LEG_NUM], dl_vec::SPolygon2* output_poly, ComType::EComPattern* output_com_pattern) const;


	//! @brief ���������p�`����������Ă��邩���m�F����
	//! @param [in] _poly �m�F���鑽�p�`
	//! @return ���������p�`����������Ă��邩
	bool checkPolygon(const dl_vec::SPolygon2& poly) const;


	static constexpr bool DO_DEBUG_PRINT = false;	// �f�o�b�O�p�ɏo�͂��s���ꍇ��true�ɂ���D�e�X�g�R�[�h�������������ǒ��ۉ��ł��Ă��Ȃ�...

	static constexpr bool DO_CHECK_POLYGON = true;	// ���p�`�̃`�F�b�N���s���ꍇ��true�ɂ���D�d���̂�false�ɂ��������C�[��5�܂łȂ���Ȃ��D


	std::shared_ptr<AbstractHexapodStateCalculator> mp_calculator;	//!< ���{�b�g�̏�Ԃ��v�Z����N���X

};


//! @file com_candidate_polygon_maker.h
//! @date 2023/08/12
//!	@author ���J��
//! @brief �d�S�ʒu�̌��n�_���������p�`���쐬����N���X
//! @n �s�� : @lineinfo
