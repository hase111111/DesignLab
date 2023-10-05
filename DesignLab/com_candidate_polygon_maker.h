//! @file com_candidate_polygon_maker.h
//! @brief �d�S�ʒu�̌��n�_���������p�`���쐬����N���X


#ifndef DESIGNLAB_COM_CANDIDATE_POLYGON_MAKER_H_
#define DESIGNLAB_COM_CANDIDATE_POLYGON_MAKER_H_


#include <array>
#include <memory>
#include <vector>

#include "abstract_hexapod_state_calculator.h"
#include "designlab_polygon2.h"
#include "discrete_com_pos.h"
#include "robot_state_node.h"


//! @struct ComPosAndPolygon
//! @brief ���U�����ꂽ�d�S�ʒu�Ƃ��̏d�S�ʒu���܂ޑ��p�`�̑g�ݍ��킹�D
struct ComPosAndPolygon
{
	ComPosAndPolygon() : com_pos(DiscreteComPos::kFront), polygon(), is_able(false) {}

	DiscreteComPos com_pos;			//!< ���U�����ꂽ�d�S�ʒu
	designlab::Polygon2 polygon;	//!< �d�S�ʒu���܂ޑ��p�`
	bool is_able;					//!< �d�S�ʒu���܂ޑ��p�`�����������ǂ���
};


//! @class ComCandidatePolygonMaker
//! @brief �d�S�ʒu�̌��n�_���������p�`���쐬����N���X
//! @details ���݂̃��{�b�g�̏�Ԃ�\���m�[�h����C�d�S�ʒu�̌��n�_���������p�`���쐬����
//! @n ��@�͔g������̑��ƌ������Q�l�ɂ��Ă��邽�߁C�ڍׂ͂�������Q�Ƃ̂���
//! @n ���R�����C���̎�@�ł̓��{�b�g�̎p���ύX���l�����Ă��Ȃ��̂ł��̃N���X���g�p����ꍇ�́C
//! @n ���{�b�g�̉�]�E����͍s�����Ƃ��ł��Ȃ�
class ComCandidatePolygonMaker final
{
public:

	ComCandidatePolygonMaker(const std::shared_ptr<const AbstractHexapodStateCalculator>& calc);


	static constexpr int MAKE_POLYGON_NUM = 7;	//!< �쐬���鑽�p�`�̐�


	//! @brief ���݂̃��{�b�g�̏�Ԃ�\���m�[�h����C�d�S�ʒu�̌��n�_���������p�`���쐬����
	//! @param [in] node ���݂̃��{�b�g�̏�Ԃ�\���m�[�h
	//! @param [out] output_poly �d�S�ʒu�̌��n�_���������p�`
	void MakeCandidatePolygon(const RobotStateNode& node, std::array<ComPosAndPolygon, MAKE_POLYGON_NUM>* output_poly) const;


private:

	//! @brief �d�S�ʒu�̌��n�_���������p�`���쐬����D���S����̐}�`��4�p�`��5�p�`��p���ĕ\������D
	void MakeCandidateBox(const std::array<designlab::Vector2, HexapodConst::LEG_NUM>& leg_pos, const int start_leg_num, designlab::Polygon2* output_poly) const;


	//! @brief �d�S�ʒu�̌��n�_���������p�`���쐬����D���S����̐}�`��3�p�`��p���ĕ\������D
	void MakeCandidateTriangle(const std::array<designlab::Vector2, HexapodConst::LEG_NUM>& leg_pos, ComPosAndPolygon* output) const;


	//! @brief ���������p�`����������Ă��邩���m�F����
	//! @param [in] _poly �m�F���鑽�p�`
	//! @return ���������p�`����������Ă��邩
	bool IsAblePolygon(const designlab::Polygon2& poly) const;


	static constexpr bool kDoCheckPolygon = true;	// ���p�`�̃`�F�b�N���s���ꍇ��true�ɂ���D�d���̂�false�ɂ��������C�[��5�܂łȂ���Ȃ��D


	const std::shared_ptr<const AbstractHexapodStateCalculator> calculator_ptr_;	//!< ���{�b�g�̏�Ԃ��v�Z����N���X

};


#endif	//DESIGNLAB_COM_CANDIDATE_POLYGON_MAKER_H_