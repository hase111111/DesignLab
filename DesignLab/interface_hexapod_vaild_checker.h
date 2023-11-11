//! @file interface_hexapod_vaild_checker.h
//! @brief 6�r���{�b�g���L���Ȏp�����Ƃ��Ă��邩���`�F�b�N����N���X

#ifndef INTERFACE_HEXAPOD_VAILD_CHECKER_H
#define INTERFACE_HEXAPOD_VAILD_CHECKER_H

#include <array>

#include "designlab_vector3.h"
#include "devide_map_state.h"
#include "hexapod_const.h"
#include "leg_state.h"
#include "robot_state_node.h"


//! @class IHexapodVaildChecker
//! @brief 6�r���{�b�g���L���Ȏp�����Ƃ��Ă��邩���`�F�b�N����N���X
class IHexapodVaildChecker
{
public:
	virtual ~IHexapodVaildChecker() = default;


	//! @brief �r�����͈͓��ɂ��邩�ǂ����𔻒肷��D
	//! @param [in] leg_index �r�ԍ��D
	//! @param [in] leg_pos �r���W�n�ɂ�����r��̍��W�D�r����W�n�Ƃ͋r�̕t���������_�Ƃ��C���̓��{�b�g���W�n�Ɠ��l�ȍ��W�n�D
	//! @return bool �r�����͈͓��ɂ����true�D���͈͊O�ɂ����false�D
	virtual bool IsLegInRange(const int leg_index, const designlab::Vector3& leg_pos) const = 0;

	//! @brief �r�����̋r�Ɗ����Ă��邩�ǂ����𔻒肷��D
	//! @param [in] leg_pos �r���W�n�ɂ�����r��̍��W�̔z��D�r����W�n�Ƃ͋r�̕t���������_�Ƃ��C���̓��{�b�g���W�n�Ɠ��l�ȍ��W�n�D
	//! @return bool �r�����̋r�Ɗ����Ă����true�D�����Ă��Ȃ����false�D
	virtual bool IsLegInterfering(const std::array<designlab::Vector3, HexapodConst::kLegNum>& leg_pos) const = 0;

	//! @brief ����]�T(Stability Margin))���v�Z����D�ڂ����́u�s���n�ɂ�������s�@�B�̐ÓI���萫�]����v�Ƃ����_����ǂ�ŗ~����
	//! @n �ڒn�r���q���ō���鑽�p�`�̕ӂƏd�S�̋����̍ŏ��l���v�Z����D
	//! @param [in] leg_state �r�̏�ԁDbit�ŕ\�������C�V�r�E�ڒn�r�̏������D
	//! @param [in] leg_pos �r���W�n�ɂ�����r��̍��W�̔z��D�r����W�n�Ƃ͋r�̕t���������_�Ƃ��C���̓��{�b�g���W�n�Ɠ��l�ȍ��W�n�D
	//! @return float ����]�T�D�傫����������ƂȂ�C�܂����̒l��0�ȉ��Ȃ�]�|����D
	virtual float CalculateStabilityMargin(const ::designlab::leg_func::LegStateBit& leg_state,
		const std::array<designlab::Vector3, HexapodConst::kLegNum>& leg_pos) const = 0;

	//! @brief ����]�T��p���āC�ÓI�Ɉ��肵�Ă��邩�ǂ����𔻒肷��D
	//! @param [in] leg_state �r�̏�ԁDbit�ŕ\�������C�V�r�E�ڒn�r�̏������D
	//! @param [in] leg_pos �r���W�n�ɂ�����r��̍��W�̔z��D�r����W�n�Ƃ͋r�̕t���������_�Ƃ��C���̓��{�b�g���W�n�Ɠ��l�ȍ��W�n�D
	//! @return bool �ÓI�Ɉ��肵�Ă����true�D�����łȂ����false�D
	virtual bool IsStable(const ::designlab::leg_func::LegStateBit& leg_state,
		const std::array<designlab::Vector3, HexapodConst::kLegNum>& leg_pos) const = 0;

	//! @brief ���̂��n�ʂƊ����Ă��邩�ǂ����𔻒肷��D
	//! @param [in] node ���{�b�g�̏�ԁD
	//! @param [in] devide_map �n�ʂ̏�ԁD
	//! @return bool ���̂��n�ʂƊ����Ă����true�D�����Ă��Ȃ����false�D
	virtual bool IsBodyInterferingWithGround(const RobotStateNode& node, const DevideMapState& devide_map) const = 0;
};


#endif	// INTERFACE_HEXAPOD_VAILD_CHECKER_H
