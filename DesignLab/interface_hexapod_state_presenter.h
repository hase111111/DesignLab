//! @file interface_hexapod_state_presenter.h
//! @brief 6�r���{�b�g�̏�Ԃ�\������N���X


#ifndef DESIGNLAB_INTERFACE_HEXAPOD_STATE_PRESENTER_H_
#define DESIGNLAB_INTERFACE_HEXAPOD_STATE_PRESENTER_H_


#include "designlab_vector3.h"


//! @class IHexapodStatePresenter
//! @brief 6�r���{�b�g�̏�Ԃ�\������N���X
class IHexapodStatePresenter
{
public:

	virtual ~IHexapodStatePresenter() = default;

	//! @brief �V�r����ʒu��Ԃ��C�r���W�n�D
	//! @param [in] leg_index �r�ԍ��D
	//! @return designlab::Vector3 �V�r����ʒu�D�r���W�n�D
	[[nodiscard]] virtual designlab::Vector3 GetFreeLegPosLegCoodinate(int leg_index) const noexcept = 0;

	//! @brief �r�̕t�����̍��W( leg base position )���擾����D���{�b�g���W�n�D
	//! @param [in] leg_index �r�ԍ��D
	//! @return designlab::Vector3 �r�̕t�����̍��W�D���{�b�g���W�n�D
	[[nodiscard]] virtual designlab::Vector3 GetLegBasePosRobotCoodinate(int leg_index) const noexcept = 0;

	//! @brief �n�ʂ̍ő卂���Əd�S�ʒu���ŏ��ǂꂾ����������Ԃ�
	[[nodiscard]] virtual float GetGroundHeightMarginMin() const noexcept = 0;

	//! @brief �n�ʂ̍ő卂���Əd�S�ʒu���ő�ǂꂾ����������Ԃ�
	[[nodiscard]] virtual float GetGroundHeightMarginMax() const noexcept = 0;

};

#endif	// DESIGNLAB_INTERFACE_HEXAPOD_STATE_PRESENTER_H_