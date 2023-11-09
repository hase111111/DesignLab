//! @file interface_hexapod_coordinate_converter.h
//! @brief ���{�b�g�͏d�S�ʒu�E�A�N�`���G�[�^�ȂǗl�X�ȓ_����Ƃ�����W�n�����D�����𑊌݂ɕϊ�����N���X�D


#ifndef INTERFACE_HEXAPOD_COORDINATE_CONVERTER_H
#define INTERFACE_HEXAPOD_COORDINATE_CONVERTER_H


#include "designlab_vector3.h"
#include "designlab_euler.h"


//! @class IHexapodCoordinateConverter
//! @brief ���{�b�g�͏d�S�ʒu�E�A�N�`���G�[�^�ȂǗl�X�ȓ_����Ƃ�����W�n�����D�����𑊌݂ɕϊ�����N���X�D
//! @details �v���O�����ɂ�3�̍��W�n�����݂��Ă���C
//! @n
//! @n [1] �O���[�o�����W�n (Global Coordinate)
//! @n		�}�b�v�̌��_�����_�Ƃ�����W�n�C���W���̓}�b�v�̍��W���Ɠ����D
//! @n
//! @n [2] ���{�b�g���W�n (Robot Coordinate)
//! @n		���{�b�g�̏d�S�����_�Ƃ�����W�n�C���W���̓��{�b�g�̎p���ɍ��킹��D
//! @n 
//! @n [3] �r���W�n (Leg Coordinate)
//! @n		�r�̕t���������_�Ƃ�����W�n�C���W���̓��{�b�g�̎p���ɍ��킹��(���[�J�����W�n�Ɠ���)�D
//! @n		�^���w�̋��ȏ��Ƃ��ǂފ����C���̍��W�n�̎����Ƃ͈Ⴄ�̂����C��s�����Ŏg��ꂽ��@�ł���C
//! @n		���p����Ȃ��̂ł��̂܂܁D
class IHexapodCoordinateConverter
{
public:
	virtual ~IHexapodCoordinateConverter() = default;	//�p������N���X�̓f�X�g���N�^��virtual�ɂ��邱�ƁD

	//! @brief �O���[�o�����W�n�ŕ\������Ă�����W���C�r���W�n�ɕϊ�����D
	//! @param [in] converted_position �ϊ��ΏہD�O���[�o�����W�n�D
	//! @param [in] leg_index �r�ԍ��D
	//! @param [in] center_of_mass_global ���{�b�g�̏d�S�̍��W�D�O���[�o�����W�n�D
	//! @param [in] robot_rot ���{�b�g�̎p���D�p�x��rad.
	//! @param [in] consider_rot ���{�b�g�̎p�����l�����邩�ǂ����Dfalse�Ȃ��]���l�����Ȃ��D
	//! @return designlab::Vector3 �r���W�n�̍��W�D�r����W�n�Ƃ͋r�̕t���������_�Ƃ��C���̓��{�b�g���W�n�Ɠ��l�ȍ��W�n�D
	virtual designlab::Vector3 ConvertGlobalToLegCoordinate(const designlab::Vector3& converted_position, int leg_index,
		const designlab::Vector3& center_of_mass_global, const designlab::EulerXYZ& robot_rot, bool consider_rot) const = 0;

	//! @brief �r���W�n�ŕ\������Ă�����W���C�O���[�o�����W�n�ɕϊ�����D
	//! @param [in] converted_position �ϊ��ΏہD�r���W�n�D
	//! @param [in] leg_index �r�ԍ��D
	//! @param [in] center_of_mass_global ���{�b�g�̏d�S�̍��W�D�O���[�o�����W�n�D
	//! @param [in] robot_rot ���{�b�g�̎p���D�p�x��rad.
	//! @param [in] consider_rot ���{�b�g�̎p�����l�����邩�ǂ����Dfalse�Ȃ��]���l�����Ȃ��D
	//! @return designlab::Vector3 �O���[�o�����W�n�̍��W�D
	virtual designlab::Vector3 ConvertLegToGlobalCoordinate(const designlab::Vector3& converted_position, int leg_index,
		const designlab::Vector3& center_of_mass_global, const designlab::EulerXYZ& robot_rot, bool consider_rot) const = 0;

	//! @brief ���{�b�g���W�n�ŕ\������Ă�����W���C�O���[�o�����W�n�ɕϊ�����D
	//! @param [in] converted_position �ϊ��ΏہD���{�b�g���W�n�D
	//! @param [in] center_of_mass_global ���{�b�g�̏d�S�̍��W�D�O���[�o�����W�n�D
	//! @param [in] robot_rot ���{�b�g�̎p���D�p�x��rad.
	//! @param [in] consider_rot ���{�b�g�̎p�����l�����邩�ǂ����Dfalse�Ȃ��]���l�����Ȃ��D
	//! @return designlab::Vector3 �O���[�o�����W�n�̍��W�D
	virtual designlab::Vector3 ConvertRobotToGlobalCoordinate(const designlab::Vector3& converted_position,
		const designlab::Vector3& center_of_mass_global, const designlab::EulerXYZ& robot_rot, bool consider_rot) const = 0;
};


#endif	// INTERFACE_HEXAPOD_COORDINATE_CONVERTER_H