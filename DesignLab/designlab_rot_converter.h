//! @file designlab_rot_converter.h
//! @brief ��]�p�s��C�I�C���[�p�C�N�H�[�^�j�I���̕ϊ����s���֐�

#ifndef DESIGNLAB_ROT_CONVERTER_H_
#define DESIGNLAB_ROT_CONVERTER_H_


#include "designlab_euler.h"
#include "designlab_quaternion.h"
#include "designlab_rotation_matrix.h"

namespace designlab 
{
	//! @brief ��]�p�s�񂩂�N�H�[�^�j�I���ւ̕ϊ�
	//! @param[in] rot ��]�p�s��
	//! @return �N�H�[�^�j�I��
	Quaternion ToQuaternion(const RotationMatrix3x3& rot);

	//! @brief �N�H�[�^�j�I������XYZ�I�C���[�p�ւ̕ϊ�
	//! @param[in] q �N�H�[�^�j�I��
	//! @return XYZ�I�C���[�p
	Quaternion ToQuaternion(const EulerXYZ& e);

	//! @brief �N�H�[�^�j�I�������]�p�s��ւ̕ϊ�
	//! @param[in] q �N�H�[�^�j�I��
	//! @return ��]�p�s��
	RotationMatrix3x3 ToRotationMatrix(const Quaternion& q);

	//! @brief XYZ�I�C���[�p�����]�p�s��ւ̕ϊ�
	//! @param[in] e XYZ�I�C���[�p
	//! @return ��]�p�s��
	RotationMatrix3x3 ToRotationMatrix(const EulerXYZ& e);

	//! @brief ��]�p�s�񂩂�XYZ�I�C���[�p�ւ̕ϊ�
	//! @param[in] rot ��]�p�s��
	//! @return XYZ�I�C���[�p
	EulerXYZ ToEulerXYZ(const RotationMatrix3x3& rot);

	//! @brief ��]�p�s�񂩂�N�H�[�^�j�I���ւ̕ϊ�
	//! @param[in] e XYZ�I�C���[�p
	//! @return �N�H�[�^�j�I��
	EulerXYZ ToEulerXYZ(const Quaternion& q);
}

#endif