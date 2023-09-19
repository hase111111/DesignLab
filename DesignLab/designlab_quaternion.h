#pragma once

#include <cmath>

#include "designlab_vector.h"
#include "designlab_rotator.h"


namespace dl_vec
{
	// https://zenn.dev/mebiusbox/books/132b654aa02124/viewer/2966c7
	// https://www.sports-sensing.com/brands/labss/motionmeasurement/motion_biomechanics/quaternion04.html#:~:text=%E3%82%AF%E3%82%A9%E3%83%BC%E3%82%BF%E3%83%8B%E3%82%AA%E3%83%B3%EF%BC%88quaternion%EF%BC%89%E3%81%AF%E5%9B%9B%E5%85%83,%E5%9B%9E%E8%BB%A2%E3%82%92%E8%A1%A8%E7%8F%BE%E3%81%97%E3%81%BE%E3%81%99%EF%BC%8E
	// https://www.f-sp.com/entry/2017/06/30/221124
	struct SQuaternion
	{
		constexpr SQuaternion() : w(1.0f), v(0.0f, 0.0f, 0.0f) {}
		constexpr SQuaternion(float w_, float x_, float y_, float z_) : w(w_), v(x_, y_, z_) {}
		constexpr SQuaternion(float w_, const SVector& v_) : w(w_), v(v_) {}
		constexpr SQuaternion(const SQuaternion& q) : w(q.w), v(q.v) {}

		constexpr SQuaternion operator + () const { return *this; }
		constexpr SQuaternion operator - () const { return { -w, -v }; }
		constexpr SQuaternion operator + (const SQuaternion& q) const { return { w + q.w, v + q.v }; }
		constexpr SQuaternion operator - (const SQuaternion& q) const { return { w - q.w, v - q.v }; }
		constexpr SQuaternion operator * (const SQuaternion& q) const { return { w * q.w - v.dot(q.v), w * q.v + q.w * v + v.cross(q.v) }; }
		constexpr SQuaternion operator * (float s) const { return { w * s, v * s }; }
		constexpr SQuaternion operator / (float s) const { return { w / s, v / s }; }


		//! @brief �N�H�[�^�j�I���̋�����Ԃ��D�����ȃN�H�[�^�j�I���Ƃ́C�x�N�g�������̕����𔽓]����������
		//! @n q = w + xi + yj + zk �Ƃ���ƁCq�̋����� w - xi - yj - zk �ƂȂ�D��]�͋t�����ɂȂ�
		//! @return dl_vec::SQuaternion �����N�H�[�^�j�I��
		constexpr SQuaternion conjugate() const { return { w, -v }; }

		//! @brief �N�H�[�^�j�I���̃m������Ԃ�
		//! @n �m�����Ƃ́C�x�N�g���̑傫���̂��ƁD�N�H�[�^�j�I���̃m�����́Cw^2 + x^2 + y^2 + z^2 �ŋ��߂���
		//! @return float �m����
		constexpr float norm() const { return w * w + v.dot(v); }

		//! @brief �N�H�[�^�j�I���̋t����Ԃ�
		//! @n �N�H�[�^�j�I��q�̋t��q^-1�́Cq�̋������m�����Ŋ���������
		constexpr SQuaternion inverse() const { return conjugate() * (1 / norm()); }

		//! @brief ���K�������N�H�[�^�j�I����Ԃ�
		//! @n �N�H�[�^�j�I���̐��K���Ƃ́C�m������1�ɂ��邱�ƁD
		//! @n �N�H�[�^�j�I��q�̐��K���́Cq / |q| �ŋ��߂���
		//! @return dl_vec::SQuaternion ���K�����ꂽ�N�H�[�^�j�I��
		inline SQuaternion normalize() const { return *this * (1 / sqrt(norm())); }

		//! @brief �N�H�[�^�j�I����XYZ�I�C���[�p�ɕϊ�����
		//! @return dl_vec::SRotator XYZ�I�C���[�p
		constexpr SRotator toRotator() const;

		//! @brief ���̃N�H�[�^�j�I���Ƃ̋�����2���Ԃ��D�N�H�[�^�j�I����4�����x�N�g���Ƃ݂Ȃ��C�x�N�g���̋�����2������߂�
		//! @return float ������2��
		constexpr float distanceSquared(const SQuaternion& q) const { return (*this - q).norm(); }

		//! @brief ��]���Ɖ�]�p���炱�̃N�H�[�^�j�I����ύX���C�ݒ肷��
		//! @n q = cos(��/2) * w + sin(��/2) * { x  + y  + z } �ƂȂ�
		//! @param [in] rad_angle ��]�p�i���W�A���j
		//! @param [in] axis ��]��
		void setRotAngleAndAxis(float rad_angle, const SVector& axis);


		float w;	//!< �X�J���[����
		SVector v;	//!< �x�N�g������

	};


	constexpr SQuaternion operator * (float s, const SQuaternion& q) { return q * s; }

	SVector rotVecByQuat(const SVector& vec, const SQuaternion& q);

}	// namespace dl_vec	


//! @file designlab_quaternion.h
//! @date 2023/08/21
//! @author ���J��
//! @brief �N�H�[�^�j�I����\������\����
//! @n �s�� : @lineinfo
