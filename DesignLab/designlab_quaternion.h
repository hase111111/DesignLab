//! @file designlab_quaternion.h
//! @brief �N�H�[�^�j�I����\���\����


#ifndef DESIGNLAB_QUATERNION_H_
#define DESIGNLAB_QUATERNION_H_


#include <cmath>

#include "designlab_euler.h"
#include "designlab_vector3.h"


namespace designlab
{
	//! @struct designlab::Quaternion
	//! @brief �N�H�[�^�j�I����\���\����
	//! @details �N�H�[�^�j�I���́C�X�J���[�����ƃx�N�g����������Ȃ�D
	//! @n �l�����Ƃ��Ă΂��D
	//! @n �Q�l
	//! @n https://zenn.dev/mebiusbox/books/132b654aa02124/viewer/2966c7
	//! @n https://www.sports-sensing.com/brands/labss/motionmeasurement/motion_biomechanics/quaternion04.html#:~:text=%E3%82%AF%E3%82%A9%E3%83%BC%E3%82%BF%E3%83%8B%E3%82%AA%E3%83%B3%EF%BC%88quaternion%EF%BC%89%E3%81%AF%E5%9B%9B%E5%85%83,%E5%9B%9E%E8%BB%A2%E3%82%92%E8%A1%A8%E7%8F%BE%E3%81%97%E3%81%BE%E3%81%99%EF%BC%8E
	//! @n https://www.f-sp.com/entry/2017/06/30/221124
	struct Quaternion
	{
		constexpr Quaternion() : w(1.0f), v(0.0f, 0.0f, 0.0f) {}
		constexpr Quaternion(float w_, float x_, float y_, float z_) : w(w_), v(x_, y_, z_) {}
		constexpr Quaternion(float w_, const Vector3& v_) : w(w_), v(v_) {}

		constexpr Quaternion(const Quaternion& q) = default;
		constexpr Quaternion(Quaternion&& q) noexcept = default;
		constexpr Quaternion& operator =(const Quaternion& q) = default;

		constexpr Quaternion operator + () const { return *this; }
		constexpr Quaternion operator - () const { return { -w, -v }; }
		constexpr Quaternion operator + (const Quaternion& q) const { return { w + q.w, v + q.v }; }
		constexpr Quaternion operator - (const Quaternion& q) const { return { w - q.w, v - q.v }; }
		constexpr Quaternion operator * (const Quaternion& q) const { return { w * q.w - v.Dot(q.v), w * q.v + q.w * v + v.Cross(q.v) }; }
		constexpr Quaternion operator * (float s) const { return { w * s, v * s }; }
		constexpr Quaternion operator / (float s) const { return { w / s, v / s }; }


		//! @brief �N�H�[�^�j�I���̋�����Ԃ��D�����ȃN�H�[�^�j�I���Ƃ́C�x�N�g�������̕����𔽓]����������
		//! @n q = w + xi + yj + zk �Ƃ���ƁCq�̋����� w - xi - yj - zk �ƂȂ�D��]�͋t�����ɂȂ�
		//! @return designlab::Quaternion �����N�H�[�^�j�I��
		constexpr Quaternion Conjugate() const { return { w, -v }; }

		//! @brief �N�H�[�^�j�I���̃m������Ԃ�
		//! @n �m�����Ƃ́C�x�N�g���̑傫���̂��ƁD�N�H�[�^�j�I���̃m�����́Cw^2 + x^2 + y^2 + z^2 �ŋ��߂���
		//! @return float �m����
		constexpr float Norm() const { return w * w + v.Dot(v); }

		//! @brief �N�H�[�^�j�I���̋t����Ԃ�
		//! @n �N�H�[�^�j�I��q�̋t��q^-1�́Cq�̋������m�����Ŋ���������
		constexpr Quaternion Inverse() const { return Conjugate() * (1 / Norm()); }

		//! @brief ���K�������N�H�[�^�j�I����Ԃ�
		//! @n �N�H�[�^�j�I���̐��K���Ƃ́C�m������1�ɂ��邱�ƁD
		//! @n �N�H�[�^�j�I��q�̐��K���́Cq / |q| �ŋ��߂���
		//! @return designlab::Quaternion ���K�����ꂽ�N�H�[�^�j�I��
		inline Quaternion Normalize() const { return *this * (1 / sqrt(Norm())); }

		//! @brief �N�H�[�^�j�I����XYZ�I�C���[�p�ɕϊ�����
		//! @return designlab::EulerXYZ XYZ�I�C���[�p
		constexpr EulerXYZ toRotator() const;

		//! @brief ���̃N�H�[�^�j�I���Ƃ̋�����2���Ԃ��D�N�H�[�^�j�I����4�����x�N�g���Ƃ݂Ȃ��C�x�N�g���̋�����2������߂�
		//! @return float ������2��
		constexpr float DistanceSquared(const Quaternion& q) const { return (*this - q).Norm(); }

		//! @brief ��]���Ɖ�]�p����N�H�[�^�j�I�����쐬����
		//! @n q = cos(��/2) * w + sin(��/2) * { x  + y  + z } �ƂȂ�
		//! @param [in] rad_angle ��]�p�i���W�A���j
		//! @param [in] axis ��]��
		static Quaternion MakeByRotAngleAndAxis(float rad_angle, const Vector3& axis);


		float w;	//!< �X�J���[����
		Vector3 v;	//!< �x�N�g������

	};


	constexpr Quaternion operator * (float s, const Quaternion& q) { return q * s; }

	Vector3 rotVecByQuat(const Vector3& vec, const Quaternion& q);

}	// namespace designlab	


#endif	// DESIGNLAB_QUATERNION_H_