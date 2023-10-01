//! @file designlab_quaternion.h
//! @brief �N�H�[�^�j�I����\���\����


#ifndef DESIGNLAB_QUATERNION_H_
#define DESIGNLAB_QUATERNION_H_


#include <cmath>

#include "cassert_define.h"
#include "designlab_euler.h"
#include "designlab_vector3.h"


//����define��L���ɂ���ƁC�m������1�łȂ��N�H�[�^�j�I���������Ȃ�
#define DESIGNLAB_QUATERNION_ASSERT_NORM_1


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
		//! @brief 1 + {0,0,0}�ŏ���������C
		constexpr Quaternion() : w(1.0f), v(0.0f, 0.0f, 0.0f) {}

		//! @brief �X�J���[�����ƃx�N�g���������w�肵�ď���������D�m������1�ɂȂ�悤�ɑ�����邱�ƁC
		//! @n �g�p�͔񐄏��CMakeByAngleAxis���g������
		constexpr Quaternion(const float w_, const float x_, const float y_, const float z_) : w(w_), v(x_, y_, z_) {}

		//! @brief �X�J���[�����ƃx�N�g���������w�肵�ď���������D�m������1�ɂȂ�悤�ɑ�����邱�ƁC
		//! @n �g�p�͔񐄏��CMakeByAngleAxis���g������
		constexpr Quaternion(const float w_, const ::designlab::Vector3& v_) : w(w_), v(v_)	{}

		constexpr Quaternion(const Quaternion& q) = default;
		constexpr Quaternion(Quaternion&& q) noexcept = default;
		constexpr Quaternion& operator =(const Quaternion& q) = default;

		constexpr Quaternion operator + () const { return *this; }
		constexpr Quaternion operator - () const { return { -w, -v }; }
		constexpr Quaternion operator + (const Quaternion& q) const { return { w + q.w, v + q.v }; }
		constexpr Quaternion operator - (const Quaternion& q) const { return { w - q.w, v - q.v }; }
		constexpr Quaternion operator * (const Quaternion& q) const { return { w * q.w - v.Dot(q.v), w * q.v + q.w * v + v.Cross(q.v) }; }
		constexpr Quaternion operator * (const float s) const { return { w * s, v * s }; }
		constexpr Quaternion operator / (const float s) const { return { w / s, v / s }; }

		bool operator == (const Quaternion& q) const { return (w == q.w) && (v == q.v); }
		bool operator != (const Quaternion& q) const { return !(*this == q); }

		bool operator < (const Quaternion& q) const { return (w < q.w) || (w == q.w && v < q.v); }
		bool operator > (const Quaternion& other) const { return other < *this; }
		bool operator <= (const Quaternion& other) const { return !(*this > other); }
		bool operator >= (const Quaternion& other) const { return !(*this < other); }


		//! @brief �N�H�[�^�j�I���̓��ς�Ԃ��D�N�H�[�^�j�I����4�����̃x�N�g���Ƃ݂Ȃ��C�x�N�g���̓��ς����߂�
		//! @param [in] other ���ς����߂�N�H�[�^�j�I��
		//! @return float ����
		constexpr float Dot(Quaternion other) const { return w * other.w + v.Dot(other.v); }

		//! @brief �N�H�[�^�j�I���̋�����Ԃ��D�����ȃN�H�[�^�j�I���Ƃ́C�x�N�g�������̕����𔽓]����������
		//! @n q = w + xi + yj + zk �Ƃ���ƁCq�̋����� w - xi - yj - zk �ƂȂ�D��]�͋t�����ɂȂ�
		//! @return designlab::Quaternion �����N�H�[�^�j�I��
		constexpr Quaternion Conjugate() const { return { w, -v }; }

		//! @brief �N�H�[�^�j�I���̒�����2���Ԃ�(�m������2��)�D
		//! @n �N�H�[�^�j�I���̒�����2��́Cw^2 + x^2 + y^2 + z^2 �ŋ��߂���
		//! @return float ������2��
		constexpr float LengthSquared() const { return w * w + v.Dot(v); }

		//! @brief �N�H�[�^�j�I���̃m������Ԃ�
		//! @n �m�����Ƃ́C�x�N�g���̑傫���̂��ƁD�N�H�[�^�j�I���̃m�����́Cw^2 + x^2 + y^2 + z^2 �ŋ��߂���
		//! @return float �m����
		inline float Norm() const { return std::sqrt(LengthSquared()); }

		//! @brief �N�H�[�^�j�I���̋t����Ԃ�
		//! @n �N�H�[�^�j�I��q�̋t��q^-1�́Cq�̋������m�����Ŋ���������
		constexpr Quaternion Inverse() const { return Conjugate() * (1 / Norm()); }

		//! @brief ���K�������N�H�[�^�j�I����Ԃ�
		//! @n �N�H�[�^�j�I���̐��K���Ƃ́C�m������1�ɂ��邱�ƁD
		//! @n �N�H�[�^�j�I��q�̐��K���́Cq / |q| �ŋ��߂���
		//! @return designlab::Quaternion ���K�����ꂽ�N�H�[�^�j�I��
		inline Quaternion Normalize() const { return *this * (1 / Norm()); }

		//! @brief �N�H�[�^�j�I����XYZ�I�C���[�p�ɕϊ�����
		//! @return designlab::EulerXYZ XYZ�I�C���[�p
		constexpr EulerXYZ toRotator() const;

		//! @brief ���̃N�H�[�^�j�I���Ƃ̋�����2���Ԃ��D�N�H�[�^�j�I����4�����x�N�g���Ƃ݂Ȃ��C�x�N�g���̋�����2������߂�
		//! @return float ������2��
		constexpr float DistanceSquared(const Quaternion& q) const { return (*this - q).LengthSquared(); }

		//! @brief ��]���Ɖ�]�p����N�H�[�^�j�I�����쐬����
		//! @n q = cos(��/2) * w + sin(��/2) * { v.x  + v.y  + v.z } �ƂȂ�
		//! @param [in] rad_angle ��]�p�� [rad]
		//! @param [in] axis ��]��
		static Quaternion MakeByAngleAxis(float rad_angle, const Vector3& axis);


		float w;	//!< �X�J���[����
		::designlab::Vector3 v;	//!< �x�N�g������

	};


	constexpr Quaternion operator * (float s, const Quaternion& q) { return q * s; }

	//! @brief 3�����̈ʒu�x�N�g������]������D
	//! @param [in] vec ��]������x�N�g���D
	//! @param [in] q ��]������N�H�[�^�j�I���D
	//! @param [in] use_normalized_quaternions ���K�����ꂽ�N�H�[�^�j�I�����g�����ǂ����D
	//! @n ���K���N�H�[�^�j�I���Ȃ�΁C�����Ƌt�����������̂ŁC�v�Z�ʂ����炷���Ƃ��ł���D
	//! @return designlab::Vector3 ��]��̃x�N�g���D
	Vector3 RotateVector3(const Vector3& vec, const Quaternion& q, bool use_normalized_quaternions);

	//! @brief ���ʐ��`��Ԃ��s���D
	//! @param [in] q1 �N�H�[�^�j�I��1
	//! @param [in] q2 �N�H�[�^�j�I��2
	//! @param [in] t ��ԌW���D0�`1�̒l�����D
	//! @return designlab::Quaternion ��Ԃ��ꂽ�N�H�[�^�j�I��
	Quaternion SlerpQuaternion(const Quaternion& q1, const Quaternion& q2, float t);

}	// namespace designlab	


// define��undef����܂ŗL���Ȃ̂ŁCundef����(���̃t�@�C���ɉe�����Ȃ��悤�ɂ��邽�߁C)
#undef DESIGNLAB_QUATERNION_ASSERT_NORM_1


#endif	// DESIGNLAB_QUATERNION_H_