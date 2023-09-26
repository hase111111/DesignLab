#include "designlab_quaternion.h"

#include "cassert_define.h"


constexpr designlab::EulerXYZ designlab::Quaternion::toRotator() const
{
	// �N�I�[�^�j�I����XYZ�I�C���[�p�ɕϊ�

	return { 0,0,0 };
}

designlab::Quaternion designlab::Quaternion::MakeByAngleAxis(float angle, const Vector3& axis)
{
	// �I�C���[�p���N�I�[�^�j�I���ɕϊ�

	const float kHalfAngle = angle * 0.5f;

	return Quaternion{ cosf(kHalfAngle) , Vector3{ axis.x ,axis.y,axis.z }.Normalize() * sinf(kHalfAngle) };
}

designlab::Vector3 designlab::RotateVector3(const designlab::Vector3& vec, const designlab::Quaternion& q,const bool use_normalized_quaternions)
{
	designlab::Quaternion p{0, vec.x, vec.y, vec.z};	// ��]������x�N�g�����X�J���[��0�̃N�I�[�^�j�I���ɕϊ�
	designlab::Quaternion res;	// ����

	if (use_normalized_quaternions)
	{
		// ���K�����ꂽ�N�H�[�^�j�I�����g���ꍇ�́C�����Ƌt�����������̂ŁC�v�Z�ʂ����炷���Ƃ��ł���

		assert(designlab::math_util::IsEqual(q.Norm(), 1.0f));	// ���K�����ꂽ�N�H�[�^�j�I�����g���ꍇ�́C���K�����ꂽ�N�H�[�^�j�I����n���K�v������
		
		res = q * p * q.Conjugate();	// Q * P * Q^-1 
	}
	else 
	{
		res = q * p * q.Inverse();	// Q * P * Q^-1
	}

	return res.v;	// �x�N�g��������Ԃ�
}