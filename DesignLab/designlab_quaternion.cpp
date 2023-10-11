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

	return Quaternion{ cosf(kHalfAngle) , Vector3{ axis.x ,axis.y,axis.z }.GetNormalized() * sinf(kHalfAngle) };
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

designlab::Quaternion designlab::SlerpQuaternion(const Quaternion& q1, const Quaternion& q2, const float t)
{
	if(q1 == q2) { return q1; }	// �N�H�[�^�j�I�����������ꍇ�́Cq1��Ԃ�

	// ���ʐ��`��Ԃ��s��
	float dot = q1.Dot(q2);		// ����
	float theta = acosf(dot);	// �p�x
	if (math_util::IsEqual(theta, 0.f)) { return q1; }	// �p�x��0�̏ꍇ�́Cq1��Ԃ�

	float sin_theta = sinf(theta);			// sin��
	float sin_theta_inv = 1 / sin_theta;	// 1 / sin��

	float sin_t_theta = sinf(t * theta);			// sin(t��)
	float sin_1_t_theta = sinf((1 - t) * theta);	// sin((1-t)��)

	return sin_1_t_theta * sin_theta_inv * q1 + sin_t_theta * sin_theta_inv * q2;	// ��Ԃ��ꂽ�N�H�[�^�j�I����Ԃ�
}
