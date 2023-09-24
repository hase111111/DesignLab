#include "designlab_quaternion.h"


designlab::Vector3 designlab::rotVecByQuat(const designlab::Vector3& vec, const Quaternion& q)
{
	designlab::Quaternion p{0, vec.x, vec.y, vec.z};
	designlab::Quaternion qInv = q.Inverse();
	designlab::Quaternion r = q * p * qInv;

	return Vector3{ r.v.x, r.v.y, r.v.z };
}

constexpr designlab::EulerXYZ designlab::Quaternion::toRotator() const
{
	// �N�I�[�^�j�I����XYZ�I�C���[�p�ɕϊ�

	return { 0,0,0 };
}

designlab::Quaternion designlab::Quaternion::MakeByRotAngleAndAxis(float angle, const Vector3& axis)
{
	// �I�C���[�p���N�I�[�^�j�I���ɕϊ�

	const float kHalfAngle = angle * 0.5f;

	return Quaternion{ cosf(kHalfAngle) , Vector3{ axis.x ,axis.y,axis.z } * sinf(kHalfAngle) }.Normalize();
}