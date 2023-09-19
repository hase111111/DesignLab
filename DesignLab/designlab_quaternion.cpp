#include "designlab_quaternion.h"


dl_vec::SVector dl_vec::rotVecByQuat(const dl_vec::SVector& vec, const SQuaternion& q)
{
	dl_vec::SQuaternion p{0, vec.x, vec.y, vec.z};
	dl_vec::SQuaternion qInv = q.inverse();
	dl_vec::SQuaternion r = q * p * qInv;

	return SVector{ r.v.x, r.v.y, r.v.z };
}

constexpr dl_vec::SRotator dl_vec::SQuaternion::toRotator() const
{
	// �N�I�[�^�j�I����XYZ�I�C���[�p�ɕϊ�

	return { 0,0,0 };
}

void dl_vec::SQuaternion::setRotAngleAndAxis(float angle, const SVector& axis)
{
	// �I�C���[�p���N�I�[�^�j�I���ɕϊ�

	const float kHalfAngle = angle * 0.5f;
	const float kSinHalfAngle = sinf(kHalfAngle);

	v.x = axis.x * kSinHalfAngle;
	v.y = axis.y * kSinHalfAngle;
	v.z = axis.z * kSinHalfAngle;
	w = cosf(kHalfAngle);

	*this = normalize();
}
