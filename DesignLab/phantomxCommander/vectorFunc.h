#ifndef __VECTOR_H__
#define __VECTOR_H__
#include <iostream>
#include <Windows.h>
#include <utility>
#include<fstream>
#include <deque>
#include <time.h>
#include <string>
#include <vector>

namespace myvector{

// �x�N�g���f�[�^�^
typedef struct _tagVECTOR
{
	double				x, y, z ;
} VECTOR;

__inline VECTOR VGet( float x, float y, float z )
{
	VECTOR Result;
	Result.x = x ;
	Result.y = y ;
	Result.z = z ;
	return Result ;
}

// �x�N�g���̉��Z
__inline VECTOR		VAdd( const VECTOR &In1, const VECTOR &In2 )
{
	VECTOR Result ;
	Result.x = In1.x + In2.x ;
	Result.y = In1.y + In2.y ;
	Result.z = In1.z + In2.z ;
	return Result ;
}

// �x�N�g���̌��Z
__inline VECTOR		VSub( const VECTOR &In1, const VECTOR &In2 )
{
	VECTOR Result ;
	Result.x = In1.x - In2.x ;
	Result.y = In1.y - In2.y ;
	Result.z = In1.z - In2.z ;
	return Result ;
}
// �x�N�g���̓���
__inline float		VDot( const VECTOR &In1, const VECTOR &In2 )
{
	return In1.x * In2.x + In1.y * In2.y + In1.z * In2.z ;
}

// �x�N�g���̊O��
__inline VECTOR		VCross( const VECTOR &In1, const VECTOR &In2 )
{
	VECTOR Result ;
	Result.x = In1.y * In2.z - In1.z * In2.y ;
	Result.y = In1.z * In2.x - In1.x * In2.z ;
	Result.z = In1.x * In2.y - In1.y * In2.x ;
	return Result ;
}

// �x�N�g���̃X�P�[�����O
__inline VECTOR		VScale( const VECTOR &In, float Scale )
{
	VECTOR Result ;
	Result.x = In.x * Scale ;
	Result.y = In.y * Scale ;
	Result.z = In.z * Scale ;
	return Result ;
}

// �x�N�g���̃T�C�Y�̂Q��
__inline float		VSquareSize( const VECTOR &In )
{
	return In.x * In.x + In.y * In.y + In.z * In.z ;
}

// �x�N�g���̑傫��
__inline float VMag(const VECTOR &In1){
	return sqrt(VDot(In1, In1));
}

// �x�N�g���̑傫�� �ʒu�x�N�g������
__inline double VMag2(const VECTOR &In1, const VECTOR &In2){
	return VMag(VSub(In1, In2));
}

//�x�N�g���̏o��
void VectorOutPut(VECTOR v1, std::ofstream& fout);
void VectorOutPut(VECTOR v1);

//�x�N�g����3D���[�e�[�V���� �I�C���[�p�i�e�C�g�E�u���C�A���p�jY-X-Z
__inline VECTOR   VRot(myvector::VECTOR In, myvector::VECTOR center, float thP, float thR, float thY){

	myvector::VECTOR ans, buf;
	buf = myvector::VSub(In, center);
	ans.x = (cos(thR) * cos(thY) + sin(thR) * sin(thP) * sin(thY)) * buf.x +
			(cos(thY) * sin(thR) * sin(thP) - cos(thR) * sin(thY)) * buf.y +
			(cos(thP) * sin(thR)								 ) * buf.z;

	ans.y = cos(thP) * sin(thY) * buf.x +
		    cos(thP) * cos(thY) * buf.y +
			(- sin(thP))        * buf.z;

	ans.z = (cos(thR) * sin(thP) * sin(thY) - cos(thY) * sin(thR)) * buf.x +
		    (cos(thR) * cos(thY) * sin(thP) + sin(thR) * sin(thY)) * buf.y +
			(cos(thR) * cos(thP)								 ) * buf.z; 

	return myvector::VAdd(ans, center);
}


//��̊֐��Ɠ�������\�肾��������...
__inline VECTOR   VRot(const myvector::VECTOR &In, float thP, float thR,float thY )
{
	VECTOR ans;
	ans.y = (cos(thR) * cos(thY) + sin(thR) * sin(thP) * sin(thY)) * In.y +
			(cos(thY) * sin(thR) * sin(thP) - cos(thR) * sin(thY)) * In.x +
			(cos(thP) * sin(thR)								 ) * In.z;

	ans.x = cos(thP) * sin(thY) * In.y +
		    cos(thP) * cos(thY) * In.x +
			(- sin(thP))        * In.z;

	ans.z = (cos(thR) * sin(thP) * sin(thY) - cos(thY) * sin(thR)) * In.y +
		    (cos(thR) * cos(thY) * sin(thP) + sin(thR) * sin(thY)) * In.x +
			(cos(thR) * cos(thP)								 ) * In.z; 
	return ans;
}

//�x�N�g����3D�t���[�e�[�V���� ��̊֐��̋t�ϊ�
__inline VECTOR   VRot_R(myvector::VECTOR In, myvector::VECTOR center, float thP, float thR, float thY){

	myvector::VECTOR ans, buf;
	buf = myvector::VSub(In, center);
	ans.x = (cos(thY) * cos(thR) - sin(thY) * sin(thP) * sin(thR)) * buf.x +
			(-cos(thP) * sin(thY)								 ) * buf.y +
			(cos(thR) * sin(thY) * sin(thP) + cos(thY) * sin(thR)) * buf.z;

	ans.y = (cos(thY) * sin(thP) * sin(thR) + cos(thR) * sin(thY)) * buf.x +
		    (cos(thY) * cos(thP)								 ) * buf.y +
			(sin(thY) * sin(thR) - cos(thY) * cos(thR) * sin(thP)) * buf.z; 


	ans.z = -cos(thP) * sin(thR)	* buf.x +
		     sin(thP)				* buf.y +
			 cos(thP) * cos(thR)	* buf.z;
	return myvector::VAdd(ans, center);
}

}
#endif