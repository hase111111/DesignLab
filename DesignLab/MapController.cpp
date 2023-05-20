#include "MapController.h"

void AreaDivide(const myvector::SVector& p1, const myvector::SVector& p2, int& x1, int& x2, int& y1, int& y2)
{
	//�r�ݒu�\�_�������Ă���̈�̑傳
	double _lengthX = (MapConst::MAP_X_MAX - MapConst::MAP_X_MIN) / (double)MapConst::LP_DIVIDE_NUM;
	double _lengthY = (MapConst::MAP_Y_MAX - MapConst::MAP_Y_MIN) / (double)MapConst::LP_DIVIDE_NUM;

	//�u���b�N�ԍ��v�Z
	x1 = (int)((p1.x - MapConst::MAP_X_MIN) / _lengthX);
	x2 = (int)((p2.x - MapConst::MAP_X_MIN) / _lengthX);
	y1 = (int)((p1.y - MapConst::MAP_Y_MIN) / _lengthY);
	y2 = (int)((p2.y - MapConst::MAP_Y_MIN) / _lengthY);

	if (x1 >= MapConst::LP_DIVIDE_NUM) { x1 = MapConst::LP_DIVIDE_NUM - 1; }//�z��O�͈̔͂̏ꍇ�͒[�����̃u���b�N�Ɏ��߂�悤�ɂ���
	if (x1 < 0) { x1 = 0; }
	if (y1 >= MapConst::LP_DIVIDE_NUM) { y1 = MapConst::LP_DIVIDE_NUM - 1; }
	if (y1 < 0) { y1 = 0; }
	if (x2 >= MapConst::LP_DIVIDE_NUM) { x2 = MapConst::LP_DIVIDE_NUM - 1; }	//�z��O�͈̔͂̏ꍇ�͒[�����̃u���b�N�Ɏ��߂�悤�ɂ���
	if (x2 < 0) { x2 = 0; }
	if (y2 >= MapConst::LP_DIVIDE_NUM) { y2 = MapConst::LP_DIVIDE_NUM - 1; }
	if (y2 < 0) { y2 = 0; }
}

void recalMap(myvector::SVector p_mapData3D[MapConst::MAPDATA3D_MAX], const SNode& _current_condition, const  SNode& _past_condition)
{
	for (int i = 0; i < MapConst::MAPDATA3D_MAX; i++)
	{
		//�ЂƂO�̃��{�b�g���W���O���[�o�����W�����݂̃��{�b�g���W
		p_mapData3D[i] = myvector::VRot(p_mapData3D[i], _past_condition.global_center_of_mass, _past_condition.pitch, _past_condition.roll, _past_condition.yaw);
		p_mapData3D[i] = p_mapData3D[i] + _past_condition.global_center_of_mass;
		p_mapData3D[i] = p_mapData3D[i] - _current_condition.global_center_of_mass;
		p_mapData3D[i] = myvector::VRot(p_mapData3D[i], _current_condition.global_center_of_mass, -_current_condition.pitch, -_current_condition.roll, -_current_condition.yaw);
	}
}
