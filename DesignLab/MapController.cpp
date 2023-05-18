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

void MapSqrtDivide(const myvector::SVector mapData[MapConst::MAPDATA3D_MAX], std::vector< std::vector< std::vector<myvector::SVector> > >& divideMapData, int pointNum[MapConst::LP_DIVIDE_NUM][MapConst::LP_DIVIDE_NUM])
{
	// 0 �ŏ���������
	for (int i = 0; i < MapConst::LP_DIVIDE_NUM; ++i)
	{
		for (int j = 0; j < MapConst::LP_DIVIDE_NUM; ++j)
		{
			pointNum[i][j] = 0;	
		}
	}

	//�r�ݒu�\�_�������Ă���̈�̑傳
	const double _lengthX = (MapConst::MAP_X_MAX - MapConst::MAP_X_MIN) / (double)MapConst::LP_DIVIDE_NUM;
	const double _lengthY = (MapConst::MAP_Y_MAX - MapConst::MAP_Y_MIN) / (double)MapConst::LP_DIVIDE_NUM;

	int x, y, t;
	for (int i = 0; i < MapConst::MAPDATA3D_MAX; ++i)
	{
		//�����ȃf�[�^�Ȃ�Ώ������΂�
		if (mapData[i].x == MapConst::INVALID_FOOT_HOLD) { continue; }

		//xy�����̃u���b�N�ԍ������ꂼ�ꋁ�߂�
		x = (int)((mapData[i].x - MapConst::MAP_X_MIN) / _lengthX);		
		y = (int)((mapData[i].y - MapConst::MAP_Y_MIN) / _lengthY);		//y�����̃u���b�N�ԍ�

		if (x >= MapConst::LP_DIVIDE_NUM) { x = MapConst::LP_DIVIDE_NUM - 1; }	//�z��O�͈̔͂̏ꍇ�͒[�����̃u���b�N�Ɏ��߂�悤�ɂ���
		if (x < 0) { x = 0; }
		if (y >= MapConst::LP_DIVIDE_NUM) { y = MapConst::LP_DIVIDE_NUM - 1; }
		if (y < 0) { y = 0; }

		t = pointNum[x][y];

		if (divideMapData[x][y].size() <= pointNum[x][y])
		{
			//����Ȃ��Ȃ����猻�݂̔{�T�C�Y�m�ہ@�g���܂킷�Ƃ���swap�ŏ���������iresize����capacity�͕ς��Ȃ����߁j
			divideMapData[x][y].resize(divideMapData[x][y].size() * 2);
		}

		divideMapData[x][y][t] = mapData[i];
		++pointNum[x][y];
	}
}


void GetRandom(int mapDataNum, int min, int max, int* Random)
{
	srand((unsigned int)time(NULL));

	for (int i = 0; i < mapDataNum; i++)
	{
		Random[i] = min + (int)(rand() * (max - min + 1.0) / (1.0 + RAND_MAX));
	}
}

void SetConditionForStripe(LNODE& node)
{
	double COM_Z = 130;
	int random_r[MapConst::START_RANDOM_R];
	int random_theta[360];
	int random_l[1000];
	GetRandom(MapConst::START_RANDOM_R, 0, MapConst::START_RANDOM_R, random_r);
	GetRandom(360, 0, 360, random_theta);
	GetRandom(1000, 0, 1000, random_l);

	for (int i = 0; i < 1000; ++i)
	{
		if (random_l[i] > 500) { random_l[i] -= 1000; }
	}

	//�d�S�ʒu�O���[�o��
	node.global_center_of_mass = myvector::VGet(random_l[1] + random_r[1] * cos(random_theta[1] * Define::MY_PI / 180), random_r[1] * sin(random_theta[1] * Define::MY_PI / 180), COM_Z);

	node.Leg[0] = myvector::VGet(120, 100, -COM_Z);//�t��������r��̈ʒu
	node.Leg[1] = myvector::VGet(130, 0, -COM_Z);
	node.Leg[2] = myvector::VGet(120, -100, -COM_Z);
	node.Leg[3] = myvector::VGet(-120, -100, -COM_Z);
	node.Leg[4] = myvector::VGet(-130, 0, -COM_Z);
	node.Leg[5] = myvector::VGet(-120, 100, -COM_Z);
	//�r�̈ʒu(z�����Œ�)
	node.Leg2[0] = myvector::VGet(120, 100, -COM_Z);
	node.Leg2[1] = myvector::VGet(130, 0, -COM_Z);
	node.Leg2[2] = myvector::VGet(120, -100, -COM_Z);
	node.Leg2[3] = myvector::VGet(-120, -100, -COM_Z);
	node.Leg2[4] = myvector::VGet(-130, 0, -COM_Z);
	node.Leg2[5] = myvector::VGet(-120, 100, -COM_Z);

	//�p���e�C�g�u���C�A���p�O���[�o��
	node.pitch = 0.0;		//x����]
	node.roll = 0.0;		//y����]
	node.yaw = 0.0;			//z����]
	node.center_of_mass = 0;//�d�S�ʒuint
	node.leg_state = 0b00000000110011001100110011001100;
	node.parent = NULL;		//�e�m�[�h�̃|�C���^
	node.node_height = 1;	//�m�[�h����
	node.debug = 24;		//���݉^�������Ƃ��Ďg�p,�O��̋r�㉺�m�[�h(�㉺�^���������ꍇ)2��,�O��̓���1��,�O�X��̓���1��
	node.delta_comz = 0;
}

void recalMap(myvector::SVector* p_mapData3D, int mapData3D_MAX, LNODE* CurrentCondition, LNODE* PastCondition)
{
	for (int i = 0; i < mapData3D_MAX; i++)
	{
		//�ЂƂO�̃��{�b�g���W���O���[�o�����W�����݂̃��{�b�g���W
		p_mapData3D[i] = myvector::VRot(p_mapData3D[i], PastCondition->global_center_of_mass, PastCondition->pitch, PastCondition->roll, PastCondition->yaw);
		p_mapData3D[i] = p_mapData3D[i] + PastCondition->global_center_of_mass;
		p_mapData3D[i] = p_mapData3D[i] - CurrentCondition->global_center_of_mass;
		p_mapData3D[i] = myvector::VRot(p_mapData3D[i], CurrentCondition->global_center_of_mass, -CurrentCondition->pitch, -CurrentCondition->roll, -CurrentCondition->yaw);
	}
}
