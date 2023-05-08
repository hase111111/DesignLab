#pragma once


// hexapod.cpp�̃������c���Ă����D
// �����ɂ͂��܂�ɂ���񂪑���



//#include "pch.h"
//#include "hexapod.h"
//
//
//Hexapod::Hexapod(void)
//{
//	//m_local_coxajoint_pos
//	m_local_coxajoint_pos[0] = myvector::VGet(FWIDTH, FLENGTH, 0);
//	m_local_coxajoint_pos[1] = myvector::VGet(MWIDTH, 0, 0);
//	m_local_coxajoint_pos[2] = myvector::VGet(RWIDTH, -RLENGTH, 0);
//	m_local_coxajoint_pos[3] = myvector::VGet(-RWIDTH, -RLENGTH, 0);
//	m_local_coxajoint_pos[4] = myvector::VGet(-MWIDTH, 0, 0);
//	m_local_coxajoint_pos[5] = myvector::VGet(-FWIDTH, FLENGTH, 0);
//
//	//m_local_coxajoint_pos[0]=VGet(  FLENGTH,   FWIDTH, 0);
//	//m_local_coxajoint_pos[1]=VGet(        0,   MWIDTH, 0);
//	//m_local_coxajoint_pos[2]=VGet(- RLENGTH,   RWIDTH, 0);
//	//m_local_coxajoint_pos[3]=VGet(- RLENGTH, - RWIDTH, 0);
//	//m_local_coxajoint_pos[4]=VGet(        0, - MWIDTH, 0);
//	//m_local_coxajoint_pos[5]=VGet(  FLENGTH, - FWIDTH, 0);
//
//}
//
//
////�x�N�g����3D���[�e�[�V���� �I�C���[�p�i�e�C�g�E�u���C�A���p�jY-X-Z
//// 2018.12.27 �R�����g��Y-X-Z���ĊԈႢ���ۂ��D�p���wikipedia�ɏڂ������C����Y-X-Y������̃I�C���[�p�̒�`���Ǝv����ł����c�c�i�e�C�g�E�u���C�A�����Ⴄ�̂ł́j
///*2020.05.15.hato
//�@����́ARPY����{�I�ɓ��̂̌X���i�ʂɓ��̂���Ȃ��Ă��������ǁj������A���̍��W�n��coxa���W�n������W��P�i�Ⴆ��coxa����݂��r����W�Ȃǁj��
//�O���[�o�����W�Ō������W��0P�ɂւ񂩂񂷂邽�߂́A
//	��0_P = �i��0_R_��)��P + (�O���[�o�����W�n�Ō������̍��W�n�̌��_���W�j
//���Ă������́i��0_R_��)��P�̌v�Z�����Ă�֐��B
//
//�i��0R��)�͉�]�s��Łi��0R��)=Ry*Rx*Rz
//Ry,Rx,Rz�͓Y�����̎����ɉ�]�������Ƃ��̉�]�s�������킷�B
//��ʓI�ɂ́A�i�s�����ɂ������Ƃ邩��A
//Rx�����[���̊֐���Rx(thR), Ry���s�b�`�̊֐�=Ry(thP),Pz �����[�̊֐�=Rz(thY)
//�ɂȂ�B
//�����A���̃V�~�����[�V�����ł́A�i�s������y��������A
//Rx���s�b�`�̊֐���Rx(thR), Ry�����[���̊֐�=Ry(thP),Pz �����[�̊֐�=Rz(thY)
//�ɂȂ��Ă���B
//�܂��A��0R���@���@R��P��Y�̏��ɉ�]�s����v�Z����̂���ʓI�H������A����ɂ��킹��ƁA
//���ʂ́A��0R���@���@Rx(R) * Ry(P) * Rz(Y)�@�̏��ԂŌv�Z����Ƃ���
//�����ł́A��0R���@���@Ry(R) * Rx(P) * Rz(Y)�@�̏��ԂŌv�Z���Ă���B
//������������Ƃ�₱�����Ƃ��낾���ǁA�v�Z���̂͊Ԉ���ĂȂ��B
//*/
//myvector::SVector Hexapod::rotation(myvector::SVector In, myvector::SVector center, double thP, double thR, double thY) {
//	myvector::SVector ans, buf;
//	buf = subVec(In, center);
//	ans.x = (cos(thR) * cos(thY) + sin(thR) * sin(thP) * sin(thY)) * buf.x +
//		(cos(thY) * sin(thR) * sin(thP) - cos(thR) * sin(thY)) * buf.y +
//		(cos(thP) * sin(thR)) * buf.z;
//
//	ans.y = cos(thP) * sin(thY) * buf.x +
//		cos(thP) * cos(thY) * buf.y +
//		(-sin(thP)) * buf.z;
//
//	ans.z = (cos(thR) * sin(thP) * sin(thY) - cos(thY) * sin(thR)) * buf.x +
//		(cos(thR) * cos(thY) * sin(thP) + sin(thR) * sin(thY)) * buf.y +
//		(cos(thR) * cos(thP)) * buf.z;
//
//	return VAdd(ans, center);
//}
//
//myvector::SVector Hexapod::getNormalVector()
//{
//	myvector::SVector ret;
//	//�e�v�f�́A��̊֐���z�̌W��
//	ret.x = cos(this->ziki.thP) * sin(this->ziki.thR);
//	ret.y = -sin(this->ziki.thP);
//	ret.z = cos(this->ziki.thR) * cos(this->ziki.thP);
//	return ret;
//}
//
//void Hexapod::setMyDirection(const double _thP, const double _thR, const double _thY)
//{
//	ziki.thP = _thP;
//	ziki.thR = _thR;
//	ziki.thY = _thY;
//}
//
//void Hexapod::setPosition_of_2(const myvector::SVector _pos[Define::LEG_NUM])
//{
//	for (int i = 0; i < Define::LEG_NUM; i++)
//	{
//		L_Position_of_2[i] = _pos[i];
//	}
//}
//
//void Hexapod::setMyLegPosition(const myvector::SVector _pos[Define::LEG_NUM])
//{
//	for (int i = 0; i < Define::LEG_NUM; i++)
//	{
//		L_Leg_Position[i] = _pos[i];
//	}
//}
//
//
//myvector::SVector Hexapod::getLocalPosition_of_2(int legNum) {	//���[�J�����W��Ԃ�
//	return this->L_Position_of_2[legNum];
//}
//
//myvector::SVector Hexapod::getPosition_of_2(int legNum) {	//�O���[�o�����W��Ԃ�
//	myvector::SVector ans, rotatePosition_of_2;
//	rotatePosition_of_2 = rotation(L_Position_of_2[legNum], myvector::VGet(0, 0, 0), this->ziki.thP, this->ziki.thR, this->ziki.thY);
//	ans = VAdd(rotatePosition_of_2, getGlobalCoxaJointPos(legNum));
//	return ans;
//}
//
//myvector::SVector Hexapod::getGlobalLegPos(int legNum) {	//�O���[�o�����W��Ԃ�
//	myvector::SVector ans, rotateLegPosition;
//	rotateLegPosition = rotation(L_Leg_Position[legNum], myvector::VGet(0, 0, 0), this->ziki.thP, this->ziki.thR, this->ziki.thY);
//	ans = VAdd(rotateLegPosition, getGlobalCoxaJointPos(legNum));//�d�S����r�̕t����+�t��������r��
//	return ans;
//}
//
//
//myvector::SVector Hexapod::getGlobalCoxaJointPos(int legNum) {	//�O���[�o�����W��Ԃ�(�d�S�ʒu����r�̕t�����̌v�Z)
//	myvector::SVector ans, rotateCoxaJointPosi;
//	rotateCoxaJointPosi = rotation(this->m_local_coxajoint_pos[legNum], myvector::VGet(0, 0, 0), this->ziki.thP, this->ziki.thR, this->ziki.thY);
//	ans = VAdd(rotateCoxaJointPosi, this->ziki.com);
//	return ans;
//}
//
//
//myvector::SVector Hexapod::getGlobalFemurJointPos(int legNum) {	//�O���[�o�����W��Ԃ�
//	myvector::SVector ans, rotateFemurJointPosi;
//	rotateFemurJointPosi = rotation(this->m_local_femurjoint_pos[legNum], myvector::VGet(0, 0, 0), this->ziki.thP, this->ziki.thR, this->ziki.thY);
//	ans = VAdd(rotateFemurJointPosi, this->ziki.com);
//	return ans;
//}
//
//
//
//myvector::SVector Hexapod::getGlobalTibiaJointPos(int legNum) {	//�O���[�o�����W��Ԃ�
//	myvector::SVector ans, rotateTibiaJointPosi;
//	rotateTibiaJointPosi = rotation(this->L_TibiaJoint_posi[legNum], myvector::VGet(0, 0, 0), this->ziki.thP, this->ziki.thR, this->ziki.thY);
//	ans = VAdd(rotateTibiaJointPosi, this->ziki.com);
//	return ans;
//}
//
//myvector::SVector Hexapod::getLocalFemurJointPos(int legNum) {
//	return this->m_local_femurjoint_pos[legNum];
//}
//
//
//myvector::SVector Hexapod::getLocalCoxaJointPos(int legNum) {
//	return m_local_coxajoint_pos[legNum];
//}
//
//
//
////���͈͂Ƌr�̐ڒn�_����  �r�̐ڒn�_�����͈͓��Ȃ��1 �O��������0��Ԃ� 	�t�^���w�C�^���w�̌v�Z����Z�p������� ���{�b�g�ŗL
//bool Hexapod::setJointPos() {
//	// �t�^���w�̌v�Z���ʂ�p���ď��^���w���v�Z����
//
//	const double PERMISSION = 1;			//�t�^���w�Ɖ^���w���s�������ʂ����aPermission^0.5�̉~�̒��Ȃ瓙�����ƍl����
//
//	const double mins[3] = { -1.428, -1.780, -1.194 };  //�r���͈�
//	const double maxs[3] = { 1.402,  1.744,  1.769 };  //������coxa,femur,tibia
//	const double coxaMins[6] = { -0.610, -1.428, -2.213, -0.960, -1.745, -2.531 };
//	const double coxaMaxs[6] = { 2.187,   1.402,  0.617,  2.531,  1.745,  0.960 };
//
//
//	const double femurMins = -1.780;
//	const double femurMaxs = 1.744;
//	const double tibiaMins = -1.194;
//	const double tibiaMaxs = 1.769;
//
//	double coxa, femur, tibia;
//	myvector::SVector kinematics;					//ans of kinematics use sorution of i_kinematics
//	myvector::SVector legposi;
//
//	for (int legNum = 0; legNum < 6; legNum++) {
//		legposi = myvector::VGet(L_Leg_Position[legNum].x, L_Leg_Position[legNum].y, -L_Leg_Position[legNum].z);
//
//		//std::cout<<"L_Leg_PositionNum["<<legNum<<"] ="<<legposi.x<<"\t"<<legposi.y<<"\t"<<legposi.z<<"\n";
//		//�t�^���w
//		// first, make this a 2DOF problem... by solving coxa
//		coxa = atan2(legposi.y, legposi.x);//coxa�p�x
//		double IK_trueX = sqrt(pow(fabs(legposi.x), 2.0) + pow(fabs(legposi.y), 2.0)) - L_COXA;//xy���ʂɂ�����femur�r��܂ł̋���
//		double im = sqrt(pow(fabs(IK_trueX), 2.0) + pow(fabs(legposi.z), 2.0));//femur����r��܂ł̋���
//
//		// get femur angle above horizon...
//		double q1 = -atan2(legposi.z, IK_trueX);//femur����r��ւ̃x�N�g����xy���ʂƂ̊p�x
//		double d1 = pow((double)L_FEMUR, 2.0) - pow((double)L_TIBIA, 2.0) + pow(fabs((double)im), 2.0);
//		double d2 = 2 * L_FEMUR * im;
//		double q2 = acos((double)d1 / (double)d2);
//		femur = q1 + q2;
//
//		// and tibia angle from femur...
//		d1 = pow((double)L_FEMUR, 2.0) - pow(fabs((double)im), 2.0) + pow((double)L_TIBIA, 2.0);
//		d2 = 2 * L_TIBIA * L_FEMUR;
//		tibia = acos((double)d1 / (double)d2) - 1.57;
//
//		//lange of motion
//		//if(legNum<3)if(coxaMaxs[legNum] < coxa  || coxa < coxaMins[legNum]){/*std::cout<<"Error1 lange of motion\n coxa["<<legNum<<"] = "<<coxa<<"\n";myvector::VectorOutPut(legposi);*/return 1;}
//		//if(legNum>2)if(coxaMins[legNum] < coxa  && coxa < coxaMaxs[legNum]){/*std::cout<<"Error2 lange of motion\n coxa["<<legNum<<"] = "<<coxa<<"\n";myvector::VectorOutPut(legposi);*/return 1;}
//		if (femurMaxs < femur || femur < femurMins) {/*std::cout<<"Error3 lange of motion femur["<<legNum<<"] = "<<femurMins<<"\n";myvector::VectorOutPut(legposi);return 1;*/ }
//		if (tibiaMaxs < tibia || tibia < tibiaMins) {/*std::cout<<"Error4 lange of motion tibia["<<legNum<<"] = "<<tibiaMins<<"\n";myvector::VectorOutPut(legposi);return 1;*/ }
//
//		//�^���w
//		double K_trueX;
//
//		K_trueX = L_FEMUR * cos(femur) + L_TIBIA * cos(femur + tibia - 1.57);
//		kinematics.x = cos(coxa) * (K_trueX + L_COXA);
//		kinematics.y = sin(coxa) * (K_trueX + L_COXA);
//		kinematics.z = -(L_FEMUR * sin(femur) + L_TIBIA * sin(femur + tibia - 1.57));
//
//
//		m_local_femurjoint_pos[legNum] = myvector::VAdd(m_local_coxajoint_pos[legNum], myvector::VGet(L_COXA * cos(coxa), L_COXA * sin(coxa), 0));
//		L_TibiaJoint_posi[legNum] = myvector::VAdd(m_local_femurjoint_pos[legNum], myvector::VGet(L_FEMUR * cos(femur) * cos(coxa), L_FEMUR * cos(femur) * sin(coxa), L_FEMUR * sin(femur)));
//
//		//m_local_femurjoint_pos[legNum] = m_local_coxajoint_pos[legNum];
//		//L_TibiaJoint_posi[legNum] = m_local_femurjoint_pos[legNum];
//
//
//
//		double Permission = VSquareSize(subVec(kinematics, legposi));
//
//		if (PERMISSION < Permission) { std::cout << "Error PERMISSION OVER \n"; return 1; };
//	}
//	return 0;
//}
//
//
//
//
////���͈͂Ƌr�̐ڒn�_����  �r�̐ڒn�_�����͈͓��Ȃ��1 �O��������0��Ԃ� 	��`�ł���Ƃ������肩��Z�p������� ���{�b�g�ŗL LineEnd�͑������ʂɉ�]�����ĂȂ�����O���[�o��(coxa���W�n�Ō����Ƃ�)��coxa����ڒn�_�܂ł̃x�N�g��
////��؂���̂�ł����g���ĂȂ�
//bool Hexapod::check_touchdown_point(int legNum, const myvector::SVector& LineEnd)
//{
//	//if (int(LineEnd.z) < MIN_DELTAZ || MAX_DELTAZ < int(LineEnd.z)) return 0;
//
//	////����80deg
//	//const double LCoxaJointMinsCos[6] = { 0.820, 0.142, -0.599, -0.819, -0.173,  0.574};//cos�̒l
//	//const double LCoxaJointMaxsCos[6] = {-0.578, 0.168,  0.816,  0.574, -0.173, -0.819};
//	//const double LCoxaJointMinsSin[6] = {-0.573, -0.990, -0.801,  0.573,  0.985,  0.819};//sin�̒l
//	//const double LCoxaJointMaxsSin[6] = {0.816,   0.986,  0.579, -0.819, -0.985, -0.573};
//
//
//	////����60deg												
//	//const	double	LCoxaJointMinsCos[6]	=	{	0.965925826	,	0.5	,	-0.258819045	,	-0.965925826	,	-0.5	,	0.258819045	};
//	//const	double	LCoxaJointMaxsCos[6]	=	{	-0.258819045	,	0.5	,	0.965925826	,	0.258819045	,	-0.5	,	-0.965925826	};										
//	//const	double	LCoxaJointMinsSin[6]	=	{	-0.258819045	,	-0.866025404	,	-0.965925826	,	0.258819045	,	0.866025404	,	0.965925826	};
//	//const	double	LCoxaJointMaxsSin[6]	=	{	0.965925826	,	0.866025404	,	0.258819045	,	-0.965925826	,	-0.866025404	,	-0.258819045	};
//
//	////����55deg
//	//const	double	LCoxaJointMinsCos[6]	=	{	0.984807753	,	0.573576436	,	-0.173648178	,	-0.984807753	,	-0.573576436	,	-0.819152044	};
//	//const	double	LCoxaJointMaxsCos[6]	=	{	-0.173648178	,	0.573576436	,	0.984807753	,	0.173648178	,	-0.573576436	,	-0.984807753	};
//	//const	double	LCoxaJointMinsSin[6]	=	{	-0.173648178	,	-0.819152044	,	-0.984807753	,	0.173648178	,	0.819152044	,	0.984807753	};
//	//const	double	LCoxaJointMaxsSin[6]	=	{	0.984807753	,	0.819152044	,	0.173648178	,	-0.984807753	,	-0.819152044	,	-0.173648178	};
//
//
//	////����50deg
//	//const	double	LCoxaJointMinsCos[6]	=	{	0.996194698	,	0.64278761	,	-0.087155743	,	-0.996194698	,	-0.64278761	,	-0.766044443	};
//	//const	double	LCoxaJointMaxsCos[6]	=	{	-0.087155743	,	0.64278761	,	0.996194698	,	0.087155743	,	-0.64278761	,	-0.996194698	};
//	//const	double	LCoxaJointMinsSin[6]	=	{	-0.087155743	,	-0.766044443	,	-0.996194698	,	0.087155743	,	0.766044443	,	0.996194698	};
//	//const	double	LCoxaJointMaxsSin[6]	=	{	0.996194698	,	0.766044443	,	0.087155743	,	-0.996194698	,	-0.766044443	,	-0.087155743	};
//
//	////����45
//	//const	double	LCoxaJointMinsCos[6]	=	{	1	,	0.707106781	,	6.12574E-17	,	-1	,	-0.707106781	,	-0.707106781	};
//	//const	double	LCoxaJointMaxsCos[6]	=	{	6.12574E-17	,	0.707106781	,	1	,	-6.12574E-17	,	-0.707106781	,	-1	};	
//	//const	double	LCoxaJointMinsSin[6]	=	{	6.12574E-17	,	-0.707106781	,	-1	,	-6.12574E-17	,	0.707106781	,	1	};
//	//const	double	LCoxaJointMaxsSin[6]	=	{	1	,	0.707106781	,	-6.12574E-17	,	-1	,	-0.707106781	,	6.12574E-17	};
//
//	//����40													
//	const	double	LCoxaJointMinsCos[6] = { 0.996194698f,	0.766044443f,	0.087155743f,	-0.996194698f,	-0.766044443f,	-0.087155743f };
//	const	double	LCoxaJointMaxsCos[6] = { 0.087155743f,	0.766044443f,	0.996194698f,	-0.087155743f,	-0.766044443f,	-0.996194698f };
//	const	double	LCoxaJointMinsSin[6] = { 0.087155743f,	-0.64278761f,	-0.996194698f,	-0.087155743f,	0.64278761f,	0.996194698f };
//	const	double	LCoxaJointMaxsSin[6] = { 0.996194698f,	0.64278761f,	-0.087155743f,	-0.996194698f,	-0.64278761f,	0.087155743f };
//
//	////����35
//	//const	double	LCoxaJointMinsCos[6]	=	{	0.984807753	,	0.819152044	,	0.173648178	,	-0.984807753	,	-0.819152044	,	-0.573576436	};
//	//const	double	LCoxaJointMaxsCos[6]	=	{	0.173648178	,	0.819152044	,	0.984807753	,	-0.173648178	,	-0.819152044	,	-0.984807753	};
//	//const	double	LCoxaJointMinsSin[6]	=	{	0.173648178	,	-0.573576436	,	-0.984807753	,	-0.173648178	,	0.573576436	,	0.984807753	};
//	//const	double	LCoxaJointMaxsSin[6]	=	{	0.984807753	,	0.573576436	,	-0.173648178	,	-0.984807753	,	-0.573576436	,	0.173648178	};
//
//	////����30
//	//const	double	LCoxaJointMinsCos[6]	=	{	0.965925826	,	0.866025404	,	0.258819045	,	-0.965925826	,	-0.866025404	,	-0.5	};
//	//const	double	LCoxaJointMaxsCos[6]	=	{	0.258819045	,	0.866025404	,	0.965925826	,	-0.258819045	,	-0.866025404	,	-0.965925826	};
//	//const	double	LCoxaJointMinsSin[6]	=	{	0.258819045	,	-0.5	,	-0.965925826	,	-0.258819045	,	0.5	,	0.965925826	};
//	//const	double	LCoxaJointMaxsSin[6]	=	{	0.965925826	,	0.5	,	-0.258819045	,	-0.965925826	,	-0.5	,	0.258819045	};
//
//
//
//
//	//static VECTOR CoxaJointMins[6], CoxaJointMaxs[6];
//
//	//for(int i = 0; i < 6; i++){
//	//
//	//CoxaJointMins[i] = VGet(CoxaJointMinsCos[i],CoxaJointMinsSin[i],0);
//	//CoxaJointMaxs[i] = VGet(CoxaJointMaxsCos[i],CoxaJointMaxsSin[i],0);
//	//}
//
//	double crossMinIn;
//	double crossMaxIn;
//
//	//double xy_LineEndSize, U_xy_LineEndSize;
//
//
//
//
//
//		//crossMinIn = CoxaJointMinsCos[legNum] * LineEnd.x - CoxaJointMinsSin[legNum] * LineEnd.y; 
//		//crossMaxIn = CoxaJointMaxsCos[legNum] * LineEnd.x - CoxaJointMaxsSin[legNum] * LineEnd.y; 
//		//if(legNum<3)if(crossMinIn < 0 || crossMaxIn > 0 )return 0;
//		//if(legNum>2)if(crossMinIn > 0 || crossMaxIn < 0 )return 0;
//
//	crossMinIn = LCoxaJointMinsCos[legNum] * LineEnd.x - LCoxaJointMinsSin[legNum] * LineEnd.y; //�O��,�r�ʒu����`�̃X�^�[�g�ʒu���<180���Ȃ�+,>180���Ȃ�-
//	crossMaxIn = LCoxaJointMaxsCos[legNum] * LineEnd.x - LCoxaJointMaxsSin[legNum] * LineEnd.y;
//	if (crossMinIn < 0 || crossMaxIn > 0)return 0;//��`�̊p�x��180�ȏ�Ȃ畄�����t�ɂȂ�
//
//	//std::cout<<"ok1\n";
//
//	myvector::SVector xy_LineEnd;
//	xy_LineEnd = LineEnd;
//	xy_LineEnd.z = 0;
//	/*std::cout << "lineend���ă��[�J���H" << LegROM_r[int(-LineEnd.z)] << std::endl;
//	std::string wait;
//	std::cin >> wait*/;
//
//	/*if (VMag(xy_LineEnd) < LegROM_r[int(LineEnd.z)] && VMag(xy_LineEnd) > 50) {*/
//
//	// LegROM_r ���̓��̍����ɂ�����r�̓��B���a�̒���
//	//std::cerr << LineEnd.z - m_local_coxajoint_pos[legNum].z << std::endl;
//	//double delta = VMag(LineEnd);
//	double delta = VMag(xy_LineEnd);//X0Y0���ʂɓ��e�����r�̍�������r��܂ł̔��a
//		//�t��������xy���ʂ̋���<���̍����ɑ΂�����͈͂̔��a//���̂���߂��ꍇ�͑̐��������̂Ŏg�p���Ȃ�20180312
//	if (MIN_LEG_RADIUS < delta && delta < LegROM_r[int(LineEnd.z)]) {//
//	//std::cout<<"ok2\n";
//
//	//double	coxa = atan2(LineEnd.x,LineEnd.y);
//	//if(legNum<3)if(coxaMaxsRad[legNum] < coxa  || coxa < coxaMinsRad[legNum])return 0;
//	//if(legNum>2)if(coxaMinsRad[legNum] < coxa  && coxa < coxaMaxsRad[legNum])return 0;
//
//
//
//	//crossMinIn = coxaMinsCos[legNum] * xy_LineEnd.x - coxaMinsSin[legNum] * xy_LineEnd.y; 
//	//crossMaxIn = coxaMaxsCos[legNum] * xy_LineEnd.x - coxaMaxsSin[legNum] * xy_LineEnd.y; 
//	//if(legNum<3)if(crossMinIn < 0 || crossMaxIn > 0 )return 0;
//	//if(legNum>2)if(crossMinIn > 0 || crossMaxIn < 0 )return 0;
//
//
//	//crossMinIn = LcoxaMinsCos[legNum] * xy_LineEnd.x - LcoxaMinsSin[legNum] * xy_LineEnd.y; 
//	//crossMaxIn = LcoxaMaxsCos[legNum] * xy_LineEnd.x - LcoxaMaxsSin[legNum] * xy_LineEnd.y; 
//	//if(crossMinIn < 0 || crossMaxIn > 0 )return 0;
//
//
//
//
//
//		return 1;
//		//}
//
//	}
//
//	return 0;
//}
//
////�r��A�ڒn�\�_�A�d�S���������łȂ��Ƃ��̉��͈͂��C���������̍��̂Ƃ���SPLP��possibleLegPointRotation�Ŏg���\��B
////delta_z�͏d�S�����i�O���[�o���j�Ƒ���̍����i�O���[�o���j�̍��ŁA�傫���Ƃ��Ă�0~200mm�̊ԂɎ��܂�Ȃ���_���BLegROM_r[]�̃C���f�b�N�X
////�������̊֐��Ƃ���Ă邱�ƕς��Ȃ��B�B��Xdelta_z�͐ڒn�ʍ��W�n�Ō����d�S�Ƌr�捂���̍��ɂ��Ȃ���΂Ȃ�Ȃ��B
//bool Hexapod::check_touchdown_point2(int legNum, const myvector::SVector& LineEnd, const double delta_z)
//{
//	if (int(delta_z) < MIN_DELTAZ || MAX_DELTAZ < int(delta_z)) return 0;
//
//	////����80deg
//	//const double LCoxaJointMinsCos[6] = { 0.820, 0.142, -0.599, -0.819, -0.173,  0.574};//cos�̒l
//	//const double LCoxaJointMaxsCos[6] = {-0.578, 0.168,  0.816,  0.574, -0.173, -0.819};
//	//const double LCoxaJointMinsSin[6] = {-0.573, -0.990, -0.801,  0.573,  0.985,  0.819};//sin�̒l
//	//const double LCoxaJointMaxsSin[6] = {0.816,   0.986,  0.579, -0.819, -0.985, -0.573};
//
//
//	////����60deg												
//	//const	double	LCoxaJointMinsCos[6]	=	{	0.965925826	,	0.5	,	-0.258819045	,	-0.965925826	,	-0.5	,	0.258819045	};
//	//const	double	LCoxaJointMaxsCos[6]	=	{	-0.258819045	,	0.5	,	0.965925826	,	0.258819045	,	-0.5	,	-0.965925826	};										
//	//const	double	LCoxaJointMinsSin[6]	=	{	-0.258819045	,	-0.866025404	,	-0.965925826	,	0.258819045	,	0.866025404	,	0.965925826	};
//	//const	double	LCoxaJointMaxsSin[6]	=	{	0.965925826	,	0.866025404	,	0.258819045	,	-0.965925826	,	-0.866025404	,	-0.258819045	};
//
//	////����55deg
//	//const	double	LCoxaJointMinsCos[6]	=	{	0.984807753	,	0.573576436	,	-0.173648178	,	-0.984807753	,	-0.573576436	,	-0.819152044	};
//	//const	double	LCoxaJointMaxsCos[6]	=	{	-0.173648178	,	0.573576436	,	0.984807753	,	0.173648178	,	-0.573576436	,	-0.984807753	};
//	//const	double	LCoxaJointMinsSin[6]	=	{	-0.173648178	,	-0.819152044	,	-0.984807753	,	0.173648178	,	0.819152044	,	0.984807753	};
//	//const	double	LCoxaJointMaxsSin[6]	=	{	0.984807753	,	0.819152044	,	0.173648178	,	-0.984807753	,	-0.819152044	,	-0.173648178	};
//
//
//	////����50deg
//	//const	double	LCoxaJointMinsCos[6]	=	{	0.996194698	,	0.64278761	,	-0.087155743	,	-0.996194698	,	-0.64278761	,	-0.766044443	};
//	//const	double	LCoxaJointMaxsCos[6]	=	{	-0.087155743	,	0.64278761	,	0.996194698	,	0.087155743	,	-0.64278761	,	-0.996194698	};
//	//const	double	LCoxaJointMinsSin[6]	=	{	-0.087155743	,	-0.766044443	,	-0.996194698	,	0.087155743	,	0.766044443	,	0.996194698	};
//	//const	double	LCoxaJointMaxsSin[6]	=	{	0.996194698	,	0.766044443	,	0.087155743	,	-0.996194698	,	-0.766044443	,	-0.087155743	};
//
//	////����45
//	//const	double	LCoxaJointMinsCos[6]	=	{	1	,	0.707106781	,	6.12574E-17	,	-1	,	-0.707106781	,	-0.707106781	};
//	//const	double	LCoxaJointMaxsCos[6]	=	{	6.12574E-17	,	0.707106781	,	1	,	-6.12574E-17	,	-0.707106781	,	-1	};	
//	//const	double	LCoxaJointMinsSin[6]	=	{	6.12574E-17	,	-0.707106781	,	-1	,	-6.12574E-17	,	0.707106781	,	1	};
//	//const	double	LCoxaJointMaxsSin[6]	=	{	1	,	0.707106781	,	-6.12574E-17	,	-1	,	-0.707106781	,	6.12574E-17	};
//
//
//	//����40													
//	const	double	LCoxaJointMinsCos[6] = { 0.996194698	,	0.766044443	,	0.087155743	,	-0.996194698	,	-0.766044443	,	-0.087155743 };
//	const	double	LCoxaJointMaxsCos[6] = { 0.087155743	,	0.766044443	,	0.996194698	,	-0.087155743	,	-0.766044443	,	-0.996194698 };
//	const	double	LCoxaJointMinsSin[6] = { 0.087155743	,	-0.64278761	,	-0.996194698	,	-0.087155743	,	0.64278761	,	0.996194698 };
//	const	double	LCoxaJointMaxsSin[6] = { 0.996194698	,	0.64278761	,	-0.087155743	,	-0.996194698	,	-0.64278761	,	0.087155743 };
//
//	////����35
//	//const	double	LCoxaJointMinsCos[6]	=	{	0.984807753	,	0.819152044	,	0.173648178	,	-0.984807753	,	-0.819152044	,	-0.573576436	};
//	//const	double	LCoxaJointMaxsCos[6]	=	{	0.173648178	,	0.819152044	,	0.984807753	,	-0.173648178	,	-0.819152044	,	-0.984807753	};
//	//const	double	LCoxaJointMinsSin[6]	=	{	0.173648178	,	-0.573576436	,	-0.984807753	,	-0.173648178	,	0.573576436	,	0.984807753	};
//	//const	double	LCoxaJointMaxsSin[6]	=	{	0.984807753	,	0.573576436	,	-0.173648178	,	-0.984807753	,	-0.573576436	,	0.173648178	};
//
//	////����30
//	//const	double	LCoxaJointMinsCos[6]	=	{	0.965925826	,	0.866025404	,	0.258819045	,	-0.965925826	,	-0.866025404	,	-0.5	};
//	//const	double	LCoxaJointMaxsCos[6]	=	{	0.258819045	,	0.866025404	,	0.965925826	,	-0.258819045	,	-0.866025404	,	-0.965925826	};
//	//const	double	LCoxaJointMinsSin[6]	=	{	0.258819045	,	-0.5	,	-0.965925826	,	-0.258819045	,	0.5	,	0.965925826	};
//	//const	double	LCoxaJointMaxsSin[6]	=	{	0.965925826	,	0.5	,	-0.258819045	,	-0.965925826	,	-0.5	,	0.258819045	};
//
//
//
//
//	//static VECTOR CoxaJointMins[6], CoxaJointMaxs[6];
//
//	//for(int i = 0; i < 6; i++){
//	//
//	//CoxaJointMins[i] = VGet(CoxaJointMinsCos[i],CoxaJointMinsSin[i],0);
//	//CoxaJointMaxs[i] = VGet(CoxaJointMaxsCos[i],CoxaJointMaxsSin[i],0);
//	//}
//
//	double crossMinIn;
//	double crossMaxIn;
//
//	//double xy_LineEndSize, U_xy_LineEndSize;
//
//
//
//
//
//		//crossMinIn = CoxaJointMinsCos[legNum] * LineEnd.x - CoxaJointMinsSin[legNum] * LineEnd.y; 
//		//crossMaxIn = CoxaJointMaxsCos[legNum] * LineEnd.x - CoxaJointMaxsSin[legNum] * LineEnd.y; 
//		//if(legNum<3)if(crossMinIn < 0 || crossMaxIn > 0 )return 0;
//		//if(legNum>2)if(crossMinIn > 0 || crossMaxIn < 0 )return 0;
//
//	crossMinIn = LCoxaJointMinsCos[legNum] * LineEnd.x - LCoxaJointMinsSin[legNum] * LineEnd.y; //�O��,�r�ʒu����`�̃X�^�[�g�ʒu���<180���Ȃ�+,>180���Ȃ�-
//	crossMaxIn = LCoxaJointMaxsCos[legNum] * LineEnd.x - LCoxaJointMaxsSin[legNum] * LineEnd.y;
//	if (crossMinIn < 0 || crossMaxIn > 0)return 0;//��`�̊p�x��180�ȏ�Ȃ畄�����t�ɂȂ�
//
//	//std::cout<<"ok1\n";
//
//	myvector::SVector xy_LineEnd;
//	xy_LineEnd = LineEnd;
//	xy_LineEnd.z = 0;
//	/*std::cout << "lineend���ă��[�J���H" << LegROM_r[int(-LineEnd.z)] << std::endl;
//	std::string wait;
//	std::cin >> wait*/;
//
//	/*if (VMag(xy_LineEnd) < LegROM_r[int(LineEnd.z)] && VMag(xy_LineEnd) > 50) {*/
//
//	// LegROM_r ���̓��̍����ɂ�����r�̓��B���a�̒���
//	//std::cerr << LineEnd.z - m_local_coxajoint_pos[legNum].z << std::endl;
//	//double delta = VMag(LineEnd);
//	double delta = VMag(xy_LineEnd);//X0Y0���ʂɓ��e�����r�̍�������r��܂ł̔��a
//		//�t��������xy���ʂ̋���<���̍����ɑ΂�����͈͂̔��a//���̂���߂��ꍇ�͑̐��������̂Ŏg�p���Ȃ�20180312
//	if (MIN_LEG_RADIUS < delta && delta < LegROM_r[int(delta_z)]) {//�d�S�����Ƃ���r�ݒu�\�_�̍����̍��ɉ����āA���e���锼�a���ς��B
//		//����LegROM_r�̌v�Z����73mm����ɂ�����ƒT���s�\�@��X�̓C���f�b�N�X�́A�ڒn�ʍ��W�n�Ō����d�S�Ƌr�捂���̍��ɂ��Ȃ���΂Ȃ�Ȃ��B
//	//std::cout<<"ok2\n";
//
//	//double	coxa = atan2(LineEnd.x,LineEnd.y);
//	//if(legNum<3)if(coxaMaxsRad[legNum] < coxa  || coxa < coxaMinsRad[legNum])return 0;
//	//if(legNum>2)if(coxaMinsRad[legNum] < coxa  && coxa < coxaMaxsRad[legNum])return 0;
//
//
//
//	//crossMinIn = coxaMinsCos[legNum] * xy_LineEnd.x - coxaMinsSin[legNum] * xy_LineEnd.y; 
//	//crossMaxIn = coxaMaxsCos[legNum] * xy_LineEnd.x - coxaMaxsSin[legNum] * xy_LineEnd.y; 
//	//if(legNum<3)if(crossMinIn < 0 || crossMaxIn > 0 )return 0;
//	//if(legNum>2)if(crossMinIn > 0 || crossMaxIn < 0 )return 0;
//
//
//	//crossMinIn = LcoxaMinsCos[legNum] * xy_LineEnd.x - LcoxaMinsSin[legNum] * xy_LineEnd.y; 
//	//crossMaxIn = LcoxaMaxsCos[legNum] * xy_LineEnd.x - LcoxaMaxsSin[legNum] * xy_LineEnd.y; 
//	//if(crossMinIn < 0 || crossMaxIn > 0 )return 0;
//
//
//
//
//
//		return 1;
//		//}
//
//	}
//
//	return 0;
//}
//
//bool Hexapod::check_touchdown_point3(int legNum, const myvector::SVector& LineEnd, const double delta_z) {
//	if (int(delta_z) < MIN_DELTAZ || MAX_DELTAZ < int(delta_z)) return 0;
//	myvector::SVector xy_LineEnd;
//	xy_LineEnd = LineEnd;
//	xy_LineEnd.z = 0;
//
//	double delta = VMag(xy_LineEnd);//X0Y0���ʂɓ��e�����r�̍�������r��܂ł̔��a
//		//�t��������xy���ʂ̋���<���̍����ɑ΂�����͈͂̔��a//���̂���߂��ꍇ�͑̐��������̂Ŏg�p���Ȃ�20180312
//	if (MIN_LEG_RADIUS < delta && delta < LegROM_r[int(delta_z)]) {//�d�S�����Ƃ���r�ݒu�\�_�̍����̍��ɉ����āA���e���锼�a���ς��B
//		return 1;
//
//
//	}
//
//	return 0;
//}
//
//bool Hexapod::check_touchdown_point4(int legNum, const myvector::SVector& LineEnd) {
//
//	////����80deg
//	//const double LCoxaJointMinsCos[6] = { 0.820, 0.142, -0.599, -0.819, -0.173,  0.574};//cos�̒l
//	//const double LCoxaJointMaxsCos[6] = {-0.578, 0.168,  0.816,  0.574, -0.173, -0.819};
//	//const double LCoxaJointMinsSin[6] = {-0.573, -0.990, -0.801,  0.573,  0.985,  0.819};//sin�̒l
//	//const double LCoxaJointMaxsSin[6] = {0.816,   0.986,  0.579, -0.819, -0.985, -0.573};
//
//
//	////����60deg												
//	//const	double	LCoxaJointMinsCos[6]	=	{	0.965925826	,	0.5	,	-0.258819045	,	-0.965925826	,	-0.5	,	0.258819045	};
//	//const	double	LCoxaJointMaxsCos[6]	=	{	-0.258819045	,	0.5	,	0.965925826	,	0.258819045	,	-0.5	,	-0.965925826	};										
//	//const	double	LCoxaJointMinsSin[6]	=	{	-0.258819045	,	-0.866025404	,	-0.965925826	,	0.258819045	,	0.866025404	,	0.965925826	};
//	//const	double	LCoxaJointMaxsSin[6]	=	{	0.965925826	,	0.866025404	,	0.258819045	,	-0.965925826	,	-0.866025404	,	-0.258819045	};
//
//	////����55deg
//	//const	double	LCoxaJointMinsCos[6]	=	{	0.984807753	,	0.573576436	,	-0.173648178	,	-0.984807753	,	-0.573576436	,	-0.819152044	};
//	//const	double	LCoxaJointMaxsCos[6]	=	{	-0.173648178	,	0.573576436	,	0.984807753	,	0.173648178	,	-0.573576436	,	-0.984807753	};
//	//const	double	LCoxaJointMinsSin[6]	=	{	-0.173648178	,	-0.819152044	,	-0.984807753	,	0.173648178	,	0.819152044	,	0.984807753	};
//	//const	double	LCoxaJointMaxsSin[6]	=	{	0.984807753	,	0.819152044	,	0.173648178	,	-0.984807753	,	-0.819152044	,	-0.173648178	};
//
//
//	////����50deg
//	//const	double	LCoxaJointMinsCos[6]	=	{	0.996194698	,	0.64278761	,	-0.087155743	,	-0.996194698	,	-0.64278761	,	-0.766044443	};
//	//const	double	LCoxaJointMaxsCos[6]	=	{	-0.087155743	,	0.64278761	,	0.996194698	,	0.087155743	,	-0.64278761	,	-0.996194698	};
//	//const	double	LCoxaJointMinsSin[6]	=	{	-0.087155743	,	-0.766044443	,	-0.996194698	,	0.087155743	,	0.766044443	,	0.996194698	};
//	//const	double	LCoxaJointMaxsSin[6]	=	{	0.996194698	,	0.766044443	,	0.087155743	,	-0.996194698	,	-0.766044443	,	-0.087155743	};
//
//	////����45
//	//const	double	LCoxaJointMinsCos[6]	=	{	1	,	0.707106781	,	6.12574E-17	,	-1	,	-0.707106781	,	-0.707106781	};
//	//const	double	LCoxaJointMaxsCos[6]	=	{	6.12574E-17	,	0.707106781	,	1	,	-6.12574E-17	,	-0.707106781	,	-1	};	
//	//const	double	LCoxaJointMinsSin[6]	=	{	6.12574E-17	,	-0.707106781	,	-1	,	-6.12574E-17	,	0.707106781	,	1	};
//	//const	double	LCoxaJointMaxsSin[6]	=	{	1	,	0.707106781	,	-6.12574E-17	,	-1	,	-0.707106781	,	6.12574E-17	};
//
//
//	//����40													
//	const	double	LCoxaJointMinsCos[6] = { 0.996194698	,	0.766044443	,	0.087155743	,	-0.996194698	,	-0.766044443	,	-0.087155743 };
//	const	double	LCoxaJointMaxsCos[6] = { 0.087155743	,	0.766044443	,	0.996194698	,	-0.087155743	,	-0.766044443	,	-0.996194698 };
//	const	double	LCoxaJointMinsSin[6] = { 0.087155743	,	-0.64278761	,	-0.996194698	,	-0.087155743	,	0.64278761	,	0.996194698 };
//	const	double	LCoxaJointMaxsSin[6] = { 0.996194698	,	0.64278761	,	-0.087155743	,	-0.996194698	,	-0.64278761	,	0.087155743 };
//
//	////����35
//	//const	double	LCoxaJointMinsCos[6]	=	{	0.984807753	,	0.819152044	,	0.173648178	,	-0.984807753	,	-0.819152044	,	-0.573576436	};
//	//const	double	LCoxaJointMaxsCos[6]	=	{	0.173648178	,	0.819152044	,	0.984807753	,	-0.173648178	,	-0.819152044	,	-0.984807753	};
//	//const	double	LCoxaJointMinsSin[6]	=	{	0.173648178	,	-0.573576436	,	-0.984807753	,	-0.173648178	,	0.573576436	,	0.984807753	};
//	//const	double	LCoxaJointMaxsSin[6]	=	{	0.984807753	,	0.573576436	,	-0.173648178	,	-0.984807753	,	-0.573576436	,	0.173648178	};
//
//	////����30
//	//const	double	LCoxaJointMinsCos[6]	=	{	0.965925826	,	0.866025404	,	0.258819045	,	-0.965925826	,	-0.866025404	,	-0.5	};
//	//const	double	LCoxaJointMaxsCos[6]	=	{	0.258819045	,	0.866025404	,	0.965925826	,	-0.258819045	,	-0.866025404	,	-0.965925826	};
//	//const	double	LCoxaJointMinsSin[6]	=	{	0.258819045	,	-0.5	,	-0.965925826	,	-0.258819045	,	0.5	,	0.965925826	};
//	//const	double	LCoxaJointMaxsSin[6]	=	{	0.965925826	,	0.5	,	-0.258819045	,	-0.965925826	,	-0.5	,	0.258819045	};
//
//
//
//
//	//static VECTOR CoxaJointMins[6], CoxaJointMaxs[6];
//
//	//for(int i = 0; i < 6; i++){
//	//
//	//CoxaJointMins[i] = VGet(CoxaJointMinsCos[i],CoxaJointMinsSin[i],0);
//	//CoxaJointMaxs[i] = VGet(CoxaJointMaxsCos[i],CoxaJointMaxsSin[i],0);
//	//}
//
//	double crossMinIn;
//	double crossMaxIn;
//
//	//double xy_LineEndSize, U_xy_LineEndSize;
//
//
//
//
//
//		//crossMinIn = CoxaJointMinsCos[legNum] * LineEnd.x - CoxaJointMinsSin[legNum] * LineEnd.y; 
//		//crossMaxIn = CoxaJointMaxsCos[legNum] * LineEnd.x - CoxaJointMaxsSin[legNum] * LineEnd.y; 
//		//if(legNum<3)if(crossMinIn < 0 || crossMaxIn > 0 )return 0;
//		//if(legNum>2)if(crossMinIn > 0 || crossMaxIn < 0 )return 0;
//
//	crossMinIn = LCoxaJointMinsCos[legNum] * LineEnd.x - LCoxaJointMinsSin[legNum] * LineEnd.y; //�O��,�r�ʒu����`�̃X�^�[�g�ʒu���<180���Ȃ�+,>180���Ȃ�-
//	crossMaxIn = LCoxaJointMaxsCos[legNum] * LineEnd.x - LCoxaJointMaxsSin[legNum] * LineEnd.y;
//	if (crossMinIn < 0 || crossMaxIn > 0)return 0;//��`�̊p�x��180�ȏ�Ȃ畄�����t�ɂȂ�
//
//	//std::cout<<"ok1\n";
//
//	myvector::SVector xy_LineEnd;
//	xy_LineEnd = LineEnd;
//	xy_LineEnd.z = 0;
//	/*std::cout << "lineend���ă��[�J���H" << LegROM_r[int(-LineEnd.z)] << std::endl;
//	std::string wait;
//	std::cin >> wait*/;
//
//	/*if (VMag(xy_LineEnd) < LegROM_r[int(LineEnd.z)] && VMag(xy_LineEnd) > 50) {*/
//
//	// LegROM_r ���̓��̍����ɂ�����r�̓��B���a�̒���
//	//std::cerr << LineEnd.z - m_local_coxajoint_pos[legNum].z << std::endl;
//	//double delta = VMag(LineEnd);
//	double delta = VMag(xy_LineEnd);//X0Y0���ʂɓ��e�����r�̍�������r��܂ł̔��a
//		//�t��������xy���ʂ̋���<���̍����ɑ΂�����͈͂̔��a//���̂���߂��ꍇ�͑̐��������̂Ŏg�p���Ȃ�20180312
//	if (50 < delta && delta < LegROM_r[0]) {//�d�S�����Ƃ���r�ݒu�\�_�̍����̍��ɉ����āA���e���锼�a���ς��B
//		//����LegROM_r�̌v�Z����73mm����ɂ�����ƒT���s�\�@��X�̓C���f�b�N�X�́A�ڒn�ʍ��W�n�Ō����d�S�Ƌr�捂���̍��ɂ��Ȃ���΂Ȃ�Ȃ��B
//	//std::cout<<"ok2\n";
//
//	//double	coxa = atan2(LineEnd.x,LineEnd.y);
//	//if(legNum<3)if(coxaMaxsRad[legNum] < coxa  || coxa < coxaMinsRad[legNum])return 0;
//	//if(legNum>2)if(coxaMinsRad[legNum] < coxa  && coxa < coxaMaxsRad[legNum])return 0;
//
//
//
//	//crossMinIn = coxaMinsCos[legNum] * xy_LineEnd.x - coxaMinsSin[legNum] * xy_LineEnd.y; 
//	//crossMaxIn = coxaMaxsCos[legNum] * xy_LineEnd.x - coxaMaxsSin[legNum] * xy_LineEnd.y; 
//	//if(legNum<3)if(crossMinIn < 0 || crossMaxIn > 0 )return 0;
//	//if(legNum>2)if(crossMinIn > 0 || crossMaxIn < 0 )return 0;
//
//
//	//crossMinIn = LcoxaMinsCos[legNum] * xy_LineEnd.x - LcoxaMinsSin[legNum] * xy_LineEnd.y; 
//	//crossMaxIn = LcoxaMaxsCos[legNum] * xy_LineEnd.x - LcoxaMaxsSin[legNum] * xy_LineEnd.y; 
//	//if(crossMinIn < 0 || crossMaxIn > 0 )return 0;
//
//
//
//
//
//		return 1;
//		//}
//
//	}
//
//	return 0;
//}
//
////���s�\�ȑ̐��Ȃ��1  leg��CoxaJoint��(0,0,0)�Ƃ����Ƃ��̈ʒu
//bool Hexapod::isAblePause(myvector::SVector* leg, int groundingLeg[6])
//{
//	for (int i = 0; i < 6; i++)
//	{
//		if (groundingLeg[i] == 1)
//		{
//			if (check_touchdown_point(i, leg[i]) == 0)
//			{
//				return 0;
//			}
//		}
//	}
//	return 1;
//}
//
////�r�ʒu�Əd�S�ʒu���l���C�]�|���Ȃ������f
//bool Hexapod::isAbleCOM(myvector::SVector* L_leg, int groundingLeg[6]) {
//	int landingLegInd[7] = { 0 };
//	int landingLegCnt = 0;
//	for (int i = 0; i < 6; i++) {
//		//std::cout<<"groundingLeg = "<<groundingLeg[0]<<groundingLeg[1]<<groundingLeg[2]<<groundingLeg[3]<<groundingLeg[4]<<groundingLeg[5]<<"\n";
//		if (groundingLeg[i] == 1)landingLegInd[landingLegCnt++] = i;
//		//std::cout<<"i = "<<i<<"\n";
//		//myvector::VectorOutPut(L_leg[landingLegInd[i]]);
//	}
//	landingLegInd[landingLegCnt++] = landingLegInd[0];
//	//std::cout<<"landingLegCnt = "<<landingLegCnt<<"\n";
//
//	myvector::SVector G_CoxaJointPosi[6];
//	for (int i = 0; i < 6; i++) {
//		G_CoxaJointPosi[i] = getGlobalCoxaJointPos(i);
//	}
//
//	for (int i = 0; i < landingLegCnt - 1; i++) {
//		//std::cout<<"i = "<<i<<"\n";
//		//std::cout<<"landingLegInd["<<i<<"] = "<<landingLegInd[i]<<"\n";
//		//std::cout<<"landingLegInd["<<i+1<<"] = "<<landingLegInd[i+1]<<"\n";
//		//n�Ԗڂ̋r�ʒu����Ƃ���n+1�ԋr�܂ł̃x�N�g���Əd�S(0,0,0)�̈ʒu���ׂď�ɉE�ɂ�������ok
//		myvector::SVector n_np1, n_g;//landingLegInd[n]�ԋr����landingLegInd[n+1]�ԋr�ւ̃��F�N�^�\�ClandingLegInd[n]�ԋr����d�S�ւ̃��F�N�^�\
//		//myvector::VectorOutPut(myvector::VAdd(getGlobalCoxaJointPos(landingLegInd[i]),L_leg[landingLegInd[i]]));
//		//myvector::VectorOutPut(myvector::VAdd(getGlobalCoxaJointPos(landingLegInd[i+1]),L_leg[landingLegInd[i+1]]));
//
//		//myvector::VectorOutPut(getGlobalCoxaJointPos(landingLegInd[i]));
//		//myvector::VectorOutPut(getGlobalCoxaJointPos(landingLegInd[i+1]));
//
//		//myvector::VectorOutPut(L_leg[landingLegInd[i]]);
//		//myvector::VectorOutPut(L_leg[landingLegInd[i+1]]);
//
//		myvector::SVector x_y_changedL_leg[6];
//		for (int iLeg = 0; iLeg < 6; iLeg++)x_y_changedL_leg[iLeg] = myvector::VGet(L_leg[iLeg].y, L_leg[iLeg].x, L_leg[iLeg].z);//x��y,y��x
//
//		n_np1 = myvector::subVec(myvector::VAdd(G_CoxaJointPosi[landingLegInd[i + 1]], x_y_changedL_leg[landingLegInd[i + 1]]), myvector::VAdd(G_CoxaJointPosi[landingLegInd[i]], x_y_changedL_leg[landingLegInd[i]]));//n�Ԗڂ̋r�悩��n+1�ւ̃x�N�g��
//		n_np1.z = 0;
//		n_g = myvector::subVec(COMPOSI, myvector::VAdd(G_CoxaJointPosi[landingLegInd[i]], x_y_changedL_leg[landingLegInd[i]]));//n�Ԗڂ̋r�悩�烍�{�b�g�̏d�S�����̃x�N�g��
//		n_g.z = 0;
//		//myvector::VectorOutPut(n_np1);
//		//myvector::VectorOutPut(n_g);
//		if (myvector::VCross(n_np1, n_g).z > 0) {
//			//std::cout<<"return 0\n";
//			return 0;//�r���N���X����ꍇ
//		}
//
//	}
//
//
//	return 1;
//}
//
//myvector::SVector Hexapod::getGlobalMyPosition() {
//	return ziki.com;
//}
////		   �����p���B��������Z�̐��A���̑O����X�A�E����W�n��Y�@�����A�̐��ɂ�炸���{�b�g�ŗL�̂��̂�����A�v�Z���ʂ�萔�ŃR�s�[�ł������B�B�����ς��Ȃ��̂ł���΁A
//void Hexapod::makeLegROM_r() {
//	// �t�^���wcoxa�Ȃ��̌v�Z���ʂ�p���ď��^���w���v�Z����
//
//	for (int i = 0; i < 200; i++)LegROM_r[i] = 0;
//
//
//	const double PERMISSION = 0.5;			//�t�^���w�Ɖ^���w���s�������ʂ����aPermission^0.5�̉~�̒��Ȃ瓙�����ƍl����
//
//	const double mins[3] = { -1.428, -1.780, -1.194 };  //�r���͈� �����炭rad �ϊ��������(-81.8�� -101.98�� -68.41��)  190527
//	const double maxs[3] = { 1.402,  1.744,  1.769 };  //������coxa,femur,tibia (80.32�� 99.92�� 101.36��)
//
//	const double femurMins = -1.780;
//	const double femurMaxs = 1.744;
//	const double tibiaMins = -1.194;
//	const double tibiaMaxs = 1.769;
//
//	double coxa, femur, tibia;
//	myvector::SVector kinematics;
//	myvector::SVector LineEnd;//�r����W�i���[�J���j
//
//	//ans of kinematics use sorution of i_kinematics 
//
//	for (int iz = 0; iz < 200; iz++) {//z�͍ő�196
//		for (int ix = 53; ix < 248; ix++) {//ix�͍ő�248
//			//std::cout<<iz<<","<<ix<<"\n";
//			LineEnd.x = ix;
//			LineEnd.y = 0;
//			LineEnd.z = iz;
//
//
//			//�t�^���wcoxa�Ȃ�
//			// first, make this a 2DOF problem... by solving coxa
//
//			coxa = atan2(LineEnd.x, LineEnd.y);//coxa�p�x
//			double IK_trueX = sqrt(pow(fabs(LineEnd.x), 2.0) + pow(fabs(LineEnd.y), 2.0)) - L_COXA;//femur���瑫��܂ł����ԃx�N�g����xy���ʂɓ��e�����Ƃ��̃x�N�g���̑傫��
//			//double im = sqrt(pow(fabs(IK_trueX), 2.0) + pow(fabs(LineEnd.z), 2.0));//femur���瑫��̋���
//			double im = sqrt(IK_trueX * IK_trueX + LineEnd.z * LineEnd.z);//��΂ɐ�
//
//
//
//			// get femur angle above horizon...
//			double q1 = -atan2(LineEnd.z, IK_trueX);//�}�C�i�X�ł������W�n�I��q1���̂͏�ɕ�//x���[�����ƒ�`��G���[
//			//std::cout << q1 * 180 /3.1415926 << std::endl;
//			//double d1 = pow((double)L_FEMUR, 2.0) - pow((double)L_TIBIA, 2.0) + pow(fabs((double)im), 2.0);
//			double q2 = acos((L_FEMUR * L_FEMUR + im * im - L_TIBIA * L_TIBIA) / (2 * L_FEMUR * im));//im=0���ƒ�`��G���[
//			//double d2 = 2 * L_FEMUR*im;
//			//double q2 = acos((double)d1 / (double)d2);	//�]���藝
//			femur = q1 + q2;//ok
//
//			// and tibia angle from femur...
//			//d1 = pow((double)L_FEMUR, 2.0) - pow(fabs((double)im), 2.0) + pow((double)L_TIBIA, 2.0);
//			//d2 = 2 * L_TIBIA*L_FEMUR;
//
//			//tibia = acos((double)d1 / (double)d2) - 1.570796326795;//ok
//			tibia = acos((L_FEMUR * L_FEMUR + L_TIBIA * L_TIBIA - im * im) / (2 * L_FEMUR * L_TIBIA)) - 1.570796326795;//ok
//			//lange of motion
//			//���@�͂킩��񂪁A�V�~�����[�V�������ƁA���ꂪ����Ȃ��B
//			//if�������ƁA�d�S�Ƒ��捂���̍����A73mm�ȉ��͎��Ȃ��Bhato
//			//if ( femur < femurMins)break;
//			//if (femurMaxs < femur)break;
//			//if (tibia < tibiaMins)break;
//			//if(tibiaMaxs < tibia )break;
//
//			//�^���w
//			double K_trueX;
//
//			K_trueX = L_FEMUR * cos(femur) + L_TIBIA * cos(femur + tibia - 1.570796326795);
//			//kinematics.x = sin(coxa) * (K_trueX + L_COXA);	//coxa�̋��߂������ςȂ��߁C������sin�ɂȂ��Ă�
//			kinematics.x = K_trueX + L_COXA;
//			//std::cout << kinematics.x << std::endl;
//			//kinematics.y = cos(coxa) * (K_trueX + L_COXA); //�v�̓[��
//			kinematics.y = 0;
//			//std::cout << kinematics.y << std::endl;
//			kinematics.z = -(L_FEMUR * sin(femur) + L_TIBIA * sin(femur + tibia - 1.570796326795));//����
//
//			double Permission = VSquareSize(subVec(kinematics, LineEnd));
//
//			if (PERMISSION > Permission) {
//
//				LegROM_r[iz] = ix - LEGROM_RMARGIN;//y=0�̂Ƃ��C�r����z�̂Ƃ���x�����̍ő�͈̔�
//#ifdef  MAX_LEG_RADIUS
//				if (iz <= 115) LegROM_r[iz] = MAX_LEG_RADIUS;//�r��u���ʒu����������ƃg���N������Ȃ��Ē��ݍ��݂�����������200�܂łɂ���2020/11/09hato
//				   //if (ix < LegROM_rmin[iz]) LegROM_rmin[iz] = ix;	//rmin�����߂����������ǂ���Ȃ�����190606
//#endif
//			}
//		}
//		//if (LegROM_rmin[iz] < 50) LegROM_rmin[iz] = 50;
//		//std::cout << "legRom[" << iz << "]= " << LegROM_r[iz] << std::endl;
//	}
//	//std::string wait;
//	//std::cin >> wait;
//}
//
////��`�����ڂ���xy���ɕ��s�Ȓ����`�̍���p1�ƉE��p2�̒��_�𓱏o���Ă���B//hato20200710
//void Hexapod::calculateRangeOfMovement(int legnum, myvector::SVector& p1, myvector::SVector& p2) {	//���݂̈ʒu�ƌ����ɂ�����r���B�͈͂�Ԃ� p1:�ŏ��p�x�̎��̍��W p2:�ő�p�x�̎��̍��W �s�b�`�E���[����]�͍l�����Ă��Ȃ� ��`�ł��邱�ƑO��
//	double coxaDefoAngle[6] = { 45.0, 0.0, -45.0, -135.0, 180, 135.0 };	//�r�֐߂̊�p�x�i���{�b�g���W�n�j -180~180�x���ł��邱��
//	for (int i = 0; i < 6; ++i)coxaDefoAngle[i] = coxaDefoAngle[i] * M_PI / 180.0;	//[rad]�ɕϊ�
//	double rangeOfAngle = 40.0 * M_PI / 180.0;	//[rad] �r�̉��͈� ���͊�p�x���}40�x�ɓ����Ɖ���
//	//double r = LegROM_r[int(this->ziki.com.z)];	//���݂̓��̍����ɂ�����r���B�����@����́A�n�ʂ�z=0�ɂ��邱�Ƃ�O��Ƃ��Ă��邩��_���B20200618
//	//���O���[�o���œ��̍���196mm�ȏ�Ƃ��ɂȂ�����A�o�O�錴��
//	//�Ƃ肠�����ԗ����邱�Ƃ��厖�B���ۂɂƂ�邩�ǂ����́Achecktouchdownpoint�ŋr�ݒu�\�_�̍������l�����Ĕ��肵�Ă��邩��B
//	double r = 240;// LegROM_r[0]; //0����237index��73���炢�ɂ���ƁAr=223���炢�B��X�̓C���f�b�N�X�́A�ڒn�ʍ��W�n�Ō����d�S�Ƌr�捂���̍��ɂ��Ȃ���΂Ȃ�Ȃ��B
//	//r�͋r�ݒu�\�_���Q�Ƃ���̈�̑傫���̗v�f�Br�̒l�ɂ���ċ����͕ω�����B
//	double t1, t2;	//t1:���̋r�ɂ�����ŏ��p�x t2:�ő�p�x
//
//	t1 = coxaDefoAngle[legnum] - rangeOfAngle + this->ziki.thY;	//[rad]
//	t2 = coxaDefoAngle[legnum] + rangeOfAngle + this->ziki.thY;
//	if ((t1 < -2 * M_PI) || (t1 > 2 * M_PI)) t1 = fmod(t1, 2 * M_PI);	//-2pi~2p��
//	if (t1 < -M_PI) t1 = t1 + 2 * M_PI;	//-pi��菬�������
//	if (t2 > M_PI) t2 = t2 - 2 * M_PI;
//
//	p1 = myvector::VGet(0, 0, 0); //������
//	p2 = myvector::VGet(0, 0, 0);
//
//	//p1p2�̍��W����
//	if ((r * cos(t1)) > p2.x) {
//		p2.x = r * cos(t1);
//	}
//	else if ((r * cos(t1)) < p1.x) {
//		p1.x = r * cos(t1);
//	}
//
//	if ((r * sin(t1)) > p2.y) {
//		p2.y = r * sin(t1);
//	}
//	else if ((r * sin(t1)) < p1.y) {
//		p1.y = r * sin(t1);
//	}
//
//	if ((r * cos(t2)) > p2.x) {
//		p2.x = r * cos(t2);
//	}
//	else if ((r * cos(t2)) < p1.x) {
//		p1.x = r * cos(t2);
//	}
//
//	if ((r * sin(t2)) > p2.y) {
//		p2.y = r * sin(t2);
//	}
//	else if ((r * sin(t2)) < p1.y) {
//		p1.y = r * sin(t2);
//	}
//
//	//90 180 -90 -180�x�̎��̏���
//	if ((t2 > M_PI / 4 && t1 < M_PI / 4) || (t1 > 0 && t1 < M_PI / 4 && t2 < -M_PI / 4)) p2.y = r;	//r*sin(90)
//	if (t2 > 0 && t1 < 0) p2.x = r;																	//r*con(0)
//	if ((t2 > -M_PI / 4 && t1 < -M_PI / 4) || (t1 > M_PI / 4 && t2 < 0 && t2 > -M_PI / 4)) p1.y = -r;	//r*sin(-90)
//	if (t2 < t1) p1.x = -r;																				//r*cos(180)
//
//	//�O���[�o�����W�ɕϊ�
//	//p1 = rotation(p1, myvector::VGet(this->ziki.com.x, this->ziki.com.y, 0), this->ziki.thP, this->ziki.thR, this->ziki.thY);
//	p1 = VAdd(p1, this->getGlobalCoxaJointPos(legnum));
//	/*std::cout << "coxa�̃O���[�o��" <<legnum << " = ";
//	myvector::VectorOutPut(this->getGlobalCoxaJointPos(legnum));*/
//	//p2 = rotation(p2, myvector::VGet(this->ziki.com.x, this->ziki.com.y, 0), this->ziki.thP, this->ziki.thR, this->ziki.thY);
//	p2 = VAdd(p2, this->getGlobalCoxaJointPos(legnum));
//
//	//if (ziki.thY > 0.01) {
//	//	std::cout << "????????@" <<legnum << std::endl;
//	//	myvector::VectorOutPut(p1);
//	//	myvector::VectorOutPut(p2);
//	//	std::string wait;
//	//	std::cin >> wait;
//	//}
//
//}