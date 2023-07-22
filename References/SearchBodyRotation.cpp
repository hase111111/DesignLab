#include "SearchBodyRotation.h"

SearchPossibleBodyRotation::SearchPossibleBodyRotation()
{
	////可能な胴体の耐性を決定する
	//phantomX.makeLegROM_r();
}

int SearchPossibleBodyRotation::PossibleLegPoint_Rotation()
{
	//こいつにはもともと処理がなかった．
	return 0;
}

int SearchPossibleBodyRotation::pass_body_rotation(int groundLeg[6])
{
	////ヨー軸の胴体回転探索
	//int DIV = 20;
	//double kaitennhankei=0.0;	//重心経路の回転半径
	//double step;//刻み幅
	//double Target_thY = 0.0;
	//int i, ii, nnn = 0;
	//myvector::SVector c_leg[6],c_leg2[6],r_leg[6],r_leg2[6],map[6];

	//for(i = 0; i < HexapodConst::LEG_NUM; i++)
	//{
	//	//現在の脚位置
	//	c_leg[i] = r_leg[i] = phantomX.getLocalLegPos(i);
	//	c_leg2[i] = r_leg2[i] = phantomX.getLocalLeg2Pos(i);
	//	map[i] = phantomX.getGlobalLegPos(i);
	//}

	//myvector::SVector Rotation_Center=phantomX.getGlobalMyPosition();	//旋回中心=重心
	////現在の角度をコピー
	//double thP=phantomX.getGlobalMyDirectionthP();
	//double thR=phantomX.getGlobalMyDirectionthR();
	//double thY=phantomX.getGlobalMyDirectionthY();

	////double leg_th[6] = {45,0,-45,-135,180,135};//脚の基準角度[deg]
	////目標角度
	//switch(phantomX.getTargetMode())
	//{
	//case ETargetMode::STRAIGHT_VECOTR:
	//case ETargetMode::STRAIGHT_POSITION:
	//	Target_thY = phantomX.getGlobalMyDirectionthY();//目標角度
	//	break;

	//case ETargetMode::TURN_ON_SPOT_DIRECTION:
	//case ETargetMode::TURN_DIRECTION:
	///*case 7:*/
	//	Target_thY = phantomX.getTargetRotation().z * 100;//方向×100
	//	break;

	//case ETargetMode::TURN_ON_SPOT_ANGLE:
	//	Target_thY = phantomX.getTargetAngle().z;//目標角度
	//	break;
	//
	//case ETargetMode::TURN_ANGLE:
	///*case 8:*/
	//	myvector::SVector Vtemp;
	//	Vtemp = myvector::subVec(Rotation_Center, phantomX.getRotaionCenter());

	//	if(myvector::V2Mag(Vtemp) <= phantomX.getTurningRadius())
	//	{
	//		Target_thY = atan2(Vtemp.y, Vtemp.x) + Define::MY_PI/2.0*phantomX.getTargetRotation().z - Define::MY_PI/2.0;//円の中心からの法線方向
	//	}
	//	else
	//	{
	//		//この部分は修正してないので間違っている
	//		myvector::SVector a;
	//		Vtemp.x = Rotation_Center.x - phantomX.getRotaionCenter().x;
	//		Vtemp.y = Rotation_Center.y - phantomX.getRotaionCenter().y;

	//		if(phantomX.getTargetRotation().z > 0)
	//		{
	//			//左回転方向接点
	//			a.x = (Vtemp.x * phantomX.getTurningRadius() - Vtemp.y * sqrt(pow(Vtemp.x,2.0) + pow(Vtemp.y,2.0) - pow(phantomX.getTurningRadius(),2.0))) * phantomX.getTurningRadius() / (pow(Vtemp.x,2.0) + pow(Vtemp.y,2.0)) + phantomX.getRotaionCenter().x;
	//			a.y = (Vtemp.y * phantomX.getTurningRadius() + Vtemp.x * sqrt(pow(Vtemp.x,2.0) + pow(Vtemp.y,2.0) - pow(phantomX.getTurningRadius(),2.0))) * phantomX.getTurningRadius() / (pow(Vtemp.x,2.0) + pow(Vtemp.y,2.0)) + phantomX.getRotaionCenter().y;
	//			Target_thY = atan2(a.y - Rotation_Center.y, a.x -Rotation_Center.x);
	//		}
	//		else if(phantomX.getTargetRotation().z < 0)
	//		{
	//			//右回転方向接点
	//			a.x = (Vtemp.x * phantomX.getTurningRadius() + Vtemp.y * sqrt(pow(Vtemp.x,2.0) + pow(Vtemp.y,2.0) - pow(phantomX.getTurningRadius(),2.0))) * phantomX.getTurningRadius() / (pow(Vtemp.x,2.0) + pow(Vtemp.y,2.0)) + phantomX.getRotaionCenter().x;
	//			a.y = (Vtemp.y * phantomX.getTurningRadius() - Vtemp.x * sqrt(pow(Vtemp.x,2.0) + pow(Vtemp.y,2.0) - pow(phantomX.getTurningRadius(),2.0))) * phantomX.getTurningRadius() / (pow(Vtemp.x,2.0) + pow(Vtemp.y,2.0)) + phantomX.getRotaionCenter().y;
	//			Target_thY = atan2(a.y - Rotation_Center.y, a.x -Rotation_Center.x);
	//		}
	//	}
	//	if (thY - Define::MY_PI > Target_thY) { Target_thY += Define::MY_PI * 2.0; }
	//	break;
	//default:
	//	break;
	//}

	////回転方向,回転角度単位(0.1は適当)
	//if (thY < Target_thY) { step = 0.1; }
	//else if (thY > Target_thY) { step = -0.1; }

	////旋回によるロボットから見た脚の移動量
	//for (ii = 0; ii < HexapodConst::LEG_NUM; ii++) 
	//{
	//	Leg_move[ii][0] = myvector::VGet(0.0, 0.0, 0.0); 
	//}

	//for(i = 0; i < 20; i++)
	//{
	//	//step=0.05なら1.0[rad]≒57.3[deg]まで
	//	for(ii = 0; ii < HexapodConst::LEG_NUM; ii++)
	//	{
	//		if(phantomX.isLegWithinRange(ii, VCangeBodyToLeg(r_leg[ii],0.0,0.0,0.0))/* || groundLeg[ii] == 0*/)
	//		{

	//		}
	//		else
	//		{
	//			break;
	//		}
	//	}
	//	if(ii != HexapodConst::LEG_NUM) 
	//	{
	//		if((int)(phantomX.getTargetMode())>=3 && nnn == 0)
	//		{
	//			if(step > 0)
	//			{
	//				//反対方向側も0.1radだけ探索
	//				i--;
	//				thY = C_thPRY[2][0] - 0.1;
	//				nnn++;
	//				phantomX.setMyDirection(thP,thR,thY);

	//				for(ii = 0; ii < HexapodConst::LEG_NUM; ii++)
	//				{
	//					if(groundLeg[ii] == 1)
	//					{
	//						//接地脚
	//						Leg_move[ii][i+1] = myvector::subVec(myvector::VRot( myvector::subVec(map[ii], phantomX.getGlobalMyPosition()), thP, thR, thY), phantomX.getLocalCoxaJointPos(ii));
	//						r_leg2[ii] = Leg_move[ii][i+1];
	//						r_leg2[ii].z = -110;
	//						Leg_move[ii][i+1].z = c_leg[ii].z;
	//						r_leg[ii] = Leg_move[ii][i+1];
	//					}
	//					else
	//					{
	//						//遊脚はロボット座標系は変わらない（胴体と一緒に回転）
	//						Leg_move[ii][i+1] = phantomX.getLocalLegPos(ii);
	//						r_leg[ii] = phantomX.getLocalLegPos(ii);
	//						r_leg2[ii] = phantomX.getLocalLeg2Pos(ii);
	//					}
	//				}
	//				phantomX.setLocalLeg2Pos(r_leg2);//脚位置更新
	//				phantomX.setLocalLegPos(r_leg);
	//			}
	//			else if(step < 0)
	//			{
	//				//反対方向側も0.1radだけ探索
	//				i--;
	//				thY = C_thPRY[2][0] + 0.1;
	//				nnn++;
	//				phantomX.setMyDirection(thP,thR,thY);

	//				for(ii = 0; ii < HexapodConst::LEG_NUM; ii++)
	//				{
	//					if(groundLeg[ii] == 1)
	//					{
	//						//接地脚
	//						Leg_move[ii][i+1] = myvector::subVec(myvector::VRot( myvector::subVec(map[ii], phantomX.getGlobalMyPosition()), thP, thR, thY), phantomX.getLocalCoxaJointPos(ii));
	//						r_leg2[ii] = Leg_move[ii][i+1];
	//						r_leg2[ii].z = -110;
	//						Leg_move[ii][i+1].z = c_leg[ii].z;
	//						r_leg[ii] = Leg_move[ii][i+1];
	//					}
	//					else
	//					{
	//						//遊脚はロボット座標系は変わらない（胴体と一緒に回転）
	//						Leg_move[ii][i+1] = phantomX.getLocalLegPos(ii);
	//						r_leg[ii] = phantomX.getLocalLegPos(ii);
	//						r_leg2[ii] = phantomX.getLocalLeg2Pos(ii);
	//					}
	//				}
	//				phantomX.setLocalLeg2Pos(r_leg2);//脚位置更新
	//				phantomX.setLocalLegPos(r_leg);
	//			}
	//			else 
	//			{
	//				break; 
	//			}
	//		}
	//		else 
	//		{
	//			break; 
	//		}
	//	}
	//	else
	//	{
	//		C_thPRY[0][i] = thP;//実行可能なら角度保存
	//		C_thPRY[1][i] = thR;
	//		C_thPRY[2][i] = thY;

	//		thY+=step;//角度更新

	//		if (nnn == 1) { break; }

	//		phantomX.setMyDirection(thP,thR,thY);//胴体回転

	//		for(ii = 0; ii < HexapodConst::LEG_NUM; ii++)
	//		{
	//			if(groundLeg[ii] == 1)
	//			{
	//				//接地脚
	//				Leg_move[ii][i+1] = myvector::subVec(myvector::VRot( myvector::subVec(map[ii], phantomX.getGlobalMyPosition()), thP, thR, thY), phantomX.getLocalCoxaJointPos(ii));
	//				r_leg2[ii] = Leg_move[ii][i+1];
	//				r_leg2[ii].z = -110;
	//				Leg_move[ii][i+1].z = c_leg[ii].z;
	//				r_leg[ii] = Leg_move[ii][i+1];
	//			}
	//			else
	//			{
	//				//遊脚はロボット座標系は変わらない（胴体と一緒に回転）
	//				Leg_move[ii][i+1] = phantomX.getLocalLegPos(ii);
	//				r_leg[ii] = phantomX.getLocalLegPos(ii);
	//				r_leg2[ii] = phantomX.getLocalLeg2Pos(ii);
	//			}
	//		}

	//		phantomX.setLocalLeg2Pos(r_leg2);//脚位置更新
	//		phantomX.setLocalLegPos(r_leg);
	//	}
	//}

	//return i;
	return 0;
}

myvector::SVector SearchPossibleBodyRotation::VCangeBodyToLeg(const myvector::SVector& Vin, const  double thP, const double thR, const double thY) const
{
	return myvector::VRot(myvector::VGet(Vin.x, -Vin.y, -Vin.z), thP, thR, -thY - Define::MY_PI / 2.0);
}