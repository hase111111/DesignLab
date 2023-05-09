#include <Windows.h>
#include "hexapodGraphic.h"

//ウィンドウ作成用の構造体
HINSTANCE g_hInstance;

HexapodGraphic::HexapodGraphic(HINSTANCE hInstance) : CrBody(GetColorU8(23, 58, 235, 0))
{
	g_hInstance=hInstance;

	//時間の設定
	startTime = clock();
	Cicle_time = 0;

	NumFootprint = 0;
	Rot = 0;
	std::cout<<"HexapodGraphic 開始.\n";
}

void HexapodGraphic::operator()()
{
	//DXライブラリの設定
 
	//タイトルを HexapodGraphic に変更
	SetMainWindowText(TEXT("PhantomXGraphic"));
	//ウインドウモードに変更
	ChangeWindowMode( TRUE ) ;
	//ログ出力無しに変更
	SetOutApplicationLogValidFlag( FALSE ) ;
	//ウインドウがアクティブではない状態でも処理を続行する
	SetAlwaysRunFlag(TRUE);
	//描画先を裏画面にする
	SetDrawScreen( DX_SCREEN_BACK );
	// 画面モードの設定
	SetGraphMode( 800 , 620 , 32 ) ;
	//ＤＸライブラリ初期化処理
	if( DxLib_Init() == -1 )
	{
		 std::cout<<"Error DxLib_Init";    //Errorの場合は終了
		 std::string stop;
		 std::cin>>stop;
	}

	//奥行0.1～1000までをカメラの描画範囲とする
	SetCameraNearFar( 0.1f, 1000.0f ) ;
	// ライティングの計算をしないように設定を変更
	SetUseLighting( FALSE ) ;
	// Ｘ軸のマイナス方向のディレクショナルライトに変更
	//ChangeLightTypeDir( VGet( 0.0f, -1.0f, -1.0f  ) ) ;
	// 標準ライトのディフューズカラー設定を変更
	//SetLightDifColor( GetColorF( 1.0f, 1.0f, 1.0f, 0.0f ) ) ;
	// 標準ライトのアンビエントカラー設定を変更
	//SetLightAmbColor( GetColorF( 1.0f, 1.0f, 1.0f, 0.0f ) ) ;
	// 標準ライトのスペキュラカラー設定を変更
	//SetLightSpcColor( GetColorF( 1.0f, 1.0f, 1.0f, 0.0f ) ) ;
	// マテリアルの自己発光色の設定を変更
	MATERIALPARAM Material ;
	Material.Diffuse  = GetColorF( 0.0f, 0.0f, 0.0f, 0.0f ) ;//ディフューズカラー	拡散光色
	Material.Specular = GetColorF( 1.0f, 1.0f, 1.0f, 0.0f ) ;//スペキュラカラー		反射光色
	Material.Ambient  = GetColorF( 0.1f, 0.1f, 0.1f, 0.0f ) ;//アンビエントカラー	環境光色
	Material.Emissive = GetColorF( 0.1f, 0.1f, 0.1f, 0.0f ) ;//エミッシブカラー		自己発光色
	Material.Power    = 20.0f ;								 //スペキュラの強さ		スペキュラハイライトの角度範囲
	SetMaterialParam( Material ) ;
	// Ｚバッファを有効にする
	SetUseZBuffer3D( TRUE ) ;
	// Ｚバッファへの書き込みを有効にする
	SetWriteZBuffer3D( TRUE ) ;
	//背景色の設定
	SetBackgroundColor(30, 30, 70);
	//////////////////////////////////////////////////// DXライブラリの設定終わり

	//色の設定
	for(int i=0;i<6; i++){ m_color_leg[i] = GetColor(23, 58, 235);}//脚
	for(int i=0;i<6; i++){ CrJoint[i] = GetColorU8(164, 175, 185, 0);}//関節
	CrGround = GetColorU8(0, 0, 255, 255);
	CrGround2 = GetColorU8(255, 0, 0, 255);
	
	char ClassName[] = "PhantomXGraphic";
	WNDCLASSEX wcex;
	HWND hWnd;
	//メッセージ関数の呼び出し
	if (!g_hInstance) {
		wcex.cbSize         = sizeof(WNDCLASSEX);
		wcex.style          = 0;//CS_HREDRAW | CS_VREDRAW;
		wcex.lpfnWndProc    = WndProcGraphic;
		wcex.cbClsExtra     = 0;
		wcex.cbWndExtra     = 0;
		wcex.hInstance      = g_hInstance;
		wcex.hIcon          = NULL;
		wcex.hCursor        = LoadCursor(NULL, IDC_ARROW);
		wcex.hbrBackground  = (HBRUSH)GetStockObject(WHITE_BRUSH);
		wcex.lpszMenuName   = NULL;
		wcex.lpszClassName  = TEXT("PhantomXGraphic");
		wcex.hIconSm        = NULL;

		RegisterClassEx(&wcex);

	}

	hWnd = CreateWindowA( ClassName,TEXT("ウィンドウの表示"),WS_OVERLAPPEDWINDOW,CW_USEDEFAULT,CW_USEDEFAULT,100,100,NULL,NULL,g_hInstance,NULL);

	while(*isCicle)
	{
		//前回 GetMouseWheelRotVol が呼ばれてから今回までの回転量を足す
		Rot += GetMouseWheelRotVol();

		nowTime = clock();//現在時刻を取得
		Cicle_time = (int)(nowTime - startTime) / CLOCKS_PER_SEC;//プログラムが実行されてからの時間を計算
		
		//視点移動は以下の３のうち１つ．//追尾移動
		Camera_position3(VGet(myvecToDxvec(newPose.global_center_of_mass).x/10, myvecToDxvec(newPose.global_center_of_mass).y/10, myvecToDxvec(newPose.global_center_of_mass).z/10));

		//ロボットの体勢の取得
		previousPose = newPose;
		{
			boost::mutex::scoped_lock lk(*mtxHexapodGraphic); //排他制御が必要
			
			if(!answer->empty())
			{											
				newPose = answer->at(0);			//最初のノードを取得
				answer->erase(answer->begin());		//最初のノードを消去
			}	
		}

		//ロボットの3D位置の計算
		VECTOR _com;
		float _pitch, _roll, _yaw;
		_com	=  myvecToDxvec(newPose.global_center_of_mass);
		_pitch	= (float)newPose.pitch;
		_roll	= (float)newPose.roll;
		_yaw	= (float)newPose.yaw;

		//地形情報の取得
		{
			boost::mutex::scoped_lock lk(*mtxMapData); //排他制御が必要
			for(int i = 0; i < Define::MAPDATA3D_MAX; i++){
				map[i] = myvecToDxvec(p_map[i]);//脚接地可能点取得+y,zを交換//接地点の描写でしか使ってない

				//以下の二つをコメントアウトでロボットから見た地形情報となる
				map[i] = myvecToDxvec( myvector::VRot(dxvecToMyvec(map[i]),myvector::VGet(_com.x, _com.z, _com.y), _pitch, _roll, _yaw) );
				map[i] = VAdd( map[i], _com);
			}
		}

		//描画の処理
 
		//画面をクリア
		ClearDrawScreen();
		DrawHexapod(_com, newPose.Leg, _pitch, _roll, _yaw);		//ロボット描画
		DrawGround();//緑の格子線
		DrawSupportPolygon(newPose.Leg, _com, _pitch, _roll, _yaw);
		DrawComPass(_com);									
		DrawPossibleLegPosi(newPose.Leg);					//脚接地可能点
		//DrawRangeOfDirection(_com, _pitch, _roll, _yaw);		//脚地点検出範囲
		DrawMessage();
		ScreenFlip() ;
		
		if (ProcessMessage() == -1) { DxLib_End(); }
	}

	std::cout<<"hexapodGraphic終了\n" << std::endl;
}


void HexapodGraphic::ColorDebugMode()
{
	MATERIALPARAM Material ;
	Material.Diffuse  = GetColorF( 1.0f, 1.0f, 1.0f, 0.0f ) ;//ディフューズカラー	拡散光色
	Material.Specular = GetColorF( 1.0f, 1.0f, 1.0f, 0.0f ) ;//スペキュラカラー		反射光色
	Material.Ambient  = GetColorF( 0.0f, 0.0f, 0.0f, 0.0f ) ;//アンビエントカラー	環境光色
	Material.Emissive = GetColorF( 0.0f, 0.0f, 0.0f, 0.0f ) ;//エミッシブカラー		自己発光色
	Material.Power    = 20.0f ;								 //スペキュラの強さ		スペキュラハイライトの角度範囲
	SetMaterialParam( Material ) ;
}

LRESULT CALLBACK HexapodGraphic::WndProcGraphic(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
	//メッセージ処理
	switch (msg)
	{
		case WM_DESTROY:
			//ＤＸライブラリの終了処理
			DxLib_End();
			PostQuitMessage(0);
			return 0;

		default:
			return DefWindowProc(hWnd, msg, wParam, lParam);
	}

	return 0;
}

//6足ロボットを描写　引数は重心位置と脚の先端座標(ロボット座標)
void HexapodGraphic::DrawHexapod(VECTOR c_o_m, myvector::SVector leg_position[6], float pitch, float roll, float yaw)
{
	//6角形(胴体)の描画
	phantomX.setMyDirection(pitch, roll, yaw);
	phantomX.setMyPosition(dxvecToMyvec(c_o_m));
	phantomX.setMyLegPosition(leg_position);
	
	//body
	DrawBody(myvecToDxvec(phantomX.getGlobalCoxaJointPos(0) ),		
			myvecToDxvec(phantomX.getGlobalCoxaJointPos(1) ),
			myvecToDxvec(phantomX.getGlobalCoxaJointPos(2) ),
			myvecToDxvec(phantomX.getGlobalCoxaJointPos(3) ),
			myvecToDxvec(phantomX.getGlobalCoxaJointPos(4) ),
			myvecToDxvec(phantomX.getGlobalCoxaJointPos(5) ), CrBody);

	//脚の先端座標(ワールド座標)を保存する変数
	VECTOR legWORLD[6];
	//脚が接地,遊脚によって色を変える
	for (int i = 0; i < 6; i++) { m_color_leg[i] = GetColor(23, 58, 235); }
	for(int i=0;i<6; i++){ CrJoint[i] = GetColorU8(100, 100, 200, 255);}

	for(int i=0; i<6; i++)
	{
		legWORLD[i] = myvecToDxvec(phantomX.getGlobalLegPos(i));
		DrawCapsule3D_s(myvecToDxvec(phantomX.getGlobalCoxaJointPos(i)), myvecToDxvec(phantomX.getGlobalFemurJointPos(i)), 15, 8, m_color_leg[i], GetColor(255, 255, 255), TRUE);	//coxa
		DrawCapsule3D_s(myvecToDxvec(phantomX.getGlobalFemurJointPos(i)), myvecToDxvec(phantomX.getGlobalTibiaJointPos(i)), 15, 8, m_color_leg[i], GetColor(255, 255, 255), TRUE);	//femur
		DrawCone3D_s(legWORLD[i], myvecToDxvec(phantomX.getGlobalTibiaJointPos(i)), 15, 8, m_color_leg[i], GetColor(255, 255, 255), TRUE);											//tibia 
		//DrawCapsule3D_s(legWORLD[i], myvecToDxvec(phantomX.getGlobalTibiaJointPos(i)), 10, 8, m_color_leg[i], GetColor(255, 255, 255), TRUE);											//tibia 
												//tibia joint

		VECTOR coxaAxle = VScale(VNorm( VCross(VSub(myvecToDxvec(phantomX.getGlobalCoxaJointPos(0)),  myvecToDxvec(phantomX.getGlobalCoxaJointPos(1))), VSub(myvecToDxvec(phantomX.getGlobalCoxaJointPos(0)),  myvecToDxvec(phantomX.getGlobalCoxaJointPos(2))))), LEG_R * 2);
		VECTOR ftAxle =   VScale(VNorm( VCross(VSub(myvecToDxvec(phantomX.getGlobalCoxaJointPos(i)), myvecToDxvec(phantomX.getGlobalFemurJointPos(i))),VSub(myvecToDxvec(phantomX.getGlobalFemurJointPos(i)), myvecToDxvec(phantomX.getGlobalTibiaJointPos(i))))), LEG_R * 2);

		DrowTawara_s(myvecToDxvec(phantomX.getGlobalCoxaJointPos(i)), coxaAxle, LEG_R * 3.0f, LEG_R * 3.0f * 0.8f, 0.8f, CrJoint[i]);	//coxa joint
		DrowTawara_s(myvecToDxvec(phantomX.getGlobalFemurJointPos(i)), ftAxle, LEG_R * 3.0f, LEG_R * 3.0f * 0.8f, 0.8f, CrJoint[i]);	//femur joint
		DrowTawara_s(myvecToDxvec(phantomX.getGlobalTibiaJointPos(i)), ftAxle, LEG_R * 3.0f, LEG_R * 3.0f * 0.8f, 0.8f, CrJoint[i]);	//tibia joint

		DrawSphere3D_s(legWORLD[i], LEG_R * 0.8, 32, GetColor(255, 0, 0), GetColor(255, 255, 255), TRUE);	//leg tip
	}
}


VECTOR HexapodGraphic::myvecToDxvec(myvector::SVector Ivector)
{
	VECTOR res;
	res.x = (float)Ivector.x;
	res.y = (float)Ivector.z;
	res.z = (float)Ivector.y;
	
	return res;
}

myvector::SVector HexapodGraphic::dxvecToMyvec(VECTOR Ivector)
{
	myvector::SVector ret;
	ret.x = Ivector.x;
	ret.y = Ivector.z;
	ret.z = Ivector.y;
	
	return ret;
}

void HexapodGraphic::DrawHexagon(VECTOR c, float Fwidth, float Mwidth, float Rwidth, float Flength, float Rlength, float thickness, COLOR_U8 color)
{
//			    10,11______0,1
//				    /		\		重心				c
//				8,9/		 \2,3	中心から前方の幅	Fwidth
//				   \	 	/		中心から中央の幅	Mwidth
//			    6,7 \ _____/4,5		中心から後方の幅	Rwidth
//									重心から前への長さ	Flength
//									重心から後への長さ	Rlength
//									中心からの厚み		thickness
	VECTOR CoxaJoint_POSI[6];
	CoxaJoint_POSI[0] = VAdd(VGet(Fwidth, 0.0f, Flength), c);
	CoxaJoint_POSI[1] = VAdd(VGet(Mwidth, 0.0f, 0.0f), c);
	CoxaJoint_POSI[2] = VAdd(VGet(Rwidth, 0.0f, -Rlength), c);
	CoxaJoint_POSI[3] = VAdd(VGet(-Rwidth, 0.0f, -Rlength), c);
	CoxaJoint_POSI[4] = VAdd(VGet(-Mwidth, 0.0f, 0.0f), c);
	CoxaJoint_POSI[5] = VAdd(VGet(-Fwidth, 0.0f, Flength), c);

	VERTEX3D Vertex[ 12 ] ;
	WORD Index[ 20*3 ] ;
	COLOR_U8 color_spc=GetColorU8( 255,0,0,255);//RGBa aは透明度
	//0番目
	Vertex[ 0 ].pos  = VAdd( VGet( 0.0f,  thickness, 0.0f ), CoxaJoint_POSI[0] ) ;
	Vertex[ 0 ].norm = VGet( Fwidth,  thickness, Flength ) ;
	Vertex[ 0 ].dif  = color ;
	Vertex[ 0 ].spc  = color_spc ;
	Vertex[ 0 ].u    = 0.0f ;
	Vertex[ 0 ].v    = 0.0f ;
	Vertex[ 0 ].su   = 0.0f ;
	Vertex[ 0 ].sv   = 0.0f ;
	//1番目
	Vertex[ 1 ].pos  = VAdd( VGet( 0.0f,  -thickness, 0.0f ), CoxaJoint_POSI[0] ) ;
	Vertex[ 1 ].norm = VGet( Fwidth,  -thickness, Flength ) ;
	Vertex[ 1 ].dif  = color ;
	Vertex[ 1 ].spc  = color_spc ;
	Vertex[ 1 ].u    = 0.0f ;
	Vertex[ 1 ].v    = 0.0f ;
	Vertex[ 1 ].su   = 0.0f ;
	Vertex[ 1 ].sv   = 0.0f ;
	//2番目
	Vertex[ 2 ].pos  = VAdd( VGet( 0.0f,  thickness, 0.0f ), CoxaJoint_POSI[1] ) ;
	Vertex[ 2 ].norm = VGet( Mwidth,  thickness, 0 ) ;
	Vertex[ 2 ].dif  = color ;
	Vertex[ 2 ].spc  = color_spc ;
	Vertex[ 2 ].u    = 0.0f ;
	Vertex[ 2 ].v    = 0.0f ;
	Vertex[ 2 ].su   = 0.0f ;
	Vertex[ 2 ].sv   = 0.0f ;
	//3番目
	Vertex[ 3 ].pos  = VAdd( VGet( 0.0f,  -thickness, 0.0f ), CoxaJoint_POSI[1] ) ;
	Vertex[ 3 ].norm = VGet( Mwidth,  -thickness, 0 ) ;
	Vertex[ 3 ].dif  = color ;
	Vertex[ 3 ].spc  = color_spc ;
	Vertex[ 3 ].u    = 0.0f ;
	Vertex[ 3 ].v    = 0.0f ;
	Vertex[ 3 ].su   = 0.0f ;
	Vertex[ 0 ].sv   = 0.0f ;
	//4番目
	Vertex[ 4 ].pos  = VAdd( VGet( 0.0f,  thickness, 0.0f ), CoxaJoint_POSI[2] ) ;
	Vertex[ 4 ].norm = VGet( Rwidth,  thickness, Rlength ) ;
	Vertex[ 4 ].dif  = color ;
	Vertex[ 4 ].spc  = color_spc ;
	Vertex[ 4 ].u    = 0.0f ;
	Vertex[ 4 ].v    = 0.0f ;
	Vertex[ 4 ].su   = 0.0f ;
	Vertex[ 4 ].sv   = 0.0f ;
	//5番目
	Vertex[ 5 ].pos  = VAdd( VGet( 0.0f,  -thickness, 0.0f ), CoxaJoint_POSI[2] ) ;
	Vertex[ 5 ].norm = VGet( Rwidth,  -thickness, -Rlength ) ;
	Vertex[ 5 ].dif  = color ;
	Vertex[ 5 ].spc  = color_spc ;
	Vertex[ 5 ].u    = 0.0f ;
	Vertex[ 5 ].v    = 0.0f ;
	Vertex[ 5 ].su   = 0.0f ;
	Vertex[ 5 ].sv   = 0.0f ;
	//6番目
	Vertex[ 6 ].pos  = VAdd( VGet( 0.0f,  thickness, 0.0f ), CoxaJoint_POSI[3] ) ;
	Vertex[ 6 ].norm = VGet( -Rwidth,  thickness, -Rlength ) ;
	Vertex[ 6 ].dif  = color ;
	Vertex[ 6 ].spc  = color_spc ;
	Vertex[ 6 ].u    = 0.0f ;
	Vertex[ 6 ].v    = 0.0f ;
	Vertex[ 6 ].su   = 0.0f ;
	Vertex[ 6 ].sv   = 0.0f ;
	//7番目
	Vertex[ 7 ].pos  = VAdd( VGet( 0.0f,  -thickness, 0.0f ), CoxaJoint_POSI[3] ) ;
	Vertex[ 7 ].norm = VGet( -Rwidth,  -thickness, -Rlength ) ;
	Vertex[ 7 ].dif  = color ;
	Vertex[ 7 ].spc  = color_spc ;
	Vertex[ 7 ].u    = 0.0f ;
	Vertex[ 7 ].v    = 0.0f ;
	Vertex[ 7 ].su   = 0.0f ;
	Vertex[ 7 ].sv   = 0.0f ;
	//8番目
	Vertex[ 8 ].pos  = VAdd( VGet( 0.0f,  thickness, 0.0f ), CoxaJoint_POSI[4] ) ;
	Vertex[ 8 ].norm = VGet( -Mwidth,  thickness, 0 ) ;
	Vertex[ 8 ].dif  = color ;
	Vertex[ 8 ].spc  = color_spc ;
	Vertex[ 8 ].u    = 0.0f ;
	Vertex[ 8 ].v    = 0.0f ;
	Vertex[ 8 ].su   = 0.0f ;
	Vertex[ 8 ].sv   = 0.0f ;
	//9番目
	Vertex[ 9 ].pos  = VAdd( VGet( 0.0f,  -thickness, 0.0f ), CoxaJoint_POSI[4] ) ;
	Vertex[ 9 ].norm = VGet( -Mwidth,  -thickness, 0 ) ;
	Vertex[ 9 ].dif  = color ;
	Vertex[ 9 ].spc  = color_spc ;
	Vertex[ 9 ].u    = 0.0f ;
	Vertex[ 9 ].v    = 0.0f ;
	Vertex[ 9 ].su   = 0.0f ;
	Vertex[ 9 ].sv   = 0.0f ;
	//10番目
	Vertex[ 10 ].pos  = VAdd( VGet( 0.0f,  thickness, 0.0f ), CoxaJoint_POSI[5] ) ;
	Vertex[ 10 ].norm = VGet( -Fwidth,  thickness, Flength ) ;
	Vertex[ 10 ].dif  = color ;
	Vertex[ 10 ].spc  = color_spc ;
	Vertex[ 10 ].u    = 0.0f ;
	Vertex[ 10 ].v    = 0.0f ;
	Vertex[ 10 ].su   = 0.0f ;
	Vertex[ 10 ].sv   = 0.0f ;
	//11番目
	Vertex[ 11 ].pos  = VAdd( VGet( 0.0f,  -thickness, 0.0f ), CoxaJoint_POSI[5] ) ;
	Vertex[ 11 ].norm = VGet( -Fwidth, -thickness, Flength ) ;
	Vertex[ 11 ].dif  = color ;
	Vertex[ 11 ].spc  = color_spc ;
	Vertex[ 11 ].u    = 0.0f ;
	Vertex[ 11 ].v    = 0.0f ;
	Vertex[ 11 ].su   = 0.0f ;
	Vertex[ 11 ].sv   = 0.0f ;

	// ２ポリゴン分のインデックスデータをセット
	int numInd=0;
	for(int i=0; i<10; i++){
		Index[ numInd++ ] = i;
		Index[ numInd++ ] = i+1;
		Index[ numInd++ ] = i+2;
	}
	Index[ 30 ] = 10;
	Index[ 31 ] = 11;
	Index[ 32 ] = 0;

	Index[ 33 ] = 11;
	Index[ 34 ] = 0;
	Index[ 35 ] = 1;


	Index[ 36 ] = 0;
	Index[ 37 ] = 2;
	Index[ 38 ] = 4;

	Index[ 39 ] = 0;
	Index[ 40 ] = 4;
	Index[ 41 ] = 6;

	Index[ 42 ] = 0;
	Index[ 43 ] = 6;
	Index[ 44 ] = 8;

	Index[ 45 ] = 0;
	Index[ 46 ] = 8;
	Index[ 47 ] = 10;

	Index[ 48 ] = 1;
	Index[ 49 ] = 3;
	Index[ 50 ] = 5;

	Index[ 51 ] = 1;
	Index[ 52 ] = 5;
	Index[ 53 ] = 7;

	Index[ 54 ] = 1;
	Index[ 55 ] = 7;
	Index[ 56 ] = 9;

	Index[ 57 ] = 1;
	Index[ 58 ] = 9;
	Index[ 59 ] = 11;

	DrawPolygonIndexed3D_s( Vertex, 12, Index, 20, DX_NONE_GRAPH, FALSE ) ;
	for(int i=0; i<11; i+=2){
		DrawLine3D_s( Vertex[ i ].pos, Vertex[ i+1 ].pos, GetColor( 0, 0, 0 ) ) ;
		DrawLine3D_s( Vertex[ i ].pos, Vertex[ (i+2)%12 ].pos, GetColor( 0, 0, 0 ) ) ;
		DrawLine3D_s( Vertex[ i+1 ].pos, Vertex[ (i+3)%12 ].pos, GetColor( 0, 0, 0 ) ) ;
	}
}


void HexapodGraphic::DrawBody(VECTOR v0, VECTOR v1, VECTOR v2, VECTOR v3, VECTOR v4, VECTOR v5, COLOR_U8 color)
{ 
	//v0,v1,v2,v3,v4,v5 の6点を頂点とする6角形　6点は同一平面上にある
	//			        5______0
	//				    /		\		
	//				  4/		 \1	
	//				   \	 	/		
	//			      3 \ _____/2

	VECTOR Vthick, minusVthick;				//厚み方向　厚みの半分の大きさを持つ
	Vthick = VNorm(VCross(VSub(v0, v1), VSub(v0, v2)));
	Vthick = VScale(Vthick, THICKNESS);
	minusVthick = VNorm(VCross(VSub(v0, v1), VSub(v0, v2)));
	minusVthick = VScale(minusVthick, THICKNESS);

	VERTEX3D Vertex[ 12 ] ;
	WORD Index[ 20*3 ] ;
	COLOR_U8 color_spc=GetColorU8( 255,0,0,255);//RGBa aは透明度
	//0番目
	Vertex[ 0 ].pos  = VAdd(v0, Vthick);
	Vertex[ 0 ].norm = Vthick;
	Vertex[ 0 ].dif  = color ;
	Vertex[ 0 ].spc  = color_spc ;
	Vertex[ 0 ].u    = 0.0f ;
	Vertex[ 0 ].v    = 0.0f ;
	Vertex[ 0 ].su   = 0.0f ;
	Vertex[ 0 ].sv   = 0.0f ;
	//1番目
	Vertex[ 1 ].pos  = VSub(v0, Vthick);
	Vertex[ 1 ].norm = minusVthick;
	Vertex[ 1 ].dif  = color ;
	Vertex[ 1 ].spc  = color_spc ;
	Vertex[ 1 ].u    = 0.0f ;
	Vertex[ 1 ].v    = 0.0f ;
	Vertex[ 1 ].su   = 0.0f ;
	Vertex[ 1 ].sv   = 0.0f ;
	//2番目
	Vertex[ 2 ].pos  = VAdd(v1, Vthick);
	Vertex[ 2 ].norm = Vthick;
	Vertex[ 2 ].dif  = color ;
	Vertex[ 2 ].spc  = color_spc ;
	Vertex[ 2 ].u    = 0.0f ;
	Vertex[ 2 ].v    = 0.0f ;
	Vertex[ 2 ].su   = 0.0f ;
	Vertex[ 2 ].sv   = 0.0f ;
	//3番目
	Vertex[ 3 ].pos  = VSub(v1, Vthick);
	Vertex[ 3 ].norm = minusVthick;
	Vertex[ 3 ].dif  = color ;
	Vertex[ 3 ].spc  = color_spc ;
	Vertex[ 3 ].u    = 0.0f ;
	Vertex[ 3 ].v    = 0.0f ;
	Vertex[ 3 ].su   = 0.0f ;
	Vertex[ 0 ].sv   = 0.0f ;
	//4番目
	Vertex[ 4 ].pos  = VAdd(v2, Vthick);
	Vertex[ 4 ].norm = Vthick;
	Vertex[ 4 ].dif  = color ;
	Vertex[ 4 ].spc  = color_spc ;
	Vertex[ 4 ].u    = 0.0f ;
	Vertex[ 4 ].v    = 0.0f ;
	Vertex[ 4 ].su   = 0.0f ;
	Vertex[ 4 ].sv   = 0.0f ;
	//5番目
	Vertex[ 5 ].pos  = VSub(v2, Vthick);
	Vertex[ 5 ].norm = minusVthick;
	Vertex[ 5 ].dif  = color ;
	Vertex[ 5 ].spc  = color_spc ;
	Vertex[ 5 ].u    = 0.0f ;
	Vertex[ 5 ].v    = 0.0f ;
	Vertex[ 5 ].su   = 0.0f ;
	Vertex[ 5 ].sv   = 0.0f ;
	//6番目
	Vertex[ 6 ].pos  = VAdd(v3, Vthick);
	Vertex[ 6 ].norm = Vthick;
	Vertex[ 6 ].dif  = color ;
	Vertex[ 6 ].spc  = color_spc ;
	Vertex[ 6 ].u    = 0.0f ;
	Vertex[ 6 ].v    = 0.0f ;
	Vertex[ 6 ].su   = 0.0f ;
	Vertex[ 6 ].sv   = 0.0f ;
	//7番目
	Vertex[ 7 ].pos  = VSub(v3, Vthick);
	Vertex[ 7 ].norm = minusVthick;
	Vertex[ 7 ].dif  = color ;
	Vertex[ 7 ].spc  = color_spc ;
	Vertex[ 7 ].u    = 0.0f ;
	Vertex[ 7 ].v    = 0.0f ;
	Vertex[ 7 ].su   = 0.0f ;
	Vertex[ 7 ].sv   = 0.0f ;
	//8番目
	Vertex[ 8 ].pos  = VAdd(v4, Vthick);
	Vertex[ 8 ].norm = Vthick;
	Vertex[ 8 ].dif  = color ;
	Vertex[ 8 ].spc  = color_spc ;
	Vertex[ 8 ].u    = 0.0f ;
	Vertex[ 8 ].v    = 0.0f ;
	Vertex[ 8 ].su   = 0.0f ;
	Vertex[ 8 ].sv   = 0.0f ;
	//9番目
	Vertex[ 9 ].pos  = VSub(v4, Vthick);
	Vertex[ 9 ].norm = minusVthick;
	Vertex[ 9 ].dif  = color ;
	Vertex[ 9 ].spc  = color_spc ;
	Vertex[ 9 ].u    = 0.0f ;
	Vertex[ 9 ].v    = 0.0f ;
	Vertex[ 9 ].su   = 0.0f ;
	Vertex[ 9 ].sv   = 0.0f ;
	//10番目
	Vertex[ 10 ].pos  = VAdd(v5, Vthick);
	Vertex[ 10 ].norm = Vthick;
	Vertex[ 10 ].dif  = color ;
	Vertex[ 10 ].spc  = color_spc ;
	Vertex[ 10 ].u    = 0.0f ;
	Vertex[ 10 ].v    = 0.0f ;
	Vertex[ 10 ].su   = 0.0f ;
	Vertex[ 10 ].sv   = 0.0f ;
	//11番目
	Vertex[ 11 ].pos  = VSub(v5, Vthick);
	Vertex[ 11 ].norm = minusVthick;
	Vertex[ 11 ].dif  = color ;
	Vertex[ 11 ].spc  = color_spc ;
	Vertex[ 11 ].u    = 0.0f ;
	Vertex[ 11 ].v    = 0.0f ;
	Vertex[ 11 ].su   = 0.0f ;
	Vertex[ 11 ].sv   = 0.0f ;

	// ２ポリゴン分のインデックスデータをセット
	int numInd=0;
	for(int i=0; i<10; i++){
		Index[ numInd++ ] = i;
		Index[ numInd++ ] = i+1;
		Index[ numInd++ ] = i+2;
	}
	Index[ 30 ] = 10;
	Index[ 31 ] = 11;
	Index[ 32 ] = 0;

	Index[ 33 ] = 11;
	Index[ 34 ] = 0;
	Index[ 35 ] = 1;


	Index[ 36 ] = 0;
	Index[ 37 ] = 2;
	Index[ 38 ] = 4;

	Index[ 39 ] = 0;
	Index[ 40 ] = 4;
	Index[ 41 ] = 6;

	Index[ 42 ] = 0;
	Index[ 43 ] = 6;
	Index[ 44 ] = 8;

	Index[ 45 ] = 0;
	Index[ 46 ] = 8;
	Index[ 47 ] = 10;

	Index[ 48 ] = 1;
	Index[ 49 ] = 3;
	Index[ 50 ] = 5;

	Index[ 51 ] = 1;
	Index[ 52 ] = 5;
	Index[ 53 ] = 7;

	Index[ 54 ] = 1;
	Index[ 55 ] = 7;
	Index[ 56 ] = 9;

	Index[ 57 ] = 1;
	Index[ 58 ] = 9;
	Index[ 59 ] = 11;

	DrawPolygonIndexed3D_s( Vertex, 12, Index, 20, DX_NONE_GRAPH, FALSE ) ;

	for(int i=0; i<11; i+=2)
	{
		DrawLine3D_s( Vertex[ i ].pos, Vertex[ i+1 ].pos, GetColor( 0, 0, 0 ) ) ;
		DrawLine3D_s( Vertex[ i ].pos, Vertex[ (i+2)%12 ].pos, GetColor( 0, 0, 0 ) ) ;
		DrawLine3D_s( Vertex[ i+1 ].pos, Vertex[ (i+3)%12 ].pos, GetColor( 0, 0, 0 ) ) ;
	}
}


void HexapodGraphic::DrawGround()
{
	//格子線
	for (int i = -5000; i <= 5000; i+=100)
	{
		//DrawLine3D_s(VGet(i, 0.0f, 5000.0f), VGet(i, 0.0f, -5000.0f), GetColor(0, 255, 0));//yが上方向
		//DrawLine3D_s(VGet(5000.0f, 0.0f, i), VGet(-5000.0f, 0.0f, i), GetColor(0, 255, 0));
		if (i == 0) 
		{
			DrawLine3D_s(VGet((float)i, -0.0f, 5000.0f), VGet((float)i, -0.0f, -5000.0f), GetColor(0, 0, 255));//原点を通る直線は赤くする
			DrawLine3D_s(VGet(5000.0f, -0.0f, (float)i), VGet(-5000.0f, -0.0f, (float)i), GetColor(0, 0, 255));
		}
		else if (i == 400) 
		{
			DrawLine3D_s(VGet(5000.0f, -0.0f, (float)i), VGet(-5000.0f, -0.0f, (float)i), GetColor(255, 0, 255));
		}
		else if (i == 2600) 
		{
			DrawLine3D_s(VGet(5000.0f, -0.0f, (float)i), VGet(-5000.0f, -0.0f, (float)i), GetColor(255, 0, 255));
		}
		else if (i == 1200) 
		{
			DrawLine3D_s(VGet(5000.0f, -0.0f, (float)i), VGet(-5000.0f, -0.0f, (float)i), GetColor(255, 0, 0));
		}
		else 
		{
			DrawLine3D_s(VGet((float)i, -0.0f, 5000.0f), VGet((float)i, -0.0f, -5000.0f), GetColor(0, 255, 0));//yが上方向
			DrawLine3D_s(VGet(5000.0f, -0.0f, (float)i), VGet(-5000.0f, -0.0f, (float)i), GetColor(0, 255, 0));
		}
	}
	DrawTriangle3D_s(VGet(5000.0f, -1000.0f, 5000.0f), VGet(-5000.0f, -1000.0f, 5000.0f), VGet(5000.0f, -1000.0f, -5000.0f), GetColor(0, 0, 0), TRUE);
	DrawTriangle3D_s(VGet(-5000.0f, -1000.0f, -5000.0f), VGet(-5000.0f, -1000.0f, 5000.0f), VGet(5000.0f, -1000.0f, -5000.0f), GetColor(0, 0, 0), TRUE);
}

//脚接地点からなる多角形
void HexapodGraphic::DrawSupportPolygon(myvector::SVector leg_position[6], VECTOR c_o_m, float pitch, float roll, float yaw)
{
	phantomX.setMyDirection((double)pitch, (double)roll, (double)yaw);
	phantomX.setMyPosition(dxvecToMyvec(c_o_m));
	phantomX.setMyLegPosition(leg_position);

	int vertexNum = 0;
	VECTOR vertex[6];

	for(int i = 0; i < 6; i++)
	{
		if(phantomX.getLocalLegPosition(i).z < LEG_HIEGHT)
		{
			vertex[vertexNum++] = VAdd(myvecToDxvec(phantomX.getGlobalLegPos(i)),VGet(0.0,-100.0,0.0) );
		}
	}

	for(int i = 0; i < vertexNum-2; i++)
	{
		DrawTriangle3D_s(vertex[0],vertex[i+1],vertex[i+2],GetColor(0, 0, 125),TRUE);
	}
	for(int i = 0; i < vertexNum-3; i++)
	{
		DrawTriangle3D_s(vertex[1],vertex[i+2],vertex[i+3],GetColor(0, 0, 125),TRUE);
	}
	for(int i = 0; i < vertexNum-4; i++)
	{
		DrawTriangle3D_s(vertex[2],vertex[i+3],vertex[i+4],GetColor(0, 0, 125),TRUE);
	}
	for(int i = 0; i < vertexNum-5; i++)
	{
		DrawTriangle3D_s(vertex[3],vertex[i+4],vertex[i+5],GetColor(0, 0, 125),TRUE);
	}

}

//脚接地可能点の描写
void HexapodGraphic::DrawPossibleLegPosi(myvector::SVector leg_position[6])
{
	//mapを表示
	for(int i = 0; i < Define::MAPDATA3D_MAX; i++)
	{
		//設定した数までにする.設定していない脚接地点が移動する
		if (is_grounded[i]) 
		{
			DrawCone3D_s(VAdd(map[i], VGet(0, -1, 0)), map[i], FOOTHOLD_R, 8, GetColor(0, 0, 255), GetColor(255, 255, 255), TRUE);//設置した脚設置可能点は色を変える
		}
		else 
		{
			DrawCone3D_s(VAdd(map[i], VGet(0, -1, 0)), map[i], FOOTHOLD_R, 8, GetColor(255, 255, 50), GetColor(255, 255, 255), TRUE);//それ以外は黄色
			if (1180 < map[i].z && map[i].z < 1220) DrawCone3D_s(VAdd(map[i], VGet(0, -1, 0)), map[i], FOOTHOLD_R, 8, GetColor(255, 0, 0), GetColor(255, 255, 255), TRUE);//ゴール地点色変える//本当は定数を引っ張ってきた方がいい
			if (-10 < map[i].x && map[i].x < 10) DrawCone3D_s(VAdd(map[i], VGet(0, -1, 0)), map[i], FOOTHOLD_R, 8, GetColor(0, 255, 0), GetColor(255, 255, 255), TRUE);
		}
	}
	//脚がついているものは色を変えてうわがき
	VECTOR legPosi;
	for(int ileg = 0; ileg < 6; ileg++)
	{
		legPosi = myvecToDxvec(phantomX.getGlobalLegPos(ileg));

		for (int i = 0; i < Define::MAPDATA3D_MAX; i++) 
		{
			//設定した数までにする.設定していない脚接地点が移動する
			if (abs(legPosi.x - map[i].x) < 0.01 && abs(legPosi.y - map[i].y) < 0.01 && abs(legPosi.z - map[i].z) < 0.01) 
			{
				DrawCone3D_s(VAdd(legPosi, VGet(0, -1, 0)), legPosi, FOOTHOLD_R_GROUND, 8, GetColor(255, 50, 50), GetColor(255, 255, 255), TRUE);//最後がFALSEなら線//現在設置している場所は赤
				is_grounded[i] = true; //設置した場所はtrueにする
			}
		}
	}
}


void HexapodGraphic::DrawComPass(const VECTOR c_o_m)
{
	//重心移動履歴
	static VECTOR comPass[10000] = {};	//描写の限界が1000? 20201117hato
	static int comPassCnt = 0;

	if (comPassCnt < 10000) 
	{	
		if (VSize(VSub(comPass[comPassCnt], VAdd(c_o_m, VGet(0.0, 20.0, 0.0)))) > 0.1) 
		{
			//移動距離がある程度ある
			comPass[++comPassCnt] = VAdd(c_o_m, VGet(0.0, 20.0, 0.0));	//新しい重心位置
			
			//DrawLine3D_s(comPass[comPassCnt-1], comPass[comPassCnt], GetColor(255, 255, 255));//追加
			//DrawLine3D_s(VAdd(comPass[comPassCnt - 1], VGet(1, 0, 0)), VAdd(comPass[comPassCnt], VGet(1, 0, 0)), GetColor(255, 255, 255));//追加
			//DrawLine3D_s(VAdd(comPass[comPassCnt - 1], VGet(-1, 0, 0)), VAdd(comPass[comPassCnt], VGet(-1, 0, 0)), GetColor(255, 255, 255));//追加
			//DrawLine3D_s(VAdd(comPass[comPassCnt - 1], VGet(0, 0, 1)), VAdd(comPass[comPassCnt], VGet(0, 0, 1)), GetColor(255, 255, 255));//追加
			//DrawLine3D_s(VAdd(comPass[comPassCnt - 1], VGet(0, 0, -1)), VAdd(comPass[comPassCnt], VGet(0, 0, -1)), GetColor(255, 255, 255));//追加 これだと、画面更新毎に軌跡が消える。20201117hato
		}
		if (comPassCnt == 10000) 
		{
			comPassCnt = 9999;
			for (int i = 1; i < 9999; i++) 
			{
				comPass[i] = VGet(comPass[i + 1].x, comPass[i + 1].y, comPass[i + 1].z);
			}
		}
		for (int i = 0; i < comPassCnt; i++) 
		{
			if (comPass[i + 1].x < 0.001 && comPass[i + 1].y < 0.001) continue;
			if (comPass[i].x < 0.001 && comPass[i].y < 0.001)  continue;
			if (abs(comPass[i].x - comPass[i + 1].x) < 0.001 && abs(comPass[i].y - comPass[i + 1].y) < 0.001 && abs(comPass[i].z - comPass[i + 1].z) < 0.001)  break;
			DrawLine3D_s(comPass[i], comPass[i + 1], GetColor(255, 255, 255));
			DrawLine3D_s(VAdd(comPass[i], VGet(1, 0, 0)), VAdd(comPass[i + 1], VGet(1, 0, 0)), GetColor(255, 255, 255));//多分線が細くて見ずらいからちょっとずらして4本描写してる
			DrawLine3D_s(VAdd(comPass[i], VGet(-1, 0, 0)), VAdd(comPass[i + 1], VGet(-1, 0, 0)), GetColor(255, 255, 255));
			DrawLine3D_s(VAdd(comPass[i], VGet(0, 0, 1)), VAdd(comPass[i + 1], VGet(0, 0, 1)), GetColor(255, 255, 255));
			DrawLine3D_s(VAdd(comPass[i], VGet(0, 0, -1)), VAdd(comPass[i + 1], VGet(0, 0, -1)), GetColor(255, 255, 255));
		}
	}
}

void HexapodGraphic::DrawMessage()
{
	unsigned int CrW, CrR, CrBack;
	CrW = GetColor( 255 , 255 , 255 );
	CrR = GetColor( 255 , 0 , 0 );
	CrBack = GetColor( 0 , 0 , 0 );

	DrawBox( 0 , 0 , 200 , 60 ,CrBack ,  TRUE ) ;
	VECTOR legPosi;
	for (int ileg = 0; ileg < 6; ileg++) 
	{
		legPosi = myvecToDxvec(phantomX.getGlobalLegPos(ileg));
		bool isground = false;
		for (int i = 0; i < Define::MAPDATA3D_MAX; i++) 
		{
			if (abs(legPosi.x - map[i].x) < 0.01 && abs(legPosi.y - map[i].y) < 0.01 && abs(legPosi.z - map[i].z) < 0.01) 
			{
				if(ileg == 0) DrawString(100, 0, TEXT("Ground_0"), CrW);
				else if (ileg == 1) DrawString(100, 20, "Ground_1", CrW);
				else if (ileg == 2) DrawString(100, 40, "Ground_2", CrW);
				else if (ileg == 3) DrawString(0, 40, "Ground_3", CrW);
				else if (ileg == 4) DrawString(0, 20, "Ground_4", CrW);
				else DrawString(0, 0, "Ground_5", CrW);
				isground = true;
				break;
			}
		}

		if (!isground) 
		{
			if (ileg == 0) DrawString(100, 0, "Free_0", CrR);
			else if (ileg == 1) DrawString(100, 20, "Free_1", CrR);
			else if (ileg == 2) DrawString(100, 40, "Free_2", CrR);
			else if (ileg == 3) DrawString(0, 40, "Free_3", CrR);
			else if (ileg == 4) DrawString(0, 20, "Free_4", CrR);
			else DrawString(0, 0, "Free_5", CrR);
		}
	}
}

void HexapodGraphic::DrowTawara_s( VECTOR center, VECTOR axle, float r1, float r2, float tawaraCoefficient, COLOR_U8 color)
{
	VECTOR V_r[16], dr;

	V_r[0] = VNorm( VCross(axle, VAdd(axle, VGet(1.0, 1.0, 1.0))) );
	
	for(int i = 0; i < 16 - 1; i++)
	{
		dr = VScale(VNorm(VCross(V_r[i], axle)), tan(2.0f*3.1415f/16.0f));
		V_r[i+1] = VNorm(VAdd(V_r[i], dr));
	}

	VERTEX3D Vertex[66];
	COLOR_U8 color_spc = GetColorU8(255, 0, 0, 0);//RGBa aは透明度

	//0番目
	Vertex[ 0 ].pos  = VAdd(center, axle);
	Vertex[ 0 ].norm = VNorm(axle);
	Vertex[ 0 ].dif  = color ;
	Vertex[ 0 ].spc  = color_spc ;
	Vertex[ 0 ].u    = 0.0f ;
	Vertex[ 0 ].v    = 0.0f ;
	Vertex[ 0 ].su   = 0.0f ;
	Vertex[ 0 ].sv   = 0.0f ;
	//65番目
	Vertex[ 65 ].pos  = VSub(center, axle);
	Vertex[ 65 ].norm = VNorm(VScale(axle,-1.0));
	Vertex[ 65 ].dif  = color ;
	Vertex[ 65 ].spc  = color_spc ;
	Vertex[ 65 ].u    = 0.0f ;
	Vertex[ 65 ].v    = 0.0f ;
	Vertex[ 65 ].su   = 0.0f ;
	Vertex[ 65 ].sv   = 0.0f ;

	for(int i = 0; i < 16; i++)
	{
		//1番目
		Vertex[ i*4+1 ].pos  = VAdd( VAdd(center, axle), VScale(V_r[i], r2));
		Vertex[ i*4+1 ].norm = VNorm(axle);
		Vertex[ i*4+1 ].dif  = color ;
		Vertex[ i*4+1 ].spc  = color_spc ;
		Vertex[ i*4+1 ].u    = 0.0f ;
		Vertex[ i*4+1 ].v    = 0.0f ;
		Vertex[ i*4+1 ].su   = 0.0f ;
		Vertex[ i*4+1 ].sv   = 0.0f ;
		//2番目
		Vertex[ i*4+2 ].pos  = VAdd( VAdd(center, VScale(axle, tawaraCoefficient)), VScale(V_r[i], r1));
		Vertex[ i*4+2 ].norm = V_r[i];
		Vertex[ i*4+2 ].dif  = color ;
		Vertex[ i*4+2 ].spc  = color_spc ;
		Vertex[ i*4+2 ].u    = 0.0f ;
		Vertex[ i*4+2 ].v    = 0.0f ;
		Vertex[ i*4+2 ].su   = 0.0f ;
		Vertex[ i*4+2 ].sv   = 0.0f ;
		//3番目
		Vertex[ i*4+3 ].pos  = VAdd(  VSub(center, VScale(axle, tawaraCoefficient)), VScale(V_r[i], r1));
		Vertex[ i*4+3 ].norm = V_r[i];
		Vertex[ i*4+3 ].dif  = color ;
		Vertex[ i*4+3 ].spc  = color_spc ;
		Vertex[ i*4+3 ].u    = 0.0f ;
		Vertex[ i*4+3 ].v    = 0.0f ;
		Vertex[ i*4+3 ].su   = 0.0f ;
		Vertex[ i*4+0 ].sv   = 0.0f ;
		//4番目
		Vertex[ i*4+4 ].pos  = VAdd( VSub(center, axle), VScale(V_r[i], r2));
		Vertex[ i*4+4 ].norm = VNorm(VScale(axle,-1.0));
		Vertex[ i*4+4 ].dif  = color ;
		Vertex[ i*4+4 ].spc  = color_spc ;
		Vertex[ i*4+4 ].u    = 0.0f ;
		Vertex[ i*4+4 ].v    = 0.0f ;
		Vertex[ i*4+4 ].su   = 0.0f ;
		Vertex[ i*4+4 ].sv   = 0.0f ;
	}

	WORD Index[ 8*16*3 ] ;

	// ポリゴンのインデックスデータをセット
	int numInd=0;
	
	for(int i=1; i<=61; i += 4)
	{
		Index[ numInd++ ] = 0;
		Index[ numInd++ ] = i;
		Index[ numInd++ ] = (i+4)%64;

		Index[ numInd++ ] = i;
		Index[ numInd++ ] = i+1;
		Index[ numInd++ ] = (i+4)%64;

		Index[ numInd++ ] = i+1;
		Index[ numInd++ ] = i+2;
		Index[ numInd++ ] = (i+5)%64;

		Index[ numInd++ ] = i+2;
		Index[ numInd++ ] = i+3;
		Index[ numInd++ ] = (i+6)%64;

		Index[ numInd++ ] = (i+4)%64;
		Index[ numInd++ ] = (i+5)%64;
		Index[ numInd++ ] = i+1;

		Index[ numInd++ ] = (i+5)%64;
		Index[ numInd++ ] = (i+6)%64;
		Index[ numInd++ ] = i+2;

		Index[ numInd++ ] = (i+6)%64;
		Index[ numInd++ ] = max((i+7)%64, (i+7)%65);
		Index[ numInd++ ] = i+3;

		Index[ numInd++ ] = i+3;
		Index[ numInd++ ] = max((i+7)%64, (i+7)%65);
		Index[ numInd++ ] = 65;
	}

	DrawPolygonIndexed3D_s( Vertex, 66, Index, numInd/3, DX_NONE_GRAPH, FALSE ) ;
}

void HexapodGraphic::DrawLine3D_s( VECTOR Pos1, VECTOR Pos2, unsigned int Color )
{
	DrawLine3D( VScale(Pos1, MAGNIFICATION), VScale(Pos2, MAGNIFICATION), Color );
}

void HexapodGraphic::DrawTriangle3D_s(  VECTOR Pos1, VECTOR Pos2, VECTOR Pos3, unsigned int Color, int FillFlag )
{
	DrawTriangle3D(  VScale(Pos1, MAGNIFICATION) , VScale(Pos2, MAGNIFICATION), VScale(Pos3, MAGNIFICATION), Color, FillFlag );
}

void HexapodGraphic::DrawCone3D_s( VECTOR TopPos, VECTOR BottomPos, float r, int DivNum, unsigned int DifColor, unsigned int SpcColor, int FillFlag )
{
	DrawCone3D( VScale(TopPos, MAGNIFICATION), VScale(BottomPos, MAGNIFICATION), r * MAGNIFICATION, DivNum,  DifColor, SpcColor, FillFlag );
}

void HexapodGraphic::DrawCapsule3D_s( VECTOR Pos1, VECTOR Pos2, float r, int DivNum, unsigned int DifColor, unsigned int SpcColor, int FillFlag )
{
	DrawCapsule3D( VScale(Pos1, MAGNIFICATION), VScale(Pos2, MAGNIFICATION), r * MAGNIFICATION, DivNum, DifColor, SpcColor, FillFlag );
}

void HexapodGraphic::DrawSphere3D_s( VECTOR CenterPos, float r, int DivNum, unsigned int DifColor, unsigned int SpcColor, int FillFlag )
{
	DrawSphere3D( VScale(CenterPos, MAGNIFICATION), r * MAGNIFICATION, DivNum, DifColor, SpcColor, FillFlag );
}

void HexapodGraphic::DrawPolygonIndexed3D_s( VERTEX3D *VertexIn, int VertexNum, unsigned short *Indices, int PolygonNum, int GrHandle, int TransFlag )
{
	//100は適当　汎用性を持たせる気がない　増やす必要があるなら増やして
	VERTEX3D Vertex[100];	

	for(int i = 0; i < VertexNum; i++)
	{
		Vertex[i] = VertexIn[i];
		Vertex[i].pos = VScale(Vertex[i].pos, MAGNIFICATION);
	}
	
	DrawPolygonIndexed3D(Vertex, VertexNum, Indices, PolygonNum, GrHandle, TransFlag );
}

void HexapodGraphic::DrawRangeOfDirection(VECTOR c_o_m, float pitch, float roll, float yaw)
{	
	//カメラによる検出範囲
	int trianglenum = 8;
	VECTOR vertex[11];//三角形の数+1以上
	MATRIX rotationZ;
	float x[11],y[11],z[11];
	float Start = (float)DX_PI / 4.0f;//円弧最初
	float End = (float)DX_PI / 4.0f * 3.0f;//円弧最後
	float dth;
	rotationZ = MGetRotY(-yaw);

	//ロボット中心
	x[0] = c_o_m.x;
	y[0] = 0.0;
	z[0] = c_o_m.z;
	vertex[0] = VGet(x[0], y[0], z[0]);

	dth = (End - Start) / float(trianglenum);
	//三角形の頂点座標を計算
	for(int i = 1; i <= trianglenum + 1; i++)
	{
		x[i] = 500.0f * cos(Start + dth * float(i - 1));
		y[i] = 1.0f;
		z[i] = 500.0f * sin(Start + dth * float(i - 1));
		vertex[i] = VAdd(VTransform(VGet(x[i], y[i], z[i]), rotationZ),vertex[0]);
	}
	SetDrawBlendMode(DX_BLENDMODE_ALPHA, 128);//透過
	for(int i = 1; i <= trianglenum; i++)
	{
		DrawTriangle3D(VScale(vertex[0], MAGNIFICATION), VScale(vertex[i], MAGNIFICATION), VScale(vertex[i + 1], MAGNIFICATION), GetColor(255, 0, 255), true);
	}
	SetDrawBlendMode(DX_BLENDMODE_ALPHA, 255);//透過なし
}