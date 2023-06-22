#include "phantomxCommander.h"

phantomxCommander::phantomxCommander()
{
	presentTime = 1000;
	commandnum = 98;
	lifted_leg = 0;
	time_initialize(&commandTime);

	endpoints[RIGHT_FRONT] = my_vec::VGet(FRL_ALIGNMENT_POSE_X, FRL_ALIGNMENT_POSE_Y, BODY_HEIGHT);
	endpoints[RIGHT_MIDDLE] = my_vec::VGet(ML_ALIGNMENT_POSE_X, ML_ALIGNMENT_POSE_Y, BODY_HEIGHT);
	endpoints[RIGHT_REAR] = my_vec::VGet(-FRL_ALIGNMENT_POSE_X, FRL_ALIGNMENT_POSE_Y, BODY_HEIGHT);
	endpoints[LEFT_REAR] = my_vec::VGet(-FRL_ALIGNMENT_POSE_X, -FRL_ALIGNMENT_POSE_Y, BODY_HEIGHT);
	endpoints[LEFT_MIDDLE] = my_vec::VGet(ML_ALIGNMENT_POSE_X, -ML_ALIGNMENT_POSE_Y, BODY_HEIGHT);
	endpoints[LEFT_FRONT] = my_vec::VGet(FRL_ALIGNMENT_POSE_X, -FRL_ALIGNMENT_POSE_Y, BODY_HEIGHT);

	baudRate = DEFAULT_BAUDRATE;
	/* ----------------------------------------------
	ファイルのクリエイトとオープン
	---------------------------------------------- */
	// クリエイトしたファイルのファイルハンドラを返す
	h = CreateFile("\\\\.\\COM3", GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL , NULL); 
	if ( h == INVALID_HANDLE_VALUE ) 
	{
		std::cout << "シリアルルートの確保に失敗\n";					//シリアルルートの確保に失敗
		system("pause");
		exit(1);
	}
	SetupComm( h, 1024, 1024 );								//バッファーを作る
	PurgeComm( h, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR );		//バッファーのクリア
	/* ----------------------------------------------
	シリアルポートの状態
	---------------------------------------------- */
	GetCommState ( h, &dcb ); // シリアルポートの状態を取得

	std::cout<<"シリアルポートの状態"<<"\t\t\t\n";
	std::cout<<"dcb.DCBlength="<<"\t\t\t"<<dcb.DCBlength<<"\n";
	std::cout<<"dcb.BaudRate="<<"\t\t\t"<<dcb.BaudRate<<"\n";
	std::cout<<"dcb.fBinary="<<"\t\t\t"<<dcb.fBinary<<"\n";
	std::cout<<"dcb.fParity="<<"\t\t\t"<<dcb.fParity<<"\n";
	std::cout<<"dcb.fOutxCtsFlow="<<"\t\t"<<dcb.fOutxCtsFlow<<"\n";
	std::cout<<"dcb.fOutxDsrFlow="<<"\t\t"<<dcb.fOutxDsrFlow<<"\n";
	std::cout<<"dcb.fDtrControl="<<"\t\t"<<dcb.fDtrControl<<"\n";
	std::cout<<"dcb.fDsrSensitivity="<<"\t\t"<<dcb.fDsrSensitivity<<"\n";
	std::cout<<"dcb.fTXContinueOnXoff="<<"\t\t"<<dcb.fTXContinueOnXoff<<"\n";
	std::cout<<"dcb.fOutX="<<"\t\t\t"<<dcb.fOutX<<"\n";
	std::cout<<"dcb.fInX="<<"\t\t\t"<<dcb.fInX<<"\n";
	std::cout<<"dcb.fErrorChar="<<"\t\t\t"<<dcb.fErrorChar<<"\n";
	std::cout<<"dcb.fNull="<<"\t\t\t"<<dcb.fNull<<"\n";
	std::cout<<"dcb.fRtsControl="<<"\t\t"<<dcb.fRtsControl<<"\n";
	std::cout<<"dcb.fAbortOnError="<<"\t\t"<<dcb.fAbortOnError<<"\n";
	std::cout<<"dcb.fDummy2="<<"\t\t\t"<<dcb.fDummy2<<"\n";
	std::cout<<"dcb.wReserved="<<"\t\t\t"<<dcb.wReserved<<"\n";
	std::cout<<"dcb.XonLim="<<"\t\t\t"<<dcb.XonLim<<"\n";
	std::cout<<"dcb.XoffLim="<<"\t\t\t"<<dcb.XoffLim<<"\n";
	std::cout<<"dcb.ByteSize="<<"\t\t\t"<<dcb.ByteSize<<"\n";
	std::cout<<"dcb.Parity="<<"\t\t\t"<<dcb.Parity<<"\n";
	std::cout<<"dcb.StopBits="<<"\t\t\t"<<dcb.StopBits<<"\n";
	std::cout<<"dcb.XonChar="<<"\t\t\t"<<dcb.XonChar<<"\n";
	std::cout<<"dcb.XoffChar="<<"\t\t\t"<<dcb.XoffChar<<"\n";
	std::cout<<"dcb.ErrorChar="<<"\t\t\t"<<dcb.ErrorChar<<"\n";
	std::cout<<"dcb.EofChar="<<"\t\t\t"<<dcb.EofChar<<"\n";
	std::cout<<"dcb.EvtChar="<<"\t\t\t"<<dcb.EvtChar<<"\n";
	std::cout<<"dcb.wReserved1="<<"\t\t\t"<<dcb.wReserved1<<"\n";

	std::cout<<"\nPlease start up PhantomX!\n";

	dcb.ByteSize =8;
	dcb.XonChar =0;
	dcb.XoffChar =0;
	dcb.EofChar =4;
	dcb.EvtChar =10;
	dcb.BaudRate = baudRate;

	SetCommState( h, &dcb ); // シリアルポートの状態を設定
	SetCommMask( h, EV_RXCHAR );
	/* ----------------------------------------------
	シリアルポートのタイムアウト状態?作
	---------------------------------------------- */
	GetCommTimeouts( h, &cto ); // タイムアウトの設定状態を取得
	//cto.ReadIntervalTimeout = 1000;
	cto.ReadIntervalTimeout = 100;
	cto.ReadTotalTimeoutMultiplier = 100;
	//cto.ReadTotalTimeoutMultiplier = 50;
	cto.ReadTotalTimeoutConstant = 100;
	cto.WriteTotalTimeoutMultiplier = 0;
	cto.WriteTotalTimeoutConstant = 0;
	//cto.WriteTotalTimeoutMultiplier = 100;
	//cto.WriteTotalTimeoutConstant = 1000;
	SetCommTimeouts( h, &cto ); // タイムアウトの状態を設定
	/* ----------------------------------------------
	受信データの読み込み（１行分の文字列）
	---------------------------------------------- */
	for(int i=0;i<50;i++)pszBuf[i]=NULL;		//バッファの初期化
//	while(pszBuf[0] == NULL)ReadFile(h, pszBuf, 50, &dwRead, 0);	//hから pszBuf に50バイト読み込み dwRead が実際に読み込んだバイト数
	std::cout<<"\n///////////PhantomX message////////////\n"<<pszBuf<<"///////////////////////////////////////\n"<<"\n";
	std::cout << "Please wait. PhantomX is standing up.\n";
}

double phantomxCommander::myRound(double r)
{
	if(r < 0){
		r -= 0.5;
		r = ceil(r);
	}
	else{
		r += 0.5;
		r = floor(r);
	}
	return r;
}

void phantomxCommander::sendEndPoints(int movingTime)
{
	int command_length;					//PCからPhantomXに送るデータの長さ[Byte]
	int writeErrorFlag;
	DWORD errorCode;
	command[0]=0xfe;					//データの始まり
	command[1]=0xff;					//
	command[2]=write_serial;			//ASCIIの'w'　書き込みの合図
	command[3]=movingTime & 0xff;		//脚先移動時間の下位8bit
	command[4]=movingTime >> 8 & 0xff;	//脚先移動時間の上位8bit
	command[5]=LEG_COUNT * 7 + 1;		//書き込むコマンドの長さ。（脚の数*（ID(1Byte)+x座標(2Byte)+y座標(2Byte)+z座標(2Byte)) + CheckSum2(1Byte))
	command[6]=((command[2]+command[3]+command[4]+command[5]) ^ 0xff) & 0xff;	//CheckSum1
	for(int i = 0; i< LEG_COUNT; i++)
	{
		command[i*7+7] = i;								//脚のID
		command[i*7+8] = (int)myRound(endpoints[i].x) & 0xff;			//脚先位置のx座標の下位8bit
		command[i*7+9] = (int)myRound(endpoints[i].x) >> 8 & 0xff;	//脚先位置のx座標の上位8bit
		command[i*7+10] = (int)myRound(endpoints[i].y) & 0xff;		//脚先位置のy座標の下位8bit
		command[i*7+11] = (int)myRound(endpoints[i].y) >> 8 & 0xff;	//脚先位置のy座標の上位8bit
		command[i*7+12] = (int)myRound(endpoints[i].z) & 0xff;		//脚先位置のz座標の下位8bit
		command[i*7+13] = (int)myRound(endpoints[i].z) >> 8 & 0xff;	//脚先位置のz座標の上位8bit
	}
	command_length = command[5] + 7;

	int commandsum = 0;
	
	for(int k=2;k<command_length-1;k++)commandsum+=command[k];
	command[command_length-1]=(255 - (commandsum)%256);

	for(int i=0;i<command_length;i++)
	{
		writeErrorFlag = WriteFile( h, &command[i], 1, &nn, 0 ); // シリアルポートに対する書き込み
		//printf("%d\t%x\n",i,command[i]);
		if(!writeErrorFlag)
		{
			errorCode = GetLastError();
			std::cout << "Write file error! : " << errorCode << "\n";
		}
	}
	std::cout << "I had sent endpoints.\n";
	presentTime = movingTime;
}

void phantomxCommander::loopSendEndPoints(int loopNum, int movingTime, int waitTime)
{
	//waitTimeは200ms以上を推奨．それ以下は暴走の危険性あり．
	if(waitTime<0)
	{
		int w;
		w = -waitTime;
		waitTime = w;
	}
	for(int i = 0; i < loopNum; i++)
	{
		sendEndPoints(movingTime);
		time_start(&commandTime);
		time_wait(&commandTime, (DWORD)waitTime);
	}
}

void phantomxCommander::SecurelySendEndPoints(int movingTime, int waitTime)
{
	//waitTimeは200ms以上を推奨．それ以下は暴走の危険性あり．

	for(int i=0;i<50;i++)pszBuf[i]=NULL;		//バッファの初期化

	if(waitTime<0)
	{
		int w;
		w = -waitTime;
		waitTime = w;
	}
	while(pszBuf[0] == NULL)
	{
		sendEndPoints(movingTime);
		time_start(&commandTime);
		time_wait(&commandTime, (DWORD)waitTime);
		std::cout << "Please wait. SendEndPoints.\n";
		ReadFile(h, pszBuf, 50, &dwRead, 0);	//hから pszBuf に読み込み dwRead が実際に読み込んだバイト数
		std::cout<<"\n///////////PhantomX message////////////\n"<<pszBuf<<"///////////////////////////////////////\n"<<"\n";
		if(dwRead == 0)std::cout << "へんじがない。\nただの　しかばねのようだ…。\n";
	}
}

void phantomxCommander::demonstration1()
{
	int i;
	std::cout << "Demonstration1.Move forward with tripod gait.\n";
	for(i = 0; i < 20; i++)
	{
		lifted_leg = move1Cycle2(0.0, 25.0);
		time_start(&commandTime);
		time_wait(&commandTime, 600);
	}
	
}

int phantomxCommander::move1Cycle2(double rads, double stride)
{
	int step_x, step_y;
	step_x = (int)myRound(stride * cos(rads));
	step_y = (int)myRound(stride * sin(rads));
	int movingTime, waitTime;
	movingTime = 500;
	waitTime = 200;

	std::cout << "Running move1Cycle ver.2.\n";
	
	if(lifted_leg == 0)
	{
		//脚グループ1
		endpoints[RIGHT_FRONT].x = FRL_ALIGNMENT_POSE_X;
		endpoints[RIGHT_FRONT].y = FRL_ALIGNMENT_POSE_Y;
		endpoints[RIGHT_FRONT].z = BODY_HEIGHT - LIFT_HEIGHT;
		endpoints[RIGHT_REAR].x = -FRL_ALIGNMENT_POSE_X;
		endpoints[RIGHT_REAR].y = FRL_ALIGNMENT_POSE_Y;
		endpoints[RIGHT_REAR].z = BODY_HEIGHT - LIFT_HEIGHT;
		endpoints[LEFT_MIDDLE].x = ML_ALIGNMENT_POSE_X;
		endpoints[LEFT_MIDDLE].y = -ML_ALIGNMENT_POSE_Y;
		endpoints[LEFT_MIDDLE].z = BODY_HEIGHT - LIFT_HEIGHT;
		//脚グループ2
		endpoints[LEFT_FRONT].x = FRL_ALIGNMENT_POSE_X;
		endpoints[LEFT_FRONT].y = -FRL_ALIGNMENT_POSE_Y;
		endpoints[LEFT_FRONT].z = BODY_HEIGHT;
		endpoints[LEFT_REAR].x = -FRL_ALIGNMENT_POSE_X;
		endpoints[LEFT_REAR].y = -FRL_ALIGNMENT_POSE_Y;
		endpoints[LEFT_REAR].z = BODY_HEIGHT;
		endpoints[RIGHT_MIDDLE].x = ML_ALIGNMENT_POSE_X;
		endpoints[RIGHT_MIDDLE].y = ML_ALIGNMENT_POSE_Y;
		endpoints[RIGHT_MIDDLE].z = BODY_HEIGHT;
		
		//sendEndPoints(movingTime / 2);
		loopSendEndPoints(3, movingTime/2, waitTime);
		lifted_leg = 1;
		//Sleep(600);
		time_start(&commandTime);
		time_wait(&commandTime, (DWORD)movingTime/2 + waitTime);
	}
	if(lifted_leg == 1)
	{
		//脚グループ1を進行方向に出す。脚グループ2を進行方向と逆方向に出す。
		//脚グループ1
		endpoints[RIGHT_FRONT].x = FRL_ALIGNMENT_POSE_X + (float)step_x;
		endpoints[RIGHT_FRONT].y = FRL_ALIGNMENT_POSE_Y + (float)step_y;
		endpoints[RIGHT_FRONT].z = BODY_HEIGHT;
		endpoints[RIGHT_REAR].x = -FRL_ALIGNMENT_POSE_X + (float)step_x;
		endpoints[RIGHT_REAR].y = FRL_ALIGNMENT_POSE_Y + (float)step_y;
		endpoints[RIGHT_REAR].z = BODY_HEIGHT;
		endpoints[LEFT_MIDDLE].x = ML_ALIGNMENT_POSE_X + (float)step_x;
		endpoints[LEFT_MIDDLE].y = -ML_ALIGNMENT_POSE_Y + (float)step_y;
		endpoints[LEFT_MIDDLE].z = BODY_HEIGHT;
		//脚グループ2
		endpoints[LEFT_FRONT].x = FRL_ALIGNMENT_POSE_X - (float)step_x;
		endpoints[LEFT_FRONT].y = -FRL_ALIGNMENT_POSE_Y - (float)step_y;
		endpoints[LEFT_FRONT].z = BODY_HEIGHT;
		endpoints[LEFT_REAR].x = -FRL_ALIGNMENT_POSE_X - (float)step_x;
		endpoints[LEFT_REAR].y = -FRL_ALIGNMENT_POSE_Y - (float)step_y;
		endpoints[LEFT_REAR].z = BODY_HEIGHT;
		endpoints[RIGHT_MIDDLE].x = ML_ALIGNMENT_POSE_X - (float)step_x;
		endpoints[RIGHT_MIDDLE].y = ML_ALIGNMENT_POSE_Y - (float)step_y;
		endpoints[RIGHT_MIDDLE].z = BODY_HEIGHT;
		
		//sendEndPoints(movingTime);
		loopSendEndPoints(3, movingTime, waitTime);
		//lifted_leg = 0;
		//Sleep(600);
		time_start(&commandTime);
		time_wait(&commandTime, (DWORD)movingTime + waitTime);

		//脚グループ2を遊脚。
		//脚グループ
		endpoints[LEFT_FRONT].z = BODY_HEIGHT - LIFT_HEIGHT;
		endpoints[LEFT_REAR].z = BODY_HEIGHT - LIFT_HEIGHT;
		endpoints[RIGHT_MIDDLE].z = BODY_HEIGHT - LIFT_HEIGHT;
		
		//sendEndPoints(movingTime);
		loopSendEndPoints(3, movingTime, waitTime);
		lifted_leg = 2;
		//Sleep(600);
		time_start(&commandTime);
		time_wait(&commandTime, (DWORD)movingTime + waitTime);
	}
	else if(lifted_leg == 2)
	{
		//脚グループ2を進行方向に出す。脚グループ1を進行方向と逆方向に出す。
		//脚グループ1
		endpoints[RIGHT_FRONT].x = FRL_ALIGNMENT_POSE_X - (float)step_x;
		endpoints[RIGHT_FRONT].y = FRL_ALIGNMENT_POSE_Y - (float)step_y;
		endpoints[RIGHT_FRONT].z = BODY_HEIGHT;
		endpoints[RIGHT_REAR].x = -FRL_ALIGNMENT_POSE_X - (float)step_x;
		endpoints[RIGHT_REAR].y = FRL_ALIGNMENT_POSE_Y - (float)step_y;
		endpoints[RIGHT_REAR].z = BODY_HEIGHT;
		endpoints[LEFT_MIDDLE].x = ML_ALIGNMENT_POSE_X - (float)step_x;
		endpoints[LEFT_MIDDLE].y = -ML_ALIGNMENT_POSE_Y - (float)step_y;
		endpoints[LEFT_MIDDLE].z = BODY_HEIGHT;
		//脚グループ2
		endpoints[LEFT_FRONT].x = FRL_ALIGNMENT_POSE_X + (float)step_x;
		endpoints[LEFT_FRONT].y = -FRL_ALIGNMENT_POSE_Y + (float)step_y;
		endpoints[LEFT_FRONT].z = BODY_HEIGHT;
		endpoints[LEFT_REAR].x = -FRL_ALIGNMENT_POSE_X + (float)step_x;
		endpoints[LEFT_REAR].y = -FRL_ALIGNMENT_POSE_Y + (float)step_y;
		endpoints[LEFT_REAR].z = BODY_HEIGHT;
		endpoints[RIGHT_MIDDLE].x = ML_ALIGNMENT_POSE_X + (float)step_x;
		endpoints[RIGHT_MIDDLE].y = ML_ALIGNMENT_POSE_Y + (float)step_y;
		endpoints[RIGHT_MIDDLE].z = BODY_HEIGHT;
		
		//sendEndPoints(movingTime);
		loopSendEndPoints(3, movingTime, waitTime);
		//lifted_leg = 0;
		//Sleep(600);
		time_start(&commandTime);
		time_wait(&commandTime, (DWORD)movingTime + waitTime);

		//脚グループ1を遊脚。
		//脚グループ1
		endpoints[RIGHT_FRONT].z = BODY_HEIGHT - LIFT_HEIGHT;
		endpoints[RIGHT_REAR].z = BODY_HEIGHT - LIFT_HEIGHT;
		endpoints[LEFT_MIDDLE].z = BODY_HEIGHT - LIFT_HEIGHT;
		
		//sendEndPoints(movingTime);
		loopSendEndPoints(3, movingTime, waitTime);
		lifted_leg = 1;
		//Sleep(600);
		time_start(&commandTime);
		time_wait(&commandTime, (DWORD)movingTime + waitTime);
	}
	return lifted_leg;
}