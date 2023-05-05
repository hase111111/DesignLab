#ifndef __DetectLandingPoint_H__
#define __DetectLandingPoint_H__

#include "pch.h"
#include <opencv2/opencv.hpp>
#include "vectorFunc.h"

#define DEG45 0.78539816339745
#define DEG40 0.69813170079773
#define DEG30 0.52359877559830
//#define TILE_SIZE	50.0f
#define TILE_SIZE	25.0f
//#define MESH_SIZE 11
//#define MESH_SIZE 12
#define MESH_SIZE	12.5f
//#define MESH_SIZE	6.25f
//#define MESH_SIZE 25
//#define MAP_MESH	25.0f
//#define MAP_MESH	40.0f
#define MAP_MESH	50.0f
#define FRAMESIZE_W 640
#define FRAMESIZE_H 480
//#define BODYHEIGHT	100
#define BODYHEIGHT	110
//#define BODYHEIGHT	120
//#define BODYHEIGHT	140
//#define BODYHEIGHT	150
//#define BODYHEIGHT	153
//#define BODYHEIGHT	0
#define DETECT_LIMIT_L	540
//#define DETECT_LIMIT_X	550
#define DETECT_LIMIT_X_MAX	500
#define DETECT_LIMIT_X_MIN	200
#define DETECT_LIMIT_Y	350
#define ERROR_LIMIT_Z	20
//#define ERROR_LIMIT_Z	10
#define MAP_SIZE 4000
#define ROI_SIZE 1000
#define EXISTENCE_TILE 0.75f
#define BASE_LINE 100.0f

static const char *CAM1C01 = "http://IO-WLCAM:7u3Mp0ACYPxamY@192.168.0.61:1226/snapshot.cgi?type=.mjpg";		//カメラ名：CAM-1C01のアドレス
//static const char *CAM1C01 = "http://IO-WLCAM:7u3Mp0ACYPxamY@192.168.11.61:1226/snapshot.cgi?type=.mjpg";	//カメラ名：CAM-1C01のアドレス（製図室の場合）
static const char *CAM1C10 = "http://IO-WLCAM:7u3Mp0ACYPxamY@192.168.0.62:4226/snapshot.cgi?type=.mjpg";		//カメラ名：CAM-1C10のアドレス
//static const char *CAM1C10 = "http://IO-WLCAM:7u3Mp0ACYPxamY@192.168.11.62:4226/snapshot.cgi?type=.mjpg";	//カメラ名：CAM-1C10のアドレス（製図室の場合）
static const cv::String MHI = "C:/Users/tasaki/Documents/Visual Studio 2010/Projects/detectLandingPoint/detectLandingPoint/MHI.png";
static const cv::String dragonBall = "C:/Users/tasaki/Documents/Visual Studio 2010/Projects/detectLandingPoint/detectLandingPoint/dragonBall.png";
static const cv::String pcBag = "C:/Users/tasaki/Documents/Visual Studio 2010/Projects/detectLandingPoint/detectLandingPoint/pcBag.png";
static const cv::String room509 = "C:/Users/tasaki/Documents/Visual Studio 2010/Projects/detectLandingPoint/detectLandingPoint/room509.png";
static const cv::String smaPho = "C:/Users/tasaki/Documents/Visual Studio 2010/Projects/detectLandingPoint/detectLandingPoint/smaPho.png";
static const cv::String hand = "C:/Users/tasaki/Documents/Visual Studio 2010/Projects/detectLandingPoint/detectLandingPoint/hand.png";
static const cv::String box = "C:/Users/tasaki/Documents/Visual Studio 2010/Projects/detectLandingPoint/detectLandingPoint/box.png";
static const cv::String redTile = "C:/Users/tasaki/Documents/Visual Studio 2010/Projects/detectLandingPoint/detectLandingPoint/redTile.png";
static const cv::String greenTile = "C:/Users/tasaki/Documents/Visual Studio 2010/Projects/detectLandingPoint/detectLandingPoint/greenTile.png";



class detectLandingPoint
{
public:
	//デフォルトコンストラクタ
	detectLandingPoint();

	//カメラの初期化
	int initializeCamera();
	//カメラストリームの表示
	void streamingCamera();
	//歪み補正と平行化変換
	void remapImage(cv::Mat* input_leftImg, cv::Mat* input_rightImg);
	//テンプレート画像の取得
	int getTemplateImage(cv::Mat* input_img, cv::Rect* selectedRoi, cv::Mat* output_img);
	//ヒストグラムの計算
	int calTempHist(cv::Mat* input_img, cv::Mat* output_hist);
	//頻出色値を求める
	int* getFrequentVal(cv::Mat* input_hist, int* frequentHue, int* frequentSat);
	int* getFrequentVal(cv::Mat* input_hist, cv::Vec2i* frequentColor);
	//色距離の計算
	int calColorDistance(cv::Mat* input_img, int* frequentHue, int* frequentSat, cv::Mat* output_hueDistance, cv::Mat* output_satDistance);
	int calColorDistance(cv::Mat* input_img, cv::Vec2i* frequentColor, cv::Mat* output_hueDistance, cv::Mat* output_satDistance);
	//二値化画像の作成
	int makeBinaryImage(cv::Mat* input_hueDistance, cv::Mat* input_satDistance, int hueThreshold, int satThreshold, cv::Mat* output_binaryImage);
	//二値化画像をメッシュに区切る
	int makeMeshImage(cv::Mat* input_img, cv::Point3d cameraPosition, double radsOfCameraDip, std::vector<cv::Point2d>* tilePosition2d);
	//カメラ画像中のタイルの座標の取得
	int tilePositionOnImage(cv::Mat* input_meshedImage, std::vector<cv::Point2d>* tilePosition2d);
	//ステレオマッチング
	int stereoMatching(cv::Mat* input_cameraFrameLeft, cv::Mat* input_cameraFrameRight, cv::Mat* output_disparty);
	//三次元再投影
	int reproject3dPosition(cv::Mat* input_disparty, cv::Mat* output_3dImage);
	//ロボット座標系におけるタイルの三次元座標を求める
	int coordinateOfTile3d(cv::Mat* input_3dImageOnCamera, std::vector<cv::Point2d>* tilePosition2d, cv::Point3d cameraPosition, double radsOfCameraDip, std::vector<cv::Point3d>* output_tilePosition3d);
	//脚接地候補点座標の記憶
	int saveLnadingPoint(std::vector<cv::Point3d>* input_newLandingPoint, std::vector<cv::Point3d>* allLandingPoint, cv::Mat* landingPointMap);
	//脚接地候補点座標の記憶(半径)
	int saveLnadingPointRound(std::vector<cv::Point3d>* input_newLandingPoint, std::vector<cv::Point3d>* allLandingPoint, cv::Mat* landingPointMap);
	//脚接地候補点座標の再計算
	int recalLnadingPoint(std::vector<cv::Point3d>* allLandingPoint, myvector::VECTOR* movementAmount);
	int recalLnadingPoint2(std::vector<cv::Point3d>* allLandingPoint, myvector::VECTOR MovementAmount, myvector::VECTOR Centerposition, myvector::VECTOR Robotposture);
	int rotationLnadingPoint(std::vector<cv::Point3d>* allLandingPoint, myvector::VECTOR* rotationAmount, myvector::VECTOR Centerposition);
	//脚接地候補点座標の受け渡し
	int sendLandingPoint(cv::Mat* landingPointMap, std::vector<myvector::VECTOR>* output_landingPoint);
	//脚接地候補点の描画
	void showLandingPoint(cv::Mat* landingPointMap);
	//表示するウインドウの制御
	void windowController();
	//視差の補正
	void correctDisparity(double* X, double* Y, double* Z, double theta);
	//オペレーター
	void operator()();
	////マウスイベント
	//static void _onMouse(int event, int x, int y, int flags, void* data);
	////トラックバーイベント
	//void _hueTrackBar(int val, void* tileMaxSat);
	//void _satTrackBar(int val, void* tileMaxhue);
	//デストラクタ
	//~detectLandingPoint();

	
	cv::Mat _camMat1, _camMat2, _distCoeffs1, _distCoeffs2;	//ステレオカメラの内部パラメータ
	cv::Mat _R, _T, _R1, _R2, _P1, _P2, _Q;					//ステレオカメラの外部パラメータ
	cv::Size _imgSize1, _imgSize2;							//カメラの画像サイズ
	cv::Mat _rmap[2][2];									//平行化変換後の歪み補正マップ
	double _baselineRatio;									//基線長の比
	double _apWidth, _apHeight;								//_apWidth – センサの物理的な幅． _apHeight – センサの物理的な高さ．
	double _fovx, _fovy;									//_fovx – 出力される，水平軸に沿った画角（度単位）． _fovy – 出力される，垂直軸に沿った画角（度単位）．
	double _focalLength;									//mm単位で表されたレンズの焦点距離．
	cv::Point2d _priPoint;									//ピクセル単位で表された主点．
	double _aspectRatio;									//アスペクト比
	cv::Point3d _cameraPosi;								//左カメラのロボット座標系における3次元位置
	cv::Mat _frameLeft, _frameRight;						//それぞれ，ステレオカメラの左目画像と右目画像
	cv::Mat _tempImage;										//頻出色値の計算に使うテンプレート画像
	bool _selectObject;										//テンプレート画像を選択中かどうかのフラグ
	int _trackObject;										//新たに選択されたテンプレート画像があるかどうかのフラグ

	myvector::VECTOR robotPostureChange;
	myvector::VECTOR centerOfGravityMovement;				//重心移動量
	myvector::VECTOR centerOfGravityPosition;
	myvector::VECTOR robotPosture;
	//myvector::VECTOR* p_allLandingPoint;
	myvector::VECTOR* p_nearbyLandingPoint;					//ロボット近傍の脚接地可能点の配列のポインタ
	//std::vector<myvector::VECTOR> allLandingPointVECTOR;
	std::vector<myvector::VECTOR> nearbyLandingPointVECTOR;	//ロボット近傍の脚接地可能点の配列
	//int allLandingPointNum;
	int nearbyLandingPointNum;								//ロボット近傍の脚接地可能点の配列の要素数
	enum Control_flag										//脚接地可能点検出の制御フラグ
	{
		WAIT,					//脚接地候補点検出の待機
		ONCE,					//脚接地候補点検出を一度だけ実行する
		LOOP_START,				//脚接地候補点検出を一定周期のループで実行する
		LOOP_CONTENUE,			//ループの継続
		LOOP_STOP,				//ループの停止
		REMAKE_COLORDISTANCE,	//色距離画像の再計算
		FINISH_DETECTION,		//operatorを終了させる
		ONLY_RECAL,				//脚接地候補点検出なしのマップ更新
		START,					//<--------------------一応作ったけど未設定

		Flag_num	//フラグの総数
	};

	Control_flag controlDetection;			//外部からアクセスするためのフラグ

private:
	//bool _showLeftStream, _showRightStream;
	bool _showTemplate, _showHistogram;
	//bool _showColorDistance;
	bool _showMesh;
	bool _showDisparity, _showDepth;
	bool _showMap;
};

//int protoType1();
//int protoType2();
//int protoType3();
void onMouse(int event, int x, int y, int, void*);
void hueTrackBar(int val, void* tileMaxSat);
void satTrackBar(int val, void* tileMaxSat);

//cv::Mat tempImage;
//bool selectObject = false;
//int trackObject = 0;
//bool showHist = true;
//cv::Point origin;
//cv::Rect selection;

#endif	//__DetectLandingPoint_H__