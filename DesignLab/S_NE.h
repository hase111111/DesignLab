/* Create by Yuya Tatsuta 2019.02.06

このプログラムは

*/

#pragma once
#include<iostream>
#include<vector>
#include<algorithm>
#include<cmath>

// G_COM と三脚以上の接地脚から正規化エネルギー安定余裕を求めるすごいやつ．

namespace S_NE
{
	typedef std::vector<double> Array;
	typedef std::vector<Array> Matrix;

	// O( n )  単位行列 (identity matrix)
	Matrix identity(int n)
	{
		Matrix A(n, Array(n));

		for (int i = 0; i < n; ++i)
		{
			A[i][i] = 1;
		}

		return A;
	}

	// O( n^2 )  行列の積 multiply
	Array mul(const Matrix& A, const Array& x)
	{
		Array y(A.size());

		for (int i = 0; i < (int)A.size(); ++i)
		{
			for (int j = 0; j < (int)A[0].size(); ++j)
			{
				y[i] = A[i][j] * x[j];
			}
		}

		return y;
	}

	// O( n^3 )
	Matrix mul(const Matrix& A, const Matrix& B) {
		Matrix C(A.size(), Array(B[0].size()));
		for (int i = 0; i < (int)C.size(); ++i)
			for (int j = 0; j < (int)C[i].size(); ++j)
				for (int k = 0; k < (int)A[i].size(); ++k)
					C[i][j] += A[i][k] * B[k][j];
		return C;
	}

	// O(n^3), Givens Rotation
#define MAKEROT(x,y,c,s) {double r = sqrt(x*x+y*y); c = x/r; s = y/r;}
#define ROT(x,y,c,s) {double u = c*x+s*y; double v = -s*x+c*y; x = u; y = v;}
	Array Givens_rotation(Matrix A, Array b) {
		int n = (int)b.size();
		for (int i = 0; i < n; ++i) {
			for (int j = i + 1; j < n; j++) {
				double c, s;
				MAKEROT(A[i][i], A[j][i], c, s);
				ROT(b[i], b[j], c, s);
				for (int k = i; k < n; k++)
					ROT(A[i][k], A[j][k], c, s);
			}
		}
		for (int i = n - 1; i >= 0; i--) {
			for (int j = i + 1; j < n; j++)
				b[i] = b[i] - A[i][j] * b[j];
			b[i] = b[i] / A[i][i];
		}
		return b;
	}


	struct POINT {
		double x, y, z;
	};
	POINT operator -(POINT a, POINT b) {
		POINT ret; ret.x = a.x - b.x; ret.y = a.y - b.y; ret.z = a.z - b.z;
		return ret;
	}
	POINT operator +(POINT a, POINT b) {
		POINT ret; ret.x = a.x + b.x; ret.y = a.y + b.y; ret.z = a.z + b.z;
		return ret;
	}
	POINT operator *(double a, POINT b) {
		POINT ret; ret.x = a * b.x; ret.y = a * b.y; ret.z = a * b.z;
		return ret;
	}
	POINT normalization(POINT a) {
		double t = sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
		POINT ret; ret.x = a.x / t; ret.y = a.y / t; ret.z = a.z / t;
		return ret;
	}
	double Mag(POINT a) { return sqrt(a.x * a.x + a.y * a.y + a.z * a.z); }
	double Dot(POINT a, POINT  b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

	class S_NE_Calc {
	public:
		std::vector<POINT> Legs; // !!!!!! GLOBAL 座標系-> Legsもglobal．
		POINT G_COM; // 胴体の重心座標

		void setLegs(POINT p) { Legs.push_back(p); }
		void resetLegs() { Legs.clear(); }
		void set_GCOM(POINT p) { G_COM = p; }

		double SNE()
		{
			double ret = 100000000000;

			if (this->Legs.size() < 3)
			{
				std::cerr << "ERROR S_NE !!!!!" << std::endl;
				return 0.0;
			}

			//まずは残渣 z = ax + by + cとして基準平面を求める．
			double a, b, c;

			{
				double A = 0.0; for (int i = 0; i < this->Legs.size(); ++i) A += this->Legs[i].x * this->Legs[i].x;
				double B = 0.0; for (int i = 0; i < this->Legs.size(); ++i) B += this->Legs[i].y * this->Legs[i].y;
				double C = 0.0; for (int i = 0; i < this->Legs.size(); ++i) C += this->Legs[i].x * this->Legs[i].y;
				double D = 0.0; for (int i = 0; i < this->Legs.size(); ++i) D += this->Legs[i].x * this->Legs[i].z;
				double E = 0.0; for (int i = 0; i < this->Legs.size(); ++i) E += this->Legs[i].y * this->Legs[i].z;
				double F = 0.0; for (int i = 0; i < this->Legs.size(); ++i) F += this->Legs[i].x;
				double G = 0.0; for (int i = 0; i < this->Legs.size(); ++i) G += this->Legs[i].y;
				double H = 0.0; for (int i = 0; i < this->Legs.size(); ++i) H += this->Legs[i].z;

				Array x(3);  x[0] = D, x[1] = E, x[2] = H;
				Array ta(3);  ta[0] = A, ta[1] = C, ta[2] = F;
				Array tb(3);  tb[0] = C, tb[1] = B, tb[2] = G;
				Array tc(3);  tc[0] = F, tc[1] = G, tc[2] = double(this->Legs.size());
				Matrix AA(3, Array(3)); AA[0] = ta; AA[1] = tb; AA[2] = tc;
				Array bb = Givens_rotation(AA, x);
				a = bb[0]; b = bb[1]; c = bb[2];
				double d = (-a * Legs[0].x - b * Legs[0].y - c * Legs[0].z);
				std::cerr << a << " " << b << " " << c << " " << d << std::endl;
			}

			// 平面の　方程式をax+by+cz + d = 0と表現する
			POINT G = this->G_COM;
			POINT Gc; Gc.x = a * G.x, Gc.y = b * G.y, Gc.z = c * G.z; // 嘘
			double theta = atan(a / b); //rad
			double xi = theta;
			POINT P;
			for (int i = 0; i < this->Legs.size(); ++i) {
				// 隣り合う2脚が回転軸になる
				int j = (i == this->Legs.size() - 1 ? 0 : i + 1);

				// P : (x - x_0) + (y - y_0) + ( z - z_0) + t(ls_vec) において二分探索してPを見つける
				POINT ls_vec = this->Legs[i] - this->Legs[j];
				double MAX_t = 100000000;
				double MIN_t = -100000000;
				double t = (MAX_t + MIN_t) / 3.0;
				{
					double d = (MAX_t - MIN_t) / 3.0;
					while (abs(MAX_t - MIN_t) < 0.0001) {
						d = (MAX_t - MIN_t) / 2.0;
						double sa = Dot(ls_vec, G_COM - (Legs[i] + (MIN_t + d) * ls_vec));
						double sb = Dot(ls_vec, G_COM - (Legs[i] + (MIN_t + 2 * d) * ls_vec));
						sa /= Mag(ls_vec) * Mag(G_COM - (Legs[i] + (MIN_t + d) * ls_vec));
						sb /= Mag(ls_vec) * Mag(G_COM - (Legs[i] + (MIN_t + 2 * d) * ls_vec));
						//std::cerr << sa << " " << sb << std::endl;
						if (abs(sa) > abs(sb))  MIN_t = t;
						else MAX_t = t + d;
					}
					P = Legs[i] + d * ls_vec;
				}
				double h = Mag(G - P);
				double d = Mag(Gc - P);
				std::cerr << h << " " << d << std::endl;
				std::cerr << "!" << sqrt(h * h + d * d) - (h * cos(theta) + d * sin(theta)) << std::endl;
				ret = (std::min)(ret, sqrt(h * h + d * d) - (h * cos(theta) + d * sin(theta)));
			}
			std::cerr << std::fixed << "Now S_NE Score = " << ret << std::endl;
			return ret;
		}
	};
};