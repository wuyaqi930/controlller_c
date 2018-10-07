#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <math.h>

using  Eigen::MatrixXd;
using namespace std;
using namespace Eigen;

int main()
{

	//———————————定义变量——————————————

	//定义运动学模型相关变量
	Vector3d P_t(10, 0, 10); //定义状态矩阵（X，Y，theta），并初始化
	Vector3d Pd_t(0, 0, 0);

	Vector3d P_derivative_t(0, 0, 0); //定义状态矩阵的导数（X，Y，theta），并初始化
	Vector3d Pd_derivative_t(0, 0, 0);
	
	Vector2d U_t(0, 0); //定义输入U（线速度，角速度），并初始化
	Vector2d Ud_t(0, 0);

	MatrixXd A(3, 2);//定义运动学方程的转换矩阵，并初始化
	A << 0, 0,
		0, 0,
		0, 0;
	
	MatrixXd Ad(3, 2);
	Ad << 0, 0,
		0, 0,
		0, 0;

	Vector3d P_delta(0, 0, 0);//定义理想状态矩阵和实际状态矩阵的差值，并初始化
	
	Vector3d E_t(0, 0, 0);//定义Kanayama类型误差变量，并初始化

	MatrixXd B(3, 3);//定义Kanayama类型误差变量方程的转换矩阵，并初始化
	B << 0, 0, 0,
		0, 0, 0,
		0, 0, 0;

	double Kx = 1; //controller的参数（王凯公式（15））
	double K_theta = 1;

	
	// 设置循环100s 

	for (int a = 1; a <= 100; a = a + 1)
	{
		// 给理想输入 Ud 赋值（走圆形的话，线速度V和角速度w都是常数

		Ud_t << 1, 1; // 线速度V = 1，角速度w = 0

		// 给实际输入 U 赋值（根据相关的公式进行输入）

		// 计算P_delta，求理想轨迹和实际轨迹的误差

		P_delta = Pd_t - P_t; //初始状态是（0,0,0）

		// 计算Kanayama类型误差变量方程的转换矩阵

		double B_1 = 0; //转换矩阵的1、2、4、5元素
		double B_2 = 0;
		double B_4 = 0;
		double B_5 = 0;

		B_1 = cos(P_t(2)); //对角度theta求sin、cos
		B_2 = sin(P_t(2));
		B_4 = -sin(P_t(2));
		B_5 = cos(P_t(2));

		B << B_1, B_2, 0, //给转换矩阵进行赋值
			B_4, B_5, 0,
			0, 0, 1;

		// 计算Kanayama类型误差变量

		E_t = B * P_delta; //王凯公式（13）

		// 计算输入
		// V（t）= U_t(0),W(t) = U_t(1)	Vd（t）= Ud_t(0),Wd(t) = Ud_t(1)
		// e_theta = E_t(2) e_x = E_t(0)

		if (E_t(2) == 0)
		{
			U_t(0) = Ud_t(0)*cos(E_t(2)) + Kx * E_t(0);  //王凯公式（15）需要进行一个判断，否则会出现没有解的情况
			U_t(1) = Ud_t(1) + K_theta * E_t(2) + Ud_t(0)*E_t(1);
		}
		else
		{
			U_t(0) = Ud_t(0)*cos(E_t(2)) + Kx * E_t(0);  //王凯公式（15）需要进行一个判断，否则会出现没有解的情况
			U_t(1) = Ud_t(1) + K_theta * E_t(2) + Ud_t(0)*E_t(1)*sin(E_t(2)) / E_t(2);
		}			  
					  
		// 给理想运动学方程转换矩阵Ad进行赋值 

		double Ad_1 = 0; //转换矩阵的1、4元素
		double Ad_3 = 0;

		Ad_1 = cos(Pd_t(2)); //对角度theta求sin、cos
		Ad_3 = sin(Pd_t(2));

		Ad << Ad_1, 0, //给转换矩阵进行赋值
			Ad_3, 0,
			0, 1;

		// 给实际运动学方程转换矩阵A进行赋值 

		double A_1 = 0; // 实际转换矩阵的1、4元素
		double A_3 = 0;

		A_1 = cos(P_t(2)); //对实际角度theta求sin、cos
		A_3 = sin(P_t(2));

		A << A_1, 0, //给实际转换矩阵进行赋值
			A_3, 0,
			0, 1;

		// 输出（ Xd，Yd，theta_d ）的导数

		Pd_derivative_t = Ad * Ud_t; //矩阵计算求得状态矩阵的导数

		// 输出（ X，Y，theta ）的导数

		P_derivative_t = A * U_t; //矩阵计算求得状态矩阵的导数

		// 输出（ Xd，Yd，theta_d ）

		Pd_t += Pd_derivative_t; //通过累加实现对导数的积分

		cout << "第" << a << "秒，期望状态Pd_t，实际状态P_t, 追踪误差P_delta ：\n" << endl;

		cout << "Pd_t \n" << Pd_t << "\n" << endl;

		// 输出（ X，Y，theta ）

		P_t += P_derivative_t; //通过累加实现对导数的积分

		cout << "P_t \n" << P_t << "\n" << endl;

		//———————————将理想轨迹和实际轨迹做差值模拟出来————————————

		cout << "P_delta \n" << Pd_t - P_t << "\n\n\n\n" << endl;

	}

	system("pause"); //使得系统暂停

	return 0;

}