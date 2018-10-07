#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <math.h>

using  Eigen::MatrixXd;
using namespace std;
using namespace Eigen;

int main()
{

	//���������������������������������������������������������

	//�����˶�ѧģ����ر���
	Vector3d P_t(10, 0, 10); //����״̬����X��Y��theta��������ʼ��
	Vector3d Pd_t(0, 0, 0);

	Vector3d P_derivative_t(0, 0, 0); //����״̬����ĵ�����X��Y��theta��������ʼ��
	Vector3d Pd_derivative_t(0, 0, 0);
	
	Vector2d U_t(0, 0); //��������U�����ٶȣ����ٶȣ�������ʼ��
	Vector2d Ud_t(0, 0);

	MatrixXd A(3, 2);//�����˶�ѧ���̵�ת�����󣬲���ʼ��
	A << 0, 0,
		0, 0,
		0, 0;
	
	MatrixXd Ad(3, 2);
	Ad << 0, 0,
		0, 0,
		0, 0;

	Vector3d P_delta(0, 0, 0);//��������״̬�����ʵ��״̬����Ĳ�ֵ������ʼ��
	
	Vector3d E_t(0, 0, 0);//����Kanayama����������������ʼ��

	MatrixXd B(3, 3);//����Kanayama�������������̵�ת�����󣬲���ʼ��
	B << 0, 0, 0,
		0, 0, 0,
		0, 0, 0;

	double Kx = 1; //controller�Ĳ�����������ʽ��15����
	double K_theta = 1;

	
	// ����ѭ��100s 

	for (int a = 1; a <= 100; a = a + 1)
	{
		// ���������� Ud ��ֵ����Բ�εĻ������ٶ�V�ͽ��ٶ�w���ǳ���

		Ud_t << 1, 1; // ���ٶ�V = 1�����ٶ�w = 0

		// ��ʵ������ U ��ֵ��������صĹ�ʽ�������룩

		// ����P_delta��������켣��ʵ�ʹ켣�����

		P_delta = Pd_t - P_t; //��ʼ״̬�ǣ�0,0,0��

		// ����Kanayama�������������̵�ת������

		double B_1 = 0; //ת�������1��2��4��5Ԫ��
		double B_2 = 0;
		double B_4 = 0;
		double B_5 = 0;

		B_1 = cos(P_t(2)); //�ԽǶ�theta��sin��cos
		B_2 = sin(P_t(2));
		B_4 = -sin(P_t(2));
		B_5 = cos(P_t(2));

		B << B_1, B_2, 0, //��ת��������и�ֵ
			B_4, B_5, 0,
			0, 0, 1;

		// ����Kanayama����������

		E_t = B * P_delta; //������ʽ��13��

		// ��������
		// V��t��= U_t(0),W(t) = U_t(1)	Vd��t��= Ud_t(0),Wd(t) = Ud_t(1)
		// e_theta = E_t(2) e_x = E_t(0)

		if (E_t(2) == 0)
		{
			U_t(0) = Ud_t(0)*cos(E_t(2)) + Kx * E_t(0);  //������ʽ��15����Ҫ����һ���жϣ���������û�н�����
			U_t(1) = Ud_t(1) + K_theta * E_t(2) + Ud_t(0)*E_t(1);
		}
		else
		{
			U_t(0) = Ud_t(0)*cos(E_t(2)) + Kx * E_t(0);  //������ʽ��15����Ҫ����һ���жϣ���������û�н�����
			U_t(1) = Ud_t(1) + K_theta * E_t(2) + Ud_t(0)*E_t(1)*sin(E_t(2)) / E_t(2);
		}			  
					  
		// �������˶�ѧ����ת������Ad���и�ֵ 

		double Ad_1 = 0; //ת�������1��4Ԫ��
		double Ad_3 = 0;

		Ad_1 = cos(Pd_t(2)); //�ԽǶ�theta��sin��cos
		Ad_3 = sin(Pd_t(2));

		Ad << Ad_1, 0, //��ת��������и�ֵ
			Ad_3, 0,
			0, 1;

		// ��ʵ���˶�ѧ����ת������A���и�ֵ 

		double A_1 = 0; // ʵ��ת�������1��4Ԫ��
		double A_3 = 0;

		A_1 = cos(P_t(2)); //��ʵ�ʽǶ�theta��sin��cos
		A_3 = sin(P_t(2));

		A << A_1, 0, //��ʵ��ת��������и�ֵ
			A_3, 0,
			0, 1;

		// ����� Xd��Yd��theta_d ���ĵ���

		Pd_derivative_t = Ad * Ud_t; //����������״̬����ĵ���

		// ����� X��Y��theta ���ĵ���

		P_derivative_t = A * U_t; //����������״̬����ĵ���

		// ����� Xd��Yd��theta_d ��

		Pd_t += Pd_derivative_t; //ͨ���ۼ�ʵ�ֶԵ����Ļ���

		cout << "��" << a << "�룬����״̬Pd_t��ʵ��״̬P_t, ׷�����P_delta ��\n" << endl;

		cout << "Pd_t \n" << Pd_t << "\n" << endl;

		// ����� X��Y��theta ��

		P_t += P_derivative_t; //ͨ���ۼ�ʵ�ֶԵ����Ļ���

		cout << "P_t \n" << P_t << "\n" << endl;

		//����������������������������켣��ʵ�ʹ켣����ֵģ�����������������������������

		cout << "P_delta \n" << Pd_t - P_t << "\n\n\n\n" << endl;

	}

	system("pause"); //ʹ��ϵͳ��ͣ

	return 0;

}