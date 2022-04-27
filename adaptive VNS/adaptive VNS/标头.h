#pragma once
#include <iostream>
#include <vector>
#include <algorithm>
#include <ctime>
#include <random>
#include <fstream>
#include <map>
#include <utility>
#include <numeric>
using namespace std;

double u_r = 0.2;//�������
double wait = 1;//���������˻������ʱ��
//���������
class SOLUTION_ELEMENT {
public:
	int upper;//�ϲ����ԣ��������Ŀͻ���
	bool lower;//�²����ԣ��������ʽ��0 (����); 1 (���˻�)
};

//�������������
class VERTICE {
public:
	int index;//���
	double x;//x����
	double y;//y����
	double demand;//����
};

//�������˻�·���࣬���ڷ������
class UAVSORTIE {
public:
	int lau;//���
	int dro;//�����
	int red;//�����
};

//��ʼ��VRP����
class INITIAL : public UAVSORTIE {
public:
	vector<vector<int>> truck_route;
	vector<UAVSORTIE> drone_route;
};

//��������Ȩ����
class WEIGHT {
public:
	int index;//�������
	double proportion;//ռ�õı���
};

//��ĳ��ַ�����
class SOLUTION {
public:
	vector<SOLUTION_ELEMENT> solution;
};

//�̳��Խ�������࣬���ڷ���Ȩ��
class _SOLUTION_WEIGHT_ : public SOLUTION
{
public:
	vector<WEIGHT> sorting;//���ظ��������Ӧ�����Լ�����һ��VND����Ե�Ȩ��
};

class MINISERT {
public:
	int lau;
	int dro;
	int rdz;
	int routeIndex;//����·��
	int interIndex;//·���еĽ���������±�
	double addCost;
};




/*         ��������          */
INITIAL nearNeiborConstru();//�����ʼ��
bool judgeFeaibility(int l, int d, int r, vector<MINISERT>& uav_rout);//�����жϵ�ǰ3�����ܷ�������˻�·��

INITIAL two_opt(INITIAL& s);//�Գ�ʼ����� 2-opt Ѱ��



/*         VNS          */
SOLUTION VNS(SOLUTION& s);
_SOLUTION_WEIGHT_ VND(SOLUTION& s, vector<WEIGHT>& weight);

/*         LS          */
SOLUTION relocated_as_drone(SOLUTION& s);//���˻�·����ȡ

//��������
SOLUTION rel_as_dro(SOLUTION& s);
SOLUTION add_dro_sor(SOLUTION& s);
SOLUTION ran_swa_nod(SOLUTION& s);
SOLUTION removaDroneSortie(SOLUTION& s);//����Ƴ�һ�����˻���Ϊ������
SOLUTION RandomSwapNode(SOLUTION& s);//�������һ����
SOLUTION RandomSwapWhole(SOLUTION& s);//�������һ���㼰�䷽ʽ
SOLUTION Relocate(SOLUTION& s);
SOLUTION RelocateAsDrone(SOLUTION& s);
SOLUTION AddDroneSortie(SOLUTION& s);//����һ����Ϊ1
SOLUTION RandomReverseNode(SOLUTION& s);//�����ת
SOLUTION RandomReverseWhole(SOLUTION& s);//�����תȫ��

SOLUTION RelocateTwice(SOLUTION& s);//�����ض�λ����


SOLUTION RotateAroundNode(SOLUTION& s);//Χ��һ������ת
SOLUTION remove_const_by_sortie(SOLUTION& s);//�Ƴ�һ��·��������ȡ���˻�·�����еķ����ع�
/*
���뺯��
*/
SOLUTION _const_by_sortie(vector<int>& D, vector<int>& d, SOLUTION& p_s, int first, int second);
#pragma once
#pragma once
