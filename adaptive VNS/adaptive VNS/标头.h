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

double u_r = 0.2;//电池消耗
double wait = 1;//卡车等无人机的最大时间
//创建解的类
class SOLUTION_ELEMENT {
public:
	int upper;//上层属性，代表服务的客户点
	bool lower;//下层属性，代表服务方式：0 (卡车); 1 (无人机)
};

//创建点的属性类
class VERTICE {
public:
	int index;//序号
	double x;//x坐标
	double y;//y坐标
	double demand;//需求
};

//创建无人机路线类，用于方便计算
class UAVSORTIE {
public:
	int lau;//起点
	int dro;//服务点
	int red;//降落点
};

//初始解VRP的类
class INITIAL : public UAVSORTIE {
public:
	vector<vector<int>> truck_route;
	vector<UAVSORTIE> drone_route;
};

//创建邻域权重类
class WEIGHT {
public:
	int index;//邻域序号
	double proportion;//占用的比例
};

//解的呈现方案类
class SOLUTION {
public:
	vector<SOLUTION_ELEMENT> solution;
};

//继承自解的派生类，用于返回权重
class _SOLUTION_WEIGHT_ : public SOLUTION
{
public:
	vector<WEIGHT> sorting;//返回各个邻域对应代号以及经过一次VND后各自的权重
};

class MINISERT {
public:
	int lau;
	int dro;
	int rdz;
	int routeIndex;//哪条路径
	int interIndex;//路径中的降落点所在下标
	double addCost;
};




/*         函数声明          */
INITIAL nearNeiborConstru();//构造初始解
bool judgeFeaibility(int l, int d, int r, vector<MINISERT>& uav_rout);//用于判断当前3个点能否组成无人机路线

INITIAL two_opt(INITIAL& s);//对初始解进行 2-opt 寻优



/*         VNS          */
SOLUTION VNS(SOLUTION& s);
_SOLUTION_WEIGHT_ VND(SOLUTION& s, vector<WEIGHT>& weight);

/*         LS          */
SOLUTION relocated_as_drone(SOLUTION& s);//无人机路线提取

//算子区域
SOLUTION rel_as_dro(SOLUTION& s);
SOLUTION add_dro_sor(SOLUTION& s);
SOLUTION ran_swa_nod(SOLUTION& s);
SOLUTION removaDroneSortie(SOLUTION& s);//随机移除一个无人机点为卡车点
SOLUTION RandomSwapNode(SOLUTION& s);//随机交换一个点
SOLUTION RandomSwapWhole(SOLUTION& s);//随机交换一个点及其方式
SOLUTION Relocate(SOLUTION& s);
SOLUTION RelocateAsDrone(SOLUTION& s);
SOLUTION AddDroneSortie(SOLUTION& s);//更改一个点为1
SOLUTION RandomReverseNode(SOLUTION& s);//随机反转
SOLUTION RandomReverseWhole(SOLUTION& s);//随机反转全部

SOLUTION RelocateTwice(SOLUTION& s);//连续重定位两次


SOLUTION RotateAroundNode(SOLUTION& s);//围绕一个点旋转
SOLUTION remove_const_by_sortie(SOLUTION& s);//移除一段路径，并采取无人机路线先行的方法重构
/*
插入函数
*/
SOLUTION _const_by_sortie(vector<int>& D, vector<int>& d, SOLUTION& p_s, int first, int second);
#pragma once
#pragma once
