/*
	基于论文 Vehicle routing problem with drones considering time windows 重新编写
	编码方式为两行向量，上面一行为客户点，下面一行为服务方式
	作者： 张世岭
	时间：2022/2/23/11:19
*/

/*
	1、计算好路径的距离，行驶速度和时间等矩阵
	2、使用最近搜索方法生成初始解，然后使用经典vrp迭代，同时生成无人机路线
	3、利用VNS迭代搜索
*/
#include "标头.h"
#define V 19//客户数量+2
#define FILE_PATH "C://Users/Administrator/Desktop/随机分布数据集/测试数据集/17/R203.txt"
//#define FILE_PATH "C://Users/Ridge/Desktop/所罗门测试数据集/C101 to C109/large_size/50/c108.txt"
//const int V = 14;//数据集中点的数量
const double t_v = 0.8;//卡车速度
//const double t_v = 0.5;//卡车速度
const double u_v = 1.5;//飞机速度
const double t_Q = 20;//卡车容量
// 大规模情况下，调整卡车容量
//const double t_Q = 80;//卡车容量
const double u_Q = 5;//飞机容量
const double u_w = 1;//飞机自重
const double t_c = 10;//卡车成本
const double u_c = 0.1;//飞机成本
const double consumeRate = 0.2;//飞机电量消耗率
const double u_b = 20;//飞机电池大小
//设置权重
double w1 = 1, w2 = 1, w3 = 1, w4 = 1, w5 = 1, w6 = 1, w7 = 1;
double totalWight = 0;
const int iter_max_times = 15 * V;
const double serve_time = 0;
int sLenth = 10;//解的长度(根据初始vrp解来确定)

double disMtx[V][V];//数组存放所有点之间的距离
VERTICE vertex[V];//数组存放所有点的属性

//存放速度矩阵
double conjestCoe[V][V];

//bool ascendInMap(const pair<int, double>& v1, const pair<int, double>& v2)  {
//	return v1.second < v2.second;
//}
void shaking(SOLUTION& s)
{
	int i = rand() % 3;
	switch (i)
	{
	case 0:
		s = ran_swa_nod(s);
		break;
	case 1:
		s = rel_as_dro(s);
		break;
	case 2:
		s = add_dro_sor(s);
		break;
	default:
		break;
	}
	/*if (i < 5) {
		s = rel_as_dro(s);
	}
	else {
		s = add_dro_sor(s);
	}*/
}
void readData() {
	fstream input;
	input.open(FILE_PATH);
	for (int i = 0; i < V; i++) {
		input >> vertex[i].index;
		input >> vertex[i].x;
		input >> vertex[i].y;
		input >> vertex[i].demand;
	}
	//计算所有点之间的距离
	for (int i = 0; i < V; i++)
		for (int j = 0; j < V; j++) {
			disMtx[i][j] = pow((vertex[i].x - vertex[j].x) * (vertex[i].x - vertex[j].x) +
				(vertex[i].y - vertex[j].y) * (vertex[i].y - vertex[j].y), 0.5);
		}
	input.close();
	//fstream readCoef("C://Users/Administrator/Desktop/所罗门测试数据集/C101 to C109/small_size/10/拥堵系数.txt");

	fstream readCoef("C://Users/Administrator/Desktop/随机分布数据集/测试数据集/17/拥堵系数.txt");
	//fstream readCoef("C://Users/Ridge/Desktop/所罗门测试数据集/C101 to C109/large_size/50拥堵系数.txt");
	for (int i = 0; i < V; i++)
		for (int j = 0; j < V; j++) {
			readCoef >> conjestCoe[i][j];
		}
	readCoef.close();
	cout << "拥堵系数" << endl;
	for (int i = 0; i < V; i++)
	{
		for (int j = 0; j < V; j++) {
			cout << conjestCoe[i][j] << "\t";
		}
		cout << endl;
	}

}
double newf(SOLUTION& s1) {
	vector<MINISERT> uav;
	SOLUTION s = s1;
	//路段超载惩罚
	double part = 0, whole = 0; double cost = 0;
	for (auto it = s.solution.begin(); it != s.solution.end(); it++) {
		if (it->upper != 0) { part += vertex[it->upper].demand; }
		else {
			if (part > t_Q) {
				whole += part - t_Q;

			}
			part = 0;
		}
	}
	cost += whole * 100000;
	for (int i = s.solution.size() - 2; i > 0; i--) {
		//进行分流， 将 0 1 的一对和后面的提取出为一个无人机路线，
		if (s.solution[i - 1].lower == 0 && s.solution[i].lower == 1) {
			uav.push_back({ s.solution[i - 1].upper, s.solution[i].upper,s.solution[i + 1].upper });
			s.solution.erase(s.solution.begin() + i);
		}

	}

	for (int i = 0; i < s.solution.size() - 1; i++) {
		cost += disMtx[s.solution[i].upper][s.solution[i + 1].upper] * t_c;
	}
	for (auto it = uav.begin(); it != uav.end(); it++) {
		cost += (disMtx[it->lau][it->dro] + disMtx[it->dro][it->rdz]) * u_c;
		//设置惩罚成本
		//1、duration penalty
		double uav_time = (disMtx[it->lau][it->dro] + disMtx[it->dro][it->rdz]) / u_v + serve_time,
			van_time = disMtx[it->lau][it->rdz] / (t_v * conjestCoe[it->lau][it->rdz]);
		double first_uav_time = disMtx[it->lau][it->dro] / u_v + serve_time,
			second_uav_time = disMtx[it->dro][it->rdz] / u_v;
		//如果无人机飞行时间超过卡车最大等待时间
		if (uav_time > van_time + wait) {
			cost += (uav_time - van_time - wait) * 10000;
		}
		//考虑重量变化的电量约束
		/*if (van_time * u_r * u_w > u_b) {
			cost += -(u_b - (van_time * u_r * u_w)) * 100;
		}*/
		//考虑重量变化
		//如果无人机飞行时间超过卡车行驶时间
		if (first_uav_time + second_uav_time > van_time) {
			if (first_uav_time * u_r * (u_w + vertex[it->dro].demand) +
				(second_uav_time) * u_r * u_w > u_b) {
				cost += ((first_uav_time * u_r * (u_w + vertex[it->dro].demand) +
					(second_uav_time) * u_r * u_w) - u_b) * 10000;
			}
		}
		//如果卡车行驶时间超过无人机飞行时间
		else {
			if (first_uav_time * u_r * (u_w + vertex[it->dro].demand) +
				(van_time - first_uav_time) * u_r * u_w > u_b) {
				cost += ((first_uav_time * u_r * (u_w + vertex[it->dro].demand) +
					(van_time - first_uav_time) * u_r * u_w) - u_b) * 10000;
			}
		}

		if (vertex[it->dro].demand > u_Q) {
			cost += -(u_Q - vertex[it->dro].demand) * 10000;
		}
	}

	return cost;
}
double f(SOLUTION& sol) {
	double cost = 0;
	double over = 0;//单挑路线累积量
	double acumOver = 0;//单路线超载量
	double allOverLoad = 0;//总超载量
	//1、卡车路线成本
	vector<int> t;
	for (int i = 0; i < sol.solution.size(); i++) {
		if (sol.solution[i].lower == 0) {
			t.push_back(sol.solution[i].upper);
		}

		over += vertex[sol.solution[i].upper].demand;
		if (over > t_Q) { acumOver = (over - t_Q); }
		if (sol.solution[i].upper == 0) {
			allOverLoad += acumOver;
			acumOver = 0;
			over = 0;
		}
	}
	for (int i = 0; i < t.size() - 1; i++) {
		cost += disMtx[t[i]][t[i + 1]] * t_c;
	}

	//2、无人机路径成本
	vector<UAVSORTIE> u;
	for (int i = 1; i < sLenth - 1; i++) {
		if (sol.solution[i].lower == 1) {
			u.push_back({ sol.solution[i - 1].upper, sol.solution[i].upper, sol.solution[i + 1].upper });
		}
	}
	for (int i = 0; i < u.size(); i++) {
		cost += (disMtx[u[i].lau][u[i].dro] + disMtx[u[i].dro][u[i].red]) * u_c;
	}

	//3、容量约束惩罚
	cost += allOverLoad * 1000;

	//4、无人机路线违反约束的惩罚
	//4.1 超重惩罚

	//4.2 超出里程惩罚

	//4.3 时间同步惩罚


	return cost;
}
double f1(INITIAL& s) {//计算初始解的适应度值
	double cost = 0;
	double overLoad = 0;
	for (int i = 0; i < s.truck_route.size(); i++) {

		for (int j = 0; j < s.truck_route[i].size() - 1; j++) {
			cost += t_c * disMtx[s.truck_route[i][j]][s.truck_route[i][j + 1]];

		}

	}
	//计算超载的惩罚成本
	for (int i = 0; i < s.truck_route.size(); i++) {
		double overLoadInOne = 0;
		for (auto it = s.truck_route[i].begin(); it != s.truck_route[i].end(); it++) {
			overLoadInOne += vertex[*it].demand;
		}
		if (overLoadInOne > t_Q) { overLoad += (overLoadInOne - t_Q); }
	}



	return cost + overLoad * 1000;
}
bool cmp(const pair<int, double>& p1, const pair<int, double>& p2) {//用于更改resort的排序规则
	return p1.second < p2.second;//升序排列
}

//邻域的比较规则
bool cmp2(const WEIGHT& p1, const WEIGHT& p2) {
	return p1.proportion > p2.proportion;//根据权重大小降序排列，权重大的排在前面
}

bool cmp1(const MINISERT& p1, const MINISERT& p2) {//用于按照节约的成本降序排列不同插入方式
	return p1.addCost > p2.addCost;
}
INITIAL nearNeiborConstru() {
	vector<int> unarranged;
	for (int i = 1; i < V - 1; i++) {
		unarranged.push_back(i);
	}
	vector<vector<int> > vrp;

	vector<int> current;
	current.push_back(0);
	int selected = 0;//当前选择的点
	double load = 0;//存放累积容量

	while (unarranged.size() != 0) {


		//map<int, double> compair;//仿函数使map发生重载，按照value值升序排列
		double nearDis = 0;
		vector<pair<int, double>> arr;

		for (int i = 0; i < unarranged.size(); i++) {
			nearDis = disMtx[selected][unarranged[i]];
			//compair.insert(make_pair(i, nearDis));
			arr.push_back(make_pair(unarranged[i], nearDis));
		}
		sort(arr.begin(), arr.end(), cmp);
		//cout << "输出距离当前点" << selected << "所有的距离" << endl;
		/*for (auto it = arr.begin(); it != arr.end(); it++) {
			cout << it->first << "\t" << it->second << endl;
		}*/
		auto iter = arr.begin();
		//计算添加该点是否会超载


		/*cout << "下一点"<< iter->first <<"的需求为 " << vertex[iter->first].demand<<endl;
		cout << "当前点" << selected << "的需求为 " << vertex[selected].demand << endl;
		cout << "下一点" << iter->first << "的需求为 " << vertex[iter->first].demand << endl;*/
		load += vertex[iter->first].demand;
		if (load > t_Q) {
			current.push_back(0);
			if (current[0] != 0) {
				current.insert(current.begin(), 0);
			}
			vrp.push_back(current);
			current.clear();
			load = 0;
			selected = 0;
		}
		else {
			/*cout << "当前点" << selected << "到下一点" << iter->first << "的需求变化为 " << endl
				<< vertex[selected].demand << "--->" << vertex[iter->first].demand << endl;*/

			current.push_back(iter->first);//存放距离最近的点在当前路线
			selected = iter->first;
			for (int i = unarranged.size() - 1; i >= 0; i--) {
				if (unarranged[i] == iter->first) {
					unarranged.erase(unarranged.begin() + i);
					break;
				}
			}
		}
		//current.push_back(0);
	}
	current.push_back(0);

	auto start = current.begin();
	if (*start != 0) { current.insert(current.begin(), 0); }
	vrp.push_back(current);
	INITIAL s;
	for (int i = 0; i < vrp.size(); i++) {
		s.truck_route.push_back(vrp[i]);
	}
	return s;
}

bool inTabu(vector<pair<int, int>>& tabuList, int& first, int& second) {

	for (auto it = tabuList.begin(); it != tabuList.end(); it++) {
		if (it->first == first && it->second == second || it->first == second && it->second == first) {
			return true;
		}
	}
	return false;
}
void eliminate_empty_route(INITIAL& s) {
	for (int i = s.truck_route.size() - 1; i >= 0; i--) {
		if (s.truck_route[i].size() <= 2) {
			s.truck_route.erase(s.truck_route.begin() + i);
		}
	}
}
INITIAL two_opt(INITIAL& s) {
	INITIAL c = s;//当前解
	INITIAL g = s;//最优解
	int maxIter = iter_max_times;
	int count = 0;
	double global_opt = 0;
	double local_opt = 0;

	//为了避免重复搜索，需要使用禁忌表
	//vector<pair<int, int>> tabuList;
	while (count < maxIter) {

		int route1 = rand() % c.truck_route.size();
		int route2 = rand() % c.truck_route.size();
		if (route1 == route2) {//如果选择的是单条路线
			int first = rand() % (c.truck_route[route1].size() - 2) + 1;
			int second = rand() % (c.truck_route[route1].size() - 2) + 1;
			/*while (first == second && inTabu(tabuList, first, second)) {
				first = rand() % (c.truck_route[route1].size() - 2) + 1;
				second = rand() % (c.truck_route[route1].size() - 2) + 1;
			}*/

			if (first > second) {
				int temp = first;
				first = second;
				second = temp;
			}
			reverse(c.truck_route[route1].begin() + first, c.truck_route[route1].begin() + second);
			if (f1(c) < f1(g)) {
				g = c;
				c = g;
				global_opt = f1(c);
				count = 0;
			}
			else {
				/*if (tabuList.size() > 10) {
					tabuList.erase(tabuList.begin());
				}
				tabuList.push_back(make_pair(first, second));*/
				count++;
			}
		}
		else {//选择的是两条路线，则选取某一段互换
			int route1First = rand() % (c.truck_route[route1].size() - 2) + 1;
			int route1Second = rand() % (c.truck_route[route1].size() - 2) + 1;

			int route2First = rand() % (c.truck_route[route2].size() - 2) + 1;
			int route2Second = rand() % (c.truck_route[route2].size() - 2) + 1;

			int Class;
			if (route1First == route1Second && route2First == route2Second) {
				Class = 0;//两条路线都是选择的一个点
			}
			if (route1First == route1Second && route2First != route2Second) {
				if (route2First > route2Second) {
					int temp = route2First;
					route2First = route2Second;
					route2Second = temp;
				}
				Class = 1;//route1 选了一个点， route2 选了两个点
			}
			if (route1First != route1Second && route2First == route2Second) {
				if (route1First > route1Second) {
					int temp = route1First;
					route1First = route1Second;
					route1Second = temp;
				}
				Class = 2;//route1 选了两个点， route2 选了一个点
			}
			if (route1First != route1Second && route2First != route2Second) {
				if (route2First > route2Second) {
					int temp = route2First;
					route2First = route2Second;
					route2Second = temp;
				}
				if (route1First > route1Second) {
					int temp = route1First;
					route1First = route1Second;
					route1Second = temp;
				}
				Class = 3;//两条路线都是选择的两个点
			}
			switch (Class) {
			case 0:
				//单纯的交换这两个点
			{int temp = c.truck_route[route1][route1First];
			c.truck_route[route1][route1First] = c.truck_route[route2][route2First];
			c.truck_route[route2][route2First] = temp; }
			break;
			case 1:
				//点和序列交换
			{int selected = c.truck_route[route1][route1First];
			c.truck_route[route1].erase(c.truck_route[route1].begin() + route1First);//清除点

			for (auto it = c.truck_route[route2].begin() + route2First; it != c.truck_route[route2].begin() + route2Second; it++) {
				c.truck_route[route1].insert(c.truck_route[route1].begin() + route1First, *it);//反转插入
			}
			for (int i = route2First; i < route2Second; i++) {
				c.truck_route[route2].erase(c.truck_route[route2].begin() + route2First);//全部清除
			}
			c.truck_route[route2].insert(c.truck_route[route2].begin() + route2First, selected);//插入点
			}
			break;
			case 2:
				//点和序列交换
			{int selected = c.truck_route[route2][route2First];
			c.truck_route[route2].erase(c.truck_route[route2].begin() + route2First);//清除点

			for (auto it = c.truck_route[route1].begin() + route1First; it != c.truck_route[route1].begin() + route1Second; it++) {
				c.truck_route[route2].insert(c.truck_route[route2].begin() + route2First, *it);//反转插入
			}
			for (int i = route1First; i < route1Second; i++) {
				c.truck_route[route1].erase(c.truck_route[route1].begin() + route1First);//全部清除
			}
			c.truck_route[route1].insert(c.truck_route[route1].begin() + route1First, selected);//插入点
			}
			break;
			case 3:
				//序列和序列交换
			{
				vector<int> tempRoute1, tempRoute2;
				for (int i = route1First; i <= route1Second; i++) {
					tempRoute1.push_back(c.truck_route[route1][i]);
				}
				for (int i = route2First; i <= route2Second; i++) {
					tempRoute2.push_back(c.truck_route[route2][i]);
				}

				for (int i = route1First; i <= route1Second; i++) {
					c.truck_route[route1].erase(c.truck_route[route1].begin() + route1First);
				}
				for (int i = route2First; i <= route2Second; i++) {
					c.truck_route[route2].erase(c.truck_route[route2].begin() + route2First);
				}
				//逆向插入
				for (auto it = tempRoute1.begin(); it != tempRoute1.end(); it++) {
					c.truck_route[route1].insert(c.truck_route[route1].begin() + route1First, *it);
				}
				for (auto it = tempRoute2.begin(); it != tempRoute2.end(); it++) {
					c.truck_route[route2].insert(c.truck_route[route2].begin() + route2First, *it);
				}
			}
			break;
			default:break;

			}
			if (f1(c) < f1(g)) {
				g = c;
				c = g;
				global_opt = f1(c);
				count = 0;
			}
			else { count++; }
		}
	}


	return g;
}
INITIAL relocate(INITIAL& s) {
	INITIAL c = s;//当前解
	INITIAL g = s;//最优解
	int maxIter = iter_max_times;
	int count = 0;
	double global_opt = 0;
	double local_opt = 0;

	while (count < maxIter) {
		int route1 = rand() % c.truck_route.size();
		int route2 = rand() % c.truck_route.size();
		while (c.truck_route[route1].size() < 4 || c.truck_route[route2].size() < 4) {
			route1 = rand() % c.truck_route.size();
			route2 = rand() % c.truck_route.size();
		}
		int index1 = rand() % (c.truck_route[route1].size() - 2) + 1;
		int index2 = rand() % (c.truck_route[route2].size() - 2) + 1;
		int selected = c.truck_route[route1][index1];
		/*if (selected == 0) {
			cout << "路线" << route1 << "总大小为 " << c.truck_route[route1].size() << endl;
			cout << "提取的点来自路线 " << route1 << " 中的第 " << index1 << " 位" << endl;

			system("pause"); }*/
		c.truck_route[route1].erase(c.truck_route[route1].begin() + index1);
		c.truck_route[route2].insert(c.truck_route[route2].begin() + index2, selected);
		if (f1(c) < f1(g)) {
			g = c;
			count = 0;
		}
		else {
			count++;
			c = g;
		}

	}
	eliminate_empty_route(g);
	return g;
}

SOLUTION init_vrp_d(INITIAL& s, int& len) {
	//建立一个集合存放所有的无人机可行点
	vector<int> candidates;
	INITIAL s2 = s;
	SOLUTION s1;
	s1.solution.push_back({ 0, 0 });

	for (int i = 0; i < s.truck_route.size(); i++) {
		for (int j = 1; j < s.truck_route[i].size(); j++) {
			s1.solution.push_back({ s.truck_route[i][j], 0 });
			if (s.truck_route[i][j] != 0 && vertex[s.truck_route[i][j]].demand <= u_Q) {
				candidates.push_back(s.truck_route[i][j]);
			}
		}
	}

	vector<MINISERT> uav_route;//存放选定的无人机路线

	//开始尝试提取无人机路线
	for (int i = 0; i < candidates.size(); i++) {
		bool skip = false;
		//因为最外层 for循环 已经在遍历了，所以不需要删除，只需要遇到已经存在在无人机路线中的点直接跳过即可
		for (auto iter1 = uav_route.begin(); iter1 != uav_route.end(); iter1++) {
			if (candidates[i] == iter1->lau || candidates[i] == iter1->dro || candidates[i] == iter1->rdz) {
				skip = true;
			}
		}
		if (skip) {
			//cout << candidates[i] << "在路线中已经有了" << endl;
			;
		}
		else {
			int currentNode = candidates[i];
			double sub_cost;//从卡车路线中移除该点后减少的成本
			for (int j = 0; j < s.truck_route.size(); j++) {
				for (int k = 1; k < s.truck_route[j].size() - 1; k++) {
					if (currentNode == s.truck_route[j][k]) {
						//从卡车路线移除当前点节约出来的成本，当前是负数
						sub_cost = (disMtx[s.truck_route[j][k - 1]][s.truck_route[j][k + 1]] -
							(disMtx[s.truck_route[j][k - 1]][s.truck_route[j][k]] + disMtx[s.truck_route[j][k]][s.truck_route[j][k + 1]])) * t_c;
						s.truck_route[j].erase(s.truck_route[j].begin() + k);//从原路线中移除当前无人机待选点
					}
				}
			}
			bool* feasi = new bool[s.truck_route.size()];//用于判断当前点插入某条路径时是否超载
			for (int j = 0; j < s.truck_route.size(); j++) {
				double acLoad = 0;
				for (int k = 0; k < s.truck_route[j].size(); k++) {
					acLoad += vertex[s.truck_route[j][k]].demand;
				}
				if (uav_route.size() > 0) {
					for (auto iter_uav = uav_route.begin(); iter_uav != uav_route.end(); iter_uav++) {
						if (iter_uav->routeIndex == j) {
							acLoad += vertex[iter_uav->dro].demand;//添加上插入该路径的无人机的重量
						}
					}
				}
				if (vertex[currentNode].demand + acLoad <= t_Q) {
					feasi[j] = true;
				}
				else {
					feasi[j] = false;
				}
			}
			vector<MINISERT> add_cost_m;//存放添加无人机之后总成本的变化

			for (int j = 0; j < s.truck_route.size(); j++) {
				if (feasi[j] == 1) {//如果当前路线插入无人机不会超载，则可以选择尝试

					for (int k = 0; k < s.truck_route[j].size() - 1; k++) {
						//1、判断当前选择的弧是否已有无人机
						//if ()
						//检验当前组成的路线是否可行
						/*cout << "路线 {" << s.truck_route[j][k] << " " << currentNode << " " << s.truck_route[j][k + 1] << "}" <<
							judgeFeaibility(s.truck_route[j][k], currentNode, s.truck_route[j][k + 1], uav_route) << endl;*/
						if (judgeFeaibility(s.truck_route[j][k], currentNode, s.truck_route[j][k + 1], uav_route)) {
							double add = (disMtx[s.truck_route[j][k]][currentNode] + disMtx[currentNode][s.truck_route[j][k + 1]]) * u_c;
							//cout << "减少了 " << sub_cost << "  增加了" << add << " ";
							double saving = -(add + sub_cost);
							//cout << "savings = " << saving << endl;
							if (saving >= 0) {
								add_cost_m.push_back({ s.truck_route[j][k], currentNode,s.truck_route[j][k + 1], j, k + 1, saving });
								//依次是	[ 起飞点、无人机点、降落点、所在路径、路径中下标、节约值 ]
							}
						}
					}
				}
			}
			sort(add_cost_m.begin(), add_cost_m.end(), cmp1);//
			//cout << "输出所有与 " << currentNode << " 组成的最优无人机路线" << endl;

			//选择 节约值最大 的作为无人机路线加入
			if (add_cost_m.size() > 0) {
				auto iter = add_cost_m.begin();
				//cout << iter->lau << "\t" << iter->dro << "\t" << iter->rdz << "\t" << iter->addCost << endl;

				uav_route.push_back(*iter);
			}
			s = s2;
			delete[] feasi;//释放内存
		}

	}
	if (uav_route.size() > 0) {
		//1、清楚原路线中的无人机点
		for (int i = s1.solution.size() - 1; i >= 0; i--) {
			for (auto it = uav_route.begin(); it != uav_route.end(); it++) {
				if (s1.solution[i].upper == it->dro) {
					s1.solution.erase(s1.solution.begin() + i);
				}
			}
		}

		//2、将这些无人机点按照最优路线重新插入原路线
		for (auto it = uav_route.begin(); it != uav_route.end(); it++) {
			for (int i = 0; i <= s1.solution.size() - 2; i++) {
				if (it->lau == s1.solution[i].upper && it->rdz == s1.solution[i + 1].upper) {
					s1.solution.insert(s1.solution.begin() + i + 1, { it->dro, 1 });
				}

			}
		}
	}

	return s1;
}
bool judgeFeaibility(int l, int d, int r, vector<MINISERT>& uav_route)//用于判断当前3个点能否组成无人机路线
{
	double firstPart = disMtx[l][d] / u_v;//第一段飞行时间
	double secondPart = disMtx[d][r] / u_v;//第二段飞行时间
	double temdom = disMtx[l][r] / t_v;//卡车的行驶时间

	bool check_exist;
	for (auto it = uav_route.begin(); it != uav_route.end(); it++) {
		if (//1、l点 和 r点都已经存在在现有路线中
			it->lau == l || it->rdz == r
			) {
			return false;
		}
		if (//2、l点 或者 r点是已经组成无人机路线的无人机点
			it->dro == l || it->dro == r
			) {
			return false;
		}
	}

	if (firstPart + secondPart <= temdom && firstPart * (u_w + vertex[d].demand) * consumeRate +
		secondPart * u_w * consumeRate <= u_b) {//如果时间满足以及电量消耗满足，返回真
		return true;
	}
	else { return false; }
}
void printS(SOLUTION& s) {

	for (int i = 0; i < s.solution.size(); i++) {
		cout << s.solution[i].upper << "\t";
	}cout << endl;
	for (int i = 0; i < s.solution.size(); i++) {
		cout << s.solution[i].lower << "\t";
	}
	cout << endl;
}

INITIAL vnd(INITIAL& s) {
	int index = 0;
	bool noExit = 1;
	INITIAL current = s;
	INITIAL best = s;
	while (noExit) {
		switch (index) {
		case 0:
			current = two_opt(current);
			if (f1(current) < f1(best)) {
				best = current;
				index = 0;
			}
			else {
				current = best;
				index++;
			}
			break;
		case 1:
			current = relocate(current);
			if (f1(current) < f1(best)) {
				best = current;
				index = 0;
			}
			else {
				current = best;
				index++;
			}
			break;
		default:
			noExit = 0;
			break;
		}
	}
	return best;
}
INITIAL vns(INITIAL& s) {
	INITIAL best = s;
	INITIAL current = s;
	int count = 0;//当前迭代次数
	int maxIter = 40;//最大迭代次数
	while (count < maxIter) {
		//current = shaking(current);
		current = vnd(current);
		if (f1(current) < f1(best)) {
			best = current;
			count = 0;
		}
		else {
			current = best;
			count++;
		}
	}
	return best;
}


/*        主要的改进算法过程         */
SOLUTION VNS(SOLUTION& s) {
	_SOLUTION_WEIGHT_ best, current;
	best.solution = s.solution;
	current.solution = s.solution;
	int count = 0;//当前迭代次数
	int maxIter = V;//最大迭代次数
		//创建一个数组，用来存放对各个邻域的权重值计算
	vector<WEIGHT> weight_values;
	for (int i = 0; i < 8; i++) {
		weight_values.push_back({ i, 0 });//每个地方的权重都初始化为0
	}

	while (count < maxIter) {
		shaking(current);
		/*cout << "每个邻域的权重为： " << endl;
		for (int i = 0; i < 7; i++) {
			cout << "邻域：" << weight_values[i].index << "\t" << "权重：" << weight_values[i].proportion << endl;
		}*/
		current = VND(current, weight_values);
		if (newf(current) < newf(best)) {
			best = current;
			for (auto it2 = 0; it2 != current.sorting.size(); it2++) {
				weight_values[it2].index = current.sorting[it2].index;
				weight_values[it2].proportion = current.sorting[it2].proportion;
			}
			count = 0;
		}
		else {
			current = best;
			count++;
		}
	}

	return best;
}

_SOLUTION_WEIGHT_ VND(SOLUTION& s, vector<WEIGHT>& values) {

	bool noExit = 1;
	SOLUTION current = s;
	SOLUTION best = s;
	vector<WEIGHT> tempValues;
	tempValues = values;
	double accumulation[7] = { 0,0,0,0,0,0,0 };
	sort(tempValues.begin(), tempValues.end(), cmp2);

	/*cout << "VND中邻域的排序如下" << endl;
	for (auto it = tempValues.begin(); it != tempValues.end(); it++) {
		cout << it->index << "--" << it->proportion << endl;
	}*/
	auto it = tempValues.begin();
	while (noExit) {
		switch (it->index) {
		case 0:
			current = RandomSwapNode(current);
			if (newf(current) < newf(best)) {
				for (int i = 0; i < tempValues.size(); i++) {
					if (tempValues[i].index == it->index) {
						tempValues[i].proportion += 1;
					}
				}
				best = current;
				accumulation[it->index] += 1;
				it = tempValues.begin();

				//cout << "RandomSwapNode" << endl;

			}
			else {
				current = best;

				it++;
			}
			break;
		case 1:
			current = RandomSwapWhole(current);
			if (newf(current) < newf(best)) {
				best = current;
				accumulation[it->index] += 1;
				for (int i = 0; i < tempValues.size(); i++) {
					if (tempValues[i].index == it->index) {
						tempValues[i].proportion += 1;
					}
				}
				it = tempValues.begin();
				//cout << "RandomSwapWhole" << endl;

			}
			else {
				current = best;

				it++;
			}
			break;
		case 2:
			current = Relocate(current);
			if (newf(current) < newf(best)) {
				for (int i = 0; i < tempValues.size(); i++) {
					if (tempValues[i].index == it->index) {
						tempValues[i].proportion += 1;
					}
				}
				best = current;
				accumulation[it->index] += 1;
				it = tempValues.begin();
				//cout << "Relocate" << endl;


			}
			else {
				current = best;

				it++;
			}
			break;
		case 3:
			current = RelocateAsDrone(current);
			if (newf(current) < newf(best)) {
				for (int i = 0; i < tempValues.size(); i++) {
					if (tempValues[i].index == it->index) {
						tempValues[i].proportion += 1;
					}
				}
				accumulation[it->index] += 1;

				best = current;
				//cout << "RelocateAsDrone" << endl;


				it = tempValues.begin();
			}
			else {
				current = best;

				it++;
			}
			break;
		case 4:
			current = AddDroneSortie(current);
			if (newf(current) < newf(best)) {
				for (int i = 0; i < tempValues.size(); i++) {
					if (tempValues[i].index == it->index) {
						tempValues[i].proportion += 1;
					}
				}
				accumulation[it->index] += 1;
				it = tempValues.begin();
				best = current;
				//cout << "AddDroneSortie" << endl;


			}
			else {
				current = best;

				it++;
			}
			break;
		case 5:
			current = RandomReverseNode(current);
			if (newf(current) < newf(best)) {
				for (int i = 0; i < tempValues.size(); i++) {
					if (tempValues[i].index == it->index) {
						tempValues[i].proportion += 1;
					}
				}
				accumulation[it->index] += 1;
				it = tempValues.begin();
				best = current;
				//cout << "RandomReverseWhole" << endl;


			}
			else {
				current = best;

				it++;
			}
			break;
		case 6:
			current = RandomReverseWhole(current);
			if (newf(current) < newf(best)) {
				for (int i = 0; i < tempValues.size(); i++) {
					if (tempValues[i].index == it->index) {
						tempValues[i].proportion += 1;
					}
				}
				accumulation[it->index] += 1;
				it = tempValues.begin();
				best = current;
				//cout << "RandomReverseNode" << endl;


			}
			else {
				current = best;

				it++;
			}
			break;
		default:
			noExit = 0;
			break;
		}
	}

	for (int i = 0; i < 7; i++) {
		for (auto it1 = tempValues.begin(); it1 != tempValues.end(); it1++) {
			if (i == it->index) {
				it->proportion = accumulation[i];
			}
		}
	}
	_SOLUTION_WEIGHT_ ui;
	ui.solution = best.solution;
	ui.sorting = tempValues;
	values = tempValues;
	return ui;
}

SOLUTION relocated_as_drone(SOLUTION& s) {
	/*随机选择一个点，若是可行点，则从原路线中剔除，再*/
	double standard_cost = f(s);
	int selectedIndex = rand() % (s.solution.size() - 2) + 1;
	while (s.solution[selectedIndex].upper == 0 || vertex[s.solution[selectedIndex].upper].demand > u_Q) {
		selectedIndex = rand() % (s.solution.size() - 2) + 1;
	}
	int selectedNode = s.solution[selectedIndex].upper;
	//从原路径中剔除
	s.solution.erase(s.solution.begin() + selectedIndex);
	SOLUTION c;//用于记录当前最优的插入方式
	//选择一对弧插入 ，这里我们选择路径中连续的两个点i j，或者是间隔一个的两个点i j k，选择后者的话，间隔的那个点需要在降落点后访问i k j
	//（无论是否是无人机点，统一作为i，j
	SOLUTION s1 = s;//记录未重组之前的解
	for (int i = 1; i < s.solution.size() - 1; i++) {
		//第一种：紧挨着的两个点之间

		s.solution.insert(s.solution.begin() + i, { selectedNode, 1 });
		s.solution[i - 1].lower = 0;
		s.solution[i + 1].lower = 0;
		if (f(s) < standard_cost) {
			c = s;
			standard_cost = f(c);
		}
		s = s1;
	}
	for (int i = 1; i < s.solution.size() - 2; i++) {
		//第二种： 两个点中间隔一个点
		s.solution.insert(s.solution.begin() + i, { selectedNode, 1 });
		int temp = s.solution[i + 1].upper;
		s.solution[i + 1].upper = s.solution[i + 2].upper;
		s.solution[i + 2].upper = temp;
		s.solution[i + 1].lower = 0;
		//需要注意，不能把最后一个点变成了非 0 点
	}
	return s;
}

SOLUTION Relocate(SOLUTION& s1) {

	int count = 0;
	int maxIter = 200;
	SOLUTION s = s1;
	SOLUTION best = s1;
	while (count < maxIter)
	{
		int first = rand() % (s.solution.size() - 2) + 1;
		while (s.solution[first].upper == 0) {
			first = rand() % (s.solution.size() - 2) + 1;
		}
		SOLUTION_ELEMENT temp = s.solution[first];
		s.solution.erase(s.solution.begin() + first);
		int second = rand() % (s.solution.size() - 1) + 1;
		while (first == second) {
			second = rand() % (s.solution.size() - 1) + 1;
		}
		s.solution.insert(s.solution.begin() + second, temp);
		if (newf(s) < newf(best)) {
			count = 0;
			best = s;
		}
		else {
			count++;
			s = best;
		}
	}
	return best;
}

//重定位为无人机的算子
SOLUTION rel_as_dro(SOLUTION& s) {
	int first = rand() % (s.solution.size() - 2) + 1;
	while (s.solution[first].upper == 0) {
		first = rand() % (s.solution.size() - 2) + 1;
	}
	SOLUTION_ELEMENT temp = s.solution[first];
	temp.lower = 1;
	s.solution.erase(s.solution.begin() + first);
	int second = rand() % (s.solution.size() - 1) + 1;
	while (first == second) {
		second = rand() % (s.solution.size() - 1) + 1;
	}
	s.solution.insert(s.solution.begin() + second, temp);
	return s;
}
SOLUTION RelocateAsDrone(SOLUTION& s1) {

	int count = 0;
	int maxIter = 100;
	SOLUTION s = s1;
	SOLUTION best = s1;
	while (count < maxIter)
	{
		s = rel_as_dro(s);
		if (newf(s) < newf(best)) {
			count = 0;
			best = s;
		}
		else {
			count++;
			s = best;
		}
	}
	return best;
}

//增加无人机路线的算子
SOLUTION add_dro_sor(SOLUTION& s) {
	int first = rand() % (s.solution.size() - 2) + 1;
	while (s.solution[first].upper == 0) {
		first = rand() % (s.solution.size() - 2) + 1;
	}
	if (s.solution[first].lower == 0) {
		s.solution[first].lower = 1;
	}
	else {
		s.solution[first].lower = 0;
	}
	return s;
}
SOLUTION AddDroneSortie(SOLUTION& s1) {
	int count = 0;
	int maxIter = 50;
	SOLUTION s = s1;
	SOLUTION best = s1;
	while (count < maxIter)
	{
		s = add_dro_sor(s);
		if (newf(s) < newf(best)) {
			count = 0;
			best = s;
		}
		else {
			count++;
			s = best;
		}
	}
	return best;
}

SOLUTION ran_swa_nod(SOLUTION& s) {
	int first = rand() % (s.solution.size() - 2) + 1;
	int second = rand() % (s.solution.size() - 2) + 1;
	while (first == second || s.solution[first].upper == 0 || s.solution[second].upper == 0) {
		first = rand() % (s.solution.size() - 2) + 1;
		second = rand() % (s.solution.size() - 2) + 1;
	}
	//交换两个点
	int temp = s.solution[first].upper;
	s.solution[first].upper = s.solution[second].upper;
	s.solution[second].upper = temp;
	return s;
}
SOLUTION RandomSwapNode(SOLUTION& s1) {

	int count = 0;
	int maxIter = iter_max_times;
	SOLUTION s = s1;
	SOLUTION best = s1;
	while (count < maxIter)
	{
		int first = rand() % (s.solution.size() - 2) + 1;
		int second = rand() % (s.solution.size() - 2) + 1;
		while (first == second || s.solution[first].upper == 0 || s.solution[second].upper == 0) {
			first = rand() % (s.solution.size() - 2) + 1;
			second = rand() % (s.solution.size() - 2) + 1;
		}
		//交换两个点
		int temp = s.solution[first].upper;
		s.solution[first].upper = s.solution[second].upper;
		s.solution[second].upper = temp;
		if (newf(s) < newf(best)) {
			count = 0;
			best = s;
		}
		else {
			count++;
			s = best;
		}
	}
	return best;
}

SOLUTION RandomSwapWhole(SOLUTION& s1) {
	int count = 0;
	int maxIter = iter_max_times;
	SOLUTION s = s1;
	SOLUTION best = s1;
	while (count < maxIter)
	{
		int first = rand() % (s.solution.size() - 2) + 1;
		int second = rand() % (s.solution.size() - 2) + 1;
		while (first == second || s.solution[first].upper == 0 || s.solution[second].upper == 0) {
			first = rand() % (s.solution.size() - 2) + 1;
			second = rand() % (s.solution.size() - 2) + 1;
		}
		//交换两个点

		SOLUTION_ELEMENT temp1 = s.solution[first];
		s.solution[first] = s.solution[second];
		s.solution[second] = temp1;
		//1、如果换过去的点是无人机点，但是换过去之后两边都是无人机点，则把换过去的这个点改为卡车点
		//if (s.solution[first - 1].lower == 1 && s.solution[first].lower == 1 && s.solution[first + 1].lower == 1) {
		//	s.solution[first].lower = 0;
		//}
		//if (s.solution[second - 1].lower == 1 && s.solution[second].lower == 1 && s.solution[second + 1].lower == 1) {
		//	s.solution[second].lower = 0;
		//}
		////2、如果换过去的是无人机点，但是还过去之后只有一边是无人机点，则把换过去的点改为卡车点，另一边的点改为无人机点  
		//// 1 -> 1 (1) 0  => 1 (0) 0  : 1 (1) 0 => 0 (1) 0
		//if (s.solution[first].lower == 1) {
		//	if (s.solution[first - 1].lower == 1 && s.solution[first + 1].lower == 0) {
		//		int choose = rand() % 2;
		//		if (choose == 0) {
		//			s.solution[first].lower = 0;
		//		}
		//		else{ s.solution[first - 1].lower = 0; }
		//		//计算选择插入成本最低的方式
		//		/*double cost100 = (disMtx[s.solution[first - 2].upper][s.solution[first].upper] + disMtx[s.solution[first].upper][s.solution[first + 1].upper]) * t_c
		//			+ (disMtx[s.solution[first - 2].upper][s.solution[first - 1].upper] + disMtx[s.solution[first - 1].upper][s.solution[first].upper]) * u_c;
		//		double cost010 = (disMtx[s.solution[first - 2].upper][s.solution[first - 1].upper] + disMtx[s.solution[first - 1].upper][s.solution[first + 1].upper]) * t_c
		//			+ (disMtx[s.solution[first - 1].upper][s.solution[first].upper] + disMtx[s.solution[first].upper][s.solution[first + 1].upper]) * u_c;
		//		if (cost100 < cost010) {
		//			s.solution[first].lower = 0;
		//		}
		//		else {
		//			s.solution[first - 1].lower = 0;
		//		}*/
		//	}
		//	else if (s.solution[first - 1].lower == 0 && s.solution[first + 1].lower == 1) {
		//		int choose = rand() % 2;
		//		if (choose == 0) {
		//			s.solution[first].lower = 0;
		//		}
		//		else { s.solution[first + 1].lower = 0; }
		//		
		//		//计算选择插入成本最低的方式
		//		// 0 -> 0 (1) 1 => 0 (0) 1 : 0 (1) 1 => 0 (1) 0
		//		/*double cost001 = (disMtx[s.solution[first - 1].upper][s.solution[first].upper] + disMtx[s.solution[first].upper][s.solution[first + 2].upper]) * t_c
		//			+ (disMtx[s.solution[first].upper][s.solution[first + 1].upper] + disMtx[s.solution[first + 1].upper][s.solution[first + 2].upper]) * u_c;
		//		double cost010 = (disMtx[s.solution[first - 1].upper][s.solution[first + 1].upper] + disMtx[s.solution[first + 1].upper][s.solution[first + 2].upper]) * t_c
		//			+ (disMtx[s.solution[first - 1].upper][s.solution[first].upper] + disMtx[s.solution[first].upper][s.solution[first + 1].upper]) * u_c;
		//		if (cost001 < cost010) {
		//			s.solution[first].lower = 0;
		//		}
		//		else {
		//			s.solution[first + 1].lower = 0;
		//		}*/
		//	}
		//}
		if (newf(s) < newf(best)) {
			count = 0;
			best = s;
		}
		else {
			count++;
			s = best;
		}
	}
	return best;
}

SOLUTION RandomReverseNode(SOLUTION& s1) {
	int count = 0;
	int maxIter = iter_max_times;
	SOLUTION s = s1;
	SOLUTION best = s1;
	while (count < maxIter)
	{
		int first = rand() % (s.solution.size() - 2) + 1;
		int second = rand() % (s.solution.size() - 2) + 1;
		while (first >= second || s.solution[first].upper == 0 || s.solution[second].upper == 0) {
			first = rand() % (s.solution.size() - 2) + 1;
			second = rand() % (s.solution.size() - 2) + 1;
		}
		//交换两个点
		reverse(s.solution.begin() + first, s.solution.begin() + second);


		if (newf(s) < newf(best)) {
			count = 0;
			best = s;
		}
		else {
			count++;
			s = best;
		}
	}
	return best;
}

SOLUTION RandomReverseWhole(SOLUTION& s1) {
	int count = 0;
	int maxIter = iter_max_times;
	SOLUTION s = s1;
	SOLUTION best = s1;
	while (count < maxIter)
	{
		int first = rand() % (s.solution.size() - 2) + 1;
		int second = rand() % (s.solution.size() - 2) + 1;
		while (first >= second || s.solution[first].upper == 0 || s.solution[second].upper == 0) {
			first = rand() % (s.solution.size() - 2) + 1;
			second = rand() % (s.solution.size() - 2) + 1;
		}
		//逆序上面的点
		//reverse(s.solution.begin() + first, s.solution.begin() + second);
		vector<int> temp;
		for (int i = 0; i < first; i++) {
			temp.push_back(s.solution[i].upper);
		}
		for (int i = second; i >= first; i--) {
			temp.push_back(s.solution[i].upper);
		}
		for (int i = second + 1; i < s.solution.size(); i++) {
			temp.push_back(s.solution[i].upper);
		}
		for (int i = 0; i < s.solution.size(); i++) {
			s.solution[i].upper = temp[i];
			if (s.solution[i].upper == 0) {
				s.solution[i].lower = 0;
			}
		}
		if (newf(s) < newf(best)) {
			count = 0;
			best = s;
		}
		else {
			count++;
			s = best;
		}
	}
	return best;
}

SOLUTION RelocateTwice(SOLUTION& s1) {
	int count = 0;
	int maxIter = 200;
	SOLUTION s = s1;
	SOLUTION best = s1;
	while (count < maxIter)
	{
		int first = rand() % (s.solution.size() - 2) + 1;
		while (s.solution[first].upper == 0) {
			first = rand() % (s.solution.size() - 2) + 1;
		}
		SOLUTION_ELEMENT temp = s.solution[first];
		s.solution.erase(s.solution.begin() + first);
		int second = rand() % (s.solution.size() - 1) + 1;
		while (first == second) {
			second = rand() % (s.solution.size() - 1) + 1;
		}
		s.solution.insert(s.solution.begin() + second, temp);

		//这是relocate一次




		if (newf(s) < newf(best)) {
			count = 0;
			best = s;
		}
		else {
			count++;
			s = best;
		}
	}
	return best;
}

////消除无人机路线的算子
//SOLUTION removaDroneSortie(SOLUTION& s) {
//	int first = rand() % (s.solution.size() - 2) + 1;
//	while (s.solution[first].upper == 0) {
//		first = rand() % (s.solution.size() - 2) + 1;
//	}
//	if (s.solution[first].lower == 0) {
//		s.solution[first].lower = 1;
//	}
//	else {
//		s.solution[first].lower = 0;
//	}
//	return s;
//}




int main() {

	srand((unsigned int)time(0));
	readData();//读取数据，初始化距离矩阵
	cout << "卡车经过不同路段的时间" << endl;
	for (int i = 0; i < V; i++)
	{
		for (int j = 0; j < V; j++) {
			cout << disMtx[i][j] / (t_v * conjestCoe[i][j]) << "\t";
		}
		cout << endl;
	}



	/*SOLUTION rr;
	rr.solution.push_back({ 0,0 });
	rr.solution.push_back({ 11,1 });
	rr.solution.push_back({ 9,0 });
	rr.solution.push_back({ 1,1 });
	rr.solution.push_back({ 6,0 });
	rr.solution.push_back({ 4,1 });
	rr.solution.push_back({ 0,0 });
	rr.solution.push_back({ 3,0 });
	rr.solution.push_back({ 5,1 });
	rr.solution.push_back({ 2,0 });
	rr.solution.push_back({ 0,0 });
	rr.solution.push_back({ 10,0 });
	rr.solution.push_back({ 14,1 });
	rr.solution.push_back({ 13,0 });
	rr.solution.push_back({ 15,1 });
	rr.solution.push_back({ 0,0 });
	rr.solution.push_back({ 8,0 });
	rr.solution.push_back({ 7,0 });
	rr.solution.push_back({ 12,0 });
	rr.solution.push_back({ 0,0 });
	
	cout << endl <<newf(rr);*/

	clock_t start, end;
	start = clock();

	INITIAL s = nearNeiborConstru();//构建纯卡车vrp
	cout << "最近搜索解：" << f1(s) << endl;
	for (int i = 0; i < s.truck_route.size(); i++) {
		for (auto it = s.truck_route[i].begin(); it != s.truck_route[i].end(); it++) {
			cout << *it << "\t";
		}
		cout << endl;
	}
	//使用经典 2-Opt 去对 VRP 路径寻优

	s = vns(s);
	cout << "优化后的初始解：" << f1(s) << endl;
	for (int i = 0; i < s.truck_route.size(); i++) {
		for (auto it = s.truck_route[i].begin(); it != s.truck_route[i].end(); it++) {
			cout << *it << "\t";
		}
		cout << endl;
	}


	int sLenth = s.truck_route.size();

	SOLUTION solut = init_vrp_d(s, sLenth);
	//solut = elimDoubleZero(solut);
	cout << "提取后的初始解：" << newf(solut) << endl;
	//cout << "新的计算方法： " << newf(solut) << endl;
	printS(solut);

	//开始 local search 改进解的质量
	solut = VNS(solut);
	cout << "改进后的解：" << newf(solut) << endl;

	printS(solut);
	end = clock();
	double time = double(end - start) / CLOCKS_PER_SEC;
	cout << "总计用时 " << time << "秒" << endl;
	system("pause");
	ofstream out;
	out.open("C://Users/Administrator/Desktop/随机分布数据集/测试数据集/17/R203+_自适应VNS_result.txt");
	////out.open("C://Users/Ridge/Desktop/所罗门测试数据集/C101 to C109/large_size/50/c108_VNS_result.txt");

	out << "目标值：" << newf(solut) << endl;
	out << "路线为：" << endl;
	for (int i = 0; i < solut.solution.size(); i++) {
		out << solut.solution[i].upper << "\t";
	}out << endl;
	for (int i = 0; i < solut.solution.size(); i++) {
		out << solut.solution[i].lower << "\t";
	}
	out << endl;
	out << "总计用时 " << time << "秒" << endl;
	out.close();
	return 0;
}