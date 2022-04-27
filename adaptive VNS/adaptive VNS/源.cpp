/*
	�������� Vehicle routing problem with drones considering time windows ���±�д
	���뷽ʽΪ��������������һ��Ϊ�ͻ��㣬����һ��Ϊ����ʽ
	���ߣ� ������
	ʱ�䣺2022/2/23/11:19
*/

/*
	1�������·���ľ��룬��ʻ�ٶȺ�ʱ��Ⱦ���
	2��ʹ����������������ɳ�ʼ�⣬Ȼ��ʹ�þ���vrp������ͬʱ�������˻�·��
	3������VNS��������
*/
#include "��ͷ.h"
#define V 19//�ͻ�����+2
#define FILE_PATH "C://Users/Administrator/Desktop/����ֲ����ݼ�/�������ݼ�/17/R203.txt"
//#define FILE_PATH "C://Users/Ridge/Desktop/�����Ų������ݼ�/C101 to C109/large_size/50/c108.txt"
//const int V = 14;//���ݼ��е������
const double t_v = 0.8;//�����ٶ�
//const double t_v = 0.5;//�����ٶ�
const double u_v = 1.5;//�ɻ��ٶ�
const double t_Q = 20;//��������
// ���ģ����£�������������
//const double t_Q = 80;//��������
const double u_Q = 5;//�ɻ�����
const double u_w = 1;//�ɻ�����
const double t_c = 10;//�����ɱ�
const double u_c = 0.1;//�ɻ��ɱ�
const double consumeRate = 0.2;//�ɻ�����������
const double u_b = 20;//�ɻ���ش�С
//����Ȩ��
double w1 = 1, w2 = 1, w3 = 1, w4 = 1, w5 = 1, w6 = 1, w7 = 1;
double totalWight = 0;
const int iter_max_times = 15 * V;
const double serve_time = 0;
int sLenth = 10;//��ĳ���(���ݳ�ʼvrp����ȷ��)

double disMtx[V][V];//���������е�֮��ľ���
VERTICE vertex[V];//���������е������

//����ٶȾ���
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
	//�������е�֮��ľ���
	for (int i = 0; i < V; i++)
		for (int j = 0; j < V; j++) {
			disMtx[i][j] = pow((vertex[i].x - vertex[j].x) * (vertex[i].x - vertex[j].x) +
				(vertex[i].y - vertex[j].y) * (vertex[i].y - vertex[j].y), 0.5);
		}
	input.close();
	//fstream readCoef("C://Users/Administrator/Desktop/�����Ų������ݼ�/C101 to C109/small_size/10/ӵ��ϵ��.txt");

	fstream readCoef("C://Users/Administrator/Desktop/����ֲ����ݼ�/�������ݼ�/17/ӵ��ϵ��.txt");
	//fstream readCoef("C://Users/Ridge/Desktop/�����Ų������ݼ�/C101 to C109/large_size/50ӵ��ϵ��.txt");
	for (int i = 0; i < V; i++)
		for (int j = 0; j < V; j++) {
			readCoef >> conjestCoe[i][j];
		}
	readCoef.close();
	cout << "ӵ��ϵ��" << endl;
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
	//·�γ��سͷ�
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
		//���з����� �� 0 1 ��һ�Ժͺ������ȡ��Ϊһ�����˻�·�ߣ�
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
		//���óͷ��ɱ�
		//1��duration penalty
		double uav_time = (disMtx[it->lau][it->dro] + disMtx[it->dro][it->rdz]) / u_v + serve_time,
			van_time = disMtx[it->lau][it->rdz] / (t_v * conjestCoe[it->lau][it->rdz]);
		double first_uav_time = disMtx[it->lau][it->dro] / u_v + serve_time,
			second_uav_time = disMtx[it->dro][it->rdz] / u_v;
		//������˻�����ʱ�䳬���������ȴ�ʱ��
		if (uav_time > van_time + wait) {
			cost += (uav_time - van_time - wait) * 10000;
		}
		//���������仯�ĵ���Լ��
		/*if (van_time * u_r * u_w > u_b) {
			cost += -(u_b - (van_time * u_r * u_w)) * 100;
		}*/
		//���������仯
		//������˻�����ʱ�䳬��������ʻʱ��
		if (first_uav_time + second_uav_time > van_time) {
			if (first_uav_time * u_r * (u_w + vertex[it->dro].demand) +
				(second_uav_time) * u_r * u_w > u_b) {
				cost += ((first_uav_time * u_r * (u_w + vertex[it->dro].demand) +
					(second_uav_time) * u_r * u_w) - u_b) * 10000;
			}
		}
		//���������ʻʱ�䳬�����˻�����ʱ��
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
	double over = 0;//����·���ۻ���
	double acumOver = 0;//��·�߳�����
	double allOverLoad = 0;//�ܳ�����
	//1������·�߳ɱ�
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

	//2�����˻�·���ɱ�
	vector<UAVSORTIE> u;
	for (int i = 1; i < sLenth - 1; i++) {
		if (sol.solution[i].lower == 1) {
			u.push_back({ sol.solution[i - 1].upper, sol.solution[i].upper, sol.solution[i + 1].upper });
		}
	}
	for (int i = 0; i < u.size(); i++) {
		cost += (disMtx[u[i].lau][u[i].dro] + disMtx[u[i].dro][u[i].red]) * u_c;
	}

	//3������Լ���ͷ�
	cost += allOverLoad * 1000;

	//4�����˻�·��Υ��Լ���ĳͷ�
	//4.1 ���سͷ�

	//4.2 ������̳ͷ�

	//4.3 ʱ��ͬ���ͷ�


	return cost;
}
double f1(INITIAL& s) {//�����ʼ�����Ӧ��ֵ
	double cost = 0;
	double overLoad = 0;
	for (int i = 0; i < s.truck_route.size(); i++) {

		for (int j = 0; j < s.truck_route[i].size() - 1; j++) {
			cost += t_c * disMtx[s.truck_route[i][j]][s.truck_route[i][j + 1]];

		}

	}
	//���㳬�صĳͷ��ɱ�
	for (int i = 0; i < s.truck_route.size(); i++) {
		double overLoadInOne = 0;
		for (auto it = s.truck_route[i].begin(); it != s.truck_route[i].end(); it++) {
			overLoadInOne += vertex[*it].demand;
		}
		if (overLoadInOne > t_Q) { overLoad += (overLoadInOne - t_Q); }
	}



	return cost + overLoad * 1000;
}
bool cmp(const pair<int, double>& p1, const pair<int, double>& p2) {//���ڸ���resort���������
	return p1.second < p2.second;//��������
}

//����ıȽϹ���
bool cmp2(const WEIGHT& p1, const WEIGHT& p2) {
	return p1.proportion > p2.proportion;//����Ȩ�ش�С�������У�Ȩ�ش������ǰ��
}

bool cmp1(const MINISERT& p1, const MINISERT& p2) {//���ڰ��ս�Լ�ĳɱ��������в�ͬ���뷽ʽ
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
	int selected = 0;//��ǰѡ��ĵ�
	double load = 0;//����ۻ�����

	while (unarranged.size() != 0) {


		//map<int, double> compair;//�º���ʹmap�������أ�����valueֵ��������
		double nearDis = 0;
		vector<pair<int, double>> arr;

		for (int i = 0; i < unarranged.size(); i++) {
			nearDis = disMtx[selected][unarranged[i]];
			//compair.insert(make_pair(i, nearDis));
			arr.push_back(make_pair(unarranged[i], nearDis));
		}
		sort(arr.begin(), arr.end(), cmp);
		//cout << "������뵱ǰ��" << selected << "���еľ���" << endl;
		/*for (auto it = arr.begin(); it != arr.end(); it++) {
			cout << it->first << "\t" << it->second << endl;
		}*/
		auto iter = arr.begin();
		//������Ӹõ��Ƿ�ᳬ��


		/*cout << "��һ��"<< iter->first <<"������Ϊ " << vertex[iter->first].demand<<endl;
		cout << "��ǰ��" << selected << "������Ϊ " << vertex[selected].demand << endl;
		cout << "��һ��" << iter->first << "������Ϊ " << vertex[iter->first].demand << endl;*/
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
			/*cout << "��ǰ��" << selected << "����һ��" << iter->first << "������仯Ϊ " << endl
				<< vertex[selected].demand << "--->" << vertex[iter->first].demand << endl;*/

			current.push_back(iter->first);//��ž�������ĵ��ڵ�ǰ·��
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
	INITIAL c = s;//��ǰ��
	INITIAL g = s;//���Ž�
	int maxIter = iter_max_times;
	int count = 0;
	double global_opt = 0;
	double local_opt = 0;

	//Ϊ�˱����ظ���������Ҫʹ�ý��ɱ�
	//vector<pair<int, int>> tabuList;
	while (count < maxIter) {

		int route1 = rand() % c.truck_route.size();
		int route2 = rand() % c.truck_route.size();
		if (route1 == route2) {//���ѡ����ǵ���·��
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
		else {//ѡ���������·�ߣ���ѡȡĳһ�λ���
			int route1First = rand() % (c.truck_route[route1].size() - 2) + 1;
			int route1Second = rand() % (c.truck_route[route1].size() - 2) + 1;

			int route2First = rand() % (c.truck_route[route2].size() - 2) + 1;
			int route2Second = rand() % (c.truck_route[route2].size() - 2) + 1;

			int Class;
			if (route1First == route1Second && route2First == route2Second) {
				Class = 0;//����·�߶���ѡ���һ����
			}
			if (route1First == route1Second && route2First != route2Second) {
				if (route2First > route2Second) {
					int temp = route2First;
					route2First = route2Second;
					route2Second = temp;
				}
				Class = 1;//route1 ѡ��һ���㣬 route2 ѡ��������
			}
			if (route1First != route1Second && route2First == route2Second) {
				if (route1First > route1Second) {
					int temp = route1First;
					route1First = route1Second;
					route1Second = temp;
				}
				Class = 2;//route1 ѡ�������㣬 route2 ѡ��һ����
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
				Class = 3;//����·�߶���ѡ���������
			}
			switch (Class) {
			case 0:
				//�����Ľ�����������
			{int temp = c.truck_route[route1][route1First];
			c.truck_route[route1][route1First] = c.truck_route[route2][route2First];
			c.truck_route[route2][route2First] = temp; }
			break;
			case 1:
				//������н���
			{int selected = c.truck_route[route1][route1First];
			c.truck_route[route1].erase(c.truck_route[route1].begin() + route1First);//�����

			for (auto it = c.truck_route[route2].begin() + route2First; it != c.truck_route[route2].begin() + route2Second; it++) {
				c.truck_route[route1].insert(c.truck_route[route1].begin() + route1First, *it);//��ת����
			}
			for (int i = route2First; i < route2Second; i++) {
				c.truck_route[route2].erase(c.truck_route[route2].begin() + route2First);//ȫ�����
			}
			c.truck_route[route2].insert(c.truck_route[route2].begin() + route2First, selected);//�����
			}
			break;
			case 2:
				//������н���
			{int selected = c.truck_route[route2][route2First];
			c.truck_route[route2].erase(c.truck_route[route2].begin() + route2First);//�����

			for (auto it = c.truck_route[route1].begin() + route1First; it != c.truck_route[route1].begin() + route1Second; it++) {
				c.truck_route[route2].insert(c.truck_route[route2].begin() + route2First, *it);//��ת����
			}
			for (int i = route1First; i < route1Second; i++) {
				c.truck_route[route1].erase(c.truck_route[route1].begin() + route1First);//ȫ�����
			}
			c.truck_route[route1].insert(c.truck_route[route1].begin() + route1First, selected);//�����
			}
			break;
			case 3:
				//���к����н���
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
				//�������
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
	INITIAL c = s;//��ǰ��
	INITIAL g = s;//���Ž�
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
			cout << "·��" << route1 << "�ܴ�СΪ " << c.truck_route[route1].size() << endl;
			cout << "��ȡ�ĵ�����·�� " << route1 << " �еĵ� " << index1 << " λ" << endl;

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
	//����һ�����ϴ�����е����˻����е�
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

	vector<MINISERT> uav_route;//���ѡ�������˻�·��

	//��ʼ������ȡ���˻�·��
	for (int i = 0; i < candidates.size(); i++) {
		bool skip = false;
		//��Ϊ����� forѭ�� �Ѿ��ڱ����ˣ����Բ���Ҫɾ����ֻ��Ҫ�����Ѿ����������˻�·���еĵ�ֱ����������
		for (auto iter1 = uav_route.begin(); iter1 != uav_route.end(); iter1++) {
			if (candidates[i] == iter1->lau || candidates[i] == iter1->dro || candidates[i] == iter1->rdz) {
				skip = true;
			}
		}
		if (skip) {
			//cout << candidates[i] << "��·�����Ѿ�����" << endl;
			;
		}
		else {
			int currentNode = candidates[i];
			double sub_cost;//�ӿ���·�����Ƴ��õ����ٵĳɱ�
			for (int j = 0; j < s.truck_route.size(); j++) {
				for (int k = 1; k < s.truck_route[j].size() - 1; k++) {
					if (currentNode == s.truck_route[j][k]) {
						//�ӿ���·���Ƴ���ǰ���Լ�����ĳɱ�����ǰ�Ǹ���
						sub_cost = (disMtx[s.truck_route[j][k - 1]][s.truck_route[j][k + 1]] -
							(disMtx[s.truck_route[j][k - 1]][s.truck_route[j][k]] + disMtx[s.truck_route[j][k]][s.truck_route[j][k + 1]])) * t_c;
						s.truck_route[j].erase(s.truck_route[j].begin() + k);//��ԭ·�����Ƴ���ǰ���˻���ѡ��
					}
				}
			}
			bool* feasi = new bool[s.truck_route.size()];//�����жϵ�ǰ�����ĳ��·��ʱ�Ƿ���
			for (int j = 0; j < s.truck_route.size(); j++) {
				double acLoad = 0;
				for (int k = 0; k < s.truck_route[j].size(); k++) {
					acLoad += vertex[s.truck_route[j][k]].demand;
				}
				if (uav_route.size() > 0) {
					for (auto iter_uav = uav_route.begin(); iter_uav != uav_route.end(); iter_uav++) {
						if (iter_uav->routeIndex == j) {
							acLoad += vertex[iter_uav->dro].demand;//����ϲ����·�������˻�������
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
			vector<MINISERT> add_cost_m;//���������˻�֮���ܳɱ��ı仯

			for (int j = 0; j < s.truck_route.size(); j++) {
				if (feasi[j] == 1) {//�����ǰ·�߲������˻����ᳬ�أ������ѡ����

					for (int k = 0; k < s.truck_route[j].size() - 1; k++) {
						//1���жϵ�ǰѡ��Ļ��Ƿ��������˻�
						//if ()
						//���鵱ǰ��ɵ�·���Ƿ����
						/*cout << "·�� {" << s.truck_route[j][k] << " " << currentNode << " " << s.truck_route[j][k + 1] << "}" <<
							judgeFeaibility(s.truck_route[j][k], currentNode, s.truck_route[j][k + 1], uav_route) << endl;*/
						if (judgeFeaibility(s.truck_route[j][k], currentNode, s.truck_route[j][k + 1], uav_route)) {
							double add = (disMtx[s.truck_route[j][k]][currentNode] + disMtx[currentNode][s.truck_route[j][k + 1]]) * u_c;
							//cout << "������ " << sub_cost << "  ������" << add << " ";
							double saving = -(add + sub_cost);
							//cout << "savings = " << saving << endl;
							if (saving >= 0) {
								add_cost_m.push_back({ s.truck_route[j][k], currentNode,s.truck_route[j][k + 1], j, k + 1, saving });
								//������	[ ��ɵ㡢���˻��㡢����㡢����·����·�����±ꡢ��Լֵ ]
							}
						}
					}
				}
			}
			sort(add_cost_m.begin(), add_cost_m.end(), cmp1);//
			//cout << "��������� " << currentNode << " ��ɵ��������˻�·��" << endl;

			//ѡ�� ��Լֵ��� ����Ϊ���˻�·�߼���
			if (add_cost_m.size() > 0) {
				auto iter = add_cost_m.begin();
				//cout << iter->lau << "\t" << iter->dro << "\t" << iter->rdz << "\t" << iter->addCost << endl;

				uav_route.push_back(*iter);
			}
			s = s2;
			delete[] feasi;//�ͷ��ڴ�
		}

	}
	if (uav_route.size() > 0) {
		//1�����ԭ·���е����˻���
		for (int i = s1.solution.size() - 1; i >= 0; i--) {
			for (auto it = uav_route.begin(); it != uav_route.end(); it++) {
				if (s1.solution[i].upper == it->dro) {
					s1.solution.erase(s1.solution.begin() + i);
				}
			}
		}

		//2������Щ���˻��㰴������·�����²���ԭ·��
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
bool judgeFeaibility(int l, int d, int r, vector<MINISERT>& uav_route)//�����жϵ�ǰ3�����ܷ�������˻�·��
{
	double firstPart = disMtx[l][d] / u_v;//��һ�η���ʱ��
	double secondPart = disMtx[d][r] / u_v;//�ڶ��η���ʱ��
	double temdom = disMtx[l][r] / t_v;//��������ʻʱ��

	bool check_exist;
	for (auto it = uav_route.begin(); it != uav_route.end(); it++) {
		if (//1��l�� �� r�㶼�Ѿ�����������·����
			it->lau == l || it->rdz == r
			) {
			return false;
		}
		if (//2��l�� ���� r�����Ѿ�������˻�·�ߵ����˻���
			it->dro == l || it->dro == r
			) {
			return false;
		}
	}

	if (firstPart + secondPart <= temdom && firstPart * (u_w + vertex[d].demand) * consumeRate +
		secondPart * u_w * consumeRate <= u_b) {//���ʱ�������Լ������������㣬������
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
	int count = 0;//��ǰ��������
	int maxIter = 40;//����������
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


/*        ��Ҫ�ĸĽ��㷨����         */
SOLUTION VNS(SOLUTION& s) {
	_SOLUTION_WEIGHT_ best, current;
	best.solution = s.solution;
	current.solution = s.solution;
	int count = 0;//��ǰ��������
	int maxIter = V;//����������
		//����һ�����飬������ŶԸ��������Ȩ��ֵ����
	vector<WEIGHT> weight_values;
	for (int i = 0; i < 8; i++) {
		weight_values.push_back({ i, 0 });//ÿ���ط���Ȩ�ض���ʼ��Ϊ0
	}

	while (count < maxIter) {
		shaking(current);
		/*cout << "ÿ�������Ȩ��Ϊ�� " << endl;
		for (int i = 0; i < 7; i++) {
			cout << "����" << weight_values[i].index << "\t" << "Ȩ�أ�" << weight_values[i].proportion << endl;
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

	/*cout << "VND���������������" << endl;
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
	/*���ѡ��һ���㣬���ǿ��е㣬���ԭ·�����޳�����*/
	double standard_cost = f(s);
	int selectedIndex = rand() % (s.solution.size() - 2) + 1;
	while (s.solution[selectedIndex].upper == 0 || vertex[s.solution[selectedIndex].upper].demand > u_Q) {
		selectedIndex = rand() % (s.solution.size() - 2) + 1;
	}
	int selectedNode = s.solution[selectedIndex].upper;
	//��ԭ·�����޳�
	s.solution.erase(s.solution.begin() + selectedIndex);
	SOLUTION c;//���ڼ�¼��ǰ���ŵĲ��뷽ʽ
	//ѡ��һ�Ի����� ����������ѡ��·����������������i j�������Ǽ��һ����������i j k��ѡ����ߵĻ���������Ǹ�����Ҫ�ڽ��������i k j
	//�������Ƿ������˻��㣬ͳһ��Ϊi��j
	SOLUTION s1 = s;//��¼δ����֮ǰ�Ľ�
	for (int i = 1; i < s.solution.size() - 1; i++) {
		//��һ�֣������ŵ�������֮��

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
		//�ڶ��֣� �������м��һ����
		s.solution.insert(s.solution.begin() + i, { selectedNode, 1 });
		int temp = s.solution[i + 1].upper;
		s.solution[i + 1].upper = s.solution[i + 2].upper;
		s.solution[i + 2].upper = temp;
		s.solution[i + 1].lower = 0;
		//��Ҫע�⣬���ܰ����һ�������˷� 0 ��
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

//�ض�λΪ���˻�������
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

//�������˻�·�ߵ�����
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
	//����������
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
		//����������
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
		//����������

		SOLUTION_ELEMENT temp1 = s.solution[first];
		s.solution[first] = s.solution[second];
		s.solution[second] = temp1;
		//1���������ȥ�ĵ������˻��㣬���ǻ���ȥ֮�����߶������˻��㣬��ѻ���ȥ��������Ϊ������
		//if (s.solution[first - 1].lower == 1 && s.solution[first].lower == 1 && s.solution[first + 1].lower == 1) {
		//	s.solution[first].lower = 0;
		//}
		//if (s.solution[second - 1].lower == 1 && s.solution[second].lower == 1 && s.solution[second + 1].lower == 1) {
		//	s.solution[second].lower = 0;
		//}
		////2���������ȥ�������˻��㣬���ǻ���ȥ֮��ֻ��һ�������˻��㣬��ѻ���ȥ�ĵ��Ϊ�����㣬��һ�ߵĵ��Ϊ���˻���  
		//// 1 -> 1 (1) 0  => 1 (0) 0  : 1 (1) 0 => 0 (1) 0
		//if (s.solution[first].lower == 1) {
		//	if (s.solution[first - 1].lower == 1 && s.solution[first + 1].lower == 0) {
		//		int choose = rand() % 2;
		//		if (choose == 0) {
		//			s.solution[first].lower = 0;
		//		}
		//		else{ s.solution[first - 1].lower = 0; }
		//		//����ѡ�����ɱ���͵ķ�ʽ
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
		//		//����ѡ�����ɱ���͵ķ�ʽ
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
		//����������
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
		//��������ĵ�
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

		//����relocateһ��




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

////�������˻�·�ߵ�����
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
	readData();//��ȡ���ݣ���ʼ���������
	cout << "����������ͬ·�ε�ʱ��" << endl;
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

	INITIAL s = nearNeiborConstru();//����������vrp
	cout << "��������⣺" << f1(s) << endl;
	for (int i = 0; i < s.truck_route.size(); i++) {
		for (auto it = s.truck_route[i].begin(); it != s.truck_route[i].end(); it++) {
			cout << *it << "\t";
		}
		cout << endl;
	}
	//ʹ�þ��� 2-Opt ȥ�� VRP ·��Ѱ��

	s = vns(s);
	cout << "�Ż���ĳ�ʼ�⣺" << f1(s) << endl;
	for (int i = 0; i < s.truck_route.size(); i++) {
		for (auto it = s.truck_route[i].begin(); it != s.truck_route[i].end(); it++) {
			cout << *it << "\t";
		}
		cout << endl;
	}


	int sLenth = s.truck_route.size();

	SOLUTION solut = init_vrp_d(s, sLenth);
	//solut = elimDoubleZero(solut);
	cout << "��ȡ��ĳ�ʼ�⣺" << newf(solut) << endl;
	//cout << "�µļ��㷽���� " << newf(solut) << endl;
	printS(solut);

	//��ʼ local search �Ľ��������
	solut = VNS(solut);
	cout << "�Ľ���Ľ⣺" << newf(solut) << endl;

	printS(solut);
	end = clock();
	double time = double(end - start) / CLOCKS_PER_SEC;
	cout << "�ܼ���ʱ " << time << "��" << endl;
	system("pause");
	ofstream out;
	out.open("C://Users/Administrator/Desktop/����ֲ����ݼ�/�������ݼ�/17/R203+_����ӦVNS_result.txt");
	////out.open("C://Users/Ridge/Desktop/�����Ų������ݼ�/C101 to C109/large_size/50/c108_VNS_result.txt");

	out << "Ŀ��ֵ��" << newf(solut) << endl;
	out << "·��Ϊ��" << endl;
	for (int i = 0; i < solut.solution.size(); i++) {
		out << solut.solution[i].upper << "\t";
	}out << endl;
	for (int i = 0; i < solut.solution.size(); i++) {
		out << solut.solution[i].lower << "\t";
	}
	out << endl;
	out << "�ܼ���ʱ " << time << "��" << endl;
	out.close();
	return 0;
}