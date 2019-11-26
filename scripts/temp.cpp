#include <iostream>
#include <bits/stdc++.h>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

 int main()
 {
 	
	//  Eigen matrix test
	Matrix<double, 2,2> M1;
 	// M1.row(0) << 2.0,1.0;
 	// M1.row(1) << 3.0, 4.0;
 	M1<<2.0, 3.0, 1.0, 5.0;
 	cout<<" matrix is: \n"<<M1<<endl;
 	cout<<M1.row(1)[0]<<endl;

	// //--------------
	// set<int> pp;
	// pp.insert(55);
	// pp.insert(25);
	// pp.insert(100);
	// pp.insert(2);
	// for(int h : pp)
	// {
	// 	cout<<h<<"\n";
	// 	pp.erase(pp.begin());
	// 	cout<<"pp size: "<<pp.size()<<endl;
	// } 
	// cout<<endl;

	MatrixXd m(3,4);
	m.resize(NoChange, 459);
	cout << "m: " << m.rows() << " rows, " << m.cols() << " cols" << endl;

	double a = 0;
	string f = to_string(a);
	cout<<f.substr(0, 4)<<endl;

	int ss = 9;
	vector<vector<int>> p (ss,vector<int> (ss,0));
	cout<<p.size()<<"-"<< p[0].size()<<endl;
 	return 0;
 }
