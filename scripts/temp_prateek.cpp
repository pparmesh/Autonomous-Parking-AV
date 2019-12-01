#include <eigen3/Eigen/Dense>
#include <iostream>
#include <math.h>
#include <unordered_map>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using namespace std;

int main()
{
  MatrixXd m(2,2);
  m(0,0) = 3;
  m(1,0) = 2.5;
  m(0,1) = -1;
  m(1,1) = m(1,0) + m(0,1);
  MatrixXd a(10,2);
//   cout<<a<<endl;
//   a.block<2,2>(0,0) = m;
//   cout<<a;
for (int i=0;i<a.rows();i=i+2)
{
    a.block<2,2>(i,0) = m;
    // i = i+2;
}
cout<<a;
// unordered_map<int,MatrixXd> lol;
// lol[1] = m;
// // cout<<lol[1];
}