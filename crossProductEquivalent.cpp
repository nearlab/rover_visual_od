#include <iostream>
#include <Eigen/Dense>


//Cross Product Equivalent
using Eigen::Matrix3f;
using Eigen::Vector3f;
Matrix3f crossProductEquivalent(Vector3f v)
{
  std::cout << v <<std::endl;
  std::cout << v(2) <<std::endl;
  Matrix3f c;
  c << 0, -v(2), v(1),
       v(2), 0, -v(0),
       -v(1), v(0), 0;
  std::cout << c << std::endl;
  return c;
}

int main()
{
  Vector3f v;
  v << 1,
       2,
       3;
  crossProductEquivalent(v);
}
