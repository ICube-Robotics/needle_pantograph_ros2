/* To run tests go to ws_pantograph dir then
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
    cd build/pantograph_library/ && ctest*/

#include <gtest/gtest.h>
#include "pantograph_library/pantograph_model.hpp"

/* Test of the FKM of the pantograph by comparing the coords of P3 calculated 
by the C++ model and the same coords calculate by the MATLAB model*/

TEST(TestModel, TestPantoFKM) { 
  pantograph_library::PantographModel panto_model_{} ;
  Eigen::Vector<double, 2> q; 
  Eigen::Vector<double, 2> P3_cpp, P3_matlab;

  double error = 1e-10;

  // q values according to MATLAB model
  q[0] = 0.5; 
  q[1] = pantograph_library::PI_CST - 0.5; // 2.64159265358979
  
  // Coords of P3 according to MATLAB model
  P3_matlab[0] = -0.0425000000000000;
  P3_matlab[1] = 0.149223285959824;

  // Coords of P3 according to C++ model
  P3_cpp = panto_model_.fk(q);

  /*Expect values to be equal (up to a 1e-10 error) between 
  the C++ and the MATLAB model */

  EXPECT_NEAR(P3_cpp[0],P3_matlab[0],error);
  EXPECT_NEAR(P3_cpp[1],P3_matlab[1],error);
}

/* Test of the IKM of the pantograph by comparing the coords of the joints calculated 
by the C++ model and the same coords calculate by the MATLAB model*/

TEST(TestModel, TestPantoIKM) { 
  pantograph_library::PantographModel panto_model_ ;
  Eigen::Vector<double, 2> q_cpp, q_matlab, P3;
  Eigen::Vector<double, 5> joints;

  double error = 1e-10;

  // P3 coords according to MATLAB model
  P3[0] = -0.0425000000000000;
  P3[1] = 0.149223285959824;

  // q values according to MATLAB model
  q_matlab[0] = 0.500000000000000;
  q_matlab[1] = 2.64159265358979;
  
  // q values according to C++ model
  joints = panto_model_.ik(P3);
  q_cpp[0] = joints[0];
  q_cpp[1] = joints[4];

  /* Expect values to be equal (up to a 1e-10 error) between 
  the C++ and the MATLAB model  */
  // Actuated joints
  EXPECT_NEAR(q_cpp[0],q_matlab[0], error);
  EXPECT_NEAR(q_cpp[1],q_matlab[1], error);

  // Other joints for visual render
}



/* Test of the FKM of the complete system by comparing the coords of PU calculated 
by the C++ model and the same coords calculate by the MATLAB model*/

TEST(TestModel, TestSystemFKM) { 
  pantograph_library::PantographModel panto_model_{} ;
  Eigen::Vector<double, 2> q; 
  Eigen::Vector<double, 3> PU_cpp, PU_matlab;

  double error = 1e-10;

  // q values according to MATLAB model
  q[0] = 0.5; 
  q[1] = 2.64159265358979; pantograph_library::PI_CST - 0.5; 

  // Coords of PU according to MATLAB model
  PU_matlab[0] = 0.0942524755499338;
  PU_matlab[1] = 0.167462388425657;
  PU_matlab[2] = 0.144796738817577;

  // Coords of PU according to C++ model
  PU_cpp = panto_model_.fk_system(q); 
  
  /*Expect values to be equal (up to a 1e-10 error) between 
  the C++ and the MATLAB model */
  EXPECT_NEAR(PU_cpp[0],PU_matlab[0],error);
  EXPECT_NEAR(PU_cpp[1],PU_matlab[1],error);
  EXPECT_NEAR(PU_cpp[2],PU_matlab[2],error);
}

/* Test of the IKM of the system by comparing the values of q calculated 
by the C++ model and the same coords calculate by the MATLAB model*/

TEST(TestModel, TestIKM) {
  pantograph_library::PantographModel panto_model_ ;
  Eigen::Vector<double, 2> q_cpp, q_matlab;
  Eigen::Vector<double, 7> joints;
  Eigen::Vector3d PU_matlab;

  double error = 1e-10;

  // PU coords according to MATLAB model
  PU_matlab[0] = 0.0942524755499338;
  PU_matlab[1] = 0.167462388425657;
  PU_matlab[2] = 0.144796738817577;

  // q values according to MATLAB model
  q_matlab[0] = 0.500000000000000;
  q_matlab[1] = 2.64159265358979; // PI_CST - 0.5 
  
  // q values according to C++ model
  joints = panto_model_.ik_system(PU_matlab);
  q_cpp[0] = joints[0];
  q_cpp[1] = joints[4];

  /* Expect values to be equal (up to a 1e-10 error) between 
  the C++ and the MATLAB model  */
  EXPECT_NEAR(q_cpp[0],q_matlab[0], error);
  EXPECT_NEAR(q_cpp[1],q_matlab[1], error);
}
