//
//  test_vec2_mat2x2.h
//
//  Created by Jonathan Tompson on 4/26/12.
//

#include "tests/test_unit/test_unit.h"
#include "math/math_types.h"
#include "math/math_base.h"

namespace tests {

  using math::Quat;
  using math::Mat3x3;
  using math::Mat4x4;

  TEST(Quaternion, SimpleManipulation) {
    Vec3<double> axis(1.817168119886437e-001, 6.198030601001485e-001, 
      -7.634285604633715e-001);
    double angle = 8.621733203681206e-001;

    Quat<double> Q1(&axis, angle);  // axis angle -> quaternion
    Quat<double> Q1_expect(7.593187686640246e-002, 2.589898486876663e-001, 
      -3.190049551002597e-001, 9.085122161940182e-001);
    EXPECT_TRUE(Q1.equal(&Q1_expect));

    Mat3x3<double> RotMat1;
    Q1.quat2Mat3x3(&RotMat1);  // quaternion -> matrix
    Mat3x3<double> RotMat1_expect(6.623201937964420e-001, 
      6.189709680704206e-001, 4.221455928650800e-001, -5.403086268696203e-001, 
      7.849403773940535e-001, -3.032081655673973e-001, -5.190361727468607e-001,
      -2.726801464073089e-002, 8.543172167045705e-001);
    EXPECT_TRUE(RotMat1.equal(&RotMat1_expect));

    Mat4x4<double> RotMat2;
    Q1.quat2Mat4x4(&RotMat2);  // quaternion -> matrix
    Mat4x4<double> RotMat2_expect(6.623201937964420e-001, 
      6.189709680704206e-001, 4.221455928650800e-001, 0, 
      -5.403086268696203e-001, 7.849403773940535e-001, -3.032081655673973e-001,
      0, -5.190361727468607e-001, -2.726801464073089e-002, 
      8.543172167045705e-001, 0, 0, 0, 0, 1);
    EXPECT_TRUE(RotMat2.equal(&RotMat2_expect));

    Quat<double> Q2(&RotMat1);  // matrix --> quaternion
    Quat<double> Q2_expect(7.593187686640246e-002, 2.589898486876663e-001, 
      -3.190049551002597e-001, 9.085122161940182e-001);
    EXPECT_TRUE(Q2.equal(&Q2_expect));

    Quat<double> Q3(&RotMat2);  // matrix --> quaternion
    Quat<double> Q3_expect(7.593187686640246e-002, 2.589898486876663e-001, 
      -3.190049551002597e-001, 9.085122161940182e-001);
    EXPECT_TRUE(Q3.equal(&Q3_expect));
  }

  }  // tests namespace
