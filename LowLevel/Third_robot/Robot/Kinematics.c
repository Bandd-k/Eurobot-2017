 #define DELTA 1
 #define DELTA_ROT 1.095
 #define POST1_3 1.300
 float MLineSpeed[4][3] = {  0.707*POST1_3,   0.707*POST1_3, 0.0,  // матрица расчета линейных скоростей
                             0.707,  -0.707, 0.0,
                            -0.707*POST1_3,  -0.707*POST1_3, 0.0,
                            -0.707,   0.707, 0.0};
  float MRotSpeed[4][3] = { 0.0, 0.0, -0.162 , // матрица расчета линейных скоростей
                            0.0, 0.0, -0.162,
                            0.0, 0.0, -0.162,
                            0.0, 0.0, -0.162};
// Changy to this

// float MLineSpeed[4][3] = {  -0.707,  -0.707, 0.0,  // матрица расчета линейных скоростей
//                             -0.707,   0.707, 0.0,
//                              0.707,   0.707, 0.0,
//                              0.707,  -0.707, 0.0};

//float MRotSpeed[4][3] =   { 0.0, 0.0, 0.162 , // матрица расчета линейных скоростей
//                            0.0, 0.0, 0.162,
//                            0.0, 0.0, 0.162,
//                            0.0, 0.0, 0.162};

//float InverseKinematics[4][4] =  {
//    -0.3536,   0.3536,   0.3536,    -0.3536,
//    -0.3536,   0.3536,   0.3536, -0.3536,
//    1.5432, 1.5432, 1.5432,  1.5432,
//    10.0000, -10.0000,   10.0000,  -10.0000};

float InverseKinematics[4][4] =  {
    0.3536,   0.3536,   -0.3536,    -0.3536,
    0.3536,   -0.3536,   -0.3536,    0.3536,
    -1.5432*DELTA_ROT, -1.5432*DELTA_ROT, -1.5432*DELTA_ROT,  -1.5432*DELTA_ROT,
    10.0000, -10.0000,   10.0000,  -10.0000};

