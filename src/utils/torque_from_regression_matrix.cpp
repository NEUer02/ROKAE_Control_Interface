//
// File: torque_from_regression_matrix.cpp
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 31-Jul-2023 17:34:56
//

// Include Files
#include "torque_from_regression_matrix.h"
#include <cmath>
#include <array>

// Function Definitions
//
// REGRESSION_MATRIX 此处显示有关此函数的摘要
//    此处显示详细说明
// Arguments    : const double q[7]
//                const double qd[7]
//                const double qdd[7]
//                double torque[7]
// Return Type  : void
//

using namespace std;

void torque_from_regression_matrix(const array<double, 7> &q, const array<double, 7> &qd, const array<double, 7> &qdd,
                                   array<double, 7> &torque) {
    static const double b[57] = {-0.32836398858277, 1.5855130573224,
                                 -0.219051390295189, 0.402156188219086, -0.00285713673628657,
                                 1.93569635394284, 0.378541240349284, -4.94441890772241, 0.197566699585055,
                                 0.118778160323032, -0.181910048528265, -0.0658304172646458,
                                 0.150220038244628, -0.122625073572199, -0.0483316151667845,
                                 0.849921593077519, 0.0663613489233306, 0.0778770100581264,
                                 -0.0175936305666053, 1.04098045193815, -0.0366450200130397,
                                 -2.10562376322665, -0.15482433284591, 0.101690238915986, 0.09590426439157,
                                 0.0256050474321042, 0.00630741524594686, 0.0982986941071783,
                                 -0.0356105732348678, 0.0547828062954622, -0.0135325918391205,
                                 0.0947397861922231, -0.108140915366554, -0.055470450145362,
                                 -0.10153300487162, -0.395111924221073, 0.0658008361449755,
                                 -0.0463183052588407, 0.00273436811269465, 0.073807647828821,
                                 -0.00782965990054055, 0.0061444888776852, 0.0193664941691511,
                                 0.0411880769243002, 0.813950364720969, 0.752198215833686, 0.208186712319382,
                                 0.180016126389841, 0.623878355793798, 0.4426965196746, 0.378895627892184,
                                 0.196544748341519, 0.26152194092663, 0.423742063845478, -0.0654102000029361,
                                 0.582084028387708, -0.02150576097161};

    double b_qdd[399];
    double A112, A113, A114, A115, A116, A117;
    double A313, A315, A317;
    double DV22_tmp, DV24_tmp;
    double DV23, DV25, DV26, DV27;
    double DV33_tmp, DV37_tmp;
    double DV53_tmp;
    double DV54, DV56, DV57;
    double DV67_tmp;
    double FDI41MX4, FDI41MX5, FDI41MX6, FDI41MY4, FDI41MY5, FDI41MY6;
    double FDI53MX5, FDI53MX6, FDI53MY5, FDI53MY6, FDI73MY7;
    double N31MX4, N31MX5, N31MX6, N31MX7;
    double N31MY5, N31MY6;
    double N31XX4, N31XX5, N31XX6;
    double N31XY4, N31XY5, N31XY6;
    double N31XZ4, N31XZ5, N31XZ6, N31XZ7;
    double N31YZ4, N31YZ5, N31YZ6;
    double N31ZZ4, N31ZZ5, N31ZZ6, N31ZZ7;
    double N33MX4, N33MX5, N33MX6, N33MX7;
    double N33MY5;
    double N33XX4, N33XX5, N33XX6;
    double N33XY4, N33XY5;
    double N33XY6, N33XY7;
    double N33XZ4, N33XZ5, N33XZ6;
    double N33YZ4, N33YZ5, N33YZ6, N33YZ7;
    double N33ZZ4, N33ZZ5, N33ZZ6;
    double N41XZ5;
    double N43MX5, N43MX6, N43MY5, N43MY6;
    double N43XX5, N43XX6;
    double N43XY5, N43XY6;
    double N43XZ5, N43XZ6;
    double N43YZ5, N43YZ6;
    double N43ZZ5, N43ZZ6, N43ZZ7;
    double N51MX6, N51MX7;
    double N53MX6, N53MX7;
    double N53XX6;
    double N53XY6;
    double N53XZ6;
    double N53YZ6;
    double N53ZZ6;
    double N63XY7;
    double N63YZ7;
    double No31XY3;
    double No31XZ3;
    double No31YZ3;
    double No32XY3;
    double No32XZ3;
    double No32YZ3;
    double No33XY3;
    double No33XZ3;
    double No33YZ3;
    double No43XY4;
    double No43XZ4;
    double No43YZ4;
    double No53XY5;
    double No53XZ5;
    double No53YZ5;
    double No63XY6;
    double No63XZ6;
    double No63YZ6;
    double No73XZ7;
    double U113;
    double U115;
    double U116;
    double U117;
    double U117_tmp;
    double U123;
    double U124;
    double U125;
    double U126;
    double U127;
    double U213;
    double U215;
    double U216;
    double U217;
    double U223;
    double U224;
    double U224_tmp;
    double U225;
    double U226;
    double U227;
    double U227_tmp;
    double U314;
    double U316;
    double U317;
    double U324;
    double U326;
    double U327;
    double VP12;
    double VP13;
    double VP14;
    double VP15;
    double VP16;
    double VP17;
    double VP22;
    double VP23;
    double VP24;
    double VP25;
    double VP26;
    double VP27;
    double VSP13;
    double VSP23;
    double VSP23_tmp;
    double VSP23_tmp_tmp;
    double VSP25;
    double VSP27;
    double VSP33;
    double W12;
    double W13;
    double W14;
    double W15;
    double W16;
    double W17;
    double W22;
    double W23;
    double W24;
    double W25;
    double W26;
    double W27;
    double W33;
    double W34;
    double W35;
    double W36;
    double W37;
    double WP12;
    double WP12_tmp;
    double WP13;
    double WP14;
    double WP15;
    double WP16;
    double WP17;
    double WP22;
    double WP22_tmp;
    double WP23;
    double WP24;
    double WP25;
    double WP26;
    double WP27;
    double WP33;
    double WP34;
    double WP35;
    double WP36;
    double WP37;
    double b_x;
    double x;
    double x_tmp;
    A112 = std::cos(q[1]);
    x = std::sin(q[1]);
    A113 = std::cos(q[2]);
    A313 = std::sin(q[2]);
    A114 = std::cos(q[3]);
    b_x = std::sin(q[3]);
    A115 = std::cos(q[4]);
    A315 = std::sin(q[4]);
    A116 = std::cos(q[5]);
    x_tmp = std::sin(q[5]);
    A117 = std::cos(q[6]);
    A317 = std::sin(q[6]);
    W12 = -x * qd[0];
    W22 = -A112 * qd[0];
    WP12_tmp = W22 * qd[1];
    WP12 = -x * qdd[0] + WP12_tmp;
    WP22_tmp = W12 * qd[1];
    WP22 = -A112 * qdd[0] - WP22_tmp;
    DV22_tmp = W12 * W22;
    VP12 = -x * 9.80665;
    VP22 = -A112 * 9.80665;
    W13 = A113 * W12 + A313 * qd[1];
    W23 = A113 * qd[1] - A313 * W12;
    W33 = -W22 + qd[2];
    WP13 = (A113 * WP12 + A313 * qdd[1]) + W23 * qd[2];
    WP23 = (A113 * qdd[1] - A313 * WP12) - W13 * qd[2];
    WP33 = -WP22 + qdd[2];
    DV23 = W13 * W23;
    DV33_tmp = W13 * W33;
    DV53_tmp = W23 * W33;
    N51MX7 = W33 * W33;
    FDI73MY7 = W23 * W23;
    U113 = -FDI73MY7 - N51MX7;
    U213 = DV23 + WP33;
    U123 = DV23 - WP33;
    N41XZ5 = W13 * W13;
    U223 = -N41XZ5 - N51MX7;
    VSP13 = -0.394 * (DV22_tmp - qdd[1]) + VP12;
    VSP23_tmp_tmp = W12 * W12;
    VSP23_tmp = qd[1] * qd[1];
    VSP23 = -0.394 * (-VSP23_tmp_tmp - VSP23_tmp) + VP22;
    VSP33 = -0.394 * (WP12_tmp + WP12);
    VP13 = A113 * VSP13 + A313 * VSP33;
    VP23 = A113 * VSP33 - A313 * VSP13;
    W14 = A114 * W13 + -b_x * W33;
    W24 = -A114 * W33 + -b_x * W13;
    W34 = W23 + qd[3];
    WP14 = (A114 * WP13 + -b_x * WP33) + W24 * qd[3];
    WP24 = (-A114 * WP33 + -b_x * WP13) - W14 * qd[3];
    WP34 = WP23 + qdd[3];
    DV24_tmp = W14 * W24;
    N33MX7 = W14 * W34;
    DV54 = W24 * W34;
    N31MX7 = W34 * W34;
    U314 = N33MX7 - WP24;
    U124 = DV24_tmp - WP34;
    U224_tmp = W14 * W14;
    U224 = -U224_tmp - N31MX7;
    U324 = DV54 + WP14;
    VP14 = A114 * VP13 - -b_x * VSP23;
    VP24 = A114 * VSP23 + -b_x * VP13;
    W15 = A115 * W14 + A315 * W34;
    W25 = A115 * W34 - A315 * W14;
    W35 = -W24 + qd[4];
    WP15 = (A115 * WP14 + A315 * WP34) + W25 * qd[4];
    WP25 = (A115 * WP34 - A315 * WP14) - W15 * qd[4];
    WP35 = -WP24 + qdd[4];
    DV25 = W15 * W25;
    N63YZ7 = W15 * W35;
    N33YZ7 = W25 * W35;
    N43ZZ7 = W35 * W35;
    N31ZZ7 = W25 * W25;
    U115 = -N31ZZ7 - N43ZZ7;
    U215 = DV25 + WP35;
    U125 = DV25 - WP35;
    N53MX7 = W15 * W15;
    U225 = -N53MX7 - N43ZZ7;
    VSP13 = -0.366 * U124 + VP14;
    VSP25 = -0.366 * U224 + VP24;
    VSP33 = -0.366 * U324 + VP23;
    VP15 = A115 * VSP13 + A315 * VSP33;
    VP25 = A115 * VSP33 - A315 * VSP13;
    W16 = A116 * W15 + -x_tmp * W35;
    W26 = -A116 * W35 + -x_tmp * W15;
    W36 = W25 + qd[5];
    WP16 = (A116 * WP15 + -x_tmp * WP35) + W26 * qd[5];
    WP26 = (-A116 * WP35 + -x_tmp * WP15) - W16 * qd[5];
    WP36 = WP25 + qdd[5];
    DV26 = W16 * W26;
    N63XY7 = W16 * W36;
    DV56 = W26 * W36;
    No73XZ7 = W36 * W36;
    N33XY7 = W26 * W26;
    U116 = -N33XY7 - No73XZ7;
    U216 = DV26 + WP36;
    U316 = N63XY7 - WP26;
    U126 = DV26 - WP36;
    N31XZ7 = W16 * W16;
    U226 = -N31XZ7 - No73XZ7;
    U326 = DV56 + WP16;
    VP16 = A116 * VP15 - -x_tmp * VSP25;
    VP26 = A116 * VSP25 + -x_tmp * VP15;
    W17 = A117 * W16 + A317 * W36;
    W27 = A117 * W36 - A317 * W16;
    W37 = -W26 + qd[6];
    WP17 = (A117 * WP16 + A317 * WP36) + W27 * qd[6];
    WP27 = (A117 * WP36 - A317 * WP16) - W17 * qd[6];
    WP37 = -WP26 + qdd[6];
    DV27 = W17 * W27;
    DV37_tmp = W17 * W37;
    DV57 = W27 * W37;
    DV67_tmp = W37 * W37;
    U117_tmp = W27 * W27;
    U117 = -U117_tmp - DV67_tmp;
    U217 = DV27 + WP37;
    U317 = DV37_tmp - WP27;
    U127 = DV27 - WP37;
    U227_tmp = W17 * W17;
    U227 = -U227_tmp - DV67_tmp;
    U327 = DV57 + WP17;
    VSP13 = -0.2503 * U126 + VP16;
    VSP27 = -0.2503 * U226 + VP26;
    VSP33 = -0.2503 * U326 + VP25;
    VP17 = A117 * VSP13 + A317 * VSP33;
    VP27 = A117 * VSP33 - A317 * VSP13;
    No31XY3 = -W13 * W33 + WP23;
    No32XY3 = DV53_tmp + WP13;
    No33XY3 = N41XZ5 - FDI73MY7;
    No31XZ3 = W13 * W23 + WP33;
    No32XZ3 = -(W13 * W13) + N51MX7;
    No33XZ3 = -W23 * W33 + WP13;
    No31YZ3 = FDI73MY7 - N51MX7;
    No32YZ3 = -W13 * W23 + WP33;
    No33YZ3 = DV33_tmp + WP23;
    W33 = -b_x * N33MX7;
    N31XX4 = A114 * WP14 + W33;
    N33XX4 = -A114 * N33MX7 + -b_x * WP14;
    VSP13 = -W14 * W34 + WP24;
    VSP33 = W24 * W34 + WP14;
    N51MX7 = W24 * W24;
    No43XY4 = U224_tmp - N51MX7;
    N31XY4 = A114 * VSP13 + -b_x * VSP33;
    N33XY4 = -A114 * VSP33 + -b_x * VSP13;
    VSP13 = DV24_tmp + WP34;
    VSP33 = -(W14 * W14) + N31MX7;
    No43XZ4 = -W24 * W34 + WP14;
    N31XZ4 = A114 * VSP13 + -b_x * VSP33;
    N33XZ4 = -A114 * VSP33 + -b_x * VSP13;
    VSP33 = N51MX7 - N31MX7;
    VSP13 = -W14 * W24 + WP34;
    No43YZ4 = N33MX7 + WP24;
    N31YZ4 = A114 * VSP33 + -b_x * VSP13;
    N33YZ4 = -A114 * VSP13 + -b_x * VSP33;
    N31ZZ4 = A114 * DV54 - W33;
    N33ZZ4 = A114 * N33MX7 + -b_x * DV54;
    FDI41MX4 = A114 * (-N51MX7 - N31MX7) + -b_x * (DV24_tmp + WP34);
    N31MX4 = b_x * VP23;
    N33MX4 = A114 * VP23;
    FDI41MY4 = A114 * U124 + -b_x * U224;
    N51MX7 = A315 * N63YZ7;
    VSP13 = A115 * WP15 - N51MX7;
    N43XX5 = A115 * N63YZ7 + A315 * WP15;
    N31XX5 = A114 * VSP13 + -b_x * DV25;
    N33XX5 = -A114 * DV25 + -b_x * VSP13;
    VSP13 = -W15 * W35 + WP25;
    FDI73MY7 = N33YZ7 + WP15;
    No53XY5 = N53MX7 - N31ZZ7;
    VSP33 = A115 * VSP13 - A315 * FDI73MY7;
    N43XY5 = A115 * FDI73MY7 + A315 * VSP13;
    N31XY5 = A114 * VSP33 - -b_x * No53XY5;
    N33XY5 = A114 * No53XY5 + -b_x * VSP33;
    VSP13 = W15 * W25 + WP35;
    VSP33 = -(W15 * W15) + N43ZZ7;
    No53XZ5 = -W25 * W35 + WP15;
    N41XZ5 = A115 * VSP13 - A315 * VSP33;
    N43XZ5 = A115 * VSP33 + A315 * VSP13;
    N31XZ5 = A114 * N41XZ5 - -b_x * No53XZ5;
    N33XZ5 = A114 * No53XZ5 + -b_x * N41XZ5;
    VSP33 = N31ZZ7 - N43ZZ7;
    VSP13 = -W15 * W25 + WP35;
    No53YZ5 = N63YZ7 + WP25;
    W33 = A115 * VSP33 - A315 * VSP13;
    N43YZ5 = A115 * VSP13 + A315 * VSP33;
    N31YZ5 = A114 * W33 - -b_x * No53YZ5;
    N33YZ5 = A114 * No53YZ5 + -b_x * W33;
    VSP13 = A115 * N33YZ7 + N51MX7;
    N43ZZ5 = -A115 * N63YZ7 + A315 * N33YZ7;
    N31ZZ5 = A114 * VSP13 - -b_x * WP35;
    N33ZZ5 = A114 * WP35 + -b_x * VSP13;
    VSP13 = A115 * U115 - A315 * U215;
    FDI53MX5 = A115 * U215 + A315 * U115;
    W33 = -A315 * VSP25;
    VSP33 = W33 - 0.366 * FDI53MX5;
    N43MX5 = A115 * VSP25 + 0.366 * VSP13;
    FDI41MX5 = A114 * VSP13 - -b_x * (N63YZ7 - WP25);
    N31MX5 = A114 * VSP33 - -b_x * VP25;
    N33MX5 = A114 * VP25 + -b_x * VSP33;
    VSP33 = A115 * U125 - A315 * U225;
    FDI53MY5 = A115 * U225 + A315 * U125;
    VSP13 = -A115 * VSP25 - 0.366 * FDI53MY5;
    N43MY5 = W33 + 0.366 * VSP33;
    FDI41MY5 = A114 * VSP33 - -b_x * FDI73MY7;
    N31MY5 = A114 * VSP13 + -b_x * VP15;
    N33MY5 = -A114 * VP15 + -b_x * VSP13;
    N51MX7 = -x_tmp * N63XY7;
    VSP13 = A116 * WP16 + N51MX7;
    N53XX6 = -A116 * N63XY7 + -x_tmp * WP16;
    VSP33 = A115 * VSP13 + A315 * DV26;
    N43XX6 = -A115 * DV26 + A315 * VSP13;
    N31XX6 = A114 * VSP33 - -b_x * N53XX6;
    N33XX6 = A114 * N53XX6 + -b_x * VSP33;
    VSP13 = -W16 * W36 + WP26;
    VSP33 = W26 * W36 + WP16;
    No63XY6 = N31XZ7 - N33XY7;
    W33 = A116 * VSP13 + -x_tmp * VSP33;
    N53XY6 = -A116 * VSP33 + -x_tmp * VSP13;
    VSP13 = A115 * W33 - A315 * No63XY6;
    N43XY6 = A115 * No63XY6 + A315 * W33;
    N31XY6 = A114 * VSP13 - -b_x * N53XY6;
    N33XY6 = A114 * N53XY6 + -b_x * VSP13;
    VSP13 = W16 * W26 + WP36;
    VSP33 = -(W16 * W16) + No73XZ7;
    No63XZ6 = -W26 * W36 + WP16;
    W33 = A116 * VSP13 + -x_tmp * VSP33;
    N53XZ6 = -A116 * VSP33 + -x_tmp * VSP13;
    VSP13 = A115 * W33 - A315 * No63XZ6;
    N43XZ6 = A115 * No63XZ6 + A315 * W33;
    N31XZ6 = A114 * VSP13 - -b_x * N53XZ6;
    N33XZ6 = A114 * N53XZ6 + -b_x * VSP13;
    W33 = N33XY7 - No73XZ7;
    VSP13 = -W16 * W26 + WP36;
    No63YZ6 = N63XY7 + WP26;
    VSP33 = A116 * W33 + -x_tmp * VSP13;
    N53YZ6 = -A116 * VSP13 + -x_tmp * W33;
    VSP13 = A115 * VSP33 - A315 * No63YZ6;
    N43YZ6 = A115 * No63YZ6 + A315 * VSP33;
    N31YZ6 = A114 * VSP13 - -b_x * N53YZ6;
    N33YZ6 = A114 * N53YZ6 + -b_x * VSP13;
    VSP13 = A116 * DV56 - N51MX7;
    N53ZZ6 = A116 * N63XY7 + -x_tmp * DV56;
    VSP33 = A115 * VSP13 - A315 * WP36;
    N43ZZ6 = A115 * WP36 + A315 * VSP13;
    N31ZZ6 = A114 * VSP33 - -b_x * N53ZZ6;
    N33ZZ6 = A114 * N53ZZ6 + -b_x * VSP33;
    VSP13 = A116 * U116 + -x_tmp * U216;
    N51MX6 = x_tmp * VP25;
    N53MX6 = A116 * VP25;
    VSP33 = A115 * VSP13 - A315 * U316;
    FDI53MX6 = A115 * U316 + A315 * VSP13;
    VSP13 = (A115 * N51MX6 - A315 * VP26) - 0.366 * FDI53MX6;
    N43MX6 = (A115 * VP26 + A315 * N51MX6) + 0.366 * VSP33;
    FDI41MX6 = A114 * VSP33 - -b_x * (-A116 * U216 + -x_tmp * U116);
    N31MX6 = A114 * VSP13 - -b_x * N53MX6;
    N33MX6 = A114 * N53MX6 + -b_x * VSP13;
    VSP13 = A116 * U126 + -x_tmp * U226;
    VSP33 = A115 * VSP13 - A315 * U326;
    FDI53MY6 = A115 * U326 + A315 * VSP13;
    VSP13 = (A115 * N53MX6 + A315 * VP16) - 0.366 * FDI53MY6;
    N43MY6 = (-A115 * VP16 + A315 * N53MX6) + 0.366 * VSP33;
    FDI41MY6 = A114 * VSP33 - -b_x * (-A116 * U226 + -x_tmp * U126);
    N31MY6 = A114 * VSP13 + -b_x * N51MX6;
    U126 = -A114 * N51MX6 + -b_x * VSP13;
    W13 = A317 * DV37_tmp;
    VSP13 = A117 * WP17 - W13;
    U226 = A117 * DV37_tmp + A317 * WP17;
    VSP33 = A116 * VSP13 + -x_tmp * DV27;
    U326 = -A116 * DV27 + -x_tmp * VSP13;
    VSP13 = A115 * VSP33 - A315 * U226;
    U116 = A115 * U226 + A315 * VSP33;
    U216 = A114 * VSP13 - -b_x * U326;
    U316 = A114 * U326 + -b_x * VSP13;
    VSP13 = -W17 * W37 + WP27;
    VSP33 = W27 * W37 + WP17;
    DV56 = U227_tmp - U117_tmp;
    W33 = A117 * VSP13 - A317 * VSP33;
    N63XY7 = A117 * VSP33 + A317 * VSP13;
    VSP13 = A116 * W33 - -x_tmp * DV56;
    WP26 = A116 * DV56 + -x_tmp * W33;
    VSP33 = A115 * VSP13 - A315 * N63XY7;
    W16 = A115 * N63XY7 + A315 * VSP13;
    W26 = A114 * VSP33 - -b_x * WP26;
    N33XY7 = A114 * WP26 + -b_x * VSP33;
    VSP13 = W17 * W27 + WP37;
    VSP33 = -(W17 * W17) + DV67_tmp;
    No73XZ7 = -W27 * W37 + WP17;
    W33 = A117 * VSP13 - A317 * VSP33;
    WP16 = A117 * VSP33 + A317 * VSP13;
    VSP13 = A116 * W33 - -x_tmp * No73XZ7;
    W36 = A116 * No73XZ7 + -x_tmp * W33;
    VSP33 = A115 * VSP13 - A315 * WP16;
    VSP25 = A115 * WP16 + A315 * VSP13;
    N31XZ7 = A114 * VSP33 - -b_x * W36;
    U225 = A114 * W36 + -b_x * VSP33;
    W33 = U117_tmp - DV67_tmp;
    VSP13 = -W17 * W27 + WP37;
    U125 = DV37_tmp + WP27;
    VSP33 = A117 * W33 - A317 * VSP13;
    N63YZ7 = A117 * VSP13 + A317 * W33;
    VSP13 = A116 * VSP33 - -x_tmp * U125;
    WP25 = A116 * U125 + -x_tmp * VSP33;
    VSP33 = A115 * VSP13 - A315 * N63YZ7;
    U115 = A115 * N63YZ7 + A315 * VSP13;
    U215 = A114 * VSP33 - -b_x * WP25;
    N33YZ7 = A114 * WP25 + -b_x * VSP33;
    VSP13 = A117 * DV57 + W13;
    W25 = -A117 * DV37_tmp + A317 * DV57;
    VSP33 = A116 * VSP13 - -x_tmp * WP37;
    W15 = A116 * WP37 + -x_tmp * VSP13;
    VSP13 = A115 * VSP33 - A315 * W25;
    N43ZZ7 = A115 * W25 + A315 * VSP33;
    N31ZZ7 = A114 * VSP13 - -b_x * W15;
    WP15 = A114 * W15 + -b_x * VSP13;
    W13 = A117 * U117 - A317 * U217;
    VSP33 = A117 * U217 + A317 * U117;
    W23 = -A317 * VSP27;
    VSP13 = W23 - 0.2503 * VSP33;
    W35 = A117 * VSP27 + 0.2503 * W13;
    W33 = A116 * W13 - -x_tmp * U317;
    N51MX7 = A116 * VSP13 - -x_tmp * VP27;
    N53MX7 = A116 * VP27 + -x_tmp * VSP13;
    VSP13 = A115 * W33 - A315 * VSP33;
    U224 = A115 * VSP33 + A315 * W33;
    VSP33 = (A115 * N51MX7 - A315 * W35) - 0.366 * U224;
    U124 = (A115 * W35 + A315 * N51MX7) + 0.366 * VSP13;
    DV54 = A114 * VSP13 - -b_x * (A116 * U317 + -x_tmp * W13);
    N31MX7 = A114 * VSP33 - -b_x * N53MX7;
    N33MX7 = A114 * N53MX7 + -b_x * VSP33;
    W13 = A117 * U127 - A317 * U227;
    FDI73MY7 = A117 * U227 + A317 * U127;
    VSP13 = -A117 * VSP27 - 0.2503 * FDI73MY7;
    WP24 = W23 + 0.2503 * W13;
    VSP33 = A116 * W13 - -x_tmp * U327;
    W33 = A116 * VSP13 + -x_tmp * VP17;
    W24 = -A116 * VP17 + -x_tmp * VSP13;
    VSP13 = A115 * VSP33 - A315 * FDI73MY7;
    W14 = A115 * FDI73MY7 + A315 * VSP33;
    VSP33 = (A115 * W33 - A315 * WP24) - 0.366 * W14;
    WP14 = (A115 * WP24 + A315 * W33) + 0.366 * VSP13;
    N41XZ5 = A114 * VSP13 - -b_x * (A116 * U327 + -x_tmp * W13);
    W34 = A114 * VSP33 - -b_x * W24;
    VSP33 = A114 * W24 + -b_x * VSP33;
    b_x = qd[0];
    if (qd[0] < 0.0) {
        b_x = -1.0;
    } else if (qd[0] > 0.0) {
        b_x = 1.0;
    } else {
        if (qd[0] == 0.0) {
            b_x = 0.0;
        }
    }

    W33 = qd[1];
    if (qd[1] < 0.0) {
        W33 = -1.0;
    } else if (qd[1] > 0.0) {
        W33 = 1.0;
    } else {
        if (qd[1] == 0.0) {
            W33 = 0.0;
        }
    }

    N51MX7 = qd[2];
    if (qd[2] < 0.0) {
        N51MX7 = -1.0;
    } else if (qd[2] > 0.0) {
        N51MX7 = 1.0;
    } else {
        if (qd[2] == 0.0) {
            N51MX7 = 0.0;
        }
    }

    FDI73MY7 = qd[3];
    if (qd[3] < 0.0) {
        FDI73MY7 = -1.0;
    } else if (qd[3] > 0.0) {
        FDI73MY7 = 1.0;
    } else {
        if (qd[3] == 0.0) {
            FDI73MY7 = 0.0;
        }
    }

    W13 = qd[4];
    if (qd[4] < 0.0) {
        W13 = -1.0;
    } else if (qd[4] > 0.0) {
        W13 = 1.0;
    } else {
        if (qd[4] == 0.0) {
            W13 = 0.0;
        }
    }

    W23 = qd[5];
    if (qd[5] < 0.0) {
        W23 = -1.0;
    } else if (qd[5] > 0.0) {
        W23 = 1.0;
    } else {
        if (qd[5] == 0.0) {
            W23 = 0.0;
        }
    }

    WP23 = qd[6];
    if (qd[6] < 0.0) {
        WP23 = -1.0;
    } else if (qd[6] > 0.0) {
        WP23 = 1.0;
    } else {
        if (qd[6] == 0.0) {
            WP23 = 0.0;
        }
    }

    b_qdd[0] = qdd[0];
    b_qdd[7] = -A112 * WP22_tmp + -x * WP12;
    b_qdd[14] = -A112 * (W22 * qd[1] + WP12) + -x * (-W12 * qd[1] + WP22);
    b_qdd[21] = -A112 * (-VSP23_tmp_tmp + VSP23_tmp) + -x * (DV22_tmp + qdd[1]);
    U224_tmp = W22 * W22;
    b_qdd[28] = -A112 * (-W12 * W22 + qdd[1]) + -x * (U224_tmp - VSP23_tmp);
    b_qdd[35] = A112 * WP22_tmp + -x * WP12_tmp;
    b_qdd[42] = 0.0;
    b_qdd[49] = 0.0;
    VSP13 = A313 * DV33_tmp;
    b_qdd[56] = -A112 * DV23 + -x * (A113 * WP13 - VSP13);
    b_qdd[63] = A112 * No33XY3 + -x * (A113 * No31XY3 - A313 * No32XY3);
    b_qdd[70] = A112 * No33XZ3 + -x * (A113 * No31XZ3 - A313 * No32XZ3);
    b_qdd[77] = A112 * No33YZ3 + -x * (A113 * No31YZ3 - A313 * No32YZ3);
    b_qdd[84] = A112 * WP33 + -x * (A113 * DV53_tmp + VSP13);
    VSP13 = -A313 * VSP23;
    b_qdd[91] = A112 * VP23 + -x * (VSP13 - 0.394 * (A113 * U213 + A313 * U113));
    b_qdd[98] = -A112 * VP13 + -x * (-A113 * VSP23 - 0.394 * (A113 * U223 + A313 *
                                                                            U123));
    b_qdd[105] = A112 * N33XX4 + -x * (A113 * N31XX4 + A313 * DV24_tmp);
    b_qdd[112] = A112 * N33XY4 + -x * (A113 * N31XY4 - A313 * No43XY4);
    b_qdd[119] = A112 * N33XZ4 + -x * (A113 * N31XZ4 - A313 * No43XZ4);
    b_qdd[126] = A112 * N33YZ4 + -x * (A113 * N31YZ4 - A313 * No43YZ4);
    b_qdd[133] = A112 * N33ZZ4 + -x * (A113 * N31ZZ4 - A313 * WP34);
    b_qdd[140] = A112 * N33MX4 + -x * ((A113 * N31MX4 - A313 * VP24) - 0.394 *
                                                                       (A113 * U314 + A313 * FDI41MX4));
    b_qdd[147] = -A112 * N31MX4 + -x * ((A113 * N33MX4 + A313 * VP14) - 0.394 *
                                                                        (A113 * U324 + A313 * FDI41MY4));
    b_qdd[154] = A112 * N33XX5 + -x * (A113 * N31XX5 - A313 * N43XX5);
    b_qdd[161] = A112 * N33XY5 + -x * (A113 * N31XY5 - A313 * N43XY5);
    b_qdd[168] = A112 * N33XZ5 + -x * (A113 * N31XZ5 - A313 * N43XZ5);
    b_qdd[175] = A112 * N33YZ5 + -x * (A113 * N31YZ5 - A313 * N43YZ5);
    b_qdd[182] = A112 * N33ZZ5 + -x * (A113 * N31ZZ5 - A313 * N43ZZ5);
    b_qdd[189] = A112 * N33MX5 + -x * ((A113 * N31MX5 - A313 * N43MX5) - 0.394 *
                                                                         (A113 * FDI53MX5 + A313 * FDI41MX5));
    b_qdd[196] = A112 * N33MY5 + -x * ((A113 * N31MY5 - A313 * N43MY5) - 0.394 *
                                                                         (A113 * FDI53MY5 + A313 * FDI41MY5));
    b_qdd[203] = A112 * N33XX6 + -x * (A113 * N31XX6 - A313 * N43XX6);
    b_qdd[210] = A112 * N33XY6 + -x * (A113 * N31XY6 - A313 * N43XY6);
    b_qdd[217] = A112 * N33XZ6 + -x * (A113 * N31XZ6 - A313 * N43XZ6);
    b_qdd[224] = A112 * N33YZ6 + -x * (A113 * N31YZ6 - A313 * N43YZ6);
    b_qdd[231] = A112 * N33ZZ6 + -x * (A113 * N31ZZ6 - A313 * N43ZZ6);
    b_qdd[238] = A112 * N33MX6 + -x * ((A113 * N31MX6 - A313 * N43MX6) - 0.394 *
                                                                         (A113 * FDI53MX6 + A313 * FDI41MX6));
    b_qdd[245] = A112 * U126 + -x * ((A113 * N31MY6 - A313 * N43MY6) - 0.394 *
                                                                       (A113 * FDI53MY6 + A313 * FDI41MY6));
    b_qdd[252] = A112 * U316 + -x * (A113 * U216 - A313 * U116);
    b_qdd[259] = A112 * N33XY7 + -x * (A113 * W26 - A313 * W16);
    b_qdd[266] = A112 * U225 + -x * (A113 * N31XZ7 - A313 * VSP25);
    b_qdd[273] = A112 * N33YZ7 + -x * (A113 * U215 - A313 * U115);
    b_qdd[280] = A112 * WP15 + -x * (A113 * N31ZZ7 - A313 * N43ZZ7);
    b_qdd[287] = A112 * N33MX7 + -x * ((A113 * N31MX7 - A313 * U124) - 0.394 *
                                                                       (A113 * U224 + A313 * DV54));
    b_qdd[294] = A112 * VSP33 + -x * ((A113 * W34 - A313 * WP14) - 0.394 * (A113 *
                                                                            W14 + A313 * N41XZ5));
    b_qdd[301] = b_x;
    b_qdd[308] = qd[0];
    b_qdd[315] = 0.0;
    b_qdd[322] = 0.0;
    b_qdd[329] = 0.0;
    b_qdd[336] = 0.0;
    b_qdd[343] = 0.0;
    b_qdd[350] = 0.0;
    b_qdd[357] = 0.0;
    b_qdd[364] = 0.0;
    b_qdd[371] = 0.0;
    b_qdd[378] = 0.0;
    b_qdd[385] = 0.0;
    b_qdd[392] = 0.0;
    b_qdd[1] = 0.0;
    b_qdd[8] = -DV22_tmp;
    b_qdd[15] = VSP23_tmp_tmp - U224_tmp;
    b_qdd[22] = -W22 * qd[1] + WP12;
    b_qdd[29] = WP22_tmp + WP22;
    b_qdd[36] = qdd[1];
    b_qdd[43] = VP22;
    b_qdd[50] = -VP12;
    b_qdd[57] = A113 * DV33_tmp + A313 * WP13;
    b_qdd[64] = A113 * No32XY3 + A313 * No31XY3;
    b_qdd[71] = A113 * No32XZ3 + A313 * No31XZ3;
    b_qdd[78] = A113 * No32YZ3 + A313 * No31YZ3;
    b_qdd[85] = -A113 * DV33_tmp + A313 * DV53_tmp;
    b_qdd[92] = A113 * VSP23 + 0.394 * (A113 * U113 - A313 * U213);
    b_qdd[99] = VSP13 + 0.394 * (A113 * U123 - A313 * U223);
    b_qdd[106] = -A113 * DV24_tmp + A313 * N31XX4;
    b_qdd[113] = A113 * No43XY4 + A313 * N31XY4;
    b_qdd[120] = A113 * No43XZ4 + A313 * N31XZ4;
    b_qdd[127] = A113 * No43YZ4 + A313 * N31YZ4;
    b_qdd[134] = A113 * WP34 + A313 * N31ZZ4;
    b_qdd[141] = (A113 * VP24 + A313 * N31MX4) + 0.394 * (A113 * FDI41MX4 - A313 *
                                                                            U314);
    b_qdd[148] = (-A113 * VP14 + A313 * N33MX4) + 0.394 * (A113 * FDI41MY4 - A313 *
                                                                             U324);
    b_qdd[155] = A113 * N43XX5 + A313 * N31XX5;
    b_qdd[162] = A113 * N43XY5 + A313 * N31XY5;
    b_qdd[169] = A113 * N43XZ5 + A313 * N31XZ5;
    b_qdd[176] = A113 * N43YZ5 + A313 * N31YZ5;
    b_qdd[183] = A113 * N43ZZ5 + A313 * N31ZZ5;
    b_qdd[190] = (A113 * N43MX5 + A313 * N31MX5) + 0.394 * (A113 * FDI41MX5 - A313
                                                                              * FDI53MX5);
    b_qdd[197] = (A113 * N43MY5 + A313 * N31MY5) + 0.394 * (A113 * FDI41MY5 - A313
                                                                              * FDI53MY5);
    b_qdd[204] = A113 * N43XX6 + A313 * N31XX6;
    b_qdd[211] = A113 * N43XY6 + A313 * N31XY6;
    b_qdd[218] = A113 * N43XZ6 + A313 * N31XZ6;
    b_qdd[225] = A113 * N43YZ6 + A313 * N31YZ6;
    b_qdd[232] = A113 * N43ZZ6 + A313 * N31ZZ6;
    b_qdd[239] = (A113 * N43MX6 + A313 * N31MX6) + 0.394 * (A113 * FDI41MX6 - A313
                                                                              * FDI53MX6);
    b_qdd[246] = (A113 * N43MY6 + A313 * N31MY6) + 0.394 * (A113 * FDI41MY6 - A313
                                                                              * FDI53MY6);
    b_qdd[253] = A113 * U116 + A313 * U216;
    b_qdd[260] = A113 * W16 + A313 * W26;
    b_qdd[267] = A113 * VSP25 + A313 * N31XZ7;
    b_qdd[274] = A113 * U115 + A313 * U215;
    b_qdd[281] = A113 * N43ZZ7 + A313 * N31ZZ7;
    b_qdd[288] = (A113 * U124 + A313 * N31MX7) + 0.394 * (A113 * DV54 - A313 *
                                                                        U224);
    b_qdd[295] = (A113 * WP14 + A313 * W34) + 0.394 * (A113 * N41XZ5 - A313 * W14);
    b_qdd[302] = 0.0;
    b_qdd[309] = 0.0;
    b_qdd[316] = W33;
    b_qdd[323] = qd[1];
    b_qdd[330] = 0.0;
    b_qdd[337] = 0.0;
    b_qdd[344] = 0.0;
    b_qdd[351] = 0.0;
    b_qdd[358] = 0.0;
    b_qdd[365] = 0.0;
    b_qdd[372] = 0.0;
    b_qdd[379] = 0.0;
    b_qdd[386] = 0.0;
    b_qdd[393] = 0.0;
    b_qdd[2] = 0.0;
    b_qdd[9] = 0.0;
    b_qdd[16] = 0.0;
    b_qdd[23] = 0.0;
    b_qdd[30] = 0.0;
    b_qdd[37] = 0.0;
    b_qdd[44] = 0.0;
    b_qdd[51] = 0.0;
    b_qdd[58] = -DV23;
    b_qdd[65] = No33XY3;
    b_qdd[72] = No33XZ3;
    b_qdd[79] = No33YZ3;
    b_qdd[86] = WP33;
    b_qdd[93] = VP23;
    b_qdd[100] = -VP13;
    b_qdd[107] = N33XX4;
    b_qdd[114] = N33XY4;
    b_qdd[121] = N33XZ4;
    b_qdd[128] = N33YZ4;
    b_qdd[135] = N33ZZ4;
    b_qdd[142] = N33MX4;
    b_qdd[149] = -N31MX4;
    b_qdd[156] = N33XX5;
    b_qdd[163] = N33XY5;
    b_qdd[170] = N33XZ5;
    b_qdd[177] = N33YZ5;
    b_qdd[184] = N33ZZ5;
    b_qdd[191] = N33MX5;
    b_qdd[198] = N33MY5;
    b_qdd[205] = N33XX6;
    b_qdd[212] = N33XY6;
    b_qdd[219] = N33XZ6;
    b_qdd[226] = N33YZ6;
    b_qdd[233] = N33ZZ6;
    b_qdd[240] = N33MX6;
    b_qdd[247] = U126;
    b_qdd[254] = U316;
    b_qdd[261] = N33XY7;
    b_qdd[268] = U225;
    b_qdd[275] = N33YZ7;
    b_qdd[282] = WP15;
    b_qdd[289] = N33MX7;
    b_qdd[296] = VSP33;
    b_qdd[303] = 0.0;
    b_qdd[310] = 0.0;
    b_qdd[317] = 0.0;
    b_qdd[324] = 0.0;
    b_qdd[331] = N51MX7;
    b_qdd[338] = qd[2];
    b_qdd[345] = 0.0;
    b_qdd[352] = 0.0;
    b_qdd[359] = 0.0;
    b_qdd[366] = 0.0;
    b_qdd[373] = 0.0;
    b_qdd[380] = 0.0;
    b_qdd[387] = 0.0;
    b_qdd[394] = 0.0;
    b_qdd[3] = 0.0;
    b_qdd[10] = 0.0;
    b_qdd[17] = 0.0;
    b_qdd[24] = 0.0;
    b_qdd[31] = 0.0;
    b_qdd[38] = 0.0;
    b_qdd[45] = 0.0;
    b_qdd[52] = 0.0;
    b_qdd[59] = 0.0;
    b_qdd[66] = 0.0;
    b_qdd[73] = 0.0;
    b_qdd[80] = 0.0;
    b_qdd[87] = 0.0;
    b_qdd[94] = 0.0;
    b_qdd[101] = 0.0;
    b_qdd[108] = -DV24_tmp;
    b_qdd[115] = No43XY4;
    b_qdd[122] = No43XZ4;
    b_qdd[129] = No43YZ4;
    b_qdd[136] = WP34;
    b_qdd[143] = VP24;
    b_qdd[150] = -VP14;
    b_qdd[157] = N43XX5;
    b_qdd[164] = N43XY5;
    b_qdd[171] = N43XZ5;
    b_qdd[178] = N43YZ5;
    b_qdd[185] = N43ZZ5;
    b_qdd[192] = N43MX5;
    b_qdd[199] = N43MY5;
    b_qdd[206] = N43XX6;
    b_qdd[213] = N43XY6;
    b_qdd[220] = N43XZ6;
    b_qdd[227] = N43YZ6;
    b_qdd[234] = N43ZZ6;
    b_qdd[241] = N43MX6;
    b_qdd[248] = N43MY6;
    b_qdd[255] = U116;
    b_qdd[262] = W16;
    b_qdd[269] = VSP25;
    b_qdd[276] = U115;
    b_qdd[283] = N43ZZ7;
    b_qdd[290] = U124;
    b_qdd[297] = WP14;
    b_qdd[304] = 0.0;
    b_qdd[311] = 0.0;
    b_qdd[318] = 0.0;
    b_qdd[325] = 0.0;
    b_qdd[332] = 0.0;
    b_qdd[339] = 0.0;
    b_qdd[346] = FDI73MY7;
    b_qdd[353] = qd[3];
    b_qdd[360] = 0.0;
    b_qdd[367] = 0.0;
    b_qdd[374] = 0.0;
    b_qdd[381] = 0.0;
    b_qdd[388] = 0.0;
    b_qdd[395] = 0.0;
    b_qdd[4] = 0.0;
    b_qdd[11] = 0.0;
    b_qdd[18] = 0.0;
    b_qdd[25] = 0.0;
    b_qdd[32] = 0.0;
    b_qdd[39] = 0.0;
    b_qdd[46] = 0.0;
    b_qdd[53] = 0.0;
    b_qdd[60] = 0.0;
    b_qdd[67] = 0.0;
    b_qdd[74] = 0.0;
    b_qdd[81] = 0.0;
    b_qdd[88] = 0.0;
    b_qdd[95] = 0.0;
    b_qdd[102] = 0.0;
    b_qdd[109] = 0.0;
    b_qdd[116] = 0.0;
    b_qdd[123] = 0.0;
    b_qdd[130] = 0.0;
    b_qdd[137] = 0.0;
    b_qdd[144] = 0.0;
    b_qdd[151] = 0.0;
    b_qdd[158] = -DV25;
    b_qdd[165] = No53XY5;
    b_qdd[172] = No53XZ5;
    b_qdd[179] = No53YZ5;
    b_qdd[186] = WP35;
    b_qdd[193] = VP25;
    b_qdd[200] = -VP15;
    b_qdd[207] = N53XX6;
    b_qdd[214] = N53XY6;
    b_qdd[221] = N53XZ6;
    b_qdd[228] = N53YZ6;
    b_qdd[235] = N53ZZ6;
    b_qdd[242] = N53MX6;
    b_qdd[249] = -N51MX6;
    b_qdd[256] = U326;
    b_qdd[263] = WP26;
    b_qdd[270] = W36;
    b_qdd[277] = WP25;
    b_qdd[284] = W15;
    b_qdd[291] = N53MX7;
    b_qdd[298] = W24;
    b_qdd[305] = 0.0;
    b_qdd[312] = 0.0;
    b_qdd[319] = 0.0;
    b_qdd[326] = 0.0;
    b_qdd[333] = 0.0;
    b_qdd[340] = 0.0;
    b_qdd[347] = 0.0;
    b_qdd[354] = 0.0;
    b_qdd[361] = W13;
    b_qdd[368] = qd[4];
    b_qdd[375] = 0.0;
    b_qdd[382] = 0.0;
    b_qdd[389] = 0.0;
    b_qdd[396] = 0.0;
    b_qdd[5] = 0.0;
    b_qdd[12] = 0.0;
    b_qdd[19] = 0.0;
    b_qdd[26] = 0.0;
    b_qdd[33] = 0.0;
    b_qdd[40] = 0.0;
    b_qdd[47] = 0.0;
    b_qdd[54] = 0.0;
    b_qdd[61] = 0.0;
    b_qdd[68] = 0.0;
    b_qdd[75] = 0.0;
    b_qdd[82] = 0.0;
    b_qdd[89] = 0.0;
    b_qdd[96] = 0.0;
    b_qdd[103] = 0.0;
    b_qdd[110] = 0.0;
    b_qdd[117] = 0.0;
    b_qdd[124] = 0.0;
    b_qdd[131] = 0.0;
    b_qdd[138] = 0.0;
    b_qdd[145] = 0.0;
    b_qdd[152] = 0.0;
    b_qdd[159] = 0.0;
    b_qdd[166] = 0.0;
    b_qdd[173] = 0.0;
    b_qdd[180] = 0.0;
    b_qdd[187] = 0.0;
    b_qdd[194] = 0.0;
    b_qdd[201] = 0.0;
    b_qdd[208] = -DV26;
    b_qdd[215] = No63XY6;
    b_qdd[222] = No63XZ6;
    b_qdd[229] = No63YZ6;
    b_qdd[236] = WP36;
    b_qdd[243] = VP26;
    b_qdd[250] = -VP16;
    b_qdd[257] = U226;
    b_qdd[264] = N63XY7;
    b_qdd[271] = WP16;
    b_qdd[278] = N63YZ7;
    b_qdd[285] = W25;
    b_qdd[292] = W35;
    b_qdd[299] = WP24;
    b_qdd[306] = 0.0;
    b_qdd[313] = 0.0;
    b_qdd[320] = 0.0;
    b_qdd[327] = 0.0;
    b_qdd[334] = 0.0;
    b_qdd[341] = 0.0;
    b_qdd[348] = 0.0;
    b_qdd[355] = 0.0;
    b_qdd[362] = 0.0;
    b_qdd[369] = 0.0;
    b_qdd[376] = W23;
    b_qdd[383] = qd[5];
    b_qdd[390] = 0.0;
    b_qdd[397] = 0.0;
    b_qdd[6] = 0.0;
    b_qdd[13] = 0.0;
    b_qdd[20] = 0.0;
    b_qdd[27] = 0.0;
    b_qdd[34] = 0.0;
    b_qdd[41] = 0.0;
    b_qdd[48] = 0.0;
    b_qdd[55] = 0.0;
    b_qdd[62] = 0.0;
    b_qdd[69] = 0.0;
    b_qdd[76] = 0.0;
    b_qdd[83] = 0.0;
    b_qdd[90] = 0.0;
    b_qdd[97] = 0.0;
    b_qdd[104] = 0.0;
    b_qdd[111] = 0.0;
    b_qdd[118] = 0.0;
    b_qdd[125] = 0.0;
    b_qdd[132] = 0.0;
    b_qdd[139] = 0.0;
    b_qdd[146] = 0.0;
    b_qdd[153] = 0.0;
    b_qdd[160] = 0.0;
    b_qdd[167] = 0.0;
    b_qdd[174] = 0.0;
    b_qdd[181] = 0.0;
    b_qdd[188] = 0.0;
    b_qdd[195] = 0.0;
    b_qdd[202] = 0.0;
    b_qdd[209] = 0.0;
    b_qdd[216] = 0.0;
    b_qdd[223] = 0.0;
    b_qdd[230] = 0.0;
    b_qdd[237] = 0.0;
    b_qdd[244] = 0.0;
    b_qdd[251] = 0.0;
    b_qdd[258] = -DV27;
    b_qdd[265] = DV56;
    b_qdd[272] = No73XZ7;
    b_qdd[279] = U125;
    b_qdd[286] = WP37;
    b_qdd[293] = VP27;
    b_qdd[300] = -VP17;
    b_qdd[307] = 0.0;
    b_qdd[314] = 0.0;
    b_qdd[321] = 0.0;
    b_qdd[328] = 0.0;
    b_qdd[335] = 0.0;
    b_qdd[342] = 0.0;
    b_qdd[349] = 0.0;
    b_qdd[356] = 0.0;
    b_qdd[363] = 0.0;
    b_qdd[370] = 0.0;
    b_qdd[377] = 0.0;
    b_qdd[384] = 0.0;
    b_qdd[391] = WP23;
    b_qdd[398] = qd[6];
    for (int i = 0; i < 7; i++) {
        VSP13 = 0.0;
        for (int i1 = 0; i1 < 57; i1++) {
            VSP13 += b_qdd[i + 7 * i1] * b[i1];
        }

        torque[i] = VSP13;
    }
}

//
// File trailer for torque_from_regression_matrix.cpp
//
// [EOF]
//
