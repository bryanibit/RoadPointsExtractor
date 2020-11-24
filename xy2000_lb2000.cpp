#include "xy2000_lb2000.h"

void XY2000::LB2000_XY2000(double L, double B, int &globalX, int &globalY) {
    double L0, delta_L;
    double gmmac;
    unsigned int unum;

    double sx, N, ita2, m, t;
    double A0, B0, C0, D0, E0;
    double t2, t4, ita4;
    double a2000, b2000, e2, e1;
    double gx, gy;

    unum = ((int)floor(L * 180.0 / M_PI)) / 6 + 1; // 计算区号

    L0 = (double)unum * 6.0 - 3.0;// 计算中心经度
    delta_L = L - L0 * M_PI / 180.0;

    a2000 = 6378137.e0;
    b2000 = 6356752.31e0;

    e2 = (a2000 * a2000 - b2000 * b2000) / (a2000 * a2000);
    e1 = sqrt((a2000 * a2000 - b2000 * b2000) / (b2000 * b2000));

    L = L * 180.0 / M_PI;
    ita2 = e1 * e1 * cos(B) * cos(B);
    t = tan(B);
    t2 = t * t;
    t4 = t2 * t2;
    ita4 = ita2 * ita2;
    N = a2000 / (sqrt(1 - e2)) / sqrt(1 + ita2);
    m = cos(B) * (M_PI) / 180.0 * (L - L0);

    A0 = 6367449.1436;
    B0 = 32009.8248;
    C0 = 133.998;
    D0 = 0.6975211868;
    E0 = 0.003914074778;

    sx = A0 * B - (B0 + C0 * sin(B) * sin(B) + D0 * sin(B) * sin(B) * sin(B) * sin(B) + E0 * sin(B) * sin(B) * sin(B) * sin(B) * sin(B) * sin(B)) * sin(B) * cos(B);
    gx = sx + N * t * (0.5e0 * pow(m, 2.e0) + 1.e0 / 24.e0 * (5.e0 - t2 + 9.e0 * ita2 + 4.e0 * ita4) * pow(m, 4.0) + (61.e0 - 58.e0 * t2 + t4) * pow(m, 6.0) / 720.e0);
    gy = N * (m + (1.0 - t2 + ita2) * pow(m, 3.0) / 6.e0 + (5.0 - 18.0 * t2 + t4 + 14.0 * ita2 - 58.0 * ita2 * t2) * pow(m, 5.0) / 120.0);
    gy = gy + 500000.0;

    //    gmmac=delta_L*sin(B);

    //    xyH2000[2]= gmmac;
    //    xyH2000[3]=(double)unum;
    globalX = round(gx * 10.0);
    globalY = round(gy * 10.0);

}

void XY2000::XY2000_LB2000(double y0, double x0, double &L, double &B, int z_zone) {
    double a2000, b2000;
    double ep2, lg2, V, c, N, t, V2, B0, Bf, y1, aa, unum, L0, gmmac;
    double delta_L;

    a2000 = 6378137.e0;
    b2000 = 6356752.31e0;

    y0 = y0 - 500000;
    unum = z_zone /*LB2000[3]*/;

    B0 = x0 / 6367449.1437023;
    Bf = B0 + 0.00251882708512 * sin(2.0 * B0) + 0.00000370095115 * sin(4.0 * B0) + 0.0000000053764 * sin(6.0 * B0);

    ep2 = (a2000 * a2000 - b2000 * b2000) / (b2000 * b2000);
    lg2 = ep2 * cos(Bf) * cos(Bf);
    V = sqrt(1 + lg2);
    c = a2000 * a2000 / b2000;
    N = c / V;
    //	M = c/(1+lg2)/V;
    t = tan(Bf);
    y1 = y0 / N;
    V2 = 1 + lg2;

    aa = 1 / cos(Bf);
    L0 = unum * 6 - 3;

    B = Bf - 0.5 * V * V * t * y1 * y1 + 1 / 24.0 * (5.0 + 3.0 * t + lg2 - 9.0 * lg2 * t * t) * V2 * t * y1 * y1 * y1 * y1
        - 1 / 720.0 * (61 + 90 * t * t + 45 * t * t * t * t) * V2 * t * y1 * y1 * y1 * y1 * y1 * y1;

    L = aa * y1 - 1.0 / 6 * (1 + 2.0 * t * t + lg2) * aa * y1 * y1 * y1 + 1.0 / 120.0 * (5 + 28.0 * t * t + 24.0 * t * t * t * t
                                                                                         + 6.0 * lg2 + 8.0 * lg2 * t * t) * aa * y1 * y1 * y1 * y1 * y1;

    L = L0 / 180.0 * M_PI + L;

    delta_L = L - L0 / 180.0 * M_PI;
    B = B / M_PI * 180.0;
    L = L / M_PI * 180.0;
}

void Mercator::XY2LB(double gx, double gy, double &L, double &B) {
    double x = gx;
    double y = gy;

    double MAXIMUM = 20037508.3427892;
    x = x / MAXIMUM * 180;
    y = y / MAXIMUM * 180;

    B = 180.0 / M_PI * (2 * atan(exp(y * M_PI / 180.0)) - M_PI_2);
    L = x;
}

void Mercator::LB2XY(double L, double B, int &x, int &y) {
    L = L * 180.0 / M_PI;
    B = B * 180.0 / M_PI;
    double x_ = (L * 20037508.3427892 / 180.0);
    double y_ = log(tan((90.0 + B) * M_PI / 360.0)) / M_PI * 180.0;
    y_ = y_ * 20037508.3427892 / 180.0;
    x = round(x_ * 10);
    y = round(y_ * 10);
}
