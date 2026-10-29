#include "csv_reader.hpp"      
#include "kalman_filter.hpp"    

#include <cmath>            
#include <fstream>            
#include <iomanip>            
#include <iostream>          
#include <string>                
#include <vector>  
#include <algorithm>    

//  用于确定R的函数
// double estimateR(const std::vector<Frame>& f, int c, bool isYaw) {
//     auto val = [c](const Frame& s) {
//         return c == 0 ? s.x : c == 1 ? s.y : c == 2 ? s.z : s.yaw;
//     };

//     std::vector<double> d;
//     for (size_t i = 1; i < f.size(); ++i) {
//         double dv = val(f[i]) - val(f[i - 1]);
//         if (isYaw) dv = std::remainder(dv, M_PI);   // 归一到 [-π/2, π/2)
//         d.push_back(std::fabs(dv));
//     }
//     std::nth_element(d.begin(), d.begin() + d.size() / 2, d.end());
//     const double mad = d[d.size() / 2];           // 中位数
//     const double sigma = 1.4826 * mad;
//     return sigma * sigma / 2.0;
// }



int main(int argc, char** argv) {

    const std::string path = (argc > 1) ? argv[1] : "armor_data.csv";       //加载armor_data.csv
    std::vector<Frame> frames = load(path);

    std::ofstream out("result.csv");        //加载result.csv
    if (!out) {
    std::cerr << "无法写入 result.csv\n"; 
    return 1;
    }

    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(4, 4); //R(4x4)
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(8, 8); //Q(8x8)
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(8, 8); //P(8x8)


    // 调用estimateR()确定R
    // std::printf("R = %.6f %.6f %.6f %.6f\n",
    //             estimateR(frames, 0, false), estimateR(frames, 1, false),
    //             estimateR(frames, 2, false), estimateR(frames, 3, true));
    //R_x = 0.005 R_y = 0.0003 R_z = 0.0001 R_yaw = 0.0003

    // 各轴过程噪声密度：分别对应 x / y / z / yaw
    // 调参目标：每轴的「分轴 NIS」都往 1 靠
    double q_x   = 8.0e-4;
    double q_y   = 3.0e-3;
    double q_z   = 3.0e-6;
    double q_yaw = 5.0e-6;

    R.diagonal() << 0.006,    // px
                    0.0003,    // py
                    0.0004,    // pz
                    0.0035;    // yaw
    Q.diagonal() << q_x, q_y, q_z, q_yaw,   // px py pz yaw
                    q_x, q_y, q_z, q_yaw;   // vx vy vz vyaw
    P.diagonal() << 0.005, 0.0003, 0.0001, 0.0003,
                    0.25, 0.25, 0.25, 0.25;

    int k = 1;
    auto it = frames.begin();
    Frame frame = *it;
    Eigen::VectorXd x(8);
    Eigen::VectorXd z(4);
    x << frame.x, frame.y, frame.z, frame.yaw, 0, 0, 0, 0;
    tools::KalmanFilter KmF(x, P, Q, R);

    // 表头
    out << "k,x,y,z,yaw,fx,fy,fz,fyaw,vx,vy,vz,vyaw\n";

    // 第一帧：作初值，滤波结果就是初始状态
    out << k << ','
        << frame.x << ',' << frame.y << ',' << frame.z << ',' << frame.yaw << ','
        << KmF.x(0) << ',' << KmF.x(1) << ',' << KmF.x(2) << ',' << KmF.x(3) << ','
        << KmF.x(4) << ',' << KmF.x(5) << ',' << KmF.x(6) << ',' << KmF.x(7) << '\n';

    ++it;
    ++k;

    // 统计 NIS：全局理论值 4，分轴理论值 1
    // 分轴 NIS_i = innov_i² / S_ii ，其中 S_ii = P_ii(预测后) + R_ii
    double nis_sum = 0.0;
    int nis_count = 0;
    double nis_axis[4] = {0.0, 0.0, 0.0, 0.0};

    for(; it != frames.end(); ++it, ++k){
        frame = *it;
        z << frame.x, frame.y, frame.z, frame.yaw;
        KmF.predict(1.0);
        const Eigen::VectorXd P_prior = KmF.P.diagonal().head(4);  // 预测后的 P⁻
        KmF.update(z);

        nis_sum += KmF.lastNis;
        ++nis_count;
        for (int i = 0; i < 4; ++i) {
            nis_axis[i] += KmF.lastInnovation(i) * KmF.lastInnovation(i) /
                           (P_prior(i) + R(i, i));
        }

    out << k << ','
        << z(0) << ',' << z(1) << ',' << z(2) << ',' << z(3) << ','
        << KmF.x(0) << ',' << KmF.x(1) << ',' << KmF.x(2) << ',' << KmF.x(3) << ','
        << KmF.x(4) << ',' << KmF.x(5) << ',' << KmF.x(6) << ',' << KmF.x(7) << '\n';
    }

    out.close();

    if (nis_count > 0) {
        std::cout << "平均 NIS = " << nis_sum / nis_count
                  << "   (帧数 " << nis_count << "，理论值 4)\n";
        const char* axis_name[4] = {"  x", "  y", "  z", "yaw"};
        std::cout << "分轴 NIS（理论值 1）：\n";
        for (int i = 0; i < 4; ++i) {
            std::cout << "  " << axis_name[i] << " = "
                      << nis_axis[i] / nis_count
                      << (nis_axis[i] / nis_count > 1.5 ? "   ← q 调大"
                          : nis_axis[i] / nis_count < 0.67 ? "   ← q 调小"
                          : "   ok")
                      << "\n";
        }
    }
}
