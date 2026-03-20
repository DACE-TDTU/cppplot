/**
 * @file mobile_robot_lqr_case_study.cpp
 * @brief Educational Case Study: Differential-Drive Mobile Robot
 * Trajectory tracking via Kinematic Decomposition, LQR, and Native Plotting
 * * Build:
 * g++ -std=c++17 -O2 -I../../cppplot/include mobile_robot_lqr_case_study.cpp -o robot_lqr
 * ./robot_lqr
 */

#include "cppplot/core/matrix.hpp"
#include "cppplot/control/controller_design.hpp"
#include "cppplot/control/state_space.hpp"

// Giả định thư viện cppplot có module visualization mô phỏng cú pháp matplotlib
#include "cppplot/cppplot.hpp" 

#include <iostream>
#include <iomanip>
#include <vector>
#include <fstream>

using namespace cppplot;
using namespace cppplot::control;

int main() {
    std::cout << "========================================================\n";
    std::cout << " Differential-Drive Mobile Robot: LQR Trajectory Tracking \n";
    std::cout << "========================================================\n\n";

    // 1. Khởi tạo Thông số Hệ thống (Kinematic Decomposition)
    double v_r = 1.0; // Vận tốc thẳng tham chiếu (m/s)

    // Mô hình động học sai lệch ngang: dx/dt = A*x + B*u
    Matrix A = {
        {0.0, v_r},
        {0.0, 0.0}
    };
    
    Matrix B = {
        {0.0},
        {1.0}
    };

    // 2. Thiết lập Ma trận Trọng số LQR
    Matrix Q = Matrix::diag({10.0, 1.0}); // Ưu tiên triệt tiêu sai lệch ngang e_lat
    Matrix R = {{0.1}};                   // Phạt tín hiệu lái u_steer

    std::cout << "[1] Solving Continuous Algebraic Riccati Equation (CARE)..." << std::endl;
    
    // 3. Giải phương trình CARE bằng lõi Hamiltonian-Schur
    Matrix P = care(A, B, Q, R);
    
    // 4. Tính toán Ma trận Độ lợi Phản hồi Tối ưu K = R^-1 * B^T * P
    Matrix K = R.inv() * B.T() * P;
    
    std::cout << "\n[2] Optimal Feedback Gain (K):\n    [ ";
    for(size_t j=0; j<K.cols; ++j) std::cout << K(0,j) << " ";
    std::cout << "]\n\n";

    // 5. Khảo sát Mô phỏng Đáp ứng Thời gian (Closed-loop Time Response)
    std::cout << "[3] Simulating Closed-Loop Time Response..." << std::endl;
    
    Matrix A_cl = A - (B * K);
    Matrix x0 = {{0.5}, {0.1}}; // Lệch 0.5m, góc lệch 0.1 rad
    
    Matrix B_cl(2, 1, 0.0);
    Matrix C_cl = Matrix::eye(2); 
    Matrix D_cl(2, 1, 0.0);
    StateSpace sys_cl(A_cl, B_cl, C_cl, D_cl);

    double t_end = 5.0;
    double dt = 0.01;
    std::vector<double> T;
    for (double t = 0; t <= t_end; t += dt) T.push_back(t);


    std::vector<std::vector<double>> U(T.size(), std::vector<double>(1, 0.0));

    auto response = lsim(sys_cl, U, T, x0);
    auto response1 = response.y;
    auto response2 = response.t;


    // 6. Trích xuất dữ liệu và Xuất CSV
    std::string csv_filename = "robot_lateral_error_response.csv";
    std::ofstream f(csv_filename);
    f << "time,e_lat,e_theta,u_steer\n";
    
    std::vector<double> e_lat, e_theta, u_steer;
    
    for (size_t i = 0; i < T.size(); ++i) {
        e_lat.push_back(response1[i]);
        e_theta.push_back(response2[i]);
        u_steer.push_back(-(K(0,0)*e_lat[i] + K(0,1)*e_theta[i])); // Tính tín hiệu điều khiển
        
        f << T[i] << "," << e_lat[i] << "," << e_theta[i] << "," << u_steer[i] << "\n";
    }
    
    std::cout << "    Simulation complete. Data exported to: " << csv_filename << "\n";
    
    // 7. VẼ ĐỒ THỊ TRỰC TIẾP TRONG C++ (NATIVE PLOTTING)
    std::cout << "[4] Generating Plots..." << std::endl;
    
    figure();
    
    // Vẽ sai lệch quỹ đạo (State Errors)
    plot(T, e_lat, "b-", {{"label", "Lateral Error (m)"}});
 

    plot(T, e_theta, "r--",{{"label",  "Heading Error (rad)"}});
    title("Mobile Robot LQR Trajectory Tracking");
    xlabel("Time (s)");
    ylabel("State Error");
    legend();
    grid(true);
    
    // Cửa sổ đồ thị sẽ bật lên và chương trình tạm dừng cho đến khi người dùng đóng cửa sổ
    savefig("mobile_robot_lqr_cs.svg"); 

    std::cout << "========================================================\n";

    return 0;
}