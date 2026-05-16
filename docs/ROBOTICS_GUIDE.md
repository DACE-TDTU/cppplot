# Huong Dan Su Dung Module Robotics

Module `cppplot::robotics` cung cap cac tien ich robotics co ban tren nen CppPlot. Phien ban hien tai tap trung vao mobile robot 2D, dac biet la differential-drive robot: dong hoc banh xe, mo phong unicycle, path following bang pure pursuit, va truc quan hoa quy dao bang SVG.

## 1. Cai Dat Va Include

Thu vien van giu kieu header-only nhu CppPlot. Co hai cach include:

```cpp
#include <cppplot/robotics/robotics.hpp>
```

hoac neu muon dung chung plotting API:

```cpp
#include <cppplot/cppplot.hpp>
```

Namespace chinh:

```cpp
using namespace cppplot;
using namespace cppplot::robotics;
```

## 2. Cac Kieu Du Lieu Co Ban

Module dinh nghia cac struct nho, ro nghia:

```cpp
Point2D p{1.0, 2.0};
Pose2D pose{0.0, 0.0, 0.0};       // x, y, theta
Twist2D cmd{0.5, 1.0};            // v, omega
WheelVelocity wheels{10.0, 12.0}; // omega_left, omega_right
```

Y nghia:

- `Point2D`: diem 2D.
- `Pose2D`: vi tri va huong robot trong mat phang.
- `Twist2D`: van toc unicycle, gom van toc thang `v` va van toc goc `omega`.
- `WheelVelocity`: van toc goc banh trai/phai.
- `TrackingSample`: mau du lieu mo phong gom thoi gian, pose, command, distance error, heading error.

Tien ich goc:

```cpp
double a = normalize_angle(4.0); // dua ve [-pi, pi]
double d = distance(Point2D{0, 0}, Point2D{3, 4}); // 5
```

## 3. Differential-Drive Kinematics

Header:

```cpp
#include <cppplot/robotics/mobile_base.hpp>
```

Tao robot:

```cpp
DifferentialDriveParams params;
params.wheel_radius = 0.05; // m
params.track_width = 0.30;  // m

DifferentialDrive robot(params);
```

Tinh van toc robot tu van toc banh:

```cpp
WheelVelocity wheels{10.0, 12.0}; // rad/s
Twist2D twist = robot.forward(wheels);

std::cout << "v = " << twist.v << "\n";
std::cout << "omega = " << twist.omega << "\n";
```

Tinh van toc banh tu lenh `v, omega`:

```cpp
Twist2D command{0.5, 1.0};
WheelVelocity wheel_cmd = robot.inverse(command);
```

Cong thuc su dung:

```text
v     = r * (omega_R + omega_L) / 2
omega = r * (omega_R - omega_L) / L

omega_L = (v - omega * L / 2) / r
omega_R = (v + omega * L / 2) / r
```

Trong do:

- `r`: ban kinh banh xe.
- `L`: khoang cach hai banh.

## 4. Mo Phong Pose

Header:

```cpp
#include <cppplot/robotics/simulation.hpp>
```

Tich phan chinh xac cho unicycle:

```cpp
Pose2D pose{0.0, 0.0, 0.0};
Twist2D cmd{0.5, 0.2};
double dt = 0.01;

pose = integrate_exact(pose, cmd, dt);
```

Neu chi can Euler:

```cpp
pose = integrate_euler(pose, cmd, dt);
```

Mo phong voi policy tuy y:

```cpp
auto poses = simulate_unicycle(
    Pose2D{0.0, 0.0, 0.0},
    0.02,
    500,
    [](const Pose2D& pose, double t) {
      return Twist2D{0.5, 0.3};
    });
```

## 5. Tao Path 2D

Header:

```cpp
#include <cppplot/robotics/trajectory.hpp>
```

Tao duong thang:

```cpp
Path2D path = make_line_path(Point2D{0.0, 0.0},
                             Point2D{5.0, 2.0},
                             80);
```

Tao duong tron:

```cpp
Path2D circle = make_circle_path(Point2D{0.0, 0.0}, 2.0, 120);
```

Them waypoint thu cong:

```cpp
Path2D path;
path.push_back({0.0, 0.0});
path.push_back({1.0, 0.5});
path.push_back({2.0, 0.0});
```

Tinh chieu dai path:

```cpp
double L = path.length();
```

Tim diem gan nhat:

```cpp
size_t idx = path.nearest_index(Point2D{0.9, 0.2});
Point2D nearest = path.points()[idx];
```

Tim diem lookahead:

```cpp
Point2D target = path.lookahead_point(Point2D{0.0, 0.0}, 0.5);
```

## 6. Pure Pursuit Controller

Header:

```cpp
#include <cppplot/robotics/controllers.hpp>
```

Tao controller:

```cpp
Path2D path = make_line_path({0.0, 0.0}, {5.0, 2.0}, 80);

PurePursuitConfig cfg;
cfg.lookahead = 0.45;
cfg.speed = 0.55;
cfg.max_speed = 1.0;
cfg.max_omega = 2.0;

PurePursuitController controller(path, cfg);
```

Dung trong loop:

```cpp
Pose2D pose{0.0, -0.6, 0.15};
double dt = 0.02;

for (int k = 0; k < 500; ++k) {
  Twist2D cmd = controller.compute(pose);
  pose = integrate_exact(pose, cmd, dt);
}
```

Pure pursuit se:

- tim diem lookahead tren path;
- tinh goc tu robot den diem do;
- tinh curvature;
- tra ve lenh `Twist2D {v, omega}`.

Tham so quan trong:

- `lookahead`: nho hon thi bam duong gat hon, nhung de dao dong.
- `speed`: van toc cruise.
- `max_omega`: gioi han toc do quay.

## 7. PID Co Ban

`PID` hien la controller scalar don gian, dung duoc cho vong dieu khien van toc banh:

```cpp
PID pid(5.0, 1.0, 0.05, -12.0, 12.0);

double voltage = pid.compute(omega_ref, omega_meas, dt);
```

Reset trang thai:

```cpp
pid.reset();
```

## 8. Visualization

Header:

```cpp
#include <cppplot/robotics/visualization.hpp>
```

Ve path:

```cpp
plot_path(path, "reference path");
```

Ve trajectory:

```cpp
std::vector<Pose2D> poses = ...;
plot_trajectory(poses, "robot trajectory");
```

Ve robot tai pose cuoi:

```cpp
draw_robot(poses.back(), params);
```

Ve tracking error:

```cpp
std::vector<TrackingSample> samples = ...;
plot_tracking_errors(samples);
```

Vi du figure day du:

```cpp
figure(1000, 420);

subplot(1, 2, 1);
plot_path(path, "reference");
plot_trajectory(poses, "actual");
draw_robot(poses.back(), params);
title("Robot Trajectory");
xlabel("x (m)");
ylabel("y (m)");
grid(true);
legend(true);

subplot(1, 2, 2);
plot_tracking_errors(samples);
title("Tracking Errors");
xlabel("time (s)");
ylabel("error");
grid(true);
legend(true);

savefig("robotics_demo.svg");
```

## 9. Vi Du Hoan Chinh: Differential Drive + Pure Pursuit

File demo da co san:

```text
examples/robotics_diffdrive_demo.cpp
```

No thuc hien:

1. tao differential-drive robot;
2. tao line path;
3. chay pure pursuit controller;
4. mo phong robot;
5. ve path, trajectory, final pose, va error plots;
6. xuat `robotics_diffdrive_demo.svg`.

Bien dich truc tiep:

```powershell
g++ -std=c++17 -D_USE_MATH_DEFINES -Iinclude examples\robotics_diffdrive_demo.cpp -o build_ci\robotics_diffdrive_demo.exe
.\build_ci\robotics_diffdrive_demo.exe
```

Neu build bang CMake, target da duoc dang ky trong `examples/CMakeLists.txt`:

```powershell
cmake --build build --target robotics_diffdrive_demo
```

## 10. Test

Test dau tien:

```text
tests/test_robotics_mobile_base.cpp
```

No kiem tra:

- forward/inverse kinematics;
- chuyen dong thang;
- quay tai cho;
- path length, nearest point, lookahead point.

Bien dich va chay truc tiep:

```powershell
g++ -std=c++17 -D_USE_MATH_DEFINES -Iinclude tests\test_robotics_mobile_base.cpp -o build_ci\test_robotics_mobile_base.exe
.\build_ci\test_robotics_mobile_base.exe
```

Ket qua mong doi:

```text
test_robotics_mobile_base PASSED
```

Neu dung CTest:

```powershell
cmake --build build --target test_robotics_mobile_base
ctest --test-dir build -R test_robotics_mobile_base --output-on-failure
```

## 11. Mau Chuong Trinh Rieng

Template toi thieu:

```cpp
#include <cppplot/robotics/robotics.hpp>

using namespace cppplot;
using namespace cppplot::robotics;

int main() {
  DifferentialDriveParams params(0.05, 0.30);
  DifferentialDrive robot(params);

  Path2D path = make_line_path({0.0, 0.0}, {4.0, 1.0}, 60);

  PurePursuitConfig cfg;
  cfg.lookahead = 0.4;
  cfg.speed = 0.4;
  cfg.max_omega = 1.5;

  PurePursuitController controller(path, cfg);

  Pose2D pose(0.0, -0.3, 0.0);
  std::vector<Pose2D> poses;
  poses.push_back(pose);

  const double dt = 0.02;
  for (int k = 0; k < 400; ++k) {
    Twist2D cmd = controller.compute(pose);
    WheelVelocity wheels = robot.inverse(cmd);
    pose = integrate_exact(pose, robot.forward(wheels), dt);
    poses.push_back(pose);
  }

  figure(800, 500);
  plot_path(path, "path");
  plot_trajectory(poses, "trajectory");
  draw_robot(poses.back(), params);
  grid(true);
  legend(true);
  savefig("my_robot_demo.svg");
}
```

## 12. Gioi Han Hien Tai

Phien ban hien tai chua phai robotics framework day du. Cac gioi han can biet:

- chua co ROS message/node integration;
- chua co collision checking tong quat;
- chua co map/occupancy grid;
- chua co global planner nhu A*, RRT;
- chua co dynamics day du cua mobile base;
- chua co manipulator kinematics trong module chinh;
- pure pursuit hien chon waypoint lookahead don gian, chua noi suy chinh xac tren segment.

## 13. Huong Mo Rong Tiep Theo

Thu tu nen lam:

1. cai thien `Path2D::lookahead_point()` bang noi suy theo segment;
2. them `simulate_tracking()` tra ve `TrackingSample` day du;
3. them LQR lateral-error tracking wrapper;
4. them MPC tracking wrapper dung `cppplot::control::MPCController`;
5. them CBF obstacle safety adapter;
6. them planar 2-link manipulator.

## 14. Quy Uoc API

- Don vi mac dinh la SI: meter, second, radian.
- Goc `theta` luon nen duoc normalize ve `[-pi, pi]`.
- `omega_left/right` la rad/s.
- `v` la m/s.
- `omega` la rad/s.
- Path luu theo thu tu waypoint.
- Simulation deterministic, khong phu thuoc thread hay random.

