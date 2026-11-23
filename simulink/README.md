# Mô hình Pure Pursuit trên Simulink

Yêu cầu MATLAB/Simulink R2025b. Chạy các lệnh sau từ thư mục này.

```matlab
open_system('KT_Robot_Pure_Pursuit')
[data, metrics] = run_kt_robot("corners","curvature");
summary = evaluate_kt_robot();
export_kt_diagrams();
```

Nhấn **Run** trong Simulink để chạy tuyến góc ngoặt với phương án cơ sở,
look-ahead 0,55 m và chu kỳ 0,05 s. Dùng `build_kt_robot` để dựng lại tệp SLX
sau khi chỉnh mã các khối MATLAB Function. Mỗi khối lưu mã trực tiếp trong mô hình.

`run_kt_robot` nhận tên tuyến (`straight`, `s_curve` hoặc `corners`) hoặc ma trận
waypoint hai cột, phương án (`baseline` hoặc `curvature`), look-ahead (m), chu kỳ
(s), thời gian tối đa (s) và tư thế đầu `[x;y;theta]` (m, m, rad). Nhật ký dùng
cùng 14 cột với bản Python. Mã chế độ 0 là bám đường, 1 là căn hướng, 2 là đạt
đích và 3 là hết thời gian.

`evaluate_kt_robot` đọc cấu hình và dữ liệu chuẩn từ
`../pure_pursuit_evaluation/outputs`, chạy lại 42 cấu hình trong Simulink và ghi
sai khác từng tín hiệu vào `outputs/python_simulink_comparison.csv`. Sáu nhật ký
danh định, bảng tổng hợp danh định, số liệu tại các góc và đồ thị MATLAB cũng
nằm trong `outputs`. Có thể gọi `plot_kt_results` để vẽ lại từ các CSV đã lưu
và `run_kt_checks` để chạy lại các ca kiểm tra chức năng.

## Nguồn tham khảo

- [Peter Corke trên GitHub](https://github.com/petercorke) và
  [Robotics Toolbox for MATLAB](https://github.com/petercorke/robotics-toolbox-matlab).
  [purepursuit.m tại commit 06fddf5](https://github.com/petercorke/robotics-toolbox-matlab/blob/06fddf5228cbfa1132fa63b8e336b43c5ed04899/purepursuit.m)
  minh họa chọn mục tiêu trên dãy waypoint. Mô hình KT-Robot giữ phép tìm giao
  điểm trên đa tuyến như bản Python.
- [P. I. Corke, A Robotics Toolbox for MATLAB, IEEE Robotics & Automation Magazine, 1996](https://doi.org/10.1109/100.486658).
- [R. Craig Coulter, Implementation of the Pure Pursuit Path Tracking Algorithm, 1992](https://www.ri.cmu.edu/publication_view.html?pub_id=278).
- [PythonRobotics, ví dụ nguồn mở đã tái lập](https://github.com/AtsushiSakai/PythonRobotics/blob/ef64608ceb0012f2a0b3d532fddc80243e807cfa/PathTracking/pure_pursuit/pure_pursuit.py).
- [Macenski và cộng sự, Regulated Pure Pursuit for Robot Path Tracking, Mục 5](https://arxiv.org/html/2305.20026#S5)
  là mẫu trình bày quan hệ giữa sai số, tốc độ và thời gian.

## Thử nghiệm độ bền

Mở `KT_Robot_Robustness.slx` để xem vòng kín dùng tư thế đo, khối động cơ và
trượt, tích phân tư thế thật, bộ kích hoạt theo tiến độ thật và điều kiện
dừng vật lý. Cấu hình mở sẵn là tuyến góc ngoặt, phương án cơ sở, tổ hợp mạnh
và hạt giống 1. Bộ điều khiển dùng cùng mã với mô hình danh định.

```matlab
build_kt_robustness
[data, metrics] = run_kt_robustness("corners","baseline","combined_strong",1);
[poses, studies] = evaluate_kt_robustness();
export_kt_robustness
```

Gọi lệnh trong thư mục `simulink` với MATLAB/Simulink R2025b. Cần có mô
hình danh định để dựng mô hình độ bền. Tùy chọn
`evaluate_kt_robustness("outputs_robustness",false)` chạy đủ ma trận và bỏ
bước tạo ZIP nhật ký đầy đủ. Có thể chạy lại kiểm tra bằng
`check_kt_robustness` và chạy đối chiếu danh định bằng `evaluate_kt_robot`.

- `initial_pose_trials.csv` ghi 70 lượt của 35 tư thế đầu. Hai luật được
  chạy tại mỗi tư thế trên tuyến góc ngoặt.
- `robustness_trials.csv` ghi 384 lượt trên tuyến góc ngoặt và chữ S. Hai
  luật nhận cùng chuỗi Gaussian chuẩn ở mỗi hạt giống, ghép theo mẫu từ
  đầu lượt chạy. Bản tổng hợp dùng toàn bộ lượt cho RMSE, sai số cuối,
  thời lượng căn hướng và cực đại của \(|v\omega|\). Thống kê thời gian báo
  đích chỉ dùng các ca thành công. `robustness_pairs.csv` lấy giảm tốc trừ cơ sở.
- `simulation_traces.zip` chứa 454 nhật ký CSV theo khóa `trace_id`.
  Mỗi mẫu ghi riêng tư thế thật, tư thế đo, lệnh bánh, tốc độ bánh thực
  và vận tốc thân. Hai tệp `incident_*.csv` dùng để vẽ sự cố tổ hợp mạnh
  trên tuyến góc ngoặt với hạt giống 1.
- `experiment_manifest.json` ghi phiên bản môi trường, tham số dùng chung
  và hạt giống. Các mức của từng yếu tố có trong `kt_robust_profile.m`.
  Mã phiên bản Python và SHA-256 đã có tại
  `../pure_pursuit_evaluation/outputs/experiment_manifest.json`.

Trong mô hình mở rộng, `success` yêu cầu có báo đích trước khi hết thời hạn,
sai số vị trí thật cuối không quá 0,05 m và tốc độ tuyệt đối của cả hai bánh
không quá 0,1 rad/s. `false_goal` là có báo đích nhưng thiếu ít nhất một
điều kiện thật. `timeout` là kết thúc do hết thời gian. Mô hình khóa lệnh
bánh bằng 0 tại mẫu báo đích hoặc hết thời gian và chạy tiếp tối đa 2 s
để theo dõi bánh giảm tốc. Đại lượng \(|v\omega|\) dùng vận tốc dọc thật và
tốc độ quay thân. RMSE theo quãng đường dùng cả vận tốc dọc lẫn ngang.

### Nguồn cho mô hình độ bền

- [Nantabut, ICINCO 2024, phương trình (35) và Mục 3.1](https://arxiv.org/html/2409.18585#S3.SS1)
  cung cấp phân bố Gaussian và độ lệch chuẩn \((0\ \mathrm{m},0{,}10\
  \mathrm{m},10^\circ)\). Bộ điều khiển trong ma trận vẫn là hai luật của KT-Robot.
- [Wu và Wang, mục Numerical simulations](https://doi.org/10.1177/01423312241239145)
  cung cấp hai dạng sóng nhiễu vận tốc dọc và ngang. KT-Robot áp chúng trong
  3 s ở gần đoạn cua, dùng pha tính từ khi sự cố bắt đầu. Các thông số còn
  lại là giả định mô phỏng.

