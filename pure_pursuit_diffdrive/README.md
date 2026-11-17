# Bước 2 — Robot hai bánh vi sai bám đường bằng Pure Pursuit

Bản bàn giao tiếp nối ví dụ gốc PythonRobotics đã chạy ở bước 1. Đây là **chương trình robot vi sai được viết lại**, kế thừa nguyên lý Pure Pursuit; không phải bản sao mã gốc đổi hình xe. Đọc nguồn tại mục 8 để phân biệt tài liệu tham khảo và phần triển khai.

## 1. Mở và chạy trong VS Code

Giải nén, đặt thư mục `pure_pursuit_diffdrive` cạnh `pure_pursuit_baseline`, ví dụ `E:\ROBOT\pure_pursuit_diffdrive`. Hai bước nằm riêng để giữ nguyên kết quả tái lập gốc.

Mở `pure_pursuit_diffdrive` bằng **File → Open Folder**. Python 3.12 được khuyến nghị. Các thư viện giống những thư viện đã dùng ở bước 1, nhưng bước này không cần SciPy.

Nếu lệnh `python` của bạn đã chạy được bước 1, tại Terminal của thư mục mới chạy:

```powershell
python -m pip install -r requirements.txt
python run_robot.py --animate
```

Nếu bạn dùng môi trường `.venv`, chọn đúng **Python: Select Interpreter** và dùng Python của môi trường đó. Có thể tạo môi trường riêng trong Windows PowerShell:

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe run_robot.py --animate
```

Trên macOS/Linux, thay bằng `python3.12 -m venv .venv` và `./.venv/bin/python`. F5 có sẵn cấu hình **Robot vi sai: mo phong chu S**.

### Các lệnh thường dùng

```powershell
# Mở cửa sổ robot bám đường chữ S
python run_robot.py --animate

# Mở cửa sổ đường có góc ngoặt
python run_robot.py --scenario corners --animate

# Chạy cả ba đường và lưu GIF, không cần mở cửa sổ
python run_robot.py --scenario all --gif

# Chỉ xuất dữ liệu và ảnh, không tạo GIF
python run_robot.py --scenario all

# Kiểm tra các trường hợp động học và điều kiện đạt đích
python -m unittest -v

# Thử một giá trị look-ahead khác (chưa phải kết quả khảo sát chính thức)
python run_robot.py --lookahead 0.8 --animate
```

Không kết hợp `--scenario all` với `--animate`; chọn một đường để xem cửa sổ. Các kết quả lưu tự động vào `outputs/<tên đường>/`. Chạy lại cùng tên đường sẽ ghi đè kết quả của đường đó; lưu bản sao nếu cần so sánh nhiều tham số. Không sửa trực tiếp tệp kết quả JSON/CSV để thay thông số.

## 2. Bài toán, mô hình và giả thiết

Robot di chuyển theo tuyến đường cho trước trên mặt phẳng, minh họa nhiệm vụ robot vận chuyển trong nhà xưởng. Tư thế robot là `(x, y, theta)` tại **trung điểm trục hai bánh**. Trục x của robot hướng tiến, trục y hướng trái, góc theta dương ngược chiều kim đồng hồ. Tốc độ bánh dương tương ứng chuyển động tiến.

Mô hình động học vi sai được tham khảo từ [2]:

```text
v     = r/2 * (phi_dot_R + phi_dot_L)
omega = r/b * (phi_dot_R - phi_dot_L)

phi_dot_L = (v - b*omega/2)/r
phi_dot_R = (v + b*omega/2)/r

x_dot     = v*cos(theta)
y_dot     = v*sin(theta)
theta_dot = omega
```

`r` là bán kính bánh [m], `b` là khoảng cách hai bánh [m], `phi_dot_L/R` là tốc độ góc bánh trái/phải [rad/s], `v` [m/s], `omega` [rad/s]. Chương trình tích phân chính xác chuyển động với lệnh vận tốc không đổi trong từng bước lấy mẫu (zero-order hold), có nhánh chuyển động thẳng khi omega gần 0.

**Giả thiết:** bánh lăn không trượt, biết chính xác tư thế robot, tốc độ bánh đáp ứng tức thời; không mô hình động cơ, quán tính, ma sát, cảm biến, nhiễu hoặc vật cản. Các thông số dưới đây là lựa chọn cho mô phỏng, không phải số đo của một robot thương mại.

| Thông số | Giá trị | Đơn vị |
|---|---:|---|
| Bán kính bánh r | 0,05 | m |
| Khoảng cách hai bánh b | 0,30 | m |
| Giới hạn trị tuyệt đối tốc độ mỗi bánh | 12 | rad/s |
| Giới hạn trị tuyệt đối tốc độ quay thân robot | 2 | rad/s |
| Vận tốc tiến yêu cầu khi bám đường | 0,50 | m/s |
| Look-ahead cố định | 0,55 | m |
| Bước lấy mẫu | 0,05 | s |
| Khoảng cách chấp nhận tại đích | 0,05 | m |
| Hệ số giảm tốc khi tiếp cận đích | 1 | 1/s |
| Ngưỡng kích hoạt căn hướng | 75 | độ |
| Hệ số điều khiển căn hướng | 2 | 1/s |
| Thời gian tối đa | 120 | s |

## 3. Điều khiển Pure Pursuit và xử lý bổ sung

Nguyên lý hình học dựa trên [1], với quy ước trục x hướng tiến của bản triển khai này:

1. Chiếu vị trí lên phần tuyến phía trước để cập nhật tiến độ dọc đường, không cho tiến độ lùi. Cửa sổ tìm kiếm có chiều dài `max(2*lookahead, 1 m)`; đây là lựa chọn triển khai, không phải định lý bảo đảm mọi tuyến đường.
2. Tìm giao điểm đầu tiên phía trước giữa đường và vòng tròn look-ahead. Nếu không có giao điểm, dùng một điểm nội suy theo chiều dài đường. Gần cuối tuyến, chọn đích làm điểm mục tiêu.
3. Đổi điểm mục tiêu sang hệ tọa độ robot: `y_body = -sin(theta)*dx + cos(theta)*dy`.
4. Tính `kappa = 2*y_body/(dx²+dy²)`, rồi `omega = v*kappa`. Dùng khoảng cách thực tới điểm mục tiêu trong mẫu số, kể cả khi điểm được chọn bằng nhánh dự phòng.
5. Quy đổi `v, omega` thành tốc độ bánh. Nếu vượt giới hạn quay thân hoặc tốc độ bánh, nhân cả hai lệnh cùng hệ số giảm để giữ độ cong yêu cầu.
6. Cập nhật trạng thái, ghi dữ liệu và kiểm tra đích.

**Các xử lý phụ trợ được tách rõ khỏi luật Pure Pursuit:**

- Khi điểm mục tiêu lệch quá 75 độ so với hướng tiến, robot quay tại chỗ bằng điều khiển P để căn hướng. Hai bánh có thể quay ngược chiều. Đây là chế độ phục hồi hướng, không phải công thức Pure Pursuit.
- Trên phần tuyến còn lại không quá look-ahead, dùng `v = min(v_nominal, goal_gain*distance_to_goal)` để tiếp cận đích.
- Chỉ xác nhận thành công khi khoảng cách tới đích không quá 0,05 m **và** tiến độ đã vào phần cuối tuyến. Tại mẫu kết thúc, cả hai lệnh bánh bằng 0.
- Nếu hết thời gian, ghi `timeout`, không tính là thành công; mã thoát chương trình là 1.

Ở giai đoạn này **chưa có luật giảm tốc chủ động theo độ cong để so sánh cải tiến**. Vận tốc thực tế có thể giảm khi vào cua do giới hạn tốc độ bánh; đây là tác dụng của bão hòa cơ cấu chấp hành và sẽ được giữ chung cho cả hai phương pháp trong bước đánh giá sau.

## 4. Kết quả kiểm chứng bước 2

Ba tuyến dưới đây là dữ liệu tổng hợp do chương trình tạo, không phải dữ liệu thực nghiệm. Cùng trạng thái ban đầu `(0; −0,6; 0)` và cùng cấu hình mặc định.

| Tuyến | Thời gian đạt điều kiện đích [s] | Sai số cuối [m] | RMSE bám đường [m] | Đạt đích |
|---|---:|---:|---:|---|
| Thẳng, dài 8 m | 17,65 | 0,0498 | 0,1468 | Có |
| Chữ S: y = sin(2πx/10), x từ 0 đến 10 m | 24,20 | 0,0484 | 0,1589 | Có |
| Góc ngoặt: (0,0) → (4,0) → (4,3) → (8,3) → (8,6) | 28,70 | 0,0485 | 0,1206 | Có |

RMSE được tính từ khoảng cách Euclid nhỏ nhất từ trung điểm trục hai bánh tới **các đoạn thẳng** của đường tham chiếu, có tính sai lệch ban đầu. Thời gian ở đây là thời gian mô phỏng, không phải thời gian máy tính chạy. Không so sánh trực tiếp RMSE này với bước 1 vì khác mô hình, đường, tốc độ, trạng thái đầu và tiêu chuẩn dừng.

Đây là kiểm chứng chức năng cho bản robot vi sai cơ sở. Các con số chưa chứng minh một cải tiến tốt hơn và chưa bảo đảm kết quả tương tự ngoài thực tế. Đường góc ngoặt có thể bị cắt góc; một khoảng nhìn trước hữu hạn không ép robot đi qua chính xác từng waypoint.

Đã chạy **9 bài kiểm tra tự động**: chuyển động thẳng, quay tại chỗ, cung 1/4 đường tròn có nghiệm giải tích, bảo toàn độ cong khi giới hạn tốc độ, ba tuyến đạt đích/đúng quan hệ tốc độ bánh, phục hồi khi quay ngược, tuyến kín không kết thúc sai ở thời điểm đầu, hết thời gian không báo thành công và từ chối tham số sai. GIF/PNG được tạo từ dữ liệu của chính lần chạy. Cửa sổ tương tác chưa được kiểm chứng trên máy người dùng.

## 5. Cấu trúc mã và dữ liệu

| Tệp | Vai trò |
|---|---|
| `model.py` | Động học thuận/ngược, giới hạn lệnh và tích phân |
| `tracking.py` | Pure Pursuit, chọn điểm, căn hướng, điều kiện đích và mô phỏng |
| `scenarios.py` | Định nghĩa ba tuyến và trạng thái đầu |
| `run_robot.py` | Lệnh chạy, xuất CSV/JSON/PNG/GIF và xem cửa sổ |
| `test_verification.py` | Kiểm tra bằng nghiệm giải tích và các điều kiện chức năng |
| `requirements.txt` | Phiên bản thư viện đã dùng |
| `.vscode/launch.json` | Cấu hình chạy F5 |
| `outputs/verification_summary.csv` | Bảng tổng hợp lần chạy cả ba tuyến |
| `outputs/<tuyến>/reference.csv` | Đường tham chiếu |
| `outputs/<tuyến>/trajectory.csv` | Trạng thái, lệnh và sai số theo thời gian |
| `outputs/<tuyến>/summary.json` | Kết quả, cấu hình, giả thiết và phiên bản thư viện |
| `outputs/<tuyến>/results.png` | Đồ thị quỹ đạo, sai số, vận tốc thân và tốc độ bánh |
| `outputs/<tuyến>/animation.gif` | Hoạt ảnh robot hai bánh |

Mỗi dòng CSV ghi tư thế tại thời điểm t và lệnh được áp dụng trên khoảng `[t, t+dt)`. Dòng cuối là tư thế kết thúc với lệnh bằng 0. `mode_code`: 0=bám đường Pure Pursuit, 1=căn hướng, 2=đạt đích, 3=hết thời gian.

GIF lấy tối đa 90 khung hình và phát 12 hình/giây để giảm dung lượng, không phát theo đúng thời gian vật lý. Khi xem bằng `--animate`, khoảng thời gian đặt giữa hai hình là dt; tốc độ hiển thị còn phụ thuộc máy tính.

## 6. Nội dung kế thừa và tự triển khai

| Nguồn / phần | Phạm vi |
|---|---|
| Ví dụ PythonRobotics của bước 1 [3] | Điểm xuất phát để học và tái lập thuật toán; không nhập hoặc sửa các tệp vendor ở bước 2 |
| Coulter [1] | Nguyên lý hình học Pure Pursuit và điểm nhìn trước |
| Dellaert và Hutchinson [2] | Động học robot hai bánh vi sai |
| Bản triển khai bước 2 | Viết lại mô hình và vòng lặp cho trung điểm trục hai bánh; tìm giao điểm look-ahead; giới hạn tốc độ đồng tỷ lệ; căn hướng; điều kiện đạt đích; xuất dữ liệu và kiểm chứng |

PythonRobotics sử dụng look-ahead phụ thuộc vận tốc; bản robot cơ sở này chọn look-ahead cố định để dễ khảo sát riêng từng yếu tố. Đây là lựa chọn cấu hình, không khẳng định ưu việt hơn.

ChatGPT hỗ trợ viết mã, tài liệu và kiểm tra kết quả. Người thực hiện cần chạy lại, hiểu công thức và tự chịu trách nhiệm khi sử dụng trong báo cáo. Các bài báo/tài liệu tham khảo chỉ được dẫn liên kết; không đóng gói lại bản PDF của tác giả.

## 7. Giới hạn và bước tiếp theo

Chương trình bám đường hình học (path following), chưa theo một lịch vị trí theo thời gian (trajectory tracking có ràng buộc thời gian). Không xử lý tổng quát các tuyến tự cắt, các đoạn rất gần nhau, vật cản hoặc nhiễu định vị. Kiểm tra tuyến kín chỉ xác nhận một trường hợp đơn giản, không phải chứng minh cho mọi đường kín.

Sau khi chạy lại bước 2 trên máy của bạn, bước 3 sẽ khảo sát look-ahead và so sánh **bản cơ sở** với **bản có luật giảm tốc theo độ cong** trên cùng robot, cùng điều kiện đầu và giới hạn. Giảm tốc khi tới đích và căn hướng sẽ giữ chung để việc so sánh có ý nghĩa. Phân tích cả sai số, thời gian và bão hòa tốc độ; không giả định trước bản mở rộng luôn tốt hơn. Báo cáo học phần sẽ được viết sau khi có bộ kết quả đánh giá này.

## 8. Tài liệu tham khảo

[1] R. C. Coulter, “Implementation of the Pure Pursuit Path Tracking Algorithm,” Carnegie Mellon University, Pittsburgh, PA, USA, Tech. Rep. CMU-RI-TR-92-01, Jan. 1992. [Online]. Available: https://publications.ri.cmu.edu/implementation-of-the-pure-pursuit-path-tracking-algorithm. Accessed: Nov. 17, 2026.

[2] F. Dellaert and S. Hutchinson, “5.2. Motion Model for the Differential Drive Robot,” *Introduction to Robotics and Perception*. [Online]. Available: https://www.roboticsbook.org/S52_diffdrive_actions.html. Accessed: Nov. 17, 2026.

[3] A. Sakai, G. Jacquenot, and contributors, “Pure pursuit path tracking simulation,” *PythonRobotics*, GitHub, commit ef64608ceb0012f2a0b3d532fddc80243e807cfa. [Online]. Available: https://github.com/AtsushiSakai/PythonRobotics/blob/ef64608ceb0012f2a0b3d532fddc80243e807cfa/PathTracking/pure_pursuit/pure_pursuit.py. Accessed: Nov. 17, 2026.
