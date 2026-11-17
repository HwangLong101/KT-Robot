# Bước 1 — Tái lập ví dụ gốc Pure Pursuit bằng Python

Đây là bản bàn giao bước 1 cho đề tài **Nghiên cứu và mô phỏng thuật toán điều khiển bám quỹ đạo cho robot di động hai bánh chủ động**. Mục tiêu hiện tại là chạy và hiểu ví dụ gốc trước khi chuyển sang mô hình robot vi sai.

## 1. Phạm vi và nguồn gốc

- Nguồn: Atsushi Sakai, Guillaume Jacquenot và các cộng tác viên, PythonRobotics [1].
- Phiên bản đã tải và chạy: `ef64608ceb0012f2a0b3d532fddc80243e807cfa`, truy cập ngày 17/11/2026.
- Các tệp trong `vendor/PythonRobotics/` được giữ nguyên nội dung theo phiên bản trên. `source_manifest.json` lưu SHA-256; chương trình xuất kết quả kiểm tra các mã này trước khi chạy.
- Chỉ đóng gói phần mã cần thiết cùng README và LICENSE gốc, không phải toàn bộ repository.
- Giấy phép MIT gốc được giữ tại `vendor/PythonRobotics/LICENSE` [3].

**Mô hình gốc là xe có góc lái, sử dụng mô hình động học bicycle. Bản này chưa phải mô hình robot hai bánh vi sai.** Việc chuyển đổi mô hình là công việc của bước 2. Không dùng số liệu của xe gốc làm đối chứng trực tiếp để kết luận robot vi sai đã được cải thiện.

## 2. Cài đặt và chạy trong VS Code

Khuyến nghị Python **3.12**, VS Code và extension **Python** của Microsoft. Môi trường đã kiểm chứng: Python 3.12.14 trên Linux; các phiên bản thư viện được ghi trong `requirements.txt` và `outputs/summary.json`. Chưa kiểm chứng giao diện VS Code hoặc cửa sổ tương tác trên máy của người dùng.

1. Giải nén và mở đúng thư mục `pure_pursuit_baseline` bằng **File → Open Folder** trong VS Code.
2. Chọn **Terminal → New Terminal**.
3. Tạo môi trường và cài thư viện theo hệ điều hành dưới đây. Cần Internet khi cài lần đầu.

### Windows — PowerShell

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe run_baseline.py --gif
```

Nếu không có lệnh `py`, dùng `python -m venv .venv` sau khi kiểm tra `python --version` là Python 3.12. Không cần kích hoạt môi trường bằng `Activate.ps1`.

### macOS hoặc Linux

```bash
python3.12 -m venv .venv
./.venv/bin/python -m pip install -r requirements.txt
./.venv/bin/python run_baseline.py --gif
```

Nếu lệnh trên không tìm được Python 3.12, cần cài phiên bản đó trước hoặc dùng đường dẫn tới bản Python 3.12 đã cài.

### Chạy bằng F5

Nhấn **Ctrl+Shift+P** trên Windows/Linux hoặc **Cmd+Shift+P** trên macOS → **Python: Select Interpreter** → chọn Python trong `.venv` vừa tạo. Vào **Run and Debug**, chọn:

- **Baseline: export results**: chạy mô phỏng và xuất PNG, CSV, JSON. Muốn thêm GIF, chạy lệnh `run_baseline.py --gif` trong terminal.
- **Original upstream: interactive animation**: chạy trực tiếp tệp gốc với cửa sổ Matplotlib; phím Space tạm dừng và Esc thoát theo mã gốc.

Có thể chạy cửa sổ gốc từ terminal Windows:

```powershell
.\.venv\Scripts\python.exe vendor/PythonRobotics/PathTracking/pure_pursuit/pure_pursuit.py
```

Hoặc macOS/Linux:

```bash
./.venv/bin/python vendor/PythonRobotics/PathTracking/pure_pursuit/pure_pursuit.py
```

Chế độ cửa sổ cần môi trường desktop và backend Matplotlib phù hợp. Nếu cửa sổ không mở, chế độ xuất kết quả vẫn chạy không cần giao diện. `run_baseline.py` luôn lưu kết quả cạnh chính tệp này trong `outputs/`, kể cả khi gọi từ thư mục khác.

## 3. Nguyên lý và cấu hình gốc

Trạng thái gồm tọa độ tâm xe `(x, y)`, góc hướng `yaw` và vận tốc `v`. Mã dùng điểm tham chiếu phía sau được tính từ tâm xe, chiều dài cơ sở và góc hướng để chọn điểm nhìn trước. Cập nhật động học trong một bước thời gian:

```text
x_next   = x + v*cos(yaw)*dt
y_next   = y + v*sin(yaw)*dt
yaw_next = wrap(yaw + v/WB*tan(delta)*dt)
v_next   = v + a*dt
```

Trong chế độ tiến mặc định, Pure Pursuit tính góc lái từ góc tới điểm mục tiêu và khoảng nhìn trước `Lf = Lfc + k*v`, sau đó giới hạn góc lái. Lưu ý **mã gốc đã có look-ahead phụ thuộc vận tốc**; không thể nhận riêng công thức này là cải tiến mới. Dù phần giới thiệu upstream dùng tên PID, hàm điều khiển vận tốc thực tế của phiên bản này là **P**: `a = Kp*(target_speed-v)`.

| Thông số | Giá trị gốc | Đơn vị |
|---|---:|---|
| Bước thời gian | 0,1 | s |
| Chiều dài cơ sở WB | 2,9 | m |
| Look-ahead cơ sở Lfc | 2,0 | m |
| Hệ số look-ahead k | 0,1 | s |
| Hệ số điều khiển tốc độ Kp | 1,0 | 1/s |
| Vận tốc đặt | 10 | km/h |
| Giới hạn góc lái | π/4 | rad |
| Thời gian mô phỏng tối đa | 100 | s |
| Trạng thái đầu (x, y, yaw, v) | (0; −3; 0; 0) | m, m, rad, m/s |

Đường tham chiếu được lấy tại `x = 0; 0,5; ...; 49,5 m`, với `y = x*sin(x/5)/2` theo mã nguồn [1].

## 4. Kết quả đã chạy

| Đại lượng | Kết quả |
|---|---:|
| Số mẫu, tính cả trạng thái đầu | 405 |
| Thời gian mô phỏng khi dừng | 40,40 s |
| Chỉ số điểm nhìn trước cuối / chỉ số cuối tuyến | 99 / 99 |
| Khoảng cách tâm xe tới đích khi dừng | 2,8092 m |
| Khoảng cách điểm tham chiếu sau tới đích | 4,2591 m |
| RMSE khoảng cách tâm xe tới đường tham chiếu | 0,8493 m |
| Khoảng cách lớn nhất của tâm xe tới đường | 3,0014 m |
| RMSE khoảng cách điểm tham chiếu sau tới đường | 1,0779 m |

Các sai số được tính bằng **khoảng cách Euclid nhỏ nhất tới các đoạn thẳng của đường tham chiếu**, lấy tất cả mẫu kể cả sai lệch ban đầu. RMSE là căn bậc hai trung bình bình phương khoảng cách. Hai điểm đo (tâm xe và điểm tham chiếu sau) được xuất riêng, không trộn lẫn.

**40,40 s là thời gian đến điều kiện dừng gốc, chưa phải thời gian đạt đích.** Vòng lặp gốc dừng khi chỉ số điểm nhìn trước chạm điểm cuối; kiểm tra `assert` cuối chương trình không xác nhận khoảng cách tới đích nhỏ hơn một ngưỡng. Hạn chế này được giữ nguyên để tái lập trung thực và sẽ được xử lý trong mô hình robot vi sai.

`baseline_results.png` là ảnh kết quả hậu xử lý từ lần chạy thực tế, không phải ảnh lấy từ tài liệu. `baseline_animation.gif` dựng lại chuyển động từ cùng dữ liệu; đã lược bớt khung hình để giảm dung lượng, thời lượng phát GIF không bằng thời gian mô phỏng. Đây là hình xe có góc lái ở bước tái lập gốc.

## 5. Các tệp bàn giao

| Tệp / thư mục | Công dụng |
|---|---|
| `vendor/PythonRobotics/` | Mã và giấy phép gốc |
| `run_baseline.py` | Chạy hàm main gốc, ghi nhận trạng thái, tính sai số và xuất kết quả |
| `requirements.txt` | Phiên bản thư viện đã kiểm chứng |
| `.vscode/launch.json` | Hai cấu hình chạy bằng F5 |
| `source_manifest.json` | Nguồn, commit, ngày truy cập và SHA-256 |
| `outputs/trajectory.csv` | Trạng thái và sai số theo thời gian, có đơn vị trong tên cột |
| `outputs/reference_path.csv` | Đường tham chiếu |
| `outputs/summary.json` | Thông số, chỉ tiêu, môi trường và giới hạn |
| `outputs/baseline_results.png` | Đồ thị quỹ đạo, vận tốc và sai số |
| `outputs/baseline_animation.gif` | Hoạt ảnh hậu xử lý |

## 6. Phân biệt phần kế thừa và phần bổ sung

**Kế thừa [1]:** mô hình xe, Pure Pursuit, điều khiển vận tốc P, đường mẫu, trạng thái đầu, tham số, bước thời gian và điều kiện dừng.

**Bổ sung trong bản bàn giao:** bộ chạy và xuất dữ liệu, kiểm tra SHA-256, tính sai số theo đoạn đường, đồ thị, GIF hậu xử lý, cấu hình VS Code và hướng dẫn tiếng Việt. Bộ chạy gọi nguyên hàm `main()` upstream, chỉ đặt `show_animation=False` để xuất dữ liệu không cần cửa sổ; dùng profiler đọc biến cục bộ lúc hàm kết thúc. Không sửa vòng lặp mô phỏng hay luật điều khiển gốc.

Công cụ AI ChatGPT hỗ trợ chuẩn bị bộ chạy, tài liệu và phân tích kết quả. Chương trình đã được thực thi trong môi trường nêu trên. Người thực hiện cần chạy lại trên máy của mình, kiểm tra và hiểu mã trước khi sử dụng trong báo cáo.

## 7. Công việc tiếp theo

1. Mô hình robot vi sai có bán kính bánh `r`, khoảng cách hai bánh `b`, điều khiển bằng `v, omega` và quy đổi sang tốc độ góc từng bánh.
2. Thiết lập Pure Pursuit cơ sở trên robot vi sai, kèm giới hạn vận tốc bánh và điều kiện đạt đích theo khoảng cách.
3. Mở rộng giảm tốc khi vào cua, khảo sát look-ahead và so sánh các cấu hình **trên cùng robot vi sai**. Không giả định trước kết quả cải thiện.
4. Đánh giá đường thẳng, chữ S và đường có góc ngoặt; tách cấu hình khảo sát khỏi kịch bản kiểm chứng khi phù hợp.
5. Viết báo cáo theo tài liệu học phần khi có kết quả; trích dẫn mô hình, thuật toán và mã đúng vị trí.

## 8. Tài liệu tham khảo

[1] A. Sakai, G. Jacquenot, and contributors, “Pure pursuit path tracking simulation,” *PythonRobotics*, GitHub, commit ef64608ceb0012f2a0b3d532fddc80243e807cfa. [Online]. Available: https://github.com/AtsushiSakai/PythonRobotics/blob/ef64608ceb0012f2a0b3d532fddc80243e807cfa/PathTracking/pure_pursuit/pure_pursuit.py. Accessed: Nov. 17, 2026.

[2] PythonRobotics contributors, “Pure pursuit tracking,” *PythonRobotics documentation*. [Online]. Available: https://atsushisakai.github.io/PythonRobotics/modules/6_path_tracking/pure_pursuit_tracking/pure_pursuit_tracking.html. Accessed: Nov. 17, 2026.

[3] A. Sakai and contributors, “MIT License,” *PythonRobotics*, GitHub. [Online]. Available: https://github.com/AtsushiSakai/PythonRobotics/blob/ef64608ceb0012f2a0b3d532fddc80243e807cfa/LICENSE. Accessed: Nov. 17, 2026.
