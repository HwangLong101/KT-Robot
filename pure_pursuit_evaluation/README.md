# Bước 3 — Khảo sát và so sánh Pure Pursuit cho robot vi sai

Bản bàn giao tiếp nối bước 2 đã chạy thành công trên máy người dùng. Giữ riêng `pure_pursuit_baseline` và `pure_pursuit_diffdrive` để lưu lịch sử. Giải nén bản này vào `E:\ROBOT\pure_pursuit_evaluation` rồi mở bằng VS Code.

## Chạy nhanh

Dùng cùng Python đã chạy được bước 2:

```powershell
python -m pip install -r requirements.txt
python run_evaluation.py --gif
```

Chương trình chạy **42 cấu hình xác định**, lưu dữ liệu, bảng CSV, đồ thị, GIF so sánh đường góc ngoặt và tạo lại `Ket_qua_danh_gia.md`. Có thể chạy `python run_evaluation.py` để bỏ phần GIF và chạy nhanh hơn. Kết quả được ghi cạnh mã nguồn trong `outputs/`.

Để mở cửa sổ xem riêng bản giảm tốc khi vào cua:

```powershell
python run_robot.py --method curvature --scenario corners --animate
```

Để xem bản cơ sở trên cùng đường:

```powershell
python run_robot.py --method baseline --scenario corners --animate
```

Nếu dùng `.venv`, chọn đúng interpreter trong VS Code và gọi Python của môi trường đó. Trên Windows có thể thay `python` bằng `.\.venv\Scripts\python.exe`; trên macOS/Linux dùng `./.venv/bin/python`. Các thư viện đã dùng là NumPy 2.3.5, Matplotlib 3.10.8, Pillow 13.0.0; môi trường kiểm chứng Python 3.12.14 trên Linux. Chưa kiểm chứng cửa sổ tương tác trên máy người dùng.

## Thay đổi kỹ thuật

Mô hình động học, robot, đường, trạng thái đầu, luật căn hướng, giới hạn bánh và tiêu chuẩn đạt đích được giữ giống bước 2. Phương pháp `baseline` giữ nguyên kết quả bước 2; bài kiểm tra hồi quy kiểm chứng lại cả ba tuyến. Phương pháp `curvature` bổ sung luật:

```text
v_selected = min(v_base, sqrt(a_limit / max(abs(kappa), epsilon)))
omega_selected = v_selected * kappa
```

Trong đó kappa [1/m] là độ cong yêu cầu của Pure Pursuit, a_limit=0,12 m/s² là ngưỡng thiết kế cho yêu cầu gia tốc ngang động học. Lệnh tiếp tục qua cùng bộ giới hạn thân và bánh; phép giảm đồng tỷ lệ giữ độ cong và không làm tăng |v·omega|. Luật này giữ |v·omega| ≤ a_limit trong mô hình, kể cả khi quay tại chỗ vì v=0. Giá trị epsilon=10⁻¹² tránh chia cho 0, không phải giới hạn vật lý.

Đây là mở rộng đơn giản, được tham khảo về định hướng điều chỉnh vận tốc từ [3]. **Không gọi đây là bản triển khai đầy đủ Nav2 Regulated Pure Pursuit**. Chưa có tránh vật cản, kiểm tra va chạm, cảm biến, lực bám, quán tính hoặc điều khiển động cơ. Do đó không suy ra đã giảm trượt bánh hay bảo đảm an toàn vật lý.

## Thiết kế đánh giá

| Nhóm | Điều kiện | Số cấu hình |
|---|---|---:|
| Khảo sát look-ahead | 3 tuyến × 2 phương pháp × {0,30; 0,40; 0,55; 0,80; 1,00} m; dt=0,05 s | 30 |
| Bổ sung kiểm tra dt | 3 tuyến × 2 phương pháp × {0,025; 0,1} s; look-ahead=0,55 m | 12 |
| Tổng số cấu hình khác nhau | So sánh chính 6 cấu hình đã nằm trong nhóm 30 | **42** |

Không dùng số lần chạy lặp của mô hình xác định làm cỡ mẫu thống kê. Tất cả cấu hình đều được lưu và công bố; không chọn riêng một look-ahead cho từng phương pháp để tạo lợi thế. So sánh chính dùng cùng look-ahead=0,55 m và ngưỡng 0,12 m/s² đã cố định.

Các thông số chung: r=0,05 m; khoảng cách hai bánh b=0,30 m; tốc độ bánh tối đa 12 rad/s; tốc độ quay thân tối đa 2 rad/s; vận tốc tiến yêu cầu 0,5 m/s; tư thế đầu (0; −0,6; 0); ngưỡng đích 0,05 m. Mỗi lần mô phỏng giới hạn 120 s. Chi tiết trong `summary.json` của từng cấu hình.

## Cách đọc kết quả

Mở `Ket_qua_danh_gia.md` bằng **Ctrl+Shift+V** trong VS Code để xem bản tổng hợp. Các chỉ tiêu gồm:

- RMSE khoảng cách tới các đoạn thẳng của đường, tính đều theo các mẫu thời gian, có sai lệch ban đầu.
- RMSE theo quãng đường: căn trung bình có trọng số ds=|v|dt của e², dùng trung bình e² ở hai đầu mỗi bước. Chỉ tiêu này giảm ảnh hưởng việc robot dành nhiều thời gian hơn ở vùng sai lệch, nhưng không tính thời gian quay tại chỗ.
- Sai số cuối, thời gian đạt đích, sai số lớn nhất và giới hạn tốc độ bánh/thân.
- Max |v·omega| [m/s²]: mức yêu cầu gia tốc ngang **động học**, không phải gia tốc đo từ cảm biến hay mô phỏng lực tiếp xúc.
- Max |v·omega| sau tiến độ 1 m để bổ sung góc nhìn ngoài giai đoạn hội tụ ban đầu. Mốc 1 m được chọn làm phân đoạn mô tả, không thay thế chỉ tiêu toàn tuyến.

Ở mô hình động học lý tưởng với look-ahead cố định, giảm vận tốc chủ yếu thay đổi thời gian di chuyển. Quỹ đạo hình học có thể rất gần nhau; khác biệt nhỏ còn phụ thuộc chu kỳ điều khiển. Vì vậy **không mặc định bản giảm tốc có RMSE nhỏ hơn**. Bản đánh giá giữ nguyên các kết quả bất lợi và trình bày đánh đổi thời gian — mức yêu cầu khi quay.

## Tệp kết quả

| Tệp | Nội dung |
|---|---|
| `Ket_qua_danh_gia.md` | Bảng và diễn giải tự sinh từ số liệu thực sự đã chạy |
| `outputs/comparison.csv` | 6 cấu hình so sánh chính |
| `outputs/all_trials.csv` | Toàn bộ 42 cấu hình |
| `outputs/lookahead_sweep.csv` | 30 cấu hình khảo sát look-ahead |
| `outputs/timestep_check.csv` | 18 cấu hình tại look-ahead=0,55 m, gồm cả dt chính |
| `outputs/comparison_straight.png` | Đồ thị đối chiếu đường thẳng |
| `outputs/comparison_s_curve.png` | Đồ thị đối chiếu chữ S |
| `outputs/comparison_corners.png` | Đồ thị đối chiếu góc ngoặt |
| `outputs/lookahead_sweep.png` | Độ nhạy look-ahead |
| `outputs/timestep_check.png` | Độ nhạy chu kỳ điều khiển |
| `outputs/comparison_corners.gif` | Hai robot chạy trên cùng trục thời gian mô phỏng |
| `outputs/experiment_manifest.json` | Ma trận thử nghiệm, phiên bản và SHA-256 mã |
| `outputs/runs/<cấu_hình>/` | CSV đường, CSV trạng thái/lệnh và JSON kết quả cho mỗi cấu hình |
| `outputs/single_runs/<phương_pháp>/<tuyến>/` | Kết quả khi gọi riêng `run_robot.py` |

GIF lấy 100 mốc thời gian và phát ở 12 hình/giây, thời lượng phát đã nén. Hai khung cùng một thời điểm mô phỏng; robot tới đích trước đứng yên trong các khung còn lại. Các hình không phải dữ liệu thực nghiệm.

Các tệp cùng tên bị ghi đè khi chạy lại. Nếu tự thay tham số rồi muốn so sánh nhiều cấu hình, lưu riêng đầu ra trước khi chạy tiếp. `run_evaluation.py` có ma trận cố định để tái lập đúng bộ thử nghiệm bàn giao.

## Kiểm chứng

```powershell
python -m unittest -v
```

12 bài kiểm tra gồm các phép kiểm động học và điều kiện kết thúc ở bước 2, hồi quy số liệu của bản cơ sở, giới hạn |v·omega| lấy từ tốc độ bánh trên ba tuyến và từ chối tên phương pháp sai. Kiểm tra động học có nghiệm giải tích cho thẳng, quay tại chỗ, cung tròn. Các trường hợp chức năng không thay thế kiểm chứng robot thực.

## Nội dung kế thừa, phần làm thêm và trích dẫn

Pure Pursuit dựa trên hình học Coulter [1], động học vi sai theo [2]. Bước 1 đã tái lập mã PythonRobotics [4]; bước 2 viết lại mô hình và bộ chạy. Bước 3 giữ cấu trúc bước 2, bổ sung luật chọn vận tốc nêu trên, ma trận thử nghiệm, các chỉ tiêu, đồ thị đối chiếu, kiểm tra hồi quy và phân tích.

ChatGPT hỗ trợ viết mã, tài liệu và xử lý kết quả. Người thực hiện cần chạy lại, hiểu mô hình và kiểm tra số liệu trước khi đưa vào báo cáo. `Ket_qua_danh_gia.md` là bản ghi kỹ thuật, chưa phải bản báo cáo cuối kỳ theo mẫu 5 chương.

[1] R. C. Coulter, “Implementation of the Pure Pursuit Path Tracking Algorithm,” CMU-RI-TR-92-01, Jan. 1992. [Online]. Available: https://publications.ri.cmu.edu/implementation-of-the-pure-pursuit-path-tracking-algorithm. Accessed: Nov. 17, 2026.

[2] F. Dellaert and S. Hutchinson, “5.2. Motion Model for the Differential Drive Robot,” *Introduction to Robotics and Perception*. [Online]. Available: https://www.roboticsbook.org/S52_diffdrive_actions.html. Accessed: Nov. 17, 2026.

[3] S. Macenski, S. Singh, F. Martin, and J. Gines, “Regulated Pure Pursuit for Robot Path Tracking,” 2023, arXiv:2305.20026. [Online]. Available: https://arxiv.org/abs/2305.20026. Accessed: Nov. 17, 2026.

[4] A. Sakai, G. Jacquenot, and contributors, “Pure pursuit path tracking simulation,” *PythonRobotics*, commit ef64608ceb0012f2a0b3d532fddc80243e807cfa. [Online]. Available: https://github.com/AtsushiSakai/PythonRobotics/blob/ef64608ceb0012f2a0b3d532fddc80243e807cfa/PathTracking/pure_pursuit/pure_pursuit.py. Accessed: Nov. 17, 2026.
