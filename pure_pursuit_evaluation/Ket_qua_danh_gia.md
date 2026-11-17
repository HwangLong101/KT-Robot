# Kết quả mô phỏng, đánh giá — bước 3

Đây là bản ghi kết quả chạy chương trình, chưa phải báo cáo cuối kỳ hoàn chỉnh.

## Thiết kế thử nghiệm

42 cấu hình xác định: 3 tuyến × 2 phương pháp × 5 look-ahead tại dt=0,05 s (30 cấu hình), cộng 12 cấu hình kiểm tra bước thời gian dt=0,025 và 0,1 s tại look-ahead=0,55 m. Không có nhiễu, không lấy trung bình từ nhiều lần chạy ngẫu nhiên. Không có tập dữ liệu thực.

So sánh chính giữ look-ahead=0,55 m, v yêu cầu=0,5 m/s, trạng thái đầu=(0; −0,6; 0), cùng mô hình robot, giới hạn bánh, căn hướng và điều kiện đích. Chỉ thay luật chọn vận tốc: baseline hoặc v=min(v_cơ_sở, sqrt(0,12/|kappa|)). Ngưỡng 0,12 m/s² là lựa chọn thiết kế cho mô phỏng, không phải thông số đo từ robot thật.

## So sánh chính

| Tuyến | Phương pháp | Thời gian [s] | Sai số cuối [m] | RMSE theo mẫu [m] | RMSE theo quãng đường [m] | Max v·omega tuyệt đối [m/s²] |
|---|---|---:|---:|---:|---:|---:|
| straight | baseline | 17.65 | 0.0498 | 0.1468 | 0.1487 | 0.5213 |
| straight | curvature | 18.85 | 0.0479 | 0.1790 | 0.1487 | 0.1200 |
| s_curve | baseline | 24.20 | 0.0484 | 0.1589 | 0.1595 | 0.5208 |
| s_curve | curvature | 26.10 | 0.0476 | 0.1984 | 0.1596 | 0.1200 |
| corners | baseline | 28.70 | 0.0485 | 0.1206 | 0.1208 | 0.5213 |
| corners | curvature | 34.15 | 0.0493 | 0.1391 | 0.1209 | 0.1200 |

## Diễn giải

Cả hai phương pháp đạt đích trên ba tuyến. Bản giới hạn theo độ cong giảm mức yêu cầu gia tốc ngang động học, nhưng hoàn thành chậm hơn. RMSE lấy mẫu theo thời gian tăng vì robot đi chậm ở vùng sai lệch ban đầu và đoạn cua; RMSE theo quãng đường gần như giữ nguyên. Kết quả không chứng minh bám đường chính xác hơn khi chỉ giảm vận tốc trong mô hình động học lý tưởng.

Sai lệch ban đầu 0,6 m khiến robot phải quay để hội tụ ngay cả trên đường thẳng. Do đó chỉ tiêu cực đại toàn tuyến bị ảnh hưởng mạnh bởi quá trình đầu. Cột max_lateral_demand_after_1m_m_s2 trong CSV tách thêm phần sau tiến độ 1 m; đây chỉ là mô tả bổ sung, không loại bỏ mẫu đầu khỏi RMSE chính.

RMSE theo quãng đường dùng trọng số ds=|v|dt và trung bình e² hai đầu mỗi khoảng. Nó giúp tách ảnh hưởng robot đi chậm, nhưng không tính thời gian đứng quay tại chỗ. RMSE theo mẫu vẫn tính tất cả mẫu đều thời gian, kể cả sai lệch ban đầu và mẫu kết thúc.

## Khảo sát look-ahead

Các giá trị 0,30; 0,40; 0,55; 0,80; 1,00 m được công bố đầy đủ trong lookahead_sweep.csv và đồ thị. Không chọn một cấu hình khác nhau cho từng tuyến để tuyên bố cải tiến. Khảo sát cho thấy đánh đổi giữa mức cắt góc, sai số hội tụ, thời gian và yêu cầu quay. Kết quả phụ thuộc tuyến, chưa xác định một tham số tối ưu cho mọi môi trường.

## Kiểm tra độ nhạy bước thời gian

| Tuyến | Phương pháp | RMSE quãng đường dt=0,025 s [m] | dt=0,05 s [m] | dt=0,1 s [m] |
|---|---|---:|---:|---:|
| straight | baseline | 0.14862 | 0.14875 | 0.14892 |
| straight | curvature | 0.14904 | 0.14865 | 0.14791 |
| s_curve | baseline | 0.15943 | 0.15951 | 0.15884 |
| s_curve | curvature | 0.15957 | 0.15960 | 0.15913 |
| corners | baseline | 0.12093 | 0.12079 | 0.12044 |
| corners | curvature | 0.12129 | 0.12087 | 0.12004 |

Việc đổi dt đồng thời đổi chu kỳ cập nhật điều khiển; đây là kiểm tra độ nhạy rời rạc hóa, không phải chứng minh hội tụ toán học. Bảng đầy đủ ghi cả thời gian và tình trạng đạt đích.

## Giới hạn kết luận

Đại lượng |v·omega| là yêu cầu gia tốc ngang suy ra từ động học; không mô phỏng lực bám, trượt bánh, động lực học động cơ hay tải trọng. Không dùng kết quả để kết luận đã giảm trượt bánh hoặc tăng độ an toàn trên robot thực. Luật vận tốc là mở rộng đơn giản để học tập, không phải triển khai đầy đủ Nav2 Regulated Pure Pursuit.

## Nguồn và tái lập

Chạy `python run_evaluation.py --gif` từ thư mục dự án. Cấu hình, phiên bản thư viện và SHA-256 mã ở outputs/experiment_manifest.json; toàn bộ 42 lần chạy có reference.csv, trajectory.csv và summary.json trong outputs/runs. Hình do chương trình tạo từ dữ liệu mô phỏng thực sự đã chạy.

[1] R. C. Coulter, “Implementation of the Pure Pursuit Path Tracking Algorithm,” CMU-RI-TR-92-01, Jan. 1992. https://publications.ri.cmu.edu/implementation-of-the-pure-pursuit-path-tracking-algorithm.

[2] F. Dellaert and S. Hutchinson, “5.2. Motion Model for the Differential Drive Robot,” Introduction to Robotics and Perception. https://www.roboticsbook.org/S52_diffdrive_actions.html.

[3] S. Macenski, S. Singh, F. Martin, and J. Gines, “Regulated Pure Pursuit for Robot Path Tracking,” 2023, arXiv:2305.20026. https://arxiv.org/abs/2305.20026. Tham khảo ý tưởng điều chỉnh vận tốc; không khẳng định công thức của bản này trùng với thuật toán trong bài.

Ngày truy cập nguồn: 17/11/2026. Mã và tài liệu được phát triển với hỗ trợ ChatGPT; cần tự kiểm tra và ghi nhận vai trò AI khi đưa vào báo cáo học phần.
