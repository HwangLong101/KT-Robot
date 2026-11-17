# Hướng dẫn nắm đề tài robot hai bánh vi sai bằng Pure Pursuit

**Thành viên:** NGUYỄN HOÀNG LONG và NGUYỄN NGỌC DUY.  
**Công cụ:** Python, VS Code.  
**Tên đề tài:** Nghiên cứu và mô phỏng điều khiển bám quỹ đạo cho robot di động hai bánh chủ động bằng Pure Pursuit.

Tài liệu này giải thích đề tài và bộ mã đã bàn giao để các thành viên có thể hiểu, chạy lại và trình bày kết quả. Nên đặt tệp tại `E:\ROBOT\README_ROBOT_CHO_NHOM.md`, cạnh ba thư mục dự án. Trong VS Code, mở tệp rồi nhấn **Ctrl + Shift + V** để xem bản trình bày.

## 1. Đọc nhanh để hiểu dự án

Nhóm mô phỏng một robot có **hai bánh chủ động độc lập**. Robot được cho trước một tuyến đường và phải điều chỉnh tốc độ hai bánh để đi về tuyến, bám theo tuyến rồi dừng gần đích.

Thuật toán chính là **Pure Pursuit**: robot chọn một điểm phía trước trên đường, tính cần quay cong bao nhiêu để hướng tới điểm đó, rồi liên tục cập nhật khi di chuyển.

Quá trình thực hiện đúng ba bước:

1. **Chạy được ví dụ gốc:** tái lập ví dụ PythonRobotics.
2. **Điều chỉnh hoặc mở rộng:** chuyển mô hình xe gốc thành robot hai bánh vi sai; sau đó thêm phương án giảm tốc khi vào cua.
3. **Mô phỏng, đánh giá:** thử các tuyến đường, thay tham số và so sánh bằng số liệu.

**Ba folder lưu ba giai đoạn của cùng một đề tài.** Folder 3 là bản dùng chính khi xem kết quả và chạy đánh giá. Không phải chạy lần lượt folder 1 rồi 2 rồi 3 mỗi lần sử dụng.

Kết luận chính đã quan sát: **giảm tốc theo độ cong làm giảm yêu cầu gia tốc ngang động học nhưng tăng thời gian hoàn thành; độ chính xác bám đường tính theo quãng đường gần như không đổi.**

## 2. Đề tài giải quyết bài toán gì

### 2.1 Tình huống ứng dụng

Có thể hình dung robot vận chuyển trong nhà xưởng di chuyển qua một tuyến đã định sẵn. Ở đề tài này, nhóm nghiên cứu riêng phần **điều khiển chuyển động theo tuyến**.

Chương trình nhận:

- **Tuyến tham chiếu:** các điểm tọa độ, được nối bằng đoạn thẳng.
- **Tư thế đầu:** robot đang ở đâu và quay theo hướng nào.
- **Thông số robot:** bán kính bánh, khoảng cách hai bánh và giới hạn tốc độ.
- **Thông số điều khiển:** vận tốc yêu cầu, look-ahead và bước thời gian.

Chương trình tính:

- Vận tốc tiến và vận tốc quay của robot.
- Tốc độ góc bánh trái và bánh phải.
- Vị trí, hướng, sai số theo thời gian và kết quả đạt đích.

### 2.2 Phạm vi hiện tại

Đây là **mô phỏng động học trên mặt phẳng**, giả thiết biết chính xác tư thế, bánh không trượt và tốc độ bánh đáp ứng ngay với lệnh. Chưa có phần cứng, động lực học động cơ, cảm biến, nhiễu định vị hoặc vật cản.

Tên đề tài dùng cụm “bám quỹ đạo”, nhưng mã hiện tại thực hiện **bám đường hình học**: robot đi theo tuyến cho trước, không bị yêu cầu phải tới từng vị trí vào đúng một thời điểm định sẵn.

Các kết quả do đó dùng để phân tích bộ điều khiển trong mô hình này, chưa chứng minh robot thật có thể tự vận hành trong mọi môi trường nhà xưởng.

## 3. Robot vi sai hoạt động thế nào

Hai bánh được điều khiển độc lập:

| Tốc độ hai bánh | Chuyển động của robot |
|---|---|
| Cùng tốc độ, cùng chiều | Đi thẳng |
| Bánh phải tiến nhanh hơn bánh trái | Quay trái khi đang tiến |
| Bánh trái tiến nhanh hơn bánh phải | Quay phải khi đang tiến |
| Hai bánh ngược chiều, cùng độ lớn | Quay tại chỗ |

Điểm đại diện của robot là **trung điểm trục nối hai bánh**, không phải đầu xe hay tâm của một bánh.

Các ký hiệu:

| Ký hiệu | Ý nghĩa | Đơn vị |
|---|---|---|
| `x, y` | Tọa độ trung điểm trục bánh | m |
| `theta` | Góc hướng robot; dương ngược chiều kim đồng hồ | rad |
| `v` | Vận tốc tiến tại trung điểm trục | m/s |
| `omega` | Tốc độ quay thân robot | rad/s |
| `r` | Bán kính mỗi bánh | m |
| `b` | Khoảng cách tâm hai bánh | m |
| `left, right` | Tốc độ góc bánh trái và phải | rad/s |

Động học thuận cho biết robot chuyển động thế nào từ tốc độ hai bánh:

```text
v     = r * (right + left) / 2
omega = r * (right - left) / b
```

Động học ngược tính tốc độ bánh từ lệnh chuyển động của robot:

```text
left  = (v - b * omega / 2) / r
right = (v + b * omega / 2) / r
```

Mô hình chuyển động:

```text
x_dot     = v * cos(theta)
y_dot     = v * sin(theta)
theta_dot = omega
```

Trong mỗi bước thời gian, mã giữ lệnh không đổi và tích phân chuyển động theo cung tròn chính xác; khi `omega` gần 0 thì dùng nhánh đi thẳng. Cơ sở động học được tham khảo từ [2].

## 4. Thuật toán Pure Pursuit

### 4.1 Ý tưởng dễ hiểu

Tưởng tượng robot liên tục “đuổi theo” một điểm nằm phía trước trên tuyến. Điểm đó được cập nhật dọc tuyến khi robot tiến lên. Robot không quay thẳng tức thời tới toàn bộ đường mà tính một **độ cong yêu cầu** dựa trên vị trí của điểm đang đuổi theo.

Một chu kỳ thực hiện:

1. Đọc vị trí và hướng hiện tại.
2. Xác định tiến độ dọc tuyến.
3. Chọn điểm nhìn trước trên tuyến.
4. Đổi tọa độ điểm đó sang hệ gắn với robot.
5. Tính độ cong `kappa` và lệnh `v, omega`.
6. Quy đổi sang tốc độ hai bánh, áp dụng giới hạn.
7. Cập nhật tư thế, ghi dữ liệu và kiểm tra đích.

Với trục dọc thân robot hướng tiến và trục ngang hướng trái:

```text
kappa = 2 * y_target_body / distance_to_target^2
omega = v * kappa
```

`y_target_body` là tọa độ ngang của điểm mục tiêu trong hệ robot. Khoảng cách trong mẫu số là khoảng cách thực tới điểm đã chọn. Độ cong có đơn vị `1/m`. Nguyên lý hình học tham khảo từ Coulter [1].

### 4.2 Look-ahead là gì

**Look-ahead là khoảng nhìn trước dùng để chọn điểm mục tiêu**, không phải sai số bám đường và cũng không phải khoảng cách tới đích.

- Look-ahead nhỏ: robot nhìn gần, phản ứng đổi hướng mạnh hơn; trên các tuyến đã thử thường bám sát đoạn đường hơn.
- Look-ahead lớn: robot nhìn xa hơn, có thể chuyển hướng sớm và cắt góc nhiều hơn.
- Không có giá trị tối ưu chung được chứng minh cho mọi đường.

Bản robot cơ sở dùng look-ahead cố định **0,55 m**. Chương trình thường tìm giao điểm phía trước của đường với vòng tròn bán kính look-ahead. Khi không có giao điểm phù hợp hoặc khi gần cuối tuyến, mã có nhánh chọn điểm thay thế; khoảng cách thực tới mục tiêu lúc đó có thể khác 0,55 m.

## 5. Nhóm đã điều chỉnh và mở rộng những gì

### 5.1 Điều chỉnh để đúng đề tài

Ví dụ gốc PythonRobotics là **xe có góc lái**, không phải robot hai bánh vi sai. Bản điều chỉnh thay đổi các nội dung sau:

| Nội dung | Ví dụ gốc | Bản robot của nhóm |
|---|---|---|
| Mô hình | Xe có góc lái | Robot hai bánh độc lập |
| Điểm tham chiếu điều khiển | Điểm phía sau tâm xe | Trung điểm trục hai bánh |
| Lệnh quay | Góc lái | Vận tốc góc thân robot |
| Điều khiển bánh | Không phải đầu ra bánh trái/phải của robot vi sai | Tính và giới hạn tốc độ từng bánh |
| Điều kiện kết thúc | Điểm nhìn trước tới cuối tuyến | Robot gần đích và đã vào vùng cuối tuyến |

**Việc chuyển mô hình này đã là phần điều chỉnh theo yêu cầu học phần.** Bản robot được viết lại từ nguyên lý; không chỉ đổi hình xe trong hoạt ảnh.

Các xử lý được bổ sung cho robot:

- Giới hạn tốc độ bánh và tốc độ quay thân; giảm đồng tỷ lệ lệnh để giữ độ cong yêu cầu.
- Quay tại chỗ để căn hướng nếu mục tiêu lệch quá 75°.
- Giảm tốc khi tiếp cận đích.
- Xác nhận thành công khi khoảng cách đích không quá 0,05 m và tiến độ đã vào vùng cuối tuyến.
- Đánh dấu `timeout` nếu hết 120 s mà chưa đạt; không coi việc chương trình dừng là đã thành công.

### 5.2 Mở rộng giảm tốc theo độ cong

Folder 3 có hai lựa chọn chạy **trên cùng robot vi sai**:

| Tên trong mã | Cách chọn vận tốc |
|---|---|
| `baseline` | Vận tốc yêu cầu danh định; vẫn có giới hạn bánh và giảm tốc tới đích |
| `curvature` | Thêm luật giảm vận tốc khi độ cong yêu cầu lớn |

Luật mở rộng:

```text
v_curvature = min(v_base, sqrt(a_limit / abs(kappa)))
omega       = v_curvature * kappa
```

Mã dùng một số dương rất nhỏ trong mẫu số để tránh chia cho 0. Khi đường gần thẳng, vận tốc giữ theo bản cơ sở. Khi độ cong lớn, robot được yêu cầu đi chậm hơn. Cùng khối giới hạn bánh vẫn được áp dụng sau đó.

`a_limit = 0,12 m/s²` được chọn cho mô phỏng vì:

```text
Yêu cầu gia tốc ngang động học = abs(v * omega) = v^2 * abs(kappa)
```

Luật này nhằm khống chế đại lượng trên. **Không kết luận rằng nó chắc chắn giảm sai số bám đường, giảm trượt bánh hay bảo đảm an toàn trên robot thật.** Mã hiện tại chưa mô phỏng lực bám hoặc trượt bánh.

Ý tưởng điều chỉnh vận tốc có tham khảo hướng nghiên cứu Regulated Pure Pursuit [4], nhưng luật đơn giản này **không phải bản triển khai đầy đủ Nav2 RPP**.

## 6. Giải thích các folder

| Folder | Mục đích | Khi nào dùng |
|---|---|---|
| `pure_pursuit_baseline` | Lưu và tái lập ví dụ nguồn mở PythonRobotics theo phiên bản cố định | Hiểu ví dụ gốc và chứng minh đã chạy được trước khi sửa |
| `pure_pursuit_diffdrive` | Lưu bản robot vi sai cơ sở cùng ba tuyến thử | Giải thích quá trình chuyển mô hình và kiểm tra robot đúng đề tài |
| `pure_pursuit_evaluation` | Chứa cả robot cơ sở, phương án giảm tốc và bộ đánh giá 42 cấu hình | **Folder chính để chạy, so sánh và lấy số liệu** |
| `baocao` | Nơi đặt bản Word và PDF báo cáo đã bàn giao | Đọc nội dung học thuật và chuẩn bị nộp; tự đặt các tệp tải về vào đây |

**Dễ nhầm:** tên folder `pure_pursuit_baseline` chỉ **ví dụ xe gốc**. Trong folder 3, tùy chọn `--method baseline` chỉ **robot vi sai cơ sở**. Hai chữ “baseline” này thuộc hai ngữ cảnh khác nhau.

Folder 3 chạy độc lập sau khi cài thư viện, không nhập mã từ folder 1 hoặc 2. Giữ folder 1 và 2 để lưu lịch sử phát triển và đối chiếu phần kế thừa; không cần ghép ba folder thành một folder duy nhất.

### Các tệp chính trong folder 2 và 3

| Tệp | Vai trò |
|---|---|
| `model.py` | Động học thuận/ngược, giới hạn tốc độ và cập nhật tư thế |
| `tracking.py` | Chọn điểm mục tiêu, Pure Pursuit, căn hướng, dừng và tính chỉ tiêu |
| `scenarios.py` | Định nghĩa đường thẳng, chữ S, góc ngoặt và tư thế đầu |
| `run_robot.py` | Chạy một phương án, xem hoạt ảnh hoặc xuất kết quả |
| `run_evaluation.py` | **Chỉ có ở folder 3**; chạy ma trận so sánh và khảo sát tham số |
| `test_verification.py` | Kiểm tra động học và các điều kiện chức năng |
| `requirements.txt` | Phiên bản thư viện cần cài |
| `.vscode/launch.json` | Cấu hình chạy bằng F5 khi mở đúng folder dự án |

Folder 1 có `vendor/PythonRobotics/` giữ mã gốc, `source_manifest.json` ghi nguồn và mã kiểm tra, cùng `run_baseline.py` để gọi mô phỏng gốc và xuất dữ liệu. Commit đã sử dụng: `ef64608ceb0012f2a0b3d532fddc80243e807cfa` [3].

## 7. Những thông số đang dùng

Đây là **tham số đầu vào**, cần phân biệt với các **chỉ tiêu đầu ra** ở mục 9.

| Thông số của robot vi sai | Giá trị danh định | Ý nghĩa |
|---|---:|---|
| Bán kính bánh `r` | 0,05 m | Dùng đổi tốc độ góc bánh sang vận tốc tiếp tuyến |
| Khoảng cách hai bánh `b` | 0,30 m | Quyết định quan hệ chênh lệch tốc độ bánh với tốc độ quay thân |
| Vận tốc tiến yêu cầu | 0,50 m/s | Mức mong muốn; vận tốc áp dụng có thể thấp hơn do các giới hạn |
| Look-ahead | 0,55 m | Bán kính chọn điểm nhìn trước trong so sánh chính |
| Chu kỳ điều khiển `dt` | 0,05 s | Cập nhật lệnh 20 lần mỗi giây mô phỏng |
| Tốc độ góc mỗi bánh tối đa | 12 rad/s | Giới hạn cơ cấu chấp hành mô phỏng |
| Tốc độ quay thân tối đa | 2 rad/s | Giới hạn quay của thân robot |
| Ngưỡng tới đích | 0,05 m | Khoảng cách chấp nhận ở cuối tuyến |
| Ngưỡng yêu cầu gia tốc ngang | 0,12 m/s² | Chỉ tác động đến luật chọn vận tốc `curvature` |
| Thời gian tối đa mỗi lần chạy | 120 s | Quá thời gian mà chưa đạt đích thì báo thất bại |

### Vì sao hình có chấm xuất phát lệch đường

Trong các thử nghiệm robot vi sai, tư thế đầu là **`(x, y, theta) = (0; -0,6; 0)`**. Trên đường thẳng `y = 0`, robot ban đầu cách tuyến **0,6 m**. Đây là điều kiện cố ý để kiểm tra khả năng đưa robot về đường.

- Chấm xuất phát: robot chưa nằm trên tuyến.
- Đường tham chiếu: đường mong muốn robot bám.
- Đường robot chạy: quá trình hội tụ rồi bám tuyến, có thể hơi vượt qua tuyến trước khi hiệu chỉnh.
- Điểm đích: điểm cuối cần đến trong ngưỡng cho phép.

Không đổi vị trí đầu về đúng đường rồi so trực tiếp với bảng hiện tại: đó là một cấu hình thử nghiệm khác. Giá trị lệch đầu 0,6 m nói trên thuộc các bản **robot vi sai**, không phải thông số của ví dụ xe gốc ở folder 1.

## 8. Đã mô phỏng và khảo sát những gì

### 8.1 Ba tuyến đường

| Tên trong mã | Dạng đường | Mục đích |
|---|---|---|
| `straight` | Từ `(0; 0)` đến `(8; 0)` m | Xem robot hội tụ từ vị trí lệch tuyến rồi đi thẳng |
| `s_curve` | `y = sin(2*pi*x/10)`, x từ 0 đến 10 m | Xem robot bám tuyến có độ cong thay đổi |
| `corners` | `(0;0) → (4;0) → (4;3) → (8;3) → (8;6)` m | Xem khả năng đổi hướng, cắt góc và tác dụng của giảm tốc |

Không bắt robot dừng tại từng waypoint trung gian hoặc đạt một góc hướng cuối cụ thể. Thành công được kiểm tra ở cuối tuyến.

### 8.2 Vì sao có 42 cấu hình

| Nhóm thử nghiệm | Phép tính | Số cấu hình khác nhau |
|---|---|---:|
| Khảo sát look-ahead | 3 tuyến × 2 phương án × 5 look-ahead, tại `dt=0,05 s` | 30 |
| Thêm kiểm tra chu kỳ | 3 tuyến × 2 phương án × 2 bước thời gian bổ sung, tại look-ahead 0,55 m | 12 |
| **Tổng cộng** | **30 + 12** | **42** |

Năm look-ahead: **0,30; 0,40; 0,55; 0,80; 1,00 m**. Hai bước thời gian bổ sung: **0,025 và 0,1 s**.

Sáu cấu hình so sánh chính là **3 tuyến × 2 phương án** tại look-ahead 0,55 m và `dt=0,05 s`. Sáu cấu hình này đã nằm trong nhóm 30, **không cộng thêm lần nữa**.

Đây là 42 điều kiện mô phỏng xác định, không phải 42 robot hay 42 lần chạy ngẫu nhiên. Không dùng con số này để tuyên bố độ tin cậy thống kê.

## 9. Các chỉ tiêu đánh giá và cách hiểu

### 9.1 Bảng chỉ tiêu

| Tên trong dữ liệu | Đơn vị | Đánh giá điều gì | Cách đọc |
|---|---|---|---|
| `reached_goal` | True/False | Có thỏa điều kiện đích không | `True` mới là đạt; cần xem cùng sai số cuối và thời gian |
| `final_goal_error_m` | m | Khoảng cách robot tới điểm đích khi kết thúc | Nhỏ hơn hoặc bằng 0,05 m là phần điều kiện về khoảng cách |
| `time_s` | s | Thời gian mô phỏng đến lúc kết thúc | Khi đã đạt đích, nhỏ hơn nghĩa là hoàn thành nhanh hơn; không phải thời gian máy tính chạy |
| `path_rmse_m` | m | Sai số bám đường tổng hợp theo các mẫu thời gian | Thường nhỏ hơn là bám sát hơn theo cách lấy mẫu này; chịu ảnh hưởng robot đi chậm ở vùng sai lệch |
| `distance_weighted_rmse_m` | m | Sai số bám đường có trọng số quãng đường | Giúp so độ chính xác hình học khi hai robot có vận tốc khác nhau; không tính thời gian đứng quay tại chỗ |
| `path_max_error_m` | m | Sai lệch lớn nhất trong cả hành trình | Nhỏ hơn thường tốt hơn; trong sáu lần chính đều bằng 0,6 m do sai lệch đầu |
| `max_lateral_demand_m_s2` | m/s² | Cực đại `abs(v*omega)` | Nhỏ hơn nghĩa là yêu cầu gia tốc ngang động học thấp hơn; chưa phải bằng chứng giảm trượt thực tế |
| `max_lateral_demand_after_1m_m_s2` | m/s² | Cực đại trên phần có tiến độ dọc tuyến từ 1 m trở đi | Hỗ trợ tách ảnh hưởng giai đoạn đầu; không thay chỉ tiêu toàn tuyến |
| `max_abs_wheel_speed_rad_s` | rad/s | Tốc độ bánh lớn nhất theo trị tuyệt đối | Phải không vượt 12 rad/s, có thể có sai số làm tròn máy rất nhỏ |
| `max_abs_yaw_rate_rad_s` | rad/s | Tốc độ quay thân lớn nhất theo trị tuyệt đối | Phải không vượt 2 rad/s |

### 9.2 Sai số được đo như thế nào

Mỗi mẫu có một vị trí robot. Chương trình tính khoảng cách ngắn nhất từ vị trí đó tới **các đoạn của tuyến tham chiếu**. Nó không chỉ tìm waypoint gần nhất và cũng không đo khoảng cách tới điểm nhìn trước.

Ví dụ: robot cách đường thẳng 0,1 m thì sai số bám đường là 0,1 m, dù điểm nhìn trước có thể ở cách robot khoảng 0,55 m.

### 9.3 RMSE là gì

RMSE là căn trung bình bình phương sai số. Bình phương làm những sai lệch lớn có trọng số mạnh hơn trong giá trị tổng hợp; lấy căn đưa kết quả trở lại đơn vị mét.

```text
RMSE theo mẫu = sqrt((e_1^2 + e_2^2 + ... + e_N^2) / N)
```

Ví dụ sai số ba mẫu là 0,1; 0,2; 0,3 m thì RMSE khoảng 0,216 m. Ví dụ số học này chỉ giải thích cách tính, không phải một kết quả chạy của dự án.

RMSE chính của dự án tính cả sai lệch ban đầu. Không loại các mẫu đầu chỉ để giá trị nhìn nhỏ hơn.

### 9.4 Vì sao phải có RMSE theo quãng đường

Nếu robot giảm tốc tại vùng đang lệch đường, nó sẽ ở vùng đó lâu hơn và sinh nhiều mẫu sai lệch hơn. Vì vậy, RMSE theo mẫu thời gian có thể tăng dù robot đi gần như cùng một hình dạng đường.

RMSE theo quãng đường dùng trọng số `ds = abs(v)*dt`; sai số bình phương của mỗi khoảng được lấy bằng trung bình hai đầu. Có thể hiểu là đánh giá theo **robot đã đi bao nhiêu mét**, thay vì chỉ theo **robot đã ở đó bao lâu**.

Cần đọc cả hai RMSE, thời gian và điều kiện đạt đích. Không chọn riêng chỉ tiêu thuận lợi rồi kết luận một phương án tốt hơn toàn diện.

## 10. Kết quả chính đã chạy được

Hai phương án dùng cùng robot, tuyến, tư thế đầu, giới hạn, look-ahead 0,55 m và `dt=0,05 s`.

| Tuyến | Phương án | Thời gian [s] | Sai số đích [m] | RMSE theo mẫu [m] | RMSE quãng đường [m] | Max yêu cầu ngang [m/s²] |
|---|---|---:|---:|---:|---:|---:|
| Thẳng | Cơ sở | 17,65 | 0,0498 | 0,1468 | 0,14875 | 0,5213 |
| Thẳng | Giảm tốc | 18,85 | 0,0479 | 0,1790 | 0,14865 | 0,1200 |
| Chữ S | Cơ sở | 24,20 | 0,0484 | 0,1589 | 0,15951 | 0,5208 |
| Chữ S | Giảm tốc | 26,10 | 0,0476 | 0,1984 | 0,15960 | 0,1200 |
| Góc ngoặt | Cơ sở | 28,70 | 0,0485 | 0,1206 | 0,12079 | 0,5213 |
| Góc ngoặt | Giảm tốc | 34,15 | 0,0493 | 0,1391 | 0,12087 | 0,1200 |

Cả **42/42 cấu hình** đã đạt điều kiện đích. Trên tuyến góc ngoặt danh định, bản giảm tốc:

- Giảm cực đại yêu cầu gia tốc ngang động học từ khoảng **0,5213 xuống 0,1200 m/s²**, khoảng **77%**.
- Tăng thời gian từ **28,70 lên 34,15 s**, khoảng **19%**.
- Có RMSE quãng đường gần như giữ nguyên, còn RMSE theo mẫu thời gian tăng.

Vì vậy, cách trình bày đúng là **đánh đổi giữa thời gian và mức yêu cầu khi đổi hướng**. Không nói “bản mở rộng bám đường chính xác hơn” hoặc “đã giảm trượt bánh”, vì dữ liệu hiện tại chưa chứng minh các kết luận đó.

## 11. Đầu ra là những gì và nằm ở đâu

### 11.1 Đầu ra của toàn đề tài

| Loại đầu ra | Dùng để làm gì |
|---|---|
| Mã Python và tệp thư viện | Chạy lại mô hình và thuật toán |
| Cửa sổ hoạt ảnh | Xem robot và điểm nhìn trước di chuyển |
| Ảnh PNG | Xem quỹ đạo, vận tốc, sai số và đồ thị khảo sát |
| GIF | Chia sẻ minh họa chuyển động mà không cần chạy Python |
| CSV | Đọc số liệu chi tiết, vẽ lại hoặc kiểm tra bằng Excel/Python |
| JSON | Lưu chỉ tiêu, tham số, giả thiết và thông tin môi trường |
| `Ket_qua_danh_gia.md` | Bản tổng hợp kỹ thuật tự sinh từ dữ liệu đánh giá |
| Báo cáo Word và PDF | Trình bày 5 chương, công thức, kết quả, thảo luận và tài liệu tham khảo |

### 11.2 Các đầu ra quan trọng trong folder 3

Các đường dẫn dưới đây tính từ `pure_pursuit_evaluation`.

| Đường dẫn | Nội dung |
|---|---|
| `outputs/comparison.csv` | Sáu cấu hình so sánh chính |
| `outputs/all_trials.csv` | Tổng hợp cả 42 cấu hình |
| `outputs/lookahead_sweep.csv` | 30 cấu hình khảo sát look-ahead |
| `outputs/timestep_check.csv` | 18 cấu hình kiểm tra dt tại look-ahead 0,55 m; có phần trùng với nhóm 30 |
| `outputs/comparison_straight.png` | Đồ thị đối chiếu đường thẳng |
| `outputs/comparison_s_curve.png` | Đồ thị đối chiếu chữ S |
| `outputs/comparison_corners.png` | Đồ thị đối chiếu đường góc ngoặt |
| `outputs/lookahead_sweep.png` | Độ nhạy theo khoảng nhìn trước |
| `outputs/timestep_check.png` | Độ nhạy theo chu kỳ điều khiển |
| `outputs/comparison_corners.gif` | Hai phương án trên cùng trục thời gian mô phỏng |
| `outputs/experiment_manifest.json` | Cấu hình thử, phiên bản Python/thư viện và mã SHA-256 |
| `outputs/runs/<run_id>/` | Dữ liệu riêng của từng cấu hình |
| `outputs/single_runs/<method>/<scenario>/` | Kết quả khi chạy riêng bằng `run_robot.py` |

Ví dụ `run_id = corners_curvature_L0.55_dt0.050` nghĩa là đường góc ngoặt, phương án giảm tốc, look-ahead 0,55 m và dt 0,05 s.

Mỗi thư mục `runs/<run_id>/` có:

- `reference.csv`: tọa độ tuyến tham chiếu.
- `trajectory.csv`: vị trí, hướng, lệnh bánh và sai số theo thời gian.
- `summary.json`: các chỉ tiêu tổng hợp cùng cấu hình lần chạy.

Trong `trajectory.csv`, lệnh trên dòng thời điểm `t` được áp dụng trong khoảng từ `t` tới `t+dt`; dòng cuối đặt các tốc độ về 0. `mode_code` có nghĩa: **0** bám đường, **1** căn hướng, **2** đạt đích, **3** hết thời gian.

GIF so sánh có giảm số khung hình và nén thời gian phát. Hai khung được đồng bộ theo thời gian mô phỏng, nhưng số giây xem GIF không bằng thời gian hoàn thành của robot. Lấy thời gian từ CSV/JSON hoặc dòng kết quả terminal.

## 12. Chạy lại trong VS Code trên Windows

### 12.1 Chạy bản đánh giá chính

Mở folder `pure_pursuit_evaluation` trong VS Code rồi vào **Terminal → New Terminal**. Nếu đang đứng ở thư mục khác, chuyển tới đúng thư mục trước:

```powershell
cd E:\ROBOT\pure_pursuit_evaluation
python -m pip install -r requirements.txt
python run_evaluation.py --gif
```

Nếu chương trình chạy đúng bộ cấu hình mặc định, sẽ có dòng:

```text
Completed 42 distinct trials; reached goal: 42/42
```

Muốn bỏ phần tạo GIF để chạy nhanh hơn:

```powershell
python run_evaluation.py
```

### 12.2 Xem từng robot chạy

Trong folder 3, chạy từng lệnh riêng:

```powershell
python run_robot.py --method baseline --scenario corners --animate
```

```powershell
python run_robot.py --method curvature --scenario corners --animate
```

Thay `corners` bằng `straight` hoặc `s_curve` để đổi đường. Để thử look-ahead khác:

```powershell
python run_robot.py --method baseline --scenario corners --lookahead 0.8 --animate
```

Lệnh thử riêng này không sửa ma trận cố định của `run_evaluation.py`. Nếu thay look-ahead trong lần chạy riêng, nhớ ghi lại tham số và lưu đầu ra trước khi chạy đè.

### 12.3 Kiểm tra mã

```powershell
python -m unittest -v
```

Folder 3 có **12 bài kiểm tra**, gồm chuyển động có nghiệm giải tích, giới hạn lệnh, đạt đích, hồi phục hướng, xử lý hết thời gian, từ chối tham số sai và kiểm tra bản cơ sở còn cho kết quả cũ. Các bài kiểm tra không đồng nghĩa mọi tình huống thực tế đều đã được bảo đảm.

### 12.4 Nếu cần xem lại folder 1 hoặc 2

Chạy và xuất kết quả ví dụ gốc:

```powershell
cd E:\ROBOT\pure_pursuit_baseline
python -m pip install -r requirements.txt
python run_baseline.py --gif
```

Xem robot vi sai cơ sở ở folder 2:

```powershell
cd E:\ROBOT\pure_pursuit_diffdrive
python -m pip install -r requirements.txt
python run_robot.py --animate
```

Mỗi lần chạy cần đúng thư mục và đúng Python đã cài thư viện. Môi trường đã chạy thành công trên máy nhóm là **Windows 11, Python 3.13.7, NumPy 2.3.5, Matplotlib 3.10.8, Pillow 13.0.0**; folder 1 còn dùng SciPy theo tệp thư viện của nó. Nếu dùng `.venv`, chọn đúng **Python: Select Interpreter** trong VS Code.

Các tệp đầu ra cùng tên sẽ bị ghi đè khi chạy lại. Cần lưu bản riêng nếu muốn giữ nhiều lần thử.

## 13. Khi thuyết trình, cần nắm chắc những ý nào

1. **Bài toán:** robot vi sai đi từ tư thế lệch ban đầu, bám tuyến cho trước rồi tới đích.
2. **Thuật toán:** Pure Pursuit chọn điểm phía trước, tính độ cong và đổi thành tốc độ hai bánh.
3. **Nguồn gốc:** đã chạy ví dụ xe có góc lái của PythonRobotics trước khi điều chỉnh.
4. **Phần nhóm triển khai:** mô hình vi sai, điều kiện đích, giới hạn lệnh, xử lý phụ trợ và bộ xuất dữ liệu.
5. **Phần mở rộng:** giảm vận tốc theo độ cong; không phải thuật toán tránh vật cản.
6. **Đánh giá:** đạt đích, sai số, thời gian, giới hạn vận tốc và yêu cầu gia tốc ngang; cần phân biệt tham số đầu vào với chỉ tiêu đầu ra.
7. **Kết quả:** 42/42 cấu hình đạt đích; giảm yêu cầu ngang đổi lấy thời gian lâu hơn, chưa cải thiện rõ độ chính xác hình học.
8. **Giới hạn:** mô phỏng động học lý tưởng; chưa có kiểm chứng lực bám, trượt bánh, cảm biến hoặc robot thật.

Không so trực tiếp RMSE folder 1 với folder 2 để nói thuật toán đã tốt hơn: hai giai đoạn khác mô hình, đường, vận tốc và cách kết thúc. Phép so sánh có kiểm soát nằm trong folder 3, giữa hai phương án **trên cùng robot và cùng điều kiện**.

## 14. Tài liệu tham khảo và ghi nhận

[1] R. C. Coulter, “Implementation of the Pure Pursuit Path Tracking Algorithm,” CMU-RI-TR-92-01, Jan. 1992. [Tài liệu gốc](https://publications.ri.cmu.edu/implementation-of-the-pure-pursuit-path-tracking-algorithm).

[2] F. Dellaert and S. Hutchinson, “5.2. Motion Model for the Differential Drive Robot,” *Introduction to Robotics and Perception*. [Mô hình động học](https://www.roboticsbook.org/S52_diffdrive_actions.html).

[3] A. Sakai, G. Jacquenot, and contributors, “Pure pursuit path tracking simulation,” *PythonRobotics*, commit `ef64608ceb0012f2a0b3d532fddc80243e807cfa`. [Mã ví dụ gốc](https://github.com/AtsushiSakai/PythonRobotics/blob/ef64608ceb0012f2a0b3d532fddc80243e807cfa/PathTracking/pure_pursuit/pure_pursuit.py). Giấy phép MIT được giữ trong bản tái lập ở folder 1.

[4] S. Macenski, S. Singh, F. Martin, and J. Gines, “Regulated Pure Pursuit for Robot Path Tracking,” 2023, arXiv:2305.20026. [Bài báo tham khảo hướng điều chỉnh vận tốc](https://arxiv.org/abs/2305.20026).

Ngày truy cập nguồn trong quá trình thực hiện: **17/11/2026**. ChatGPT hỗ trợ xây dựng mã, tài liệu và phân tích; thành viên cần tự chạy lại, hiểu công thức và kiểm tra kết quả trước khi trình bày hoặc nộp bài.
