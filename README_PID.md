# Tóm Tắt PID & Ứng Dụng Trong Dự Án KT-Robot

Tài liệu gọn gàng phục vụ ôn thi tự luận và vấn đáp: trình bày **định nghĩa PID**, bản chất từng khâu **$K_p, K_i, K_d$** (và $T_i, T_d, T_{ds}$) cùng câu trả lời chuẩn xác cho câu hỏi **"Dự án KT-Robot ứng dụng PID ở điểm nào?"**.

---

## 1. Điều khiển vòng kín (Feedback Control) là gì?

Để điều khiển một hệ thống (*Process*) hoạt động đúng mong muốn, ta cần đủ **3 thành phần**:

- **SP (Setpoint — Giá trị đặt):** mức mong muốn (ví dụ: tốc độ đặt $5\;\text{rad/s}$, mực nước $10\;\text{m}$, nhiệt độ $60^\circ\text{C}$).
- **MV / PV (Measured / Process Variable — Giá trị đo):** giá trị thực tế đo được từ cảm biến (ví dụ: tốc độ thực đo từ encoder).
- **$u$ (Control Input — Tín hiệu điều khiển):** tín hiệu bộ điều khiển xuất ra để tác động vào hệ thống (ví dụ: xung PWM / điện áp cấp cho động cơ, độ mở van).

```text
         +           e(t)               u(t)
  SP ----->( Σ )----------> [ Bộ PID ] ----------> [ Quá trình ] ----+----> PV (Đầu ra)
             ^ -                                                     |
             |______________________ MV (Cảm biến) __________________|
```

- **Sai số điều khiển:** $e(t) = \text{SP} - \text{MV}$.
- **Tại sao cần hồi tiếp vòng kín?** Vì hệ thống thực luôn chịu nhiễu bên ngoài (tải trọng, ma sát, gió, sụt áp pin…) và ta không biết chính xác 100% mô hình hệ thống. Vòng kín giúp liên tục đo lại đầu ra thực để tự động sửa sai lệch.

---

## 2. Định nghĩa PID & Bản chất các hệ số $K_p, K_i, K_d$

**PID** (*Proportional – Integral – Derivative*) là bộ điều khiển kết hợp **3 thành phần** ứng với 3 mốc thời gian: **Hiện tại (P) – Quá khứ (I) – Tương lai (D)**.

### 2.1. Khâu P — Tỷ lệ (*Proportional* — Hiện tại)
- **Công thức:** $u_P(t) = K_p \cdot e(t)$
- **Ý nghĩa:** Sai số ngay lúc này càng lớn thì tín hiệu sửa càng mạnh.
- **2 nhược điểm nếu chỉ dùng P thuần:**
  1. **Gây dao động (*Oscillations*):** Giống **con lắc đơn** chịu trọng lực kéo về điểm cân bằng — khi đi qua $e = 0$ nó vẫn còn vận tốc nên lao vọt qua bên kia và dao động mãi.
  2. **Còn sai số xác lập (*Steady-State Error*):** Giống **bồn nước có van xả đều** ($20\;\text{lít/s}$). Khi nước lên đúng SP ($e = 0$) thì $u_P = K_p \cdot 0 = 0$ (vòi cấp đóng tịt) nên nước bị tụt xuống; hệ chỉ dừng ở mức **thấp hơn SP** sao cho $K_p \cdot e_{\text{ss}} = 20\;\text{lít/s}$.

### 2.2. Khâu I — Tích phân (*Integral* — Quá khứ)
- **Công thức:**
  $$u_I(t) = K_i \int_0^t e(\tau)\,d\tau = \frac{K_p}{T_i}\int_0^t e(\tau)\,d\tau \qquad \left(K_i = \frac{K_p}{T_i}\right)$$
- **Ý nghĩa:** Cộng dồn toàn bộ sai số quá khứ (giống "con heo đất" tích lũy dần). Nhờ có "trí nhớ", ngay cả khi đầu ra chạm đúng SP ($e = 0$), giá trị tích phân đã tích lũy vẫn giữ nguyên $u_I \ne 0$ để chống tải ngoài $\Rightarrow$ **triệt tiêu sai số xác lập về 0**.
- **Lưu ý bẫy hay sai về $T_i$:** Vì $T_i$ nằm ở **mẫu số**, nên **tăng $T_i$ làm khâu I yếu đi** (giảm vọt lố, hệ êm hơn nhưng đáp ứng chậm hơn); ngược lại **giảm $T_i$** (hoặc **tăng $K_i$**) làm khâu I mạnh lên (dễ vọt lố, dao động hơn).

### 2.3. Khâu D — Đạo hàm (*Derivative* — Tương lai)
- **Công thức:**
  $$u_D(t) = K_d \frac{de(t)}{dt} = K_p T_d \frac{de(t)}{dt} \qquad \left(K_d = K_p \cdot T_d\right)$$
- **Ý nghĩa:** Kiểm tra **độ dốc (tốc độ thay đổi)** của sai số để dự đoán xu hướng tương lai và hãm phanh sớm, giúp **giảm dao động và giảm độ vọt lố**.
  - *Ví dụ hệ thống treo ô tô:* **Lò xo = khâu P** (kéo bánh về vị trí đặt), **Bộ giảm chấn / phuộc nhún = khâu D** (chống lại vận tốc nảy để dập dao động).
- **Nhược điểm & Bộ lọc thông thấp ($T_{ds}$):** Cảm biến thật luôn có nhiễu rung nhanh (độ dốc rất lớn). Khâu D thuần sẽ **khuếch đại nhiễu đo** làm tín hiệu $u$ giật mạnh (hại động cơ/van), nên trong thực tế phải gắn thêm **bộ lọc thông thấp** có tham số $T_{ds}$.

---

## 3. Công thức PID tổng hợp (Miền thời gian & Miền Laplace)

- **Dạng song song (miền thời gian):**
  $$u(t) = K_p e(t) + K_i \int_0^t e(\tau)\,d\tau + K_d \frac{de(t)}{dt}$$
- **Dạng chuẩn ISA (miền thời gian — chú ý số hạng đầu là $e(t)$, không phải `1`):**
  $$u(t) = K_p \left( e(t) + \frac{1}{T_i}\int_0^t e(\tau)\,d\tau + T_d \frac{de(t)}{dt} \right)$$
- **Dạng chuẩn ISA (miền Laplace có lọc khâu D — số hạng đầu là `1`):**
  $$\frac{U(s)}{E(s)} = K_p \left( 1 + \frac{1}{T_i s} + \frac{T_d s}{1 + \frac{T_d}{T_{ds}}s} \right)$$

---

## 4. Ảnh hưởng của $K_p, K_i, K_d$ & Cách chọn P / PI / PD / PID

| Khi tăng độc lập | Thời gian tăng (*Rise Time*) | Độ vọt lố (*Overshoot*) | Thời gian xác lập (*Settling Time*) | Sai số xác lập (*Steady-State Error*) | Độ ổn định |
|---|---|---|---|---|---|
| **Tăng $K_p$** | Giảm (nhanh hơn) | Tăng | Tăng nhẹ | Giảm (chưa về 0) | Giảm |
| **Tăng $K_i$** *(giảm $T_i$)* | Giảm nhẹ | Tăng | Tăng | **Triệt tiêu về 0** | Giảm |
| **Tăng $K_d$** *(tăng $T_d$)* | Giảm nhẹ | **Giảm rõ rệt** | Giảm | Hầu như không đổi | **Cải thiện** *(nhưng nhạy nhiễu)* |

**Chọn cấu trúc nhanh:**
- **PI** *(phổ biến nhất)*: cần sai số xác lập = 0 và tín hiệu có nhiễu (tốc độ động cơ, lưu lượng, áp suất).
- **PD:** cần phản ứng nhanh, giảm vọt lố/dao động (vị trí khớp servo).
- **PID:** hệ quán tính lớn, vừa cần hết sai số tĩnh vừa cần ít vọt lố, cảm biến sạch.
- **P:** hệ đơn giản hoặc bản thân đối tượng đã có khâu tích phân.

---

## 5. Dự án KT-Robot ứng dụng PID ở điểm nào?

### Trả lời ngắn gọn (1 câu chốt)
> Dự án **KT-Robot** hiện tại là mô phỏng **động học** bám đường bằng Pure Pursuit nên **chưa dùng bộ PID đầy đủ cả 3 khâu (P–I–D)**, mà **chỉ ứng dụng khâu P (Proportional)** ở **3 điểm cụ thể trong code**; còn **PI/PID đầy đủ** sẽ được ứng dụng ở **vòng trong điều khiển 2 động cơ bánh trái và bánh phải** khi triển khai lên robot thực tế.

### 3 điểm cụ thể đang ứng dụng khâu P trong mã nguồn
1. **Điều khiển vận tốc xe ở ví dụ gốc PythonRobotics (Giai đoạn 1):**
   - Công thức: $a = K_p \cdot (v_{\text{target}} - v)$ với `Kp = 1.0` *(tác giả gốc ghi là "PID speed control" nhưng thực tế chỉ dùng khâu P, tức $K_i = K_d = 0$)*.
   - Tệp: `pure_pursuit_baseline/vendor/PythonRobotics/PathTracking/pure_pursuit/pure_pursuit.py`.
2. **Điều khiển vận tốc góc khi căn hướng tại chỗ (Mode 1 — Heading Alignment):**
   - Khi góc lệch mục tiêu vượt $75^\circ$, robot dừng tiến ($v = 0$) và xoay tại chỗ theo luật P: $\omega_{\text{cmd}} = K_{\text{heading}} \cdot \alpha$ với `heading_gain = 2.0`.
   - Tệp: `pure_pursuit_evaluation/tracking.py` và `simulink/pp_command.m`.
3. **Giảm vận tốc tỷ lệ với khoảng cách khi tiếp cận đích (Goal Approach):**
   - Khi vào vùng cuối tuyến ($\le 0.55\;\text{m}$), vận tốc tiến giảm theo khoảng cách còn lại: $v_{\text{cmd}} = \min(v_{\text{base}},\; K_{\text{goal}} \cdot d_{\text{goal}})$ với `goal_gain = 1.0`.
   - Tệp: `pure_pursuit_evaluation/tracking.py` và `simulink/pp_command.m`.

### Liên hệ Pure Pursuit & Vai trò của I, D khi lên robot thật
- **Pure Pursuit ($\omega = \frac{2v}{L_d^2}y_{\text{body}}$)** bản chất hoạt động tương tự một **bộ điều khiển P hình học** theo khoảng nhìn trước $L_d$ ($L_d$ nhỏ $\Rightarrow$ độ lợi lớn, bám sát nhưng gắt; $L_d$ lớn $\Rightarrow$ độ lợi nhỏ, mượt nhưng cắt góc nhiều hơn).
- **Lý do hiện tại chưa dùng I và D:** mô phỏng đang là động học lý tưởng (chưa xét tải trọng, ma sát động cơ), góc hướng và khoảng cách vốn có tính tích phân nên khâu P vẫn hội tụ về đích; còn khâu D dễ khuếch đại nhiễu đo (như kịch bản `paper_noise` trong mô phỏng độ bền Simulink).
- **Khi triển khai lên robot 2 bánh vi sai thực tế:** bộ điều khiển **PI/PID đầy đủ** sẽ đặt ở **vòng trong cho 2 động cơ bánh trái và bánh phải**, đọc tốc độ thực từ **encoder (MV)** so sánh với tốc độ đặt từ Pure Pursuit (**SP**) để xuất xung **PWM ($u$)** bám đúng tốc độ (kèm lọc nhiễu và chống bão hòa tích phân vì bánh giới hạn $\pm 12\;\text{rad/s}$).

### Đoạn văn mẫu chuẩn để học thuộc
> Trong dự án **KT-Robot**, hệ thống hiện tại tập trung vào mô phỏng động học bám đường bằng thuật toán Pure Pursuit nên **chưa dùng bộ PID đầy đủ cả 3 khâu**, mà **chỉ ứng dụng khâu tỷ lệ P** tại **3 điểm**: (1) điều khiển vận tốc xe ở ví dụ gốc (giai đoạn 1) với `Kp = 1.0`; (2) điều khiển vận tốc góc khi căn hướng tại chỗ nếu góc lệch quá $75^\circ$ với `heading_gain = 2.0`; và (3) giảm vận tốc tiến tỷ lệ với khoảng cách khi robot vào vùng tiếp cận đích với `goal_gain = 1.0`. Bản thân Pure Pursuit cũng hoạt động tương tự một bộ điều khiển P hình học theo khoảng nhìn trước $L_d$. Hiện tại chưa dùng khâu I và D vì mô phỏng động học chưa xét tải trọng, ma sát động cơ và khâu D dễ khuếch đại nhiễu đo. Khi triển khai lên robot hai bánh vi sai thực tế, **PI/PID đầy đủ sẽ được ứng dụng ở vòng trong cho hai động cơ bánh trái và bánh phải** (đọc encoder hồi tiếp để xuất PWM bám tốc độ đặt).
