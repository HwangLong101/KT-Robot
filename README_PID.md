# Tóm Tắt PID & Giải Đáp Đề Tự Luận (Liên Hệ KT-Robot)

Tài liệu tổng hợp đầy đủ định nghĩa PID, bản chất $K_p, K_i, K_d$ (với công thức chuẩn hiển thị đẹp trên GitHub), các câu hỏi tình huống hay ra thi và phần ứng dụng thực tế trong dự án **KT-Robot**.

---

## 1. Định nghĩa PID & Điều khiển vòng kín

**PID** (*Proportional – Integral – Derivative*, tức **Tỷ lệ – Tích phân – Đạo hàm**) là một bộ điều khiển hồi tiếp vòng kín dùng để điều chỉnh một đại lượng vật lý của hệ thống (*process*) nhằm đạt và duy trì đầu ra mong muốn **SP** (ví dụ: muốn xe đạt vận tốc mong muốn thì cần vặn ga bao nhiêu, hay muốn robot bám sát đường tham chiếu).

**Cách hoạt động:**
- Trong suốt quá trình vận hành, cảm biến liên tục đo trạng thái thực tế của hệ thống gọi là **tín hiệu phản hồi (MV / PV)**, rồi đưa ngược về so sánh với **tín hiệu đặt mong muốn (SP)** để tính ra **sai lệch điều khiển**:
  $$e(t) = \text{SP} - \text{MV}$$
- Nếu còn sai lệch ($e(t) \ne 0$), bộ PID xử lý $e(t)$ qua 3 khâu **P (hiện tại), I (quá khứ), D (tương lai)** rồi cộng lại thành **tín hiệu điều khiển $u(t)$** tác động vào cơ cấu chấp hành để điều chỉnh hệ thống (*process*). Quá trình đo – so sánh – sửa lỗi diễn ra liên tục thành vòng khép kín nhằm giúp hệ thống đáp ứng nhanh, bám sát SP, ít dao động và tự bù được nhiễu ngoài.

### Sơ đồ vòng phản hồi hệ thống PID

```text
                         +--------------+
                    +--->|  P: Kp·e(t)  |---+
                    |    +--------------+   |
        +     e(t)  |    +--------------+   v  (+)  u(t)   +-------------+
SP (r) --->( Σ )----+--->| I: Ki∫e(τ)dτ |-->( Σ )--------->|  Đối tượng  |---+---> Đầu ra (y)
             ^ -    |    +--------------+   ^              |  (Process)  |   |
             |      |    +--------------+   |              +-------------+   |
             |      +--->| D: Kd de/dt  |---+                                |
             |           +--------------+                                    |
             |                                       +-------------+         |
             +------------ Tín hiệu phản hồi MV <----|  Cảm biến   |<--------+
                                                     +-------------+
```

---

## 2. Phân tích 3 khâu P, I, D và các hệ số $K_p, K_i, K_d$

### 2.1. Khâu P — Tỷ lệ (*Proportional* — Xét hiện tại) & Hệ số $K_p$
- **Công thức:**
  $$u_P(t) = K_p \cdot e(t)$$
- **Cách hoạt động:** Tạo ra tín hiệu điều khiển tỷ lệ trực tiếp với sai số ngay tại thời điểm hiện tại. Sai số càng lớn thì lực sửa càng mạnh; sai số càng nhỏ thì lực sửa càng giảm; khi $e(t) = 0$ thì $u_P = 0$.
- **Vai trò của $K_p$:** Quyết định độ nhạy và tốc độ phản ứng của hệ thống:
  - **$K_p$ nhỏ:** hệ thống phản ứng chậm, mất nhiều thời gian để hội tụ về giá trị đặt SP.
  - **$K_p$ lớn:** hệ thống sửa lỗi nhanh hơn, hội tụ nhanh hơn nhưng dễ gây ra **vọt lố (*overshoot*)** nếu quá lớn, thậm chí **dao động (*oscillate*)** mạnh quanh giá trị đặt gây mất ổn định.
  - **Hạn chế:** nếu chỉ dùng P thuần thường vẫn còn **sai số xác lập (*steady-state error*)** (ví dụ: bồn nước có van xả đều, tại $e=0$ thì $u_P=0$ nên mức nước tụt xuống dưới SP).

### 2.2. Khâu I — Tích phân (*Integral* — Xét quá khứ) & Hệ số $K_i$
- **Công thức:**
  $$u_I(t) = K_i \int_0^t e(\tau)\,d\tau = \frac{K_p}{T_i}\int_0^t e(\tau)\,d\tau \qquad \left(K_i = \frac{K_p}{T_i}\right)$$
- **Cách hoạt động:** Cộng dồn toàn bộ các sai số đã xảy ra trong quá khứ theo thời gian; tổng tích phân sẽ tăng dần lên để tạo đủ lực bù đẩy đầu ra chạm đúng giá trị đặt SP.
- **Vai trò của $K_i$:** Quyết định tốc độ tích lũy để **triệt tiêu hoàn toàn sai số xác lập về 0**, giải quyết triệt để vấn đề *steady-state error*:
  - **$K_i$ nhỏ** *(tức $T_i$ lớn)*: khử sai số xác lập chậm hơn nhưng hệ êm, ít *overshoot*.
  - **$K_i$ quá lớn** *(tức $T_i$ nhỏ)*: dễ gây *overshoot* mạnh, dao động kéo dài và tăng *settling time*.

### 2.3. Khâu D — Đạo hàm (*Derivative* — Xét tương lai) & Hệ số $K_d$
- **Công thức:**
  $$u_D(t) = K_d \frac{de(t)}{dt} = K_p T_d \frac{de(t)}{dt} \qquad \left(K_d = K_p \cdot T_d\right)$$
- **Cách hoạt động:** Dựa vào tốc độ thay đổi (độ dốc) của sai số để dự đoán xu hướng trong tương lai gần. Khi thấy hệ thống đang lao rất nhanh về phía SP, khâu D lập tức tạo tín hiệu hãm ngược lại ("phanh sớm") để tránh *overshoot*.
- **Vai trò của $K_d$:** Đóng vai trò **giảm chấn (*damping*)** giúp ổn định hệ thống, hạn chế *overshoot* *(và hầu như không ảnh hưởng đến sai số xác lập)*:
  - **Tăng $K_d$ hợp lý:** giúp giảm *overshoot*, dập tắt dao động nhanh và rút ngắn *settling time*.
  - **$K_d$ quá lớn:** khuếch đại mạnh nhiễu đo từ cảm biến làm tín hiệu điều khiển rung giật liên tục *(nên thực tế thường phải thêm bộ lọc thông thấp $T_{ds}$)*, đồng thời "phanh quá gắt" khiến hệ phản ứng ì/chậm lại.

---

## 3. Công thức PID đầy đủ & Chú thích ký hiệu

- **Dạng song song (miền thời gian):**
  $$u(t) = K_p e(t) + K_i \int_0^t e(\tau)\,d\tau + K_d \frac{de(t)}{dt}$$
- **Dạng chuẩn ISA (miền thời gian):**
  $$u(t) = K_p \left( e(t) + \frac{1}{T_i}\int_0^t e(\tau)\,d\tau + T_d \frac{de(t)}{dt} \right)$$
- **Dạng chuẩn ISA trong miền Laplace (có bộ lọc thông thấp khâu D):**
  $$\frac{U(s)}{E(s)} = K_p \left( 1 + \frac{1}{T_i s} + \frac{T_d s}{1 + \frac{T_d}{T_{ds}}s} \right)$$

**Chú thích đại lượng:**
- $u(t)$: tín hiệu điều khiển đầu ra tại thời điểm $t$.
- $e(t) = \text{SP} - \text{MV}$: sai lệch điều khiển.
- $K_p, K_i, K_d$: các hệ số khuếch đại tỷ lệ, tích phân, đạo hàm.
- $T_i, T_d$: hằng số thời gian tích phân và hằng số thời gian đạo hàm (đơn vị giây).
- $T_{ds}$: tham số băng thông bộ lọc thông thấp cho khâu đạo hàm.
- $\tau$: biến thời gian lấy tích phân chạy từ $0$ đến $t$.

### Bảng tác động nhanh khi tăng độc lập $K_p, K_i, K_d$

| Khi tăng độc lập | Thời gian tăng (*Rise Time*) | Độ vọt lố (*Overshoot*) | Thời gian xác lập (*Settling Time*) | Sai số xác lập (*Steady-State Error*) | Độ ổn định |
|---|---|---|---|---|---|
| **Tăng $K_p$** | Giảm (nhanh hơn) | Tăng | Tăng nhẹ | Giảm (chưa về 0) | Giảm |
| **Tăng $K_i$** *(giảm $T_i$)* | Giảm nhẹ | Tăng | Tăng | **Triệt tiêu về 0** | Giảm |
| **Tăng $K_d$** *(tăng $T_d$)* | Giảm nhẹ | **Giảm rõ rệt** | Giảm | Hầu như không đổi | **Cải thiện** *(nhạy nhiễu)* |

---

## 4. Các câu hỏi tự luận trọng tâm hay ra thi

### Câu A: Tại sao tăng $T_i$ thì tác động tích phân (và độ vọt lố / tốc độ đáp ứng) lại giảm?
1. **Toán học:** Vì $K_i = \frac{K_p}{T_i}$, tham số **$T_i$ nằm ở mẫu số** (tỷ lệ nghịch với $K_i$). Khi giữ nguyên $K_p$ mà tăng $T_i$ thì $K_i$ giảm $\Rightarrow$ tác động tích phân yếu đi.
2. **Ý nghĩa vật lý:** $T_i$ là thời gian khâu I tích lũy để khử sai số xác lập. Tăng $T_i$ nghĩa là kéo dài thời gian tích lũy $\Rightarrow$ khâu I cộng dồn chậm và nhẹ nhàng hơn.
3. **Hệ quả:** Độ vọt lố (*overshoot*) **giảm**, hệ êm và ít dao động hơn; tốc độ đáp ứng **giảm** (về SP chậm hơn), nhưng sai số xác lập cuối cùng **vẫn bằng 0**.

### Câu B: Sau *settling time*, đầu ra đã vào dải ổn định quanh SP nhưng vẫn rung/lệch li ti rất nhỏ — gọi là lỗi gì và fix thế nào?
- **Tên gọi:** Rung giật / dao động biên độ nhỏ quanh điểm đặt (*Hunting / Chattering* trong dải xác lập).
- **Nguyên nhân:** Nhiễu cảm biến (nhất là khi khâu D khuếch đại nhiễu), độ phân giải encoder / độ rơ cơ khí (*backlash*), hoặc $K_p, K_i$ còn hơi nhạy quanh SP.
- **Cách fix:**
  1. Lọc nhiễu tín hiệu đo MV, gắn bộ lọc thông thấp $T_{ds}$ cho khâu D hoặc giảm/tắt $K_d$ (dùng PI).
  2. Thêm **vùng chết (*Deadband*)**: khi $|e(t)| < \varepsilon$ (nằm trong sai số cho phép) thì cho $e = 0$ để ngừng nhấp nhả.
  3. Hạ nhẹ $K_p$ hoặc $K_i$ (tăng $T_i$).

### Câu C: Khi tăng $K_p$ lên, xe bám đường nhanh hơn nhưng lại bị chuyển động quay tại chỗ — giải thích?
- **Bám nhanh hơn:** Vì $u_P = K_p e(t)$, tăng $K_p$ làm lệnh quay $\omega$ lớn hơn ngay cả khi lệch nhỏ $\Rightarrow$ xe bẻ lái về đường nhanh hơn.
- **Bị quay tại chỗ / lắc mạnh:** Khi $K_p$ quá lớn kết hợp quán tính và trễ, xe bị **vọt lố** qua bên kia đường rồi đổi dấu bẻ ngược hết cỡ liên tục. Đặc biệt trên robot 2 bánh vi sai ($\omega_L = \frac{v - b\omega/2}{r}$, $\omega_R = \frac{v + b\omega/2}{r}$), lệnh quay $\omega$ quá lớn làm hai bánh quay ngược chiều chạm bão hòa ($\pm 12\;\text{rad/s}$) và ép vận tốc tiến $v$ về gần 0 (hoặc kích hoạt chế độ căn hướng tại chỗ $v=0$ khi góc lệch $> 75^\circ$ trong KT-Robot) $\Rightarrow$ xe chỉ xoay vòng tại chỗ.
- **Khắc phục:** Giảm $K_p$, thêm khâu D (có lọc) để giảm chấn, giới hạn tốc độ góc và lọc nhiễu góc hướng.

### Câu D: Tại sao robot luôn bị trôi lệch so với đường tham chiếu trong thời gian dài? Vì sao chỉnh $K_i$ khắc phục được?
- **Nguyên nhân trôi lệch dài (*Steady-State Error*):** Khi robot chịu tác nhân kéo lệch liên tục (như lệch bán kính 2 bánh $\pm 2\%\text{–}4\%$, ma sát hai bên không đều, sàn nghiêng, trượt bánh hoặc bám cung cong liên tục) mà bộ điều khiển chỉ có khâu P (không có I): tại $e = 0$ thì $u_P = 0$ không chống được lực kéo lệch $\Rightarrow$ robot chỉ cân bằng khi lệch hẳn ra một khoảng $e_{\text{ss}} \ne 0$ sao cho $K_p e_{\text{ss}}$ thắng đúng lực kéo lệch kia.
- **Vì sao chỉnh $K_i$ khắc phục được:** Khâu I ($u_I = K_i \int_0^t e(\tau)\,d\tau$) cộng dồn sai số trôi lệch theo thời gian, tăng dần lực lái bù kéo robot về đúng tâm đường. Khi đã về đúng đường ($e = 0$), khâu P bằng 0 nhưng **giá trị tích phân đã tích lũy vẫn giữ $u_I \ne 0$** để liên tục bù sai lệch $\Rightarrow$ triệt tiêu trôi lệch về 0. *(Lưu ý không chỉnh $K_i$ quá lớn để tránh vọt lố và bão hòa tích phân).*

### Câu E: Làm sao biết robot bám đường tốt hay không? Nêu ít nhất 2 nguyên nhân gây mất bám (Chứng minh theo báo cáo KT-Robot)
- **Các chỉ tiêu đánh giá trong báo cáo KT-Robot:**
  1. `reached_goal` và sai số đích cuối `final_goal_error_m` ($\le 0.05\;\text{m}$).
  2. Sai số bám đường `path_rmse_m`, **`distance_weighted_rmse_m`** (RMSE theo quãng đường) và sai số lớn nhất `path_max_error_m`.
  3. Yêu cầu gia tốc ngang cực đại $|v\omega|$, giới hạn tốc độ bánh ($\le 12\;\text{rad/s}$), tốc độ quay thân ($\le 2\;\text{rad/s}$) và thời gian hoàn thành.
- **Chứng minh thực tế từ báo cáo KT-Robot (tốt nói tốt, xấu nói xấu):**
  - **Mặt bám tốt:** Trong mô phỏng danh định, **42/42 cấu hình** đạt đích ($\le 0.05\;\text{m}$); trên tuyến góc ngoặt ($L_d = 0.55\;\text{m}$), `distance_weighted_rmse_m` $\approx 0.1208\;\text{m}$ (đã tính độ lệch đầu $0.6\;\text{m}$); phương án giảm tốc theo độ cong (`curvature`) giảm 77% cực đại $|v\omega|$ (từ $0.5213$ xuống $0.1200\;\text{m/s}^2$).
  - **Mặt chưa tốt:** Khi look-ahead lớn ($0.80$–$1.00\;\text{m}$), robot bị **cắt góc** ở các cua $90^\circ$; trong 384 lượt mô phỏng độ bền Simulink, khi có nhiễu đo mạnh (`paper_noise`: $\sigma_y = 0.10\;\text{m}, \sigma_\theta = 10^\circ$) tỷ lệ đạt đích chuẩn tụt mạnh còn **14/60** (cơ sở) và **5/60** (giảm tốc) do báo đích giả (`false_goal`) và căn hướng giả; ở `combined_strong` chỉ đạt **16/60** (cơ sở) và **32/60** (giảm tốc).
- **3 nguyên nhân chính gây mất bám:**
  1. **Tham số điều khiển chưa phù hợp:** $K_p$ quá lớn (hoặc $L_d$ quá nhỏ) gây dao động, vọt lố; $K_p$ quá nhỏ (hoặc $L_d$ quá lớn) phản ứng chậm, gây cắt góc.
  2. **Nhiễu cảm biến, trễ đo và mất tín hiệu:** làm tính sai sai số $e(t)$, bẻ lái sai hướng và kích hoạt quay căn hướng giả.
  3. **Trượt bánh, lệch bán kính bánh và quán tính/bão hòa cơ cấu chấp hành:** khiến chuyển động thực của robot lệch khỏi lệnh tính toán.

---

## 5. Dự án KT-Robot ứng dụng PID ở điểm nào?

> Trong dự án **KT-Robot**, hệ thống hiện tại tập trung vào mô phỏng động học bám đường bằng thuật toán Pure Pursuit nên **chưa dùng bộ PID đầy đủ cả 3 khâu**, mà **chỉ ứng dụng khâu tỷ lệ P** tại **3 điểm**:
> 1. **Điều khiển vận tốc xe ở ví dụ gốc PythonRobotics (Giai đoạn 1):** $a = K_p (v_{\text{target}} - v)$ với `Kp = 1.0` (`pure_pursuit_baseline/.../pure_pursuit.py`).
> 2. **Điều khiển vận tốc góc khi căn hướng tại chỗ (Mode 1):** khi góc lệch vượt $75^\circ$, $\omega_{\text{cmd}} = K_{\text{heading}} \cdot \alpha$ với `heading_gain = 2.0` (`tracking.py`, `pp_command.m`).
> 3. **Giảm vận tốc tiến tỷ lệ với khoảng cách khi tiếp cận đích:** $v_{\text{cmd}} = \min(v_{\text{base}}, K_{\text{goal}} \cdot d_{\text{goal}})$ với `goal_gain = 1.0` (`tracking.py`, `pp_command.m`).
>
> Bản thân **Pure Pursuit** ($\omega = \frac{2v}{L_d^2}y_{\text{body}}$) cũng hoạt động tương tự một bộ điều khiển **P hình học** theo khoảng nhìn trước $L_d$. Hiện tại chưa dùng khâu I và D vì mô phỏng động học chưa xét tải trọng, ma sát động cơ và khâu D dễ khuếch đại nhiễu đo. Khi triển khai lên robot hai bánh vi sai thực tế, **PI/PID đầy đủ sẽ được ứng dụng ở vòng trong cho hai động cơ bánh trái và bánh phải** (đọc encoder hồi tiếp để xuất PWM bám đúng tốc độ đặt từ Pure Pursuit, kèm lọc nhiễu và chống bão hòa tích phân vì bánh giới hạn $\pm 12\;\text{rad/s}$).
