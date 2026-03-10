# Lời Nói Đầu

---

## Vì Sao Cuốn Sách Này Ra Đời

Mỗi học kỳ, tôi lại chứng kiến một cảnh quen thuộc: một lớp sinh viên kỹ thuật thông minh, thành thạo Python, quen với MATLAB, nhưng lại gặp khó khăn khi kết nối giữa toán học đẹp đẽ của biến đổi Laplace, biểu đồ Bode với mã C++ nhúng chạy trên vi điều khiển mà các bạn sẽ lập trình sau khi tốt nghiệp. Khoảng cách giữa lý thuyết sách giáo khoa và thực tiễn công nghiệp không chỉ là vấn đề sư phạm — mà là vấn đề ngôn ngữ. Ngôn ngữ của lý thuyết điều khiển là toán học; ngôn ngữ của triển khai là mã lệnh. Hầu hết các giáo trình chỉ nói ngôn ngữ đầu tiên. Cuốn sách này nói cả hai.

**Kỹ Thuật Điều Khiển Hiện Đại: Lý Thuyết và Triển Khai C++** ra đời từ hai niềm tin:

1. **Bạn sẽ không thực sự hiểu một hàm truyền cho đến khi tự xây dựng nó bằng mã, vẽ đáp ứng của nó, và chứng kiến nó mất ổn định khi bạn thay đổi một hệ số.** Các phép biến đổi toán học là cần thiết nhưng chưa đủ. Một sinh viên có thể chứng minh ổn định Lyapunov nhưng không thể lập trình bộ lọc Kalman trên phần cứng thực tế thì mới chỉ học được một nửa ngành điều khiển.

2. **C++ là ngôn ngữ của điều khiển thời gian thực.** Dù MATLAB và Python không thể thiếu cho thiết kế và phân tích, phần lớn các bộ điều khiển thực tế — từ ECU ô tô đến PLC công nghiệp, từ máy bay không người lái đến DSP trạm gốc 5G — đều chạy mã C hoặc C++ biên dịch. Sinh viên học lý thuyết điều khiển bằng C++ ngay từ đầu sẽ phát triển trực giác về chi phí tính toán, độ chính xác số và ràng buộc thời gian thực — những điều chuyển hóa trực tiếp vào nghề nghiệp sau này.

---

## Điều Gì Làm Cuốn Sách Này Khác Biệt

### "Cơ chế", Không Phải "Công Thức"

Sách này dạy **cơ chế**, không phải **công thức**.

Sự khác biệt là căn bản. Cách "công thức" nói: *"Bước 1: tính sai lệch. Bước 2: nhân với hệ số khuếch đại. Bước 3: áp dụng cho đối tượng."* Sinh viên có thể làm đúng mà không hiểu gì. Họ học được một quy trình, nhưng không thể thích nghi.

Cách "cơ chế" nói: *"Phản hồi hoạt động vì tín hiệu sai lệch mang thông tin về tác động của nhiễu lên đầu ra. Bộ điều khiển không nhìn thấy nhiễu trực tiếp — nó chỉ quan sát hệ quả của nhiễu qua sai lệch. Bằng cách tác động lên thông tin này, bộ điều khiển bù cho những gì nó không đo được."* Sinh viên hiểu cơ chế này có thể thiết kế hệ thống chưa từng gặp.

Bảng dưới đây minh họa sự khác biệt này:

| "Công thức" (chúng tôi tránh)                  | "Cơ chế" (chúng tôi dạy)                                 |
|-------------------------------------------------|----------------------------------------------------------|
| "Vẽ biểu đồ Bode theo các bước này"            | "Biểu đồ Bode cho thấy HỆ THỐNG lọc tần số ra sao — và VÌ SAO biên độ khuếch đại dự đoán mất ổn định" |
| "Áp dụng tiêu chuẩn Routh"                     | "Mảng Routh đếm số lần đổi dấu vì mỗi lần đổi dấu là một nghiệm đi qua trục ảo" |
| "Tính K LQR từ phương trình Riccati"           | "Phương trình Riccati cân bằng hai chi phí: lệch khỏi trạng thái mong muốn và tiêu tốn điều khiển" |

### Ý Nghĩa Vật Lý và Triển Khai của Mọi Tín Hiệu

Mỗi tín hiệu trong mọi sơ đồ khối của sách này đều có bốn thuộc tính:

1. **Tên** — ký hiệu ($e(t)$, $u(t)$, $y(t)$, $d(t)$)
2. **Đơn vị** — đại lượng vật lý (°C, Watt, Volt, rad/s)
3. **Ý nghĩa vật lý** — nó đại diện cho gì ngoài thực tế
4. **Hiện thực phần cứng** — nó tồn tại ra sao trong hệ thống vật lý

Ví dụ, trong bộ điều khiển nhiệt độ phòng:

| Tín hiệu | Tên | Đơn vị | Ý nghĩa vật lý | Phần cứng |
|----------|-----|--------|----------------|-----------|
| $r(t)$   | Giá trị đặt | °C | Nhiệt độ mong muốn của người dùng | Núm xoay → chiết áp → ADC → vi điều khiển |
| $e(t)$   | Sai lệch | °C | Độ lệch nhiệt độ: khoảng cách tới thoải mái. **Cửa sổ duy nhất của bộ điều khiển với thế giới.** | Tính trong firmware: `e = r - y` |
| $u(t)$   | Điều khiển | W | Công suất điện cấp cho máy sưởi | Rơ-le/MOSFET điều khiển điện trở |
| $y(t)$   | Đầu ra | °C | Nhiệt độ thực tế trong phòng | Cảm biến nhiệt → mạch chia áp → ADC |
| $d(t)$   | Nhiễu | °C | Nhiệt độ ngoài trời — môi trường tác động. **Không kiểm soát được.** | Thời tiết, mở cửa sổ |

Đây không chỉ là dán nhãn. Khi sinh viên hiểu $e(t)$ là "độ lệch nhiệt độ tính bằng độ C, được vi điều khiển tính toán, và là *kênh duy nhất* bộ điều khiển nhận biết nhiễu", các bạn hiểu *vì sao* phản hồi hoạt động. Sai lệch không còn là khái niệm trừu tượng — nó là kênh mang thông tin vật lý. Mọi tín hiệu trong mọi sơ đồ khối đều được gắn với thực tế như vậy.

### Lý Thuyết Trước, Luôn Luôn

Đây không phải là sách lập trình có nhắc đến điều khiển. Đây là giáo trình điều khiển hệ thống nghiêm túc — từ ổn định Routh-Hurwitz đến điều khiển bền vững H∞, hàm mô tả — sử dụng C++ làm ngôn ngữ tính toán thay cho MATLAB. Mọi định lý đều được phát biểu chính xác. Mọi phép biến đổi đều được trình bày. Mọi quy trình thiết kế đều được chứng minh toán học trước khi lập trình.

### Thư Viện CppPlot

Tất cả ví dụ mã đều dùng **CppPlot**, thư viện C++ hiện đại, chỉ gồm header, thiết kế riêng cho sách này. CppPlot cung cấp API giống MATLAB cho phân tích hệ điều khiển:

```cpp
#include <cppplot/cppplot.hpp>
#include <cppplot/control/control.hpp>
using namespace cppplot;
using namespace cppplot::control;

int main() {
    // Định nghĩa đối tượng
    TransferFunction G({10}, {1, 3, 2});
    
    // Thiết kế với công cụ miền tần số
    auto m = margin(G);
    std::cout << "GM = " << 20*std::log10(m.Gm) << " dB, "
              << "PM = " << m.Pm << "°" << std::endl;
    
    // Vẽ đồ thị
    figure(800, 500);
    bode(G);
    savefig("bode_plot.svg");
    
    return 0;
}
```

Thư viện hỗ trợ `TransferFunction`, `StateSpace`, `Matrix`, vẽ Bode/Nyquist/root locus, đáp ứng step/impulse, thiết kế điều khiển (`lqr`, `place`, `acker`), bộ lọc Kalman, MPC, H∞ — đủ cho một khóa học điều khiển hoàn chỉnh. Phụ lục C là tài liệu API đầy đủ.

### Ưu Tiên Vật Lý, Đậm Chất Điện-Điện Tử/Viễn Thông

Mỗi chương bắt đầu từ một hệ vật lý thực — động cơ DC, con lắc ngược, bộ nghịch lưu Buck, PLL, inverter ba pha — và dẫn dắt mô hình toán học từ nguyên lý vật lý trước khi phân tích và thiết kế. Ứng dụng điện-điện tử và viễn thông không bị gói gọn trong một chương mà xuyên suốt toàn bộ sách:

- **Chương 1:** PLL như một hệ phản hồi
- **Chương 2:** Mạch RLC, mô hình động cơ DC
- **Chương 8:** Thiết kế bộ lọc PLL bằng phương pháp miền tần số
- **Chương 9:** Nhận dạng hệ từ dữ liệu thực nghiệm
- **Chương 10:** Mô hình dq-frame inverter ba pha
- **Chương 12:** DPLL như thiết kế phản hồi trạng thái
- **Chương 14:** PID số, chống bão hòa, bộ cộng hưởng PR rời rạc
- **Phụ lục A:** Thiết kế truyền động EV tích hợp (FOC + vòng dòng + vòng tốc độ)

### Xác Định Vấn Đề, Không Chỉ Giải Quyết Vấn Đề

Với sự phát triển bùng nổ của AI, kỹ năng còn lại không thể thay thế là **xác định đúng vấn đề**. Sách này dạy sinh viên phân biệt triệu chứng và nguyên nhân, đặt câu hỏi về giả định, và nhận ra khi nào cần tiếp cận hoàn toàn mới.

#### Ẩn Dụ Bưu Điện

Câu hỏi: *làm sao cải thiện dịch vụ bưu điện?*

```
Cấp 0 — Vấn đề bề mặt:
"Cần chuyển thư nhanh hơn."
→ Giải pháp: ngựa nhanh hơn, tàu, máy bay, chuyển phát nhanh.
   Công nghệ: vận chuyển.

Cấp 1 — Sâu hơn một lớp:
"Mục tiêu không phải chuyển GIẤY mà là chuyển NỘI DUNG."
→ Giải pháp: điện báo, fax, email.
   Công nghệ: tín hiệu điện.

Cấp 2 — Sâu hơn nữa:
"Nội dung là phương tiện. Mục tiêu là truyền
   SUY NGHĨ và CẢM XÚC của người gửi đến người nhận."
→ Giải pháp: điện thoại, gọi video.
   Công nghệ: mã hóa và truyền tải âm thanh/hình ảnh.

Cấp 3 — Định nghĩa lại sâu nhất:
"Giọng nói và hình ảnh vẫn gián tiếp. Nếu truyền tín hiệu não trực tiếp thì sao?"
→ Giải pháp: giao diện não-máy tính, truyền tín hiệu thần kinh.
   Công nghệ: khoa học thần kinh + xử lý tín hiệu.
```

Mỗi cấp độ sâu hơn không cải tiến giải pháp cũ mà làm nó lỗi thời. Ngựa nhanh hơn không dẫn đến email. Fax tốt hơn không dẫn đến gọi video. Đột phá đến từ việc *định nghĩa lại vấn đề thực sự là gì*.

Trong điều khiển cũng vậy:

| Ẩn dụ bưu điện | Điều khiển tương tự |
|---|---|
| "Chuyển thư nhanh hơn" | "Tăng tốc PID" |
| "Chuyển nội dung, không phải giấy" | "Vấn đề không phải tốc độ mà là khử nhiễu. Đổi kiến trúc." |
| "Truyền suy nghĩ/cảm xúc" | "Vấn đề không ở điều khiển mà ở mô hình cảm biến. Sửa khả năng quan sát trước." |
| "Não-đến-não" | "Thiết kế lại toàn bộ — thay phản hồi bằng dự đoán feedforward dựa trên mô hình vật lý" |

Mỗi chương đều hỏi **"Vấn đề thực sự là gì?"** trước khi hỏi "Giải quyết ra sao?". Sinh viên được rèn luyện để bóc tách lớp vấn đề, phân biệt triệu chứng và nguyên nhân, và nhận ra khi nào cần tiếp cận mới hoàn toàn.

### Học Theo Kết Quả

Mỗi chương mở đầu bằng bảng **Kết Quả Học Tập** theo thang Bloom — từ *Nhớ* (định nghĩa thuật ngữ) đến *Sáng tạo* (thiết kế bộ điều khiển hoàn chỉnh). Bài tập cuối chương bám sát các mức này, giúp giảng viên và sinh viên tự đánh giá mức độ thành thạo.

---

## Cấu Trúc Sách

Sách gồm 5 phần, từ nền tảng đến ứng dụng:

```
PHẦN I — NỀN TẢNG (Chương 1–3)
  Ch 1: Giới thiệu hệ điều khiển
  Ch 2: Mô hình toán học hệ động lực
  Ch 3: Biến đổi Laplace và hàm truyền

PHẦN II — ĐIỀU KHIỂN CỔ ĐIỂN (Chương 4–8)
  Ch 4: Phân tích miền thời gian (đáp ứng quá độ, ổn định, Routh-Hurwitz)
  Ch 5: Quỹ tích nghiệm và thiết kế
  Ch 6: Phân tích tần số (Bode, biên độ/pha)
  Ch 7: Tiêu chuẩn ổn định Nyquist
  Ch 8: Thiết kế điều khiển miền tần số (lead, lag, PID)

PHẦN III — TỪ DỮ LIỆU ĐẾN MÔ HÌNH (Chương 9)
  Ch 9: Nhận dạng hệ thống từ dữ liệu thực nghiệm

PHẦN IV — ĐIỀU KHIỂN HIỆN ĐẠI (Chương 10–13)
  Ch 10: Biểu diễn không gian trạng thái
  Ch 11: Phân tích không gian trạng thái (điều khiển được, quan sát được, ổn định)
  Ch 12: Phản hồi trạng thái và đặt cực
  Ch 13: Bộ quan sát và lọc Kalman

PHẦN V — CHỦ ĐỀ NÂNG CAO (Chương 14–17)
  Ch 14: Điều khiển số
  Ch 15: Điều khiển tối ưu (LQR, LQG, Kalman-Bucy)
  Ch 16: Điều khiển bền vững (H∞, μ, bất định cấu trúc)
  Ch 17: Điều khiển phi tuyến (Lyapunov, hàm mô tả, sliding mode)

PHẦN VI — ỨNG DỤNG VÀ BIÊN GIỚI
  Ch 18: Điều khiển trong kỷ nguyên AI
  Phụ lục A: Nghiên cứu thiết kế tích hợp — truyền động EV
  Phụ lục B: Hệ thống robot
  Phụ lục C: Tài liệu thư viện CppPlot
  Phụ lục D: Lập trình C++ cho kỹ sư điều khiển
```

**Phần I–II** (Chương 1–8): khóa học điều khiển cổ điển một học kỳ. **Phần I–IV** (Chương 1–13): hai học kỳ, với Ch9 (Nhận dạng hệ) là cầu nối. **Phần V–VI**: cho cao học hoặc kỹ sư thực hành.

### Sơ Đồ Phụ Thuộc

```
Ch1 ──▶ Ch2 ──▶ Ch3 ──▶ Ch4 ──▶ Ch5 ──▶ Ch8
                  │       │              ▲
                  │       └──▶ Ch6 ──▶ Ch7 ─┘
                  │
                  └──▶ Ch8 ──▶ Ch9 (Nhận dạng hệ)
                                  │
                  ┌──────────────┘
                  ▼
                 Ch10 ──▶ Ch11 ──▶ Ch12 ──▶ Ch13
                               │          │
                               ▼          ▼
                             Ch14       Ch15 ──▶ Ch16
                                                   │
                             Ch17 ◀──────────────┘
                               │
                             Ch18 (cần Ch1-Ch17)
```

---

## Yêu Cầu Kiến Thức Nền

Sách này yêu cầu:

- **Toán:** Giải tích (đạo hàm, tích phân, chuỗi Taylor), phương trình vi phân thường, đại số tuyến tính cơ bản (ma trận, trị riêng, định thức). Chương 3 ôn lại biến đổi Laplace.
- **Lập trình:** Biết C++ cơ bản — biến, hàm, vòng lặp, lớp, thư viện chuẩn. Không cần template nâng cao. Phụ lục D là phần ôn tập C++ cho điều khiển.
- **Kỹ thuật:** Vật lý cơ bản (định luật Newton, Kirchhoff, bảo toàn năng lượng). Đã học mạch điện hoặc động lực học là lợi thế nhưng không bắt buộc — mọi mô hình đều dẫn dắt từ đầu.

---

## Hướng Dẫn Sử Dụng Sách

### Cho Giảng Viên

- **Một học kỳ:** Chương 1–8 (điều khiển cổ điển). Giao 4–5 bài tập/chương. Dùng Phụ lục A làm đồ án.
- **Hai học kỳ:** Thêm Ch9–15. Học kỳ 2 bắt đầu với nhận dạng hệ (Ch9), tiếp đến không gian trạng thái (Ch10–13), điều khiển số (Ch14), tối ưu (Ch15).
- **Cao học:** Ch15–18 (tối ưu, bền vững, phi tuyến, AI), cần Ch10–13. Ch18 phù hợp seminar.
- **Thực hành:** Mỗi chương đều có mã chạy được. Sinh viên có thể tái tạo mọi đồ thị bằng cách gõ mã và chạy.

### Cho Sinh Viên

- **Đọc chủ động:** Đừng bỏ qua các phép biến đổi. Lý luận toán học là nền tảng giúp điều khiển dự đoán được.
- **Tự gõ mã:** Đừng chỉ đọc — hãy gõ, biên dịch, chạy, rồi *thay đổi gì đó*. Điều gì xảy ra nếu tăng tỷ số tắt dần? Nếu biên độ khuếch đại âm?
- **Làm bài tập:** Theo thang Bloom. Hoàn thành bài ⭐⭐⭐ là đã làm chủ chương đó.
- **Thường xuyên tra cứu Phụ lục C:** Đây là tài liệu API cho mọi hàm dùng trong sách. Khi thấy `margin(G)` trong ví dụ, hãy tra cứu để hiểu rõ.

### Cho Kỹ Sư Thực Hành

- **Tập trung Ch14–18** nếu đã biết điều khiển cổ điển/hiện đại. Điều khiển số (Ch14), bền vững (Ch16), AI (Ch18) là chủ đề công nghiệp quan trọng.
- **Dùng Phụ lục A** làm mẫu cho dự án tích hợp.
- **Dùng mã mẫu** làm khởi đầu cho dự án thực tế. Thư viện CppPlot chỉ cần C++17 và không phụ thuộc ngoài.

---

## Vì Sao Chọn C++

Chọn C++ thay MATLAB/Python là quyết định sư phạm quan trọng nhất của sách này.

**MATLAB** là công cụ chuẩn cho giáo dục điều khiển, nhưng là phần mềm thương mại, đắt đỏ, và — quan trọng nhất — không chạy trên hệ nhúng. Sinh viên học `tf`, `bode`, `step` bằng MATLAB sau này phải chuyển sang C/C++, thường gặp vấn đề về độ chính xác số, quản lý bộ nhớ, ràng buộc thời gian thực.

**Python** (với `python-control`) là lựa chọn miễn phí tốt, nhưng kiểu động và GC khiến nó không phù hợp cho điều khiển nhúng thời gian thực, và thư viện `python-control` chưa đầy đủ như MATLAB.

**C++** có ưu điểm:
- **Hiệu năng:** Thời gian thực xác định, không GC, truy cập phần cứng trực tiếp.
- **Công nghiệp:** Ngôn ngữ mặc định cho điều khiển nhúng (ô tô, hàng không, tự động hóa, robot).
- **Hiện đại:** C++17 với structured bindings (`auto [t, y] = step_data(G)`) giúp mã dễ đọc như MATLAB.
- **Tự chứa:** CppPlot không phụ thuộc ngoài. Chỉ cần trình biên dịch C++17 và editor.

Nhược điểm là đường cong học tập cao hơn. Sách khắc phục bằng Phụ lục D (ôn tập C++), ví dụ mã tăng dần độ phức tạp, và API CppPlot giống MATLAB (`TransferFunction`, `bode()`, `step()`, `lqr()`, `margin()`).

---

## Lời Cảm Ơn

Sách này dựa trên nền tảng của các bậc thầy:
- **Katsuhiko Ogata** — *Modern Control Engineering*
- **Gene Franklin, J. David Powell, Abbas Emami-Naeini** — *Feedback Control of Dynamic Systems*
- **Karl Johan Åström, Richard Murray** — *Feedback Systems: An Introduction for Scientists and Engineers*
- **Lennart Ljung** — *System Identification: Theory for the User*
- **Kemin Zhou, John Doyle, Keith Glover** — *Robust and Optimal Control*
- **Hassan Khalil** — *Nonlinear Systems*

Ứng dụng EE/Viễn thông dựa trên kinh nghiệm thực tế và các tài liệu:
- **Ned Mohan** — *Power Electronics*
- **Peter Vas** — *Sensorless Vector and Direct Torque Control*

CppPlot được phát triển như công cụ giảng dạy, phản ánh niềm tin rằng học tốt nhất khi thấy sự tương ứng giữa toán và mã trên cùng một trang.

Cảm ơn đồng nghiệp, sinh viên đã thử nghiệm bản thảo, phát hiện lỗi, điểm chưa rõ, thiếu sót. Đặc biệt cảm ơn các reviewer đã audit bốn vòng, phát hiện lỗi toán, API, sư phạm trên toàn bộ 22 chương và phụ lục.

---

## Về AI và Tương Lai

Chương 18 — *Điều khiển trong kỷ nguyên AI* — trả lời câu hỏi: nếu AI có thể thiết kế bộ điều khiển, tại sao phải học điều khiển?

Câu trả lời bề mặt: AI không thay thế lý thuyết điều khiển như máy tính không thay thế toán học. Nhưng sâu xa hơn, cần phân biệt **khoa học**, **kỹ thuật**, **công nghệ**:

- **Công nghệ** (công cụ, mã, thuật toán) thay đổi nhanh — AI là công nghệ mới mạnh mẽ nhất một thế hệ.
- **Khoa học** (lý thuyết ổn định, Lyapunov, Shannon) không thay đổi. Đó là chân lý tự nhiên.
- **Kỹ thuật** (xác định đúng vấn đề, mô hình hóa, đánh giá giải pháp) trường tồn qua mọi thay đổi công nghệ.

AI làm tốt các cấp thấp của kỹ năng kỹ sư:

| Cấp | Kỹ năng | AI hiện nay |
|-----|---------|-------------|
| 1 | **Thực thi** — chuyển thiết kế thành mã chạy được | AI làm tốt, ngày càng mạnh |
| 2 | **Giải quyết vấn đề** — cho bài toán rõ ràng, tìm lời giải tối ưu | AI cạnh tranh, tiến bộ nhanh |
| 3 | **Xác định vấn đề** — nhận ra *nên* giải gì và *vì sao* | Không thể thay thế |
| 4 | **Tư duy nguyên bản** — thấy kết nối, cấu trúc, khả năng chưa ai nói ra | Không thể thay thế |

Kỷ nguyên mà tốc độ code quyết định kỹ sư giỏi đang qua đi. Điều còn lại là *tư duy rõ ràng* về máy nên tính gì và vì sao — xác định đúng vấn đề trước khi viết (hoặc sinh) một dòng mã. Nhớ lại ẩn dụ bưu điện: kỹ sư thấy vấn đề thực là "truyền nội dung" chứ không phải "chuyển thư nhanh hơn" sẽ phát minh ra điện báo.

Điều này ảnh hưởng sâu sắc đến giáo dục điều khiển. Sinh viên học sách này không chỉ biết thiết kế PID — AI làm được. Các bạn sẽ:

- **Nhận ra** vấn đề thực trong lưới điện không phải điều tần mà là ngăn sụp đổ dây chuyền — cần H∞ chứ không chỉ điều khiển tỉ lệ.
- **Đặt câu hỏi** về mô hình: "Hệ này thật sự tuyến tính? Nhiễu có thật sự Gauss? Mô hình đã đủ liên kết nhiệt-điện chưa?"
- **Kết nối** liên ngành: thấy PLL là hệ phản hồi (Ch1), Kalman là suy luận Bayes (Ch13), LQR và RL tối ưu cùng hàm mục tiêu (Ch15/18).
- **Đánh giá** thiết kế AI sinh ra: kiểm tra biên độ ổn định trên Bode, chứng minh Lyapunov, đảm bảo bền vững — vì AI đề xuất bộ điều khiển không có chứng chỉ ổn định thì đẹp nhưng có thể nguy hiểm.

Kỹ sư 2030 sẽ không chọn giữa điều khiển cổ điển và AI. Họ sẽ dùng cả hai — dùng Bode để kiểm tra bền vững, LQR để tối ưu, H∞ để giới hạn trường hợp xấu nhất, RL để thích nghi với điều kiện chưa ai dự đoán. Nhưng *không thể thay thế* là tư duy nguyên bản: xác định vấn đề, chọn mô hình, đánh giá giải pháp, chịu trách nhiệm.

**Công nghệ sẽ thay đổi. Khoa học thì không.** Sách này trang bị cho bạn cả hai — khoa học trường tồn và tư duy kỹ thuật mà không máy nào thay thế được. AI có thể tính biên độ ổn định. Chỉ bạn mới quyết định biên độ đó *đủ an toàn* cho hệ thống liên quan đến con người.

---

*Hệ điều khiển tốt nhất là hệ bạn không nhận ra nó tồn tại. Sách tốt nhất cũng vậy — nó xóa mọi rào cản giữa bạn và sự hiểu biết, khiến mọi ý tưởng trở nên hiển nhiên. Đó là mục tiêu của cuốn sách này. Thành công hay không là do bạn đánh giá.*

\vspace{1cm}

\hfill *Tri-Vien Vu*

\hfill *Tháng 2, 2026*