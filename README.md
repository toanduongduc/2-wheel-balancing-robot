# Robot tự cân bằng 2 bánh (STM32 + MPU6050)

Đây là project mình tự làm để tìm hiểu về điều khiển PID và xử lý dữ liệu cảm biến IMU trên STM32.  
Robot có nhiệm vụ giữ thăng bằng trên 2 bánh bằng cách đọc góc nghiêng và điều khiển động cơ phù hợp.

## 1. Phần cứng sử dụng

- STM32 (dùng HAL)
- MPU6050 (giao tiếp I2C)
- Driver L298N
- 2 động cơ DC
- Nguồn pin

## 2. Cách hoạt động

### Đọc dữ liệu MPU6050

- Đọc 14 byte liên tục từ thanh ghi 0x3B
- Lấy Accel và Gyro
- Khi khởi động sẽ calibrate offset trong khoảng 3 giây
- Sau đó trừ offset để giảm sai số

### Tính góc nghiêng

Góc Pitch được tính bằng bộ lọc bù:

Pitch = 0.98 × (Gyro tích phân) + 0.02 × (Accel)

- Gyro giúp phản ứng nhanh
- Accel giúp ổn định lâu dài
- α = 0.98

### Điều khiển PID

Sai số:

Error = Pitch_Angle - Setpoint

PID gồm 3 thành phần:

- P: phản ứng chính
- I: bù sai lệch lâu dài
- D: giảm dao động

Thông số hiện tại:

- Kp = 110
- Ki = 0
- Kd = 12

Có thêm:
- Giới hạn I-term để tránh tràn
- Giới hạn PWM ±999
- Vùng chết nếu nghiêng quá 45° thì tắt motor
- Bù ma sát để động cơ vượt qua lực cản ban đầu

## 3. Điều khiển động cơ

- PWM dùng TIM2 (Period = 999)
- Điều khiển chiều quay bằng PB12–PB15
- Set duty bằng __HAL_TIM_SET_COMPARE()

## 4. Debug

In dữ liệu qua UART 115200:

- Góc
- Error
- P, I, D
- PWM

Giúp dễ tuning PID.

## 5. Hướng phát triển

- Tuning PID tốt hơn
- Thêm điều khiển tiến/lùi
- Thêm Bluetooth
- Làm PCB gọn hơn

Project này mình làm để hiểu rõ hơn về:
- Bộ lọc bù
- Điều khiển PID thời gian thực
- Cách đọc cảm biến qua I2C
- Điều khiển PWM trên STM32