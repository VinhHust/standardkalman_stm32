giàn ý 
                FILE .H
- struct chứa 
    + predicted state bằng ssm 
    + corrected state bằng kalman gain 
    + cờ first time cho lần đầu khởi tạo (lần đầu khởi tạo thì chưa có xpred)

- hàm khởi tạo cho kalman (dành cho lần đầu chạy)
    + con trỏ vào các thành phần trong struct trên 
    + 4 trạng thái 

* 4 trạng thái:
- innovation (sai số giữa đo được và predict từ SSM)
- correction: tìm ra corrected state dựa vào Kalman Gain 
- feed trạng thái correct vào LQR
- predict bằng SSM cho chu kì TIẾP THEO 

- hàm corection:
    + con trỏ vào struct đầu tiên 
    + input là trạng thái đo được trực tiếp (x và theta)

- hàm predict: 
    + nhận vào con trỏ trỏ vào struct đầu tiên 
    + gia tốc vừa tính được ởh phần feed LQR phía trên 

                FILE.C
- các ma trận từ hệ rời rạc cần nạp vào từ matlab
    + ma trận Add
    + ma trận Bdd
    + ma trận Ld (Kalman gain)

- nội dung hàm khởi tạo cho kalman 
    + con trỏ truy cập vào các vector khai báo trong struct, gán 4 thành phần trong vector thành tên các trạng thái, đây chính là gán giá trị lúc chạy kalman chính bằng state ngay tại lúc đó 
    + predicted state bằng correction state (vì lúc này innovation =0ta)
    + lần đầu chạy kalman thì giá trị xpred chính là giá trị của state đo được nên innovation = 0 

- nội dung hàm correct cho kalman
    + tính innovation = sai lệch trạng thái - sai lệch dự đoán
    + sửa sai cho cả 4 trạng thái, for
        * trạng thái sửa sai = trạng thái dự đoán + L.(innovation)

- nội dung hàm predict cho chu kì tiếp theo kalman 
    + tính Add * trạng thái sửa sai 
    + Bđ * gia tốc lqr 
    + trạng thái dự đoán (dùngg cho vòng lặp tiếp) = 2 thằng trên cộng lại 



