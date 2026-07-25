#include "lqgcontroller.h"
#include <math.h>
#define DT 0.005f
#define PI 3.141592625f
#define MiliMtoM 0.001f //hệ số chuyển từ mm sang m
#define GRAVITY 9.81f

//CẢNH BÁO: ma trận Add trong kalman.c đang dựng với dt = 0.01 (100Hz)
//nhưng vòng điều khiển này chạy DT = 0.005 (200Hz).
//Phải rời rạc hóa lại Add/Bdd/Ldd trong MATLAB với dt = 0.005 thì LQG mới đúng.

//các tham số của hệ (DÙNG CHO SWING UP)
float pendulum_mass = 0.1f;
float pendulum_length = 0.15f;
float pendulum_inertia = 0.00075f; //phần này có thể tune

//các tham số cho phần SWING UP
float k_swing = 8.0f;
float max_swing_acce = 12.0f;
float max_cart_limit = 0.25f;

//các tham số cho phần KICK START
float kick_acce = 2.5f;
float kick_time = 0.05f;
static float kick_counter = 0.0f; //static để lưu lại giá trị kick_counter để cộng dồn trong mỗi lần gọi hàm
float max_kick_speed = 5.0f;

//các tham số để tune LQR
//LQR tương ứng theta,theta',x,x'
float LQR_tune[4] = {525.2304,  96.8291,-132.2876,-137.9898};
static float reference_point = 0.0f;
static float  target_speed = 0.0f;

float max_speed = 10.0f;
float max_accel = 6.0f;
//u=-K(X-Xref)=K(Xref-X)

//bộ PD cho swing up
float k1_swing = 3.0f; //gain để kéo vị trí quanh tâm
float k2_swing = 2.0f; //gain để ko bị oscillate quá
float debug_sign_val = 0.0f; // Biến toàn cục để lôi sign_val ra ngoài
float debug_sign = 0.0f; // Biến toàn cục để lôi sign_val ra ngoài
float debug_pend_velo = 0.0f;

//lôi state ước lượng của Kalman ra ngoài để so với encoder thô
float debug_kf_theta = 0.0f;
float debug_kf_omega = 0.0f;
float debug_kf_x = 0.0f;
float debug_kf_v = 0.0f;

//CÁC THAM SỐ TUNE PHẦN SWING DOWN
float k_angle_down = 8.0f;
float k_position_down = 12.0f;
float k_vel_down = 8.0f;
float max_swingdown_accel = 5.0f;

//THAM SỐ CHO PHẦN NGƯỠNG DỪNG HẲN VÀ VỀ IDLE THAY VÌ DAO ĐỘNG MÃI MÃI
float down_angle_ok = 0.08f;   //rad
float down_omega_ok = 0.5f;    //rad/s
float down_x_ok     = 0.015f;  //m
float down_v_ok     = 0.05f;   //m/s
static float down_hold_timer  = 0.0f; //đếm thời gian con lắc đứng yên liên tục

//EMA FILTER CHO VẬN TỐC
float alpha = 0.8f;
static float filter_speed = 0.0f; // Biến lưu trữ vận tốc của chu kỳ trước

//HÀM LẤY SAI LỆCH TRẠNG THÁI CỦA CON LẮC VÀ CART ĐỂ DÙNG CHO LQG
static float angle_error(float angle){
    float deviation = 0.0f;
    if (angle > 0.0f){
        deviation = angle - PI;
}
    else{
        deviation = angle + PI;
    }
    return deviation;
}

//gom phần chuyển sang trạng thái cân bằng vào 1 chỗ, vì có 2 nơi gọi tới (IDLE và SWING_UP)
//HAM KHOI TAO LQG CONTROL, GAN REFERENCE POINT, RESET TARGET SPEED, KHOI TAO KALMAN
static void enter_lqg_control(InvertedPendulumTypeDef* pendulum){
    //gan reference_point = vị trí hiện tại của cart
	reference_point = pendulum->cart_enc->linear_position * MiliMtoM;
	target_speed = 0.0f; //reset để lần catch tiếp theo không bị dính vận tốc cũ

    //CA 4 BIEN NAP VAO HAM NAY DEU LA BIEN SAI LECH
	Kalman_Firsttime(pendulum->kalman_filter, angle_error(pendulum->pend_enc->angle_position),pendulum->pend_enc->angle_speed,
	                 0.0f,pendulum->cart_enc->linear_speed * MiliMtoM);

	pendulum->state = LQG_CONTROL;
}

//hàm khởi tạo trạng thái ban đầu cho con lắc, hàm nào nhận vào con trỏ
//pendulum chỉ là tên nội bộ, khi lôi ra sử dụng thì sẽ dùng tên biến khác
void init_pendulum(InvertedPendulumTypeDef* pendulum) {
	pendulum->state = IDLE_PENDULUM;
	//con trỏ lồng con trỏ, con trỏ pendulum truy cập tới con trỏ steppẻ
	Stepper_UpdateCartFrequency(pendulum->stepper, 0);
	target_speed = 0.0f; //reset lại target speed mỗi khi từ LQR về lại IDLE
	filter_speed = 0.0f;

}
float run_pendulum(InvertedPendulumTypeDef* pendulum){
	//cập nhật dữ liệu của encoder thông qua 2 con trỏ pend_enc và cart_enc
	update_encoder_pendulum(pendulum->pend_enc); //con trỏ pendulum truy cập vào con trỏ pend_enc trỏ tới kiểu dữ liệu EncoderTypeDef
	update_encoder_cart(pendulum->cart_enc);

	switch(pendulum->state){

		//TRẠNG THÁI NGHỈ CỦA PENDULUM
		case IDLE_PENDULUM:{
			Stepper_UpdateCartFrequency(pendulum->stepper, 0);
			float current_angle = pendulum->pend_enc->angle_position;

			//TRƯỜNG HỢP 1
			if(current_angle > (PI-0.17f) || current_angle < -(PI-0.17f)){
				filter_speed = 0.0f; //reset lại filterspeed lúc ở LQR dể reset lại filter speed lúc LQR
				enter_lqg_control(pendulum); //gán reference_point, reset target_speed, khởi tạo Kalman
                //cai nay la gan reference point ngay luc bat lqr chu khong phai gan reference point o diem giua ray
				break;
		}
			//TRƯÒNG HỢP 2
			//ấn nút PC13 trên mạch thì bay sang phần kickstart
			if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET){
				kick_counter = 0.0f;
				target_speed = 0.0f;
				filter_speed = 0.0f;
				pendulum->state = KICK_START;
 			}
			break;
	}

		//TRẠNG THÁI KICKSTART CẤP GIA TỐC BAN ĐẦU
		case KICK_START:{
			kick_counter += DT; //CỘNG DỒN MỖI LẦN GỌI HÀM RUN_PENDULUM TRONG INTERRUPT Ở MAIN
			target_speed = target_speed + (kick_acce * DT); //tích phân về vận tốc

		if (kick_counter > kick_time){
			pendulum -> state = SWING_UP; //quá 8 chu kì lấy mẫu thì bay sang swing up
			break;
		}
		break;
		}

		//TRẠNG THÁI SWING UP
		case SWING_UP:{
			float current_cart_pos = pendulum->cart_enc->linear_position * MiliMtoM;
			float current_angle = pendulum->pend_enc->angle_position;
			float current_cart_vel = pendulum->cart_enc->linear_speed * MiliMtoM;

			//chuyển trạng thái sang cân bằng
			if(current_angle > (PI-0.17f) || current_angle < -(PI-0.17f)){
							//nếu như vung đến vùng bắt thì sẽ bật bắt
							enter_lqg_control(pendulum); //gán reference_point, reset target_speed, khởi tạo Kalman
				break;}

			float pend_velo = pendulum->pend_enc->angle_speed;
			                //các thành phần trong năng lượng của HỆ
			float mlsquare = pendulum_mass * pendulum_length * pendulum_length; 	  // ml^2 //thử bỏ ml bình này đi
			float Ek =  0.5f * (pendulum_inertia) * (pend_velo*pend_velo); // 0.5(I+ml^2)
			float Ep = pendulum_mass * GRAVITY *pendulum_length * (1.0f - cosf(current_angle)); // mgl(1-cos(theta))
			float E = Ek + Ep;

			//năng lượng MONG MUỐN
			float Er = 2.0f * pendulum_mass * GRAVITY * pendulum_length; //năng lương mong muốn là 2mgl
			float sign = pend_velo * cosf(current_angle);
			float sign_val = 0.0f;

			if(sign>0.0f){ //thử tăng giới hạn đổi dấu của sign
				sign_val = 1;
			}
			else if(sign<-0.0f){
				sign_val = -1;
			}
			else{
				sign_val = 0; //nếu như nằm ở khoảng giữa thì = 0 luôn để ko bị nhiễu
			}
			debug_sign = sign;
			debug_sign_val = sign_val;
			debug_pend_velo = pend_velo;
			//a = k.(E-Eo).sign(theta'.costheta)
			float aswing = k_swing * (E - Er) * sign_val;


			//thêm bộ PD để giữ phần swing up loanh quanh tâm
			aswing += (-k1_swing * current_cart_pos) - (k2_swing * current_cart_vel); //cộng thêm bộ PD

			//giới hạn vận tốc swing
			if (aswing>max_swing_acce){
				aswing = max_swing_acce;
			}
			else if (aswing<-max_swing_acce){
				aswing = -max_swing_acce;
			}
			pendulum ->current_accel = aswing;
			target_speed = target_speed + (aswing * DT); //tích phân về vận tốc

			//giới hạn vận tốc
			if(target_speed > max_speed){
							target_speed = max_speed;}
							else if(target_speed<-max_speed){
							target_speed = -max_speed;
						}
			break;
		}

		//TRẠNG THÁI CÂN BẰNG DÙNG LQG 
		case LQG_CONTROL:{
			float current_angle = pendulum->pend_enc->angle_position;

			//ẤN NÚT PC13 trong lúc đang cân bằng -> chủ động chuyển sang SWING DOWN
			if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET){
				down_hold_timer = 0.0f; //reset bộ đếm đứng yên cho lần swing down mới
				pendulum->state = SWING_DOWN;
				break;
			}

			//phần này để bảo vệ cơ khí nếu con lắc rủ xuống thấp quá
			if(fabsf(current_angle) < (PI*0.75f)){
				pendulum->state = IDLE_PENDULUM;
				break;
			}
            
            //con trỏ pendulum trỏ tới con trỏ kalman_filter, con trỏ này trỏ tới struct KalmanFilter
            //đặt tên là kf để dễ quản lí    
			KalmanFilter *kf = pendulum->kalman_filter;

			//quy đổi 2 phép đo sang hệ tọa độ ĐỘ LỆCH mà Kalman đang dùng
			float theta_lech_meas = angle_error(current_angle);
			float x_lech_meas = (pendulum->cart_enc->linear_position * MiliMtoM) - reference_point;

			//CORRECTION	
			//nội dung đầy đủ của hàm Kalman_Correct đã được triển khai trong file kalman.c, giờ gọi hàm ra là đã tính toán hết
			//giờ chỉ lôi các biến xcorrect đã được tính ra để dùng cho LQR
			Kalman_Correct(kf, theta_lech_meas, x_lech_meas);

			debug_kf_theta = kf->xcorrect[0];
			debug_kf_omega = kf->xcorrect[1];
			debug_kf_x     = kf->xcorrect[2];
			debug_kf_v     = kf->xcorrect[3];

			//BƯỚC 2: LUẬT ĐIỀU KHIỂN u = -K.xhat
			//state đã là độ lệch quanh upright nên xref = 0, dấu trừ nằm ngoài
			//tương đương y hệt LQR cũ: K[0]*(PI-angle) chính là -K[0]*theta_lech
			float lqr_giatoc = -( LQR_tune[0]*kf->xcorrect[0]
			                    + LQR_tune[1]*kf->xcorrect[1]
			                    + LQR_tune[2]*kf->xcorrect[2]
			                    + LQR_tune[3]*kf->xcorrect[3] );

			//GIỚI HẠN GIA TỐC
			if (lqr_giatoc > max_accel) {
			    lqr_giatoc = max_accel;
			} else if (lqr_giatoc < -max_accel) {
			    lqr_giatoc = -max_accel;
			}

			pendulum ->current_accel = lqr_giatoc; //ghi biến cục bộ vào currentaccel để debug

			//BƯỚC 3: PREDICT cho chu kỳ sau, nạp vào u ĐÃ BÃO HÒA
			//vì đó mới là gia tốc thực tế mà hệ nhận được
			Kalman_Predict(kf, lqr_giatoc);

			//TÍCH PHÂN THÀNH VẬN TỐC v = v0 + a.dT
			target_speed = target_speed + (lqr_giatoc * DT);

			//GIOI HẠN VẬN TỐC TỐI ĐA
			if(target_speed > max_speed){
				target_speed = max_speed;}
				else if(target_speed<-max_speed){
				target_speed = -max_speed;
			}
			break;
		}

		case SWING_DOWN:{
			float theta = pendulum->pend_enc->angle_position; 			//lấy góc
			float w  = pendulum->pend_enc->angle_speed;					//lấy vận tốc góc
			float x  = pendulum->cart_enc->linear_position * MiliMtoM;  //lấy vị trí
			float v  = pendulum->cart_enc->linear_speed    * MiliMtoM;  //lấy vận tốc dài

			//con lắc rủ qua hơn nửa vòng thì mới bật swing down chứ k bật ngay từ đầu 
			if(fabsf(theta) < (PI*0.5f)){
			
			float Ek = 0.5f * pendulum_inertia * (w*w); // 0.5(I+ml^2)
			float Ep = pendulum_mass * GRAVITY * pendulum_length * (1.0f - cosf(theta)); // mgl(1-cos(theta))
			float E = Ek + Ep;

			float sign_swingdown = w * cosf(theta);
			float sign_val_swingdown = 0.0f;

			if(sign_swingdown>0.0f){ //thử tăng giới hạn đổi dấu của sign
				sign_val_swingdown = 1;
			}
			else if(sign_swingdown<-0.0f){
				sign_val_swingdown = -1;
			}
			else{
				sign_val_swingdown = 0; //nếu như nằm ở khoảng giữa thì = 0 luôn để ko bị nhiễu
			}
			
			//Lúc ở vị trí cân bằng dưới thì Er = 0, nên E - Er = E - 0 = E
			float a_down = k_swing * (E - 0.0f) * sign_val_swingdown;
			a_down += -k_position_down * x - k_vel_down * v; //bộ PD kéo về tâm

			//GIỚI HẠN GIA TỐC
			if (a_down > max_swingdown_accel) {
					a_down = max_swingdown_accel;
			} else if (a_down < -max_swingdown_accel) {
					a_down = -max_swingdown_accel;
			}

			pendulum ->current_accel = a_down; //ghi biến cục bộ vào currentaccel để debug

			target_speed = target_speed + (a_down * DT);

			//giới hạn vận tốc
			if(target_speed > max_speed){
				target_speed = max_speed;}
			else if(target_speed<-max_speed){
				target_speed = -max_speed;}

			/* điều kiện để thoát swing down và sang idle: con lắc và cart đứng yên liên tục trong 1s  */
			if (fabsf(theta) < down_angle_ok &&
				fabsf(w) < down_omega_ok &&
				fabsf(x) < down_x_ok &&
				fabsf(v) < down_v_ok) {

				down_hold_timer += DT; //nếu nhỏ hơn các gía trị đã đặt LIÊN TỤC thì cứ LIÊN TỤC cộng dồn DT vào biến
			}
			else{
				down_hold_timer = 0.0f; //nếu lớn hơn thì lại reset biến đếm này về 0
			}
			if (down_hold_timer > 1.0f){
				target_speed = 0.0f;
				filter_speed = 0.0f;
				pendulum->state = IDLE_PENDULUM;
				break;

			}
		}
		else {
			//neu con lac o nua tren thi chua can dieu khien gi 
			target_speed = 0.0f;
			pendulum->current_accel = 0.0f;
			down_hold_timer = 0.0f; //reset bộ đếm đứng yên cho lần swing down mới
		}
			break;
		}
		
	}
	//bộ lọc EMA cho vận tốc
	//alpha lớn thì ưu tiên vận tốc mới, alpha nhỏ thì ưu tiên vận tốc cũ
	filter_speed = (alpha*target_speed) + ((1.0f-alpha)*filter_speed); //filter speed là vận tốc cũ, target speed là vận tốc mới
	return filter_speed;
}

