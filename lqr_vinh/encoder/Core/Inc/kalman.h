/*
 * kalman.h
 *
 *  Bo loc Kalman tuyen tinh roi rac (steady-state) cho he con lac nguoc tren xe
 *  State:      x = [theta, theta_dot, x_cart, x_cart_dot]  (deu la bien LECH quanh upright)
 *  Do luong:  y = [theta_lech, x_lech]                     (2 phep do: goc va vi tri cart)
 *  Input:     u = gia toc cart lenh (m/s^2, scalar)
 *
 *  Gain Ld (4x2) da tinh san offline bang dlqe() trong MATLAB
 *  -> tren chip KHONG chay nhanh hiep phuong sai P nua, chi con Correct + Predict.
 *
 *  Author: vu thanh vinh
 */

#ifndef KALMAN_H
#define KALMAN_H

#include <stdint.h>

typedef struct {
    float xpred[4]; //dự đoán dựa trên SSM
    float xcorrect[4]; //state sau khi correct
} KalmanFilter;

//Hàm khởi tạo ban đầu cho kalman 
void Kalman_Firsttime(KalmanFilter *kf, float theta, float theta_dot, float x_cart, float x_cart_dot);

//Hàm correct state dựa trên gain của kalman 
void Kalman_Correct(KalmanFilter *kf, float theta_meas, float x_cart_meas);

//Hàm predict state dựa trên mô hình SSM
void Kalman_Predict(KalmanFilter *kf, float u); //nhận vào gia tốc u từ LQR

#endif /* KALMAN_H */