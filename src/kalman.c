#include "kalman.h"

//all the matrices must be discrete
//matrix Add, Bdd, Cdd, Ldd from matlab 
static const float Add[4][4] = {
    {1.0f, 0.01f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 1.0f, 0.01f},
    {0.0f, 0.0f, 0.0f, 1.0f}
};

static const float Bdd[4] = {
    0.0f,
    0.01f,
    0.0f,
    0.0f
};

static const float Cdd[2][4] = {
    {1.0f, 0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 1.0f, 0.0f}
};

//phan nay se thay vao cac gia tri gain Ld tinh san offline tu matlab, nen khong can tinh lai tren chip
static const float Ldd[4][2] = {
    {0.0, 0.0f},
    {0.0f, 0.0f},
    {0.0f, 0.0f},
    {0.0f, 0.0f}
};

//trien khai noi dung trong ham kalmanfirst time 
void Kalman_Firsttime(KalmanFilter *kf, float theta, float theta_dot, 
                        float x_cart, float x_cart_dot) {
//gan cac gia tri ban dau cho state cua kalman filter chinh bang gia tri do luong 
//duoc khi kich hoat kalman filter lan dau tien

                            //MEASUREMENT
    kf->xpred[0] = theta;
    kf->xpred[1] = theta_dot;
    kf->xpred[2] = x_cart;
    kf->xpred[3] = x_cart_dot;
                            //PREDICTION
    kf->xcorrect[0] = theta;
    kf->xcorrect[1] = theta_dot;
    kf->xcorrect[2] = x_cart;
    kf->xcorrect[3] = x_cart_dot;
//first time thi innovation = measurement - prediction = 0
    kf->first_time = 1; //co danh dau lan chay dau tien 
}

void Kalman_Correct(KalmanFilter *kf,float delta_theta, float delta_x) {
    //correct state
    float innovation[2]; //innovation vector innovation 
    innovation[0] = delta_theta - kf->xpred[0];
    innovation[1] = delta_x - kf->xpred[2];

//chi co 2 innovation nhung co the correct duoc toan bo state
    kf->xcorrect[0] = kf->xpred[0] + Ldd[0][0]*innovation[0] + Ldd[0][1]*innovation[1];
    kf->xcorrect[1] = kf->xpred[1] + Ldd[1][0]*innovation[0] + Ldd[1][1]*innovation[1];
    kf->xcorrect[2] = kf->xpred[2] + Ldd[2][0]*innovation[0] + Ldd[2][1]*innovation[1];
    kf->xcorrect[3] = kf->xpred[3] + Ldd[3][0]*innovation[0] + Ldd[3][1]*innovation[1];

    kf->first_time = 0; //da chay qua lan dau tien roi
}

void Kalman_Predict(KalmanFilter *kf, float u) {
    //predict state
    kf->xpred[0] = Add[0][0]*kf->xcorrect[0] + Add[0][1]*kf->xcorrect[1] + Add[0][2]*kf->xcorrect[2] + Add[0][3]*kf->xcorrect[3] + Bdd[0]*u;
    kf->xpred[1] = Add[1][0]*kf->xcorrect[0] + Add[1][1]*kf->xcorrect[1] + Add[1][2]*kf->xcorrect[2] + Add[1][3]*kf->xcorrect[3] + Bdd[1]*u;
    kf->xpred[2] = Add[2][0]*kf->xcorrect[0] + Add[2][1]*kf->xcorrect[1] + Add[2][2]*kf->xcorrect[2] + Add[2][3]*kf->xcorrect[3] + Bdd[2]*u;
    kf->xpred[3] = Add[3][0]*kf->xcorrect[0] + Add[3][1]*kf->xcorrect[1] + Add[3][2]*kf->xcorrect[2] + Add[3][3]*kf->xcorrect[3] + Bdd[3]*u;
}