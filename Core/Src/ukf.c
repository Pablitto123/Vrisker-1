//
/**
 ******************************************************************************
 * @file           : kalman_filer.c
 * @brief          : Unscented Kalman Filter Implementation
 ******************************************************************************
 * @attention
 * This code causes epilepsy.
 * MIT License
 ******************************************************************************
 */
// Created by Łukasz Zelek on 17.03.2020.
#include "ukf.h"
#include "math_library.h"
#include "lowlvl.h"
#include <I2C_access.h>

#define dt (0.01f)
extern float __velocity;
extern float __omega;
float lidar_sdev_perc = 0.04f;
float enc_sdev = 10.045f;
float accel_sdev = 30.28f;
float omega_sdev = 1.f;
float bias_sdev = 0.1f;
float gyro_sdev = 1.f;
float alpha_sdev = 1.f;
float Rlidar = 0;
float Rodo = 0;
float Rgyro = 0;
extern matrix* matrix_stack[30];
matrix *f, *h, *P, *Q, *R, *x, *z, *Wm, *Wc, *A, *Y, *X, *X2, *m1, *Y1, *P1,
		*Z1, *z1, *Z2, *S, *C, *K;

matrix *tmp01, *tmp1, *tmp2, *tmp21, *tmp3, *tmp4, *tmp5, *tmp51, *tmp6, *tmp7;
void unscented_kalman_filter(float32_t acceleration, float32_t omega_enc) {
	int l = x->mat->numRows;
	float32_t alpha = 1;
	float32_t kappa = 0;
	float32_t beta = 0;
	float32_t lambda = alpha * alpha * ((float32_t) l + kappa)
			- (float32_t) beta;
	float32_t c = (float32_t) l + lambda;
	Wm->mat->pData[0] = lambda / c;
	Wm->mat->pData[1] = 0.5f / c;
	Wm->mat->pData[2] = 0.5f / c;
	Wm->mat->pData[3] = 0.5f / c;
	Wm->mat->pData[4] = 0.5f / c;
	Wm->mat->pData[5] = 0.5f / c;
	Wm->mat->pData[6] = 0.5f / c;
	Wc->mat->pData[0] = lambda / c;
	Wc->mat->pData[1] = 0.5f / c;
	Wc->mat->pData[2] = 0.5f / c;
	Wc->mat->pData[3] = 0.5f / c;
	Wc->mat->pData[4] = 0.5f / c;
	Wc->mat->pData[5] = 0.5f / c;
	Wc->mat->pData[6] = 0.5f / c;
	Wc->mat->pData[0] = Wc->mat->pData[0] + (1 - alpha * alpha + beta);
	c = sqrtf(c);
	tmp01 = matcho(P);
	A = matsca(tmp01, c);
	Y = matdup(x, l);
	tmp1 = matadd(Y, A);
	tmp2 = matsub(Y, A);
	X = matmer(matmer(x, tmp1), tmp2);
	memset(m1->mat->pData, 0, (m1->mat->numCols) * (m1->mat->numRows));
	for (int k = 0; k < X->mat->numCols; k++) {
		tmp21 = matfra(X, 0, X->mat->numRows, k, k + 1);
		tmp3 = matmul(f, tmp21);
		tmp3->mat->pData[0] = tmp3->mat->pData[0] * arm_cos_f32(tmp21->mat->pData[2]) + tmp21->mat->pData[0];
		tmp3->mat->pData[1] = tmp3->mat->pData[1] * arm_sin_f32(tmp21->mat->pData[2]) + tmp21->mat->pData[1];
		tmp3->mat->pData[4] = acceleration;
		tmp3->mat->pData[5] = omega_enc;
		X2->mat->pData[k] = tmp3->mat->pData[0];
		X2->mat->pData[15 + k] = tmp3->mat->pData[1];
		X2->mat->pData[30 + k] = tmp3->mat->pData[2];
		X2->mat->pData[45 + k] = tmp3->mat->pData[3];//TODO matinj
		X2->mat->pData[60 + k] = tmp3->mat->pData[4];
		X2->mat->pData[75 + k] = tmp3->mat->pData[5];
		X2->mat->pData[90 + k] = tmp3->mat->pData[6];
		m1 = matadd(m1, matsca(tmp3, Wm->mat->pData[k]));
	}
	tmp4 = matdup(m1, 15);
	Y1 = matsub(X2, tmp4);
	tmp5 = matdia(Wc);
	P1 = matadd(matmul(matmul(Y1, tmp5), mattra(Y1)), Q);
	memset(z1->mat->pData, 0, (z1->mat->numCols) * (z1->mat->numRows));
	for (int k = 0; k < X2->mat->numCols; k++) {
		tmp51 = matfra(X2, 0, X2->mat->numRows, k, k + 1);
		tmp6 = matmul(h, tmp51);
		Z1->mat->pData[k] = tmp6->mat->pData[0];
		Z1->mat->pData[15 + k] = tmp6->mat->pData[1]; //TODO matinj
		Z1->mat->pData[30 + k] = tmp6->mat->pData[2];
		Z1->mat->pData[45 + k] = tmp6->mat->pData[3];
		z1 = matadd(z1, matsca(tmp6, Wm->mat->pData[k]));
	}
	tmp7 = matdup(z1, 15);
	Z2 = matsub(Z1, tmp7);
	S = matadd(matmul(matmul(Z2, tmp5), mattra(Z2)), R); //S = Z2 * diag(Wc) * Z2.T + R;
	C = matmul(matmul(Y1, tmp5), mattra(Z2)); //C = Y1 * diag(Wc) * Z2.T;
	K = matmul(C, matinv(S)); //K = C * inv (S);
	x = matadd(m1, matmul(K, matsub(z, z1))); //m = m1 + K * (zmat - z1);
	P = matsub(P1, matmul(K, mattra(C))); //P = P1 - K * C.T;
}


void kalman_init() {
	matbeg(2600);
	Rlidar = ((1000.f * lidar_sdev_perc) * (1000.f * lidar_sdev_perc));
	Rodo = (enc_sdev * enc_sdev);
	Rgyro = (gyro_sdev * gyro_sdev);
	f = matini(7, 7, 'f');
	h = matini(4, 7, 'h');
	P = matini(7, 7, 'P');
	Q = matini(7, 7, 'Q');
	R = matini(4, 4, 'R');
	x = matini(7, 1, 'x');
	z = matini(4, 1, 'z');
	Wm = matini(1, 15, 'W');
	Wc = matini(1, 15, 'W');
	A = matini(7, 7, 'A');
	Y = matini(7, 7, 'Y');
	X = matini(7, 15, 'X');
	X2 = matini(7, 15, 'X');
	m1 = matini(7, 1, 'm');
	Y1 = matini(7, 15, 'Y');
	P1 = matini(7, 7, 'P');
	Z1 = matini(4, 15, 'Z');
	z1 = matini(4, 1, 'z');
	Z2 = matini(4, 15, 'Z');
	S = matini(4, 4, 'S');
	C = matini(7, 4, 'C');
	K = matini(7, 4, 'K');
	f->mat->pData[0 + 3] = dt;
	f->mat->pData[0 + 4] = dt * dt / 2;
	f->mat->pData[7 + 3] = dt;
	f->mat->pData[7 + 4] = dt * dt / 2;
	f->mat->pData[14 + 2] = 1;
	f->mat->pData[14 + 5] = dt;
	f->mat->pData[21 + 3] = 1;
	f->mat->pData[21 + 4] = dt;
	f->mat->pData[42 + 6] = 1;
	h->mat->pData[0] = 1;
	h->mat->pData[7 + 1] = 1;
	h->mat->pData[14 + 3] = 1;
	h->mat->pData[21 + 5] = 1;
	h->mat->pData[21 + 6] = 1;
	P->mat->pData[0] = (100 * lidar_sdev_perc) * (100 * lidar_sdev_perc);
	P->mat->pData[7 + 1] = (100 * lidar_sdev_perc) * (100 * lidar_sdev_perc);
	P->mat->pData[14 + 2] = alpha_sdev*alpha_sdev;
	P->mat->pData[21 + 3] = enc_sdev * enc_sdev / 100.f;
	P->mat->pData[28 + 4] = accel_sdev * accel_sdev / 100.f;
	P->mat->pData[35 + 5] = omega_sdev * omega_sdev;
	P->mat->pData[42 + 6] = bias_sdev * bias_sdev;
	Q->mat->pData[0] = Rlidar * dt * dt / 2;
	Q->mat->pData[7 + 1] = Rlidar * dt * dt / 2;
	Q->mat->pData[14 + 2] = alpha_sdev*alpha_sdev *dt;
	Q->mat->pData[21 + 3] = Rodo * dt;
	Q->mat->pData[28 + 4] = (accel_sdev * accel_sdev);
	Q->mat->pData[35 + 5] = omega_sdev * omega_sdev;
	Q->mat->pData[42 + 6] = 1;
	R->mat->pData[0] = Rlidar;
	R->mat->pData[4 + 1] = Rlidar;
	R->mat->pData[8 + 2] = Rodo;
	R->mat->pData[12 + 3] = Rgyro;
}

int micros=10000000;
int kalman_filter_loop_counter = 0;
void kalman_filter() {
	clock_timer_start_point();
	//FIXME użyć stałych przyspieszenia ziemskiego do kalibracji i wyliczenia tych faktorów
	float32_t a = get_acceleration('x')*ACC_SCALAR, o = get_angular_velocity('z')*GYRO_SCALAR;//target enc_omega
	f=matrix_stack[0], h=matrix_stack[1], P=matrix_stack[2],
			Q=matrix_stack[3], R=matrix_stack[4], x=matrix_stack[5],
			z=matrix_stack[6], Wm=matrix_stack[7], Wc=matrix_stack[8],
			A=matrix_stack[9], Y=matrix_stack[10], X=matrix_stack[11],
			X2=matrix_stack[12], m1=matrix_stack[13], Y1=matrix_stack[14],
			P1=matrix_stack[15], Z1=matrix_stack[16], z1=matrix_stack[17],
			Z2=matrix_stack[18], S=matrix_stack[19], C=matrix_stack[20], K=matrix_stack[21];
	f->mat->pData[28 + 4] = a;
	f->mat->pData[35 + 5] = o;
	z->mat->pData[0] = 0;//x
	z->mat->pData[1] = 0;//y
	z->mat->pData[2] = 0;//enc_v
	z->mat->pData[3] = 0;//gyr_omega
	matfre();
	float32_t acc_const = 0; // empiryczna stała
	unscented_kalman_filter(a-acc_const, o);
	matrix* secured_pointers[22] = {
				f, h, P, Q, R, x, z, Wm, Wc, A, Y, X, X2, m1, Y1, P1, Z1, z1, Z2, S, C, K
			};
	matcop(secured_pointers,22);
	micros = (int)clock_timer_end_point();
	//printf("%.1f %.1f %.1f %.1f %.1f %.1f %.1f\n", x->mat->pData[0],x->mat->pData[1],x->mat->pData[2],x->mat->pData[3],x->mat->pData[4],x->mat->pData[5],x->mat->pData[6]);
	kalman_filter_loop_counter++;
}


