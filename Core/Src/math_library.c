//
// Created by Łukasz Zelek on 17.03.2020.
//

#include <stdlib.h>
#include "math_library.h"
#include "stdio.h"

extern DMA_HandleTypeDef hdma_memtomem_dma2_stream1;
matrix *matrix_stack[30];
matrix *secured_matrix_stack[30];
matrix *temporary_matrix_stack[200];
float32_t *temporary_matrix_data;
int matrix_stack_size = 0;
int secured_matrix_stack_size = 0;
int secured_matrix_stack_iterator = 0;
int tmp_matrix_stack_size = 0;
arm_status errors = 0;
uint32_t allocated_bytes = 0;
uint32_t allocated_matrix_data;
int copy_status = 0;
int flag_finish_tmp_allocation = 0;
static void TransferError(DMA_HandleTypeDef *DmaHandle);
static void TransferComplete(DMA_HandleTypeDef *DmaHandle);

matrix* matpre(int n, int m) {
	matrix *res_mat;
	if(flag_finish_tmp_allocation == 0)
	{
		float32_t *new_tab = temporary_matrix_data + allocated_matrix_data;
		arm_matrix_instance_f32 *new_mat = malloc(sizeof(arm_matrix_instance_f32));
		allocated_bytes += sizeof(arm_matrix_instance_f32);
		if (new_mat == NULL)
			return NULL;
		arm_mat_init_f32(new_mat, n, m, new_tab);
		res_mat = (matrix*) malloc(sizeof(matrix));
		allocated_bytes += sizeof(matrix);
		if (res_mat == NULL)
			return NULL;
		temporary_matrix_stack[tmp_matrix_stack_size++] = res_mat;
		res_mat->mat = new_mat;
		allocated_matrix_data += n * m;
	}else{
		res_mat = temporary_matrix_stack[tmp_matrix_stack_size++];
	}
	return res_mat;
}

matrix* matini(int n, int m, char c) {
	matrix *res_mat;
	float32_t *new_tab = calloc(n * m, sizeof(float32_t));
	if (new_tab == NULL)
		return NULL;
	arm_matrix_instance_f32 *new_mat = malloc(sizeof(arm_matrix_instance_f32));
	allocated_bytes += sizeof(arm_matrix_instance_f32);
	if (new_mat == NULL)
		return NULL;
	arm_mat_init_f32(new_mat, n, m, new_tab);
	res_mat = (matrix*) malloc(sizeof(matrix));
	allocated_bytes += sizeof(matrix);
	if (res_mat == NULL)
		return NULL;
	res_mat->mat = new_mat;
	matrix_stack[matrix_stack_size++] = res_mat;
	res_mat->friendly_name = c;
	return res_mat;
}

matrix* matinv(matrix *A) {
	if (A->mat->numRows != A->mat->numCols) {
		printf("Error using matinv\n"
				"Matrix must be square.\n");
		return NULL;
	}
	int n = A->mat->numRows;
	matrix *res_mat = matpre(n, n);
	errors += arm_mat_inverse_f32(A->mat, res_mat->mat);
	res_mat->friendly_name = '^';
	return res_mat;
}

matrix* mattra(matrix *A) {
	int n = A->mat->numRows;
	int m = A->mat->numCols;
	matrix *res_mat = matpre(m, n);
	errors += arm_mat_trans_f32(A->mat, res_mat->mat);
	res_mat->friendly_name = '\'';
	return res_mat;
}

matrix* matsca(matrix *A, float32_t s) {
	int n = A->mat->numRows;
	int m = A->mat->numCols;
	matrix *res_mat = matpre(n, m);
	errors += arm_mat_scale_f32(A->mat, s, res_mat->mat);
	res_mat->friendly_name = '$';
	return res_mat;
}

matrix* matmul(matrix *A, matrix *B) {
	int an = A->mat->numRows;
	int am = A->mat->numCols;
	int bn = B->mat->numRows;
	int bm = B->mat->numCols;
	if (am != bn) {
		printf("Error using matmul\n"
				"Bad matrix dimensions\n"
				"Matrices should have am equal bn.\n");
		return NULL;
	}
	matrix *res_mat = matpre(an, bm);
	errors += arm_mat_mult_f32(A->mat, B->mat, res_mat->mat);
	res_mat->friendly_name = '*';
	return res_mat;
}

matrix* matadd(matrix *A, matrix *B) {
	int an = A->mat->numRows;
	int am = A->mat->numCols;
	int bn = B->mat->numRows;
	int bm = B->mat->numCols;
	if (an != bn || am != bm) {
		printf("Error using matadd\n"
				"Matrices should have identical dimensions.\n");
		return NULL;
	}
	matrix *res_mat = matpre(an, am);
	errors += arm_mat_add_f32(A->mat, B->mat, res_mat->mat);
	res_mat->friendly_name = '+';
	return res_mat;
}

matrix* matsub(matrix *A, matrix *B) {
	int an = A->mat->numRows;
	int am = A->mat->numCols;
	int bn = B->mat->numRows;
	int bm = B->mat->numCols;
	if (an != bn || am != bm) {
		printf("Error using matsub\n"
				"Matrices should have identical dimensions.\n");
		return NULL;
	}
	matrix *res_mat = matpre(an, am);
	errors += arm_mat_sub_f32(A->mat, B->mat, res_mat->mat);
	res_mat->friendly_name = '-';
	return res_mat;
}

/**
 * @brief Rozkład Choleskiego macierzy 3x3
 * @param A wskaźnik na macierz wejściową.
 */
matrix* matcho(matrix *A) {
	int n = A->mat->numRows;
	int m = A->mat->numCols;
	if (m != n) {
		printf("Error using matcho\n"
				"Matrix must be square and symetric.\n");
		return NULL;
	}
	matrix *res_mat = matpre(n, n);
	res_mat->friendly_name = '&';
	int n_chol = n;

	for (int i = 0; i < n_chol; i++) {
		for (int j = 0; j <= i; j++) {
			float32_t sum = 0;
			if (j == i) {
				for (int k = 0; k < j; k++)
					sum += powf(res_mat->mat->pData[j * n_chol + k], 2);
				res_mat->mat->pData[j * n_chol + j] = sqrtf(
						A->mat->pData[j * n_chol + j] - sum);
			} else {
				for (int k = 0; k < j; k++)
					sum += (res_mat->mat->pData[i * n_chol + k]
							* res_mat->mat->pData[j * n_chol + k]);
				res_mat->mat->pData[i * n_chol + j] = (A->mat->pData[i * n_chol
						+ j] - sum) / res_mat->mat->pData[j * n_chol + j];
			}
		}
	}

	return res_mat;
}

matrix* matfra(matrix *A, int n_start, int n_end, int m_start, int m_end) {
	int n = n_end - n_start;
	int m = m_end - m_start;
	matrix *res_mat = matpre(n, m);
	res_mat->friendly_name = '#';
	for (int i = 0; i < n; i++)
		for (int j = 0; j < m; j++)
			res_mat->mat->pData[i * m + j] = A->mat->pData[(i + n_start)
					* (A->mat->numCols) + (j + m_start)];
	return res_mat;
}

matrix* matdia(matrix *v) {
	int n = (v->mat->numRows) * (v->mat->numCols);
	matrix *res_mat = matpre(n, n);
	res_mat->friendly_name = '\\';
	for (int i = 0; i < n; i++)
		res_mat->mat->pData[i * n + i] = v->mat->pData[i];
	return res_mat;
}

/*v - column vector*/
matrix* matdup(matrix *v, int k) {
	int n = (v->mat->numRows) * (v->mat->numCols);
	matrix *res_mat = matpre(n, k);
	res_mat->friendly_name = '|';
	for (int i = 0; i < n; i++)
		for (int j = 0; j < k; j++)
			res_mat->mat->pData[i * k + j] = v->mat->pData[i];
	return res_mat;
}

matrix* matmer(matrix *A, matrix *B) {
	int n = A->mat->numRows;
	int m1 = A->mat->numCols;
	int m2 = B->mat->numCols;
	int m = m1 + m2;
	matrix *res_mat = matpre(n, m);
	res_mat->friendly_name = '_';
	for (int i = 0; i < n; i++)
		for (int j = 0; j < m1; j++)
			res_mat->mat->pData[i * m + j] = A->mat->pData[i * m1 + j];
	for (int i = 0; i < n; i++)
		for (int j = 0; j < m2; j++)
			res_mat->mat->pData[m1 + i * m + j] = B->mat->pData[i * m2 + j];
	return res_mat;
}

/**
 * @brief Funkcja wypisująca macierz, do debugowania.
 * @param matrix : wskaźnik na wypisywaną macierz.
 */
void matprt(matrix *A) {
	printf("%c[%ix%i] = \n", A->friendly_name, A->mat->numRows,
			A->mat->numCols);
	for (int i = 0; i < A->mat->numRows; i++) {
		for (int j = 0; j < A->mat->numCols; j++)
			printf("%7.2lf ",
					(double) A->mat->pData[i * (A->mat->numCols) + j]);
		printf("\n");
	}
}

void matfre() {
	tmp_matrix_stack_size = 0;
}

void matcop(matrix *secured[], int size) {
	flag_finish_tmp_allocation=1;
	for (int i = 0; i < size; i++)
		secured_matrix_stack[i] = secured[i];
	secured_matrix_stack_size = size;
	secured_matrix_stack_iterator = 1;
	copy_status = 1;
	if (HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream1,
			(uint32_t) &(secured[0]->mat->pData[0]),
			(uint32_t) &(matrix_stack[0]->mat->pData[0]),
			(secured[0]->mat->numCols * secured[0]->mat->numRows)) != HAL_OK) {
		Error_Handler();
	}
}

void matbeg(uint32_t count) {
	HAL_DMA_RegisterCallback(&hdma_memtomem_dma2_stream1,
			HAL_DMA_XFER_CPLT_CB_ID, TransferComplete);
	HAL_DMA_RegisterCallback(&hdma_memtomem_dma2_stream1,
			HAL_DMA_XFER_ERROR_CB_ID, TransferError);
	temporary_matrix_data = calloc(count, sizeof(float32_t));
	if (temporary_matrix_data == NULL)
		Error_Handler();
}

void matfin() {
	if (temporary_matrix_data != NULL)
		free(temporary_matrix_data);
}

static void TransferComplete(DMA_HandleTypeDef *DmaHandle) {
	if (secured_matrix_stack_iterator < secured_matrix_stack_size) {
		int i = secured_matrix_stack_iterator;
		if (HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream1,
				(uint32_t) &(secured_matrix_stack[i]->mat->pData[0]),
				(uint32_t) &(matrix_stack[i]->mat->pData[0]),
				(secured_matrix_stack[i]->mat->numCols
						* secured_matrix_stack[i]->mat->numRows)) != HAL_OK) {
			Error_Handler();
		}
		secured_matrix_stack_iterator++;
	} else
		copy_status = 0;
}

static void TransferError(DMA_HandleTypeDef *DmaHandle) {
	printf("DMA COPY MEMTOMEM error\n");
}
