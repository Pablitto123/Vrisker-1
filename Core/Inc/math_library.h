/*
 * math_library.h
 *
 *  Created on: Nov 1, 2020
 *      Author: Łukasz Zelek
 */

#ifndef INC_MATH_LIBRARY_H_
#define INC_MATH_LIBRARY_H_

#include <main.h>
#include <math.h>

matrix* matini(int n, int m, char c); /* matrix initialization */
matrix* matinv(matrix* A); /* matrix inversion */
matrix* mattra(matrix* A); /* matrix transposition */
matrix* matsca(matrix* A, float32_t s); /* matrix scale */
matrix* matmul(matrix* A, matrix* B); /* matrix multiplication */
matrix* matadd(matrix* A, matrix* B); /* matrix addition */
matrix* matsub(matrix* A, matrix* B); /* matrix subtraction */
matrix* matcho(matrix *A); /* matrix cholesky decomposition */
matrix* matfra(matrix* A, int n_start, int n_end, int m_start, int m_end); /* matrix fragment */
matrix* matdia(matrix* v); /* matrix diagonal */
matrix* matdup(matrix* v, int k); /* matrix duplication */
matrix* matmer(matrix* A, matrix* B); /* matrix merge */
void matprt(matrix* A); /* matrix print */
void matfre(); /* matrix free allocated memory by library */
void matinj(matrix* A, int n_start, int n_end, int m_start, int m_end, matrix *v); /* matrix inject */
void matfin();
void matbeg(uint32_t count);
void matcop(matrix* secured[], int size);

#endif /* INC_MATH_LIBRARY_H_ */
