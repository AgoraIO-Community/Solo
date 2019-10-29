/***********************************************************************
Copyright (c) 2006-2012, Skype Limited. All rights reserved. 
Redistribution and use in source and binary forms, with or without 
modification, (subject to the limitations in the disclaimer below) 
are permitted provided that the following conditions are met:
- Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.
- Redistributions in binary form must reproduce the above copyright 
notice, this list of conditions and the following disclaimer in the 
documentation and/or other materials provided with the distribution.
- Neither the name of Skype Limited, nor the names of specific 
contributors, may be used to endorse or promote products derived from 
this software without specific prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED 
BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
CONTRIBUTORS ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND 
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF 
USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***********************************************************************/

#include "SKP_Silk_main_FLP.h" 
#include "SKP_Silk_tuning_parameters.h"

/**********************************************************************
 * LDL Factorisation. Finds the upper triangular matrix L and the diagonal
 * Matrix D (only the diagonal elements returned in a vector)such that 
 * the symmetric matric A is given by A = L*D*L'.
 **********************************************************************/
void SKP_Silk_LDL_FLP(
    SKP_float           *A,      /* (I/O) Pointer to Symetric Square Matrix */
    SKP_int             M,       /* (I) Size of Matrix */
    SKP_float           *L,      /* (I/O) Pointer to Square Upper triangular Matrix */
    SKP_float           *Dinv    /* (I/O) Pointer to vector holding the inverse diagonal elements of D */
);

/**********************************************************************
 * Function to solve linear equation Ax = b, when A is a MxM lower 
 * triangular matrix, with ones on the diagonal.
 **********************************************************************/
void SKP_Silk_SolveWithLowerTriangularWdiagOnes_FLP(
    const SKP_float     *L,     /* (I) Pointer to Lower Triangular Matrix */
    SKP_int             M,      /* (I) Dim of Matrix equation */
    const SKP_float     *b,     /* (I) b Vector */
    SKP_float           *x      /* (O) x Vector */  
);

/**********************************************************************
 * Function to solve linear equation (A^T)x = b, when A is a MxM lower 
 * triangular, with ones on the diagonal. (ie then A^T is upper triangular)
 **********************************************************************/
void SKP_Silk_SolveWithUpperTriangularFromLowerWdiagOnes_FLP(
    const SKP_float     *L,     /* (I) Pointer to Lower Triangular Matrix */
    SKP_int             M,      /* (I) Dim of Matrix equation */
    const SKP_float     *b,     /* (I) b Vector */
    SKP_float           *x      /* (O) x Vector */  
);

/**********************************************************************
 * Function to solve linear equation Ax = b, when A is a MxM  
 * symmetric square matrix - using LDL factorisation
 **********************************************************************/
void SKP_Silk_solve_LDL_FLP(
          SKP_float                 *A,                 /* I/O  Symmetric square matrix, out: reg.      */
    const SKP_int                   M,                  /* I    Size of matrix                          */
    const SKP_float                 *b,                 /* I    Pointer to b vector                     */
          SKP_float                 *x                  /* O    Pointer to x solution vector            */
)
{
    SKP_int   i;
    SKP_float L[    MAX_MATRIX_SIZE ][ MAX_MATRIX_SIZE ];
    SKP_float T[    MAX_MATRIX_SIZE ];
    SKP_float Dinv[ MAX_MATRIX_SIZE ]; // inverse diagonal elements of D

    SKP_assert( M <= MAX_MATRIX_SIZE );

    /***************************************************
    Factorize A by LDL such that A = L*D*(L^T),
    where L is lower triangular with ones on diagonal
    ****************************************************/
    SKP_Silk_LDL_FLP( A, M, &L[ 0 ][ 0 ], Dinv );

    /****************************************************
    * substitute D*(L^T) = T. ie:
    L*D*(L^T)*x = b => L*T = b <=> T = inv(L)*b
    ******************************************************/
    SKP_Silk_SolveWithLowerTriangularWdiagOnes_FLP( &L[ 0 ][ 0 ], M, b, T );

    /****************************************************
    D*(L^T)*x = T <=> (L^T)*x = inv(D)*T, because D is 
    diagonal just multiply with 1/d_i
    ****************************************************/
    for( i = 0; i < M; i++ ) {
        T[ i ] = T[ i ] * Dinv[ i ];
    }
    /****************************************************
    x = inv(L') * inv(D) * T
    *****************************************************/
    SKP_Silk_SolveWithUpperTriangularFromLowerWdiagOnes_FLP( &L[ 0 ][ 0 ], M, T, x );
}

void SKP_Silk_SolveWithUpperTriangularFromLowerWdiagOnes_FLP(
    const SKP_float     *L,     /* (I) Pointer to Lower Triangular Matrix */
    SKP_int             M,      /* (I) Dim of Matrix equation */
    const SKP_float     *b,     /* (I) b Vector */
    SKP_float           *x      /* (O) x Vector */  
)
{
    SKP_int   i, j;
    SKP_float temp;
    const SKP_float *ptr1;
    
    for( i = M - 1; i >= 0; i-- ) {
        ptr1 =  matrix_adr( L, 0, i, M );
        temp = 0;
        for( j = M - 1; j > i ; j-- ) {
            temp += ptr1[ j * M ] * x[ j ];
        }
        temp = b[ i ] - temp;
        x[ i ] = temp;      
    }
}

void SKP_Silk_SolveWithLowerTriangularWdiagOnes_FLP(
    const SKP_float     *L,     /* (I) Pointer to Lower Triangular Matrix */
    SKP_int             M,      /* (I) Dim of Matrix equation */
    const SKP_float     *b,     /* (I) b Vector */
    SKP_float           *x      /* (O) x Vector */  
)
{
    SKP_int   i, j;
    SKP_float temp;
    const SKP_float *ptr1;
    
    for( i = 0; i < M; i++ ) {
        ptr1 =  matrix_adr( L, i, 0, M );
        temp = 0;
        for( j = 0; j < i; j++ ) {
            temp += ptr1[ j ] * x[ j ];
        }
        temp = b[ i ] - temp;
        x[ i ] = temp;
    }
}

void SKP_Silk_LDL_FLP(
    SKP_float           *A,      /* (I/O) Pointer to Symetric Square Matrix */
    SKP_int             M,       /* (I) Size of Matrix */
    SKP_float           *L,      /* (I/O) Pointer to Square Upper triangular Matrix */
    SKP_float           *Dinv    /* (I/O) Pointer to vector holding the inverse diagonal elements of D */
)
{
    SKP_int i, j, k, loop_count, err = 1;
    SKP_float *ptr1, *ptr2;
    double temp, diag_min_value;
    SKP_float v[ MAX_MATRIX_SIZE ], D[ MAX_MATRIX_SIZE ]; // temp arrays

    SKP_assert( M <= MAX_MATRIX_SIZE );

    diag_min_value = FIND_LTP_COND_FAC * 0.5f * ( A[ 0 ] + A[ M * M - 1 ] ); 
    for( loop_count = 0; loop_count < M && err == 1; loop_count++ ) {
        err = 0;
        for( j = 0; j < M; j++ ) {
            ptr1 = matrix_adr( L, j, 0, M );
            temp = matrix_ptr( A, j, j, M ); // element in row j column j
            for( i = 0; i < j; i++ ) {
                v[ i ] = ptr1[ i ] * D[ i ];
                temp  -= ptr1[ i ] * v[ i ];
            }
            if( temp < diag_min_value ) {
                /* Badly conditioned matrix: add white noise and run again */
                temp = ( loop_count + 1 ) * diag_min_value - temp;
                for( i = 0; i < M; i++ ) {
                    matrix_ptr( A, i, i, M ) += ( SKP_float )temp;
                }
                err = 1;
                break;
            }
            D[ j ]    = ( SKP_float )temp;
            Dinv[ j ] = ( SKP_float )( 1.0f / temp );
            matrix_ptr( L, j, j, M ) = 1.0f;
            
            ptr1 = matrix_adr( A, j, 0, M );
            ptr2 = matrix_adr( L, j + 1, 0, M);
            for( i = j + 1; i < M; i++ ) {
                temp = 0.0;
                for( k = 0; k < j; k++ ) {
                    temp += ptr2[ k ] * v[ k ];
                }
                matrix_ptr( L, i, j, M ) = ( SKP_float )( ( ptr1[ i ] - temp ) * Dinv[ j ] );
                ptr2 += M; // go to next column
            }   
        }
    }
    SKP_assert( err == 0 );
}

