//
// <GMat.cpp>
//
// Lightweight float matrix struct for use on GPU side see header for details
//
#include <math.h>
#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include "GMat.cuh"

//
//-----------------------------------------------------------------------
// Following functions are not GMat class members but using GMat Objects
//
__device__ int inverse_GMat(GMat& A, GMat& B)
{
//
//  Inverts a general floating point precision matrix
//
//  USER NOTES:
//  
//  This function takes A as input and returns its inverse in B.
//  The method works best for a general square matrix of any order
//  Matrices of order less than 4 are solved explicitly, higher order
//  matrices are solved using Gaussian elimination with partial pivoting 
//
//  Note unlike the FMat version this does overwrite the input A
//  Both matrices must be correctly allocated before the call
//
//  Returns 1 if all went well 
//          0 if it fails due to non-square or NULL 
//         -1 if its singular or ill-conditioned
//
// Locals
//
	int I, J, NM1, K, KP1, M, JJ, N;
	float T, SUM, Trc, nabla, det;
//
	N = A.rows();
//
// Check for square matrix
//
	if ( (N != A.cols()) || (N==0) ) {
    	return 0;
	}
//
	Trc = A.Trace();
    
	nabla = fabs(Trc)*FLT_MIN;
    switch (N)
    {
    case 1:
        if(fabs(A(0,0)) <= nabla ){
			return -1;
		}
		B(0,0) = 1.0/A(0,0);
        break;
    case 2:
		det = A(0,0)*A(1,1)-A(1,0)*A(0,1);
		if(fabs(det) < nabla){
			return -1;
		}
		B(0,0) =  A(1,1)/det;
		B(1,1) =  A(0,0)/det;
		B(0,1) = -A(0,1)/det;
		B(1,0) = -A(1,0)/det;
        break;
    case 3:
    
        det =  A(0,0)*( A(1,1)*A(2,2) - A(2,1)*A(1,2) );
        det += A(0,1)*( A(2,0)*A(1,2) - A(1,0)*A(2,2) );
        det += A(0,2)*( A(1,0)*A(2,1) - A(2,0)*A(1,1) );

//  small det = ill-conditioned or singular
        if(fabs(det) <= nabla ){
            return -1;
        }
        B.Zero();

// generate the Adjoint = Transpose of the cofactor matrix    
        B(0,0) = A(1,1)*A(2,2) - A(2,1)*A(1,2);
        B(1,0) = A(2,0)*A(1,2) - A(1,0)*A(2,2);
        B(2,0) = A(1,0)*A(2,1) - A(2,0)*A(1,1);

        B(0,1) = A(2,1)*A(0,2) - A(0,1)*A(2,2);
        B(1,1) = A(0,0)*A(2,2) - A(2,0)*A(0,2);
        B(2,1) = A(2,0)*A(0,1) - A(0,0)*A(2,1);

        B(0,2) = A(0,1)*A(1,2) - A(1,1)*A(0,2);
        B(1,2) = A(1,0)*A(0,2) - A(0,0)*A(1,2);
        B(2,2) = A(0,0)*A(1,1) - A(1,0)*A(0,1);

// and divide the Adjoint by the determinant    
        for(I=0;I<3;I++){
            for(J=0;J<3;J++){
                B(I,J) = B(I,J)/det;
            }
        }
        break;

// N > 3
    default:
// initialize the rhs
//
        int pIPVT[10];      // warning this could be expensive with many threads
    	B.Identity();
        NM1=N-1;
//
// first half
//
        for(K=0; K<NM1; K++){
        	KP1=K+1;
            M=K;
            for(I=KP1;I<N; I++){
            	if(fabs(A(I,K)) > fabs(A(M,K))) {
    				M=I;
    			}
           }
//
            pIPVT[K]=M;

            T=A(M,K);
            A(M,K)=A(K,K);
            A(K,K)=T;

            if(fabs(T) < nabla ) {
    			continue;
    		}

            for(I=KP1; I<N; I++){
              A(I,K) = -A(I,K)/T;
    		}

            for(J=KP1; J<N; J++){
    			T=A(M,J);
    			A(M,J)=A(K,J);
    			A(K,J)=T;

    			if(fabs(T) < nabla ) {
    				continue;
    			}

    			for(I=KP1; I<N; I++){
              		A(I,J)=A(I,J)+A(I,K)*T;
    			}
    		}
    	}
//
    	Trc = fabs(A(NM1,NM1));
        if( Trc <= nabla ){
// must be ill-conditioned or rank-deficient
    		return -1;
    	}
//
// 2nd half
//
    	for(JJ=0; JJ<N; JJ++){
        	for(K=0; K<NM1; K++){
    			KP1=K+1;
    			M = pIPVT[K];
    			T=B(M,JJ);
    			B(M,JJ)=B(K,JJ);
    			B(K,JJ)=T;
    			for(I=KP1; I<N; I++){
    				B(I,JJ)=B(I,JJ)+A(I,K)*T;
    			}
    		}
    	}

        for(JJ=0; JJ<N; JJ++){

            B(NM1,JJ) = B(NM1,JJ)/A(NM1,NM1);

            for(I=NM1-1; I > -1; I--){
    			SUM=0.0;
    			for(J=I+1; J<N; J++){
    				SUM+= A(I,J)*B(J,JJ);
    			}
    			B(I,JJ)=(B(I,JJ)-SUM)/A(I,I);
    		}
    	}
        break;
    }
    return 1;   // sucessfull paths 
}

__device__ int Jacobi_GMat(GMat& C, GMat& V, GMat& W, GMat& R)
{
// Jacobi decompostition of a symmetric matrix
//
// Determine Eigen-values and vectors for a positive-definite, symmetric matrix
// using Jacobi iteration, the matrices should already be allocated.
// Typical applications include [3x3] or higher covariance matrices
// however will also solve dynamic vibration probems if damping is neglected
//
// C           Input Matrix which on Exit contains the Eigen-values on the lead diagonal
// V           Eigen-vectors in asscending order of Eigen-values
// W           Work matrix
// R           Rotation Matrix
//
// returns number of iterates taken if OK or -1 if it failed
// This version is simplified relative to the FMat version
//
    int i,j,k,m,n,p, iTest;
    int N;
    float sum, deno;

    N = C.rows();
    V.Identity();
    W.Zero();

    const float Tr = fabs(C.Trace());
    const float eta = FLT_MIN*Tr;  // this only works for Gentle
    const float nabla = FLT_EPSILON;
//
// Jacobi iteration is usually pretty rapid for small matrices
//
    for (k=0;k<JACOBI_LIMIT;k++){

        for(i=0;i<N-1;i++){
            for(j=i+1;j<N;j++){

                R.Identity();

// no point trying to zero-out an already zero element
                if(fabs(C(i,j))<eta ){
                    C(i,j) = 0.0;
                    continue;
                }
//
// This algorithm is crippled if the lead diagonal contains equal values
// return -1
//
                deno = C(i,i) - C(j,j);
                if(fabs(deno) < nabla*fabs(C(i,i)) ) {
                    return -1;
                }
//
// This algorithm version from Matrix Algebra by Gentle converges very rapidly 
//
                const float tan2T = 2.0*C(i,j)/deno;
                const float deno2 = sqrtf(1.0 + tan2T*tan2T);
                const float tanT = tan2T/(1.0 + deno2);
                const float cosT = 1.0/sqrtf(1.0 + tanT*tanT);
                const float sinT = cosT*tanT;
 
                R(i,i) = cosT;
                R(i,j) = -sinT;
                R(j,i) = sinT;
                R(j,j) = cosT;

                for(m=0;m<N;m++){
                    for(n=0;n<N;n++){
                        W(m,n) = 0.0;
                        for(p=0;p<N;p++){
                            W(m,n) += C(m,p)*R(p,n);
                        }
                    }
                }
// C = R'.C.R
                for(m=0;m<N;m++){
                    for(n=0;n<N;n++){
                        C(m,n) = 0.0;
                        for(p=0;p<N;p++){
                            C(m,n) += R.Trans(m,p)*W(p,n);
                        }
                    }
                }
// W = V.R Accumulate Transform in W
                for(m=0;m<N;m++){
                    for(n=0;n<N;n++){
                        W(m,n) = 0.0;
                        for(p=0;p<N;p++){
                            W(m,n) += V(m,p)*R(p,n);
                        }
                    }
                }
// back copy to V for next round
                for(m=0;m<N;m++){
                    for(n=0;n<N;n++){
                        V(m,n) = W(m,n);
                    }
                }
            }
        }
//
// test for convergence
//
        sum = 0.0;
        for(i=0;i<N-1;i++){
            for(j=i+1;j<N;j++){
                sum += C(i,j)*C(i,j);
            }
        }

        if(sum<eta){
//
// number of iterates taken
// all other operations taken out
            iTest = k+1;
            return iTest;    
        }

    }
    return -1; // iteration exhausted

}

__device__ int Cholesky_GMat(GMat& A, GMat& Decomp)
{
//
// Decompose a symmetric positive definite matrix to Lower Cholesky form
// This is more efficient than LU decomposition for symm +ve def matrices
//
    int i,j,k,N;
    float sum;
//
// Check for square matrix
//
    N = A.rows();
	if ( (N != A.cols()) || (N==0) ) {
    	return 0;
	}

// Copy A into Decomp

    for(i=0;i<N;i++){
        for(j=0;j<N;j++){
            Decomp(i,j) = A(i,j);
        }
    }
 
    for(i=0;i<N;i++){
        for(j=i;j<N;j++){
            sum = Decomp(i,j);
            for(k=i-1; k>=0; k--){
                sum -= Decomp(i,k)*Decomp(j,k);
            }
            if(i==j){
                if(sum<=0.0){
                    return -1;  // failed due to rounding error or not +ve definite
                }
                Decomp(i,i) = sqrt(sum);
            } else {
                Decomp(j,i) = sum/Decomp(i,i);
            }
        }
    }
// cleans out the upper triangle leaving 
// Decomp contained in the lower 
    for(i=0;i<N;i++){
        for(j=0;j<i;j++){
            Decomp(j,i)=0.0;
        }
    }
    return 1;
}

__device__ int CholSolve_GMat(GMat& Decomp, float* pX, float* pB)
{
//
// Uses the (Decomp)osed Lower diagonal Cholesky form to solve for X in A.X = B
// Decomp must already be decomposed using the above Cholesky_GMat() function
// pB is a solution vector to the problem A.pX = pB and of compatible length
// for the given Decomp. Both this and the above were transcribed
// from Numerical Recipies 3rd Edition Press et al. 
//
// Note this GPU version is different to the FMat version in that it uses
// float * vectors pX and bB both of which must be correctly initialised in
// the calling kernel
//
    int N, i,k;
    N = Decomp.rows();
    float sum;
//

    for(i=0;i<N;i++){
        sum = pB[i];
        for(k=i-1; k>=0; k--){
            sum -= Decomp(i,k)*pX[k];      // tricksy code! it looks like pX[i] it is being used before set
        }                                  // but it won't happen first time through because k=0>i-1
        pX[i]=sum/Decomp(i,i);
    }

    for(i=N-1; i>=0; i--){
        sum=pX[i];
        for(k=i+1; k<N; k++){
            sum -= Decomp(k,i)*pX[k];
        }
        pX[i] = sum/Decomp(i,i);
    }
    return 1;
}


__device__ int ShellSort_GPU(float* A2Sort, int* pIdx, int N)
{
//
// Sorts A2Sort into ascending order and concurrently matches these swaps
// in pIdx Where pIdx starts as a zero-indexed ascending integer array
// This enables sorting of eigen-vectors from the sorted values but 
// for this version only real eigen-values allowed
// A2Sort and pIdx must be correctly allocated and at least equal to N
// elements before this call. 
//
// The code was transcribed from Numerical Recipies Press et al.
// and extended to provide a sorted pIdx array
//
    int i,j,ij,inc,isav,ret=0;
    float v;
//
// setup the Idx array
//
    for(i=0;i<N;i++){
        pIdx[i] = i;
    }

    inc=0;
    do {
        inc*=3;
        inc++;
    } while(inc <= N);

    do {
        inc /=3;
        for(i=inc;i<N;i++){
            v=A2Sort[i];
            isav = pIdx[i];
            j=i;
            ij = j-inc;
            while(A2Sort[ij] > v){
                ret+=1;
                A2Sort[j] = A2Sort[ij];
                pIdx[j] = pIdx[ij];
                j -= inc;
                if(j<inc){
                    break;
                }
                ij = j-inc;
            }
            A2Sort[j]=v;
            pIdx[j]=isav;
        }
    } while(inc>0);
    return ret;
}


