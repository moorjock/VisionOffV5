//
// <FMat.cpp>
//
// Lightweight float matrix class see header for details
//
#include <math.h>
#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include "FMat.h"

// used exclusively in FindNextLine() though it can be used anywhere
// a char buffer is needed

char ReadBuf[READ_BUF_SIZE];

FVec::FVec(int Size)
{
    m_pData = NULL;
    m_Len = 0;
    m_Allocated = 0;
    if(Size>0){
        ZeroInit(Size);
    }
}

FVec::FVec(float* pDat, int Size)
{
    SetData(pDat,Size);
}

FVec::FVec( const FVec& Cpy)
{   // this will only copy the active sub-vector
    Copy(Cpy);
}

void FVec::SetData(float* pDat, int Size)
{
    m_pData = pDat;
    m_Len = Size;
    m_Allocated = -1;
}

FVec& FVec::operator=(const FVec& Src)
{
	Copy(Src);
    return *this;
}

FVec& FVec::Copy(const FVec& Src)
{
    int i;
    ZeroInit(Src.Size());
    for(i=0;i<m_Len;i++){
        m_pData[i]=Src(i);
    }
    return *this;
}

FVec::~FVec()
{
    if(m_Allocated>0 && m_pData){
        delete m_pData;
    }
    m_pData=NULL;
    m_Allocated=0;
    m_Len = 0;
}

void FVec::ZeroInit(int Size)
{
    int k;

    if(m_pData!=NULL){
        if(Size>m_Allocated){
            ReSize(Size);
        }
    }else{
        m_Allocated=0;
        ReSize(Size);
    }
    for(k=0;k<Size;k++){
        m_pData[k]=0.0F;
    }

}

int FVec::ReSize(int Size)
{
//
// will resize a vector upwards if size is greater than allocated the
// original data will be lost. However can only do this if m_Allocated >=0
//
    if(m_Allocated<0){
        m_Len = Size;
        return m_Len;
    }
    if(m_Allocated>0){
        if(Size > m_Allocated){  // if the new vector is longer than previously allocated
            delete m_pData;      // delete it and allocate a new one
            m_Allocated = Size;
		    m_pData = new float[m_Allocated];
        }
    } else {
// This is the first allocation
        m_Allocated = Size;
        m_pData = new float[m_Allocated]; 
    }
    m_Len = Size;
    return m_Len;
}

FVec& FVec::Copy(const float* pVec, int Sz)
{
// creates a copy from an existing c-style vector
// it must be correctly initialised and sized (no checks)
    int i;
    if(Sz){
        ZeroInit(Sz);
        for(i=0;i<Sz;i++){
            operator()(i)=pVec[i];
        }
    }
    return *this;
}

int FVec::ReadVectorFromFile(FILE* pFile, int Size)
{
//
// Most of the work is done below In VectorFromString
// The expectation is that the entire vector will be contained on 
// the next none-comment single line where the entries are 
// separated by spaces or commas
//
	FString Line;
	int i;
	ZeroInit(Size);
	i=FindNextLine(pFile, Line);
	if (i>0) {
		i=VectorFromString(Line, Size);
	}
	return i;

}

void FVec::printVector(const char* pString, FILE* pFile)
{
//
// Output to screen or file, the vector will write out 10 columns wide and double back
// if the output is longer
//
    int i,jj,j, N, Lines, Rem=0;

    N = Size();
    Lines = N/10;
    if(Lines>0){
        Rem = N%10; 
    } else {
        Rem = N;
    }  
    if(pFile!=NULL){
        if(pString!=NULL){
            fprintf(pFile,"\n %s\n",pString);
        }
        for(i=0;i<Lines;i++){
            for(j=0;j<10;j++){
                jj = i*10 + j;
                fprintf(pFile," %#11.4E,",operator()(jj));
            }
            fprintf(pFile,"\n");
        }
        if(Rem>0){
            for(i=0;i<Rem;i++){
                jj = Lines*10 + i;
                fprintf(pFile," %#11.4E,",operator()(jj));            
            }
        }
        fprintf(pFile,"\n");

    } else {
        if(pString!=NULL){
            printf("\n %s\n",pString);
        }
        for(i=0;i<Lines;i++){
            for(j=0;j<10;j++){
                jj = i*10 + j;
                printf(" %#11.4E, ",operator()(jj));
            }
            printf("\n");
        }
        if(Rem>0){
            for(i=0;i<Rem;i++){
                jj = Lines*10 + i;
                printf(" %#11.4E, ",operator()(jj));            
            }
        }
        printf("\n");
    }
}

int FVec::VectorFromString(const char* pString, int Num)
{
    int i, Index=0, Count=0;
    FString Word;
    float val;
//
    ZeroInit(Num);
    Index=0;
	for(i=0;i<Num;i++){
		FindNextWord(pString, Word, Index);
    	Count++;
		if(Index>-2){
			val = STR_2_FLOAT(Word);
			operator()(i) = val;
		}
		else{
			Index=-2;	// format problem encountered
			break;
    	    Count--;
		}
	}
    return Count;
}

//
//-------------------------------------------------------------------------------------------------------------
//
FMat::FMat(int i, int j)
{
    m_pData = NULL;
    m_MaxI = 0;
    m_MaxJ = 0;
    m_Allocated = 0;
    int size = i*j;
    if(size>0){
        ZeroInit(i,j);
    }
}

FMat::FMat(float* pDat, int i, int j)
{
//
// to keep things simple this constructor takes a copy
// but does not allocate memory (or clean it up)
    SetData(pDat,i,j);
}

void FMat::SetData(float* pDat, int i, int j)
{
    int size = i*j;
    if(size>0 && (pDat!=NULL) ){
        m_MaxI = i;
        m_MaxJ = j;
        m_Allocated=-1;              // we did not allocate
        m_pData= pDat;
    }
}

FMat::FMat( const FMat& Cpy)
{
    Copy(Cpy);
}

FMat::~FMat()
{
    if(m_Allocated>0){
        delete m_pData;
    }
    m_pData=NULL;
    m_Allocated=0;
    m_MaxI = 0;
    m_MaxJ = 0;
}

FMat& FMat::operator=(const FMat& A)
{
	return Copy(A);
}

FMat& FMat::Copy(const FMat& Src)
{
    int i,j;
    ReSize(Src.rows(),Src.cols());
    for(i=0;i<rows();i++){
        for(j=0;j<cols();j++){
            operator()(i,j)=Src(i,j);
        }
    }
    return *this;
}

void FMat::ZeroInit(int i, int j)
{
    int k, size;
    size = i*j;
    if(m_pData!=NULL){
        if(size>m_Allocated){
            ReSize(i,j);
        }
    }else{
        m_Allocated=0;
        ReSize(i,j);
    }
    for(k=0;k<size;k++){
        m_pData[k]=0.0F;
    }
}		// sets all elements to 0 resizing if needed


int FMat::ReSize(int i, int j)
{
    int Size;
    m_MaxI = i;
    m_MaxJ = j;
    Size = m_MaxI*m_MaxJ;
    if(m_Allocated<0){
        return Size;
    }
    if(m_pData!=NULL){
        if(Size > m_Allocated){  // if the new vector is longer than previously allocated
            delete m_pData;      // delete it and allocate a new one
            m_Allocated = Size;
		    m_pData = new float[m_Allocated];
        }
    } else {
// This is the first allocation
        m_Allocated = Size;
        m_pData = new float[m_Allocated]; 
    }
    return Size;
}

void FMat::Identity(int i, int j)
{
// Forms an Identity Matrix if the matrix is not square the 
// it will place 1's in the upper or left square sub-matrix
    int k, m;

    ZeroInit(i,j);
    m = my_min(i,j);
    for(k=0;k<m;k++){
        operator()(k,k) = 1.0;
    }   
}

int	FMat::ReadFromTextFile(FILE* pFile)
{
//
// WARNING! This function ONLY reads data into a pre-allocated
// matrix space. The File being read needs also to contain data
// in the correct *.csv format comment lines indicated with '#'
//
	FString Line, Word;
	int i, j, idx;
	float val;
//
	for( i=0; i<rows(); i++ ){
		if(FindNextLine(pFile,Line)){;
			idx=0;
			for( j=0; j<cols(); j++ ){
				FindNextWord(Line,Word,idx);
				if(idx==-2){
					return 0;
				}
				val = STR_2_FLOAT(Word);
				operator()(i,j) = val;
			}
		}
		else{
			return 0;
		}
	}
	return 1;
}

void FMat::printMatrix(const char* pName, FILE* pFile)
{
    int i,j;
    if(pFile!=NULL){
        if(pName!=NULL){
            fprintf(pFile,"\n %s\n",pName);
        }
        for(i=0;i<m_MaxI;i++){
            for(j=0;j<m_MaxJ;j++){
                fprintf(pFile," %#11.4E,",operator()(i,j));
            }
            fprintf(pFile,"\n");
        }
        fprintf(pFile,"\n");
    } else {
        if(pName!=NULL){
            printf("\n %s\n",pName);
        }
        for(i=0;i<m_MaxI;i++){
            for(j=0;j<m_MaxJ;j++){
                printf(" %#11.4E,",operator()(i,j));
            }
            printf("\n");
        }
        printf("\n");
    }
}

int FMat::GetRow(int Row, FVec& RowVec)
{
//
// Returns Matrix data at Row in RowVec
// All dimensions must be compatible
// returns -1 if matrix dimensions are exceeded
//
//
    if(Row<0 || Row>=rows()){
        return -1;
    }
    RowVec.ZeroInit(cols());
    for(int j=0;j<cols();j++){
        RowVec(j) = operator()(Row,j);
    }
    return 1;
}
//
//-----------------------------------------------------------------------------
// MATLAB Interfacing functions
//-----------------------------------------------------------------------------
//
int FMat::LoadMatlabFile(FString MatrixName, const char* pFileName)
{
//
// Loads data from matlab file, resizing matrix as necessary
// returns 1 if OK and -Error code if it failed.
//
	MatLabMatrix x;
	FILE *fp;
	char *pname;
//
// Open file for reading
//
	fp = fopen(pFileName, "rb");
	if (fp == NULL) {
		return -1;
	}
//
// Loop around until correct matrix found
//
	do {
//
// Get MatLabMatrix structure from file
//
		if (fread((char *)&x, sizeof(MatLabMatrix), 1, fp) != 1) {
			fclose(fp);
			return -2;
		}
//
// Create a character array for name
//
		pname = new char[x.namlen];
//
// Get matrix name from file
//
		if (fread(pname, sizeof(char), x.namlen, fp) != (unsigned)x.namlen) {
			fclose(fp);
			return -3;
		}
		if (strcmp(pname, MatrixName) != 0) {
//
// Jump over the incorrect variable
//
			fseek(fp, (sizeof(double)*x.mrows*x.ncols), SEEK_CUR);
			if (x.imagf) {
//
// Jump over imaginary part
//
				fseek(fp, (sizeof(double)*x.mrows*x.ncols), SEEK_CUR);
			}
			delete pname;
		}
	} while (strcmp(pname, MatrixName));
//
// Resize this matrix
//
	ReSize(x.mrows, x.ncols);
//
// Read data from file	
//
	if (fread(m_pData, sizeof(float), Size(), fp) != (size_t)Size()) {
		fclose(fp);
		return -4;
	}
//
// Success
//
	fclose(fp);
	return 1;
}
//
//-----------------------------------------------------------------------------
//
int FMat::SaveMatlabFile(const char* MatrixName, const char* pFileName)
{
//
// Saves a square matrix to a format my Matlab script can read
// This is my variant on an earler routine. Matlab is flexible
// enough that any format can be read provided it is known
// This function uses the MatlabMatrix structure to specify
// details and format of the matrix. It is written in column
// order as for FMat class
// returns 1 if OK and -Error code if it failed.
//
//
	FILE *fp;
	MatLabMatrix x;
	int count;
// 
	x.type = 2;
	x.mrows = rows();
	x.ncols = cols();
	x.imagf = 0;
	x.namlen = strlen(MatrixName) + 1;
//
// Open file for appending (writing)
//
	fp = fopen(pFileName, "ab");
	if (fp == NULL) {
		return -1;
	}
//
// Write the structure first as is contains dimensions of everything
// 
	count = fwrite(&x, sizeof(MatLabMatrix), 1, fp);    // write the structure
	if (count != 1) {
		fclose(fp);
		return -2;
	}
//
	count = fwrite( MatrixName, sizeof(char), (int)x.namlen, fp);   // write the name
	if (count != (int)x.namlen) {
		fclose(fp);
		return -3;
	}
//
	count = fwrite(m_pData, sizeof(float), Size(), fp);
	if (count != m_MaxI*m_MaxJ) {
		fclose(fp);
		return -4;
	}
//
// Success
//
	fclose(fp);
	return 1;
}
//
//-----------------------------------------------------------------------
// Following functions are not FMat class members but using FMat Objects
//
int inverse_fmat(FMat& Ain, FMat& B)
{
//
//  Inverts a general single precision matrix
//
//  USER NOTES:
//  
//  This function takes Ain as input and returns its inverse in B.
//  The method works best for a general square matrix of any order
//  Matrices of order less than 4 are solved explicitly, higher order
//  matrices are solved using Gaussian elimination with partial pivoting 
//  Ain is conserved in this version. The code is a direct conversion
//  from a FORTRAN source, hence the heavy duty SYNTAX
//
//  Returns 1 if all went well 
//          0 if it fails due to non-square or NULL 
//         -1 if its singular or ill-conditioned
//
// Locals
//
	int I, J, NM1, K, KP1, M, JJ, N;
	float T, SUM, Trc, nabla, det;
    int* pIPVT;
    FMat A;
//
// make a deep copy of the input matrix so we dont overwrite
// the input argument as we used to do in the bad old days of FORTRAN
//
    A.Copy(Ain);
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
        B.ReSize(1,1);
		B(0,0) = 1.0/A(0,0);
        break;
    case 2:
		det = A(0,0)*A(1,1)-A(1,0)*A(0,1);
		if(fabs(det) < nabla){
			return -1;
		}
        B.ReSize(2,2);
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
        B.ZeroInit(3,3);

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
// Initialise the target and pivot vector
    	pIPVT = new int[N];
//
// initialize the rhs
//
    	B.Identity(N,N);
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
// delete the pivot vector
    	delete pIPVT;
    	pIPVT=(int*)0;

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

int Jacobi_fmat(FMat& C, FMat& V, FMat& W, FMat& R)
{
// Jacobi decompostition of a symmetric matrix
//
// Determine Eigen-values and vectors for a positive-definite, symmetric matrix
// using Jacobi iteration, the matrices should already be allocated.
// Typical applications include [3x3] or higher covariance matrices
// however will also solve dynamic vibration probems if damping is neglected
//
// C           Input Matrix
// V           Eigen-vectors in asscending order of Eigen-values
// W           Work matrix
// R           Rotation Matrix
// Lambda      Eigen-values in no particular order 
//
// returns 1 if OK or -1 if it failed due to multiple identical values on the lead diagonal
//
    int i,j,k,m,n,p, iTest;
    int N;
    float sum, deno;

    N = C.rows();
    V.Identity(N,N);
    W.ZeroInit(N,N);

    const float Tr = fabs(C.Trace());
    const float eta = FLT_MIN*Tr;  // this only works for Gentle
    const float nabla = FLT_EPSILON;
//
// Jacobi iteration is usually pretty rapid
// for small matrices iteration limit is typically less than 2N
//
    for (k=0;k<N*2;k++){

        for(i=0;i<N-1;i++){
            for(j=i+1;j<N;j++){

                R.Identity(N,N);

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
// return number of iterates
//
            iTest = k+1;
            return iTest;    
        }

    }
    return -1; // iteration limit exhausted

}

int Cholesky_fmat(FMat& A, FMat& Decomp)
{
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
//
    Decomp.Copy(A);

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

int CholSolve_fmat(FMat& Decomp, FVec& X, FVec& B)
{
//
// uses the (Decomp)osed Lower diagonal Cholesky form to solve for X in A.X = B
// Decomp must already be decomposed using the above Cholesky_fmat() function
// B is a solution vector to the problem A.X = B and of compatible length
// for the given Decomp. Both this and the above were transcribed
// from Numerical Recipies 3rd Edition Press et al. 
//
    int N, i,k;
    N = Decomp.rows();
    float sum;
    X.ReSize(N);
//
    if(B.Size() !=N ){
        return -1;
    }

    for(i=0;i<N;i++){
        sum = B(i);
        for(k=i-1; k>=0; k--){
            sum -= Decomp(i,k)*X(k);      // tricksy code! it looks like X(i) it is being used before set
        }                                 // but it won't happen first time through because k=0>i-1
        X(i)=sum/Decomp(i,i);
    }

    for(i=N-1; i>=0; i--){
        sum=X(i);
        for(k=i+1; k<N; k++){
            sum -= Decomp(k,i)*X(k);
        }
        X(i) = sum/Decomp(i,i);
    }
    return 1;
}

void ShellSort(float* A2Sort, int* pIdx, int N)
{
//
// Sorts A2Sort into assending order and concurrently matches these swaps
// in pIdx Where pIdx starts as a zero-indexed assending integer array
// This enables sorting of eigen-vectors from the sorted values but 
// for this code version only real eigen-values allowed
// A2Sort and pIdx must be correctly allocated and at least equal to N
// elements before this call.
//
// The code was transcribed from Numerical Recipies Press et al.
// and extended to provide sorted pIdx array
//
    int i,j,ij,inc,isav;
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
}

//
//-----------------------------------------------------------------------------
//

int SaveDepthFile(const char* MatrixName, const char* pFileName, UINT16* pData, float Scale, int Rows, int Cols)
{
//
// Same as SaveMatlabFile but not using FMat class
// also matrix stored in row order for VisionOn depth data
//
	FILE *fp;
	MatLabMatrix x;
	int count, Size;

    Size = Rows*Cols;
// 
	x.type = 2;
	x.mrows = Rows;
	x.ncols = Cols;
	x.imagf = 0;
	x.namlen = strlen(MatrixName) + 1;
//    x.Scale = Scale;
//
// Open file for appending (writing)
//
	fp = fopen(pFileName, "w+b");
	if (fp == NULL) {
		return -1;
	}
//
// Write the structure first as is contains dimensions of everything
// 
	count = fwrite(&x, sizeof(MatLabMatrix), 1, fp);    // write the structure
	if (count != 1) {
		fclose(fp);
		return -2;
	}
//
	count = fwrite( MatrixName, sizeof(char), (int)x.namlen, fp);   // write the name
	if (count != (int)x.namlen) {
		fclose(fp);
		return -3;
	}
//
	count = fwrite(pData, sizeof(UINT16), Size, fp);
	if (count != Size) {
		fclose(fp);
		return -4;
	}
//
// Success
//
	fclose(fp);
	return 1;

}
//
//-----------------------------------------------------------------------------------------------
//
// Almost imposible to read/write text data to/from file without some 
// helper functions.
// So we have a small string class, probably my limitations showing here.
// It is always easier to work with familiar methods but at the same time 
// I dont want to bloat this code (intended for embedded platforms) 
// with unneeded features (otherwise I could just as well included MString)
//
FString::FString(const char* pStr)
{
    m_AllocLength=0;
    if(pStr!=NULL){
        SetString(pStr);
    }
}

FString::FString(const FString& Cpy)    
{
    SetString((const char*)Cpy);
}

FString::~FString()
{
    if(m_AllocLength>0){
        delete m_pStr;
    }
//
// this next bit is needed because memory cleanup is not universally performed
// many compiliers only deallocated so when you re-run the code
// it picks up the residual class state data and crashes
//
    m_pStr=NULL;
    m_AllocLength=0;
    m_Size=0;
}

int FString::ReSize(int Size)
{
    if(Size>0){
        if(m_AllocLength>0){        // if already allocated
            if(Size > (m_AllocLength-1)){  // if the new string is longer than allocated
// already allocated but less than needed so delete and re-allocate
                delete m_pStr;
          	   	m_AllocLength = Size+1; // = n + an additional char for '\0'
	            m_pStr = new char[m_AllocLength];     
            }
        } else {
// This is the first allocation
            m_AllocLength = Size+1;     // = n + an additional char for '\0'
            m_pStr = new char[m_AllocLength];     
        }
    
        m_Size = Size;
        m_pStr[m_Size] = '\0';
    }
    return m_Size;
}

int FString::SetString(const char* pStr, int Len)
{
// 
// Maintains the string memory allocation 
// to minimise memory usage on the Arduino
// any string <= m_AllocLength will
// copy onto the existing allocation
//
    int i, n;
    if(pStr!=NULL){                     // if this is a valid string
        if(Len>0){
            n = Len;
        }else{
            n = strlen(pStr);
        }
        ReSize(n);
// Finally copy the string
        for(i=0;i<n;i++){
            m_pStr[i]=(char)pStr[i];
        }
    }else{
        return -1;
    }
    return n;
}

int FString::SetString(const char* pStr, int start, int count)
{
// 
// Creates a new string from pStr from start for count characters
// pStr must be longer than start+count. This is
// equivalent to extracting count characters (i.e. a Word or number)
// from a longer string containing that Word
// 
    int i, m, n;
    if(pStr!=NULL){                     // if this is a valid string
        m = strlen(pStr);
        if(m>=(count+start)){
            n=count;
        } else {
            return -1;
        }
        ReSize(n);
// Finally copy the string
        for(i=0;i<n;i++){
            m_pStr[i]=(char)pStr[start+i];
        }
    }
    return n;
}

int FString::Add(const FString& s2)
{
//
// Creates a new string by concatenating string s2 to the end of the
// existing string, reallocating memory and copy in place if necessary
// otherwise the later part of the string is simply overwritten
// this is to enable multiple calls to the same string without
// undue memory re-allocation/fragmentation
//
    int i, m, n;
    char* pNew;
    const char* pCpy;
    m = s2.Size();
    n = m_Size + m;
    pCpy = s2.GetString();
    if(n>(m_AllocLength-1)){
        pNew = new char[n+1];
        for(i=0;i<m_Size;i++){  //copy the original string
            pNew[i]=m_pStr[i];
        }
        for(i=0;i<m;i++){   // add the new string
            pNew[i+m_Size]=pCpy[i];
        }
// free up the old string

        delete m_pStr;
        m_pStr = pNew;
        m_AllocLength = n+1;
    }
    else{
        for(i=0;i<m;i++){
            m_pStr[i+m_Size] = pCpy[i];
        }
    }
    m_Size = n;
    m_pStr[m_Size] = '\0';
    return m_Size;
}	

int FString::Add(const char* pCpy, int Len)
{
//
// Creates a new string by concatenating the first Len chars from pStr
// to the end of the existing string
// if Len==0 strlen() is used to determine pCpy string length
// reallocating memory and copy in place if necessary
// otherwise the later part of the output string is simply overwritten
// this is to enable multiple calls to the same string with different 
// additions without undue memory re-allocation/fragmentation
//
    int i, m, n;
    char* pNew;
    if(Len==0){
        m = strlen(pCpy);
    }else{
        m = Len;
    }
    n = m_Size + m;
    if(n>(m_AllocLength-1)){
        pNew = new char[n+1];
        for(i=0;i<m_Size;i++){
            pNew[i]=m_pStr[i];
        }
        for(i=0;i<m;i++){
            pNew[i+m_Size]=pCpy[i];
        }
// free up the old string and swap in pNew
        delete m_pStr;
        m_pStr = pNew;
        m_AllocLength = n+1;
    }
    else{
        for(i=0;i<m;i++){
            m_pStr[i+m_Size] = pCpy[i];
        }
    }
    m_Size = n;
    m_pStr[m_Size] = '\0';
    return m_Size;
}

int FString::Add(char ch)
{
//
// Creates a new string by concatenating the first Len chars from pStr
// to the end of the existing string
// if Len==0 strlen() is used to determine pCpy string length
// reallocating memory and copy in place if necessary
// otherwise the later part of the output string is simply overwritten
// this is to enable multiple calls to the same string with different 
// additions without undue memory re-allocation/fragmentation
//
    int i, n;
    char* pNew;
    n = m_Size + 1;
    if(n>(m_AllocLength-1)){
        pNew = new char[n+1];
        for(i=0;i<m_Size;i++){
            pNew[i]=m_pStr[i];
        }
        pNew[m_Size+1]=ch;
// free up the old string and swap in pNew
        delete m_pStr;
        m_pStr = pNew;
        m_AllocLength = n+1;
    }
    else{
        m_pStr[m_Size+1] = ch;
    }
    m_Size = n;
    m_pStr[m_Size] = '\0';
    return m_Size;
}

int FString::Floats2String(float* pFloats, int N)
{
//
// Converts vector of N pFloats into a string of numbers with
// precision of %#12.5E whether you need it or not!
// Ordinarily an internal write using sprintf would suffice
// whilst Arduino supports this for ints it doesnt support 
// floats so we do it here, this code is mainly for debug
//
// Each number will be followed by a trailing comma and space
// so that the output is well separated and CSV compatible.
//
    int i;
    int Len1 = (14);
    int Len2 = N*Len1;
    int count;
    ReSize(Len2);
    count=0;
    for(i=0;i<N;i++){
        count = FloatString(count,pFloats[i]);
    }
    return count;
}

int FString::FloatString(int count, const float F)
{
// This is an internal function called from Floats2String()
// Whatever the number it is always cast into this
// exact format: "-1.97465E-02, " 14 chars per number
// this is a bit rigid but it simplifies formatting
// and is flexible enough for most purposes
// the base FString array MUST already be
// correctly sized or a segmentation fault will occour
// no checks are made here 
// 
    static char nums[] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9'};
    char bits[] = {'-', '.', 'E', '+',' ', ',' };
    float F2, FA;
    char E[3];
    int i, index, test;

// check for sign
    if(F<0){
        m_pStr[count] = bits[0];
    }
    else{
        m_pStr[count]=bits[4];
    }
    count++;

// and then remove it
    FA = fabs(F);

// find the exponent index
    index = (int)(abs(log10(FA)));

// check for number less than unity
    test = int(F);
    if(test){
// +ve index
        E[0] = bits[3];
        test = index/10;
        E[1] = nums[test];
        test = index - 10*test;
        E[2] = nums[test];
        F2 = FA/pow(10,index);
    } else{

// -ve or zero index
        index+=1;
        if(index==1){
            E[0] = bits[3];
            E[1] = nums[0];
            E[2] = nums[0];
            F2 = FA;
        } else {
            E[0] = bits[0];
            test = index/10;
            E[1] = nums[test];
            test = index - 10*test;
            E[2] = nums[test];
            F2 = FA*pow(10,index);
        }
    }
// first digit
    test = int(F2);
    m_pStr[count]= nums[test];
    count++;
    m_pStr[count]= bits[1]; // decimal point
    count++;

// rest of digits
    F2 = 10.0*(F2 - float(test));
    for(i=0;i<5;i++){
        test = int(F2);
        m_pStr[count] = nums[test];
        F2 = 10*(F2-float(test));
        count++;
    }
    m_pStr[count] = bits[2]; // the E
    count++;
// followed by exponent
    for(i=0;i<3;i++){
        m_pStr[count]=E[i];
        count++;
    }
// finally the comma and a space
    m_pStr[count] = bits[5];
    count++;
    m_pStr[count] = bits[4];
    count++;
    return count;  
}

int FString::Ints2String(int* pInts, int N)
{
    int i,j;
    int Len1 = (10);
    int Len2 = N*Len1;
    int count;
    char buf[10];
    ReSize(Len2);
    count=0;
    for(i=0;i<N;i++){
        sprintf(buf,"%d, ",pInts[i]);
        Len1 = strlen(buf);
        for(j=0;j<Len1;j++){
            m_pStr[j+count] = buf[j];
        }
        count += Len1;
    }
    m_Size=count;
    m_pStr[m_Size]='\0';
    return count;
}

const FString& FString::operator=(const FString& Src)
{
	SetString((const char*)Src);
	return *this;
}

void FString::MakeUpper()
{
	int i,j;
	for (i = 0; i < Size(); i++) {
		j = m_pStr[i];
		if ( (j > 96) && (j < 123) ) {
			m_pStr[i] = char(j-32);
		}
	}
}

void FString::MakeLower()
{
	int i,j;
	for (i = 0; i < Size(); i++) {
		j = m_pStr[i];
		if ( (j > 64) && (j < 91) ) {
			m_pStr[i] = char(j+32);
		}
	}
}

int FString::Find(const char* pszSub) const
{
//
// find a sub-string within the target string
// Heavily modified from original 
// uses a numeric comparison, OK with 32bit ints 
// but we will declare long just in case
// returns the zero-based index of the first char if found
// returns -1 if not found and -2 if a dimension error occured
//
	int i,l,m,n;
	long j=(long)0, k=(long)0;

	m = strlen(pszSub);

	if(m<=0){ return -2;}
	if(m>Size()){ return -2;}
//
// compute a long integer for the substring 
// add 128*i to each character to ensure a unique number
// should be quicker than char by char comparison
	for (i = 0; i < m; i++) {
		j += pszSub[i]+128*i;
	}
// now seach the target for the same combination using the same trick
	n = Size() - m + 1;
	for (i = 0; i < n; i++) {
		k=0;
		for (l = i; l < m+i; l++) {
			k += m_pStr[l]+128*(l-i);
		}
// if found return the start index of the found string
		if (k == j) {
			return i;
		}
	}
	return -1;
}



int FindNextLine(FILE* pFile, FString& Line)
{
//
// Reads pFile for next none whitespace, none comment '#' line
// and returns by reference the valid line in Line also
// returns the count of characters in line if a
// line was read. Returns 0 for EOF
//
	int idx, len, k;
//
//
	idx=0;
	while(fgets(::ReadBuf, READ_BUF_SIZE-1, pFile) != NULL){
        len = strlen(::ReadBuf);
        while(idx<len){
            k = (int)::ReadBuf[idx];
            if(!white_space(k)){
                if(k==35){
                    goto Next;
                }
                else{
                    Line.SetString(::ReadBuf);
//
// fgets (rather unhelpfully) includes '\n' in the string
// to get round this we search backwards for either '\r' 
// or '\n' and downsize the string to eliminate them
//
					while( (::ReadBuf[len-1]=='\n' || ::ReadBuf[len-1]=='\r')  && (len>1) ){
						len--;
					}
					Line.ReSize(len);
                    return len;
                }
            }
            idx++;
        }
Next:	idx=0;
    }
    return 0;
}
//
//-----------------------------------------------------------------------------
//
int FindNextWord(const char* pLine, FString & Word, int& Index)
{
//
// This function gets the next Word on the pLine starting from Index
// A word being any contigious combination of characters in the range 
// 32 < k < 127, Commas (decimal 44) and pound '#' (decimal 35) serve
// as word delimitors.
//
// On return the Word found is returned by reference and Index is set to
// the location of the Last Character of the Word found + 1 and returns
// the count of characters in the word 
// If the String Length is exhausted in the process Index returns -1.
// If the string was exhausted on entry (and nothing found) returns -2
//
	int i, j, k, Len, num;
//
    Len = strlen(pLine);
    i = Index;
//
// for case where called with invalid index
//
	if(Index < 0){
		Index=-2;
		return Index;
	}
//
// find the start of a word signified by the first legit character
//
	while(i<Len){
        k=(int)(char)pLine[i];
        if(k == 35) break;  // end of line
		if(k > 32 && k < 127 && k != 44 ) goto next;
		i++;
	}
	Index = -2;
	return Index;
//
// find end of word (this being the first occurence
// of a none ligit character
//	
next:
	j = i;
	while(j<Len){
        k=(int)(char)pLine[j];
		if( k < 33 || k > 126 || k == 44 || k == 35 ) break;
		j++;
	}
//
    Index = j;
	if((Index>=(Len-1)) || k==35 ){ Index=-1;}
    num = j-i;
	Word.SetString(pLine,i,num);
	return num;
}	
//
//-----------------------------------------------------------------------------------------
// Find the next float on a line starting from Index
float FindNextFloat(const char* pBuf, int& Index)
{
    char Ch;
    int i;
    float Num;
    FString KeyWord;
    int N = strlen(pBuf);

// check for valid string and index in range
    if(N<=0 ||Index>=N){
        Index=-2;
        return 0.0;
    }
// loop past none numeric characters like ' ' && '=' 
    for( i=Index;i<N; i++ ){
        Ch = pBuf[i];
        if(IsNumeric(Ch)){
            Index = i;
                break;
        }
    }

    FindNextWord(pBuf,KeyWord,Index);
    if(Index<-1){
        return 0.0;
    }
    Num = STR_2_FLOAT(KeyWord);
    return Num;
}

int FindNextNumber(const char* pLine, FString & Number, int& Index)
{
// find the next number without convertion return it as a string
    char Ch;
    int i;
    FString KeyWord;
    int N = strlen(pLine);

// check for valid string and index in range
    if( N<=0 || Index>=(N-1)){
        Index=-2;
        return Index;
    }
// loop past none numeric characters like ' ' && '=' 
    for( i=Index;i<N; i++ ){
        Ch = pLine[i];
        if(IsNumeric(Ch)){
            Index = i;
            break;
        }
    }

    FindNextWord(pLine,KeyWord,Index);
    if(Index<-1){
        Number = "Not Found";
        return Index;
    }
//    Number.SetString(KeyWord,KeyWord.GetLength());
    Number = KeyWord;
    return Index;
}

int IndexFileName(const char* FileName, const char* Ext, FString& IndexedFileName, int IndexNo)
{
//
// function to produce IndexedFileName in the format FileName00X.Ext given pFileName and pExt
//
    static char nums[] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9'};
    char FIndex[3] = {0};
    char pad = '0';
    int len, n1, n2, n3;
//
    if (IndexNo < 0 || IndexNo > 999) {
        return -1;  // index not in range
    }

    if (IndexNo < 10) {
        FIndex[0] = pad;
        FIndex[1] = pad;
        FIndex[2] = nums[IndexNo];
    }
    else if ( IndexNo < 100 ) {
        n1 = IndexNo/10;
        n2 = IndexNo - 10*n1;
        FIndex[0] = pad;
        FIndex[1] = nums[n1];
        FIndex[2] = nums[n2];
    }
    else {
        n1 = IndexNo/100;
        n2 = (IndexNo - n1*100)/10;
        n3 = (IndexNo - n1*100 - n2*10);
        FIndex[0] = nums[n1];
        FIndex[1] = nums[n2];
        FIndex[2] = nums[n3];
    }

    n1 = strlen(FileName);
    n2 = strlen(FIndex);                 // 000
    n3 = strlen(Ext);
    len = n1 + n2 + n3;
    IndexedFileName.ReSize(len);
    IndexedFileName = FileName;
    IndexedFileName.Add(FIndex);
    IndexedFileName.Add(Ext);

    return IndexNo;
}

int ReadIntVectorFromFile(FILE* pFile, int* pVec, int Size)
{
// Reads an Integer Vector from file
// It is assumed that pVec points to a valid vector memory of at least Size*sizeof(int)
//
// The expectation is that the entire vector will be contained on 
// the next none-comment single line where the entries are 
// separated by spaces or commas
//
	FString Word, Line;
	int i, val, Count=0;
    int Index;
	i=FindNextLine(pFile, Line);
	if (i>0) {
        Index=0;
        Count=0;
    	for(i=0;i<Size;i++){
	    	FindNextWord(Line, Word, Index);
    	    Count++;
    		if(Index>-2){
	    		val = STR_2_I(Word);
		    	pVec[i] = val;
		    }
		    else{
			    Index=-1;	// format problem encountered
			    break;
        	    Count--;
		    }
	    }
        return Count;
	}
	return i;

}

WaveForm::WaveForm(int Type, float Amp, float freq, int NStep)
{
    SetParams(Type, Amp, freq, NStep);

    Reset();
}

void WaveForm::SetParams(int Type, float Amp, float freq, int NStep)
{
    m_Type = Type;
    m_Freq = freq;
    m_Amp  = Amp;
    m_Step = NStep;
	T1=1.0/(4.0*m_Freq);
	T2=2.0*T1;
    TStep=0.0;
    if(NStep){
        TStep = T1/float(NStep);
    }
	Slope=4.0*m_Freq;
    Omega = 2.0*MY_PI*m_Freq;
//
// Used by Trangular Waveform
//
}
void WaveForm::Reset()
{
	TMark=T1;
	TSub=T1;
    TStair = 0.0;
    out = 0.0;
}

float WaveForm::operator()(float Time)
{
// Produces a time-based waveform based on m_Type
    float tri;
    switch(m_Type)
    {
        case 1: // Sine 
            out = m_Amp*sin(Omega*Time);
        break;
        case 2: // square
            out = FSIGN(m_Amp,sin(Omega*Time));
        break;
        case 3: // triangle
            if(Time>=TMark){
                TMark+=T2;
                TSub -=T2;
            }
            S1=FSIGN(m_Amp,cos(Omega*Time));
            out = S1*(-1.0+Slope*(Time+TSub));
        break;
        case 4:     // staircase function
            if(Time>=TMark){
                TMark+=T2;
                TSub -=T2;
            }
            S1=FSIGN(m_Amp,cos(Omega*Time));
            tri = S1*(-1.0+Slope*(Time+TSub));
            if(Time>=TStair){
                out = tri;
                TStair+=TStep;
            }
        break;
        default:
            out = m_Amp*sin(Omega*Time);
        break;
    }
    return out;
}

