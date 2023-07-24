//
// <FMat.h>
//
// Two small and lightweight float vector and matrix classes to support basic
// matrix operations in a real-time, resource-constrained embedded environment.
// FString and useful string manipulations mainly for reading also added.
// 
// FMat provides Fortran-based, column-major storage but zero-based C-indexing
// using float-precision it can only be used on conventional CPU's using gcc.
// A companion struct GMat has been developed for GPU using cuda __device__ codes.
// FMat and GMat are designed to work in step with FMat used on the host.
//
// FMat and FVec can maintain their own memory with Allocate()) and ReSize() but
// copy-by-reference and reference-counting not implemented, all copies are deep 
// (you can pass by reference or a pointer). 
//
// FMat and FVec objects can be allocated with pointers to existing data
// in which case they cannot ReSize() and allocation and cleanup are the
// responsibilty of the caller.
//
//-----------------------------------------------------------------------------------
//  Version:      	2.5
//  Changes:	  	Further Modifications for compatibility with GMat
//  Date:	  		17 September 2021 
//  Programmer:   	A. Shepherd
//
//  Version:      	2.4
//  Changes:	  	Modifications for compatibility with GMat
//  Date:	  		10 September 2021 
//  Programmer:   	A. Shepherd
//
//  Version:      	2.3
//  Changes:	  	Added Matrix Inversion, Jacobi Iteration and Cholesky decomposition
//  Date:	  		23 August 2021 
//  Programmer:   	A. Shepherd
//
//  Version:      	2.2
//  Changes:	  	Matlab save function added
//  Date:	  	9 June 2021 
//  Programmer:   	A. Shepherd
//
//  Version:      	2.1
//  Changes:	  	Added WaveForm class
//  Date:	  	20 May 2020 
//  Programmer:   	A. Shepherd
//
//  Version:      	2.0
//  Changes:	  	Added DHFmat class and added operator overloads to FMat, FVec
//  Date:	  	20 March 2020 
//  Programmer:   	A. Shepherd
//
//  Version:      	1.0
//  Changes:	  	First version
//  Date:	  	01 October 2019 
//  Programmer:   	A. Shepherd
//-----------------------------------------------------------------------------------
//
#ifndef ROBOT_FMAT
#define ROBOT_FMAT

#include <stdio.h>	// needed for FILE*
#include <math.h>	// needed for atan
#include <float.h>	// epslilon

#define MY_PI 	3.1415926
#define DPR   	57.295780
#define RPD     (MY_PI/180.0)
#define D_THETA (5.0*MY_PI/180.0)

#ifndef UINT16  
    #define UINT16 unsigned short int
#endif

inline int my_min(int a, int b)
		{
			return ( (a<=b) ? a : b);
		}
inline int my_max(int a, int b)
		{
			return ( (a>=b) ? a : b);
		}

inline float my_fmin(float a, float b)
		{
			return ( (a<=b) ? a : b);
		}
inline float my_fmax(float a, float b)
		{
			return ( (a>=b) ? a : b);
		}
//
// Reduced complexity float vector class
// Cautionary Note! These vectors cannot be passed
// by value in calls to functions using them
// use either pass by reference or pointers
//

class FVec
{
    int     m_Len;          // current working vector length
    float*  m_pData;        // the data vector
    int     m_Allocated;    // the Allocated data vector length

public:

    FVec(int Size = 0);

	FVec( const FVec& Cpy);

	FVec(float* pData, int Size);

    ~FVec();

	void SetData(float* pDat, int Size);
	
    int Size() const {return m_Len;}                  // returns the current working length

	int Allocated()const {return m_Allocated;}

	void ZeroInit(int i=0);		                // sets all elements to 0 resizing if needed

    int ReSize(int Size);                          // Caution this works differently to the full SubUtils/Vector class

	FVec& Copy(const FVec& Src);

	FVec& Copy( const float* pVec, int Sz);			// create a copy from an original c-vector

    inline float &operator()(int i); 			// Indexing the primary reason for this class

    inline float &operator()(int i) const; 			// Indexing the primary reason for this class

    int ReadVectorFromFile(FILE* pFile, int Size);

	int VectorFromString(const char* pString, int Num);

	void printVector(const char* pString=NULL, FILE* pFile=NULL);

	inline void CopyToTarget(float* pTarget, int& Index)
	{	// pTarget must be allocated and big enough
		for(int i=0;i<Size();i++){
			pTarget[Index] = m_pData[i];
			Index++;
		}
	}

	float* GetDataPointer()
		{ return m_pData;}

	FVec& operator=(const FVec& Src);

};

inline float& FVec::operator()(int i)
{
    return m_pData[i];      // there are no checks
} 

inline float& FVec::operator()(int i) const
{
    return m_pData[i];      // there are no checks
} 
//
//----------------------------------------------------------------------------------------------------------------
//
class FString;

class FMat
{
    int     m_MaxI;         // number of rows or depth of matrix
    int     m_MaxJ;         // number of cols or width of matrix
    int     m_Allocated;    // length of allocated storage memory
    float*  m_pData;        // the data storage vector

public:
    FMat(int i=0, int j=0);
    ~FMat();

	FMat(float* pDat, int i, int j);		// Copies data from elsewhere (so no ownership)

	FMat(const FMat& Cpy);

    int rows(){return m_MaxI;}
	int rows() const {return m_MaxI;}

    int cols(){return m_MaxJ;}
	int cols() const {return m_MaxJ;}

	int Size(){return m_MaxI*m_MaxJ;}
	int Size() const {return m_MaxI*m_MaxJ;}

	void	ZeroInit(int i, int j);				// sets all elements to 0 resizing if needed
	void	Identity(int i, int j);				// sets an Identity Matrix

	void SetData(float* pDat, int i, int j);	// Copies data from elsewhere (so no ownership)
	
    int ReSize(int i, int j);               	// Caution this works differently to the full SubUtils/Matrix class

	FMat& Copy(const FMat& Src);				// deep copy

    FMat& Multiply(FMat* pMat, float* pVec);    // Post Multiply a Matrix by a vector they must be compatible

    inline FMat& operator*=(const FMat& A);

	FMat& operator=(const FMat& A);

	inline float& operator() (int i, int j)
	{
   		return m_pData[m_MaxI*j + i];
	} 

	inline float& operator() (int i, int j) const
	{
   		return m_pData[m_MaxI*j + i];
	}

	inline float* GetPointerAt(int i, int j)
	{
   		return &m_pData[m_MaxI*j + i];
	}

	inline float& Trans(int i, int j)	// transpose access to matrix elements
	{
   		return m_pData[m_MaxI*i + j];
	} 

	inline float& Trans(int i, int j) const
	{
   		return m_pData[m_MaxI*i + j];
	}
	
	inline float Trace()
	{
		float Tr=0.0;
		for(int i=0;i<m_MaxI;i++){
			Tr += fabs(operator()(i,i));
		}
		return Tr;
	}

	inline void CopyToTarget(float* pTarget, int& Index)
	{	// pTarget must be allocated and big enough
		for(int i=0;i<Size();i++){
			pTarget[Index] = m_pData[i];
			Index++;
		}
	}

	int	ReadFromTextFile(FILE* pFile);

	void printMatrix(const char* pName=NULL, FILE* pFile=NULL);

	int LoadMatlabFile(FString MatrixName, const char* pFileName);

	int SaveMatlabFile(const char* MatrixName, const char* pFileName);

    int GetRow(int Row, FVec& RowVec);
};

// functions using FMat objects
int inverse_fmat(FMat& A, FMat& Ainv);	// my general inverse

int Jacobi_fmat(FMat& C, FMat& V, FMat& W, FMat& R);	// Jacobi decompostition of a symmetric matrix

int Cholesky_fmat(FMat& A, FMat& Decomp);	// Decompose a symmetric matrix to Lower Cholesky form

int CholSolve_fmat(FMat& Decomp, FVec& X, FVec& B);	// use the Decomposed form to solve for X in A.X = B

void ShellSort(float* A2Sort, int* pIdx, int N);	// Sort N elements in A2Sort and save swaps in pIdx

inline FMat& FMat::operator*=(const FMat& A)
{
//
// The target must be pre-dimensioned and conformal with A
// it is also assumed to be correctly initialised
//
	int i,j,k,maxi,maxj,maxk; 
	maxi = rows();
	maxj = A.cols();
	maxk = cols();
	FMat C;
	C.ZeroInit(maxi,maxj);
	for(i=0;i<maxi;i++){
		for(j=0;j<maxj;j++){
			for(k=0;k<maxk;k++){
				C(i,j) += operator()(i,k)*A(k,j);
			}
		}
	}

	for(i=0;i<maxi;i++){
		for(j=0;j<maxj;j++){
			operator()(i,j) = C(i,j);
		}
	}
	return *this;
}

inline FMat operator*(const FMat& A, const FMat& B)
{
//
// A and B should be conformable under Cayley multiplcation
// i.e A.cols() = B.rows() however this function will perform
// Cayley multiplcation on the common subset
//
	int i, j, k;
	int maxi, maxj, maxk;
//
	static FMat C;
    maxi = A.rows();
	maxj = B.cols();
	maxk = my_min(A.cols(),B.rows());

	C.ReSize(maxi,maxj);

	for(i=0; i<maxi; i++) {
		for(j=0; j<maxj; j++) {
			C(i,j) = 0.0;
			for (k=0; k<maxk; k++) {
				C(i,j) += A(i,k) * B(k,j);
			}
		}
	}
//
	return C;
}
inline int Amat_X_Bmat(FMat& C, const FMat& A, const FMat& B)
{
//
// A and B must be comute under Cayley multiplication
// i.e A.cols() = B.rows() 
//
	int i, j, k;
	int maxi, maxj, maxk;
//
    maxi = A.rows();
	maxj = B.cols();
	maxk = A.cols();

	C.ZeroInit(maxi,maxj);

	if( maxk != B.rows() ){
		return -1;
	}
	for(i=0; i<maxi; i++) {
		for(j=0; j<maxj; j++) {
			for (k=0; k<maxk; k++) {
				C(i,j) += A(i,k) * B(k,j);
			}
		}
	}
//
	return 0;
}

inline FVec operator*(const FMat& A,const FVec& B)
{
//
//
// returns C = [A]*D 
// It uses the rows of A as the lead dimension of C
// and the min of A.cols() and D.size() 
//
	int i,j, M,N;
//	
	M = A.rows();
	N = my_min(A.cols(),B.Size());
//
	static FVec C;
	C.ReSize(M);
//
	for(i=0; i<M; i++){
		C(i) = 0.0;
		for(j=0; j<N; j++){
			C(i) += A(i,j)*B(j);;
		}
	}
	return C;
}
inline int Amat_X_Bvec(FVec& C, const FMat& A, const FVec& B)
{
//
// C = A*B 	Where A is FMat and B and C are FVec 
// Size of B must match Cols of A
//
	int i,j, M,N;
//	
	M = A.rows();
	N = A.cols();
//
	C.ZeroInit(M);
	if(N!=B.Size()){
		return -1;
	}
//
	for(i=0; i<M; i++){
		for(j=0; j<N; j++){
			C(i) += A(i,j)*B(j);;
		}
	}
	return 0;
}
//
//------------------------------------------------------------------------------
//
class FString
{
    char* m_pStr;
    int m_AllocLength;	// allocated string length 
	int m_Size;			// current length can be < m_AllocLength
public:

    FString(const char* pStr=NULL);

	FString(const FString& Cpy);	// this is a deep copy

    ~FString();

	int Size(){return m_Size;}

	int Size() const {return m_Size;}

	operator const char*() const		// one of the most useful overloads of all
		{ return (const char*)m_pStr; }

	char GetChar(int i){return m_pStr[i];} // use with caution (no brakes)

	void SetChar(int i, char c){m_pStr[i]=c;}

	int Add(const FString& s2);	

	int Add(const char*pStr, int Len=0);

	int Add(char ch);
	
	const FString& operator=(const FString& Src);

    int SetString(const char* pStr, int Len=0);

	int SetString(const char* pStr, int start, int count);

    char* GetString(){return m_pStr;}

    const char* GetString() const
			{return m_pStr;}

    int GetLength(){return m_Size;}

	int GetAllocated(){return m_AllocLength;}

    int ReSize(int Size);

	int Find(const char* pszSub) const;

	int Floats2String(float* pFloats, int N);

	int Ints2String(int* pInts, int N);

	void MakeUpper();

	void MakeLower();

protected:
// meant to be called internally from Floats2String
	int FloatString(int StartPos, float F);


};

//
// Class to generate various different time-based input waveforms
// Sine, Square, Triangular and Staircase
//
class WaveForm
{
	float T1;
	float T2;
	float Slope;
	float TMark;
	float TSub;
	float TStep;
	float TStair;
	float S1;
	float Omega;
	float out;
protected:

	int	  m_Type;		// Type of waveform 1=sin, 2=square, 3=triangle, 4 = staircase
	float m_Amp;		// amplitude
	float m_Freq;		// frequency
	int   m_Step;		// number of steps (used only for staircase)

public:

	WaveForm(int Type, float Amp, float freq, int NStep=0);

	void SetParams(int Type, float Amp, float freq, int NStep=0);
	
	float operator()(float Time);		// note time must be monotonic increasing, Time==0 will cause a reset

	void Reset();		// reset internal states for a new run i.e. Time=0.0
};
// some string helper functions, note that these are identical to those in MYString.h/cpp
// they will conflict if mixed

inline int operator==(const FString& s1, const char* s2)
						{ return s1.Find(s2) == 0; }

inline int white_space(char c)
            {
                int k = (int)c;
                return (k < 33 || k > 126 || k == 44 );
            }
// looks for chars in normal ascii printing range, counts comma as white-space

inline int IsComma(char c)
			{return ((int)c == 44);}

inline int IsNumeric(char c)
			{
				int i = (int)c;
				if(i==45){
					return 1;
				}
				if( ((i > 47) && (i < 58)) ){
					return 1;
				}
				return 0;
			}

inline int IsLower(char c)
			{
				int i = (int)c;
				if( (i > 96) && (i < 123) ){
					return 1;
				}
				return 0;
			}

inline int IsUPPER(char c)
			{
				int i = (int)c;
				if( (i > 64) && (i < 91 ) ){
					return 1;
				}
				return 0;
			}

// 
//-----------------------------------------------------------------------------
// Conversions
//
inline double STR_2_DOUB(const char* word)
{
	return atof(word);// String to double conversion no error checking
}

inline float STR_2_FLOAT(const char* word)
{
	return (float)atof(word);// String to float conversion no error checking
}

inline int STR_2_I(const char* word)
{
		return atoi(word);// String to integer conversion no error checking returns
}
//
// fortran look-alikes to ease migration
//
inline int NINT(float a)
{
// FORTRAN look-alike to ease migration (and because its useful)
// small function to find nearest integer rather than truncation
// obtained by direct cast to (int) there are equivalent functions in cuda
//
	if(a >= 0){
		return (int)(a+0.5f);
	}
	else{
		return (int)(a-0.5f);
	}
}

inline float FSIGN(float Amp, float Test)
{
// another fortran look-alike to replicate 
// the intrinsic function of the same name
	if(Test>=0.0f) {
		return Amp;
	} else {
		return -Amp;
	}
}

inline float my_atan2(float x, float y)
{
//
// x and y have their usual graphical meaning
//
    float at = atan(y/x);
    if( x > 0.0 ){     
        return at;
	}
    else if( x < 0.0 ){
        if( at < 0.0 ) {
			return (at+MY_PI);
		}
        else{
			return (at-MY_PI);
		}
	}
    else{
		if(y>0) {
			return MY_PI/2.0;
		} else if(y<0){
			return -MY_PI/2.0;
		} else{
			return 0.0f;
		}
	} 
}

inline float my_atan360(float x, float y)
{
//
// x and y have their usual graphical meaning
//
    float EPS = 1.0E-7;
// This needs tidying up
    if(fabs(x) < EPS and fabs(y) > EPS){
        return (y*MY_PI/(fabs(y)*2.0));
    } else {

        float at = atan(y/x);

        if( x > 0.0 && y>0.0 ){
            return at;
        } else if( x < 0.0) {
            return (MY_PI + at);
        } else if( x>0.0 && y< 0.0){
            return (2.0*MY_PI + at);
	    } else{
            return 0.0;
        }
    }
}

//
// Helper functions for text file reading
// some coupling with compiler inevitable for i/o
//
int FindNextLine(FILE* pFile, FString& Line);
// get the next none-comment, none-whitespace line from pFile

int FindNextWord(const char* pLine, FString & Word, int& Index);
// find next Word in pLine starting at Index

int FindNextNumber(const char* pLine, FString & Number, int& Index);
// find the next number on a line and return as a string in Number

float FindNextFloat(const char* pBuf, int& Index);
// find the next float on a line and return it as such

int IndexFileName(const char* FileName, const char* Ext, FString& IndexedFileName, int IndexNo);

int SaveDepthFile(const char* MatrixName, const char* pFileName, UINT16* pData, float Scale, int Rows, int Cols);

int ReadIntVectorFromFile(FILE* pFile, int* pVec, int Size);

#define READ_BUF_SIZE 256
extern char ReadBuf[READ_BUF_SIZE];

//
// Define data structure for MATLAB matrices
//
typedef struct {
	 long type;     /* type 0=ushort, 1=int2, 2=int4, 3=float4, 4=double8 */
	 long mrows;    /* row dimension */
	 long ncols;    /* column dimension */
	 long imagf;    /* flag indicating data contains an imaginary part */
	 long namlen;   /* name length (including NULL) */
//	 float Scale;	/* Scale of data i.e.CAM_D2M */
} MatLabMatrix;

#endif

//
