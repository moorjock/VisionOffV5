//
// <GMat.cuh>
//
// Small lightweight float matrix struct derived from FMat
// to support matrix operations in real-time on GPU.
// 
// As with FMat, GMat provides Fortran-based, column-major, matrix storage
// but uses zero-based C-indexing rules and float precision. 
// It is designed to work directly with FMat on the Host-side and GMat on the device
// with helper functions here and in FMat to facilitate this.
//
// All data storage memory must already be allocated and maintained by the caller
// this stuct does not maintain memory and cannot Allocate() or ReSize().
// With those exceptions most of the functionallity is the same as FMat
// There is no equivalent the GVec class, it is easier to use float* 
// vectors or GPU inbuilt types such as float3. 
//
// This code has been tested using: /Documents/Gen/CPP/GMatTest/GMatTest
// GMatTest should be reworked and repeated for any further extensions.
//-----------------------------------------------------------------------------------
//
//  Version:      	1.1
//  Changes:	  	Modifications to suit GPU better and some bug fixes
//  Date:	  		17 Sept 2021 
//  Programmer:   	A. Shepherd
//
//  Version:      	1.0
//  Changes:	  	First version derived from FMat.h/cpp for use in GPU
//  Date:	  		27 August 2021 
//  Programmer:   	A. Shepherd
//-----------------------------------------------------------------------------------
//
#ifndef ROBOT_GMAT
#define ROBOT_GMAT

#include <stdio.h>	// needed for FILE*
#include <math.h>	// needed for atan
#include <float.h>	// epslilon

#ifndef MY_MAT_CONSTANTS
	#define MY_MAT_CONSTANTS
	#define MY_PI 	3.1415926
	#define DPR   	57.295780
	#define RPD     (MY_PI/180.0)
	#define D_THETA (5.0*MY_PI/180.0)

	#define JACOBI_SIZE  10			// Jacobi Matrix Size Limit
	#define JACOBI_LIMIT 20			// Jacobi iteration limit

	#ifndef UINT16  
	    #define UINT16 unsigned short int
	#endif
#endif
//
// The functionallity of this struct is based on FMat but is lightweight 
// has only device-side (GPU) functionallity
//
// Objects of this type cannot be recycled or ReSized and they must be
// allocated as pointers into existing allocated GPU memory before any operations
// can be carried out. Maintenence of the donated memory is the responsibilty
// of the calling code and no checks on array bound violations are made
//
struct GMat
{
private:
    int     m_MaxI;         // number of rows or depth of matrix
    int     m_MaxJ;         // number of cols or width of matrix
//
	float*  m_dData;		// Device side data storage pointer

public:

// device side initialisation
	__device__ inline GMat(float* pDataStart, int Rows, int Cols)
	{
		SetMatrix(pDataStart, Rows, Cols);
	}

	__device__ inline void SetMatrix(float* pDataStart, int Rows, int Cols)
	{
		m_dData = pDataStart;
		m_MaxI  = Rows;
		m_MaxJ  = Cols; 
	}

// both sides can see these 
    __device__ inline int rows(){return m_MaxI;}
	__device__ inline int rows() const {return m_MaxI;}

    __device__ inline int cols(){return m_MaxJ;}
	__device__ inline int cols() const {return m_MaxJ;}

	__device__ inline int Size(){return m_MaxI*m_MaxJ;}
	__device__ inline int Size() const {return m_MaxI*m_MaxJ;}
	__device__ inline float* GetDataPointer(){return m_dData;};

	__device__ inline float& operator() (int i, int j)
	{
   		return m_dData[m_MaxI*j + i];
	} 

	__device__ inline float& operator() (int i, int j) const
	{
   		return m_dData[m_MaxI*j + i];
	}

	__device__ inline float& Trans(int i, int j)
	{
   		return m_dData[m_MaxI*i + j];
	} 

	__device__ inline float& Trans(int i, int j) const
	{
   		return m_dData[m_MaxI*i + j];
	}

	__device__ inline GMat& Zero()
	{
		for(int i=0;i<m_MaxI*m_MaxJ;i++){
			m_dData[i] = 0.0;
		}
		return *this;
	}

	__device__ inline GMat& Identity()
	{
		Zero();
		for(int i=0;i<m_MaxI;i++){
			operator()(i,i) = 1.0;
		}
		return *this;
	}

	__device__ inline float Trace()
	{
		float Tr=0.0;
		for(int i=0;i<m_MaxI;i++){
			Tr += fabsf(operator()(i,i));
		}
		return Tr;
	}

};


__device__ __host__ inline float* GetData(float* pBase, int& Index, int Size)
{
//
// small helper to do matrix mapping and update Index
// can be used on both input and output but care needed to
// specify Size needed for each allocation in the mapping
//
	float* pData = &pBase[Index];
	Index += Size;
	return pData;
}

__device__ inline void AXB(float* C, const GMat A, float* B)
{
//
// C = A*B 	Where A is GMat and B and C are c-vectors 
// C and B must be preallocated and the correct size
// based on the dimensions of A
	int i,j, M,N;
//	
	M = A.rows();
	N = A.cols();
//
	for(i=0; i<M; i++){
		C[i]=0.0;
		for(j=0; j<N; j++){
			C[i] += A(i,j)*B[j];
		}
	}
}


inline __host__ int GPUStep(int WorkStep)
{
//
// returns the nearest WorkStep rounded up to a whole
// multiple of 64 bytes (16 integers or floats)
//
	int N = WorkStep/16;
	int rem = WorkStep%16;
	WorkStep = 16*N;
	if(rem>0){
		WorkStep+=16;
	}
	return WorkStep;
}
//
// functions using GMat objects have similar but not identical 
// functionallity with similar FMat functions but for GPU __device__ side only
//
__device__ int inverse_GMat(GMat& A, GMat& Ainv);	// my general inverse

__device__ int Jacobi_GMat(GMat& C, GMat& V, GMat& W, GMat& R);	// Jacobi decompostition of a symmetric matrix

__device__ int Cholesky_GMat(GMat& A, GMat& Decomp);	// Decompose a symmetric matrix to Lower Cholesky form

__device__ int CholSolve_GMat(GMat& Decomp, float* pX, float* pB);	// use the Decomposed form to solve for X in A.X = B

__device__ int ShellSort_GPU(float* A2Sort, int* pIdx, int N);	// Sort N elements in A2Sort and save swaps in pIdx


#endif

//
