#ifndef DEVICEFUNCS_CUH
#define DEVICEFUNCS_CUH

#include "FMat.h"

#include "constants.h"

typedef struct {
float3 o, d; // origin and direction
} Ray;

//
//---------------------------------------------------------------------------------------------------------------
// This struct provides a 1-D lookup Table
// it must be a constant m_Delta mesh size monotomically increasing
// in X from m_MinX, this speeds up processing by allowing direct
// index access i = int((X-m_XMin)/m_Delta) thus 
// only the Y axis m_hTable/m_dTable data, m_MinX and m_Delta are needed
// See SetTable for further info.
//
typedef struct 
{
	float*	m_dTable;
    float*  m_hTable;
	int		m_Size;
	float 	m_Delta;
	float 	m_MinX;

	int SetTable(float* pTab, int Sz, float del, float MinX);

	inline __device__ float dLookup(float x)
	{
		// __device__ table lookup function
		float dx = x - m_MinX;
		if(dx<0.0) return m_dTable[0];
		int i =  int(dx/m_Delta);
		if(i>=(m_Size-1)) return m_dTable[m_Size-1];
		dx -= float(i)*m_Delta;
		return (m_dTable[i]+(m_dTable[i+1]-m_dTable[i])*dx/m_Delta);
	}

	inline __host__ float hLookup(float x)
	{
		// __host__ table lookup function
		float dx = x - m_MinX;
		if(dx<0.0) return m_hTable[0];
		int i =  int(dx/m_Delta);
		if(i>=(m_Size-1)) return m_hTable[m_Size-1];
		dx -= float(i)*m_Delta;
		return (m_hTable[i]+(m_hTable[i+1]-m_hTable[i])*dx/m_Delta);
	}

    void FreeTable()
    {
        if(m_dTable){
            cudaFree(m_dTable);
            m_dTable=NULL;
        }
    }

} Look1Dc; 
//
// This GBHMat matrix struct is intended for use with 4x4 homogenous
// transformation matrices and many operations are at least partially unrolled.
// However it has since been extended to include some general matrix operations
//
// The starting assumption is that all operations that affect the transformation 
// will be carried out on the host CPU and updated to the GPU matrix, thus the 
// only operations needed on the GPU side are applying the transformation
// to lots of vertex data in parallel
//
// Still not happy about GPU and classes otherwise I would have derived this
// from FMat. The sticking point is that GPU kernels cannot be class members
// Anyway this struct is more complex than a simple class since it has a duality of
// members with both CPU and GPU memory allocation and transfers
//
struct GBHMat
{
    float*  m_hData;	// host side matrix storage
    float*  m_dData;	// GPU device side

	GBHMat(){
		m_dData = NULL;
		m_hData = NULL;
	}
//
// host matrix operators
//
	__host__ float& operator()(int i, int j)
	{
   		return m_hData[4*j + i];
	} 

	__host__ float& operator()(int i, int j) const
	{
   		return m_hData[4*j + i];
	}

	__host__ float& TransH(int i, int j)
	{
   		return m_hData[4*i + j];
	} 

	__host__ float& TransH(int i, int j) const
	{
   		return m_hData[4*i + j];
	}
//
// device equivalents, note they access a different matrix
//
	__device__ float& Get(int i, int j)
	{
   		return m_dData[4*j + i];
	} 

	__device__ float& Get(int i, int j) const
	{
   		return m_dData[4*j + i];
	}
	__device__ void Set(int i, int j, float val)
	{
   		m_dData[4*j + i] = val;
	} 

	__device__ float& TransD(int i, int j)
	{
   		return m_dData[4*i + j];
	} 

	__device__ float& TransD(int i, int j) const
	{
   		return m_dData[4*i + j];
	}
//
// Initial setup and maintenance is done on the host
//

	__host__ int InitMatrix(const float3 & Angles,const float3 & Offsets);	// initialise memory

	__host__ int InitMatrix(const FVec& Angles,const FVec& Offsets);	// initialise 

	__host__ int InitMatrix(const FVec& Pose);

	__host__ int InitMatrix();

    __host__ int ReMapToOldPose(const FVec& Pose);  // For volume

	__host__ void TransposeCopy(float* pCMat, int RowMax, int ColMax);

	__host__ int SetMatrix(const float3 & Angles, const float3 & Offsets);	// set values

	__host__ int SetMatrix(const FVec& Angles, const FVec& Offsets);

	__host__ int SetMatrix(const FVec& AnglesAndDisp);

	__host__ float determinant();	// determinant of the [3x3] rotation matrix

	__host__ float normf();			// Frobenius norm of the [4x4] sqrt( sigma( A(i,j)*A(i,j) ) )

	__host__ int SetDisplacement(const float3 & disp);

	__host__ int SetDisplacement(const FVec& disp);

	__host__ float3 GetDisplacement();

	__host__ int  UpdateGPU();

	__host__ int UpdateHost();

	__host__ GBHMat& Transpose(GBHMat& Trans);

	__host__ float3 RotateH(const float3& xyzg);

	__host__ float3 InvRotateH(const float3& xyzb);

	__host__ FVec InvRotateH(const FVec& xyzb);

    __host__ GBHMat& operator=(GBHMat& Tran);

	__host__ GBHMat& operator*=(GBHMat& Tran);				// compound in place

	__host__ GBHMat& operator^=(GBHMat& Tran);				// equivalent to A*(Tran^T)

	inline __host__ void Free(){
		if(m_hData!=NULL){
			cudaFree(m_dData);
			m_dData = NULL;
			free(m_hData);
		}
	}
	__host__ void printPose(const char* pText=NULL, int EndLn=0, FILE* pFile=NULL);

	__host__ void printMatrix(const char* pName=NULL, FILE* pFile=NULL);	// only works with the host matrix

	__host__ 	FVec& GetPose();							// returns a 6-vector {phi,theta,psi,tx,ty,tz}

	__device__ float3 operator*(const float3& xyzg);		// transform from global axes to body xyzg -> xyzb Xb = Tbg*(xg-xg0)

	__device__ float3 Rotate(const float3& xyzg);			// rotate from global to body axes but not translate xyzg ->xyzb

	__device__ float3 InvR(const float3& xyzb);				// transform body axis to global xyzb -> xyzg Xg = (Tbg^-1)*Xb+Xg0

	__device__ float3 InvRotate(const float3& xyzb);		// inverse rotate from body axis to global without translation


};


inline __host__ __device__ int _NINT(float a)
{
	return ( (a >= 0) ? (int)(a+0.5f) : (int)(a-0.5f));
}

inline __host__ __device__ int divUp(int a, int b) 
{
  return (a + b - 1)/b; 
}

inline __device__ unsigned char clip(int n) 
{
  return n > 255 ? 255 : (n < 0 ? 0 : n); 
}

inline __device__ int clipWithBounds(int n, int n_min, int n_max)
{
  return n > n_max ? n_max : (n < n_min ? n_min : n);
}

inline __device__ float3 yRotate(float3 pos, float thetaY)
{
  const float cs = cosf(thetaY), sn = sinf(thetaY);

  return make_float3(cs*pos.x + sn*pos.z, pos.y, -sn*pos.x + cs*pos.z);
}

inline __device__ float3 xRotate(float3 pos, float thetaX)
{
  const float cs = cosf(thetaX), sn = sinf(thetaX);
  return make_float3(pos.x, cs*pos.y -sn*pos.z,  sn*pos.y + cs*pos.z);
}

inline __device__ int3 posToVolIndex(float3 pos, int3 volSize) 
{
  return make_int3(pos.x + volSize.x/2, pos.y + volSize.y/2, pos.z + volSize.z/2);
}

inline __device__ int flatten(int3 index, int3 volSize) 
{
  return index.x + index.y*volSize.x + index.z*volSize.x*volSize.y;
}

inline __device__ int flatten(int x, int y, int z, int3 volSize) 
{
  return (x + y*volSize.x + z*volSize.x*volSize.y);
}


inline __device__ float3 scrIdxToPos(int c, int r, int w, int h, float zs)
{
  return make_float3(c - w / 2, r - h / 2, zs);
}

__device__ float planeSDF(float3 pos, float3 norm, float d); 

__device__ float3 Vertex2Image(const float3& p);

__device__ float3 Image2Vertex(int u, int v, float d);

__device__ float3 paramRay(Ray r, float t);

__device__ float func(int c, int r, int s, int Shape, int3 volSize,float4 params);

__device__ float face(int s, float val);

__device__ float3 scrIdxToPos(int c, int r, int w, int h, float zs);

__device__ bool intersectBox(Ray r, float3 boxmin, float3 boxmax, float *tnear, float *tfar);

__device__ uchar4 MySliceShader(float *d_vol, int3 volSize, Ray boxRay, float threshold, float dist, float3 norm, unsigned int *pPallet, int colours);

__device__ uchar4 volumeRenderShader(float *d_vol, int3 volSize, Ray boxRay, float dist, int numSteps);

__device__ uchar4 rayCastShader(float *d_vol, int3 volSize, Ray boxRay, float threshold);

__device__ uchar4 MyRayShader(float *d_vol, int3 volSize, Ray boxRay, float threshold, unsigned int *pPallet, int colours);

__device__ uchar4 MySimpleShader(float *d_vol, int3 volSize, Ray boxRay, float threshold, unsigned int *pPallet, int colours);

__device__ float density(float *d_vol, int3 volSize, float3 pos);

__device__ float MyDensity(float *d_vol, int3 volSize, float3 pos);

__device__ uchar4 ChooseColour(unsigned int* pPallet, int idx );

__device__ bool rayPlaneIntersect(Ray myRay, float3 n, float dist, float *t);

__forceinline__ __device__ __host__ int Get2DIndex(int x, int y, int xlim){ return x + xlim*y;}


#define EPS 0.01f

#define THRESH  1.0f

#endif
