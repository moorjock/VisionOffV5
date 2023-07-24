
#include <helper_math.h>
#include <cuda_runtime.h>
#include <stdio.h>
#include "device_funcs.cuh"

#include "FMat.h"

__device__ float3 Vertex2Image(const float3& p)
{
// this function will ONLY work in device code due to
// the presence of cuda __fmam_rz() and __fdividef()
// Projects a physical vertex p{x,y,z} into an image plane {u,v|d}
// CAM_K is the projection constant and
// CAM_CX = rows/2, CAM_CY = cols/2 for the camera image centre
// thus:
// 		Img.x = CAM_K*px/pz + cols/2
// 		Img.y = CAM_K*py/pz + rows/2
//
    float3 Img;
    Img.x = __fmaf_rz(CAM_K, __fdividef(p.x,p.z), CAM_CX); 
    Img.y = __fmaf_rz(CAM_K, __fdividef(p.y,p.z), CAM_CY);
	  Img.z = p.z;
    return Img;
}

__device__ float3 Image2Vertex(int u, int v, float d)
{
// note d must be in m units
// takes a camera image pixel {u,v,z} and computes the corresponding vertex {x,y,z}
// CAM_Kinv = 1/CAM_K 
// CAM_CX = rows/2, CAM_CY = cols/2 for the camera image centre (zero point)
//
    float x = d * (u - CAM_CX) * CAM_Kinv;
    float y = d * (v - CAM_CY) * CAM_Kinv;
    return make_float3(x, y, d);
}

//
//-------------------------------------------------------------------------------------------------------------
//

int Look1Dc::SetTable(float* pTab, int Sz, float delta, float MinX)
{
//
// This look-up table is set from a c-vector pTab which must be 
// correctly allocated, sized and initialised with data by the host
// delta must also be set to a +ve non-zero value
//
    if(delta<=0 || Sz<=0) return -1;
    m_Size = Sz;
    m_Delta = delta;
    m_hTable = pTab;  // host can access the supplied table no need to copy
    m_MinX = MinX;
    int error = cudaMalloc(&m_dTable, m_Size*sizeof(float));
    error += cudaMemcpy(m_dTable,m_hTable,m_Size*sizeof(float),cudaMemcpyHostToDevice);  
    return error;
}
//
//-------------------------------------------------------------------------------------------------------------
//
__host__ int GBHMat::InitMatrix(const float3 & ptp, const float3 & disp)
{ 	
//
// initialise the memory and set up the transformation matrix
// Both GPU and Host memory are allocated for debugging
//
//  ptp     Transformation angles Tx (Pitch), Ty(Yaw), Tz(Roll) (in rads)
//  disp    displacement vector x,y,z in metres
//
    int error=0;
    if(m_dData==NULL){
        m_hData = (float*) malloc(16*sizeof(float));
        error = cudaMalloc(&m_dData, 16*sizeof(float));
    }
    if(error) return error;
    return SetMatrix(ptp,disp);
}

__host__ int GBHMat::InitMatrix(const FVec& ptp, const FVec& disp)
{ 	
// 
// Initialise the memory and set up the transformation matrix
// Both GPU and Host memory are allocated for debugging 
// This overload is for FVec data
//
//  ptp     Transformation angles Tx (Pitch), Ty(Yaw), Tz(Roll) (in rads)
//  disp    displacement vector x,y,z in metres
//
    int error=0;
    if(m_dData==NULL){
        m_hData = (float*) malloc(16*sizeof(float));
        error = cudaMalloc(&m_dData, 16*sizeof(float));
    }
    if(error) return error;
    return SetMatrix(ptp,disp);
}

__host__ int GBHMat::InitMatrix(const FVec& Pose)
{
// 
// Initialise the memory and set up the transformation matrix
// Both GPU and Host memory are allocated for debugging 
// This overload is for FVec Pose data
//
//  Pose Vector order: Tx(Pitch), Ty(Yaw), Tz(Roll) (rads) X, Y, Z (metres)
//
    int error=0;
    if(m_dData==NULL){
        m_hData = (float*) malloc(16*sizeof(float));
        error = cudaMalloc(&m_dData, 16*sizeof(float));
    }
    if(error) return error;
    return SetMatrix(Pose);
}

__host__ int GBHMat::ReMapToOldPose(const FVec& Pose)
{
// 
// Initialise the memory and set up the transformation matrix
// Both GPU and Host memory are allocated for debugging 
// This overload is for FVec Pose data
//
//  Pose Vector order: Tx(Pitch), Ty(Yaw), Tz(Roll) (rads) X, Y, Z (metres)
//  This remaps Pose and creates the corresponding Matrix
//  ReMap      My Vector      Prev Vector
//             (0) Pitch to    (1) Roll
//             (1) Yaw   to    (2) Pitch
//             (2) Roll  to    (0) Yaw
    float Pitch, Roll, Yaw;
    Pitch = Pose(0); 
    Yaw = Pose(1); 
    Roll = Pose(2);
    Pose(0) = Yaw;
    Pose(1) = Pitch;
    Pose(2) = Roll;
//
    return SetMatrix(Pose);
}


__host__ int GBHMat::InitMatrix()
{
//
// It assumes [4,4] but no initialisation of angles and offset is made
//
    int error=0;
    if(m_dData==NULL){
        m_hData = (float*) malloc(16*sizeof(float));
        error = cudaMalloc(&m_dData, 16*sizeof(float));
    }
    if(error) return error;
    memset(m_hData,0,16*sizeof(float));
    error = UpdateGPU();
    return error;
}

__host__ void GBHMat::TransposeCopy(float* pCMat, int RowMax, int ColMax)
{
// copies a C-style matrix into Normal format
// Note this will only work with Matrices [4x4] or lower (like [3x3])
    int i, j, in;

    InitMatrix(); // [4x4]
    for(i=0;i<RowMax;i++){
        for(j=0;j<ColMax;j++){
            in = RowMax*i + j;
            operator()(i,j) = pCMat[in];
        }
    }
}

__host__ int GBHMat::SetMatrix(const float3 & ptp, const float3 & disp)
{
// init to a position, rotation and Ref xyzg
// ptp is stored phi(roll)(x), theta(pitch)(y), psi(yaw)(z)
// this buildup gives a Right Hand Set
//
    int i,error=0;

    FVec cs(3); FVec sn(3);
    cs(0) = cos(ptp.x);
    sn(0) = sin(ptp.x);
    cs(1) = cos(ptp.y);
    sn(1) = sin(ptp.y);
    cs(2) = cos(ptp.z);
    sn(2) = sin(ptp.z);
//
// Revised version based on developed set 29/08/22

    operator()(0,0) = cs(2)*cs(1);
    operator()(1,0) = sn(0)*sn(1)*cs(2) - cs(0)*sn(2);
    operator()(2,0) = cs(0)*sn(1)*cs(2) + sn(0)*sn(2);

    operator()(0,1) = sn(2)*cs(1);
    operator()(1,1) = sn(0)*sn(1)*sn(2) + cs(0)*cs(2);
    operator()(2,1) = cs(0)*sn(1)*sn(2) - sn(0)*cs(2);

    operator()(0,2) = -sn(1);
    operator()(1,2) = sn(0)*cs(1);
    operator()(2,2) = cs(0)*cs(1);

// Rotation matrix for computer graphics axis set as above
/*
    operator()(0,0) = sn(1)*sn(0)*sn(2)+cs(1)*cs(2);
    operator()(1,0) = sn(1)*sn(0)*cs(2)-cs(1)*sn(2);
    operator()(2,0) = sn(1)*cs(0);

    operator()(0,1) = cs(0)*sn(2);
    operator()(1,1) = cs(0)*cs(2);
    operator()(2,1) = -sn(0);

    operator()(0,2) = cs(1)*sn(0)*sn(2)-cs(2)*sn(1); 
    operator()(1,2) = cs(1)*sn(0)*cs(2)+sn(2)*sn(1); 
    operator()(2,2) = cs(1)*cs(0); 
*/
//
/*
// This is Aircraft Axis set
// Below is ordered: (0) = Phi = Tz;   (1) = Tx = Theta;   (2) = Ty = Psi
// this is the reverse of Mike's order
//
    operator()(0,0) = cs(2)*cs(1);
    operator()(1,0) = cs(2)*sn(1)*sn(0)-sn(2)*cs(0);
    operator()(2,0) = cs(2)*sn(1)*cs(0)+sn(2)*sn(0);

    operator()(0,1) = sn(2)*cs(1);
    operator()(1,1) = sn(2)*sn(1)*sn(0)+cs(2)*cs(0);
    operator()(2,1) = sn(2)*sn(1)*cs(0)-cs(2)*sn(0);

    operator()(0,2) = -sn(1);
    operator()(1,2) = cs(1)*sn(0);
    operator()(2,2) = cs(1)*cs(0);
*/

// the offset and last row [0,0,0|1]
    operator()(0,3) = disp.x;
    operator()(1,3) = disp.y;
    operator()(2,3) = disp.z;
    for(i=0;i<3;i++){
        operator()(3,i) = 0.0;
    }
    operator()(3,3)=1.0;
//
    error = UpdateGPU();
    return error;
}

__host__ float3 GBHMat::GetDisplacement()
{
    float3 disp;
    disp.x = operator()(0,3);
    disp.y = operator()(1,3);
    disp.z = operator()(2,3);
    return disp;
}

__host__ int GBHMat::SetMatrix(const FVec& AnglesAndDisp)
{
    FVec ptp, disp;
    ptp.ZeroInit(3);
    disp.ZeroInit(3);
    for(int i=0;i<3;i++){
        ptp(i)=AnglesAndDisp(i);
        disp(i)=AnglesAndDisp(i+3);
    }
    return SetMatrix(ptp, disp);
}

__host__ int GBHMat::SetMatrix(const FVec& ptp, const FVec& disp)
{
// init to a position, rotation and Ref xyzg
// ptp is stored phi(roll)(0), theta(pitch)(1), psi(yaw)(2)
// this buildup gives a Right Hand Set
//
    int i,error=0;
    FVec cs(3); FVec sn(3);
    cs(0) = cos(ptp(0));    // ThetaX  = Pitch = Theta
    sn(0) = sin(ptp(0));
    cs(1) = cos(ptp(1));    // ThetaY  = Yaw = Psi
    sn(1) = sin(ptp(1));
    cs(2) = cos(ptp(2));    // ThetaZ  = Roll = Phi
    sn(2) = sin(ptp(2));
//
// Revised version based on developed set 29/08/22

    operator()(0,0) = cs(2)*cs(1);
    operator()(1,0) = sn(0)*sn(1)*cs(2) - cs(0)*sn(2);
    operator()(2,0) = cs(0)*sn(1)*cs(2) + sn(0)*sn(2);

    operator()(0,1) = sn(2)*cs(1);
    operator()(1,1) = sn(0)*sn(1)*sn(2) + cs(0)*cs(2);
    operator()(2,1) = cs(0)*sn(1)*sn(2) - sn(0)*cs(2);

    operator()(0,2) = -sn(1);
    operator()(1,2) = sn(0)*cs(1);
    operator()(2,2) = cs(0)*cs(1);

// Rotation matrix for computer graphics axis set as above
/*
    operator()(0,0) = sn(1)*sn(0)*sn(2)+cs(1)*cs(2);
    operator()(1,0) = sn(1)*sn(0)*cs(2)-cs(1)*sn(2);
    operator()(2,0) = sn(1)*cs(0);

    operator()(0,1) = cs(0)*sn(2);
    operator()(1,1) = cs(0)*cs(2);
    operator()(2,1) = -sn(0);

    operator()(0,2) = cs(1)*sn(0)*sn(2)-cs(2)*sn(1); 
    operator()(1,2) = cs(1)*sn(0)*cs(2)+sn(2)*sn(1); 
    operator()(2,2) = cs(1)*cs(0); 
*/
//
/*
// This is Aircraft Axis set
// Below is ordered: (0) = Phi = Tz;   (1) = Tx = Theta;   (2) = Ty = Psi
// this is the reverse of Mike's order
//
    operator()(0,0) = cs(2)*cs(1);
    operator()(1,0) = cs(2)*sn(1)*sn(0)-sn(2)*cs(0);
    operator()(2,0) = cs(2)*sn(1)*cs(0)+sn(2)*sn(0);

    operator()(0,1) = sn(2)*cs(1);
    operator()(1,1) = sn(2)*sn(1)*sn(0)+cs(2)*cs(0);
    operator()(2,1) = sn(2)*sn(1)*cs(0)-cs(2)*sn(0);

    operator()(0,2) = -sn(1);
    operator()(1,2) = cs(1)*sn(0);
    operator()(2,2) = cs(1)*cs(0);
*/
    // the offset and last row [0,0,0|1]
    for(i=0;i<3;i++){
        operator()(i,3) =  disp(i);
        operator()(3,i) = 0.0;
    }
    operator()(3,3)=1.0;
//
    error = UpdateGPU();
    return error;
}

__host__ float GBHMat::determinant()
{
// determinant of the [3x3] rotation matrix
//
  float det1 = operator()(0,0)*( operator()(1,1)*operator()(2,2) - operator()(2,1)*operator()(1,2) );
  float det2 = operator()(0,1)*( operator()(1,0)*operator()(2,2) - operator()(2,0)*operator()(1,2) );
  float det3 = operator()(0,2)*( operator()(1,0)*operator()(2,1) - operator()(2,0)*operator()(1,1) );
  return (det1 - det2 + det3);
}

__host__ float GBHMat::normf()
{
//
// returns the Frobenius norm of the matrix
// this is not the L2 or spectral norm
//
    int i,j;
    float sum=0.0;
    for(i=0; i<4; i++){
        for(j=0; j<4; j++){
          sum+= operator()(i,j)*operator()(i,j);
        }
    }
    if(sum>0.0f){
        return sqrt(sum);
    }
    return 0.0f;
}	

__host__ int GBHMat::SetDisplacement(const float3 & disp)
{
// independently of Rotation
    // the offset and last row [0,0,0|1]
    operator()(0,3) = disp.x;
    operator()(1,3) = disp.y;
    operator()(2,3) = disp.z;
    operator()(3,3) = 1.0f;
//
    int error = UpdateGPU();
    return error;
}

__host__ int GBHMat::SetDisplacement(const FVec& disp)
{
// independently of Rotation
    // the offset and last row [0,0,0|1]
    operator()(0,3) = disp(0);
    operator()(1,3) = disp(1);
    operator()(2,3) = disp(2);
    operator()(3,3) = 1.0f;
//
    int error = UpdateGPU();
    return error;
}



__host__ int GBHMat::UpdateGPU()
{
//    Update the companion GPU matrix
    cudaError_t error;
    error = cudaMemcpy(m_dData, m_hData, 16*sizeof(float), cudaMemcpyHostToDevice);
    if(error!=cudaSuccess){
        const char* pError = cudaGetErrorString(error);
        printf(" UpdateGPU Copy Error %s\n",pError);
        return -1;
    }
    return 0;
}

__host__ int GBHMat::UpdateHost()
{
// reverse of the above
    cudaError_t error;
    error = cudaMemcpy(m_hData, m_dData, 16*sizeof(float), cudaMemcpyDeviceToHost);
    if(error!=cudaSuccess){
        const char* pError = cudaGetErrorString(error);
        printf(" UpdateHost Copy Error %s\n",pError);
        return -1;
    }
    return 0;
}

__host__ GBHMat& GBHMat::operator=(GBHMat& A)
{
    int i,j;
    InitMatrix();

    for(i=0;i<4;i++){
        for(j=0;j<4;j++){
            operator()(i,j) = A(i,j);
        }
    }
    UpdateGPU();
    return *this;
}


__host__ GBHMat& GBHMat::operator*=(GBHMat& A)
{
// compound in place both matrices must be dimensioned
// and correctly intialised, 
// this will only work as a host function

    int i, j, k;

    GBHMat C;
    C.InitMatrix();
    for(i=0;i<4;i++){
        for(j=0;j<4;j++){
            for(k=0;k<4;k++){
                C(i,j) += operator()(i,k)*A(k,j);
            }
        }
    }
    for(i=0;i<4;i++){
        for(j=0;j<4;j++){
            operator()(i,j) = C(i,j);
        }
    }
    UpdateGPU();
    return *this;
}

__host__ GBHMat& GBHMat::operator^=(GBHMat& A)
{
// compound in place both matrices must be dimensioned
// and correctly intialised, 
// this will only work as a host function

int i, j, k;

    FMat C;
    C.ZeroInit(4,4);
    for(i=0;i<4;i++){
        for(j=0;j<4;j++){
            for(k=0;k<4;k++){
                C(i,j) += operator()(i,k)*A(j,k);
            }
        }
    }

    for(i=0;i<4;i++){
        for(j=0;j<4;j++){
            operator()(i,j) = C(i,j);
        }
    }
    UpdateGPU();
    return *this;
}

__host__ GBHMat& GBHMat::Transpose(GBHMat& Trans)
{
//
// the rotation matrix is transposed and the
// offset part is inverted
//
    int i, j;
    InitMatrix();
    for(i=0;i<3;i++){
        for(j=0;j<3;j++){
            operator()(i,j) = Trans(j,i);
        }
        operator()(i,3) = -Trans(i,3);
    }
    operator()(3,3) = 1.0;  
    UpdateGPU();
    return *this;
}

__host__ float3 GBHMat::RotateH(const float3& xyzg)
{
// rotate (but not translate) global vector to body axis
// on host 
// Warning GPU matrix is not updated
//
    float3 xyzb;

    xyzb.x  = operator()(0,0)*xyzg.x;
    xyzb.x += operator()(0,1)*xyzg.y;
    xyzb.x += operator()(0,2)*xyzg.z;

    xyzb.y  = operator()(1,0)*xyzg.x;
    xyzb.y += operator()(1,1)*xyzg.y;
    xyzb.y += operator()(1,2)*xyzg.z;

    xyzb.z  = operator()(2,0)*xyzg.x;
    xyzb.z += operator()(2,1)*xyzg.y;
    xyzb.z += operator()(2,2)*xyzg.z;
    return xyzb;
}

__host__ float3 GBHMat::InvRotateH(const float3& xyzb)
{
  float3 xyzg;

  xyzg.x  = operator()(0,0)*xyzb.x;
  xyzg.x += operator()(1,0)*xyzb.y;
  xyzg.x += operator()(2,0)*xyzb.z;

  xyzg.y  = operator()(0,1)*xyzb.x;
  xyzg.y += operator()(1,1)*xyzb.y;
  xyzg.y += operator()(2,1)*xyzb.z;

  xyzg.z  = operator()(0,2)*xyzb.x;
  xyzg.z += operator()(1,2)*xyzb.y;
  xyzg.z += operator()(2,2)*xyzb.z;
  return xyzg;
}

__host__ FVec GBHMat::InvRotateH(const FVec& xyzb)
{
    FVec Out;
    Out.ZeroInit(3);
    for(int i=0;i<3;i++){
        for(int j=0;j<3;j++){
            Out(i) = Out(i) + operator()(j,i)*xyzb(j);
        }
    }
    return Out;
}

__device__ float3 GBHMat::operator*(const float3& xyzg)
{
//
// Transform from global axes to body Xb = TBG*(Xg-Xg0)
// into xyzg -> xyzb
//
// For this GPU version loops are unrolled and used float3
// rather than FVec since float3 is a type known to the GPU
//
    float3 xyzb;
    const float dxg = xyzg.x-Get(0,3);
    const float dyg = xyzg.y-Get(1,3);
    const float dzg = xyzg.z-Get(2,3);

    xyzb.x  = Get(0,0)*dxg;
    xyzb.x += Get(0,1)*dyg;
    xyzb.x += Get(0,2)*dzg;

    xyzb.y  = Get(1,0)*dxg;
    xyzb.y += Get(1,1)*dyg;
    xyzb.y += Get(1,2)*dzg;

    xyzb.z  = Get(2,0)*dxg;
    xyzb.z += Get(2,1)*dyg;
    xyzb.z += Get(2,2)*dzg;

    return xyzb;
}

__device__ float3 GBHMat::Rotate(const float3& xyzg)
{
// rotate from global to body axes but not translate xyzg ->xyzb
// Transform from global axes to body Xb = TBG*Xg
// into xyzg -> xyzb
//
// For this GPU version loops are unrolled and used float3
// rather than FVec since float3 is a type known to the GPU
//
    float3 xyzb;

    xyzb.x  = Get(0,0)*xyzg.x;
    xyzb.x += Get(0,1)*xyzg.y;
    xyzb.x += Get(0,2)*xyzg.z;

    xyzb.y  = Get(1,0)*xyzg.x;
    xyzb.y += Get(1,1)*xyzg.y;
    xyzb.y += Get(1,2)*xyzg.z;

    xyzb.z  = Get(2,0)*xyzg.x;
    xyzb.z += Get(2,1)*xyzg.y;
    xyzb.z += Get(2,2)*xyzg.z;
    return xyzb;
}

__device__ float3 GBHMat::InvR(const float3& xyzb)
{
//
// transform from body axes to global Xg = (TBG^-1)*Xb + Xg0
// xyzb -> xyzg
//
    float3 xyzg;

    xyzg.x  = Get(0,0)*xyzb.x;
    xyzg.x += Get(1,0)*xyzb.y;
    xyzg.x += Get(2,0)*xyzb.z;
    xyzg.x += Get(0,3);

    xyzg.y  = Get(0,1)*xyzb.x;
    xyzg.y += Get(1,1)*xyzb.y;
    xyzg.y += Get(2,1)*xyzb.z;
    xyzg.y += Get(1,3);

    xyzg.z  = Get(0,2)*xyzb.x;
    xyzg.z += Get(1,2)*xyzb.y;
    xyzg.z += Get(2,2)*xyzb.z;
    xyzg.z += Get(2,3);
    return xyzg;
}
__device__ float3 GBHMat::InvRotate(const float3& xyzb)
{
// inverse rotate from body axis to global without translation
//
// transform from body axes to global Xg = (TBG^-1)*Xb
// xyzb -> xyzg
//
    float3 xyzg;

    xyzg.x  = Get(0,0)*xyzb.x;
    xyzg.x += Get(1,0)*xyzb.y;
    xyzg.x += Get(2,0)*xyzb.z;

    xyzg.y  = Get(0,1)*xyzb.x;
    xyzg.y += Get(1,1)*xyzb.y;
    xyzg.y += Get(2,1)*xyzb.z;

    xyzg.z  = Get(0,2)*xyzb.x;
    xyzg.z += Get(1,2)*xyzb.y;
    xyzg.z += Get(2,2)*xyzb.z;

    return xyzg;
}

__host__ 	FVec& GBHMat::GetPose()
{
//
// Recovers pose data from the rotation matrix
// returns the 6-vector {Thx,Thy,ThZ,tx,ty,tz}
// Note this is setup for graphical axes convensions
// X to right, Y down and Z into screen 
// with datum (0,0,0) typically top Y, front Z, left X
// but can also be vehicle-body or view-volume centric
//
    int i;
    static FVec Ret;
    Ret.ReSize(6);
    UpdateHost();
//
// Revised from build up set 29/08/22
// however it does not work
/*
    float T02;
    T02 = my_fmax( -1.0f, my_fmin( 1.0f, operator()(0,2) ));  
    Ret(0) = my_atan2(operator()(2,2), operator()(1,2));
    Ret(1) = -asin(T02);                                    // Ty = asin(sin(Ty))
    Ret(2) = my_atan2(operator()(0,0), operator()(0,1));
*/
//
// This is in the order for conventional Computer Graphics
// where Tx = Theta, Ty = Psi, Tz = phi are the substitutions
// for the standard right hand set
//
    float T02;
    T02 = my_fmax( -1.0f, my_fmin( 1.0f, operator()(2,1) ));  
    Ret(0) = -asin(T02);                                    // Tx = asin(sin(Tx)
    Ret(1) = my_atan2( operator()(2,2), operator()(2,0) );  // Ty = atan( cosTy, sinTy )
    Ret(2) = my_atan2( operator()(1,1), operator()(0,1));   // atan( cosTz, sinTz )
/*
// This is in the order: Phi, Theta, Psi and using Aircraft axes systems

    float T02;
    T02 = my_fmax( -1.0f, my_fmin( 1.0f, operator()(0,2) ));  

    Ret(0) =  my_atan2( operator()(2,2), operator()(1,2) );   // my_atan2( operator()(2,2), operator()(1,2) );
    Ret(1) = -asin( T02 );
    Ret(2) =  my_atan2( operator()(0,0), operator()(0,1) );   // my_atan2( operator()(0,0), operator()(0,1) );   
*/
    for(i=0;i<3;i++){
        Ret(i+3)=operator()(i,3);
    }
    return Ret;
}

__host__ void GBHMat::printPose(const char* pText, int EndLn, FILE* pFile)
{
    int i;
    FVec Out;
    Out = GetPose();
    if(pFile==NULL){
        if(pText!=NULL){
            printf("%s ",pText);
        }
        else {
            printf("Pose ");        
        }
        for(i=0;i<3;i++){
            printf(" %f, ",Out(i)*DPR);
        }
        for(i=3;i<6;i++){
            printf(" %f, ",Out(i));
        }
        if(EndLn) printf("\n");
    } else {
        if(pText!=NULL){
            fprintf(pFile,"%s ",pText);
        }
        else {
            fprintf(pFile,"Pose ");        
        }
        for(i=0;i<3;i++){
            fprintf(pFile," %f, ",Out(i)*DPR);
        }
        for(i=3;i<6;i++){
            fprintf(pFile," %f, ",Out(i));
        }
        if(EndLn) fprintf(pFile,"\n");
    }
}

__host__ void GBHMat::printMatrix(const char* pName, FILE* pFile)
{
    int i,j;
    UpdateHost();
    if(pFile!=NULL){
        if(pName!=NULL){
            fprintf(pFile,"\n %s\n",pName);
        }
        for(i=0;i<4;i++){
            for(j=0;j<4;j++){
                fprintf(pFile," %f, ",operator()(i,j));
            }
            fprintf(pFile,"\n");
        }
        fprintf(pFile,"\n");
    } else {
        if(pName!=NULL){
            printf("\n %s\n",pName);
        }
        for(i=0;i<4;i++){
            for(j=0;j<4;j++){
                printf(" %f, ",operator()(i,j));
            }
            printf("\n");
        }
        printf("\n");
    }
}

__device__ float3 paramRay(Ray r, float t) 
{
  return r.o + t*(r.d);
}

__device__ float face(int s, float val1)
{

// this maps the camera 16bit data representation of distance into the voxel z-dimension
// thus zval represents the measured depth interface for this pixel
// This only works if the x-y axis frame of the volume matches that of
// the camera and screen image in both orientation and dimensions. Thus the volume
// will also be camera alligned regardless of view orientation
//
  float val;
  int zval;
// zval is the z-voxel corresponding to the camera depth value
  zval = int(val1*zscale);

  if(zval<0) zval = 0;

  if(zval>VOL_NZ) zval = VOL_NZ;

  val = abs(float(zval - s));  // s is the current z-voxel

  if(val>1.f){
    val = 1.f;
  }
//
  return val;
}

__device__ float func(int c, int r, int s, int Shape, int3 volSize, float4 params)
{
//
// for the given position indicated by x=c,y=r,z=s, and the given function
// selected by Shape compute the density value at that location based on the geometry
// and params = {32.f, 21.334f, 8f., 1.0f}
// I have modified this to include an additional off-centre sphere (Shape==1)
// The axis system origin is in the top left front corner of the world
// as presented in the viewport, as expected given conventional window coordinates
// More work needed to resolve this and other issues.
//
  const int3 pos0 = { volSize.x/2, volSize.y/2, volSize.z/2 };
  const int3 pos1 = { volSize.x/8, volSize.y/8, volSize.z/8 };

  const float dx = c - pos0.x, dy = r - pos0.y, dz = s - pos0.z;
  const float dx2 = c - pos1.x, dy2 = r - pos1.y, dz2 = s - pos1.z;

  float x, y, z, rt, big, small, taurus, cent;

  switch (Shape) {
    case 0:
// sphere
      big = sqrtf(dx*dx + dy*dy + dz*dz) - params.x;
      if(big>THRESH){
        big = THRESH;
      }
      return big;
    case 1:
// two spheres one big in centre and one small in upper left front corner
      big = sqrtf(dx*dx + dy*dy + dz*dz) - params.x;
      if(big>THRESH){
        big = THRESH;
      }
      small =sqrtf(c*c + r*r + s*s) - params.y;
      if(small>THRESH){
        small = THRESH;
      }
      return (big+small);
    case 2:
// torus
      rt = sqrtf(dx*dx + dy*dy);
      taurus = sqrtf((rt - params.x)*(rt - params.x) + dz*dz) - params.y;
      return taurus;
    case 3:
// torus + two small spheres one in centre of torus and one front upper left 
      rt = sqrtf(dx*dx + dy*dy);
      taurus = sqrtf((rt - params.x)*(rt - params.x) + dz*dz) - params.y;
      if(taurus>THRESH){
        taurus = THRESH;
      }
      small =sqrtf(dx2*dx2 + dy2*dy2 + dz2*dz2) - params.z;
      if(small>THRESH){
        small = THRESH;
      }
        cent =sqrtf(dx*dx + dy*dy + dz*dz) - params.z;
      if(cent>THRESH){
        cent = THRESH;
      }
      return (taurus+cent+small);
    case 4: 
// block
      x = fabsf(dx) - params.x;
      y = fabsf(dy) - params.y;
      z = fabsf(dz) - params.z;

      if (x <= 0 && y <= 0 && z <= 0){
        return fmaxf(x, fmaxf(y, z));
      }else {
        x = fmaxf(x, 0), y = fmaxf(y, 0), z = fmaxf(z, 0);
        return sqrtf(x*x + y*y + z*z);
      }
    default:
      return 0.f;

  }
}

__device__ float planeSDF(float3 pos, float3 norm, float d) 
{
  return dot(pos, normalize(norm)) - d;
}

__device__ bool rayPlaneIntersect(Ray myRay, float3 n, float dist, float *t) 
{
  const float f0 = planeSDF(paramRay(myRay, 0.f), n, dist);
  const float f1 = planeSDF(paramRay(myRay, 1.f), n, dist);
  bool result = (f0*f1 < 0);
  if (result){
    *t = (0.f - f0) / (f1 - f0);
  }
  return result;
}

// Intersect ray with a box from volumeRender SDK sample.
__device__ bool intersectBox(Ray r, float3 boxmin, float3 boxmax, float *tnear, float *tfar) 
{
  // Compute intersection of ray with all six bbox planes.
  const float3 invR = make_float3(1.0f) / r.d; // = { 1/r.d.x, 1/r.d.y, 1/r.d.z }
  const float3 tbot = invR*(boxmin - r.o);     // These are consistent and element wise
  const float3 ttop = invR*(boxmax - r.o);     // i.e. tbot.x = invR.x*(boxmin.x - r.o.x) etc.
  
  // Re-order intersections to find smallest and largest on each axis.
  const float3 tmin = fminf(ttop, tbot);
  const float3 tmax = fmaxf(ttop, tbot);

  // Find the largest tmin and the smallest tmax.
  *tnear = fmaxf( fmaxf(tmin.x, tmin.y), fmaxf(tmin.x, tmin.z) );
  *tfar = fminf( fminf(tmax.x, tmax.y), fminf(tmax.x, tmax.z) );
  
  return *tfar > *tnear;
}

__device__ float density(float *d_vol, int3 volSize, float3 pos) 
{

  int3 index = posToVolIndex(pos, volSize);

  int i = index.x, j = index.y, k = index.z;

  const int w = volSize.x, h = volSize.y, d = volSize.z;
  
  const float3 rem = fracf(pos);
  
  index = make_int3(clipWithBounds(i, 0, w - 2),  clipWithBounds(j, 0, h - 2), clipWithBounds(k, 0, d - 2));
  
  // directed increments for computing the gradient
  const int3 dx = { 1, 0, 0 }, dy = { 0, 1, 0 }, dz = { 0, 0, 1 };
  
  // values sampled at surrounding grid points
  const float dens000 = d_vol[flatten(index, volSize)];
  const float dens100 = d_vol[flatten(index + dx, volSize)];
  const float dens010 = d_vol[flatten(index + dy, volSize)];
  const float dens001 = d_vol[flatten(index + dz, volSize)];
  const float dens110 = d_vol[flatten(index + dx + dy, volSize)];
  const float dens101 = d_vol[flatten(index + dx + dz, volSize)];
  const float dens011 = d_vol[flatten(index + dy + dz, volSize)];
  const float dens111 = d_vol[flatten(index + dx + dy + dz, volSize)];
  // trilinear interpolation
  return (1 - rem.x)*(1 - rem.y)*(1 - rem.z)*dens000 +
    (rem.x)*(1 - rem.y)*(1 - rem.z)*dens100 +
    (1 - rem.x)*(rem.y)*(1 - rem.z)*dens010 +
    (1 - rem.x)*(1 - rem.y)*(rem.z)*dens001 +
    (rem.x)*(rem.y)*(1 - rem.z)*dens110 +
    (rem.x)*(1 - rem.y)*(rem.z)*dens101 +
    (1 - rem.x)*(rem.y)*(rem.z)*dens011 +
    (rem.x)*(rem.y)*(rem.z)*dens111;
}

__device__ float MyDensity(float *d_vol, int3 volSize, float3 pos) 
{

  int3 index = posToVolIndex(pos, volSize);

  int i = index.x, j = index.y, k = index.z;

  const int w = volSize.x, h = volSize.y, d = volSize.z;
  
//  index = make_int3(clipWithBounds(i, 0, w - 2),  clipWithBounds(j, 0, h - 2), clipWithBounds(k, 0, d - 2));
  index = make_int3(clipWithBounds(i, 0, w-1),  clipWithBounds(j, 0, h-1), clipWithBounds(k, 0, d-1));

  // values sampled at the grid point
  const float dens000 = d_vol[flatten(index, volSize)];
  return dens000;

}

__device__ uchar4 sliceShader(float *d_vol, int3 volSize, Ray boxRay, float gain, float dist, float3 norm)
{
  float t;
  uchar4 shade = make_uchar4(96, 0, 192, 0); // background value
  if (rayPlaneIntersect(boxRay, norm, dist, &t)) {
    float sliceDens = density(d_vol, volSize, paramRay(boxRay, t));
    shade = make_uchar4(48, clip(-10.f * (1.0f + gain) * sliceDens), 96, 255);
  }
  return shade;
}

__device__ uchar4 MySliceShader(float *d_vol, int3 volSize, Ray boxRay, float gain, float dist, float3 norm, unsigned int *pPallet, int colours)
{
//
// This is used to render a slice of the current view volume
// Colour coding is based on the value within the given voxel
//
  float t;
  int idx, Index = 1;
  uchar4 shade = make_uchar4(96, 0, 192, 0); // background value
  if (rayPlaneIntersect(boxRay, norm, dist, &t)) {
    float Dens = MyDensity(d_vol, volSize, paramRay(boxRay, t));
    float test = Dens*100.0;
// nonlinear colour scheme focused on the range -1 <= Dens <= 1
// using values from the Rule250 colour palette
    if(test<-9.0) Index = 1;                                 // dark blue
    if( (test>=-9.0) && (test<-EPS)) Index = int(10.0+test)*10; // blues-to-green
    if(fabsf(test)<=EPS) Index=10;                           // black
    if(test>EPS && (test<=9.0)) Index = int(10.0+test)*10;      // yellow-to-red
    if(test>9.0) Index = 190;                                 // red
    idx = Index%colours;
    shade = ChooseColour(pPallet, idx);
  }
  return shade;
}

__device__ uchar4 volumeRenderShader(float *d_vol, int3 volSize, Ray boxRay, float threshold, int numSteps) 
{
  uchar4 shade = make_uchar4(96, 0, 192, 0); // background value
  const float dt = 1.f / numSteps;
  const float len = length(boxRay.d) / numSteps;
  float accum = 0.f;
  float3 pos = boxRay.o;
  float val = density(d_vol, volSize, pos);
  for (float t = 0.f; t<1.f; t += dt) {
    if (val - threshold < 0.f) accum += (fabsf(val - threshold))*len;
    pos = paramRay(boxRay, t);
    val = density(d_vol, volSize, pos); 
  }
  if (clip(accum) > 0.f) shade.y = clip(accum);
  return shade;
}

__device__ uchar4 rayCastShader(float *d_vol, int3 volSize, Ray boxRay, float threshold) 
{
// background set in RenderKernel is { 64, 0, 128, 0 };
// This is only a slightly different shade
// 
  uchar4 shade = make_uchar4(0, 0, 192, 0);  // blue
  float3 pos = boxRay.o;

  float len = length(boxRay.d); // len = sqrt(V.V)

  float t = 0.0f;

  float f = density(d_vol, volSize, pos);

  while (f > threshold + EPS && t < 1.0f) {
    f = density(d_vol, volSize, pos);
    t += (f - threshold) / len;
    pos = paramRay(boxRay, t);
    f = density(d_vol, volSize, pos);
  }

  if (t < 1.f) {
    const float3 ux = make_float3(1, 0, 0), uy = make_float3(0, 1, 0),
                 uz = make_float3(0, 0, 1);
    float3 grad = {(density(d_vol, volSize, pos + EPS*ux) -
                    density(d_vol, volSize, pos))/EPS,
                   (density(d_vol, volSize, pos + EPS*uy) -
                   density(d_vol, volSize, pos))/EPS,
                   (density(d_vol, volSize, pos + EPS*uz) -
                   density(d_vol, volSize, pos))/EPS};
//
    float intensity = -dot(normalize(boxRay.d), normalize(grad));
    shade = make_uchar4(255 * intensity, 0, 0, 255);
  }

  return shade;
}

__device__ uchar4 MyRayShader(float *d_vol, int3 volSize, Ray boxRay, float threshold, unsigned int *pPallet, int colours) 
{
//
// background set in RenderKernel is { 64, 0, 128, 0 };
// This is only a slightly different shade
//
  int index, idx;
  uchar4 shade = make_uchar4(0, 0, 192, 0);  // blue

  float3 pos = boxRay.o;

  float len = length(boxRay.d);

  float t = 0.0f;

//###  float f = density(d_vol, volSize, pos);

  float f = MyDensity(d_vol, volSize, pos);

  while (f > threshold + EPS && t < 1.0f) {
    t += (f - threshold) / len;
    pos = paramRay(boxRay, t);
//###    f = density(d_vol, volSize, pos);
    f = MyDensity(d_vol, volSize, pos);
  }

  if ( (f < 1.f) && (f > -EPS) ) {
//
//    float intensity = -dot(normalize(boxRay.d), normalize(grad));
//    shade = make_uchar4(255 * intensity, 255*(abs(1.0-intensity)), 0, 255);
// colour on the z-depth only
      index = (int)pos.z + volSize.z/2;
      idx = index%colours;
      shade = ChooseColour(pPallet, idx);
      if( (pos.x==0.0) && (pos.y==0.0)){
//###          printf(" pos z %.2f, index %d, idx %d \n",pos.z, index, idx);
      }

  }

  return shade;
}

__device__ uchar4 MySimpleShader(float *d_vol, int3 volSize, Ray boxRay, float threshold, unsigned int *pPallet, int colours)
{
// This is not TSDF it is an occupancy model
//
// The bxRay contains a pixel ray into the model volume
// if it detects a surface f>0.5 
 
// background set in RenderKernel is { 64, 0, 128, 0 };
// This is only a slightly different shade
//
    int index, idx;
    uchar4 shade = make_uchar4(0, 0, 192, 0);  // blue

    float3 pos = boxRay.o;

    float len = length(boxRay.d);
  
    float step = 1.0f;
    float f = 0.0;
    float t = 0.0;
//
// chase along boxRay for a surface cell
// according to the MyDensity. This density function
// does not interpolate across adjcent cells this could
// present a problem if shading fast
//
    while (t < 1.0f && f < 0.5) {
        t += step/ len;
        pos = paramRay(boxRay, t);
        f = MyDensity(d_vol, volSize, pos);   // crude but fast!
//        f = density(d_vol, volSize, pos);   // linear interpolation (TSDF)
    }
//
// if a surface detected colour it according to depth in volume z axis
//  
    if ( f> 0.5) {
// 
// colour on volume z-depth
//
        index = (int)pos.z + volSize.z/2;
        idx = index%colours;
        shade = ChooseColour(pPallet, idx);
    }
    return shade;
}

__device__ uchar4 ChooseColour(unsigned int* pPallet, int idx )
{
    unsigned char b,g,r;
    unsigned int icol;

//    clipWithBounds(idx, 0, colours);
    icol = pPallet[idx];

	  b = (unsigned char)(icol&255);
	  g = (unsigned char)((icol>>8)&255);
	  r = (unsigned char)((icol>>16)&255);
    return make_uchar4( r, g, b ,255);
}

