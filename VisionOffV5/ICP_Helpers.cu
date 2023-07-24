//
// ICP_Helpers.cu
//
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <helper_math.h>
#include "../Subs/constants.h"

#include "../Subs/FMat.h"
#include "../Subs/GMat.cuh"
#include "../Subs/device_funcs.cuh"

#include "../Subs/SVD_AS.h"

#include "VisionOffV5.h"
#include "kernel.h"
#include "ICP_Helpers.cuh"
#include "P2P_ICP.cuh"

#include "ControlParams.h"


__global__ 
void reset_pointVec_element(float* pointVec, int2 pntSz, int pointCount, int* ipVec)
{
    const int pntIdx = threadIdx.x + blockIdx.x * blockDim.x;    // indexes pointVec
    if(pntIdx>=pointCount){
        return;
    }
//
// Caution this resets element any given element of pointVec
//
    pointVec[pntIdx + PVEC_ELEMENT*pntSz.x] = 1.0;   // fTest
    ipVec[pntIdx ] =  -1;   //  prevIdx
}


int Reset_pointVec_element_Launch(dim3 grid, dim3 TBP, float* pointVec, int2 pntSz, int pointCount, int* ipVec)
{
    reset_pointVec_element<<< grid, TBP >>>(pointVec, pntSz, pointCount, ipVec);
    cudaDeviceSynchronize();
    cudaError_t CudaErr = cudaGetLastError();
    if(CudaErr!=cudaSuccess){
        const char* pError = cudaGetErrorString(CudaErr);
        printf("Reset_pointVec_element %s\n",pError);
        return -1;
    }
    return 0;
}
//
//-------------------------------------------------------------------------------------------------
//
__global__ 
void point_pair_match_2(float* pointVec, int2 pntSz, int pointCount, int* ipVec, float* refVec, int refCount, float DistMin, float CosMin, int* dMatch, int* dAtomVec)
{
//
// This version 2 works with refVec rather than ref_tex and is similar to point_pair_Match() above.
// 
// Arguments:
//  pointVec    set of vertices for point-pair matching for this frame in vertial order
//  pntSz       size of pointVec
//  pointCount  Number of vertices in pointVec
//  ipVec       vector containing flags used in pair matching
//  refVec      Reference vertices from previous frame stored in same format as pointVec
//  refCount    Number of vertices in refVec, in general refCount != pointCount
//  DistMin     The minimum L2 distance to be a qualifying point-pair match
//  CosMin      the minimum cos(theta) value i.e. cos(30) = 0.866 to qualify (typically 0.95 or greater)
//  dMatch      Contains indices into pointVec that most closely matches refVec
//              Thus: pointVec[dMatch[refIdx]] ~ refVec[refIdx]
//              if dMatch[i] < 0 the point is to be excluded from further processing
//  dAtomVec[3] Count of non-negative point matches, (MatchCount) must be zero on launch 
//  dAtomVec[4] Count of broken matches
//
// key #define
//  PVEC_ELEMENT     Index of pointVec element for pair matching
//
// these locals could be placed in dWorkVec however check for register spill
//
    float Xp[6], Xr[6], Dx[6];
    int i, Points;
//
    const int refIdx = threadIdx.x + blockIdx.x * blockDim.x;

// Do nothing if we are out of array bounds
    if (refIdx >= refCount) return;

//    if(pntSz.y<5){
//        Points = 3;
//    } else {
        Points = 6;
//    }

// the refVec point to compare in this thread

    for(i=0;i<Points;i++){
        Xr[i] = refVec[refIdx + i*pntSz.x];
    }
    float savMin = 1.0E+3;
    int idxMin = -1;
    dMatch[refIdx] = idxMin;
//
// test this point against all points in pointVec
//
    for (int pntIdx=0; pntIdx<refCount; pntIdx++) {

        for(i=0;i<Points;i++){
            Xp[i] = pointVec[pntIdx + i*pntSz.x];
            Dx[i] = Xp[i] - Xr[i];
        }
  
        float TestD = norm3df( Dx[0], Dx[1], Dx[2] );  // the L2 norm of the vertex delta
//
// first an unconstrained point-pair matching 
// selects the closest match every time
//
        if(TestD<savMin){
            savMin=TestD;
            idxMin=pntIdx;
        }
    }
//
// Is the best match found within the tollerance criteria DistMin and CosMin?
// These criteria could be refined as iteration proceeds. Note the cosMin
// computation requires quite a lot of effort
//
    if(savMin <= DistMin){
        float Sxr=0.0;
        float Sxp=0.0;
        float Arp=0.0;
        for(i=0;i<Points;i++){
            Xp[i] = pointVec[idxMin + i*pntSz.x];
            Arp += Xr[i]*Xp[i];
            Sxr += Xr[i]*Xr[i];
            Sxp += Xp[i]*Xp[i];
        }
        Sxr = sqrtf(Sxr);
        Sxp = sqrtf(Sxp);
        const float cosRP = fabsf(Arp)/(Sxr*Sxp);

        if(cosRP > CosMin){
            float* fTest = &pointVec[idxMin + PVEC_ELEMENT*pntSz.x];
            int* prevIdx = &ipVec[idxMin];
            if(savMin < fTest[0]){
                atomicExch(fTest, savMin);
                if(prevIdx[0] > -1){           // if there was a previous match
                    dMatch[prevIdx[0]]=-1;     // break it
                    atomicAdd(&dAtomVec[4],1);  // count the number of breaks
                    atomicAdd(&dAtomVec[3],-1); // decrement the counter to remove the prevIdx
                }
                atomicAdd(&dAtomVec[3],1);   // increment pair-matches 
                atomicExch(prevIdx,refIdx);
                dMatch[refIdx] = idxMin;
            }else{
                idxMin = -idxMin;            // failed to match better than fTest
            }
        } else{
            idxMin = -idxMin;            // failed on CosMin
        }
    } else {
        idxMin = -idxMin;                // failed on DistMin
    }
//
// an index < 0 indicates no qualifying match was found for this point
//
    dMatch[refIdx]=idxMin;
}

int point_pair_match_2_Launch(dim3 grid, dim3 TBP, float* pointVec, int2 pntSz, int pointCount, int* ipVec,
                                float* refVec, int refCount, float DistVal, float CosVal, int* dMatch, int* dAtomVec)
{

    point_pair_match_2<<< grid, TBP >>>(pointVec, pntSz, pointCount, ipVec, refVec, refCount,  DistVal,  CosVal, dMatch, dAtomVec);
    cudaDeviceSynchronize();
    cudaError_t CudaErr = cudaGetLastError();
    if(CudaErr!=cudaSuccess){
        const char* pError = cudaGetErrorString(CudaErr);
        printf("point_pair_match_2 %s\n",pError);
        return -1;
    }
    return 0;
}

//
//---------------------------------------------------------------------------------------------------------------
//
__global__ void get_tex_value(int i, int k, cudaTextureObject_t ref_tex, int refCount, float* dResVec)
{
//
// tedious but the only way to check for correct indexing of
// a texture is in a device function
//
    dResVec[7] = tex2D<float>(ref_tex, (float)k, (float)i);
}
//
//-------------------------------------------------------------------------------------------------
//
// these two are used internally by compute_mean
__device__ unsigned int countMean = 0;
__shared__ bool compute_meanDone;

__global__ void  compute_Mean_2(float* pointVec, int2 pntSz, int pointCount, float* refVec, int refCount, 
                            int* dMatch, int MatchCount, int SegSzX, float* WorkVec, int workSzX, float*dResVec, int* dAtomVec)
{
//
// Computes the mean of all vertex in refVec and pointVec returns this in components of dResVec
// This version only computes the means for matched vector elements in both pointVec and refVec
// it should be called after every point_pair_match_2 call to keep the means in sync with
// the pair matching process
//
// grid             Grid size
// TPB              Threds per block
// pointVec         Vector of vertex data to be summed
// pntSz            Dimensions of pointVec and refVec
// pointCount       The actual number of vertex in pointVec
// refVec           Vector of reference vertex to be summed
// dMatch           Vector of reference matches in pointVec
// MatchCount       Number of matches in dMatch
// SegSzX           The number of points in this thread-segment typically 32 can be smaller
// WorkVec          Work Vector contains partial sums per thread
// workSzX          The x-dimension of WorkVec 
// dResVec          The Mean Vertex for pointVec, pointSum refVec and refSum
// dAtomVec[4]      A diagnostic check of thread visits to the final part (should be only 1)
//
// Each 1-dimensional thread is summing a partial segment of pointVec and refvec filtered on dMatch
// The partial results are stored in WorkVec[cellx..]
// atomicIdx() is used to count thread execution and only when all 
// threads have completed the first part, the final thread to execute
// sums the partials and computes the mean vertex. 
//
// This code is considerably simpler than the programming guide example
// using __threadfence() etc.
//
// However a key simplification is that only 1 thread block executes for the
// entire kernel operation, this is fine and within the limits of the GPU
// threads/block (1024) but care is needed if this situation changes
//
    float PointX, PointY, PointZ;
    float RefX, RefY, RefZ;
    float Sum;
    int i;

    const int cellx = threadIdx.x + blockIdx.x * blockDim.x;

    const int startv = cellx*SegSzX;
    const int startw = cellx*workSzX;

// clear the WorkVec space for this thread
    for(i=0;i<workSzX;i++){
        WorkVec[startw+i]=0.0;
    }
//
// sum all vectors in this thread call store result in WorkVec[cellx]...
//
    for( i=0;i<SegSzX;i++){
        const int in2r = startv + i;
        if( in2r >= refCount ) break;             // the last thread segment may exceed refCount so stop

        const int in2v = dMatch[in2r];
        if(in2v < 0) {
            continue;  // if we do not have a valid companion exclude the points
        }

        WorkVec[startw]   += 1.0;                                   // checkSum should == MatchCount
        WorkVec[startw+1] += refVec[in2r            ];              // X
        WorkVec[startw+2] += refVec[in2r +   pntSz.x];              // Y
        WorkVec[startw+3] += refVec[in2r + 2*pntSz.x];              // Z
        WorkVec[startw+4] += pointVec[in2v            ];    // X
        WorkVec[startw+5] += pointVec[in2v +   pntSz.x];    // Y
        WorkVec[startw+6] += pointVec[in2v + 2*pntSz.x];    // Z
    }
//
// atomicInc will index for every thread that gets to this point
// so countValue == (blockDim.x - 1) will happen only once and hence
// compute_meanDone==true will happen only once and the final 
// code will then be excuted
//
    unsigned int countValue = atomicInc(&countMean, blockDim.x);
    compute_meanDone = (countValue == (blockDim.x - 1));
//
// only the last thread to execute (and it does not matter which)
// should do this part
//
    if(compute_meanDone){

        dAtomVec[4] += 1;   // checks that only 1 thread did this

        PointX=0.0;
        PointY=0.0;
        PointZ=0.0;
        RefX=0.0;
        RefY=0.0;
        RefZ=0.0;
        Sum = 0.0;

        for(int i=0;i<countMean;i++){
            Sum += WorkVec[i*workSzX];

            RefX += WorkVec[i*workSzX+1];
            RefY += WorkVec[i*workSzX+2];
            RefZ += WorkVec[i*workSzX+3];

            PointX += WorkVec[i*workSzX+4];
            PointY += WorkVec[i*workSzX+5];
            PointZ += WorkVec[i*workSzX+6];
        }

        Sum = (float)_NINT(Sum);
        dResVec[0] = RefX/Sum;
        dResVec[1] = RefY/Sum;
        dResVec[2] = RefZ/Sum;

        dResVec[3] = PointX/Sum;
        dResVec[4] = PointY/Sum;
        dResVec[5] = PointZ/Sum;

        dResVec[6] = Sum;
//
// important that these two are reset to ensure
// the kernel can be reused without false triggering 
        compute_meanDone=false;
        countMean=0;
    }
}

int Compute_Mean_2_Launch(dim3 grid, dim3 TPB, float* pointVec, int2 pntSz, int pointCount, float* refVec, int refCount, 
                            int* dMatch, int MatchCount, int SegSzX, float* WorkVec, int workSzX, float*dResVec, int* dAtomVec)
{
    compute_Mean_2<<<grid, TPB>>>(pointVec, pntSz, pointCount, refVec, refCount, dMatch, MatchCount, SegSzX, WorkVec, workSzX, dResVec, dAtomVec);
    cudaDeviceSynchronize();
    cudaError_t CudaErr = cudaGetLastError();
    if(CudaErr!=cudaSuccess){
        const char* pError = cudaGetErrorString(CudaErr);
        printf("ComputeMean_Launch %s\n",pError);
        return -1;
    }
    return 0;
}
//
//-------------------------------------------------------------------------------------------------
//
__global__ void rotate_ref_points(cudaTextureObject_t ref_tex, int2 pntSz, int refCount, GBHMat dTCG, float* rotVec)
{
//
// Transforms all points in the reference ref_tex by dTCG and returns the result in rotVec
// The code is extended to include a surface normal if it is specified (based on the test pntSz.y>4)
//
// Arguments:
//  ref_tex     texture memory of vertices stored in column order
//  refCount    number of points in pointVec and rotVec
//  dTCG        delta Transformation to be applied to pointVec
//              note that this is in DH format R|t so the offset t
//              will also be applied
//  rotVec      The rotated and translated vector set
//
//
    const int refIdx = threadIdx.x + blockIdx.x * blockDim.x;

// Do nothing if we are out of bounds
    if (refIdx >= refCount) return;

// Input vertex

    const float Xr = tex2D<float>(ref_tex, (float)refIdx, (float)0);
    const float Yr = tex2D<float>(ref_tex, (float)refIdx, (float)1);
    const float Zr = tex2D<float>(ref_tex, (float)refIdx, (float)2);
    const float Part = tex2D<float>(ref_tex, (float)refIdx, (float)3);
    const float3 Vr = make_float3(Xr, Yr, Zr);

// transform and translate vertex
    const float3 Vo = dTCG.InvR(Vr);

    rotVec[refIdx]             = Vo.x;
    rotVec[refIdx +   pntSz.x] = Vo.y;
    rotVec[refIdx + 2*pntSz.x] = Vo.z;
    rotVec[refIdx + 3*pntSz.x] = Part;
//
// If pntSz.y > 4 then we have vector augmented with a surface normal
// rotate this with the vertex but no translation
// 
    if(pntSz.y>4){
        const float nxr = tex2D<float>(ref_tex, (float)refIdx, (float)3);
        const float nyr = tex2D<float>(ref_tex, (float)refIdx, (float)4);
        const float nzr = tex2D<float>(ref_tex, (float)refIdx, (float)5);

        const float3 Nr = make_float3(nxr, nyr, nzr);
        const float3 No = dTCG.InvRotate(Nr);

        rotVec[refIdx + 3*pntSz.x] = No.x;
        rotVec[refIdx + 4*pntSz.x] = No.y;
        rotVec[refIdx + 5*pntSz.x] = No.z;

//###        rotVec[refIdx + 6*pntSz.x] = tex2D<float>(ref_tex, (float)refIdx, (float)6);
    } 
}

int rotate_ref_points_Launch(dim3 grid, dim3 TBP, cudaTextureObject_t ref_tex, int2 pntSz, int refCount, GBHMat dTCG, float* rotVec)
{

    rotate_ref_points<<<grid, TBP>>>(ref_tex, pntSz, refCount, dTCG, rotVec);
    cudaDeviceSynchronize();
    cudaError_t CudaErr = cudaGetLastError();
    if(CudaErr!=cudaSuccess){
        const char* pError = cudaGetErrorString(CudaErr);
        printf("rotate_ref_points_Launch %s\n",pError);
        return -1;
    }
    return 0;
}
//
//------------------------------------------------------------------------------------------------------
//
__global__ void rotate_points(float* pointVec, int2 pntSz, int pointCount, GBHMat dTCG, float* rotVec)
{
//
// Transforms all vertex points in pointVec by dTCG and returns the result in rotVec
// The code is extended to include a surface normal if it is specified (based on the test pntSz.y>4)
//
// Arguments:
//  pointVec    Vector of vertices stored in column order
//  pntSz       Size of pointVec 
//  pointCount  number of points in pointVec and rotVec
//  dTCG        delta Transformation to be applied to pointVec
//              note that this is in DH format R|t so the offset t
//              will also be applied
//  rotVec      The rotated and translated vector set
//
//
    const int pntIdx = threadIdx.x + blockIdx.x * blockDim.x;

// Do nothing if we are out of bounds
//
    if (pntIdx >= pointCount) return;

// Input vertex
    const float3 Vp = make_float3(pointVec[pntIdx], pointVec[pntIdx+pntSz.x], pointVec[pntIdx+2*pntSz.x]);

// transform
    const float3 Vo = dTCG*Vp; 

    rotVec[pntIdx]             = Vo.x;
    rotVec[pntIdx +   pntSz.x] = Vo.y;
    rotVec[pntIdx + 2*pntSz.x] = Vo.z;
    rotVec[pntIdx + 3*pntSz.x] = pointVec[pntIdx+3*pntSz.x];
//
// If the vertex is augmented with a surface normal rotate this but no translation
//
    if(pntSz.y>4){
        const float3 Np = make_float3(pointVec[pntIdx+3*pntSz.x], pointVec[pntIdx+4*pntSz.x], pointVec[pntIdx+5*pntSz.x]);
        const float3 No = dTCG.Rotate(Np);
        rotVec[pntIdx + 3*pntSz.x] = No.x;
        rotVec[pntIdx + 4*pntSz.x] = No.y;
        rotVec[pntIdx + 5*pntSz.x] = No.z;
//###        rotVec[pntIdx + 6*pntSz.x] = pointVec[pntIdx+6*pntSz.x];
    }

}
int rotate_points_Launch(dim3 grid, dim3 TBP, float* pointVec, int2 pntSz, int pointCount, GBHMat dTCG, float* rotVec)
{
    rotate_points<<<grid, TBP>>>(pointVec, pntSz, pointCount, dTCG, rotVec);
    cudaDeviceSynchronize();
    cudaError_t CudaErr = cudaGetLastError();
    if(CudaErr!=cudaSuccess){
        const char* pError = cudaGetErrorString(CudaErr);
        printf("rotate_points_Launch %s\n",pError);
        return -1;
    }
    return 0;
}
//
//-------------------------------------------------------------------------------------------------
//
__global__ void inv_rotate_points(float* pointVec, int2 pntSz, int pointCount, GBHMat dTCG, float* rotVec)
{
//
// Inverse Transforms all vertex points in pointVec by dTCG and returns the result in rotVec
// The code is extended to include a surface normal if it is specified (based on the test pntSz.y>4)
//
// Arguments:
//  pointVec    Vector of vertices stored in column order
//  pntSz       Size of pointVec 
//  pointCount  number of points in pointVec and rotVec
//  dTCG        delta Transformation to be applied to pointVec
//              note that this is in DH format R|t so the offset t
//              will also be applied to the vertex (not the normal)
//  rotVec      The rotated and translated vector set
//
//
    const int pntIdx = threadIdx.x + blockIdx.x * blockDim.x;

// Do nothing if we are out of bounds
//
    if ( pntIdx >= pointCount) return;

// Input vertex
    const float3 Vp = make_float3(pointVec[pntIdx], pointVec[pntIdx+pntSz.x], pointVec[pntIdx+2*pntSz.x]);

// transform
    const float3 Vo = dTCG.InvR(Vp);

    rotVec[pntIdx]             = Vo.x;
    rotVec[pntIdx +   pntSz.x] = Vo.y;
    rotVec[pntIdx + 2*pntSz.x] = Vo.z;
    rotVec[pntIdx + 3*pntSz.x] = pointVec[pntIdx+3*pntSz.x];
//
// If the vertex is augmented with a surface normal rotate this but no translation
//
    if(pntSz.y>4){
        const float3 Np = make_float3(pointVec[pntIdx+3*pntSz.x], pointVec[pntIdx+4*pntSz.x], pointVec[pntIdx+5*pntSz.x]);
        const float3 No = dTCG.InvRotate(Np);
        rotVec[pntIdx + 3*pntSz.x] = No.x;
        rotVec[pntIdx + 4*pntSz.x] = No.y;
        rotVec[pntIdx + 5*pntSz.x] = No.z;
    }
}

int inv_rotate_points_Launch(dim3 grid, dim3 TBP, float* pointVec, int2 pntSz, int pointCount, GBHMat dTCG, float* rotVec)
{
    inv_rotate_points<<<grid, TBP >>>(pointVec, pntSz, pointCount, dTCG, rotVec);
    cudaDeviceSynchronize();
    cudaError_t CudaErr = cudaGetLastError();
    if(CudaErr!=cudaSuccess){
        const char* pError = cudaGetErrorString(CudaErr);
        printf("inv_rotate_points_Launch %s\n",pError);
        return -1;
    }
    return 0;


}

//
//-------------------------------------------------------------------------------------------------
//
__device__ unsigned int coveCount = 0;
__shared__ bool compute_coveDone;


__global__ void compute_Covariance_2(float* pointVec, int2 pntSz, int pointCount, int SegSz, float* refVec, int refCount, int* refMatch, int MatchCount, float* WorkVec, int workSzX, float3 pointMean, float3 refMean, GBHMat Cove, int* dAtomVec)
{
//
// This version 2 uses refVec in place of ref_tex
//
// Computes the cross-covariance matrix 'Cove' for a point-to-point
// ICP registration process.
//
// Arguments:
//  pointVec     Latest vertex data
//  pntSz        dimensions of pointVec
//  pointCount   Number of vertices in pointVec
//  SegSz        The number of vertices processed by this thread
//  refVec       reference vertex data
//  refCount     points in reference vector 
//  refMatch     matching index such that: pointVec[pntIdx] =~ refVec[refIdx]]
//               gives the closest match
//  MatchCount   Total number of matches this pass
//  WorkVec      vector for addition of partial sums
//  workSzX      Workspace needed per thread (28)
//  pointMean    Mean Vertex for pointVec
//  refMean      Mean Vertex set for reference dataset
//  CoveC        (3,3) cross-covariance matrix between pointVec and refVec data sets
//  dAtomVec[5]  A diagnostic checks the number of visits to the final part
//
// This kernel assumes that there is only 1 cuda block of 1-dimensional 
// threads each of which is processing a segment of the vertex array pointVec
// and stores the intermediate partial results in WorkVec[cellx..]
//
// Note that although the vertex can be augmented by a surface normal (indicated by pntSz.y>4)
// This normal plays no part in the covariance matrix hence this code is not affected.
// There may be a potential second solution here, can the normal be used?
//
// When all threads have completed indicated by: compute_coveDone = (countValue == (blockDim.x - 1))
// the final thread sums all the partials and computes the (3,3) covariance array
//
    const int cellx = threadIdx.x + blockIdx.x * blockDim.x;    // indexes through Vec and Ref

    int i,j,k, idxC;

    int startv = cellx*SegSz;
    int startw = cellx*workSzX;

// clear the WorkVec space for this thread
    for(i=0;i<workSzX;i++){
        WorkVec[startw+i]=0.0;
    }
//
// sum all Covariance matrix coeffs in this thread call, store result in WorkVec[cellx..]...
// WorkVec is laid out in rows of the Covariance matrix it must be initialised
// before this kernel can be called
//
  
    for( i=0;i<SegSz;i++){
        const int in2r = startv + i;
        if( in2r >= refCount ) break; // the last thread segment may exceed refVec array bound so we stop here

        const int in2v = refMatch[in2r];
        if(in2v < 0) {
            continue;  // if we do not have a valid companion exclude the point
        }

        const float Xp = pointVec[in2v            ];
        const float Yp = pointVec[in2v +   pntSz.x];
        const float Zp = pointVec[in2v + 2*pntSz.x];

        const float Xpb = Xp - pointMean.x;
        const float Ypb = Yp - pointMean.y;
        const float Zpb = Zp - pointMean.z;

        const float Xr = refVec[in2r            ];
        const float Yr = refVec[in2r +   pntSz.x];
        const float Zr = refVec[in2r + 2*pntSz.x];

        const float Xrb = Xr - refMean.x;
        const float Yrb = Yr - refMean.y;
        const float Zrb = Zr - refMean.z;

// Cross-covariance matrix stored in row-dominant order in WorkVec
        WorkVec[startw  ] += Xpb*Xrb;  // (Xp-muxp)*(Xr-muxr)
        WorkVec[startw+1] += Xpb*Yrb;  // Xp*Yr etc.
        WorkVec[startw+2] += Xpb*Zrb;  // Xp*Zr
 
        WorkVec[startw+3] += Ypb*Xrb;  // Yp*Xr
        WorkVec[startw+4] += Ypb*Yrb;  // Yp*Yr
        WorkVec[startw+5] += Ypb*Zrb;  // Yp*Zr
    
        WorkVec[startw+6] += Zpb*Xrb;  // Zp*Xr
        WorkVec[startw+7] += Zpb*Yrb;  // Zp*Yr
        WorkVec[startw+8] += Zpb*Zrb;  // Zp*Zr
    }
//
// atomicInc will index for every thread that gets to this point
// so coveCount == (blockDim.x - 1) only once and hence
// compute_coveDone==true will happen only once and the final 
// code can then be excuted
//
    unsigned int countValue = atomicInc(&coveCount, blockDim.x);
    compute_coveDone = (countValue == (blockDim.x - 1));
//
// only the last thread to execute (and it does not matter which)
// should do this part
//
    if(compute_coveDone){
// compute the Covariance matrix from the partials
// confusingly perhaps there is a switch from row-dominant in WorkVec to column-dominant in Cove
//
        dAtomVec[5] +=1;    // check that it visits only once
        float temp;
        for(i=0;i<3;i++){
            for(j=0;j<3;j++){
                Cove.Set(i,j,0.0);
                for(k=0;k<coveCount;k++){
                    idxC = k*workSzX;
                    temp = Cove.Get(i,j) + WorkVec[idxC + i + j*3 ];
                    Cove.Set(i,j,temp);
                }
                temp = Cove.Get(i,j)/(MatchCount-1);
                Cove.Set(i,j,temp);
            }
        }
        coveCount=0;
        compute_coveDone=false;
    }
}

int compute_Covariance_2_Launch(dim3 grid, dim3 TBP, float* pointVec, int2 pntSz, int pointCount, int SegSz, float* refVec, 
                                    int refCount, int* refMatch, int MatchCount, float* WorkVec, int workSzX, float3 pointMean,
                                        float3 refMean, GBHMat Cove, int* dAtomVec)
{
    compute_Covariance_2<<< grid, TBP>>>(pointVec, pntSz, pointCount, SegSz, refVec, refCount, refMatch, MatchCount, WorkVec, workSzX, pointMean, refMean, Cove, dAtomVec);
    cudaDeviceSynchronize();
    cudaError_t CudaErr = cudaGetLastError();
    if(CudaErr!=cudaSuccess){
        const char* pError = cudaGetErrorString(CudaErr);
        printf("compute_Covariance_2_Launch %s\n",pError);
        return -1;
    }
    return 0;

}
//
//-------------------------------------------------------------------------------------------------
//
__device__ unsigned int errorCount = 0;
__shared__ bool compute_ErrorDone;
//
//---------------------------------------------------------------------------------------------------------------------------
//
__global__ void compute_P2P_Error_2(float* pointVec, int2 pntSz, int pointCount, int SegSz, float* refVec, int refCount, int* dMatch, int MatchCount, float* WorkVec, int workSzX, float3 pointMean, float3 refMean, int* dAtomVec, float* dResVec)
{
//
// This version 2 uses refVec rather than ref_tex
// Computes the P2P Error for the ICP registration process
// points in pointVec and the matched point in refVec
//
// There may be cases where there is no match (refMatch[idx]<0) 
// in which case the point is excluded from further processing.
//
// Arguments:
//  pointVec     Vector of latest vertex data
//  pntSz        dimensions of pointVec
//  pointCount   Number of vertices in pointVec
//  SegSz        The number of vertices processed by this thread
//  refVec       reference vertex data
//  refCount     points in reference vector 
//  dMatch       index vector of pointVec matches with refVec dataset
//  MatchCount   The number of matches by which ICP_Error is normalised
//  WorkVec      vector for addition of partial sums
//  workSzX      For this case 3 element per cellx needed
//  pointMean    Mean Vertex for pointVec
//  refMean      Mean Vertex set for reference dataset
//  dAtomVec[6]  A diagnostic, checks the number of visits to the final part
//  dResVec[5]   The number of segment summations for this frame
//  dResVec[6]   The ICP error = L2 Norm over distance between corresponding 
//               points in pointVec and refVec normalised by MatchCount
//  dResVec[7]   ICP_Xr the L2 norm of Xr normalised by MatchCount
//  dResVec[8]   ICP_Xp the L2 norm of Xp normalised by MatchCount
//
    const int cellx = threadIdx.x + blockIdx.x * blockDim.x;    // indexes through Vec and Ref

    int i,k;

    int startv = cellx*SegSz;
    int startw = cellx*workSzX;

// clear the WorkVec space for this thread
    for(i=0;i<workSzX;i++){
        WorkVec[startw+i]=0.0;
    }
//
// sum all Covariance matrix coeffs in this thread call, store result in WorkVec[cellx..]...
// WorkVec is laid out in rows of the Covariance matrix it must be initialised
// before this kernel can be called
// This traps the obvious fail of zero matchcount (it can happen)
// It needs to be dealt with at the calling level 
    if(MatchCount<=0){
        dResVec[5] = -1;
        return;
    }

    for( i=0;i<SegSz;i++){
        const int in2r = startv + i;
        if( in2r >= refCount ) break; // the last thread segment may exceed refCount so we stop here
 
        const int in2v = dMatch[in2r];
        if(in2v < 0) {
            continue;  // if we do not have a valid companion exclude the point
        }
        const float Xp = pointVec[in2v            ];
        const float Yp = pointVec[in2v +   pntSz.x];
        const float Zp = pointVec[in2v + 2*pntSz.x];

        const float Xr = refVec[in2r            ];
        const float Yr = refVec[in2r +   pntSz.x];
        const float Zr = refVec[in2r + 2*pntSz.x];

        WorkVec[startw]   += norm3d((Xp-Xr),(Yp-Yr),(Zp-Zr));
        WorkVec[startw+1] += norm3d(Xr,Yr,Zr);
        WorkVec[startw+2] += norm3d(Xp,Yp,Zp);
    }
//
// atomicInc will index for every thread that gets to this point
// so coveCount == (blockDim.x - 1) only once and hence
// compute_coveDone==true will happen only once and the final 
// code can then be excuted by the last thread to complete
// this only works for 1 dimensional grids
//
    unsigned int countValue = atomicInc(&errorCount, blockDim.x);
    compute_ErrorDone = (countValue == (blockDim.x - 1));
//
// only the last thread to execute (and it does not matter which)
// should do this part
//
    if(compute_ErrorDone){
        dAtomVec[6] +=1;    // check that it visits only once
// compute ICP_Error
        float ICP_Error=0.0;
        float ICP_Xr = 0.0;
        float ICP_Xp = 0.0;
        for(k=0;k<errorCount;k++){
            ICP_Error += WorkVec[k*workSzX];
            ICP_Xr += WorkVec[k*workSzX+1];
            ICP_Xp += WorkVec[k*workSzX+2];
        }
// return the error in WorkVec
        dResVec[5] = float(errorCount);
        dResVec[6] = ICP_Error/MatchCount;
        dResVec[7] = ICP_Xr/MatchCount;
        dResVec[8] = ICP_Xp/MatchCount;

        errorCount=0;
        compute_ErrorDone=false;
    }
}

int compute_P2P_Error_2_Launch(dim3 grid, dim3 TBP, float* pointVec, int2 pntSz, int pointCount, int SegSz, float* refVec, int refCount, int* dMatch, int MatchCount, float* WorkVec, int workSzX, float3 pointMean, float3 refMean, int* dAtomVec, float* dResVec)
{
    compute_P2P_Error_2<<< grid, TBP>>>(pointVec, pntSz, pointCount, SegSz, refVec, refCount, dMatch, MatchCount, WorkVec, workSzX, pointMean, refMean, dAtomVec, dResVec);

    cudaDeviceSynchronize();
    cudaError_t CudaErr = cudaGetLastError();
    if(CudaErr!=cudaSuccess){
        const char* pError = cudaGetErrorString(CudaErr);
        printf("compute_P2P_Error_2_Launch %s\n",pError);
        return -1;
    }
    return 0;
}
//
//-------------------------------------------------------------------------------------------------
//
__device__ unsigned int P2L_errorCount = 0;
__shared__ bool compute_P2L_ErrorDone;

__global__ void compute_P2L_Error(float* pointVec, int2 pntSz, int pointCount, int SegSz, float* refVec, int refCount, int* dMatch, int MatchCount, float* WorkVec, int workSzX, int* dAtomVec, float* dResVec)
{
//
// Computes the P2L Error for the ICP registration process.
// 
// This version now computes the total error between pointVec and refVec
// There may be cases where there is no match (refMatch[idx]<0) in which case the point is excluded from 
// further processing.
//
// Arguments:
//  pointVec     Vector of latest processed camera vertex dataset
//  pntSz        dimensions of pointVec
//  pointCount   Number of vertices in pointVec
//  SegSz        The number of vertices processed by this thread
//  refVec       reference dataset stored in same format as pointVec
//  refCount     points in reference vector 
//  dMatch       index vector into pointVec for the closest match with refVec[refIdx]
//  MatchCount   The number of matches by which ICP_Error is normalised
//  WorkVec      vector for addition of partial sums
//  workSzX      For this case 3 element per cellx needed
//  dAtomVec[6]  A diagnostic, checks the number of visits to the final part
//  dResVec[5]   The number of segment summations for this frame
//  dResVec[6]   The ICP error = L2 Norm over distance between corresponding 
//               points in pointVec and ref_tex
//
    const int cellx = threadIdx.x + blockIdx.x * blockDim.x;    // indexes through Vec and Ref

    int i,k;
    int startv = cellx*SegSz;
    int startw = cellx*workSzX;
//
// sum all Covariance matrix coeffs in this thread call, store result in WorkVec[cellx..]...
// WorkVec is laid out in rows of the Covariance matrix it must be initialised
// before this kernel can be called
// This traps the obvious fail of zero matchcount (it can happen)
// It needs to be dealt with at the calling level 
    if(MatchCount<=0){
        dAtomVec[5] = -1;
        return;
    }

    WorkVec[startw]  =0.0;
    WorkVec[startw+1]=0.0;
    WorkVec[startw+2]=0.0;
    WorkVec[startw+3]=0.0;
    WorkVec[startw+4]=0.0;  // count

    for( i=0;i<SegSz;i++){
        const int refIdx = startv + i;
        if( refIdx >= refCount ) break; // the last thread segment may exceed refCount so we stop here

        const int pntIdx = dMatch[refIdx];
        if(pntIdx < 0) {
            continue;  // if we do not have a valid comparison exclude the point
        }
        const float Xp = pointVec[pntIdx            ];
        const float Yp = pointVec[pntIdx +   pntSz.x];
        const float Zp = pointVec[pntIdx + 2*pntSz.x];

        const float Xr = refVec[refIdx            ];
        const float Yr = refVec[refIdx +   pntSz.x];
        const float Zr = refVec[refIdx + 2*pntSz.x];

        const float dx = Xp-Xr;
        const float dy = Yp-Yr;
        const float dz = Zp-Zr;

        WorkVec[startw]   += norm3d(dx,dy,dz);
        WorkVec[startw+1] += dx;
        WorkVec[startw+2] += dy;
        WorkVec[startw+3] += dz;
        WorkVec[startw+4] += 1.0;
    }
//
// atomicInc will index for every thread that gets to this point
// so coveCount == (blockDim.x - 1) only once and hence
// compute_coveDone==true will happen only once and the final 
// code can then be excuted
//
    unsigned int countValue = atomicInc(&P2L_errorCount, blockDim.x);
    compute_P2L_ErrorDone = (countValue == (blockDim.x - 1));
//
// only the last thread to execute (and it does not matter which)
// should do this part
//
    if(compute_P2L_ErrorDone){
        dAtomVec[6] +=1;    // check that it visits only once
// compute ICP_Error
        float ICP_Error=0.0;
        float ICP_X = 0.0;
        float ICP_Y = 0.0;
        float ICP_Z = 0.0;
        float count = 0.0;
        for(k=0;k<P2L_errorCount;k++){
            ICP_Error += WorkVec[k*workSzX];
            ICP_X += WorkVec[k*workSzX+1];
            ICP_Y += WorkVec[k*workSzX+2];
            ICP_Z += WorkVec[k*workSzX+3];
            count += WorkVec[k*workSzX+4];
        }
// return the error in WorkVec
        count = (float)_NINT(count);
        dResVec[0] = ICP_Error/MatchCount;
        dResVec[1] = ICP_X/MatchCount;
        dResVec[2] = ICP_Y/MatchCount;
        dResVec[3] = ICP_Z/MatchCount;
        dResVec[4] = count;
        P2L_errorCount=0;
        compute_P2L_ErrorDone=false;
    }
}

int P2L_Error_Launch(dim3 grid, dim3 TBP, float* pointVec, int2 pntSz, int pointCount, int SegSz, float* refVec, int refCount, int* dMatch, int MatchCount, float* WorkVec, int workSzX, int* dAtomVec, float* dResVec)
{

    compute_P2L_Error<<< grid, TBP >>>(pointVec, pntSz, pointCount, SegSz, refVec, refCount, dMatch, MatchCount, WorkVec, workSzX, dAtomVec, dResVec);
    cudaDeviceSynchronize();
    cudaError_t CudaErr = cudaGetLastError();
    if(CudaErr!=cudaSuccess){
        const char* pError = cudaGetErrorString(CudaErr);
        printf("P2L_Error_Launch %s\n",pError);
        return -1;
    }
    return 0;
}
//
//-------------------------------------------------------------------------------------------------
//
__device__ unsigned int P2L_updateCount = 0;
__shared__ bool compute_P2L_updateDone;

__global__ void compute_P2L_Update(float* pointVec, int2 pntSz, int pointCount, int SegSz, float* refVec, int refCount, int* dMatch, int MatchCount, float* dWorkVec, int WorkStep, float* dResVec, int* dAtomVec)
{
//
// Takes pointVec, refVec and dMatch as input and computes matrix A[6,6] and vector B[6]
// returned in dResVec
// They can then be used to compute the vector Tau giving rotation and translation between frames.
//
// The point-to-plane algorithm used is defined in Pomerleau, "A Review of Point Cloud Registration
// Algorithms for Mobile Robotics". using similar variable names.
// Note my rotation matrix is the transpose of the one used in Pomerleau, this is
// the reason for some inverse rotations. The convention I am using is apparently not
// the same as that used in the report though all the transformation are consistent.
// 
//  pointVec        New Vertex and Normal data for each cell processed
//  pntSz           Size limits of pointVec and refVec
//  pointCount      Number of active points in pointVec
//  SegSz           Number of vertex points processed in this thread
//  refVec          Reference vector same format as pointVec
//  refCount        Number of active points in refVec != pointCount
//  dMatch          Contains indices that maps points in refVec to pointVec
//  MatchCount      Number of matches in refVec to pointVec
//  dWorkVec        Workspace vector used to store internals for each thread
//  WorkStep        Storage requirement in dWorkVec per thead step
//  dResVec         Output AMat and BVec stored in sequence
//  dAtomVec        Used for atomics and debug
//
// internals
//
    int i, j, k, Index;

    float *ck, *pk, *dk;
    float *qk, *nk;
    float val;
// these are stored in the dWorkVec with one entry per thread
    float *Bsub, *Csub, *BVec;

    const int cellx = threadIdx.x + blockIdx.x * blockDim.x;    // indexes through Vec and Ref

    int startv = cellx*SegSz;
    int startw = cellx*WorkStep;
//
    if(MatchCount<=0){
        dAtomVec[0] = -1;
        return;
    }
//
// start index for this thread into dWorkVec
//
    Index=startw;
    GMat Asub(GetData(dWorkVec,Index,36),6,6);
    Bsub = GetData(dWorkVec,Index,6);
    Csub = GetData(dWorkVec,Index,6);
    pk = GetData(dWorkVec,Index,3);
    qk = GetData(dWorkVec,Index,3);
    ck = GetData(dWorkVec,Index,3);
    dk = GetData(dWorkVec,Index,3);
    nk = GetData(dWorkVec,Index,3);
//
// zero the partial sums for this thread
//
    Asub.Zero();
    for(i=0;i<6;i++){
        Bsub[i]=0.0;
    }
//
// for each vertex-pair in this thread segment
// compute the partial sums Asub and Bsub
//
    for( i=0;i<SegSz;i++){

        const int refIdx = startv + i;
        if( refIdx >= refCount ) break; // the last thread segment may exceed pointCount so we stop here

        const int pntIdx = dMatch[refIdx];
        if(pntIdx < 0) {
            continue;  // if we do not have a valid comparison exclude the point
        }
        pk[0] = pointVec[pntIdx            ];
        pk[1] = pointVec[pntIdx +   pntSz.x];
        pk[2] = pointVec[pntIdx + 2*pntSz.x];

        qk[0] = refVec[refIdx            ];
        qk[1] = refVec[refIdx +   pntSz.x];
        qk[2] = refVec[refIdx + 2*pntSz.x];

        nk[0] = refVec[refIdx + 3*pntSz.x];
        nk[1] = refVec[refIdx + 4*pntSz.x];
        nk[2] = refVec[refIdx + 5*pntSz.x];

// ck = pk x nk
        ck[0] = pk[1]*nk[2] - nk[1]*pk[2];
        ck[1] = nk[0]*pk[2] - pk[0]*nk[2];
        ck[2] = pk[0]*nk[1] - nk[0]*pk[1];

// dk = qk - pk
        dk[0] = qk[0] - pk[0];
        dk[1] = qk[1] - pk[1];
        dk[2] = qk[2] - pk[2];

// compute rhs
        Csub[0] = ck[0];
        Csub[1] = ck[1];
        Csub[2] = ck[2];
        Csub[3] = nk[0];
        Csub[4] = nk[1];
        Csub[5] = nk[2];

        val = dk[0]*nk[0] + dk[1]*nk[1] + dk[2]*nk[2];

        Bsub[0] += Csub[0]*val;
        Bsub[1] += Csub[1]*val;
        Bsub[2] += Csub[2]*val;
        Bsub[3] += Csub[3]*val;
        Bsub[4] += Csub[4]*val;
        Bsub[5] += Csub[5]*val;
//
// Compute and sum the covariance matrix for this thread
//
        for(j=0;j<6;j++){
            for(k=0;k<6;k++){
                Asub(j,k) += Csub[j]*Csub[k];
            }
        }
    }
//
// atomicInc will index for every thread that gets to this point
// so countValue == (blockDim.x - 1) only once and hence
// compute_P2L_Done = true will happen only once and the final 
// code can then be executed. Note this strategy will only work
// for simple kernel calls: <<< 1, nPoints>>> 
//
    unsigned int countValue = atomicInc(&P2L_updateCount, blockDim.x);
    compute_P2L_updateDone = (countValue == (blockDim.x - 1));
//
// only the last thread to execute (and it does not matter which)
// should do this part
//
    if(compute_P2L_updateDone){
        dAtomVec[0] +=1;
        dAtomVec[1] = P2L_updateCount;
// check that it visits only once

// Sum A and B matrices for all threads in this kernel
// the results are mapped back to dResVec
        Index=0;
        GMat AMat(GetData(dResVec,Index,36),6,6);
        BVec = GetData(dResVec,Index,6);
        AMat.Zero();
        for(i=0;i<6;i++) {
            BVec[i] = 0.0;
        }
        for(k=0;k<P2L_updateCount;k++){
            Index = k*WorkStep;
            GMat Asub(GetData(dWorkVec,Index,36),6,6);
            Bsub = GetData(dWorkVec,Index,6);

            for (i=0;i<6;i++){
                BVec[i] += Bsub[i];
                for(j=0;j<6;j++){
                    AMat(i,j) += Asub(i,j);
                }
            }
        }

        P2L_updateCount=0;
        compute_P2L_updateDone=false;
    }
}

int P2L_Update_Launch(dim3 grid, dim3 TBP, float* pointVec, int2 pntSz, int pointCount, int SegSz, float* refVec, int refCount, int* dMatch, int MatchCount, float* dWorkVec, int WorkStep, float* dResVec, int* dAtomVec)
{

    compute_P2L_Update<<<grid, TBP>>>(pointVec, pntSz, pointCount, SegSz, refVec, refCount, dMatch, MatchCount, dWorkVec, WorkStep, dResVec, dAtomVec);

    cudaDeviceSynchronize();
    cudaError_t CudaErr = cudaGetLastError();
    if(CudaErr!=cudaSuccess){
        const char* pError = cudaGetErrorString(CudaErr);
        printf("P2L_Update_Launch %s\n",pError);
        return -1;
    }
    return 0;
}

//
//-------------------------------------------------------------------------------------------------
//

__global__ 
void pca_pair_match(float* pointVec, int2 pntSz, int pointCount, int* ipVec, float* refVec, int refCount, 
                            float* dWorkMat, float DistMin, float CosMin, float FMThresh, 
                                int* dMatch, int* dAtomVec)
{
//
//
// This is a point-pair matching algorithm based on the Forstner-Moonen Metric  
// to establish a relationship between pointVec and refVec based on their proximity
// AND similarity of covariance matrices based on FMThresh.
//
// Computing the FM-metric is computationally expensive so it is only applied as a test
// on vectors that have already met the DistMin and CosMin constraints 
// 
// Arguments:
//  pointVec    set of vertices for point-pair matching for this frame 
//  pntSz       size of pointVec
//  pointCount  Number of vertices in pointVec
//  ipVec       vector of pointVec indices to refVec
//  refVec      Reference vertices from previous frame 
//  refCount    Number of vertices in refVec, in general refCount != pointCount
//  dWorkMat    Workspace containing Covariance matrices generated in PCA_Reduce
//
//  DistMin     The minimum L2 distance to be a qualifying point-pair match
//  CosMin      the minimum cos(theta) value i.e. cos(30) = 0.866 to qualify (typically 0.95 or greater)
//  FMThresh    Forstner-Moonen threshold below which a match is accepted
//
//  dMatch      Resulting Indices in RefVec of point-pair matches with pointVec
//              refIdx = refMatch[i] is the closest point in RefVec[] corresponding to pointVec[i,:]
//              if refMatch[i] < 0 the point is not matched and excluded from dMatch
//
//  dAtomVec[4] Count of non-negative matches, (MatchCount) and must be zeroed before launch 
//
// these locals could be placed in dWorkVec
    float Xp[6], Xr[6], Dx[6];
    int i;
//
    const int refIdx = threadIdx.x + blockIdx.x * blockDim.x;

// Do nothing if we are out of array bounds
    if (refIdx >= refCount) return;

    const int Points=6;
// the refVec point to compare in this thread

    for(i=0;i<Points;i++){
        Xr[i] = refVec[refIdx + i*pntSz.x];
    }


    float savMin = 1.0E+3;
    int idxMin = -1;
    dMatch[refIdx] = idxMin;
//
// test this point against all points in the reference dataset
//
    for (int pntIdx=0; pntIdx<pointCount; pntIdx++) {

        for(i=0;i<Points;i++){
            Xp[i] = pointVec[pntIdx + i*pntSz.x];
            Dx[i] = Xp[i] - Xr[i];
        }
  
        float TestD = norm3df( Dx[0], Dx[1], Dx[2] );  // the L2 norm of the vertex delta
//
// first part is an unconstrained point-pair matching 
// selects the closest match every time, this may 
// not necessarily be the closest FM match
//
        if(TestD<savMin){
            savMin=TestD;
            idxMin=pntIdx;            
        }
    }
//
// Test that the closest point found is less than DistMin and 
// within CosMin
//
    if(savMin <= DistMin){
        float Szr=0.0;
        float Szp=0.0;
        float Arp=0.0;
        for(i=0;i<Points;i++){
            Xp[i] = pointVec[idxMin + i*pntSz.x];
            Arp += Xr[i]*Xp[i];
            Szp += Xp[i]*Xp[i];
            Szr += Xr[i]*Xr[i];
        }
        Szr = sqrtf(Szr);
        Szp = sqrtf(Szp);
        const float cosRP = fabsf(Arp)/(Szr*Szp);

        if(cosRP > CosMin){
//
            float* fTest = &pointVec[idxMin + PVEC_ELEMENT*pntSz.x];
            int* prevIdx = &ipVec[idxMin];
//
// Test that this pair-match is closer than any previous match on this
// pointVec element
            if(savMin < fTest[0]){
                atomicExch(fTest,savMin);      // if it is save is in pointVec
                if(prevIdx[0] > -1){           // and if there was a previous match
                    dMatch[prevIdx[0]]=-1;     // break it
                    atomicAdd(&dAtomVec[4],1);  // track the number of breaks
                    atomicAdd(&dAtomVec[3],-1); // decrement the counter to remove the prevIdx
                }
                atomicAdd(&dAtomVec[3],1);   // increment of the number of pair-matches made
                atomicExch(prevIdx,refIdx);  // make this refIdx the new match in pointVec
            }else{
                idxMin = -idxMin;
            }
        } else {
            idxMin = -idxMin;
        }
    } else {
        idxMin = -idxMin;
    }
//
// perform the FM test only on points that meet the above pair-matching criteria
//
    if(idxMin>=0){
        int iTest = 0;
        float FM = ComputeFM(idxMin, refIdx, pointVec, refVec, pntSz, dWorkMat, iTest );
        if(FM>FMThresh){
            idxMin = -idxMin;
        } else{
            atomicAdd(&dAtomVec[5],1);   // keeps a count of the number of FM matches made
        }
    }
//
// an index < 0 indicates no valid match was found for this point
// this is a valid and common condition so we save it to prevent
// further inclusion of this point in the processing algorithm
//
    dMatch[refIdx] = idxMin;

}

int PCA_Match_Launch(dim3 grid, dim3 TPB, float* pointVec, int2 pntSz, int pointCount, int* ipVec,  float* refVec, 
                                int refCount, float* dWorkMat, float DistVal, float CosVal, float FMThresh, 
                                int* dMatch, int* dAtomVec)
{

    pca_pair_match<<< grid, TPB >>>(pointVec, pntSz, pointCount, ipVec, refVec, refCount, dWorkMat, DistVal, CosVal, FMThresh, dMatch, dAtomVec);
    cudaDeviceSynchronize();
    cudaError_t CudaErr = cudaGetLastError();
    if(CudaErr!=cudaSuccess){
        const char* pError = cudaGetErrorString(CudaErr);
        printf("PCA_Match_Launch %s\n",pError);
        return -1;
    }
    return 0;
}

//
//-------------------------------------------------------------------------------------------------
//
__global__ void pca_inv_rotate_points(float* pointVec, int2 pntSz, int pointCount, GBHMat dTCG, float* rotVec)
{
//
// Inverse Transforms all vertex points in pointVec by dTCG and returns the result in rotVec
// The code is extended to include a surface normal if it is specified (based on the test pntSz.y>4)
//
// Arguments:
//  pointVec    Vector of vertices stored in column order
//  pntSz       Size of pointVec 
//  pointCount  number of points in pointVec and rotVec
//  dTCG        delta Transformation to be applied to pointVec
//              note that this is in DH format R|t so the offset t
//              will also be applied to the vertex (not the normal)
//  rotVec      The rotated and translated vector set
//
//
    const int pntIdx = threadIdx.x + blockIdx.x * blockDim.x;

// Do nothing if we are out of bounds
//
    if ( pntIdx >= pointCount) return;

// Input vertex
    const float3 Vp = make_float3(pointVec[pntIdx], pointVec[pntIdx+pntSz.x], pointVec[pntIdx+2*pntSz.x]);

// transform
    const float3 Vo = dTCG.InvR(Vp);

    rotVec[pntIdx]             = Vo.x;
    rotVec[pntIdx +   pntSz.x] = Vo.y;
    rotVec[pntIdx + 2*pntSz.x] = Vo.z;
//
// If the vertex is augmented with a surface normal rotate this but no translation
//
    if(pntSz.y>4){
        const float3 Np = make_float3(pointVec[pntIdx+3*pntSz.x], pointVec[pntIdx+4*pntSz.x], pointVec[pntIdx+5*pntSz.x]);
        const float3 No = dTCG.InvRotate(Np);
        rotVec[pntIdx + 3*pntSz.x] = No.x;
        rotVec[pntIdx + 4*pntSz.x] = No.y;
        rotVec[pntIdx + 5*pntSz.x] = No.z;
    }
//
// copy/transfer any remaining data without rotation 
// Warning! this may require a re-evaluation of the covariance matrices for pointVec in dWorkMat
// 
    for(int i=6;i<pntSz.y;i++){
        rotVec[pntIdx + i*pntSz.x] = pointVec[pntIdx + i*pntSz.x];
    }
}



int PCA_inv_rotate_Launch(dim3 grid, dim3 TBP, float* pointVec, int2 pntSz, int pointCount, GBHMat dTCG, float* rotVec)
{

    pca_inv_rotate_points<<<grid, TBP>>>(pointVec, pntSz, pointCount, dTCG, rotVec);

    cudaDeviceSynchronize();
    cudaError_t CudaErr = cudaGetLastError();
    if(CudaErr!=cudaSuccess){
        const char* pError = cudaGetErrorString(CudaErr);
        printf("PCA_Match_Launch %s\n",pError);
        return -1;
    }
    return 0;
}

__device__ float ComputeFM(int PntIdx, int RefIdx, float* pointVec, float* refVec, int2 pntSz, float* dWorkMat, int& iTest )
{
//
// Computes the Forstner-Moonen (FM) metric as defined in: "A Metric for Covariance Matrices"
// This is a similarity metric between regions of an image FM=0 if they are the same
// FM > 0 if they are different with a proportionallity related to their difference
//
// For correct operation PointVec and dWorkMat should have been pre-processed
// by the PCA_Reduce and not further changed before here. 
//
// The reference CRef is recovered from RefVec, and should not be changed
// AM1Pnt and the other pointVec matrices are recovered from dWorkMat 
// at the index location stored in pointVec 
// CPnt and AM1Pnt should also not be changed
//
//  Arguments:
//  PntIdx          PointVec index for test matrices
//  RefIdx          RefVec index for corresponding cell
//  pointVec        Point "test" vector
//  refVec          Reference vector
//  pntSz           Size of pointVec and RefVec (for indexing)
//  dWorkMat        Work Vector containing matrices computed in PCA_Reduce
//                  for pointVec used here for furthe processing
//  iTest           Number of Jacobi iterates for this operation
//
    int i, j, k;
    float cref[9];
    float *unused;
    GMat CRef(cref,3,3);
//
// recover CRef from refVec

    int refStart = 9;
    for(i=0;i<3;i++){
        for(j=0;j<3;j++){
            CRef(i,j) = refVec[RefIdx + refStart*pntSz.x];
            refStart++;
        }
    }
//
// get the location of pointVec matrices in dWorkMat
    const int wrkSrc = (int)pointVec[PntIdx + WVEC_START*pntSz.x];
    float* pWorkSource = &dWorkMat[wrkSrc];
//
// matrices for pointVec don't overwrite AM
//
    int InIndex=0;
    GMat CPnt(GetData(pWorkSource,InIndex,9),3,3);
    GMat AM1Pnt(GetData(pWorkSource,InIndex,9),3,3);
// some workspace matrices are recycled here so PCA_Reduce versions will be lost
// not a problem so long as CPnt and AM1Pnt are not overwritten, some are also unused
    unused = GetData(pWorkSource,InIndex,3);
    unused = GetData(pWorkSource,InIndex,3);
    GMat VPnt(GetData(pWorkSource,InIndex,9),3,3);
    GMat WPnt(GetData(pWorkSource,InIndex,9),3,3);
    GMat RPnt(GetData(pWorkSource,InIndex,9),3,3);
    GMat SPnt(GetData(pWorkSource,InIndex,9),3,3);
//
// Compose SPnt = AM1Pnt.CRef.AM1Pnt
//
    for(i=0;i<3;i++){
        for(j=0;j<3;j++){
            VPnt(i,j) = 0.0;
            for(k=0;k<3;k++){
                VPnt(i,j) += CRef(i,k)*AM1Pnt(k,j);
            }
        }
    }
//
    for(i=0;i<3;i++){
        for(j=0;j<3;j++){
            SPnt(i,j) = 0.0;
            for(k=0;k<3;k++){
                SPnt(i,j) += AM1Pnt(i,k)*VPnt(k,j);
            }
        }
    }
//
// Perform Jacobi iteration on SPnt to get Eigen-values
// on lead diagonal
//
    iTest += Jacobi_GMat(SPnt, VPnt, WPnt, RPnt);
//
// The FM metric is then formed from the sum of log^2(Lambda(i))
//
    float FM = logf(SPnt(0,0))*logf(SPnt(0,0)) 
                    + logf(SPnt(1,1))*logf(SPnt(1,1))
                        + logf(SPnt(2,2))*logf(SPnt(2,2));
    return FM;
}
