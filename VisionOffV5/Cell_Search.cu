//
// Cell_Search.cu
//
// Uses a combination of Point-to-Plane ICP and cellular search using the Forstner-Moonen metric
// see header for details
//
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <helper_math.h>
#include "../Subs/constants.h"

#include "../Subs/FMat.h"
#include "../Subs/GMat.cuh"
#include "../Subs/device_funcs.cuh"

#include "VisionOffV5.h"
#include "kernel.h"

#include "ControlParams.h"
#include "ICP_Helpers.cuh"
#include "Cell_Search.cuh"

//
GBHMat TCB2_S;


int CellSearch_Launcher( float* d_Depth, int2 arrSz, int2 cellSz, int* dMap, int2 mapSz,
                            float* pointVec, int2 pntSz, int pointCount, int* ipVec, float* refVec, int& refCount, 
                                 float*rotVec, int* dMatch, int matchBytes, int& MatchCount, int ICPFrames,
                                    float* dWorkVec,  int CellWorkStep, float* dWorkMat, int wMatStep,  
                                        float DistMin, float CosMin, float FMThresh, int FrameNumber, 
                                            GBHMat TCG, float3& xgbf, GBHMat dTCB, 
                                                float* dResVec, int* dAtomVec, FILE* pDmpFile, FILE* pResFile)
{
//
// Performs a cell-based search between the reference frame and the test frame
//
// Arguments:
//  d_Depth      camera depth array in m
//  arrSz        Size of d_Depth
//  cellSz       Size of cells in depth array, 
//  dMap         Cell map array indicates which cells have valid data
//               if data exists dMap(cx,cy) will contain the index into pointVec
//               or -1 to indicate a dead cell
//  dRefMap      Reference copy of dMap (unused)
//  dCellMap     common subset within the pointVec and refVec (unused)
//  mapSz        dMap size
//
//  pointVec     Vector into which the vertex and normal for this cell is placed
//               It is extended to include the covariance matrix for FMTest
//  pntSz        Size of pointVec
//  pointCount   Number of points in pointVec
//
//  rotVec       rotated pointVec  = T^-1(pointVec)
//  refVec       Reference version of pointVec
//  refCount     Number of points in refVec
//
//  dMatch       The Index array of associations pointVec with refVec
//  matchBytes   Size of dMatch
//  MatchCount   Number of associations found
//
//  ICPFrames    Number of frames between Reference Vector (refVec) update
//
//  dWorkVec     Scratch pad used in various GPU kernels
//               Note the step size is currently SEG64 per thread
//
//  dWorkMat     Memory reserved to store/transfer GMat PCA matrix data
//              (computed in PCA_Reduce and passed here to ComputeFM)
//  wMatStep     Step size of dWorkMat per thread
//  dResVec      Vector used to return kernel results
//
//  DistMin      Threshold minimum for point-pair match
//  CosMin       Minimum cos(theta) for point-pair match
//  FMThresh     Threshold on allowable Forstner-Moonen covariance metric
//
//  FrameNumber  The number of this frame step
//
//  TGB          Camera to global transformation
//  xgbf         Initial global position of camera in view volume
//  dTCB         camera delta transformation between pointVec and refVec
//
//  dAtomVec     GPU Atomic variables
//  pDmpFile     Output diagnostic file
//  pResFile     Tracking Results file
//
//------------------------------------------------------------------------------------------
// In this implementation a lot is common with P2L_ICP
// It is kept as a separate code to enable partitioned development
//
    float3 dangi, dxgbi;
    float3 refMean, pointMean, rotMean;
//
    int LoopCount=0;
    FMat Decomp;
    FVec Tau;

    float ICP_Error1, ICP_Error2;
    float3 ICP_ErrVec1, ICP_ErrVec2;

    int Index=0, MatchStart=0, MatchBreaks=0, MatchFM=0;
//    int i,j,k, 
    int count=0;

    const int resBytes = RES_VEC_SIZE*sizeof(float);
    const int atomBytes = ATOM_VEC_SIZE*sizeof(int);
//
    Tau.ZeroInit(6);
    Decomp.ZeroInit(6,6);
    dxgbi = make_float3(0.0,0.0,0.0);
//
// grid and block dimensions are specified early as they are used multiple times
// At this level the code is dealing with linear vectors from PCA_reduce
// so 1-D grids and blocks make more sense as long as GPU_TBP_LIM is respected
//
    int SEGSZ = SEG32;
    int WorkSzX = divUp(pointCount, SEGSZ);
    while(WorkSzX>GPU_TBP_LIM){
        SEGSZ *= 2;
        WorkSzX = divUp(pointCount,SEGSZ);
    }

// used by: Compute_Mean_2_Launch, P2L_Error_Launch, P2L_Update_Launch
    const dim3 block0( WorkSzX, 1);     
    const dim3 grid0( 1 , 1 ); 

// used by: Reset_pointVec_element_Launch, PCA_Match_Launch
    const dim3 block1( SEGSZ, 1);
    const dim3 grid1( WorkSzX, 1 );

//>>>
    Reset_pointVec_element_Launch(grid1, block1,pointVec, pntSz, pointCount, ipVec);
    cudaMemset(dAtomVec, 0, atomBytes);
    PCA_Match_Launch(grid1, block1, pointVec, pntSz, pointCount, ipVec, refVec, refCount, dWorkMat, DistMin, CosMin, FMThresh, dMatch, dAtomVec);
    cudaMemcpy(hMatch,dMatch,matchBytes,cudaMemcpyDeviceToHost);
    cudaMemcpy(hAtomVec,dAtomVec,atomBytes,cudaMemcpyDeviceToHost);
//<<<

    MatchCount = hAtomVec[3];
    MatchBreaks = hAtomVec[4];
    MatchFM = hAtomVec[5];   // Number of points that passed FM Test
    MatchStart = MatchFM;

    printf("\nMatchStart, %d, MatchBreaks %d, MatchFM %d, refCount, %d, \n",MatchStart,MatchBreaks,MatchFM,refCount);
    fprintf(pDmpFile,"\nMatchStart, %d, MatchBreaks %d, MatchFM, %d, refCount, %d, \n",MatchStart,MatchBreaks,MatchFM,refCount);
    if(MatchStart < 0.02*pointCount){
        printf("\nWarning!!!, MatchStart, %d, < 0.02*pointCount, %d,\n",MatchStart,pointCount);
    }
//>>>
        cudaMemset(dResVec,0,resBytes);
        cudaMemset(dAtomVec,0,atomBytes);
    Compute_Mean_2_Launch( grid0, block0, pointVec, pntSz, pointCount, refVec, refCount, dMatch, MatchFM, SEGSZ, dWorkVec, 7, dResVec, dAtomVec);
        cudaMemcpy(hResVec,dResVec,resBytes,cudaMemcpyDeviceToHost);
//<<<
    refMean = make_float3(hResVec[0],hResVec[1],hResVec[2]);
    pointMean = make_float3(hResVec[3], hResVec[4], hResVec[5]);
    count = _NINT(hResVec[6]);
//###
    printf(" refX %.5f, refY %.5f, refZ %.5f, refCount %d,\n",refMean.x, refMean.y, refMean.z, refCount);
    printf(" pntX %.5f, pntY %.5f, pntZ %.5f, pntCount %d,\n",pointMean.x, pointMean.y, pointMean.z, pointCount);
//###    printf(" count %d, \n", count);

    fprintf(pDmpFile," refX %.5f, refY %.5f, refZ %.5f, refCount %d,",refMean.x, refMean.y, refMean.z, refCount);
    fprintf(pDmpFile," pntX %.5f, pntY %.5f, pntZ %.5f, pntCount %d,",pointMean.x, pointMean.y, pointMean.z, pointCount);

//### debug start
/*
        fprintf(pDmpFile," hMatch \n");
        for(i=0;i<100;i++){
            fprintf(pDmpFile," %d,",hMatch[i]);
        }
        fprintf(pDmpFile,"\n");
//
// This debug allows comparison of refVec with rotated pointVec
// note the order is critical 
//
        cudaMemcpy(hRefVec,refVec,pntBytes,cudaMemcpyDeviceToHost);
        fprintf(pDmpFile," refVec \n");     
        for(i=0;i<pntSz.y;i++){
            for(j=100;j<200;j++){
                    k = hMatch[j];
                    if(k>0){
                        const float val = hRefVec[j + i*pntSz.x];
                        fprintf(pDmpFile," %.5f,",val);
                    }else {
                        fprintf(pDmpFile," %.5f,",0.0);                    
                    }
            }
            fprintf(pDmpFile,"\n");
        }
        fprintf(pDmpFile,"\n");
*/
//### end of debug

        cudaMemset(dResVec,0,resBytes);
    P2L_Error_Launch(grid0, block0, pointVec, pntSz, pointCount, SEGSZ, refVec, refCount, dMatch, MatchFM, dWorkVec, 5, dAtomVec, dResVec);
        cudaMemcpy(hResVec,dResVec,resBytes,cudaMemcpyDeviceToHost);
//<<<
    ICP_Error1 = hResVec[0];
    ICP_ErrVec1 = make_float3(hResVec[1],hResVec[2],hResVec[3]);
    count = _NINT(hResVec[4]);

    printf(" Frame, %d, LoopCount, %d, MatchFM, %d, count, %d, ICP_Error1, %.5f, ",FrameNumber,LoopCount,MatchFM,count,ICP_Error1);
    fprintf(pDmpFile," Frame, %d, LoopCount, %d, MatchFM, %d, count, %d, ICP_Error1, %.5f, refMean, %.5f, %.5f, %.5f, ICP_ErrVec1, %0.5f, %0.5f, %0.5f,\n ",FrameNumber,LoopCount,MatchFM,count,ICP_Error1, refMean.x,refMean.y,refMean.z, ICP_ErrVec1.x,ICP_ErrVec1.y,ICP_ErrVec1.z);
//
// P2L Loop
//
    LoopCount=1;
    ICP_Error2=ICP_Error1;
//
// Note this loop will only revise its estimate of the delta-transformation based on revision of
// the point-pair match between pointVec and refVec. It does not use rotVec to update pointVec
// since we are looking for the best transformation matching the previous frame to the current frame.
// This is essentially a one-step process i.e. find T which solves pointVec = T(refVec)
//
    do {
//
// compute the covariance matrix 
//
//>>>
        cudaMemset(dResVec,0.0,resBytes);
        cudaMemset(dAtomVec,0,atomBytes);
        P2L_Update_Launch(grid0, block0, pointVec, pntSz, pointCount, SEGSZ, refVec, refCount, dMatch, MatchFM, dWorkVec, CellWorkStep, dResVec, dAtomVec);
        cudaMemcpy(hResVec,dResVec,resBytes,cudaMemcpyDeviceToHost);
        cudaMemcpy(hAtomVec,dAtomVec,atomBytes,cudaMemcpyDeviceToHost);
//
// Recover A and B from ResVec and perform Cholesky decomposition
// to obtain the Pose in Tau, (note the order needs checking) 
//
        Index=0;
        FMat AMat(GetData(hResVec,Index,36),6,6);
        FVec BVec(GetData(hResVec,Index,6),6);
//###
//    AMat.printMatrix(" AMat ",pDmpFile);
//    BVec.printVector(" BVec ",pDmpFile);
//###
        Cholesky_fmat(AMat, Decomp);
        CholSolve_fmat(Decomp, Tau, BVec);
//
//###
//    Tau.printVector(" Tau ", pDmpFile);
//###
        dTCB.InitMatrix(Tau);
        rotMean = dTCB.InvRotateH(pointMean);
// 
// inverse rotate all points in pointVec should give back refVec
//
//>>>
        PCA_inv_rotate_Launch(grid1, block1, pointVec, pntSz, pointCount, dTCB, rotVec);
//<<<
//
//>>>
        Reset_pointVec_element_Launch(grid1, block1,rotVec, pntSz, pointCount, ipVec);
        cudaMemset(dAtomVec, 0, atomBytes);
        PCA_Match_Launch(grid1, block1, rotVec, pntSz, pointCount, ipVec, refVec, refCount, dWorkMat, DistMin, CosMin, FMThresh, dMatch, dAtomVec);
        cudaMemcpy(hMatch,dMatch,matchBytes,cudaMemcpyDeviceToHost);
        cudaMemcpy(hAtomVec,dAtomVec,atomBytes,cudaMemcpyDeviceToHost);
//<<<
        MatchCount = hAtomVec[3];
        MatchBreaks = hAtomVec[4];
        MatchFM = hAtomVec[5];   // Number of points that passed FM Test
        printf("\nLoopCount, %d, MatchCount, %d, MatchBreaks, %d, MatchFM, %d refCount, %d, \n",LoopCount,MatchCount,MatchBreaks,MatchFM,refCount);
//###            fprintf(pDmpFile,"\nLoopCount, %d, MatchCount, %d, MatchBreaks, %d, MatchFM, %d, refCount, %d, \n",LoopCount,MatchCount,MatchBreaks,MatchFM,refCount);

        if(MatchFM<0.1*MatchStart){
            printf("Frame, %d, Error at LoopCount, %d, MatchFM, %d, < 0.1*MatchStart, %d, Resetting, to, Previous, Transform,\n",FrameNumber,LoopCount,MatchFM,MatchStart);
            dTCB.InitMatrix(dangi,dxgbi);
            ICP_Error2 = ICP_Error1;
            break;
        }
/* 
//### debug start
// This is rotated hPointVec should match hRefVec above
        cudaMemcpy(hPointVec,rotVec,pntBytes,cudaMemcpyDeviceToHost);
        for(i=0;i<pntSz.y;i++){ 
            for(j=100;j<200;j++){
                k = hMatch[j];
                if(k >=0 ){   
                    int idxR = k + i*pntSz.x;
                    fprintf(pDmpFile," %.5f,",hPointVec[idxR]);
                } else{ // if the point does not have a match dont show it
                    fprintf(pDmpFile," %.5f,",0.0);
                }
            }
            fprintf(pDmpFile,"\n");
        }
        fprintf(pDmpFile,"\n");
//
//### debug end
*/
//>>>
        cudaMemset(dResVec,0,resBytes);
        P2L_Error_Launch(grid0, block0, rotVec, pntSz, pointCount, SEGSZ, refVec, refCount, dMatch, MatchFM, dWorkVec, 5, dAtomVec, dResVec);
        cudaMemcpy(hResVec,dResVec,resBytes,cudaMemcpyDeviceToHost);
//<<<
        ICP_Error2 = hResVec[0];
        ICP_ErrVec2 = make_float3(hResVec[1],hResVec[2],hResVec[3]);
        count = _NINT(hResVec[4]);

        printf(" Frame, %d, LoopCount, %d, MatchFM, %d, count, %d, ICP_Error2, %.5f, ",FrameNumber,LoopCount,MatchFM, count,ICP_Error2);
        dTCB.printPose(" | dTCB, ",1,NULL);
        fprintf(pDmpFile," Frame, %d, LoopCount, %d, MatchCount, %d, MatchBreaks, %d, MatchFM, %d, count, %d, ICP_Error2, %.5f, ",FrameNumber,LoopCount,MatchCount,MatchBreaks,MatchFM,count,ICP_Error2);
        dTCB.printPose(" | dTCB, ",0,pDmpFile);
        fprintf(pDmpFile," |rotMean, %.5f, %.5f, %.5f, ICP_ErrVec2, %.5f, %.5f, %.5f, \n", rotMean.x,rotMean.y,refMean.z, ICP_ErrVec2.x,ICP_ErrVec2.y,ICP_ErrVec2.z);

// check we are progressing and if not after 5 loops ditch out
        if((ICP_Error1-ICP_Error2)<1.0E-2*ICP_Error1) {
            if(LoopCount>5){
                break;
            }
        }

        LoopCount++;
        
        ICP_Error1=ICP_Error2;

    } while ( (LoopCount < 25) && (ICP_Error2 >ICP_TOL) );

    fprintf(pResFile," |, %.5f, %d, %d, %d, %d, ",ICP_Error2, pointCount,MatchCount,MatchFM,LoopCount);
//
    return 0;
}
//
//-------------------------------------------------------------------------------------------------
//
