//
// P2L_ICP.cu
//
// Point-to-plane ICP algorithms and helpers see headers for details
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
#include "P2Plane_ICP.cuh"
#include "ICP_Helpers.cuh"
#include "ControlParams.h"
//
GBHMat TCGL2;
//
//================================================================================================
//
int P2Plane_ICP_Launcher(float* pointVec, int2 pntSz, int pointCount, int* ipVec, float*rotVec,
                            float* refVec, int& refCount, int* dMatch, int& MatchCount, int ICPFrames, 
                                float* dWorkVec, int CellWorkStep, float* dResVec, float DistMin, float CosMin, int FrameNumber,
                                    GBHMat TCG, float3& xgbf, GBHMat dTCB, int* dAtomVec, FILE* pDmpFile, FILE* pResFile)
{
//
// Point-to-Plane ICP 
//
// This registration method uses the same input arrays as point-to-point but a different
// algorithm based on the surface normals. The normals are computed in PCA_Reduce_Launcher
// and stored in pointVec.
//
// pointVec     The latest camera depth vertex data in camera axis coordinates
// pntSz        Dimensions of the pointVec rotVec and RefVec arrays
// pointCount   Number of vertex in pointVec
// ipVec        Vector containing pointVec reference matches
//
// rotVec       rotated pointVec  = dTCB(pointVec)
// refVec       Reference version of pointVec
// refCount     Number of points in refVec
//
// dMatch       The Index array of associations in refVec with pointVec:
// MatchCount   Number of associations found
// ICPFrames    Number of frames between Reference Vector (refVec) updates
//
// dWorkVec     Device Scratch pad memory 
// CellWorkStep Size of workspace used per step used by P2L_Update_Launch
// dResVec      Vector containing intermediate results
// DistMin      Threshold minimum for point-pair match
// CosMin       Minimum cos(theta) for point-pair match
// FrameNumber  The number of this frame step
//
// TCG          Camera to global transformation
// xgbf         Initial global position of camera in view volume
// dTCB         Camera Body Axis pose delta
//
// dAtomVec     GPU Atomic variables
// pDmpFile     Output diagnostic file
// pResFile     Tracking Results
//
// Use of cuda necessarily generates lots of clutter due to memory transfers to and
// from GPU kernels. To separate this cuda code from host C code I have used: 
//>>> 
//  cuda code in here 
//<<<
//
    float3 dxgbi;
    float3 refMean, pointMean, rotMean;
//    float3 dispb, dispg, ddispg;
    int LoopCount=0;
    FMat Decomp;
    FVec Tau;
//
    float ICP_Error1, ICP_Error2;
    float3 ICP_ErrVec1, ICP_ErrVec2;    // delta X, Y and Z components of error

    int Index, MatchStart=0, MatchBreaks=0, count=0;
//    int i, j, k;

    const int resBytes = RES_VEC_SIZE*sizeof(float);
    const int atomBytes = ATOM_VEC_SIZE*sizeof(int);
    const int matchBytes = pntSz.x * sizeof(int);

    LoopCount=0;
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

// used by: Compute_Mean_2_Launch, P2L_Error_Launch, P2L_Update_Launch, 
    const dim3 block0( WorkSzX, 1);     
    const dim3 grid0( 1 , 1 ); 

// used by: Reset_pointVec_element_Launch, point_pair_match_2_Launch 
    const dim3 block1( SEGSZ, 1);
    const dim3 grid1( WorkSzX, 1 );
//
// point-pair matching on initial unrotated data
//
//>>>
    Reset_pointVec_element_Launch(grid1, block1, pointVec, pntSz, pointCount, ipVec);
    cudaMemset(dAtomVec, 0, atomBytes);
    point_pair_match_2_Launch( grid1, block1, pointVec, pntSz, pointCount, ipVec, refVec, refCount, DistMin, CosMin, dMatch, dAtomVec);
    cudaMemcpy(hMatch,dMatch,matchBytes,cudaMemcpyDeviceToHost);
    cudaMemcpy(hAtomVec,dAtomVec,atomBytes,cudaMemcpyDeviceToHost);
//<<<

    MatchCount = hAtomVec[3];
    MatchBreaks = hAtomVec[4];
    MatchStart = MatchCount;

    printf("\nMatchStart, %d, MatchBreaks, %d, refCount, %d, \n",MatchStart,MatchBreaks,refCount);
    fprintf(pDmpFile,"\nMatchStart, %d, MatchBreaks, %d, refCount, %d, \n",MatchStart,MatchBreaks,refCount);
    if(MatchStart < 0.2*pointCount){
        printf("\nWarning!!!, MatchStart, %d, pointCount, %d, in, P2Plane_ICP, @1\n",MatchCount,pointCount);
        fprintf(pDmpFile,"\nWarning!!!, MatchStart, %d, pointCount, %d, in, P2Plane_ICP, @1\n",MatchStart,pointCount);
    }
//>>>
        cudaMemset(dResVec,0,resBytes);
        cudaMemset(dAtomVec,0,atomBytes);
    Compute_Mean_2_Launch( grid0, block0, pointVec, pntSz, pointCount, refVec, refCount, dMatch, MatchCount, SEGSZ, dWorkVec, 7, dResVec, dAtomVec);
        cudaMemcpy(hResVec,dResVec,resBytes,cudaMemcpyDeviceToHost);
//<<<
    refMean = make_float3(hResVec[0],hResVec[1],hResVec[2]);
    pointMean = make_float3(hResVec[3], hResVec[4], hResVec[5]);
    count = _NINT(hResVec[6]);
//###
    printf(" refX %.5f, refY %.5f, refZ %.5f, refCount %d,\n",refMean.x, refMean.y, refMean.z, refCount);
    printf(" pntX %.5f, pntY %.5f, pntZ %.5f, pntCount %d,\n",pointMean.x, pointMean.y, pointMean.z, pointCount);
//###    printf(" count %d, \n", count);

    fprintf(pDmpFile," refX %.5f, refY %.5f, refZ %.5f, refCount %d,\n",refMean.x, refMean.y, refMean.z, refCount);
    fprintf(pDmpFile," pntX %.5f, pntY %.5f, pntZ %.5f, pntCount %d,\n",pointMean.x, pointMean.y, pointMean.z, pointCount);

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
        for(i=0;i<6;i++){
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
    P2L_Error_Launch(grid0, block0, pointVec, pntSz, pointCount, SEGSZ, refVec, refCount, dMatch, MatchCount, dWorkVec, 5, dAtomVec, dResVec);
        cudaMemcpy(hResVec,dResVec,resBytes,cudaMemcpyDeviceToHost);
//<<<
    ICP_Error1 = hResVec[0];
    ICP_ErrVec1 = make_float3(hResVec[1],hResVec[2],hResVec[3]);
    count = _NINT(hResVec[4]);

    printf(" Frame, %d, LoopCount, %d, Matchcount, %d, count, %d, ICP_Error1, %.5f, ",FrameNumber,LoopCount,MatchCount,count,ICP_Error1);
    fprintf(pDmpFile," Frame, %d, LoopCount, %d, Matchcount, count, %d, %d, ICP_Error1, %.5f, refMean, %.5f, %.5f, %.5f, ICP_ErrVec1, %.5f, %.5f, %.5f,\n ",FrameNumber,LoopCount,MatchCount,count,ICP_Error1, refMean.x,refMean.y,refMean.z, ICP_ErrVec1.x,ICP_ErrVec1.y,ICP_ErrVec1.z);
//
//=============================================================================================================
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
        P2L_Update_Launch(grid0, block0, pointVec, pntSz, pointCount, SEGSZ, refVec, refCount, dMatch, MatchCount, dWorkVec, CellWorkStep, dResVec, dAtomVec);
        cudaMemcpy(hResVec,dResVec,resBytes,cudaMemcpyDeviceToHost);
        cudaMemcpy(hAtomVec,dAtomVec,atomBytes,cudaMemcpyDeviceToHost);
//<<<
//
// Recover A and B from ResVec and perform Cholesky decomposition
// to obtain the Pose in Tau, (note the order needs checking) 
//
        Index=0;
        FMat AMat(GetData(hResVec,Index,36),6,6);
        FVec BVec(GetData(hResVec,Index,6),6);
//###
//    AMat.printMatrix(" AMat ",NULL);
//    BVec.printVector(" BVec ",NULL);
//###
        Cholesky_fmat(AMat, Decomp);
        CholSolve_fmat(Decomp, Tau, BVec);
//###
//     Tau.printVector(" Tau ", pDmpFile);
//###
        dTCB.InitMatrix(Tau);
        rotMean = dTCB.InvRotateH(pointMean);
// 
// inverse rotate all points in pointVec should give back refVec
//
//>>>
        inv_rotate_points_Launch( grid1, block1, pointVec, pntSz, pointCount, dTCB, rotVec);
//<<<
//
//>>>
        Reset_pointVec_element_Launch(grid1, block1,rotVec, pntSz, pointCount, ipVec);
        cudaMemset(dAtomVec, 0, atomBytes);
        point_pair_match_2_Launch( grid1, block1, rotVec, pntSz, pointCount, ipVec, refVec, refCount, DistMin, CosMin, dMatch, dAtomVec);
        cudaMemcpy(hMatch,dMatch,matchBytes,cudaMemcpyDeviceToHost);
        cudaMemcpy(hAtomVec,dAtomVec,atomBytes,cudaMemcpyDeviceToHost);
//<<<
        MatchCount = hAtomVec[3];
        MatchBreaks = hAtomVec[4];

        printf(" LoopCount, %d, MatchCount, %d, MatchBreaks, %d,\n",LoopCount,MatchCount,MatchBreaks);
//###        fprintf(pDmpFile," LoopCount, %d, MatchCount, %d, MatchBreaks, %d,\n",LoopCount,MatchCount,MatchBreaks);

        if(MatchCount<0.2*MatchStart){
            printf("/n FrameNumber, %d, Error at LoopCount, %d, MatchCount, %d, < 0.2*MatchStart, %d, Resetting, to, Previous, Transform,\n",FrameNumber,LoopCount,MatchCount,MatchStart);
            dTCB.InitMatrix(dxgbi,dxgbi);
            ICP_Error2 = ICP_Error1;
            break;
        }
/* 
//### debug start
// This is rotated hPointVec should match hRefVec above
        cudaMemcpy(hPointVec,rotVec,pntBytes,cudaMemcpyDeviceToHost);
        for(i=0;i<6;i++){ 
            for(j=100;j<200;j++){
                k = hMatch[j];
                if(k >=0 ){   
                    int idxR = k + i*pntSz.x;
                    fprintf(pDmpFile," %.5f,",hPointVec[idxR]);
                } else{ // if the point does not have a match set it zero
                    fprintf(pDmpFile," %.5f,",0.0);
                }
            }
            fprintf(pDmpFile,"\n");
        }
        fprintf(pDmpFile,"\n");
//### debug end
*/
        cudaMemset(dResVec,0,resBytes);
        P2L_Error_Launch(grid0, block0, rotVec, pntSz, pointCount, SEGSZ, refVec, refCount, dMatch, MatchCount, dWorkVec, 5, dAtomVec, dResVec);
        cudaMemcpy(hResVec,dResVec,resBytes,cudaMemcpyDeviceToHost);
//<<<
        ICP_Error2 = hResVec[0];
        ICP_ErrVec2 = make_float3(hResVec[1],hResVec[2],hResVec[3]);
//
        printf(" Frame, %d, LoopCount, %d, Matchcount, %d, ICP_Error2, %.5f, ",FrameNumber,LoopCount,MatchCount,ICP_Error2);
        dTCB.printPose(" | dTCB, ",1,NULL);
        fprintf(pDmpFile," Frame, %d, LoopCount, %d, Matchcount, %d, ICP_Error2, %.5f, ",FrameNumber,LoopCount,MatchCount,ICP_Error2);
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
    
    fprintf(pResFile," |, %.5f, %d, %d, %d, ",ICP_Error2, pointCount,MatchCount,LoopCount);
//
/*
    TCGL2.InitMatrix();

    for(i=0;i<3;i++){
        for(j=0;j<3;j++){
            for(k=0;k<3;k++){
                TCGL2(i,j) = TCGL2(i,j) + TCG(i,k)*dTCB(k,j);  // increment the rotation matrix
            }
        }
    }
    TCGL2(3,3) = 1.0;
    TCGL2.UpdateGPU();

// get back the delta displacements
    dispb  = dTCB.GetDisplacement();
    ddispg = TCG.InvRotateH(dispb);
    dispg  = TCG.GetDisplacement();

    dispg.x = dispg.x + ddispg.x;
    dispg.y = dispg.y + ddispg.y;
    dispg.z = dispg.z + ddispg.z;

    TCG = TCGL2;

    TCG.SetDisplacement(dispg);
*/
    return 0;
}
