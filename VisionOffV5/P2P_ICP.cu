//
// P2P_ICP.cu
//
// Point-to-Point ICP see headers for details
//
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <helper_math.h>
#include "../Subs/constants.h"

#include "../Subs/FMat.h"
#include "../Subs/GMat.cuh"
#include "../Subs/device_funcs.cuh"

//#include "../Subs/MY_NR3.h"
#include "../Subs/SVD_AS.h"

#include "VisionOffV5.h"
#include "kernel.h"
#include "ICP_Helpers.cuh"
#include "P2P_ICP.cuh"

#include "ControlParams.h"


GBHMat TCGP2;

FMat Us(3,3);
FMat Vs(3,3);
FVec Ws(3);
//
//=======================================================================================
//

int P2P_ICP2_Launcher(float* pointVec, int2 pntSz, int pointCount, int* ipVec, float*rotVec,
                             float* refVec, int& refCount, int* dMatch, int& MatchCount,
                                int ICPFrames, float* dWorkVec, float* dResVec, float DistMin, float CosMin, int FrameNumber,
                                    GBHMat TCG, float3& xgbf, GBHMat dTCB, int* dAtomVec, FILE* pDmpFile, FILE* pResFile)
{
//
// P2P_ICP2 This is version 2 of the orginal Point to Point ICP (P2P_ICP below)
// This version uses better point-pair matching with surface normals
//
// pointVec     The latest camera depth vertex data in camera axis coordinates
// pntSz        Dimensions of the pointVec rotVec and RefVec arrays
// pointCount   Number of vertex in pointVec
// rotVec       rotated pointVec  = Trans(pointVec)
//
// refVec       Reference vector
// refCount     Number of points in ref_tex
//
// dMatch       The Index array of associations in ref_tex with pointVec:
// MatchCount   Number of associations found
//
// ICPFrames    Number of frames between Reference Vector update
// dWorkVec     Device Scratch pad memory 
// dResVec      Vector containing GPU intermediate results
//
// DistMin      Threshold minimum for point-pair match
// CosMin       Minimum cos(theta) for point-pair match
// FrameNumber  The number of this step
//
// TCG          Camera transformation computed and updated here using P2P_ICP
// xgbf         Camera start position in global "volume" axes
// dTCB         delta transform between refVec and pointVec in camera body axes
//
// dAtomVec     GPU Atomic variables
// pDmpFile     Output diagnostic file
// pResFile     Results output file
//
// Use of cuda necessarily generates lots of clutter due to memory transfers to and
// from GPU kernels. To separate this cuda code from host C code I have used: 
//>>> 
//  cuda code in here 
//<<<
// I have also move the grid and block dimensions to the start of this function
// as they are used multiple times but it is important to identify which grid/blocks
// are used where
//
    GBHMat Cove;
    SVD_AS svd;  
//
    float ICP_Error1, ICP_Error2;
    int LoopCount=0;  
    float3 pointMean, refMean, dxgb, dxgbi;
    float3 dispb, ddispg, dispg;

    int i,j,k;
    int MatchBreaks=0, MatchStart=0;

    const int pntBytes = pntSz.x*pntSz.y*sizeof(float);
    const int resBytes = RES_VEC_SIZE*sizeof(float);
    const int atomBytes = ATOM_VEC_SIZE*sizeof(int);
    const int matchBytes = pntSz.x * sizeof(int);

    float DistVal = DistMin;
    float CosVal = CosMin;
//
// GPU grid and block dimensions moved here to avoid repetions
//
    dxgbi = make_float3(0.0,0.0,0.0);
//
// grid and block dimensions are specified early as they are used multiple times
// At this level the code is dealing with linear vectors from PCA_reduce
// so 1-D grids and blocks make more sense as long as TPB limit (1024) is respected
//
    int WorkSzX = divUp(pointCount, SEG32);
    int SEGSZ = SEG32;
    while(WorkSzX>1024){
        SEGSZ *= 2;
        WorkSzX = divUp(pointCount,SEGSZ);
    }
    int P2L_WorkStep = GPUStep(SEG64); // currently uses 63

// used by compute_mean(), recompute_mean, compute_P2P_Error(), compute_Covariance()
    const dim3 block0( WorkSzX, 1);     
    const dim3 grid0( 1 , 1 ); 

// used by rotate_points(), inv_rotate_points(),  point_pair_match()
    const dim3 block1( SEGSZ, 1);
    const dim3 grid1( WorkSzX, 1 );
//
    dTCB.InitMatrix(dxgbi,dxgbi);
    Cove.InitMatrix();
//
// point-pair matching on initial unrotated data
//
//>>>
    Reset_pointVec_element_Launch(grid1, block1, pointVec, pntSz, pointCount, ipVec);
        cudaMemset(dAtomVec, 0, atomBytes);
    point_pair_match_2_Launch( grid1, block1, pointVec, pntSz, pointCount, ipVec, refVec, refCount, DistVal, CosVal, dMatch, dAtomVec);
        cudaMemcpy(hMatch,dMatch,matchBytes,cudaMemcpyDeviceToHost);
        cudaMemcpy(hAtomVec,dAtomVec,atomBytes,cudaMemcpyDeviceToHost);
//<<<
    MatchCount = hAtomVec[3];
    MatchBreaks = hAtomVec[4];
    MatchStart = MatchCount;
    printf("\nMatchStart, %d, MatchBreaks %d, refCount, %d, \n",MatchStart,MatchBreaks,refCount);

    if(MatchStart < 0.2*pointCount){
        printf("\nWarning!!!, MatchStart, %d, pointCount, %d, in, P2P_ICP, @1\n",MatchCount,pointCount);
//        fprintf(pDmpFile,"\nWarning!!!, MatchStart, %d, pointCount, %d, in, P2P_ICP, @1\n",MatchStart,pointCount);
    }
//>>>
        cudaMemset(dResVec,0,resBytes);
        cudaMemset(dAtomVec,0,atomBytes);
    Compute_Mean_2_Launch( grid0, block0, pointVec, pntSz, pointCount, refVec, refCount, dMatch, MatchCount, SEGSZ, dWorkVec, 7, dResVec, dAtomVec);
        cudaMemcpy(hResVec,dResVec,resBytes,cudaMemcpyDeviceToHost);
//<<<
    refMean = make_float3(hResVec[0],hResVec[1],hResVec[2]);
    pointMean = make_float3(hResVec[3], hResVec[4], hResVec[5]);
//
    printf(" refX %.5f, refY %.5f, refZ %.5f, refCount %d,\n",refMean.x, refMean.y, refMean.z, refCount);
    printf(" pntX %.5f, pntY %.5f, pntZ %.5f, pntCount %d,\n",pointMean.x, pointMean.y, pointMean.z, pointCount);
    printf(" sum %.5f, check %.5f,\n", hResVec[6], hResVec[7]);

    fprintf(pDmpFile," refX %.5f, refY %.5f, refZ %.5f, refCount %d,\n",refMean.x, refMean.y, refMean.z, refCount);
    fprintf(pDmpFile," pntX %.5f, pntY %.5f, pntZ %.5f, pntCount %d,\n",pointMean.x, pointMean.y, pointMean.z, pointCount);
    fprintf(pDmpFile," sum %.5f, check %.5f,\n", hResVec[6], hResVec[7]);

//### debug start
/*
        fprintf(pDmpFile," hMatch \n");
        for(i=100;i<200;i++){
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

//>>>
        cudaMemset(dResVec,0,resBytes);
    compute_P2P_Error_2_Launch( grid0, block0, pointVec, pntSz, pointCount, SEGSZ, refVec, refCount, dMatch, MatchCount, dWorkVec, 3, pointMean, refMean, dAtomVec, dResVec);
        cudaMemcpy(hResVec,dResVec,resBytes,cudaMemcpyDeviceToHost);
//<<<
//    errorCount1 = int(hResVec[5]);
    ICP_Error1 = hResVec[6];
//    ICP_Xr = hResVec[7];
//    ICP_Xp = hResVec[8];
//    printf(" errorCount1 %d, ICP_Error1 %.5f ICP_Xr %.5f, ICP_Xp %.5f\n",errorCount1, ICP_Error1,ICP_Xr,ICP_Xp);
    printf("\n Frame, %d, LoopCount, %d, Matchcount, %d, ICP_Error1, %.5f, \n",FrameNumber,LoopCount,MatchCount,ICP_Error1);
    fprintf(pDmpFile,"\n Frame, %d, LoopCount, %d, Matchcount, %d, ICP_Error1, %.5f, \n",FrameNumber,LoopCount,MatchCount,ICP_Error1);
//
//=============================================================================================================
//
// ICP update Loop
//
    LoopCount=1;
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
        compute_Covariance_2_Launch( grid0, block0, pointVec, pntSz, pointCount, SEGSZ, refVec, refCount, dMatch, MatchCount, dWorkVec, 9, pointMean, refMean, Cove, dAtomVec);
        Cove.UpdateHost();
//<<<
//###
//        Cove.printMatrix(" Cove ",pDmpFile);
//###
//
// perform SVD or other method to produce update estimate of dTCB and dxgb 
// using host code. 
//
        svd.GetSingularValuesAndVectors(Cove, Us, Vs, Ws);
//
//        dTCB = V*U^T 
// care needed this is only the rotation matrix
//
//        Us.printMatrix(" Us ",pDmpFile);
//        Vs.printMatrix(" Vs ",pDmpFile);
//        Ws.printVector(" Ws ",pDmpFile);

        for(i=0;i<3;i++){
            for(j=0;j<3;j++){
                dTCB(i,j)=0.0;
                for(k=0;k<3;k++){
                    dTCB(i,j) += Vs(i,k)*Us(j,k);
                }
            }
        }

        float det = dTCB.determinant();
        if(det<0.99999f){
            printf(" ICP Error:, det %.5f, LoopCount %d\n",det,LoopCount);
            printf(" Singular Values:  W %.5f, %.5f, %.5f \n",Ws(0),Ws(1),Ws(2));
        }

        dxgb = refMean - dTCB.InvRotateH(pointMean);

        dTCB.SetDisplacement(dxgb); // this will also update the GPU 
//       dTCB.printMatrix(" dTCB ",pDmpFile);
// 
// inverse rotate all points in pointVec should give back refVec
//
//>>>
        inv_rotate_points_Launch( grid1, block1, pointVec, pntSz, pointCount, dTCB, rotVec);
//
//### This debug allows comparison of refVec with mapped and rotated pointVec
        cudaMemcpy(hPointVec,rotVec,pntBytes,cudaMemcpyDeviceToHost);   
//        fprintf(pDmpFile,"\n LoopCount %d, ",LoopCount);
//        dTCB.printPose(" dTCB, ",1,pDmpFile);

//>>>
            Reset_pointVec_element_Launch(grid1, block1,rotVec, pntSz, pointCount, ipVec);
            cudaMemset(dAtomVec, 0, atomBytes);
        point_pair_match_2_Launch( grid1, block1, rotVec, pntSz, pointCount, ipVec, refVec, refCount, DistVal, CosVal, dMatch, dAtomVec);
            cudaMemcpy(hMatch,dMatch,matchBytes,cudaMemcpyDeviceToHost);
            cudaMemcpy(hAtomVec,dAtomVec,atomBytes,cudaMemcpyDeviceToHost);
//<<<
        MatchCount = hAtomVec[3];
        MatchBreaks = hAtomVec[4];

        printf("\nrotVec, MatchCount, %d, MatchBreaks %d, refCount, %d, \n",MatchCount,MatchBreaks,refCount);
//>>>
        cudaMemset(dResVec,0,resBytes);
        cudaMemset(dAtomVec,0,atomBytes);
    Compute_Mean_2_Launch( grid0, block0, pointVec, pntSz, pointCount, refVec, refCount, dMatch, MatchCount, SEGSZ, dWorkVec, 7, dResVec, dAtomVec);
        cudaMemcpy(hResVec,dResVec,resBytes,cudaMemcpyDeviceToHost);
//<<<
        refMean = make_float3(hResVec[0],hResVec[1],hResVec[2]);
        pointMean = make_float3(hResVec[3], hResVec[4], hResVec[5]);
//###
    printf(" refX %.5f, refY %.5f, refZ %.5f, refCount %d,\n",refMean.x, refMean.y, refMean.z, refCount);
    printf(" pntX %.5f, pntY %.5f, pntZ %.5f, pntCount %d,\n",pointMean.x, pointMean.y, pointMean.z, pointCount);
    printf(" sum %.5f, check %.5f,\n", hResVec[6], hResVec[7]);

//### This is rotated hPointVec should match hRefVec above
        cudaMemcpy(hPointVec,rotVec,pntBytes,cudaMemcpyDeviceToHost);
//
/*
        fprintf(pDmpFile," Rotated PointVec \n");

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
*/
//###
        if(MatchCount<0.2*MatchStart){
            printf("Frame %d Error after Step %d MatchCount %d < 0.2*MatchStart %d Resetting to Previous Transform\n",FrameNumber,LoopCount,MatchCount,MatchStart);
            MatchCount = MatchStart;
            dTCB.InitMatrix(dxgbi,dxgbi);
            ICP_Error2 = ICP_Error1;
            break;
        }

//>>>
            cudaMemset(dResVec,0,resBytes);
        compute_P2P_Error_2_Launch( grid0, block0, rotVec, pntSz, pointCount, SEGSZ, refVec, refCount, dMatch, MatchCount, dWorkVec, 3, pointMean, refMean, dAtomVec, dResVec);
            cudaMemcpy(hResVec,dResVec,resBytes,cudaMemcpyDeviceToHost);
//<<<
//        errorCount2 = int(hResVec[5]);
        ICP_Error2 = hResVec[6];
        printf(" Frame, %d, LoopCount, %d, Matchcount, %d, ICP_Error2, %.5f, \n",FrameNumber,LoopCount,MatchCount,ICP_Error2);
        dTCB.printPose(" | dTCB, ",1,NULL);

        fprintf(pDmpFile," Frame, %d, LoopCount, %d, Matchcount, %d, ICP_Error2, %.5f, \n",FrameNumber,LoopCount,MatchCount,ICP_Error2);
        dTCB.printPose(" | dTCB, ",1,pDmpFile);

// check we are progressing and if not after 5 loops ditch out
        if((ICP_Error1-ICP_Error2)<1.0E-2*ICP_Error1) {
            if(LoopCount>5){
                break;
            }
        }

        LoopCount++;
        
        ICP_Error1=ICP_Error2;

    } while ( (LoopCount < 25) && (ICP_Error2 >ICP_TOL) );

    fprintf(pResFile," |, %.5f, %d, %d, %d, ",ICP_Error2,pointCount,MatchCount,LoopCount);
//
// do it long hand to ensure intent
//
    TCGP2.InitMatrix();

    for(i=0;i<3;i++){
        for(j=0;j<3;j++){
            for(k=0;k<3;k++){
                TCGP2(i,j) = TCGP2(i,j) + TCG(i,k)*dTCB(k,j);  // increment the rotation matrix
            }
        }
    }

    TCGP2.UpdateGPU();

// get back the delta displacements
    dispb = dTCB.GetDisplacement();
    ddispg = TCG.InvRotateH(dispb);
    dispg = TCG.GetDisplacement();

    dispg.x = dispg.x + ddispg.x;
    dispg.y = dispg.y + ddispg.y;
    dispg.z = dispg.z + ddispg.z;

    TCG = TCGP2;

    TCG.SetDisplacement(dispg);

    return 0;
}
