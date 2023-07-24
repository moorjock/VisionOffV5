//
// P2P_ICP.cuh
//
//  Contains the Point-2-Point ICP kernel and all supporting called GPU Kernel functions
//
// :
//
//  compute_mean(pointVec, pntSz, pointCount, SEG32, dWorkVec, 4, dResVec, dAtomVec);
//
//  compute_Inertia(pointVec, pntSz, pointCount, SEG32, dWorkVec, 10, dResVec, dAtomVec, refMean);
//
//  rotate_point(pointVec, pntSz, pointCount, dTCG, rotVec);
//
//  point_pair_match(pointVec, pntSz, pointCount, ref_tex, refCount, scaleFactor, dMatch, dAtomVec);
//
//  compute_ICP_Error(pointVec, pntSz, pointCount, SEG32, ref_tex, refCount, dMatch, dWorkVec, 10,
//                      									 pointMean, refMean, dAtomVec, dResVec);
//
//  get_tex_value(i,k,ref_tex,refCount,dResVec);
//
//  compute_Covariance(pointVec, pntSz, pointCount, SEG32, ref_tex, refCount, dMatch, dWorkVec, 10, 
//                          									pointMean, refMean, Cove, dAtomVec);
//  inv_rotate_points(pointVec, pntSz, pointCount, dTCG, rotVec);
//
//
#ifndef P2P_ICP_CUH
#define P2P_ICP_CUH

int P2P_ICP2_Launcher(float* pointVec, int2 pntSz, int pointCount, int* ipVec, float*rotVec,
                             float* refVec, int& refCount, int* dMatch, int& MatchCount,
                                int ICPFrames, float* dWorkVec, float* dResVec, float DistMin, float CosMin, int FrameNumber,
                                    GBHMat TCG, float3& xgbf, GBHMat dTCB, int* dAtomVec, FILE* pDmpFile, FILE* pResFile);


#endif

