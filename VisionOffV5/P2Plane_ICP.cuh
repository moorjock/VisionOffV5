//
// P2Plane_ICP.cuh
// Point-to-plane ICP algorithms and helpers see headers for details
//
#ifndef P2Plane_ICP_H
#define P2Plane_ICP_H

int P2Plane_ICP_Launcher(float* pointVec, int2 pntSz, int pointCount, int* ipVec, float*rotVec, float* refVec,
                            int& refCount, int* dMatch, int& MatchCount, int ICPFrames, 
                                float* dWorkVec,  int CellWorkStep, float* dResVec, float DistMin, float CosMin, int FrameNumber,
                                    GBHMat TCG, float3& xgbf, GBHMat dTCB, int* dAtomVec, FILE* pDmpFile, FILE* pResFile);




#endif
