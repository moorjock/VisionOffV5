//
// <Cell_Search.cuh>
//
// Performs registration by cellular matching between reference and test frame
// this method works at a cellular level and re-samples the image on each registration update
//
#ifndef CELL_SEARCH_CUH
#define CELL_SEARCH_CUH
//
// These function headers are quite verbose and could be reduced since most of the pointers to vectors are declared
// in the main program and as externals in the header. However declaring them here makes flow of information
// explicit and avoids the old what-affects-what syndrome. 
//
int CellSearch_Launcher( float* d_Depth, int2 arrSz, int2 cellSz, int* dMap, int2 mapSz,
                            float* pointVec, int2 pntSz, int pointCount, int* ipVec, float* refVec, int& refCount, 
                                 float*rotVec, int* dMatch, int matchBytes, int& MatchCount, int ICPFrames,
                                    float* dWorkVec,  int CellWorkStep, float* dWorkMat, int wMatStep,  
                                        float DistMin, float CosMin, float FMThresh, int FrameNumber, 
                                            GBHMat TCG, float3& xgbf, GBHMat dTCB, 
                                                float* dResVec, int* dAtomVec, FILE* pDmpFile, FILE* pResFile);


#endif

