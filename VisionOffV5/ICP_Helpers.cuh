//
// ICP_Helpers.cuh
//
// ICP_Helpers used by P2P_ICP, P2Plane_ICP, Cell_Search and GradDescent
//
#ifndef ICP_HELPERS_CUH
#define ICP_HELPERS_CUH

int Reset_pointVec_element_Launch(dim3 grid, dim3 TBP, float* pointVec, int2 pntSz, int pointCount, int* ipVec);

int point_pair_match_2_Launch(dim3 grid, dim3 TBP, float* pointVec, int2 pntSz, int pointCount, int* ipVec,
                                float* refVec, int refCount, float DistVal, float CosVal, int* dMatch, int* dAtomVec);

int Compute_Mean_2_Launch(dim3 grid, dim3 TPB, float* pointVec, int2 pntSz, int pointCount, float* refVec, int refCount, 
                            int* dMatch, int MatchCount, int SegSzX, float* WorkVec, int workSzX, float*dResVec, int* dAtomVec);

int rotate_ref_points_Launch(dim3 grid, dim3 TBP, cudaTextureObject_t ref_tex, int2 pntSz, int refCount,
                                GBHMat dTCG, float* rotVec);

int rotate_points_Launch(dim3 grid, dim3 TBP, float* pointVec, int2 pntSz, int pointCount,
                                    GBHMat dTCG, float* rotVec);

int inv_rotate_points_Launch(dim3 grid, dim3 TBP, float* pointVec, int2 pntSz, int pointCount, 
                                    GBHMat dTCG, float* rotVec);


int compute_Covariance_2_Launch(dim3 grid, dim3 TBP, float* pointVec, int2 pntSz, int pointCount, int SegSz, float* refVec, 
                                    int refCount, int* refMatch, int MatchCount, float* WorkVec, int workSzX, float3 pointMean,
                                        float3 refMean, GBHMat Cove, int* dAtomVec);

int compute_P2P_Error_2_Launch(dim3 grid, dim3 TBP, float* pointVec, int2 pntSz, int pointCount, int SegSz, float* refVec, 
                                    int refCount, int* dMatch, int MatchCount, float* WorkVec, int workSzX, float3 pointMean, 
                                        float3 refMean, int* dAtomVec, float* dResVec);


int P2L_Error_Launch(dim3 grid, dim3 TBP, float* pointVec, int2 pntSz, int pointCount, int SegSz, float* refVec, int refCount,
                         int* dMatch, int MatchCount, float* WorkVec, int workSzX, int* dAtomVec, float* dResVec);

int P2L_Update_Launch(dim3 grid, dim3 TBP, float* pointVec, int2 pntSz, int pointCount, int SegSz, float* refVec, int refCount,
                         int* dMatch, int MatchCount, float* dWorkVec, int WorkStep, float* dResVec, int* dAtomVec);


int PCA_Match_Launch(dim3 grid, dim3 TPB, float* pointVec, int2 pntSz, int pointCount, int* ipVec,  float* refVec, 
                                int refCount, float* dWorkMat, float DistVal, float CosVal, float FMThresh, 
                                int* dMatch, int* dAtomVec);

int PCA_inv_rotate_Launch(dim3 grid, dim3 TBP, float* pointVec, int2 pntSz, int pointCount, GBHMat dTCG, float* rotVec);

__device__ float ComputeFM(int PntIdx, int RefIdx, float* pointVec, float* refVec, int2 pntSz, float* dWorkMat, int& iTest );


#endif