#ifndef KERNEL_H
#define KERNEL_H
/*
struct uchar4;
struct int3;
struct float4;
struct int2;
*/

#define K_N_POINTS     18       // maximum number of points in each vector in pointVec and refVec etc.
#define K_TEST         2.5f     // the upper limit of the camera is 3.2m
#define ICP_TOL        0.0001f  // Iteration tolerance
#define PVEC_ELEMENT   8        // location of fTest in each pointVec (used in point-pair-match-2 matching)
#define WVEC_START     6        // start location of dWorkVec in pointVec & refVec

struct Cloud
{
    int    Count;   // count of vertex points in this cloud
    float3 Mean;    // point cloud vertex mean
    float  L2;      // L2 measure of Mean
    GBHMat I;       // point cloud inertia matrix
};


void renderKernelLauncher(uchar4 *d_out, float *d_vol, int w, int h,
  int3 volSize, int RenderMethod, float xs, float ys, float zs, float thetaX, float thetaY, float threshold,
  float dist, unsigned int *pPallet, int colours, uchar3* pCV3Out);

int DepthSurface_Launcher(UINT16* depth, cudaSurfaceObject_t DarrySurf, int2 arrSz, float ViewScale, Look1Dc DepthTab, int Filter);

void DarryVolumeKernelLauncher(float *d_vol, cudaSurfaceObject_t DarrySurf, int2 arrSz, int3 volSize);

void bilateralFilterLauncher(UINT16* src, UINT16* dst, int2 arrSz, int bilat_ksz, float ssinv2, float sdinv2);

int volume3DKernel_Launcher(float* d_vol, int3 volSize, cudaSurfaceObject_t DarrySurf, int2 arrSz, GBHMat TCG, float3 xgbf, float ViewScale);

int Depth_to_Float_Launcher(UINT16* c_Depth, float* d_Depth, int2 arrSz, float ViewScale);

int Float_to_Depth_Launcher(float* d_Depth, UINT16* cam_Depth, int2 arrSz, float ViewScale);

int Depth_From_Vol_Launcher(float* d_vol, int3 volSize, float* g_Depth, int2 arrSz, GBHMat TCG, float3 xgbf, float ViewScale, float* dResVec);

int PCA_Reduce_Launcher(float* d_Depth, int2 arrSz, int2 cellSz, int* dMap, int2 mapSz, float* pointVec, int2 pntSz, float* dWorkMat, int wMatStep, float ViewScale, float Thresh, float DetScale, int* dAtomVec);

int PCA_Surface_Launcher(cudaSurfaceObject_t DarrySurf, int2 arrSz, int2 cellSz, int* dMap, int2 mapSz,  float* pointVec, int pointCount, int2 pntSz, int* dAtomVec);

void HistogramLauncher(float* Hyst , int2 hstSz, UINT16* b_Depth, UINT16* o_Depth, int2 arrSz, float ViewScale, float DZ, float DV, float FPS);

void CropFilterLauncher (UINT16* src, UINT16* dest, int2 arrSz, int2 cellSz, float ViewScale, float CropDist, int ZeroOut);

void Init_Volume_Launcher(float* d_vol, int3 volumeSize, float val);



#endif