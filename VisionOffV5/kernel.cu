#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <helper_math.h>

#include "../Subs/device_funcs.cuh"
#include "../Subs/GMat.cuh"

#include "kernel.h"
#include "P2P_ICP.cuh"

__global__
void renderKernel(uchar4 *d_out, float *d_vol, int scr_w, int scr_h,
    int3 volSize, int RenderMethod, float xs, float ys, float zs, float thetaX, float thetaY, float threshold,
    float dist, unsigned int *pPallet, int colours, uchar3* pCV3Out)
{
// These are the standard 2D indexing into d_out for screen rendering
//
    const int c = blockIdx.x*blockDim.x + threadIdx.x;
    const int r = blockIdx.y*blockDim.y + threadIdx.y;
    const int i = c + r * scr_w;

// I still do not understand why we need this check
    if ((c >= scr_w) || (r >= scr_h)) return; // Check if within image bounds
  
//  The background colour
    const uchar4 background = { 192, 0, 0, 0 }; // red

// In the original code, the the viewer position (src) for ray-tracing
// is placed centre of the view and infront of the model volume
// this creates an intended perspective distortion and also provides an
// ability to zoom into/magnify the image:
//
    float3 src = { xs, ys, -zs };
//
// The following variant keeps the ray parallel to the sensor Z axis for all c and r   
// It does not affect the depth data as much and improves the view width slightly:
// However it removes the ability to zoom the picture since all rays are now parallel
// 
//  float3 src = { float(c - scr_w / 2), float(r - scr_h / 2), -zs };
//  float3 pix = make_float3(c - scr_w / 2, r - scr_h / 2, volSize.z);

    float3 pix = scrIdxToPos(c, r, scr_w, scr_h, volSize.z);
//  
// apply viewing transformation: here rotate about y-axis
// so effectively we are rotating the viewer position and the pixel
// in the rendered output not the object being viewed, this makes sense?
// sort of in this simple case, but not in a more complex scene with other
// none moving objects
//
    src = yRotate(src, thetaY);
    pix = yRotate(pix, thetaY);

    src = xRotate(src, thetaX);
    pix = xRotate(pix, thetaX);

  // prepare inputs for ray-box intersection
    float t0, t1;
    const Ray pixRay = {src, pix - src};

    float3 center = {volSize.x/2.f, volSize.y/2.f, volSize.z/2.f};

    const float3 boxmin = -center;

    const float3 boxmax = {volSize.x - center.x, volSize.y - center.y,
                                                          volSize.z - center.z};
  // perform ray-box intersection test
    const bool hitBox = intersectBox(pixRay, boxmin, boxmax, &t0, &t1);

    uchar4 shade;

    if (!hitBox){
        shade = background; //miss box => background color
    } else {
        if (t0 < 0.0f) t0 = 0.0f; // clamp to 0 to avoid looking backward

// bounded by points where the ray enters and leaves the box
        const Ray boxRay = { paramRay(pixRay, t0),  paramRay(pixRay, t1) - paramRay(pixRay, t0) };

        switch (RenderMethod)
        {
          case 1:
              shade = volumeRenderShader(d_vol, volSize, boxRay, threshold, NUMSTEPS);
          break;
          case 2:
              shade = MySliceShader(d_vol, volSize, boxRay, threshold, dist, src, pPallet, colours);
          break;
          case 3:
              shade = rayCastShader(d_vol, volSize, boxRay, threshold);
          break;
          case 4:
              shade = MyRayShader(d_vol, volSize, boxRay, threshold, pPallet, colours);
          break;
          case 5:   // this is the standard shader it is occupancy not TSDF
              shade = MySimpleShader(d_vol, volSize, boxRay, threshold, pPallet, colours);
          break;
          default:
              shade = MyRayShader(d_vol, volSize, boxRay, threshold, pPallet, colours);
          }
      }

// need to get rid of this wasteful duality of RGBA and RGB
// the irritation is that OpenCV video recording would not work with RGBA format
// which is the default for OpenGL
//###    pCV3Out[i] = make_uchar3(shade.z,shade.y,shade.x);
    d_out[i] = shade;
}


void renderKernelLauncher(uchar4 *d_out, float *d_vol, int w, int h,
    int3 volSize, int RenderMethod, float xs, float ys, float zs, float thetaX, float thetaY, float threshold,
    float dist, unsigned int *pPallet, int colours, uchar3* pCV3Out) 
{
    dim3 blockSize(TX_2D, TY_2D);
    dim3 gridSize(divUp(w, TX_2D), divUp(h, TY_2D));
    renderKernel<<<gridSize, blockSize>>>(d_out, d_vol, w, h, volSize, RenderMethod, xs, ys, zs, thetaX, thetaY, threshold, dist, pPallet, colours, pCV3Out);
}

__global__ 
void DepthSurface_kernel(UINT16* depth, cudaSurfaceObject_t DarrySurf, int2 arrSz, float ViewScale, Look1Dc DepthTable, int Filter)
{
//
// Reads input from depth and performs a filter operation
// using the lookup DepthTable to assess the depth probability
// and optionally filters the data into the surface
// 
// depth        Camera image pixel data (u,v|d) d in mm
// DarrySurf    Output is written to the cudaSurfaceObject_t DarrySurf
// arrSz        defines the size of both depth and DarrySurf
// ViewScale    scales from depth in mm to voxels
// DepthTable   Lookup table of depth probability 
// Filter       switches probability filtering on/off
//
// The surface is not 3D projected output it is in image coordinates (u,v,|d)
// the surface could be extended to include other parameters such as
// exclusion of points and/or object segmentation of data points.
//
    int x = threadIdx.x + blockIdx.x * blockDim.x;
    int y = threadIdx.y + blockIdx.y * blockDim.y;

    const int cols = arrSz.x;
    const int rows = arrSz.y;
    if (x >= cols || y >= rows) {
      return;
    }

    const int in11 = x + y * cols;

    const float Depth = ViewScale*(float)depth[in11];
    float d2 = Depth;
    float p2 = 1.0; 
           
    if(Filter){
        p2 = DepthTable.dLookup(Depth);  // probability lookup table is in m 
    }

// This does work well
    float2 outd = make_float2(d2,p2);   
// write to output
    surf2Dwrite(outd, DarrySurf, x*8, y);
}

int DepthSurface_Launcher(UINT16* depth, cudaSurfaceObject_t DarrySurf, int2 arrSz, float ViewScale, Look1Dc DepthTab, int Filter)
{
    dim3 block (TX_2D, TY_2D);
    dim3 grid ( divUp (arrSz.x, block.x), divUp (arrSz.y, block.y) );

    DepthSurface_kernel<<<grid, block>>>(depth, DarrySurf, arrSz, ViewScale, DepthTab, Filter);
    
    cudaDeviceSynchronize();
    cudaError_t CudaErr = cudaGetLastError();
    if(CudaErr!=cudaSuccess){
        const char* pError = cudaGetErrorString(CudaErr);
        printf("DepthSurface %s\n",pError);
        return -1;
    }
    return 0;
}

__global__
void DarryVolumeKernel(float *d_vol, cudaSurfaceObject_t DarrySurf, int2 arrSz, int3 volSize) 
{
//
// This is the volume kernel for the above Darry_kernel. The dimensions of the
// sensor array may not match the volume X,Y dimensions 
// When the sensor array is bigger than the volume (X,Y) we scale the sensor array 
// data into the volume
//
    const int w = volSize.x, h = volSize.y, d = volSize.z;

    const int c = blockIdx.x * blockDim.x + threadIdx.x; // column      x
    const int r = blockIdx.y * blockDim.y + threadIdx.y; // row         y
    const int s = blockIdx.z * blockDim.z + threadIdx.z; // stack depth z

    if ((c >= w) || (r >= h) || (s >= d)) return;
    const int i3d = c + r * w + s * w * h;
//
// the data frame dimensions cx,cy may not match the volume w,h
// so a simple linear scaling is applied
    float kx = float(arrSz.x)/float(volSize.x);
    float ky = float(arrSz.y)/float(volSize.y);

    // keep the scaling isotropic
//    float kk = fmaxf(kx,ky);

    int cxi = _NINT(float(c)*kx);
    int cyi = _NINT(float(r)*ky);

    if( (cxi >= arrSz.x) || (cyi >= arrSz.y) ) return;
  
    float2 data;
// Read from input surface

    surf2Dread(&data, DarrySurf, cxi * 8, cyi);

  // for this point in the voxel volume compute the density function value

    d_vol[i3d] = face(s, data.x);

}

void DarryVolumeKernelLauncher(float *d_vol, cudaSurfaceObject_t DarrySurf, int2 arrSz, int3 volSize)
{
    dim3 blockSize(TX_3D, TY_3D, TZ_3D);
    dim3 gridSize( divUp(volSize.x, TX_3D), divUp(volSize.y, TY_3D), divUp(volSize.z, TZ_3D) );
    DarryVolumeKernel<<<gridSize, blockSize>>>(d_vol, DarrySurf, arrSz, volSize); 
}


__global__ 
void compute_Float_Depth(UINT16* c_Depth, float* d_Depth, int2 arrSz, float ViewScale)
{
//
// This trivial kernel rescales camera Depth Units to m for each pixel.
// It may be a good idea to implement this kernel for all those following. 
// For now the output is only used by PCA_Reduce as it unifies data directly 
// from the camera and that recovered from the view volume
//
    int x = threadIdx.x + blockIdx.x * blockDim.x;
    int y = threadIdx.y + blockIdx.y * blockDim.y;

    int cols = arrSz.x;
    int rows = arrSz.y;

    if (x >= cols || y >= rows) {
      return;
    }
// the pixel index in depth[]
    const int in2d = x + y * cols;
    d_Depth[in2d] = ViewScale*float(c_Depth[in2d]);
}

int Depth_to_Float_Launcher( UINT16* c_Depth, float* d_Depth, int2 arrSz, float ViewScale)
{
//
// trivial kernel to convert c_Depth to float d_Depth
//
    dim3 block (TX_2D, TY_2D);
    dim3 grid ( divUp (arrSz.x, block.x), divUp (arrSz.y, block.y) );

    compute_Float_Depth<<<grid, block>>>(c_Depth,d_Depth,arrSz,ViewScale);

    cudaDeviceSynchronize();
    cudaError_t CudaErr = cudaGetLastError();
    if(CudaErr!=cudaSuccess){
        const char* pError = cudaGetErrorString(CudaErr);
        printf("Depth_to_Float %s\n",pError);
        return -1;
    }
    return 0;
}


__global__ 
void compute_Float_To_Depth(float* d_Depth, UINT16* cam_Depth, int2 arrSz, float ViewScale)
{
//
// This trivial kernel rescales depth in m to camera units for each pixel.
// tedious but provides compatibility for debug.
// ViewScale needs to be the conversion from camera units to m
//
    int x = threadIdx.x + blockIdx.x * blockDim.x;
    int y = threadIdx.y + blockIdx.y * blockDim.y;

    int cols = arrSz.x;
    int rows = arrSz.y;

    if (x >= cols || y >= rows) {
      return;
    }

// the pixel index in depth[]
    const int in2d = x + y * cols;
    float val = fabsf(d_Depth[in2d]/ViewScale);
    cam_Depth[in2d] = (unsigned short int)val;
}

int Float_to_Depth_Launcher(float* d_Depth, UINT16* cam_Depth, int2 arrSz, float ViewScale)
{
//
// trivial kernel to convert c_Depth to float d_Depth
//
    dim3 block (TX_2D, TY_2D);
    dim3 grid ( divUp (arrSz.x, block.x), divUp (arrSz.y, block.y) );

    compute_Float_To_Depth<<<grid, block>>>(d_Depth, cam_Depth, arrSz,ViewScale);

    cudaDeviceSynchronize();
    cudaError_t CudaErr = cudaGetLastError();
    if(CudaErr!=cudaSuccess){
        const char* pError = cudaGetErrorString(CudaErr);
        printf("Float_to_Depth %s\n",pError);
        return -1;
    }
    return 0;
}


__global__ 
void compute_volume3D_kernel(float* d_vol, int3 volSize, cudaSurfaceObject_t DarrySurf, int2 arrSz,  GBHMat TCG, float3 xgbf, float ViewScale)
{
//
// In this model we indicate surface by a voxel 0 < val <=1.0 and zero everywhere else.
// The benefit being that we can write directly to the view volume voxel elements
// affected so we dont need to traverse in z over every voxel element in the modelling
// part, therefore this should be more efficient than TSDF
//
// d_vol        View volume in linear memory
// volSize      Voxel dimensions of d_vol 
// DarrySurf    Depth camera surface image data (u,v|d,p)  
//              this holds both the latest depth and probability 
//              data based on an imprical depth lookup table
// arrSz        Pixel dimensions of the DarrySurf image
// TCG          Camera pose transformation Xc = TCG.Xg i.e Xg = TCG-1.Xc
// xgbf         Global position of camera datum in global coordinates
// ViewScale    Converts from metres to voxels
//
// Note that TCG contains the camera offset tc in camera axes from the global datum
//
// Although working on the view volume this algorithm is scanning the image sensor
// pixel-by-pixel so we use we use (u,v) coordinates, these are then projected
// into the global coordinate frame before discretising into the volume pixel
//
// This method only requires that the voxels affected are updated but 
// a more complex volume model would allow for more complex integration
// of depth camera data
//
    int u = threadIdx.x + blockIdx.x * blockDim.x;
    int v = threadIdx.y + blockIdx.y * blockDim.y;

    int cols = arrSz.x;
    int rows = arrSz.y;

    if (u >= cols || v >= rows) {
        return;
    }
//
// Pick off the depth and probability values
//
    float2 data;
    surf2Dread(&data, DarrySurf, u * 8, v);
    float Depth = data.x;   // depth in metres (converted in DarryKernel)
    float Prob = data.y;    // this must be set based on depth lookup
    if( Depth<CAM_DMIN || Depth>CAM_DMAX ){
        return;
    }
//
// Project from camera image coordinates into physical camera body coordinates
//
    float3 Cam;
    Cam.x = float(u - CAM_CX) * Depth*CAM_Kinv;
    Cam.y = float(v - CAM_CY) * Depth*CAM_Kinv;
    Cam.z = Depth;
//
// Transform camera body axis data into global volume coordinates
//
    float3 Glob = TCG.InvR(Cam) + xgbf;
//    float3 Glob = TCG.Rotate(Cam) + xgbf;
//###
//
// Turn these physical coordinates back into voxel coordinates
// ensuring we are in the view volume
//
    int3 VolPos = make_int3(Glob.x*ViewScale, Glob.y*ViewScale, Glob.z*ViewScale);

    VolPos.x = clipWithBounds(VolPos.x, 0, volSize.x-1);
    VolPos.y = clipWithBounds(VolPos.y, 0, volSize.y-1);
    VolPos.z = clipWithBounds(VolPos.z, 0, volSize.z-1);

    const int volidx = flatten(VolPos, volSize);

// this works but it 
    float OldVal = d_vol[volidx];       // the volume must be initialised ==0 before starting
    float NewVal = (OldVal+Prob);       // (OldVal+Prob)/2.0 cuts out a lot of background;
    NewVal = fmaxf(0.0,fminf(1.0,NewVal));

    d_vol[volidx] = NewVal;
}

int volume3DKernel_Launcher(float* d_vol, int3 volSize, cudaSurfaceObject_t DarrySurf, int2 arrSz, GBHMat TCG, float3 xgbf, float ViewScale)
{
    dim3 block (TX_2D, TY_2D);
    dim3 grid ( divUp (arrSz.x, block.x), divUp (arrSz.y, block.y) );

    compute_volume3D_kernel<<<grid, block>>>(d_vol, volSize, DarrySurf, arrSz, TCG, xgbf, ViewScale);
    cudaDeviceSynchronize();
    cudaError_t CudaErr = cudaGetLastError();
    if(CudaErr!=cudaSuccess){
        const char* pError = cudaGetErrorString(CudaErr);
        printf("volume3DKernel_Launcher %s\n",pError);
        return -1;
    }
    return 0;

}

__global__ 
void compute_Depth_From_Vol(float* d_vol, int3 volSize, float* v_Depth, int2 arrSz,  GBHMat TCG, float3 xgbf, float ViewScale, float* dResVec)
{
//
// From the current position given in TCG, xgbf get back the depth surface from within the view volume
// This can then be used in another ICP loop to correct for the forward pass in a loop closure.
//
// d_vol        View volume in linear memory
// volSize      Voxel dimensions of d_vol 
// v_depth      Depth array containing depth image data (u,v|d)  
//              recovered from the view-volume in camera coordinates
// arrSz        Pixel dimensions of v_Depth
// TCG          Camera pose transformation Xc = TCG.Xg i.e Xg = TCG-1.Xc
// xgbf         Global position of camera datum in global coordinates
// ViewScale    Converts from metres to voxels and back
//
// Note that TCG contains the camera offset tc in camera axes from the global datum xgbf
//
// Given the view volume is made up of voxels the best resolution is going to be 5mm in all directions
// we need to scan along the depth ray to find the first non-zero voxel
// the model assumes that the depth will be more than CAM_DMIN and less than CAM_DMAX in any direction
// However the hitbox is applied to ensure we dont step out of the view volume
//
// The easiest way to project along a ray is to have a Min and Max points along it
// and then step along the ray, this requires only two transformations per pixel
// much more efficient than up to 800 if we single stepped
//
    const int u = threadIdx.x + blockIdx.x * blockDim.x;
    const int v = threadIdx.y + blockIdx.y * blockDim.y;
//###
//    const int u = 3;    // test case
//    const int v = 30;
//###
    const int cols = arrSz.x;
    const int rows = arrSz.y;

    if (u >= cols || v >= rows) {
        dResVec[0] -= 1.0;
        return;
    }
//
// Index into depth vector
//
    int in2d = u + v*arrSz.x;
    v_Depth[in2d] = 0.0;    // default value
//
// Transform camera body axis data into global volume 
// coordinates at the start and end of the ray
// any surface cannot be closer than CAM_DMIN or further
// than CAM_DMAX as the sensor is blind outside these limits
//
    float3 CamMin, CamMax;
    const float tanPsi   = float(u - CAM_CX)*CAM_Kinv;
    const float tanTheta = float(v - CAM_CY)*CAM_Kinv;

    CamMin.z = CAM_DMIN;
    CamMin.x = tanPsi*CAM_DMIN;    // = tan(psi)*zmin
    CamMin.y = tanTheta*CAM_DMIN;    // = tan(theta)*zmin

    float3 GlobMin = TCG.InvR(CamMin) + xgbf;

    CamMax.z = CAM_DMAX;
    CamMax.x = tanPsi*CAM_DMAX;    // = tan(psi)*zmin
    CamMax.y = tanTheta*CAM_DMAX;    // = tan(theta)*zmin

    float3 GlobMax = TCG.InvR(CamMax) + xgbf;
/*
    dResVec[0] = CamMin.x;
    dResVec[1] = CamMin.y;
    dResVec[2] = CamMin.z;

    dResVec[3] = CamMax.x;
    dResVec[4] = CamMax.y;
    dResVec[5] = CamMax.z;

    dResVec[6] = GlobMin.x;
    dResVec[7] = GlobMin.y;
    dResVec[8] = GlobMin.z;

    dResVec[9] = GlobMax.x;
    dResVec[10] = GlobMax.y;
    dResVec[11] = GlobMax.z;
*/
    float t0, t1;

    float3 PixMin = GlobMin*ViewScale;
    float3 PixMax = GlobMax*ViewScale;
    float3 PixGrad = PixMax-PixMin;
/*
    dResVec[12] = PixMin.x;
    dResVec[13] = PixMin.y;
    dResVec[14] = PixMin.z;

    dResVec[15] = PixMax.x;
    dResVec[16] = PixMax.y;
    dResVec[17] = PixMax.z;
*/
//
// we are now in voxel units
//
    const Ray pixRay = {PixMin,PixGrad};

    const float3 boxmin = make_float3(0.0,0.0,0.0);
    const float3 boxmax = make_float3(volSize.x-1,volSize.y-1, volSize.z-1);

// perform ray-box intersection test
    const bool hitBox = intersectBox(pixRay, boxmin, boxmax, &t0, &t1);

    if (!hitBox){
        dResVec[1] -= 1;
        return;
    }
    
    if (t0 < 0.0f){
        t0 = 0.0f; // clamp to 0 to avoid looking backward
    }
//###
//    dResVec[18] = t0;
//    dResVec[19] = t1;
//    dResVec[20] = float(hitBox);
//###

// boxRay is bounded by points where the ray enters and leaves the box

    const Ray boxRay = { paramRay(pixRay, t0),  paramRay(pixRay, t1) - paramRay(pixRay, t0) };
/*
    dResVec[21] = boxRay.o.x;
    dResVec[22] = boxRay.o.y;
    dResVec[23] = boxRay.o.z;

    dResVec[24] = boxRay.d.x;
    dResVec[25] = boxRay.d.y;
    dResVec[26] = boxRay.d.z;
*/
    float len = length(boxRay.d);

    float dt = 1.0/(len);   // should visit every voxel along the ray
    float f = 0.0;
    float t = 0.0;
    float3 pos;
    int3 ipos;
//
// the clever bit trace along the projected and bounded ray for a hit
//
    while (t<1.0 && f < 0.5) {
        t += dt;
        pos = paramRay(boxRay, t);
        ipos = make_int3(pos.x,pos.y,pos.z);
        f = d_vol[flatten(ipos, volSize)];
//        f = density(d_vol, volSize, pos);
    }
/*
    dResVec[27] = len;
    dResVec[28] = f;
    dResVec[29] = t;

    pos = paramRay(boxRay,0.0);

    dResVec[30] = pos.x;
    dResVec[31] = pos.y;
    dResVec[32] = pos.z;

    pos = paramRay(boxRay,0.99);
    dResVec[33] = pos.x;
    dResVec[34] = pos.y;
    dResVec[35] = pos.z;
*/
//
    float dep = 0.0;
    if(f>=0.5){
        const float deno = rsqrtf(1.0 + tanPsi*tanPsi + tanTheta*tanTheta);
        dep = t*len*deno/ViewScale + CAM_DMIN;
        v_Depth[in2d] = dep;
    } else{
        dResVec[1] -= 1;
    }

//###    dResVec[37] = dep;
}


int Depth_From_Vol_Launcher(float* d_vol, int3 volSize, float* v_Depth, int2 arrSz, GBHMat TCG, float3 xgbf, float ViewScale, float* dResVec)
{

    dim3 block (TX_2D, TY_2D);
    dim3 grid ( divUp (arrSz.x, block.x), divUp (arrSz.y, block.y) );

    compute_Depth_From_Vol<<<grid, block>>>(d_vol, volSize, v_Depth, arrSz, TCG, xgbf, ViewScale, dResVec);
//    compute_Depth_From_Vol<<<1, 1>>>(d_vol, volSize, v_Depth, arrSz, TCG, xgbf, ViewScale, dResVec);
    cudaDeviceSynchronize();
    cudaError_t CudaErr = cudaGetLastError();
    if(CudaErr!=cudaSuccess){
        const char* pError = cudaGetErrorString(CudaErr);
        printf("volume3DKernel %s\n",pError);
        return -1;
    }
    return 0;

}

__global__ 
void bilateral_kernel(UINT16* src, UINT16* dst, int2 arrSz, const int ksz, const float ssinv2, const float sdinv2)
{
// This function performs a bilateral filter on the src data and puts it out on dst
// both sized by arrSz
//
// ksz defines the size of the kernel typically = 7
// ssinv2 is the spatial constant = 0.5/(ss*ss)
// sdinv2 is the depth constant = 0.5/(sd*sd)
// the dimensions of src and dst have changed to UINT16 to match the rest of the code
// The other parameters have been tidied up/simplified a bit
//
    int x = threadIdx.x + blockIdx.x * blockDim.x;
    int y = threadIdx.y + blockIdx.y * blockDim.y;

    int cols = arrSz.x;
    int rows = arrSz.y;

    if (x >= cols || y >= rows) {
      return;
    }

    const int i2d = x + y * cols;
    float value = float(src[i2d]);

// these loop limits ensure the filter indexes remain bounded
    int startX = max (x - ksz / 2, 0);
    int endX = min (x - ksz / 2 + ksz, cols - 1);
    int startY = max (y - ksz / 2, 0);
    int endY = min (y - ksz / 2 + ksz, rows - 1);

    float sigmaDW = 0;
    float sigmaW = 0;

    for (int cy = startY; cy < endY; ++cy){
        for (int cx = startX; cx < endX; ++cx){
            const int cindex = cx + cy * cols;
             
            float depth = float(src[cindex]);

            float space2 = (x - cx) * (x - cx) + (y - cy) * (y - cy);
            float depth2 = (value - depth) * (value - depth);

            float weight = __expf (-(space2 * ssinv2 + depth2 * sdinv2));

            sigmaDW += depth * weight;
            sigmaW += weight;
        }
    }
    dst[i2d] = (UINT16)(sigmaDW / sigmaW);
}


void bilateralFilterLauncher (UINT16* src, UINT16* dst, const int2 arrSz, int kernel_size, float ssinv2, float sdinv2)
{

// my additions to make code a bit more legible 

    dim3 block (TX_2D, TY_2D);

    dim3 grid (divUp (arrSz.x, block.x), divUp (arrSz.y, block.y));

    bilateral_kernel<<<grid, block>>>(src, dst, arrSz, kernel_size, ssinv2, sdinv2);
}

__global__ 
void PCA_Reduce_kernel(float* d_Depth, int2 arrSz, int2 cellSz, int* dMap, int2 mapSz, float* pointVec, int2 pntSz, float* dWorkMat, int wMatStep, float ViewScale, float Thresh, float DetScale, int* dAtomVec)
{
// 
// This kernel performs a PCA reduction on the point cloud pixel data based on a cellular breakdown
// of the depth image data.
//
// Arguments:
//  d_Depth     Depth image array input from the camera (coverted to m)
//  arrSz       Size of depth camera array (CAM_W,CAM_H)
//  cellSz      pixel cell sizes in x,y, these must be integer divisors of arrSz.x and arrSz.y
//              The dimensions don't have to be the same but 16x16 works ok for 640x480
//  dMap        Cell map array indicates which cells have valid output
//              if output exists dMap(cx,cy) will contain the index into pointVec
//              from which the dWorkMat index can be obtained
//  mapSz       dMap size
//  pointVec    Vector into which the vertex and normal for this cell is placed
//              Depending on the cell contents there may not be a vector
//  pntSz       Size of pointVec
//  dWorkMat    Work vector used for matrix processing this data is passed to Cell_Search
//              for further processing so it is important to keep it separate from 
//              other temporary storage such as dWorkVec
//  wMatStep    Length of dWorkMat used per thread by this kernel
//  ViewScale   Converts camera units to m
//  Thresh      Curvature Threshold for surface no longer used as a rejection criteria
//  DetScale    Scales the Determinant into a useful range
//
//  dAtomVec[0] Contains total number of valid pointVec's found
//              It must be set zero before the kernel is launched
//  dAtomVec[1] Used for dWorkMat memory allocation, each thread gets its slice of dWorkMat memory
//  dAtomVec[2] Number of cells which contain Nulls
//  dAtomVec[3] Number of Cells which fail Thresh
//  dAtomVec[4] Number of Cells which fail Vector Match 
//
//  dResVec     Used for debug output of matrices
//
// macros:
// CAM_DMIN     Minimum distance for a valid depth point (varies with camera see constants.h)
// PVEC_ELEMENT Location of fTest (used in point-pair matching) in pointVec
//
// The camera depth data is segmented into (cellSz.x,cellSz.y) cells and processed independently  
// to produce up to KNVecs mean vertex points. Each vertex is stored in pointVec for later use in ICP.
//
// The output for each cell (is stored vertically in pointVec  [X, Y, Z, nx,ny,nz,startW,det....]^T
//
// where:
// X,Y,Z        are the mean vertex for this cell, 
// nx,ny,nz     are the surface normal
// startW       Start location into dWorkMat
// det          The determinant for this cells covariance matrix
// pointVec[myIndex + 8*pntSz.x]        is used in pair-matching to inhibit double matches to the same point
// if(pntSz.y>9) the upper portion will contain the [3x3] covariance matrix for this cell
//
//-------------------------------------------------------------------------------------------------------
//
    float Xm=0.0, Ym=0.0, Zm=0.0;
    float Xp=0.0, Yp=0.0, Zp=0.0;
    float det=1.0, sum=0.0, Vtrap=0.0;
    float *Lambda, *SLambdaM1;
   	int Idx3[3];
    int x, y, i, j, k;
    int iTest=0;
    int in2d ,ix, iy;

// these are used to compute an approximate normal
    float3 x0, x1, y1, p1, p2, ng, Vj;
//
    const int Cellx = threadIdx.x + blockIdx.x * blockDim.x;
    const int Celly = threadIdx.y + blockIdx.y * blockDim.y;
    const int colsd = arrSz.x;
//
// trap and log overshoots (should be zero)
    if( ((Cellx+1)*cellSz.x>arrSz.x) || ((Celly+1)*cellSz.y>arrSz.y) ){
        atomicAdd(&dAtomVec[5], 1);
        return;
    }
//
// the start index for this cell in depth[]
    const int startd = Cellx*cellSz.x + Celly*colsd*cellSz.y;

// index into dMap
    const int mapIdx = Get2DIndex(Cellx, Celly, mapSz.x);

    dMap[mapIdx] = -1;  // this indicates a cell with zero elements in dMap

// total pixels processed in this cell
    const float total = float(cellSz.x*cellSz.y);

// For the first pass reject any patch which contains zero elements
// perform the vertex mean summation along the way
//
    for(x=0;x<cellSz.x;x++){
        for(y=0;y<cellSz.y;y++){
            ix = Cellx*cellSz.x + x; 
            iy = Celly*cellSz.y + y;
            in2d = startd + x + y*colsd;
//
            Zp = d_Depth[in2d];
            if(Zp<CAM_DMIN ) {
//
// this atomicAdd bumps the count of null cells in the frame
// and since cells with zeros are not processed further leave
//
                atomicAdd(&dAtomVec[2], 1);
                return;
            }
            Zm += Zp;
            Xm += float(ix - CAM_CX)*Zp*CAM_Kinv;
            Ym += float(iy - CAM_CY)*Zp*CAM_Kinv;
        }
    }
// compute the mean for this patch
    Xm = Xm/total;
    Ym = Ym/total;
    Zm = Zm/total;
//
// Pull off the corner vectors for a geometric normal ng
// a bit long winded but better than burying the above loop with conditionals
//
    x = y = 0;
    in2d = startd + x + y*colsd;
    ix = Cellx*cellSz.x + x; 
    iy = Celly*cellSz.y + y;
    Zp = d_Depth[in2d];
    Xp = float(ix - CAM_CX)*Zp*CAM_Kinv;
    Yp = float(iy - CAM_CY)*Zp*CAM_Kinv;
    x0 = make_float3(Xp,Yp,Zp);
// x1
    x = cellSz.x-1; y=0;
    in2d = startd + x + y*colsd;
    ix = Cellx*cellSz.x + x; 
    iy = Celly*cellSz.y + y;
    Zp = d_Depth[in2d];
    Xp = float(ix - CAM_CX)*Zp*CAM_Kinv;
    Yp = float(iy - CAM_CY)*Zp*CAM_Kinv;
    x1 = make_float3(Xp,Yp,Zp);
// y1
    x=0; y=cellSz.y-1;
    in2d = startd + x + y*colsd;
    ix = Cellx*cellSz.x + x; 
    iy = Celly*cellSz.y + y;
    Zp = d_Depth[in2d];
    Xp = float(ix - CAM_CX)*Zp*CAM_Kinv;
    Yp = float(iy - CAM_CY)*Zp*CAM_Kinv;
    y1 = make_float3(Xp,Yp,Zp);
// ng 
    p1 = x1 - x0;
    p2 = y1 - x0;
    ng = normalize(cross(p1, p2));
//
// using wIndex = atomicAdd[1] will access a chunk of contiguious memory in the work vector
// for use by this cell/thread.
// 
// wIndex is different to myIndex to allow the process to
// dump out and myIndex should not contain gaps.
//
    const int wIndex = atomicAdd(&dAtomVec[1], 1);
    const int startW = wIndex*wMatStep;

    float* pWorkSt = &dWorkMat[startW];

    int InIndex=0;
//  matrices for Covariance, Eigen-vectors and Rotation matrix data spaces
    GMat C(GetData(pWorkSt,InIndex,9),3,3);
    GMat AM1(GetData(pWorkSt,InIndex,9),3,3);
    Lambda = GetData(pWorkSt,InIndex,3);
    SLambdaM1 = GetData(pWorkSt,InIndex,3);
    GMat V(GetData(pWorkSt,InIndex,9),3,3);
    GMat W(GetData(pWorkSt,InIndex,9),3,3);
    GMat R(GetData(pWorkSt,InIndex,9),3,3);
    GMat S(GetData(pWorkSt,InIndex,3*int(total)),3,int(total));
//
// Build the S matrix S(3,N) from the image patch
//
    for(y=0;y<cellSz.y;y++){
        for(x=0;x<cellSz.x;x++){
            in2d = startd + x + y*colsd;
            const int ix = Cellx*cellSz.x + x; 
            const int iy = Celly*cellSz.y + y;
            const int inS =  x + y*cellSz.x;
//
            Zp = d_Depth[in2d];
            Xp = float(ix - CAM_CX)*Zp*CAM_Kinv;
            Yp = float(iy - CAM_CY)*Zp*CAM_Kinv;
            S(0,inS) = Xp - Xm;
            S(1,inS) = Yp - Ym;
            S(2,inS) = Zp - Zm;
        }
    }
//
// Compute the Covariance Matrix C = S*S'/(n-1)
//
    for (i=0;i<3;i++){
        for (j=0;j<3;j++){ 
            C(i,j) = 0.0;
            for(k=0;k<total;k++){
                C(i,j) = C(i,j) + S(i,k)*S.Trans(k,j);
            }
            C(i,j) = C(i,j)/float(total-1);
        }
    }
//
// Save C into S as Jacobi will destroy it and it is needed for Forstner metric
//
    for(i=0;i<3;i++){
        for(j=0;j<3;j++){
            S(i,j) = C(i,j);
        }
    }
//
// Perform Jacobi iteration on C to get Eigen-values Lambda and vectors V
// this reduces C to eigen-values on lead diagonal and residuals
//
    iTest = Jacobi_GMat(C,V,W,R);

	for(i=0;i<3;i++){
		Lambda[i] = C(i,i);
	}

	ShellSort_GPU(Lambda, Idx3, 3);
//
// Swap the e-vectors to match e-value-order
//
	for(i=0;i<3;i++){
		for(j=0;j<3;j++){
			W(i,j) = V(i,Idx3[j]);
		}
	}

	for(i=0;i<3;i++){
		for(j=0;j<3;j++){
			V(i,j) = W(i,j);
		}
	}
//
// compute some intermediates 
    sum=0.0;
    det=1.0;
    for(i=0;i<3;i++){
        SLambdaM1[i] = rsqrtf(Lambda[i]);       // = sqrt(1/Lambda)
        sum += Lambda[i];
        det *= Lambda[i];
    }
//
// The determinants are rather small so this operation scales them depending on cell size
// see int AllocateMemory() below main. 
//
    det*=DetScale;
//
// recover C from S
//
    for(i=0;i<3;i++){
        for(j=0;j<3;j++){
            C(i,j) = S(i,j);
        }
    }
//
// Compute AM1
//
    for(i=0;i<3;i++){
        for(j=0;j<3;j++){
            W(i,j) = SLambdaM1[i]*V(j,i);   // [1/sqrt(Lambda)].V'
        }
    }

    for(i=0;i<3;i++){
        for(j=0;j<3;j++){
            AM1(i,j) = 0.0;
            for(k=0;k<3;k++){
                AM1(i,j) += V(i,k)*W(k,j);  // V.[sqrt(1/Lambda)].V'
            }
        }
    }

// this is an indicator of surface curvature    
    if(Lambda[0]>Thresh*sum){
        atomicAdd(&dAtomVec[3], 1);
    }
//
// get the principal normal from the reduction
// this part could possibly be removed. 
// The issue is the random scalling/sign of eigen-vectors
// this is checked against the geometric vector computed above
//
    Vj = make_float3(V(0,0),V(1,0),V(2,0));

// check for sign agreement between the geometric and PCA-derived normal 
    int sx = signbit(ng.x*Vj.x);
    int sy = signbit(ng.y*Vj.y);
    int sz = signbit(ng.z*Vj.z);

    Vtrap=0;
    if(sx && sy && sz){
// they disagree in all 3 components therefore a sign change to Vj
        Vj = -Vj;
    } else if(!sx && !sy && !sz){
// they totally agree do nowt
    } else {
// islolated sign disagree check if it is above the noise threshold
        const float Vprod = ng.x*Vj.x + ng.y*Vj.y + ng.z*Vj.z;
        if(Vprod<0.98){
            atomicAdd(&dAtomVec[4], 1);
            Vtrap=1.0;
        }
    }
//
// Since a vertex and surface normal exists for this patch
// atomicAdd will grab the next available index for the pointVec vector
//
    const int myIndex = atomicAdd(&dAtomVec[0], 1); // this is essential
//
// if a cell is sucessfully processed the index into pointVec is stored in dMap
// this allows the 2D stucture of the downsampled/processed image to be retained
//
    dMap[mapIdx] = myIndex;
//
// save the mean vertex and normal for this cell and the number of pixels used
//
    pointVec[myIndex            ] = Xm;     // mean X
    pointVec[myIndex +   pntSz.x] = Ym;     // mean Y
    pointVec[myIndex + 2*pntSz.x] = Zm;     // mean Z
    pointVec[myIndex + 3*pntSz.x] = Vj.x;   // nx
    pointVec[myIndex + 4*pntSz.x] = Vj.y;   // ny
    pointVec[myIndex + 5*pntSz.x] = Vj.z;   // nz
    pointVec[myIndex + WVEC_START*pntSz.x] = startW; // starting point in dWorkMat for this cell/threads data
    pointVec[myIndex + 7*pntSz.x] = det;    // determinant of covariance for this cell
//
    pointVec[myIndex + PVEC_ELEMENT*pntSz.x] = 0.0;       // this is fTest
//
// Store the covariance matrix in pointVec if it is required
// this is needed for processing in ComputeFM and removes the
// need for two copies of dWorkMat from which the other matrices
// can be recovered
//
    if(pntSz.y>11){    
        int pointStart = PVEC_ELEMENT+1;
        for(i=0;i<3;i++){
            for(j=0;j<3;j++){
                pointVec[myIndex + pointStart*pntSz.x] = C(i,j);
                pointStart++;
            }
        }
    }
//### debug
    W(0,0) = Lambda[0];
    W(1,0) = Lambda[1];
    W(2,0) = Lambda[2];
    W(0,1) = ng.x;
    W(1,1) = ng.y;
    W(2,1) = ng.z;
    W(0,2) = det;
    W(1,2) = Vtrap;
    W(2,2) = 0.0;
//### end of debug
//
// were done
}

int PCA_Reduce_Launcher(float* d_Depth, int2 arrSz, int2 cellSz, int* dMap, int2 mapSz, float* pointVec, int2 pntSz, float* dWorkMat, int workStep, float ViewScale, float Thresh, float DetScale, int* dAtomVec)
{
//
// To facilitate flexible cell sizes the grid/block dimension need to be flexible
// and within the constraints of the GPU (max threads per block=1024, 
// total threads 36864 on RTX-2070)
// for this version we assume square cells cellSz.x = cellSz.y. This
// leads to cellSz.x*cellSz.y cells per thread. Thus the grid and TPB 
// sizes must add up to give [640x480] total pixels processed.
// The ratio of pixels is X/Y pixles 4/3 and this leads naturally to
// the grid used. The TBP is then simply obtained from the above rules.
// This will obviously change if a different camera pixel setting is used.
//
    int2 Bl;
    int2 Gr;

    switch(cellSz.x)
    {
        case 32:
            Bl.x = 5; Bl.y=5;   
            Gr.x = 4; Gr.y=3;   
            break;
        case 16:
            Bl.x = 10; Bl.y=10; 
            Gr.x = 4; Gr.y=3;   
            break;
        case 10:
            Bl.x=16; Bl.y=16; 
            Gr.x=4; Gr.y=3;
            break;
        case 8:
            Bl.x=20;Bl.y=20;
            Gr.x=4; Gr.y=3;
            break;
        case 5:
            Bl.x=16;Bl.y=16;
            Gr.x=8; Gr.y=6;
            break;
        case 4:
            Bl.x=20;Bl.y=20;
            Gr.x=8; Gr.y=6;
            break;
        case 2:
            Bl.x=20;Bl.y=20;
            Gr.x=16; Gr.y=12;
            break;
        default:
            printf(" Error PCA_Reduce_Launcher CellSz %d Not Allowed \n",cellSz.x);
            return -1;
            break;
    }
    dim3 block ( Bl.x, Bl.y );
    dim3 grid ( Gr.x, Gr.y );

    PCA_Reduce_kernel<<<grid, block>>>(d_Depth, arrSz, cellSz, dMap, mapSz, pointVec, pntSz, dWorkMat, workStep, ViewScale, Thresh, DetScale, dAtomVec);

    cudaDeviceSynchronize();
    cudaError_t CudaErr = cudaGetLastError();
    if(CudaErr!=cudaSuccess){
        const char* pError = cudaGetErrorString(CudaErr);
        printf("PCA_Reduce %s\n",pError);
        return -1;
    }
    return 0;

}

__global__
void PCA_Surface(cudaSurfaceObject_t DarrySurf, int2 arrSz, int2 cellSz, int* dMap, int2 mapSz, float* pointVec, int pointCount, int2 pntSz, int* dAtomVec)
{
//
// This function populates DarrySurf with segmented surfaces based on the contents of
// pointVec and information contained in dMap, currently this is only needed to visualise
// the effect of PCA_Reduce for debug however it has the potential to take a nonparametric
// volume model and turn it into a parametric one based on triangulation of the surfaces.
//
// The surfaces are constructed according to cellSz projected into the model
// and using surface normal data also contained in pointVec 
// The ordering of data in pointVec is random so dMap is used as the primary iterator 
// The surface is written to DarrySurf to allow volume integration to be seamless
//
    float dx, dy, deno;
// 
    const int Cellx = threadIdx.x + blockIdx.x * blockDim.x;
    const int Celly = threadIdx.y + blockIdx.y * blockDim.y;
//
// trap and log overshoots
//
    const int cellMaxX = arrSz.x/cellSz.x;
    const int cellMaxY = arrSz.y/cellSz.y;

    if( ((Cellx+1)>cellMaxX) || ((Celly+1)>cellMaxY) ){
        atomicAdd(&dAtomVec[1], 1);
        return;
    }
//
// these are the corner pixels of the current cell
// clockwise from top left a->b->c->d
//
    const int2 a = make_int2(Cellx*cellSz.x , Celly*cellSz.y);
    const int2 b = make_int2( a.x + cellSz.x, a.y );
    const int2 c = make_int2( a.x + cellSz.x, a.y + cellSz.y);
    const int2 d = make_int2( a.x           , a.y + cellSz.y);
//
// get pointVec index from dMap
//
    const int mapIdx = Get2DIndex(Cellx, Celly, mapSz.x);
    const int myIndex = dMap[mapIdx];

// dont draw if the cell index is not valid
    if( (myIndex<0) || (myIndex>=pointCount) ){
        atomicAdd(&dAtomVec[2], 1);
        return;
    }
//
// This is the mean vertex of the current cell in metres
    const float Xp = pointVec[myIndex            ];     // mean X
    const float Yp = pointVec[myIndex +   pntSz.x];     // mean Y
    const float Zp = pointVec[myIndex + 2*pntSz.x];     // mean Z
//
// surface normal for the cell
    const float Nx = pointVec[myIndex + 3*pntSz.x];   // nx
    const float Ny = pointVec[myIndex + 4*pntSz.x];   // ny
    const float Nz = pointVec[myIndex + 5*pntSz.x];   // nz

    const float nemo = Nx*Xp + Ny*Yp + Nz*Zp; 

    dx = float(a.x-CAM_CX)*CAM_Kinv;
    dy = float(a.y-CAM_CY)*CAM_Kinv;
    deno = Nx*dx + Ny*dy + Nz;
    const float Za = nemo/deno;

    dx = float(b.x-CAM_CX)*CAM_Kinv;
    dy = float(b.y-CAM_CY)*CAM_Kinv;
    deno = Nx*dx + Ny*dy + Nz;
    const float Zb = nemo/deno;

    dx = float(d.x-CAM_CX)*CAM_Kinv;
    dy = float(d.y-CAM_CY)*CAM_Kinv;
    deno = Nx*dx + Ny*dy + Nz;
    const float Zd = nemo/deno;
//
// Za, Zb, Zc & Zd are the depths at the corners of this cell
// now populate every pixel within this cell at the correct depth
//
    const float Mab = (Zb - Za)/float(cellSz.x);
    const float Mad = (Zd - Za)/float(cellSz.y);
//
    for(int i=0; i<cellSz.x; i++){
        for(int j=0;j<cellSz.y; j++){
            float Zc = Za + Mab*float(i) + Mad*float(j);
            float2 outd = make_float2(Zc ,0.51);   

            int x = a.x + i;
            int y = a.y + j;
            surf2Dwrite(outd, DarrySurf, x*8, y);
        }
    }
    atomicAdd(&dAtomVec[0],1);
}

int PCA_Surface_Launcher(cudaSurfaceObject_t DarrySurf, int2 arrSz, int2 cellSz, int* dMap, int2 mapSz,  float* pointVec, int pointCount, int2 pntSz, int* dAtomVec)
{
//
// Although a surface is being developed all the data is contained
// in pointVec, dMap arrSz and cellSz
//
    int2 Bl;
    int2 Gr;

    switch(cellSz.x)
    {
        case 32:
            Bl.x = 5; Bl.y=5;   
            Gr.x = 4; Gr.y=3;   
            break;
        case 16:
            Bl.x = 10; Bl.y=10; 
            Gr.x = 4; Gr.y=3;   
            break;
        case 10:
            Bl.x=16; Bl.y=16; 
            Gr.x=4; Gr.y=3;
            break;
        case 8:
            Bl.x=20;Bl.y=20;
            Gr.x=4; Gr.y=3;
            break;
        case 5:
            Bl.x=16;Bl.y=16;
            Gr.x=8; Gr.y=6;
            break;
        case 4:
            Bl.x=20;Bl.y=20;
            Gr.x=8; Gr.y=6;
            break;
        case 2:
            Bl.x=20;Bl.y=20;
            Gr.x=16; Gr.y=12;
            break;
        default:
            printf(" Error PCA_Surface_Launcher CellSz %d Not Allowed \n",cellSz.x);
            return -1;
            break;
    }

    dim3 block ( Bl.x, Bl.y );
    dim3 grid ( Gr.x, Gr.y );

    PCA_Surface<<<grid, block>>>(DarrySurf, arrSz, cellSz, dMap, mapSz, pointVec, pointCount, pntSz, dAtomVec);
    cudaDeviceSynchronize();
    cudaError_t CudaErr = cudaGetLastError();
    if(CudaErr!=cudaSuccess){
        const char* pError = cudaGetErrorString(CudaErr);
        printf("PCA_Kernel %s\n",pError);
        return -1;
    }
    return 0;
}
//
//-----------------------------------------------------------------------------------------------------------------------
//

__global__ 
void compute_histogram(float* Hyst ,int2 hstSz, UINT16* newDepth, UINT16* oldDepth, int2 arrSz, float ViewScale, float DZ, float DV, float FPS)
{
//
// computes a histogram of anything using GPU
// 
//
    int u = threadIdx.x + blockIdx.x * blockDim.x;
    int v = threadIdx.y + blockIdx.y * blockDim.y;

    int cols = arrSz.x;
    int rows = arrSz.y;

    if (u >= cols || v >= rows) {
        return;
    }

    const int in2d = u + v * cols;
    float vel = float(newDepth[in2d]-oldDepth[in2d])*FPS*ViewScale;
    int iz = int(float(newDepth[in2d])*ViewScale/DZ);
    int iv = int(vel/DV)+hstSz.x/2;
    int iy = clipWithBounds(iz, 0, hstSz.y-1);
    int ix = clipWithBounds(iv, 0, hstSz.x-1);

    int iHyst = ix + iy*hstSz.x;
    Hyst[iHyst] += 1.0;
}

void HistogramLauncher(float* Hyst, int2 hstSz, UINT16* b_Depth, UINT16* o_Depth, int2 arrSz, float ViewScale, float DZ, float DV, float FPS)
{
    dim3 block (TX_2D, TY_2D);
    dim3 grid ( divUp (arrSz.x, block.x), divUp (arrSz.y, block.y) );

    compute_histogram<<<grid, block>>>(Hyst ,hstSz, b_Depth, o_Depth, arrSz, ViewScale, DZ, DV, FPS);
}



__global__ 
void CropFilter(UINT16* src, UINT16* dest, int2 arrSz, int2 cellSz, float ViewScale, float CropDist, int ZeroOut)
{
//
// A crude filter to clip anything further than CropDist.
// ZeroOut used to specify which axis or all
//   ZeroOut = 1 remove X axis items further than CropDist
//   ZeroOut = 2 remove Y axis items further than CropDist
//   ZeroOut = 3 remove Z axis items further than CropDist
//   ZeroOut = 4 remove data from all axes further than CropDist
//
// The filter uses the same
// cellular structure formed for the PCA_Reduce kernel but in this case
// it is applied as a filter to the whole data image This allows 
// the effect of cell reduction to be seen visually. If ZeroOut is set
// all data beyond CropDist is set zero to kill the background
//
    int cx, cy;
//
    float val;
    const int Cellx = threadIdx.x + blockIdx.x * blockDim.x;
    const int Celly = threadIdx.y + blockIdx.y * blockDim.y;

    const int colsd = arrSz.x;
//
// the start index for this cell in depth[]

    if((Cellx+1)*cellSz.x>=arrSz.x) return;
    if((Celly+1)*cellSz.y>=arrSz.y) return;

    const int startd = Cellx*cellSz.x + Celly*colsd*cellSz.y;
//
// Simply copy the image if we are less than crop distance
//
    for(cx=0;cx<cellSz.x;cx++){
        for(cy=0;cy<cellSz.y;cy++){
            const int invs = startd + cx + colsd*cy;
            const int ix = Cellx*cellSz.x + cx; 
            const int iy = Celly*cellSz.y + cy;
            const float Zp = src[invs]*ViewScale;
            const float Xp = float(ix - CAM_CX)*Zp*CAM_Kinv;
            const float Yp = float(iy - CAM_CY)*Zp*CAM_Kinv;

            switch(ZeroOut)
            {
                case 1:
                    val = Xp;
                    break;
                case 2:
                    val = Yp;
                    break;
                case 3:
                    val = Zp;
                    break;
                case 4:
                    val = norm3df(Xp,Yp,Zp);
                    break;
                default:
                    val = norm3df(Xp,Yp,Zp);
                break;
            }

            if( val < CropDist ){
                dest[invs] = src[invs];
            }else{
                dest[invs]=0.0;
            }
        }
    }
}
//
//----------------------------------------------------------------------------------------------
//
void CropFilterLauncher (UINT16* src, UINT16* dest, int2 arrSz, int2 cellSz, float ViewScale, float CropDist, int ZeroOut)
{
    int2 Bl;
    int2 Gr;
    
    switch(cellSz.x)
    {
        case 32:
            Bl.x = 5; Bl.y=5;   
            Gr.x = 4; Gr.y=3;   
            break;
        case 16:
            Bl.x = 10; Bl.y=10; 
            Gr.x = 4; Gr.y=3;   
            break;
        case 10:
            Bl.x=16; Bl.y=16; 
            Gr.x=4; Gr.y=3;
            break;
        case 8:
            Bl.x=20;Bl.y=20;
            Gr.x=4; Gr.y=3;
            break;
        case 5:
            Bl.x=16;Bl.y=16;
            Gr.x=8; Gr.y=6;
            break;
        case 4:
            Bl.x=20;Bl.y=20;
            Gr.x=8; Gr.y=6;
            break;
        case 2:
            Bl.x=20;Bl.y=20;
            Gr.x=16; Gr.y=12;
            break;
        default:
            printf(" Error CropFilter CellSz %d Not Allowed \n",cellSz.x);
            return;
            break;
    }
    dim3 block ( Bl.x, Bl.y );
    dim3 grid ( Gr.x, Gr.y );

    CropFilter<<<grid, block>>>(src, dest, arrSz, cellSz, ViewScale, CropDist, ZeroOut);

}

__global__ 
void Initialise_Volume_Kernel(float* d_vol, int3 volumeSize, float val)
{
    int x = threadIdx.x + blockIdx.x * blockDim.x;
    int y = threadIdx.y + blockIdx.y * blockDim.y;
    int z = threadIdx.z + blockIdx.z * blockDim.z;
    
    if (x > volumeSize.x || y > volumeSize.y || z > volumeSize.z){
        return;
    }
    int index = flatten(x,y,z,volumeSize);
    d_vol[index] = val;
}

void Init_Volume_Launcher(float* d_vol, int3 volumeSize, float val)
{
    dim3 block (TX_3D, TY_3D, TZ_3D);
    dim3 grid (divUp(volumeSize.x, TX_3D), divUp(volumeSize.y, TY_3D), divUp(volumeSize.z, TZ_3D) );

    Initialise_Volume_Kernel<<<grid, block>>>(d_vol, volumeSize, val);
}

