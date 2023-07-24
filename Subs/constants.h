//
// constants.h  contains camera and viewing parameters
//
#ifndef MY_CONSTANTS_H
#define MY_CONSTANTS_H

#define CAMERA_L515

#ifndef UINT16  
    #define UINT16 unsigned short int
#endif
//
// Some Physical Constants

//
// the current camera pixel settings are optimal for the RealSense D435i camera
// they can be changed for this camera or for another device but the code must
// be self consistent so make the changes here reflect throughout the code
//
// Physical parameters for the Intel RealSense D435i Depth Camera
// RealSense D435i depth camera
#define D435i_W     640          // D435i Pixel Width recommended by Intel
#define D435i_H     480          // D435i Depth Camera Sensor Pixel Height 
#define D435i_ANG_H 87.0f        // D435i Depth Camera Field of View Horizontal (degrees)
#define D435i_ANG_W 58.0f        // D435i Depth Camera Field of View Vertical (degrees)
#define D435i_K     432.0f       // D435i Depth Camera projection constant
#define D435i_Kinv 2.3148E-3     // D435i 1/D435i_K
#define D435i_DMax   3.2f        // D435i Depth camera max distance (m) (not proven)
#define D435i_DMin  0.28f        // D435i Depth camera min distance (m) (proven)
#define D435i_D2M   0.001f       // D435i Depth camera Depth output(mm) to Metres conversion

// RealSense L515 Lidar Camera
#define L515_W     640           // L515 Pixel Width
#define L515_H     480           // L515 Pixel Height 
#define L515_ANG_H 70.0f         // L515 Depth Camera Field of View Horizontal (degrees)
#define L515_ANG_W 55.0f         // L515 Depth Camera Field of View Vertical (degrees)
#define L515_K     458.0f        // L515 Depth Camera projection constant
#define L515_Kinv 2.1834E-3      // L515 1/L515_K
#define L515_DMax   4.0f         // L515 max distance (m) (the stated is 9m but >5m likely will error prone)
#define L515_DMin  0.475f        // L515 Depth camera min distance (m) (proven)
#define L515_D2M   0.00025f      // L515 Depth camera Depth output(mm) to Metres conversion

#ifdef CAMERA_L515
    #define CAM_W     L515_W
    #define CAM_H     L515_H
    #define CAM_ANG_H L515_ANG_H
    #define CAM_ANG_W L515_ANG_W
    #define CAM_K     L515_K
    #define CAM_Kinv  L515_Kinv
    #define CAM_DMAX  L515_DMax
    #define CAM_DMIN  L515_DMin
    #define CAM_D2M   L515_D2M
#else // current camera =D435i
    #define CAM_W     D435i_W
    #define CAM_H     D435i_H
    #define CAM_ANG_H D435i_ANG_H
    #define CAM_ANG_W D435i_ANG_W
    #define CAM_K     D435i_K
    #define CAM_Kinv  D435i_Kinv
    #define CAM_DMAX  D435i_DMax
    #define CAM_DMIN  D435i_DMin
    #define CAM_D2M   D435i_D2M
#endif

// useful derived parameters
#define CAM_CX    (CAM_W/2.0)
#define CAM_CY    (CAM_H/2.0)
#define CAM_SIZE  (CAM_W*CAM_H)
#define CAM_BYTES (CAM_SIZE*sizeof(UINT16))
#define CAM_F_BYTES (CAM_SIZE*sizeof(float))
//
// Cuda Kernel Block Dimensions 
//
#define TX_2D 16
#define TY_2D 16

#define TX_3D 10
#define TY_3D 10
#define TZ_3D 10
//
// World Volume Dimensions the numbers were chosen to give integral 
// GPU grid, block size compatibility
// The VOL_NZ dimension (640) with a VOX_RES of 5mm per voxel gives 3.2m
// VOL_NY (480) gives 2.4m
//
#define VOX_RES     0.005f     // m per voxel, the volume is isotropic
#define VOL_NX      1200        // 1200        // 1200       // 424      // 600
#define VOL_NY      400         // 240      // 400
#define VOL_NZ      2000        // 500      // 600
const int3  volumeSize = { VOL_NX, VOL_NY, VOL_NZ }; // size of volumetric voxel grid

// screen pixels
#define SCREEN_W 1200        // CAM_W      // these work fine but there is no reason 
#define SCREEN_H 800         // CAM_H      // they need to be the same as the camera

#define MAX_COLOURS 256     // current colour palette limit

// review the following some of which are obsolete
#define DELTA 1000           // pixel increment for arrow keys

#define SEG32   32          // used in cuda Kernels for sizing segmented arrays
#define SEG64   64
#define SEG50   50
#define SEG100  100
#define GPU_TBP_LIM 1024    // GPU Threads per block limit

const float distmax = (VOL_NZ*VOX_RES);
const float zscale =  1.0;
const float zdist = 100*VOL_NZ;
const float xdist = 10*VOL_NX/2;

#define NUMSTEPS 20
#define KEY_BUF_LIM     80

#define POINT_HIST_SIZE     128         // number of points in Histogram
#define ATOM_VEC_SIZE       32          // number of points in dAtomVec and hAtomVec
#define RES_VEC_SIZE        128         // max number of points in dResVec & hResVec

#define CELL_SPACE          5           // grid spacing between cell centres in CellMap
#define CELL_LIFE_LIMIT    10000        // number of iterates a cell can survive even if not infected
#define CELL_FM_THRESH     100000.0f    // the FD metric for the threshold below which 1 cell can infect another
#define CELL_SEED_RATE     0.2f         // the fraction of cells seeded 

const float4 params = { VOL_NX / 4.f, VOL_NY / 6.f, VOL_NZ / 16.f, 1.f };




#endif  // MY_CONSTANTS_H
