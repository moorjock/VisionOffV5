//
// <VisionOffV5.h>
//
// This program uses pre-recorded camera data off-line for development
// of vision processing algorithms.
//
// Version      5.1
// Changes      Final tidy up for export to github.
// Date         21 July 2023
// Programmer   A. Shepherd
//
// Version      5.0
// Changes      Tidy up and removal of obsolete code paths.
// Date         16 May 2022
// Programmer   A. Shepherd
//
// Version      4.0
// Changes      Developed to investigate improvements to VisionOffV3 and in particular
//              reduction of lateral drift in the ICP registration process.
// Date         24 November 2021
// Programmer   A. Shepherd
//
// Version      3.0
// Changes      Developed to introduce PCA-computed surface normals and Point-to-Plane
//              Registration algorithm. Further code reorganisation and moved kernels
//              and GPU helpers to separate files.
// Programmer   A. Shepherd
//
// Version      2.0
// Changes      Signficant restructuring of the code with text-file control added
//              memory allocation moved to a new function and general re-organisation
// Programmer   A. Shepherd
//
// Version      1.0
// Changes      First off-line version created from VisionOn
// Programmer   A. Shepherd
//
//-------------------------------------------------------------------------------------
// Copyright (C) Alan Shepherd 2023
//-------------------------------------------------------------------------------------
//
// This program will only compile and run on a computer with a reasonably recent 
// NVIDIA GPU and the NVIDIA SDK also installed.
//-------------------------------------------------------------------------------------
//
#ifndef VISIONOFF_V5_H
#define VISIONOFF_V5_H


int LoadData(const char* pDataFile);

void FileUpdate();

int StartPipes();

int AllocateMemory();

int WriteFrameMatrix(UINT16* pData, const char* pMatName, const char* pFileName);

int ReadMatrix(UINT16* pData, int SZ, const char* pFileName);

void PCA_Reduce_Debug(float* hPointVec, int points, float* hWorkMat, int workMatStep, int* hMap, int* hAtomVec);

/*
extern float  *pointMat;
extern float  *pointVec; 
extern float  *refVec;
extern int    *ipVec;
extern float  *pointSav;
extern float  *rotVec;  
extern float  *virtVec;
extern float  *dWorkVec;           // scratch pad used in the kernels
extern float  *dWorkMat;          // storage per thread for covariance and other matrices produced by PCA_Reduce
extern float  *dResVec;
extern int   *dAtomVec;         // device-side Atomic parameters to be retrieved
extern int   *dMatch;
extern int   *dSaveMatch;
extern int *dMap;
extern int *dRefMap;
extern int *dSavPointMap;
extern int *dRefMap;
extern int *dSavRefMap;
extern int *dCellMap;
extern int *dSeedCells;

*/
extern float  *d_vol;                // pointer to device array for storing volume data

extern float  *hRotVec;
extern float  *hWorkMat;           // and used by Cell_Search
extern float  *hRefVec;
extern float  *hPointVec;        // host copy
extern float  *hWorkVec;
extern float  *hResVec;
extern int   *hAtomVec;         // host-side vector for holding device debug parameters
extern int   *hMatch;

extern int *hMap;
extern int *hRefMap;
extern int *hCellMap;
extern int *hSeedCells;       // vector containing list of seeded cells

extern float  *Hyst;
/*
extern UINT16 *b_Depth;         // unfiltered new depth array
extern UINT16 *c_Depth;         // pre-filtered depth array
extern UINT16 *o_Depth;         // old depth previous depth array
extern UINT16 *p_Depth;         // older depth previous previous depth array
extern float  *c_Dists;
*/
extern int FrameCount;
#endif
