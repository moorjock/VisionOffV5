//
// <ControlParams.h>
//
// This is a bit like a FORTRAN common block used to transfer
// parameters to functions without the need for call arguments
// This strategy may not work with GPU kernels but will
// work with Host code
//
#ifndef CONTROL_PARAMS_H
#define CONTROL_PARAMS_H

extern int ImageTest;
extern int RegMethod;
extern int RenderMethod;
extern float dist, theta, threshold;
extern int FPS;
extern int VideoOn, OuputOn, PlayBackOn, StepMode;

// cell reduction
extern int CellX, KNVecs, KNPoints, PreFilter1, PreFilter2, OverlapOn;

// bilateral filter constants
extern int   bilat_ksz;
extern float bilat_spatial_const;
extern float bilat_depth_const;

// Gradient Descent
extern float Grad_DX,  Grad_Alpha;
extern int   Grad_LIM, Grad_Min;

#endif
