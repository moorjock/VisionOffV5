//
// VisionOffV5.cpp See Header for details.
// This program uses pre-recorded camera data off-line for
// development of vision processing algorithms with a fixed baseline
// set of repeatable data, this is only part of the development process
//
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <unistd.h>
#include <pthread.h>

#include <thread>
#include <mutex> // std::mutex, std::lock_guard
#include <condition_variable>
#include <stdexcept> // std::logic_error
#include <queue>
#include <atomic>
#include <fstream>

#include <GL/glew.h>
#include <GL/freeglut.h>

#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include "../Subs/constants.h"
#include "../Subs/MyDataTypes.h"

#include "../Subs/FMat.h"

#include "../Subs/MyUnixSerial.h"
#include "../Subs/CommTypes.h"
#include "../Subs/MyTimer.h"

#include "../Subs/device_funcs.cuh"
#include "../Subs/GMat.cuh"

#include "kernel.h"
#include "P2P_ICP.cuh"
#include "P2Plane_ICP.cuh"
#include "Cell_Search.cuh"

#include "OpenGLFuncs.h"

#include "VisionOffV5.h"

#include "ControlParams.h"

using namespace cv;
using namespace std;

const char *MsgSrc = "VisionOffV5";

int HaveScreen = 1; // output to local console
int HavePipe = 0;   // input/output to carrage
int HaveWIFI = 0;   // input/output to track
int HaveDump = 1;   // output to local dump file

int Msg_Pipe_FD = 0;
int Msg_WIFI_FD = 0;

short RobSeqCmd::m_SeqIndex = 0;

cudaError_t CudaErr;
const char *pCudaError;

// texture and pixel objects
GLuint pbo = 0; // OpenGL pixel buffer object
GLuint tex = 0; // OpenGL texture object
struct cudaGraphicsResource *cuda_pbo_resource;

float *d_vol; // device array for 3D model volume

int2 arrFullSz; // size of camera array [640x480] for L525
int2 cellSz;    // cellular array size
int2 mapSz;     // size of downsampled imagemap
int cxn;        // X-dimension of cell array
int cyn;        // Y-dimension of cell array
int CellWorkStep;
/*
int2 matSz;       // sizes of pointMat
int matBytes;     // pointMat bytes
float *pointMat=NULL;  // matrix for storing cell array vertices
float *hPointMat=NULL; // host side copy
*/

int2 pntSz;      // sizes of pointVec and rotVec
int pointBytes;  // pointVec bytes
int pointCount;  // actual number of points in pointVec
int virtCount;
float *pointVec; // vector for storing processed frame vertices
int *ipVec;
float *pointSav;
float *refVec;    // reference vector
float *hRefVec;   // host side copy of refVec
float *hPointVec; // host side copy
float *rotVec;    // rotated pointVec
float *virtVec;   // derived from v_Depth
float *hRotVec;
float *subVec;
float *hSubVec;
float *subRef;
float *hSubRef;

int refCount;   // number of points in reference refVec
float3 refMean; // mean of reference points

cudaArray *ref_array;

int matchBytes;  // size of dMatch vector maximum [53*30*6]*sizeof(int) = [9540]*4
int MatchCount;  // count of point-pair matches
int *dMatch;     // index of matching vectors of pointVec in refVec
int *hMatch;     // host side copy
int *dSaveMatch; // saves a point-pair match

int workStep;    // the size of dWorkVec used per thread, this is a general scratch pad used by various
int workSz;      // be sure to check this against most severe code requirements
int workBytes;   // if the Segment size changes
float *dWorkVec; // device scratch pad used in the kernels
float *hWorkVec; // host side copy
float *dWorkMat; // storage per thread for covariance and other matrices produced by PCA_Reduce
float *hWorkMat; // and used by Cell_Search
int wMatStep;    // incremental size of dWorkMat used per thread
int wMatSize;
int wMatBytes;

int *hMap;
int *dMap;
int *dSavPointMap;
int *dRefMap;
int *hRefMap;
int *dSavRefMap;

int *dCellMap;
int *hCellMap;
int *hSeedCells;
int *dSeedCells;
int SeedCount;
int MapSeedLimit;

int MapSize;
int mapBytes;

int atomSize;
int *dAtomVec; // device-side atomic parameters
int *hAtomVec; // host side copy

int resSize;
float *dResVec; // results from device kernels
float *hResVec; // host side copy

UINT16 *b_Filt;  // depth array from RecordFile
UINT16 *b_Depth; // unfiltered depth array
UINT16 *c_Depth; // pre-filtered depth array
UINT16 *uhDepth; // for output of camera format data
float *d_Depth;  // conversion to actual depth in m for each pixel (used by PCA_reduce)
float *v_Depth;  // recovered depth data in m from d_vol
float *hDepth;   // host side depth from volume

uchar3 *pCV3Out = NULL;   // cuda memory pointer for OpenCV video output
uchar3 *pOutImage = NULL; // local host pointed for OpenCV video output

cudaArray *ppDarry;            // cudaArray to hold depth and probability data
cudaSurfaceObject_t DarrySurf; // the cuda Surface object

Look1Dc DepthTab;
float ProbTab[] = {0.735, 0.735, 0.735, 0.735, 0.7317, 0.6917, 0.6115, 0.5027, 0.4081, 0.3007, 0.2102, 0.1726, 0.15, 0.15, 0.15, 0.10, 0.05, 0.05};
float ProbTabDel = 0.2;
float ProbTabMin = 0.0;
int ProbTabSize = 18;

unsigned int *pPallet;
unsigned int m_Palette[MAX_COLOURS];
int colours;

// 3D view parameters
float zs = 9000;        // zdist; // distance from origin to source
float ys = 0.0f;
float xs = 0.0f;

int FPS = 30; // 30;   // frames per second Note only 15,30,60,90 FPS available on camera!
float dist = 0.f, thetaX = 0.f, thetaY = 0.f, threshold = 0.f;

int RenderMethod = 4; // 1 = volumeRender, 2 = slice, 3 = raycast 4 = MyRayShader 5=MySimpleShader
int ImageTest = 4;    // 1=2D, 2=3D, 3=3D+rotate, 4=3D+registration
int RegMethod = 1;    // Registration method 1=P2P_ICP, 2=P2L_ICP, 3=CellSearch
int FrameDump = 0;
int ClosedLoop = 0;
int CLStep = 1;
int CLStart = 10;
int CellX = 16;
int KNVecs = 1;
int KNPoints = 4;
int ICPFrames = 10;

int FrameCount = 0;
Cloud Ref;
Cloud Pnt;
FVec Pose;
FVec Pose1;
FVec Pose2;
FVec PoseErr;

// view control flags
int VideoOn = 0;
int OutputOn = 1;   // turn on/off printout of time data to console
int PlaybackOn = 0; // Playback of recorded screen images from data file
int StepMode = 1;

// bilateral filter parameters
int bilat_ksz = 11;               // pixels
float bilat_spatial_const = 6.0;  // pixels
float bilat_depth_const = 2000.0; // mm
float ssinv2 = 0.5 / (bilat_spatial_const * bilat_spatial_const);
float sdinv2 = 0.5 / (bilat_depth_const * bilat_depth_const);
float CropDepth;
int ZeroOut = 0;

float CurveThresh = 0.02;
float DistMin = 0.5;
float CosMin = 0.95;
float FMThresh = 1.0;
float DetScale = 1.0;
int PreFilter1 = 1; // switch bilateral prefiltering on/off This will filter all ImageTest types
int PreFilter2 = 0; // switch depth surface filtering on/off only applies to selected ImageTests
// internal states
int VolumeInit = 1;
int Stopped = 0;
int UpdateView = 1;

// timing
float Time = 0.0;
float GPU_elapsedTime;
// video recording (do we need this?)
Mat src = Mat::zeros(CAM_W, CAM_W, CV_8UC4);
VideoWriter writer;
string VideoFile; // name of the output video file

FString HeadString;
FString FrameNbr;
FString FrameStr;
FString PaletFile;
FString RecordFile;

FString DumpFile;
FILE *pDmpFile = NULL;

FString ResultsFile;
FILE *pResFile = NULL;

FString FrameFileName;
FString FrameFile;
FILE *pFrmFile = NULL;

// servo parameters in the recording file
#define FileDataInCount 14
float InVec[FileDataInCount];
FVec ThetaCB(3);
FVec ThetaDem(3);
FVec XCG(3);
FVec LCB(3);
FVec ThetaGB(3);
FVec XGB(3);
FVec RigPose(6);

float3 xgbf;
float3 CamL;
GBHMat TCB;
GBHMat TCG;
GBHMat TCG2;
GBHMat SaveTCG;
GBHMat dTCB;
GBHMat AltTCG;

FMat DetMap;

int nullcells, vecCount, threshFail, vecFail, overShoots, mapIdx, totalCells;

int Fore, Turn, Left, Right, Top, Bot;
int XLR, XRR, XTR, XBR, XLP, XRP , XTP, XBP;


int ii, jj, im, pointIdx, subCount, subRefCount, refIdx;
float Val;

float *pWorkSt = NULL;
FMat C(pWorkSt, 3, 3);
FMat V(pWorkSt, 3, 3);
FMat AM1(pWorkSt, 3, 3);
FMat W(pWorkSt, 3, 3);
FVec Lambda(pWorkSt, 3);
FVec SLambdaM1(pWorkSt, 3);

// thead for reading/writing the imagefile
std::mutex Global_mtx;
std::condition_variable_any condvar;

bool ready = false;
int FrameRecord = 0;
size_t DiskFileSize = 0;
size_t OpenFileSize = 0;
size_t FrameSize = 0;

// datafile reading/writing
std::atomic_bool UpdateFile(false);
Serial DiskFile;
// binary data file output
DepthDataComm DepthIn;
#define DATAINITEMS 14
FloatDataComm DataIn;
float OutVec[DATAINITEMS];

// this duality has to stop!
CommString MatName;
IntDataComm MatSz;
Serial MatFileOut;    // the file object
DepthDataComm MatOut; // the CommType

Serial MatFileIn;
DepthDataComm MatIn;
SHORTI SzVec[2] = {0};
MyTimer Clock;

float ViewScale = CAM_D2M; // warning used many times with different values

// currently the threading is disabled for single stepping
// It would be good to be able to do both
GBHMat RigRot;

void FileUpdate()
{
//
// This function should run in a std::thread and
// read in frame data from the already open file.
// CommIn, DataIn and DepthIn must be initialised
// to a correctly opened disk file before this is called
//
    int Index, iOpt;
    FString Word;
    static char head[6];
    int countBytes = 0;

//###    while(!Stopped && ((DiskFileSize+FrameSize)<OpenFileSize) ){

    usleep(33333); // normal camera frame rate is 30FPS = 33333 usec

//###        if(PlaybackOn && UpdateView){        // && !UpdateFile){
//###            Global_mtx.lock();
    countBytes = 0;
//###            printf(" In FileUpdate() and Reading \n");
// frame
    DiskFile.readBytes(head, 6);
    CommIn.DecodeHeader(head);
    iOpt = CommIn.GetOpt();
    DiskFile.readBytes(TextBuf, iOpt);
    Index = 0;
    FindNextWord(TextBuf, Word, Index);
    FindNextWord(TextBuf, Word, Index);
    FindNextWord(TextBuf, Word, Index);
    FrameRecord = atoi(Word);
    // headstring
    DiskFile.readBytes(head, 6);
    CommIn.DecodeHeader(head);
    iOpt = CommIn.GetOpt();
    HeadString.ReSize(iOpt);
    DiskFile.readBytes(TextBuf, iOpt);

//            HeadString = TextBuf;
// datain
    DiskFile.readBytes(head, 6);
    CommIn.DecodeHeader(head);
    iOpt = CommIn.GetOpt();
    countBytes += DataIn.Receive();
//
// Output from test rig XCG are body axis relative to rig
// XGB are global parameters (for use with a moving robot)
//
    Time = InVec[0];
    ThetaCB(0) = InVec[2];  // Servo Pitch
    ThetaCB(1) = InVec[4];  // servo Yaw
    ThetaCB(2) = 0.0;       // roll=0
    ThetaDem(0) = InVec[1]; // Demand Pitch
    ThetaDem(1) = InVec[3]; // demand Yaw
    ThetaDem(2) = 0.0;      // demand Roll = 0
//###
    XCG(1) = CamL.y * cosf(ThetaCB(0)) - CamL.z * sinf(ThetaCB(0));                    // this is a mess it should be done
    XCG(2) = CamL.y * sinf(ThetaCB(0)) + CamL.z * cosf(ThetaCB(0)) * cosf(ThetaCB(1)); // on the rig where the data is
    XCG(0) = XCG(2) * sinf(ThetaCB(1)) + CamL.x;                                       // generated
    XCG(2) += InVec[7]; // = XCG[2] = TrackPPos
//####
    XCG.ZeroInit(3);    // This works and it is more general
    RigRot.InitMatrix(ThetaCB,XCG);
    XCG = RigRot.InvRotateH(LCB);
    XCG(2) += InVec[5]/1000.0;   // for ServoTrack step-mode tests
//##    XCG(2) += InVec[7];            // for ServoTrack real-time tests
//####
    ThetaGB(0) = InVec[8];      // global pitch // moving robot
    ThetaGB(1) = InVec[9];      // global Yaw   // moving Robot
    ThetaGB(2) = InVec[10];     // global roll  // moving Robot
    XGB(0) = InVec[11];         // global X position
    XGB(1) = InVec[12];         // global Y position
//    XGB(2) = InVec[5]/1000.0;   // for ServoTrack step-mode tests
    XGB(2) = InVec[7];          // for ServoTrack real-time tests

    // depth data
    DiskFile.readBytes(head, 6); // Depth head
    DepthIn.SetData(b_Filt, CAM_SIZE);
    countBytes += DepthIn.Receive(); // depth data
    DiskFileSize += countBytes;
    FrameSize = countBytes;
    UpdateFile = true;

//###  File data Outputs
    fprintf(pDmpFile,"\n Frame, Time,  Thdem, Pitch, Yawdem,  Yaw,   Xcb,  Ycb,   Zcb,   XGB,   YGB,   ZGB,   Tcbx,  Tcby,  Tcbz,\n");
//###

    fprintf(pDmpFile,"  %d,  %0.3f, ",FrameRecord,Time);

    for (int i = 1; i < 5; i++)
    {
        fprintf(pDmpFile," %.3f,", InVec[i] * DPR);
    }
    for (int i = 0; i < 3; i++)
    {
        fprintf(pDmpFile," %.3f,", XCG(i));
    }
    for (int i = 0; i < 3; i++)
    {
        fprintf(pDmpFile," %.3f,", XGB(i));
    }
    for (int i = 0; i < 3; i++)
    {
        fprintf(pDmpFile," %.3f,", ThetaCB(i)*DPR);
    }

    fprintf(pDmpFile,"\n");

}
//
//========================================================================================
//
int main(int argc, char **argv)
{
//
// declare variables only accessible in main
//
    cudaEvent_t GPU_start, GPU_stop;
    float3 dispb, dispg, ddispg;

    float initVal = 0.0;
    int PrintCount = 0;
    int FirstFileInput = 1;
    const char *pDataFile = "../Data/VisionOffV5Control.txt";
    int i, j, k;
    int CLTrip=0;
//
// Load the run configuration data
//
    if (LoadData(pDataFile) < 0)
    {
        printf("Failed LoadData\n");
        return -1;
    }
//
// allocate host and GPU memory
//
    if (AllocateMemory() < 0)
    {
        printf("Failed AllocateMemory\n");
        return -1;
    }
//
// load the false colour palette for rendering
//
    colours = LoadPaletteFile(PaletFile);
    if (colours < 0)
    {
        printf("Failed LoadPaletteFile\n");
        return -1;
    }
    cudaMemcpy(pPallet, m_Palette, colours * sizeof(unsigned int), cudaMemcpyHostToDevice);
//
// Tracking Results File
//
    pResFile = fopen(ResultsFile, "w+");
//
// Output diagnostics
//
    pDmpFile = fopen(DumpFile, "w+");
//
// One-time output of key data and header
//
    if (pResFile)
    {
//        fprintf(pResFile, " VisionOffV5, ImageTest, %d, RegMethod, %d, ClosedLoop, %d, RenderMethod, %d, PlaybackFile, %s \n", ImageTest, RegMethod, ClosedLoop, RenderMethod, (const char *)RecordFile);
//        fprintf(pResFile, " Cell Size, %d, KNpoints, %d, PreFilter1, %d, PreFilter2, %d,\n", CellX, KNPoints, PreFilter1, PreFilter2);
        if (PreFilter1 == 1)
        {
            fprintf(pResFile, " Bilat_Ksz, %d, space_const, %f, Depth_const, %f\n", bilat_ksz, bilat_spatial_const, bilat_depth_const);
        }
        else if (PreFilter1 == 2)
        {
            fprintf(pResFile, " CropDepth, %.5f m, ZeroOut, %d,\n", CropDepth, ZeroOut);
        }
        if(ImageTest==4)
        {
            switch(RegMethod)
            {
                case 1:
                    fprintf(pResFile, " Frame, Time, Tx, Ty, Tz, Dx, Dy, Dz, Txd, Tyd, Tzd, |, ICP_Err, PointCount, MatchCount, LoopCount, |, pTx, pTy, pTz, pDx, pDy, pDz, |errors, errTx, errTy, errTz, errDx, errDy, errDz, GPU_Delta,\n");
                break;
                case 2:
                    fprintf(pResFile, " Frame, Time, Tx, Ty, Tz, Dx, Dy, Dz, Txd, Tyd, Tzd, |, ICP_Err, PointCount, MatchCount, LoopCount, |pose, pTx, pTy, pTz, pDx, pDy, pDz, |errors, errTx, errTy, errTz, errDx, errDy, errDz, GPU_Delta,\n");
                break;
                case 3:
                    fprintf(pResFile, " Frame, Time, Tx, Ty, Tz, Dx, Dy, Dz, Txd, Tyd, Tzd, |, ICP_Err, PointCount, MatchCount, MatchFM, LoopCount, |pose, pTx, pTy, pTz, pDx, pDy, pDz, |errors, errTx, errTy, errTz, errDx, errDy, errDz, GPU_Delta,\n");
                break;
                default:
                    fprintf(pResFile, " Method Not Available\n");
            }
        } else if(ImageTest==5){
            fprintf(pResFile, " Frame, Time, Tx, Ty, Tz, Dx, Dy, Dz, Txd, Tyd, Tzd, |, ICP_Err, pointCount, MatchCount, LoopCount, |pose1, pTx1, pTy1, pTz1, pDx1, pDy1, pDz1, |, ICP_Err2, pointCount2, MatchCount2, LoopCount2, |pose2, pTx2, pTy2, pTz2, pDx2, pDy2, pDz2, |errors, errTx, errTy, errTz, errDx, errDy, errDz, GPU_Delta,\n");
        } else if(ImageTest==6){
            fprintf(pResFile, " Frame, Time, Tx, Ty, Tz, Dx, Dy, Dz, Txd, Tyd, Tzd, |, ICP_Err, PointCount, MatchCount, LoopCount, |pose, pTx, pTy, pTz, pDx, pDy, pDz, |errors, errTx, errTy, errTz, errDx, errDy, errDz, GPU_Delta,\n");
        }
    }
//
// Camera depth probability lookup table
//
    int ret = DepthTab.SetTable(ProbTab, ProbTabSize, ProbTabDel, ProbTabMin);
    if (ret > 0)
    {
        printf("Error Creating Depth Lookup Table = %d\n", ret);
        return 0;
    }
/*
//
// OpenCV Recording of Output
//
    pOutImage = (uchar3 *)calloc(CAM_SIZE, sizeof(char3));
    cudaMalloc(&pCV3Out, CAM_SIZE * sizeof(uchar3));
    int codec = VideoWriter::fourcc('M', 'J', 'P', 'G'); // select desired codec
    bool isColor = true;
    writer.open(VideoFile, codec, FPS, Size(CAM_W, CAM_H), isColor);

    if (VideoOn && !writer.isOpened())
    {
        printf("Could not open the OpenCV video file for write\n");
        return -1;
    }
*/
    Clock.StartTimer();
    RigRot.InitMatrix();
    AltTCG.InitMatrix();
//
// Depth Camera Data RecordFile
//
    int diskFD = DiskFile.OpenFile(2, RecordFile);
    if (diskFD <= 0)
    {
        printf("failed to open RecordFile %s\n", (const char *)RecordFile);
        return -1;
    }
    struct stat fs;
    fstat(diskFD, &fs);
    OpenFileSize = fs.st_size; // use the actual size

    TextIn.SetCommsPort(&DiskFile);
    DataIn.SetCommsPort(&DiskFile);
    DataIn.SetData(InVec, FileDataInCount);
    DepthIn.SetCommsPort(&DiskFile);
//
// initialise memory elements ahead of file reading
// and before kicking off the FileThread
//
    ThetaGB.ZeroInit(3);
    RigPose.ZeroInit(6);

    xgbf = make_float3(XGB(0), XGB(1), XGB(2)); // locates the camera in the view volume

    SaveTCG.InitMatrix();
    dTCB.InitMatrix();

    TCG.InitMatrix(ThetaGB, ThetaGB); // simply initialise to zero the displacement is computed
    TCG2.InitMatrix(ThetaGB, ThetaGB);

    ThetaCB.ZeroInit(3);

    TCB.InitMatrix(ThetaCB, XCG);
    TCB.printMatrix(" Body Position in Camera Axes\n", NULL);
//
    UpdateFile = false;
//
    printInstructions();
    initScreen(&argc, argv);
    gluOrtho2D(0, SCREEN_W, SCREEN_H, 0); // standard screen format: Left=0,Right=SCREEN_W, Bottom=SCREEN_H Top=0;
                                          //
                                          // register openGL call-back functions
                                          // used in glutMainLoop()
                                          //
    glutKeyboardFunc(keyboard);
    glutSpecialFunc(handleSpecialKeypress);
    glutDisplayFunc(display); // this is the key display driver

    initPixelBuffer();
    //
    Pose.ZeroInit(6);
    PoseErr.ZeroInit(6);
    Pose2.ZeroInit(6);
    Pose1.ZeroInit(6);
//
// Main processing loop initialisation
//
    refCount = 0;
//
// This is the main processing loop it takes in camera position data from the
// the recorded datafile and plays it back either by stepping through it
// or at a frame rate of Approx FPS
//
    do
    {
//
// we re-initialise the view volume d_vol as a one-shot after every change of ImageTest
//
        if (VolumeInit)
        {
            if (ImageTest > 1)
            {
                initVal = 0.0;
                RenderMethod = 5;
            }
            else
            {
                initVal = 1.0;
                RenderMethod = 4;
            }
            Init_Volume_Launcher(d_vol, volumeSize, initVal);
            cudaMemset(b_Depth, 0, CAM_BYTES);
            cudaDeviceSynchronize();
            VolumeInit = 0;
            printf("Volume Initialised ImageTest %d\n", ImageTest);

        }

        if (PlaybackOn && UpdateView)
        {
//
            FileUpdate();
//
// PreFiltering
//
            switch (PreFilter1)
            {
            case 1: // Bilateral
                cudaMemcpy(b_Depth, b_Filt, CAM_BYTES, cudaMemcpyHostToDevice);
                bilateralFilterLauncher(b_Depth, c_Depth, arrFullSz, bilat_ksz, ssinv2, sdinv2);
                cudaDeviceSynchronize();
                break;

            case 2: // Crop Filter
                cudaMemcpy(b_Depth, b_Filt, CAM_BYTES, cudaMemcpyHostToDevice);
                ViewScale = CAM_D2M;
                CropFilterLauncher(b_Depth, c_Depth, arrFullSz, cellSz, ViewScale, CropDepth, ZeroOut);
                cudaDeviceSynchronize();
                cudaMemcpy(b_Filt, c_Depth, CAM_BYTES, cudaMemcpyDeviceToHost);

                break;

            default: // Copy b_filt directly to c_Depth
                cudaMemcpy(c_Depth, b_Filt, CAM_BYTES, cudaMemcpyHostToDevice);
                break;
            }

            PrintCount++;

// Using *.bin file data
            for (int i = 0; i < 3; i++)
            {
                RigPose(i) = ThetaCB(i);
                RigPose(i + 3) = XCG(i);
            }
//
            if (OutputOn && PrintCount == 1)
            {
                if (FirstFileInput == 0)
                { // skip the first
                    fprintf(pResFile, " %d, %0.5f, ", FrameRecord, Time);
                    for (int i = 0; i < 3; i++){
                        fprintf(pResFile, " %0.5f,", RigPose(i)*DPR);
                    }
                    for (int i = 3; i < 6; i++){
                        fprintf(pResFile, " %0.5f,", RigPose(i));
                    }

                    fprintf(pResFile, " %.5f, %.5f, %.5f, ", ThetaDem(0) * DPR, ThetaDem(1) * DPR, ThetaDem(2) * DPR);
                }
                PrintCount = 0;
            }
            if (FirstFileInput > 0)
            {
//
// TCB matrix is initialised from the first data file item
// TCG is also initialised to this one coincident point
//
                TCB.InitMatrix(ThetaCB, LCB);
                SaveTCG = TCB; // save the original pose
                cudaEventCreate(&GPU_start);
                cudaEventCreate(&GPU_stop);
            }

            switch (ImageTest)
            {
            case 1: // simple camera image projection
                ViewScale = CAM_D2M / VOX_RES;

                fprintf(pDmpFile, " Source Data, %s, PreFilter=%d, \n", (const char *)RecordFile, PreFilter1);
                if (PreFilter1)
                {
                    fprintf(pDmpFile, "\n Bilateral, Params,  depthc=%.5f, spacec=%.5f, \n", bilat_depth_const, bilat_spatial_const);
                }
                printf(" FrameRecord %d,\n",FrameRecord);

                DepthSurface_Launcher(c_Depth, DarrySurf, arrFullSz, ViewScale, DepthTab, PreFilter2);
                cudaDeviceSynchronize();

                DarryVolumeKernelLauncher(d_vol, DarrySurf, arrFullSz, volumeSize);
                cudaDeviceSynchronize();

                break;
            case 2:                  // 3D projection with Transformation
                ViewScale = CAM_D2M; // camera output to metres
                DepthSurface_Launcher(c_Depth, DarrySurf, arrFullSz, ViewScale, DepthTab, PreFilter2);
                cudaDeviceSynchronize();

// for this mode drive with the servo position data
                TCG.InitMatrix(ThetaCB, LCB);
                printf(" FrameRecord, %d, ",FrameRecord);
                TCG.printPose(" TCG, ", 1, NULL);
                ViewScale = 1.0 / VOX_RES; // metres to voxels
                volume3DKernel_Launcher(d_vol, volumeSize, DarrySurf, arrFullSz, TCG, xgbf, ViewScale);
                cudaDeviceSynchronize();
                break;
            case 3:                  // PCA_Reduce Testing
                ViewScale = CAM_D2M; // camera units to metres
                Depth_to_Float_Launcher(c_Depth, d_Depth, arrFullSz, ViewScale);
                cudaMemset(dAtomVec, 0, ATOM_VEC_SIZE * sizeof(int));
                cudaMemset(dWorkMat, 0, wMatBytes);
                cudaMemset(dMap, 0, mapBytes);
                PCA_Reduce_Launcher(d_Depth, arrFullSz, cellSz, dMap, mapSz, pointVec, pntSz, dWorkMat, wMatStep, ViewScale, CurveThresh, DetScale, dAtomVec);

// back-copy for debug
                cudaMemcpy(hPointVec, pointVec, pointBytes, cudaMemcpyDeviceToHost);
                cudaMemcpy(hAtomVec, dAtomVec, ATOM_VEC_SIZE * sizeof(int), cudaMemcpyDeviceToHost);
                cudaMemcpy(hWorkMat, dWorkMat, wMatBytes, cudaMemcpyDeviceToHost);
                cudaMemcpy(hMap, dMap, mapBytes, cudaMemcpyDeviceToHost);

                pointCount = hAtomVec[0];
                cudaMemset(dAtomVec, 0, ATOM_VEC_SIZE * sizeof(int));
                PCA_Surface_Launcher(DarrySurf, arrFullSz, cellSz, dMap, mapSz, pointVec, pointCount, pntSz, dAtomVec);
                cudaMemcpy(hAtomVec, dAtomVec, ATOM_VEC_SIZE * sizeof(int), cudaMemcpyDeviceToHost);
//###
                printf("## FrameRecord %d pointCount %d, AtomVec [0] %d, [1] %d, [2] %d,\n", FrameRecord, pointCount, hAtomVec[0], hAtomVec[1], hAtomVec[2]);
//###
                TCG.InitMatrix(ThetaCB, LCB);
                TCG.printPose(" TCG ", 1, NULL);
                ViewScale = 1.0 / VOX_RES; // metres to voxels
                volume3DKernel_Launcher(d_vol, volumeSize, DarrySurf, arrFullSz, TCG, xgbf, ViewScale);
                cudaDeviceSynchronize();

                break;
            case 4:                  // open loop ICP integration by 3 methods P2P, P2L and CellSearch
                cudaEventRecord(GPU_start, 0);

                ViewScale = CAM_D2M; // camera units to metres
                DepthSurface_Launcher(c_Depth, DarrySurf, arrFullSz, ViewScale, DepthTab, PreFilter2);
                //

                Depth_to_Float_Launcher(c_Depth, d_Depth, arrFullSz, ViewScale);

                cudaMemset(dAtomVec, 0, ATOM_VEC_SIZE * sizeof(int));
                cudaMemset(dWorkMat, 0, wMatBytes);
                cudaMemset(dMap, 0, mapBytes);
                PCA_Reduce_Launcher(d_Depth, arrFullSz, cellSz, dMap, mapSz, pointVec, pntSz, dWorkMat, wMatStep, ViewScale, CurveThresh, DetScale, dAtomVec);

// back-copy for debug
                cudaMemcpy(hPointVec, pointVec, pointBytes, cudaMemcpyDeviceToHost);
                cudaMemcpy(hAtomVec, dAtomVec, ATOM_VEC_SIZE * sizeof(int), cudaMemcpyDeviceToHost);
                cudaMemcpy(hWorkMat, dWorkMat, wMatBytes, cudaMemcpyDeviceToHost);
                cudaMemcpy(hMap, dMap, mapBytes, cudaMemcpyDeviceToHost);

// pointCount is essential output (not debug)
                pointCount = hAtomVec[0];
//###
//              PCA_Reduce_Debug(hPointVec, hWorkMat, wMatBytes, hMap, hAtomVec);
//###
                TCG = SaveTCG;
                if (refCount <= 0){
//
// The first frame is copied to refVec
                    refCount = pointCount;
//>>>
                    cudaMemcpy(refVec, pointVec, pointBytes, cudaMemcpyDeviceToDevice);
//<<<
                    FrameCount = 0;
                } else {
                    switch (RegMethod)
                    {
                    case 1: // Point to Point ICP

                        P2P_ICP2_Launcher(pointVec, pntSz, pointCount, ipVec, rotVec,
                                          refVec, refCount, dMatch, MatchCount,
                                          ICPFrames, dWorkVec, dResVec, DistMin, CosMin, FrameRecord,
                                          TCG, xgbf, dTCB, dAtomVec, pDmpFile, pResFile);
                        break;
                    case 2: // point to plane

                        P2Plane_ICP_Launcher(pointVec, pntSz, pointCount, ipVec, rotVec, refVec, 
                                             refCount, dMatch, MatchCount, ICPFrames,
                                             dWorkVec, CellWorkStep, dResVec, DistMin, CosMin, FrameRecord,
                                             TCG, xgbf, dTCB, dAtomVec, pDmpFile, pResFile);
                        break;
                    case 3: // Cell-matching search based on Forstner and Moonen metric

                        CellSearch_Launcher(d_Depth, arrFullSz, cellSz, dMap, mapSz, 
                                            pointVec, pntSz, pointCount, ipVec, refVec, refCount,
                                            rotVec, dMatch, matchBytes, MatchCount, ICPFrames,
                                            dWorkVec, CellWorkStep, dWorkMat, wMatStep, 
                                            DistMin, CosMin, FMThresh, FrameRecord,
                                            TCG, xgbf, dTCB, 
                                            dResVec, dAtomVec, pDmpFile, pResFile);

                        break;
                    default:
                        printf(" RegMethod %d Not Implemented\n", RegMethod);
                        break;
                    }

                    TCG2.InitMatrix();

//###
//                    if(FrameRecord>99 && FrameRecord < 121){
//                        dTCB.printMatrix("dTCB",pDmpFile);
//                    }
//###

                    for(i=0;i<3;i++){
                        for(j=0;j<3;j++){
                            for(k=0;k<3;k++){
                                TCG2(i,j) = TCG2(i,j) + TCG(i,k)*dTCB(k,j);  // increment the rotation matrix
                            }
                        }
                    }
                    TCG2(3,3) = 1.0;
                    TCG2.UpdateGPU();

// get back the delta displacements
                    dispb  = dTCB.GetDisplacement();
                    ddispg = TCG.InvRotateH(dispb);
                    dispg  = TCG.GetDisplacement();

                    dispg.x = dispg.x + ddispg.x;
                    dispg.y = dispg.y + ddispg.y;
                    dispg.z = dispg.z + ddispg.z;

                    TCG = TCG2;

                    TCG.SetDisplacement(dispg);

                    TCG.printPose(" Pose, ", 0, pResFile);
                    Pose = TCG.GetPose();
                    Pose2 = Pose;
// output pose errors between rig sensor data and P2L computed pose

                    fprintf(pResFile, " errors, ");
                    for (i = 0; i < 3; i++)
                    {
                        PoseErr(i) = (RigPose(i) - Pose(i) ) * DPR;
                        fprintf(pResFile, " %0.5f,", PoseErr(i));
                    }
                    for (i = 3; i < 6; i++)
                    {
                        PoseErr(i) = RigPose(i) - Pose(i);
                        fprintf(pResFile, " %0.5f,", PoseErr(i));
                    }
//
// update the reference Array it is now pointVec
// may want to threshold this reference update as a means to counteract drift
//
                    FrameCount++;
                    if (FrameCount >= ICPFrames)
                    {
                        cudaMemcpy(refVec, pointVec, pointBytes, cudaMemcpyDeviceToDevice);
                        FrameCount = 0;
                        Ref.Count = pointCount;
                        SaveTCG = TCG;
                    }
                }
//
                ViewScale = 1.0 / VOX_RES; // metres to voxels
                TCG.printPose(" TCG before vol3D", 1, NULL);

// this trap removes the first few frames to prevent a startup issue with ServoTrack
               if(FrameRecord<10 && fabs(ThetaCB(0))<1.0E-5){
                    // skip
                } else {
                    volume3DKernel_Launcher(d_vol, volumeSize, DarrySurf, arrFullSz, TCG, xgbf, ViewScale);
                }
                cudaEventRecord(GPU_stop, 0);
                cudaEventSynchronize(GPU_stop);
                cudaEventElapsedTime(&GPU_elapsedTime, GPU_start, GPU_stop);
                if(FirstFileInput){
                    FirstFileInput = 0;
                } else {
                    fprintf(pResFile, " %0.5f,",GPU_elapsedTime);
                    fprintf(pResFile, "\n");
                }
                break;
//
//========== case 5 =====================================================================================
//
            case 5: // point-to-plane with loop closure
                cudaEventCreate(&GPU_start);
                cudaEventCreate(&GPU_stop);
                cudaEventRecord(GPU_start, 0);

                ViewScale = CAM_D2M; // camera units to metres
                DepthSurface_Launcher(c_Depth, DarrySurf, arrFullSz, ViewScale, DepthTab, PreFilter2);

// convert c_Depth to float d_Depth
                Depth_to_Float_Launcher(c_Depth, d_Depth, arrFullSz, ViewScale);
//
// First part compute PCA decomposition d_Depth->pointVec
//
                cudaMemset(dAtomVec, 0, ATOM_VEC_SIZE * sizeof(int));
                cudaMemset(dWorkMat, 0.0, wMatBytes);
                cudaMemset(dMap, 0, mapBytes);
                PCA_Reduce_Launcher(d_Depth, arrFullSz, cellSz, dMap, mapSz, pointVec, pntSz, dWorkMat, wMatStep, ViewScale, CurveThresh, DetScale, dAtomVec);

                cudaMemcpy(hPointVec, pointVec, pointBytes, cudaMemcpyDeviceToHost);
                cudaMemcpy(hAtomVec, dAtomVec, ATOM_VEC_SIZE * sizeof(int), cudaMemcpyDeviceToHost);
                cudaMemcpy(hDepth, d_Depth, CAM_SIZE * sizeof(float), cudaMemcpyDeviceToHost);
                pointCount = hAtomVec[0];

                TCG = SaveTCG;
                if (refCount <= 0){
//
// The first frame is copied to refVec
                    refCount = pointCount;
//>>>
                    cudaMemcpy(refVec, pointVec, pointBytes, cudaMemcpyDeviceToDevice);
//<<<
                    FrameCount = 0;
                    goto jmp1;
                } 
//
// Compute TCG update based on TCB update
                P2Plane_ICP_Launcher(pointVec, pntSz, pointCount, ipVec, rotVec, refVec, 
                                     refCount, dMatch, MatchCount, ICPFrames,
                                     dWorkVec, CellWorkStep, dResVec, DistMin, CosMin, FrameRecord,
                                     TCG, xgbf, dTCB, dAtomVec, pDmpFile, pResFile);

                TCG2.InitMatrix();

                for(i=0;i<3;i++){
                    for(j=0;j<3;j++){
                        for(k=0;k<3;k++){
                            TCG2(i,j) = TCG2(i,j) + TCG(i,k)*dTCB(k,j);  // increment the rotation matrix
                        }
                    }
                }

                TCG2(3,3) = 1.0;
                TCG2.UpdateGPU();

// get back the delta displacements
                dispb  = dTCB.GetDisplacement();
                ddispg = TCG.InvRotateH(dispb);
                dispg  = TCG.GetDisplacement();

                dispg.x = dispg.x + ddispg.x;
                dispg.y = dispg.y + ddispg.y;
                dispg.z = dispg.z + ddispg.z;

                TCG = TCG2;

                TCG.SetDisplacement(dispg);

                TCG.printPose(" Pose 1, ", 0, pResFile);
                Pose1 = TCG.GetPose();
                Pose2.ZeroInit(6);
                CLTrip=0;

                if (ClosedLoop && (FrameRecord % CLStep == 0) && (FrameRecord>=CLStart) )
                {
                    CLTrip=1;
//
// get a depth image v_Depth back from the volume model d_vol at TCG
//
                    ViewScale = 1.0 / VOX_RES; // metres to voxels
                    cudaMemset(dResVec, 0, RES_VEC_SIZE * sizeof(float));
                    Depth_From_Vol_Launcher(d_vol, volumeSize, v_Depth, arrFullSz, TCG, xgbf, ViewScale, dResVec);

                    if (FrameDump){
                        IndexFileName(FrameFile, ".bin", FrameFileName, FrameRecord);
                        ViewScale = CAM_D2M;
                        Float_to_Depth_Launcher(v_Depth, b_Depth, arrFullSz, ViewScale);
                        cudaMemcpy(uhDepth, b_Depth, CAM_BYTES, cudaMemcpyDeviceToHost);
                        WriteFrameMatrix(uhDepth, "v_Depth\n", FrameFileName);
                        ViewScale = 1.0 / VOX_RES; // metres to voxels
                    }

                    cudaMemset(dAtomVec, 0, ATOM_VEC_SIZE * sizeof(int));
                    cudaMemset(dWorkMat, 0, wMatBytes);
                    cudaMemset(dMap, 0, mapBytes);

// get the virtVec from v_Depth
                    ViewScale = CAM_D2M; // camera units to metres

                    PCA_Reduce_Launcher(v_Depth, arrFullSz, cellSz, dMap, mapSz, virtVec, pntSz, dWorkMat, wMatStep, ViewScale, CurveThresh, DetScale, dAtomVec);

                    cudaMemcpy(hRefVec, virtVec, pointBytes, cudaMemcpyDeviceToHost);
                    cudaMemcpy(hAtomVec, dAtomVec, ATOM_VEC_SIZE * sizeof(int), cudaMemcpyDeviceToHost);
                    cudaMemcpy(hMap, dMap, mapBytes, cudaMemcpyDeviceToHost);
                    cudaMemcpy(hWorkMat, dWorkMat, wMatBytes, cudaMemcpyDeviceToHost);

                    virtCount = hAtomVec[0];

                    P2Plane_ICP_Launcher(pointVec, pntSz, pointCount, ipVec, rotVec, virtVec, 
                                         virtCount, dMatch, MatchCount, ICPFrames,
                                         dWorkVec, CellWorkStep, dResVec, DistMin, CosMin, FrameRecord,
                                         TCG2, xgbf, dTCB, dAtomVec, pDmpFile, pResFile);

                    TCG2.InitMatrix();

                    for(i=0;i<3;i++){
                        for(j=0;j<3;j++){
                            for(k=0;k<3;k++){
                                TCG2(i,j) = TCG2(i,j) + TCG(i,k)*dTCB(k,j);  // increment the rotation matrix
                            }
                        }
                    }
                    TCG2(3,3) = 1.0;
                    TCG2.UpdateGPU();

// get back the delta displacements
                    dispb  = dTCB.GetDisplacement();
                    ddispg = TCG.InvRotateH(dispb);
                    dispg  = TCG.GetDisplacement();

                    dispg.x = dispg.x + ddispg.x;
                    dispg.y = dispg.y + ddispg.y;
                    dispg.z = dispg.z + ddispg.z;

                    TCG = TCG2;

                    TCG.SetDisplacement(dispg);

                    SaveTCG = TCG;
                } else {
                    float nowt = 0.0;
                    fprintf(pResFile," |, %0.5f, %0.5f, %0.5f, %0.5f, ",nowt,nowt,nowt,nowt);
                }

                TCG.printPose(" Pose 2, ", 0, pResFile);


// output pose errors between rig sensor data and P2L computed pose
                Pose2 = TCG.GetPose();
/*
                fprintf(pResFile, " errors, ");
                for (i = 0; i < 3; i++)
                {
                    PoseErr(i) = (RigPose(i) - Pose(i)) * DPR;
                    fprintf(pResFile, " %0.5f,", PoseErr(i));
                }
                for (i = 3; i < 6; i++)
                {
                    PoseErr(i) = RigPose(i) - Pose(i);
                    fprintf(pResFile, " %0.5f,", PoseErr(i));
                }
*/
//###
                fprintf(pResFile, " poseDiff, ");
                for (i = 0; i < 3; i++)
                {
                    PoseErr(i) = (Pose1(i) - Pose2(i)) * DPR;
                    fprintf(pResFile, " %0.5f,", PoseErr(i));
                }
                for (i = 3; i < 6; i++)
                {
                    PoseErr(i) = Pose1(i) - Pose2(i);
                    fprintf(pResFile, " %0.5f,", PoseErr(i));
                }

//###
//
// update the reference Array it is now pointVec
// may want to threshold this reference update as a means to counteract drift
//
                FrameCount++;
                if (FrameCount >= ICPFrames )
                {
                    cudaMemcpy(refVec, pointVec, pointBytes, cudaMemcpyDeviceToDevice);
                    FrameCount = 0;
                    refCount = pointCount;
                    SaveTCG = TCG;
                }

jmp1:
                ViewScale = 1.0 / VOX_RES; // metres to voxels
                TCG.printPose(" TCG before vol3D", 1, NULL);


                if(FrameRecord<10 && fabs(ThetaCB(0))<1.0E-5){
                    // skip
                } else {
                    volume3DKernel_Launcher(d_vol, volumeSize, DarrySurf, arrFullSz, TCG, xgbf, ViewScale);
                }


                cudaEventRecord(GPU_stop, 0);
                cudaEventSynchronize(GPU_stop);
                cudaEventElapsedTime(&GPU_elapsedTime, GPU_start, GPU_stop);
                if(FirstFileInput){
                    FirstFileInput = 0;
                } else {
                    fprintf(pResFile, " %0.5f,",GPU_elapsedTime);
                    fprintf(pResFile, "\n");
                }
                break;
//
// case 6 Same as case 4 but with 
//
            case 6:
                cudaEventRecord(GPU_start, 0);

                ViewScale = CAM_D2M; // camera units to metres
                DepthSurface_Launcher(c_Depth, DarrySurf, arrFullSz, ViewScale, DepthTab, PreFilter2);
                //

                Depth_to_Float_Launcher(c_Depth, d_Depth, arrFullSz, ViewScale);

                cudaMemset(dAtomVec, 0, ATOM_VEC_SIZE * sizeof(int));
                cudaMemset(dWorkMat, 0, wMatBytes);
                cudaMemset(dMap, 0, mapBytes);
                PCA_Reduce_Launcher(d_Depth, arrFullSz, cellSz, dMap, mapSz, pointVec, pntSz, dWorkMat, wMatStep, ViewScale, CurveThresh, DetScale, dAtomVec);

// back-copy for debug
                cudaMemcpy(hPointVec, pointVec, pointBytes, cudaMemcpyDeviceToHost);
                cudaMemcpy(hAtomVec, dAtomVec, ATOM_VEC_SIZE * sizeof(int), cudaMemcpyDeviceToHost);
                cudaMemcpy(hWorkMat, dWorkMat, wMatBytes, cudaMemcpyDeviceToHost);
                cudaMemcpy(hMap, dMap, mapBytes, cudaMemcpyDeviceToHost);

// pointCount is essential output (not debug)
                pointCount = hAtomVec[0];
//###
//              PCA_Reduce_Debug(hPointVec, hWorkMat, wMatBytes, hMap, hAtomVec);
//###
                TCG = SaveTCG;
                if (refCount <= 0){
//
// The first frame is copied to refVec
                    refCount = pointCount;
//>>>
                    cudaMemcpy(refVec, pointVec, pointBytes, cudaMemcpyDeviceToDevice);
                    cudaMemcpy(hRefVec, refVec, pointBytes, cudaMemcpyDeviceToHost);
                    cudaMemcpy(hRefMap, dMap, mapBytes, cudaMemcpyDeviceToHost);
//<<<
                    FrameCount = 0;
                } else {
//
// PointVec Masking
                    subCount=0;
                    for(ii=XLP;ii<(cxn-XRP);ii++){
                        for(jj=XTP;jj<(cyn-XBP);jj++){
                            mapIdx = Get2DIndex(ii, jj, mapSz.x);
                            pointIdx = hMap[mapIdx]; // pointIdx for this cell
                            if(pointIdx>-1){
                                for(im=0;im<pntSz.y;im++){
                                    Val = hPointVec[pointIdx + pntSz.x*im];
                                    hSubVec[subCount + pntSz.x*im] = Val;
                                }
                                subCount++;
                            }
                        }
                    }
//
                    cudaMemcpy(subVec,hSubVec,pointBytes,cudaMemcpyHostToDevice);
//
// RefVec Masking                
                    subRefCount=0;
                    for(ii=XLR;ii<(cxn-XRR);ii++){
                        for(jj=XTR;jj<(cyn-XBR);jj++){
                            mapIdx = Get2DIndex(ii, jj, mapSz.x);
                            refIdx = hRefMap[mapIdx]; // refVec for this cell
                            if(refIdx>-1){
                                for(im=0;im<pntSz.y;im++){
                                    Val = hRefVec[refIdx + pntSz.x*im];
                                    hSubRef[subRefCount + pntSz.x*im] = Val;
                                }
                                subRefCount++;
                            }
                        }
                    }

                    cudaMemcpy(subRef,hSubRef,pointBytes,cudaMemcpyHostToDevice);

                    printf("\n subCount, %d, subRefCount, %d, \n",subCount,subRefCount);

                    P2Plane_ICP_Launcher(subVec, pntSz, subCount, ipVec, rotVec, subRef, 
                                        subRefCount, dMatch, MatchCount, ICPFrames,
                                        dWorkVec, CellWorkStep, dResVec, DistMin, CosMin, FrameRecord,
                                        TCG, xgbf, dTCB, dAtomVec, pDmpFile, pResFile);

                    TCG2.InitMatrix();

                    for(i=0;i<3;i++){
                        for(j=0;j<3;j++){
                            for(k=0;k<3;k++){
                                TCG2(i,j) = TCG2(i,j) + TCG(i,k)*dTCB(k,j);  // increment the rotation matrix
                            }
                        }
                    }
                    TCG2(3,3) = 1.0;
                    TCG2.UpdateGPU();

// get back the delta displacements
                    dispb  = dTCB.GetDisplacement();
                    ddispg = TCG.InvRotateH(dispb);
                    dispg  = TCG.GetDisplacement();

                    dispg.x = dispg.x + ddispg.x;
                    dispg.y = dispg.y + ddispg.y;
                    dispg.z = dispg.z + ddispg.z;

                    TCG = TCG2;

                    TCG.SetDisplacement(dispg);

                    TCG.printPose(" Pose, ", 0, pResFile);
                    Pose = TCG.GetPose();
                    Pose2 = Pose;
// output pose errors between rig sensor data and P2L computed pose

                    fprintf(pResFile, " errors, ");
                    for (i = 0; i < 3; i++)
                    {
                        PoseErr(i) = (RigPose(i) - Pose(i) ) * DPR;
                        fprintf(pResFile, " %0.5f,", PoseErr(i));
                    }
                    for (i = 3; i < 6; i++)
                    {
                        PoseErr(i) = RigPose(i) - Pose(i);
                        fprintf(pResFile, " %0.5f,", PoseErr(i));
                    }
//
// update the reference Array it is now pointVec
// may want to threshold this reference update as a means to counteract drift
//
                    FrameCount++;
                    if (FrameCount >= ICPFrames)
                    {
                        cudaMemcpy(refVec, pointVec, pointBytes, cudaMemcpyDeviceToDevice);
                        cudaMemcpy(hRefVec,refVec, pointBytes, cudaMemcpyDeviceToHost);
                        cudaMemcpy(hRefMap, dMap, mapBytes, cudaMemcpyDeviceToHost);
                        FrameCount = 0;
                        Ref.Count = pointCount;
                        SaveTCG = TCG;
                    }
                }
//
                ViewScale = 1.0 / VOX_RES; // metres to voxels
                TCG.printPose(" TCG before vol3D", 1, NULL);

// this trap removes the first few frames to prevent a startup issue with ServoTrack
               if(FrameRecord<10 && fabs(ThetaCB(0))<1.0E-5){
                    // skip
                } else {
                    volume3DKernel_Launcher(d_vol, volumeSize, DarrySurf, arrFullSz, TCG, xgbf, ViewScale);
                }
                cudaEventRecord(GPU_stop, 0);
                cudaEventSynchronize(GPU_stop);
                cudaEventElapsedTime(&GPU_elapsedTime, GPU_start, GPU_stop);
                if(FirstFileInput){
                    FirstFileInput = 0;
                } else {
                    fprintf(pResFile, " %0.5f,",GPU_elapsedTime);
                    fprintf(pResFile, "\n");
                }

                break;
            default:
                printf(" No Operation\n");
                break;
            }
            UpdateView = false; // this only needs to happen once for every FileUpdate
        }
        usleep(33333);
        glutPostRedisplay(); // sets the OpenGL update flag
        glutMainLoopEvent(); // pumps the screen update
        Clock.Tick();
    } while (!Stopped);
    //
    fsync(diskFD);
    close(diskFD);
    fclose(pDmpFile);
    DepthTab.FreeTable();
    DiskFile.close();
//###    Global_mtx.unlock();
//###    condvar.notify_all();
    atexit(exitfunc);
    return 0;
}
//
//========================================================================================
//
int AllocateMemory()
{
//
// Initialisation of GPU memory and other functions
// Caution Needed some of these are fixed by parameters
// others are flexible and data file driven
// All should be declared as externs in constants.h
// and declared in this file above main()
//
    const char *pError;
    size_t cudaMemUsed, palMemBytes, volMemBytes;
    size_t hostMemUsed;
    //
    palMemBytes = MAX_COLOURS * sizeof(unsigned int);
    volMemBytes = VOL_NX * VOL_NY * VOL_NZ * sizeof(float);

    cudaMalloc(&pPallet, palMemBytes);
    cudaMalloc(&d_vol, volMemBytes); // 3D volume data
    cudaMemUsed = palMemBytes + volMemBytes;

    cudaMalloc(&b_Depth, CAM_BYTES);
    cudaMalloc(&c_Depth, CAM_BYTES);
    cudaMemUsed += 2 * CAM_BYTES;

    cudaMalloc(&d_Depth, CAM_F_BYTES );
    cudaMalloc(&v_Depth, CAM_F_BYTES );
    cudaMemUsed += 2 * CAM_F_BYTES;

    if (cudaSuccess != cudaGetLastError())
    {
        pError = cudaGetErrorString(CudaErr);
        printf(" AllocateMemory Error %s, cudaMemUsed %lu,\n", pError, cudaMemUsed);
        return -1;
    }

    uhDepth = (UINT16 *)calloc(CAM_SIZE, sizeof(UINT16));
    hDepth = (float *)calloc(CAM_SIZE, sizeof(float));
    hostMemUsed = CAM_BYTES + CAM_F_BYTES;
//
//---------------------------------------------------------------------------------------------------
// RealSense Camera Array
//
    arrFullSz = make_int2(CAM_W, CAM_H);
    b_Filt = (UINT16 *)calloc(CAM_SIZE, sizeof(UINT16));
    hostMemUsed += CAM_BYTES;
//
// Celular matrices                                         // accounting is based on a 16x16 cell
//                                                          // and Intel RealSense L515 camera
    cellSz = make_int2(CellX, CellX); // example (16,16)
    cxn = CAM_W / cellSz.x;           //
    cyn = CAM_H / cellSz.y;           //
    mapSz = make_int2(cxn, cyn);      //
    MapSize = mapSz.x * mapSz.y;
    MapSeedLimit = (int)MapSize * CELL_SEED_RATE;
//
// used in PCA_Reduce to scale the determinant into a reasonable range based on Cell Size
// DetScale was determined in: ../../Matlab/VisionOffV3/MeshDeterminants.m
//
    DetScale = 1.0 / (((1.5454E-16 * cellSz.x - 3.1227E-15) * cellSz.x + 2.141E-14) * cellSz.x - 4.8157E-14);
//
// dMap maps the downsampled 2D image cells to pointVec indices for that thread/cell
// if a cell contained zeros its entry is set -1 indicating it cannot be matched
// dRefMap is a retained copy of dMap for the reference frame
// dCellMap identifies the common subset within the test and reference images
// if a cell cannot be mapped its va
//
    mapBytes = MapSize * sizeof(int);
    hMap = (int *)calloc(MapSize, sizeof(int));
    hRefMap = (int *)calloc(MapSize, sizeof(int));
    memset(hMap, 0, mapBytes);
    hostMemUsed += 2*mapBytes;

    cudaMalloc(&dMap, mapBytes);
    cudaMemset(dMap, 0, mapBytes);
    cudaMalloc(&dRefMap, mapBytes);
    cudaMalloc(&dCellMap, mapBytes);
    cudaMemset(dCellMap, 0, mapBytes);
    cudaMemUsed += 6 * mapBytes;

    if (cudaSuccess != cudaGetLastError())
    {
        pError = cudaGetErrorString(CudaErr);
        printf(" AllocateMemory Error %s, cudaMemUsed %lu,\n", pError, cudaMemUsed);
        return -1;
    }

/* no longer used
    hCellMap = (int *)calloc(MapSize, sizeof(int));
    hSeedCells = (int *)calloc(MapSeedLimit, sizeof(int));

// pointMat cell matrix of vertex data
        matSz = make_int2(cxn * KNPoints, cyn * KNVecs); //
        matBytes = matSz.x * matSz.y * sizeof(float);    //
        cudaMalloc(&pointMat, matBytes);                 //
        cudaMemset(pointMat, 0, matBytes);               //
        hPointMat = (float *)calloc(matSz.x * matSz.y, sizeof(float));
        cudaMemUsed += matBytes;
*/

// pointVec, refVec and rotVec are the same size
    pntSz = make_int2(cxn * cyn, KNPoints);         //
    pointBytes = pntSz.x * pntSz.y * sizeof(float); //
    cudaMalloc(&pointVec, pointBytes);
    hPointVec = (float *)calloc(pntSz.x * pntSz.y, sizeof(float));
 
    cudaMalloc(&ipVec, pntSz.x * 2 * sizeof(int));

    cudaMalloc(&refVec, pointBytes);
    hRefVec = (float *)calloc(pntSz.x * pntSz.y, sizeof(float));
    cudaMalloc(&pointSav, pointBytes);
    memset(hPointVec, 0, pntSz.x * pntSz.y * sizeof(float));
    cudaMalloc(&rotVec, pointBytes);
    hRotVec = (float *)calloc(pntSz.x * pntSz.y, sizeof(float));
    cudaMalloc(&virtVec, pointBytes);

    cudaMalloc(&subVec, pointBytes);
    hSubVec = (float *)calloc(pntSz.x * pntSz.y, sizeof(float));

    cudaMalloc(&subRef, pointBytes);
    hSubRef = (float *)calloc(pntSz.x * pntSz.y, sizeof(float));

    cudaMemUsed += 6 * pointBytes;
    hostMemUsed += 4 * pointBytes;

    if (cudaSuccess != cudaGetLastError())
    {
        pError = cudaGetErrorString(CudaErr);
        printf(" AllocateMemory Error %s, cudaMemUsed %lu,\n", pError, cudaMemUsed);
        return -1;
    }


// scratch pad size determined by biggest demand
    CellWorkStep = SEG64;                                  // currently sized by ICP_P2L->compute_P2L_Update()
    CellWorkStep = GPUStep(CellWorkStep);                      // increments to end on an allowable byte boundary
    workSz = cxn * cyn * (CellWorkStep);                   // be sure to check this against worst case code requirements
    workBytes = workSz * sizeof(float);                //
    cudaMalloc(&dWorkVec, workBytes);                  // I dont believe this is a problem but it may be better to
    hWorkVec = (float *)calloc(workSz, sizeof(float)); // use pitched memory
    cudaMemUsed += workBytes;
    hostMemUsed += workBytes;
//
// dWorkMat is a specific workspace for GMAT matrix processing
// matrices created in PCA_Reduce and used in Cell_Search
//
    wMatStep = cellSz.x * cellSz.y * 3 + 5 * 9 + 2 * 3;
    wMatStep = GPUStep(wMatStep);
    wMatSize = cxn * cyn * (wMatStep);
    wMatBytes = wMatSize * sizeof(float);
    cudaMalloc(&dWorkMat, wMatBytes);                    // storage per thread for covariance and other matrices produced by PCA_Reduce
    hWorkMat = (float *)calloc(wMatSize, sizeof(float)); // and used by Cell_Search
    cudaMemUsed += wMatBytes;
    hostMemUsed += wMatBytes;
    if (cudaSuccess != cudaGetLastError())
    {
        pError = cudaGetErrorString(CudaErr);
        printf(" AllocateMemory Error %s, cudaMemUsed %lu,\n", pError, cudaMemUsed);
        return -1;
    }

// point-pair matching vector                               // this has to match the indexing of pointVec
    matchBytes = pntSz.x * sizeof(int);           // [cxn*cyn*KNVecs*KNPoints]
    hMatch = (int *)calloc(pntSz.x, sizeof(int)); //
    cudaMalloc(&dMatch, matchBytes);
    cudaMemset(dMatch, -1, matchBytes);
    cudaMalloc(&dSaveMatch, matchBytes);
    cudaMemset(dSaveMatch, -1, matchBytes);
    cudaMemUsed += 2 * matchBytes;
    hostMemUsed += matchBytes;

    if (cudaSuccess != cudaGetLastError())
    {
        pError = cudaGetErrorString(CudaErr);
        printf(" AllocateMemory Error %s, cudaMemUsed %lu,\n", pError, cudaMemUsed);
        return -1;
    }

// Vector used for atomic Indexing in Kernels
    atomSize = ATOM_VEC_SIZE;
    int atomBytes = atomSize * sizeof(int);
    hAtomVec = (int *)calloc(atomSize, sizeof(int));
    cudaMalloc(&dAtomVec, atomBytes);
    cudaMemset(dAtomVec, 0, atomBytes);
    cudaMemUsed += atomBytes;
    hostMemUsed += atomBytes;
// ResVec is used to return kernel results
    resSize = RES_VEC_SIZE;
    int resBytes = resSize * sizeof(float);
    hResVec = (float *)calloc(resSize, sizeof(float));
    cudaMalloc(&dResVec, resBytes);
    cudaMemset(dResVec, 0, resBytes);
    cudaMemUsed += resBytes;
    hostMemUsed += resBytes;
//
// CUDA surface array integrates depth and probability
// based on lookup table of distance from camera
//
    cudaChannelFormatDesc DarryDesc = cudaCreateChannelDesc(32, 32, 0, 0, cudaChannelFormatKindFloat);
    cudaMallocArray(&ppDarry, &DarryDesc, CAM_W, CAM_H, cudaArraySurfaceLoadStore);

    cudaResourceDesc DarrysurfRes;
    memset(&DarrysurfRes, 0, sizeof(cudaResourceDesc));
    DarrysurfRes.resType = cudaResourceTypeArray;
    DarrysurfRes.res.array.array = ppDarry;
    cudaCreateSurfaceObject(&DarrySurf, &DarrysurfRes);
    if (cudaSuccess != cudaGetLastError())
    {
        pError = cudaGetErrorString(CudaErr);
        printf(" AllocateMemory Error %s, cudaMemUsed %lu,\n", pError, cudaMemUsed);
        return -1;
    }

    cudaMemUsed += 2 * CAM_BYTES;
    int GlobalMemPercent = (int)(100.0 * double(cudaMemUsed) / 7974.0E+6);
    int HostMemPercent =  (int)(100.0 * double(hostMemUsed) / 1550.0E+06);

    printf(" CUDA Memory Allocated %lu, Percent of GPU Global Memory %d,\n", cudaMemUsed, GlobalMemPercent);
    printf(" Host Memory Allocated %lu, Percent of Host Memory %d,\n", hostMemUsed, HostMemPercent);

    return 0;
}
//
//========================================================================================
//
int LoadData(const char *pDataFile)
{
//
// Loads Program Configuration Data from File
// Most of the data is declared in constant.h and OpenGLFuncs.h
// and instantiated at the top of this file
//
// The order of items in the datafile is strict though comments '#'
// can be interspersed as needed. Any text following a '#' will
// be treated as comment and skipped
//
// This code does not check for missing or defective data items
//
    FString Line, Word, Num;
    float XFac, YFac, ZFac;
    int idx;

    FILE *pFile = fopen(pDataFile, "r");

    if (pFile == NULL)
    {
        printf(" Failed to Open Control File %s\n,", pDataFile);
        return -1;
    }
// ImageTest
    idx = 0;
    FindNextLine(pFile, Line);
    FindNextWord(Line, Word, idx);
    ImageTest = STR_2_I(Word);

// Registration Method
    idx = 0;
    FindNextLine(pFile, Line);
    FindNextNumber(Line, Num, idx);
    RegMethod = STR_2_I(Num);

// ClosedLoop
    idx = 0;
    FindNextLine(pFile, Line);
    FindNextNumber(Line, Word, idx);
    ClosedLoop = STR_2_I(Word);
    FindNextNumber(Line, Word, idx);
    CLStep = STR_2_I(Word);
    FindNextNumber(Line, Word, idx);
    CLStart = STR_2_I(Word);

// Cell Masking Fore, Turn, Left, Right, Top, Bot
    idx = 0;
    FindNextLine(pFile, Line);
    FindNextNumber(Line, Word, idx);
    Fore = STR_2_I(Word);
    FindNextNumber(Line, Word, idx);
    Turn = STR_2_I(Word);
    FindNextNumber(Line, Word, idx);
    Left = STR_2_I(Word);
    FindNextNumber(Line, Word, idx);
    Right = STR_2_I(Word);
    FindNextNumber(Line, Word, idx);
    Top = STR_2_I(Word);
    FindNextNumber(Line, Word, idx);
    Bot = STR_2_I(Word);
    if(Fore>=0){
        XLR = Left;
        XRR = Right;
        XTR = Top;
        XBR = Bot;
        XLP = Left;
        XRP = Right;
        XTP = Top;
        XBP = Bot;
    }
// this only affects sides and superposes Fore/Aft, Turn=0 has no effect
    if(Turn>0){     // right turn
        XLR = Left;
        XRR = 0;
        XLP = 0; 
        XRP = Right;
    } else if(Turn<0) { // left turn
        XLR = 0;
        XRR = Right;
        XLP = Left; 
        XRP = 0;
    }
        

// RenderMethod
    idx = 0;
    FindNextLine(pFile, Line);
    FindNextWord(Line, Word, idx);
    RenderMethod = STR_2_I(Word);

// render paramerers: dist, theta, threshold
    idx = 0;
    FindNextLine(pFile, Line);
    FindNextNumber(Line, Num, idx);
    dist = STR_2_FLOAT(Num);
    FindNextNumber(Line, Num, idx);
    thetaX = STR_2_FLOAT(Num);
    FindNextNumber(Line, Num, idx);
    thetaY = STR_2_FLOAT(Num);
    FindNextNumber(Line, Num, idx);
    threshold = STR_2_FLOAT(Num);

// FPS = frames per second
    idx = 0;
    FindNextLine(pFile, Line);
    FindNextNumber(Line, Num, idx);
    FPS = STR_2_I(Num);

// VideoOn, OutputOn, PlaybackOn, StepMode, FrameDump
    idx = 0;
    FindNextLine(pFile, Line);
    FindNextNumber(Line, Num, idx);
    VideoOn = STR_2_I(Num);
    FindNextNumber(Line, Num, idx);
    OutputOn = STR_2_I(Num);
    FindNextNumber(Line, Num, idx);
    PlaybackOn = STR_2_I(Num);
    FindNextNumber(Line, Num, idx);
    StepMode = STR_2_I(Num);
    FindNextNumber(Line, Num, idx);
    FrameDump = STR_2_I(Num);
//
// camera body axis position
//
    idx = 0;
    FindNextLine(pFile, Line);
    FindNextNumber(Line, Num, idx);
    CamL.x = STR_2_FLOAT(Num);
    FindNextNumber(Line, Num, idx);
    CamL.y = STR_2_FLOAT(Num);
    FindNextNumber(Line, Num, idx);
    CamL.z = STR_2_FLOAT(Num);
//
    LCB(0) = CamL.x;
    LCB(1) = CamL.y;
    LCB(2) = CamL.z;
//
// camera start position in 3D model
    idx = 0;
    FindNextLine(pFile, Line);
    FindNextNumber(Line, Num, idx);
    XFac = STR_2_FLOAT(Num);
    FindNextNumber(Line, Num, idx);
    YFac = STR_2_FLOAT(Num);
    FindNextNumber(Line, Num, idx);
    ZFac = STR_2_FLOAT(Num);

    XGB(0) = VOX_RES * volumeSize.x *XFac;
    XGB(1) = VOX_RES * volumeSize.y *YFac;
    XGB(2) = VOX_RES * volumeSize.z *ZFac ;       

// depth data false colour PaletteFile
    idx = 0;
    FindNextLine(pFile, Line);
    FindNextWord(Line, Word, idx);
    PaletFile = Word;

// RecordingFile
    idx = 0;
    FindNextLine(pFile, Line);
    FindNextWord(Line, Word, idx);
    RecordFile = Word;

// Tracking Results File
    idx = 0;
    FindNextLine(pFile, Line);
    FindNextWord(Line, Word, idx);
    ResultsFile = Word;

// OpenCV VideoFile
    idx = 0;
    FindNextLine(pFile, Line);
    FindNextWord(Line, Word, idx);
    VideoFile = Word;

// Dump File
    idx = 0;
    FindNextLine(pFile, Line);
    FindNextWord(Line, Word, idx);
    DumpFile = Word;

// FrameFile
    idx = 0;
    FindNextLine(pFile, Line);
    FindNextWord(Line, Word, idx);
    FrameFile = Word;

// Cellular Reduction
// CellX  KNPoints  ICPFrames PreFilter1  PreFilter2
    idx = 0;
    FindNextLine(pFile, Line);
    FindNextNumber(Line, Num, idx);
    CellX = STR_2_I(Num);
    cellSz.x = cellSz.y = CellX;
    FindNextNumber(Line, Num, idx);
    KNPoints = STR_2_I(Num);
    FindNextNumber(Line, Num, idx);
    ICPFrames = STR_2_I(Num);
    FindNextNumber(Line, Num, idx);
    PreFilter1 = STR_2_I(Num);
    FindNextNumber(Line, Num, idx);
    PreFilter2 = STR_2_I(Num);
    FindNextNumber(Line, Num, idx);

// Patch filtering and Point-pair matching critera
    idx = 0;
    FindNextLine(pFile, Line);
    FindNextNumber(Line, Num, idx);
    CurveThresh = STR_2_FLOAT(Num);
    FindNextNumber(Line, Num, idx);
    DistMin = STR_2_FLOAT(Num);
    FindNextNumber(Line, Num, idx);
    CosMin = STR_2_FLOAT(Num);
    FindNextNumber(Line, Num, idx);
    FMThresh = STR_2_FLOAT(Num);

// Bilateral Filter parameters
    idx = 0;
    FindNextLine(pFile, Line);
    FindNextNumber(Line, Num, idx);
    bilat_ksz = STR_2_I(Num);
    FindNextNumber(Line, Num, idx);
    bilat_spatial_const = STR_2_FLOAT(Num);
    FindNextNumber(Line, Num, idx);
    bilat_depth_const = STR_2_FLOAT(Num);

    ssinv2 = 0.5 / (bilat_spatial_const * bilat_spatial_const);
    sdinv2 = 0.5 / (bilat_depth_const * bilat_depth_const);

// CropFilter
    idx = 0;
    FindNextLine(pFile, Line);
    FindNextNumber(Line, Num, idx);
    CropDepth = STR_2_FLOAT(Num);
    FindNextNumber(Line, Num, idx);
    ZeroOut = STR_2_I(Num);
//
// more here
    fclose(pFile);
    return 1;
}

int WriteFrameMatrix(UINT16 *pData, const char *pMatName, const char *pFileName)
{
//
// Writes a Depth Data frame to pFileName for interface with Matlab
// A lot of work needed to set up the objects using file local variables
// could be better optimised. The Frame
//
    int matFD = MatFileOut.OpenFile(1, pFileName);
    if (matFD <= 0)
    {
        printf("Problem Opening Output File %s\n", (const char *)FrameFileName);
        return 0;
    }
    else
    {
        MatName.SetCommsPort(&MatFileOut);
        MatName.SetString(pMatName);
        MatSz.SetCommsPort(&MatFileOut);
        MatOut.SetCommsPort(&MatFileOut);
        MatOut.SetData(pData, CAM_SIZE);
        SzVec[0] = CAM_W;
        SzVec[1] = CAM_H;
        MatSz.SetData(SzVec, 2);

        // finally commit
        MatName.Send();
        MatSz.Send();
        MatOut.Send();
        MatFileOut.close();
    }
    return 1;
}

int ReadMatrix(UINT16 *pData, int SZ, const char *pFileName)
{
//
// reads back a matrix dimesions of matrix read in
// must be compatible with that allocated for pData
//
    int FSz;
    int matFD = MatFileIn.OpenFile(2, pFileName);
    if (matFD <= 0)
    {
        printf("Problem Opening Input File %s\n", (const char *)FrameFileName);
        return 0;
    }
    else
    {
        TextIn.SetCommsPort(&MatFileIn);
        MatSz.SetCommsPort(&MatFileIn);
        MatSz.SetData(SzVec, 2);
        MatIn.SetCommsPort(&MatFileIn);

        // finally...
        TextIn.ReceiveAll();
        MatSz.ReceiveAll();
        FSz = SzVec[0] * SzVec[1];
        if (FSz > SZ)
        {
            printf("Error Receive Matrix Greater than SZ\n");
        }
        else
        {
            MatIn.SetData(pData, FSz);
            MatIn.ReceiveAll();
        }
        MatFileIn.close();
    }

    return FSz;
}

void PCA_Reduce_Debug(float *hPointVec, int points, float *hWorkMat, int workMatStep, int *hMap, int *hAtomVec)
{
//
// PCA_Reduce produces a lot of output and consequently debugging it requires
// a lot of inspection of internals, consequently this output is distracting when
// you dont really need it so this function contains it and can be
// commented out when not required.
// Care needed there can be no secondary actions this is strictly output only!
//
    int ii, jj, kk, mapIdx;
    int pointIdx, WorkIdx, OutIdx;
    int nullcells, vecCount, threshFail, vecFail, overShoots;

    vecCount = hAtomVec[1];
    nullcells = hAtomVec[2];
    threshFail = hAtomVec[3];
    vecFail = hAtomVec[4];
    overShoots = hAtomVec[5];

    fprintf(pDmpFile, "\n PCA_Reduce, elapsedTime, %#.5f, pointCount %d, vecCount %d, NullCells %d, ThreshFail %d, VecFail %d, overShoots %d,\n", GPU_elapsedTime, points, vecCount, nullcells, threshFail, vecFail, overShoots);
    printf("\n PCA_Reduce, elapsedTime, %#.5f, pointCount %d, vecCount %d, NullCells %d, ThreshFail %d, VecFail %d, overShoots %d,\n", GPU_elapsedTime, pointCount, vecCount, nullcells, threshFail, vecFail, overShoots);
    //
    fprintf(pDmpFile, " ii, jj, pointIdx, WorkIdx, X, Y, Z, nx, ny, nz, startW, det , 1.0,      C matrix \n");
    DetMap.ZeroInit(10, 10);
    //###
    //    for(jj = 1; jj<(mapSz.y-1); jj++){
    //        for(ii= 1; ii<(mapSz.x-1); ii++){
    //###
    for (jj = 10; jj < 20; jj++)
    {
        for (ii = 10; ii < 20; ii++)
        {

            mapIdx = Get2DIndex(ii, jj, mapSz.x);
            pointIdx = hMap[mapIdx]; // pointIdx for this cell
            DetMap(jj, ii) = 0.0;
            if (pointIdx >= 0)
            {
                WorkIdx = int(hPointVec[pointIdx + pntSz.x * 6]);
                pWorkSt = &hWorkMat[WorkIdx];
                OutIdx = 0;
                C.SetData(GetData(pWorkSt, OutIdx, 9), 3, 3);
                AM1.SetData(GetData(pWorkSt, OutIdx, 9), 3, 3);
                Lambda.SetData(GetData(pWorkSt, OutIdx, 3), 3);
                SLambdaM1.SetData(GetData(pWorkSt, OutIdx, 3), 3);
                V.SetData(GetData(pWorkSt, OutIdx, 9), 3, 3);
                W.SetData(GetData(pWorkSt, OutIdx, 9), 3, 3);

                //
                fprintf(pDmpFile, " %d, %d, %d, %d, ", ii, jj, pointIdx, WorkIdx);
                for (kk = 0; kk < 18; kk++)
                {
                    fprintf(pDmpFile, " %#11.4E,", hPointVec[pointIdx + pntSz.x * kk]);
                }

                fprintf(pDmpFile, " \n");

                Lambda.printVector("Lambda ", pDmpFile);
                SLambdaM1.printVector("SLambdaM1 ", pDmpFile);
                C.printMatrix(" C ", pDmpFile);
                AM1.printMatrix(" AM1 ", pDmpFile);
                V.printMatrix(" V ", pDmpFile);
                W.printMatrix(" W ", pDmpFile);

                float detScale = ((3.2903E-15 * cellSz.x - 6.4876E-14) * cellSz.x + 4.3712E-13) * cellSz.x - 9.7098E-13;
                DetMap(jj, ii) = W(0, 2) / detScale; // Lambda(0)*Lambda(1)*Lambda(2);
            }
        }
    }
    //###
    DetMap.printMatrix("DetMap", pDmpFile);
    //###
}
//
void exitfunc()
{
//
// This could be extended to free up all arrays
//
    if (pbo)
    {
        cudaGraphicsUnregisterResource(cuda_pbo_resource);
        glDeleteBuffers(1, &pbo);
        glDeleteTextures(1, &tex);
    }
    writer.release();
    cudaDeviceReset(); // this releases all
}
//
//-----------------------------------------------------------------------------------
//
