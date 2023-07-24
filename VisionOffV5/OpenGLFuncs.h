//
// OpenGLFuncs.h
//
// OpenGL user-interface stuff for the main program
//
#ifndef OPENGL_FUNCS_H
#define OPENGL_FUNCS_H

extern unsigned int *pPallet;
extern unsigned int m_Palette[MAX_COLOURS];
extern int colours;

extern float zs; // distance from origin to source
extern float ys;   // vertical distance from source
extern float xs; // lateral distance from origin 
extern int FPS;   // frames per second

extern float dist;
extern int RenderMethod;
extern float thetaX;
extern float thetaY;

extern float threshold;
extern int Stopped;
extern int ImageTest;  //=1 for image = 0 for 3D
extern int RegMethod;   // registration method 1=P2P 2 = P2L
extern int PreFilter1;     //=1 for filter =0 for no filter
extern int VolumeInit;  // initialise the view volume;
extern int VideoOn;
extern int PlaybackOn; // recording to data file
extern int ImageTest;   // 1=simple image projection, 2=My 3D, 3=Tsdf with transform 
extern int CanWrite;
extern int UpdateView;

extern cv::Mat src;
extern cv::VideoWriter writer;

extern struct cudaGraphicsResource *cuda_pbo_resource;

class MyTimer;
extern MyTimer Clock;
extern uchar3 *pCV3Out;         // cuda memory pointer for OpenCV video output
extern uchar3 *pOutImage;       // local host pointed for OpenCV video output
extern GLuint pbo;              // OpenGL pixel buffer object
extern GLuint tex;              // OpenGL texture object
extern int OutputOn;            // switch console output of time history on/off
// void mymenu(int value);
// void createMenu();
void display();

void render();

void draw_texture();

void initScreen(int *argc, char **argv);

void initPixelBuffer();

void exitfunc();

void keyboard(unsigned char key, int x, int y);

void handleSpecialKeypress(int key, int x, int y);

void printInstructions();

int LoadPaletteFile(const char* pPalFile);

#endif      // OPENGL_FUNCS_H