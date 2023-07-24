//
// Some of the OpenGLFunctions are separated from the main module code
// they are essentially OpenGL companion UI code 
// they don't change much and they clutter the main code
//
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <unistd.h>

#include <GL/glew.h>
#include <GL/freeglut.h>

#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include "../Subs/constants.h"
#include "../Subs/FMat.h"
#include "../Subs/device_funcs.cuh"
#include "../Subs/MyDataTypes.h"
#include "../Subs/MyTimer.h"

#include "VisionOffV5.h"
#include "OpenGLFuncs.h"
#include "kernel.h"


void render() 
{
  uchar4 *d_out = 0;
  char title[128];
  int Tdelta;
  size_t* d_outSz=NULL;
  cudaGraphicsMapResources(1, &cuda_pbo_resource, 0);
  
  cudaGraphicsResourceGetMappedPointer((void **)&d_out, d_outSz,
                                       cuda_pbo_resource);
// 
// render the volume to screen
// 
  renderKernelLauncher(d_out, d_vol, SCREEN_W, SCREEN_H, volumeSize, RenderMethod, xs, ys, zs, thetaX, thetaY,
                 threshold, dist, pPallet, colours, pCV3Out);


//###  cudaMemcpy(pOutImage, pCV3Out, CAM_W*CAM_H*sizeof(uchar3),cudaMemcpyDeviceToHost);  

  cudaGraphicsUnmapResources(1, &cuda_pbo_resource, 0);

  Tdelta = Clock.Tock();
  Clock.Tick();
//
// Capture frame to video file
//
  if( (VideoOn==true) && writer.isOpened() ){
      src = cv::Mat( cv::Size(CAM_W,CAM_H), CV_8UC3, pOutImage);
      writer.write(src);
  }

  sprintf(title, " ImageTest %d RegMethod %d xs %.1f ys %.1f zs %.1f theX %.1f theY %.1f, DT %d",ImageTest,RegMethod,xs,ys,zs,thetaX*DPR,thetaY*DPR,Tdelta);

  glutSetWindowTitle(title);
}

void initPixelBuffer() 
{
// 

  glGenBuffers(1, &pbo);
  glBindBuffer(GL_PIXEL_UNPACK_BUFFER, pbo);

  glBufferData(GL_PIXEL_UNPACK_BUFFER, 4*SCREEN_W*SCREEN_H*sizeof(GLubyte), 0, GL_STREAM_DRAW);

  glGenTextures(1, &tex);
  glBindTexture(GL_TEXTURE_2D, tex);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

  cudaGraphicsGLRegisterBuffer(&cuda_pbo_resource, pbo,
                               cudaGraphicsMapFlagsWriteDiscard);
}


void display() {
//
  render();
  draw_texture();
  glutSwapBuffers();
}

void draw_texture() {
//
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, SCREEN_W, SCREEN_H, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
  glEnable(GL_TEXTURE_2D);
  glBegin(GL_QUADS);
  glTexCoord2f(0.0f, 0.0f); glVertex2f(0, 0);
  glTexCoord2f(0.0f, 1.0f); glVertex2f(0, SCREEN_H);
  glTexCoord2f(1.0f, 1.0f); glVertex2f(SCREEN_W, SCREEN_H);
  glTexCoord2f(1.0f, 0.0f); glVertex2f(SCREEN_W, 0);
  glEnd();
  glDisable(GL_TEXTURE_2D);
}

void initScreen(int *argc, char **argv) {
//
  glutInit(argc, argv);
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
  glutInitWindowSize(SCREEN_W, SCREEN_H);
  glutCreateWindow("Volume Visualizer");
  GLenum err = glewInit();
  if (err != GLEW_OK){
    printf("problem initialising GLEW\n");
  }
}

int LoadPaletteFile(const char* pPalFile)
{
//
// Loads the colour palette from file these are
// Windows RGB-encoded numbers representing
// R-G-B-A components in the first 4 bytes
// This is a hack down from the original.
// For full colour treatment see:
//		Foley and van Damm
//
	int i, idx1, idx2, idx, found, count, ival;
	unsigned int dval;
	FILE* pFile;
	FString Line, Word;
//
	pFile = fopen(pPalFile,"r");
	if(pFile>0){
	    if(FindNextLine(pFile,Line)) {
//
  			idx1 = 0;
  			FindNextWord(Line,Word,idx1);
  			if(Word=="CONTENT"){
  				if(FindNextLine(pFile,Line)){
  					idx2=0;
	          FindNextWord(Line,Word,idx2);
		        if((ival=STR_2_I(Word))>0){
			        count = abs(ival);
			        if(count>MAX_COLOURS){
				  			printf("Too Many Colours in File\n Maximum = %d\n",MAX_COLOURS);
				        return -1;
				  		}
	          }
		        else{return 0;}
			      if(FindNextLine(pFile,Line)){
			  			idx2=0;
              FindNextWord(Line,Word,idx2);
		          if(Word=="MATERIALS"){
//
// Main Materials Read loop
//
			          found=0;
				        for(i=0;i<count;i++){
    							if(FindNextLine(pFile,Line)){
//
// basic colour setting is same variable as m_Diffuse
//
  						      idx=0;
  					        FindNextWord(Line,Word,idx);
 								    FindNextWord(Line,Word,idx);
       							if(idx>-2){
      								dval = (unsigned int)STR_2_DOUB(Word);
      								m_Palette[i]=dval;
	  			            found++;
		  					    } else {
      								break;
      							}
	                }
		            }
		  	        if(found<(count-1)){
                  printf(" Number of Colours found %d less than expected %d\n",found,count);
					        count = found;
		  					}
			  				fclose(pFile);
				  			return count;
		  			  }
            }
				  }
			  }
		}
		fclose(pFile);
	} else{
		perror("Palette File Open");
	}
	return -1;
}

void keyboard(unsigned char key, int x, int y) {
  if (key == 'v') RenderMethod = 1; // volume render
  if (key == 's') RenderMethod = 2; // my slice shader
  if (key == 'r') RenderMethod = 3; // raycast
  if (key == 'm') RenderMethod = 4; // my ray shader
  if (key == 'n') RenderMethod = 5; // my simple shader
  
  if (key == 'D') {
    --dist;
    if(dist<0.0) dist = 0.0; // decrease slice distance
  }

  if (key == 'd') {
    ++dist; // increase slice distance
    if(dist > float(VOL_NZ)){
      dist = float(VOL_NZ);
    }
  }

  // on image key, select and reset
  if(key=='1') {
    ImageTest=1;
    key='a';
  }
  if(key=='2') {
    ImageTest=2;
    key='a';
  }
  if(key=='3') {
    ImageTest=3;
    key='a';
  }

  if(key=='4') {
    ImageTest=4;
    key='a';
  }

  if (key == 'a') {
    zs = zdist;
    xs = 0.f;
    ys = 0.0f;
    thetaX = 0.f;
    thetaY = 0.f;
    dist = 0.f;
//    VolumeInit=1;
  }


  if (key == 'z'){
    zs -= DELTA; // move source closer (zoom in)
    if(zs<-zdist){
      zs = -zdist;
    }
  }
  if (key == 'Z') {
    zs += DELTA; // move source farther (zoom out)
    if(zs>zdist){
      zs=zdist;
    }
  }

  if (key == 'y'){
    ys -= DELTA; // move source closer (zoom in)
    if(ys<-zdist){
      ys = -zdist;
    }
  }
  if (key == 'Y') {
    ys += DELTA; // move source farther (zoom out)
    if(ys>zdist){
      ys=zdist;
    }
  }

  if (key == 'x'){
    xs -= DELTA; // move source left
    if(xs<-zdist){
      xs = -zdist;
    }
  }
  if (key == 'X') {
    xs += DELTA; // move source right
    if(xs>zdist){
      xs=zdist;
    }
  }


  if (key == 't' ||  key == 'T') {
      VideoOn = !VideoOn;   // toggle the video output
  }
  if (key == 'f' ||  key == 'F') {
      PreFilter1 = !PreFilter1;   // toggle filtering
  }
  if (key == 'o' ||  key == 'O') {
      OutputOn = !OutputOn;   // toggle console output (of time history text to console)
  }

  if (key == 'g' ||  key == 'G') {
      PlaybackOn = !PlaybackOn;   // toggle playback (of recorded camera data from file)
  }

  if ( key == 'u' || key == 'U') {
      UpdateView=true;
  }

  if (key == 27 || key =='q' || key =='Q') {
      VideoOn = 0;
      Stopped=1;
  }
  glutPostRedisplay();
}

void handleSpecialKeypress(int key, int x, int y) {
  if (key == GLUT_KEY_LEFT) {
    thetaY -= D_THETA; // rotate left
    if(thetaY <= -2.0*MY_PI){
      thetaY += 2.0*MY_PI; // unroll
    }
  }
  if (key == GLUT_KEY_RIGHT) {
    thetaY += D_THETA; // rotate right
    if(thetaY >= 2.0*MY_PI){
      thetaY -= 2.0*MY_PI; // unroll
    }
  }

  if (key == GLUT_KEY_UP) {
    thetaX -= D_THETA; // rotate up
    if(thetaX <= -2.0*MY_PI){
      thetaX += 2.0*MY_PI; // unroll
    }
  }

  if (key == GLUT_KEY_DOWN) {
    thetaX += D_THETA; // rotate right
    if(thetaX >= 2.0*MY_PI){
      thetaX -= 2.0*MY_PI; // unroll
    }
  }

  glutPostRedisplay();
}

void printInstructions() {
  printf(" Controls:\n"
         " Simple depth image    : 1\n"
         " Simple 3D projection  : 2\n"
         " 3D Proj and Rotation  : 3\n"
         " Volume render mode    : v\n"
         " Slice render mode     : s\n"
         " Raycast mode          : r\n"
         " My Ray Shader         : m\n"
         " My Simple Shader      : n\n"
         " Video on/off          : t\n"
         " Filter on/off         : f\n"
         " File Recording on/off : g\n"
         " Zoom out/in           : up/down arrow keys\n"
         " Rotate view           : left/right arrow keys\n"
         " Dist In/out           : D/d\n"
         " Reset parameters      : z\n"
         " Quit (esc) or q\n");
}


