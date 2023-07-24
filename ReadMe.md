Computer-Vision-Based Tracking Project ReadMe

The codes in this directory were developed as part of a PhD project entitled:

   Development and Testing of Advanced Imaging Methods for Autonomous Robot Control
   Copyright (C) Alan Shepherd 2023

A pre-release issue of this report is contained in the Doc sub-directory

The work developed a robot tracking system using: 
	Intel RealSense L515 depth camera
	NVIDIA GeForce RTX 2070 GPU

The depth camera data was recorded using test rigs and off-line postprocessing
using the VisionOffV5 program provided here.

The program was run on Ubuntu 18.04 LTS and compiled using:
	gcc version 7.5.0 (Ubuntu 7.5.0-3ubuntu1~18.04)
	and NVIDIA nvcc release 10.2, V10.2.89

To build and run the program will require a decent NVIDIA GPU and their SDK
For installation testing (only) one binary data file and the corresponding 
*.csv and *.ods results files are included. 
 
Sub-Dir		Contents
Data		Source Binary files
Doc		PhD Thesis (Pre-Publication)
Results		Resulting *.csv files
Subs		Utility functions and classes used by VisionOffV5
VisionOffV5	Main program components including make file

Alan Shepherd
24/07/23




