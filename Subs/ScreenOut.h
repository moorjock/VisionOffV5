//
// <ScreenOut.h>
//
// Small class to manage printing output to an auxilary xterm screen via execlp()
//
// Due to the complexities of fork()ing/execpl()ing, the process of starting 
// the receiving xterm is the responsibility of the main program and must 
// have taken place before this pipe can be opened and used.
//
// This class opens the pipe and manages communcation with it
// 
//
class ScreenOutPipe
{
    int m_FD;   // the FD for this pipe

    const char* m_pipeName;

public:

    ScreenOutPipe();

    int GetFD(){return m_FD;}

    int Open(const char* pPipeName);

    void Close();

    int ScreenOut(const char* pText);

    int ScreenOut(int* pInts);

    int ScreenOut(float* pFloats);

    int ScreenOut(FVec& FV);

    int ScreenOut(double* pDoubs);

    void EndLn();

};
