
class MyTimer
{
    SHORTI          m_Ident;    // my ident number
    MYDWORD         m_Tick;     // set to the start record time in msec
    MYDWORD         m_Tock;     // returns delta time since last tick or -1 if tick not set

    struct timeval  m_Start;    // used by MyMillis()
    struct timeval  m_Now;      // used by MyMillis()

public:

    MyTimer(int ID=0);

    void SetIdent(int ID){m_Ident = ID;}
    int  GetIdent(){return m_Ident;}

    void    WriteStart();
    int     ReadStart();

    void    StartTimer();    // resets the timer m_Start

    MYDWORD MyMillis();      // returns milliseconds elapsed time since last StartTimer() 

    MYDWORD MyMicros();      // returns microseconds elapsed since StartTimer

    void SetTick(MYDWORD Tick){m_Tick = Tick;}

    MYDWORD GetTick(){return m_Tick;}

    virtual void Tick();     // start the timer

    virtual MYDWORD Tock();     // returns the time delta since Tick() will be -1 if Tick not previously set

    void MuTick();

    MYDWORD MuTock();


};
