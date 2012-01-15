/*******************************************************************************

                     Copyright(c) by Ericsson Technologes

All rights reserved. No part of this document may  be  reproduced  in  any  way,
or  by  any means, without the prior written permission of Ericsson Technologies.

                  Licensed to Ericsson Technologies(tm) 2009

Description:

header file to the control socket of ardrone.

Author: Tom, Zhang Jiangtao

History:

26/12/2011 Tom  Initial code.

*******************************************************************************/
#ifndef __CONTROL_SOCKET_H_
#define __CONTROL_SOCKET_H_
#include "CommonLib.h"
#include "WorkerThread.h"
extern bool takeoff;
// config command will have some ms delay one by one
#define DELAY_IN_MS     30
#define MAX_COMMAND_LEN 1024
#define AT_PORT         5556
#define ARDRONE_IP_ADDR "192.168.1.1"

typedef struct 
{
    char instruction[255];
    char parameters[512]; 
    bool input; 
} control_command_t;

class CControlThread: public CWorkerThread
{
public:
    CControlThread();
    ~CControlThread();
    
protected:
    virtual int Run();
    virtual void OnStart();
    virtual void OnTerminate();    

public:       
    int Speedup();
    int Speeddown(); 
    int AddJob(control_command_t job);
    
    // This interface is for non-action control command
    int SendAT(char * command);                 
    int EnableNav();
    int EnableVideo();

    int Takeoff();
    int Landing(); 
    int Hovering();
    int RollLeft();
    int RollRight();
    int PitchForward();
    int PitchBackward();
    int GoUp();
    int GoDown();
    int RotateLeft();
    int RotateRight();
    int Emergency();
    int Watchdog();
      
private:
    int Init();
    int Trim();
    int GetSeq();
    
    // Commands to control ARdrone
    int SendPcmd(int enable, float roll, float pitch, float gaz, float yaw);
   
    int SendAT();
    int StopMoving();
    CPThreadCondition * condition_p;
    CPThreadMutex * mutex_p;          // used to protect the command race condition
    CDiagramSocket * control_p; 
    char command[1024];  
    // the speed
    float speed; 
    unsigned int seq;  
    control_command_t action; 
};    

extern CControlThread controlthread;
#endif
  
