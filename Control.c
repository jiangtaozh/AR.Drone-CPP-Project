/*******************************************************************************

                     Copyright(c) by Upstart Technologes

You can modify and publish all the code under the licence of GPLv2.

Description:

This file is used to implement all the control command to the ARDrone. 

Author: Tom, Zhang Jiangtao

History:

26/12/2011 Tom  Initial code.

*******************************************************************************/
#include <math.h>
#include "Control.h"

CControlThread controlthread;
bool takeoff= false;
bool landing_flag = false;

// This function is used to convert float into the integer type format.
int floatToIntBits(float  x)
{
    union 
    {
        float f;  // assuming 32-bit IEEE 754 single-precision
        int i;    // assuming 32-bit 2's complement int
    }u;

    u.f = x;
    return u.i;
}

CControlThread::CControlThread()
{
    seq = 1;
    speed = 0.1;    
}

CControlThread::~CControlThread()
{
    
}

int CControlThread::GetSeq()
{
    return seq++;    
} 

int CControlThread::Run()
{
    printf("Control thread is startup\n");
    int count = 0;
    int l = 0;
    bool flag = false;
    while (!m_nTerminated)
    {
      // landing is ordered, or already takeoff, otherwise, keep silent
      if (takeoff || landing_flag)
      {
        if (landing_flag)
        {
            landing_flag = false;
        }

        // To avoid watchdog drop the connection, we need to maintain a regular command to the 
        // ARdrone, otherwise, ARdrone will regard the connection is lost if no command in 2s. 
        // roll and pitch will not work then.
        
        if (count++ > 3)
        {
            // lock inside to protect seq
            Watchdog();
            count = 0;
        }
        
        // the command will repeat if not changed
        mutex_p->Lock();
        if (strlen(action.parameters) == 0)
        {
            sprintf(command, "%s%d\r", action.instruction, GetSeq());
        }   
        else
        {
            sprintf(command, "%s%d,%s\r", action.instruction, GetSeq(), action.parameters);
        }

         // old command repeat mechanism 
        flag = false;
        if (action.input == false)
        {
            if (l++ >= 20) 
            {
                // stop to send action command, 2s passed
                flag = true;
            }
         }
         else
         {
             l = 0;
             // mark to old command
             action.input = false;
             flag = false;
         }

        if (flag)
        {
            StopMoving();
        }
        else
        {
            SendAT();
        }
        mutex_p->Unlock();
      }       

      // only wakeup under immediate execte command
      condition_p->Wait(100);
    }

    return 0;
}

void CControlThread::OnStart()
{      
    control_p = new CDiagramSocket(INADDR_ANY, 8899);
    if (control_p == NULL)
    {
        assert(0);
        return;
    }
        
    int ret = control_p->Open();
    if (ret == FAIL)
    {
        printf("control socket open failed\n");
        return;
    }    
    
    condition_p = new CPThreadCondition();
    if (condition_p == NULL)
    {
        assert(0);
        return;
    }
    
    mutex_p = new CPThreadMutex();
    if (mutex_p == NULL)
    {
        assert(0);
        return;
    }    
}               

void CControlThread::OnTerminate()
{
    if (control_p != NULL)
    {
        delete control_p;
        control_p = NULL;
    }
    
    if (condition_p != NULL)
    {
        delete condition_p;
        condition_p = NULL;
    }
    
    if (mutex_p != NULL)
    {
        delete mutex_p;
        mutex_p = NULL;
    }                  
} 

int CControlThread::EnableNav()
{
    // navdata demo 
    CPThreadMutexLocker lock(mutex_p);
    sprintf(command, "AT*CONFIG=%d,\"general:navdata_demo\",\"TRUE\"\r", GetSeq());
    SendAT();
    return 0;
}

int CControlThread::EnableVideo()
{ 
    CPThreadMutexLocker lock(mutex_p);
    sprintf(command, "AT*CONFIG=%d,\"general:video_enable\",\"TRUE\"\r", GetSeq());
    SendAT();
    return 0;
}

int CControlThread::Init()
{
    // Init the drone
    // altitude maximum, mm, 10,000 is unlimited.
    sprintf(command, "AT*CONFIG=%d,\"control:altitude_max\",\"2000\"\r", GetSeq());
    SendAT(); 
    nanoSleep(DELAY_IN_MS);

    //control level, 0 biginner, 1 ace, 2 max
    sprintf(command, "AT*CONFIG=%d,\"control:control_level\",\"0\"\r", GetSeq());
    SendAT(); 
    nanoSleep(DELAY_IN_MS);
     
    // enable video
    sprintf(command, "AT*CONFIG=%d,\"general:video_enable\",\"TRUE\"\r", GetSeq());
    SendAT();
    nanoSleep(DELAY_IN_MS);

    nanoSleep(DELAY_IN_MS);

    printf("Init to the ardrone is done\n");
    return 0;        
}

int CControlThread::Watchdog()
{
    CPThreadMutexLocker lock(mutex_p);
    sprintf(command, "AT*COMWDG=%d\r", GetSeq());
    SendAT();  
    return 0;
}

// Outside interface, only used to send at command without sequence number needed.
int CControlThread::SendAT(char * comstr)
{
    u32 destip = ntohl(inet_addr(ARDRONE_IP_ADDR));
    u16 destport = AT_PORT; 
    printf("%s\n", comstr);
    return control_p->Send((unsigned char *)comstr, strlen(comstr), destip, destport);    
}

// Inside usage
int CControlThread::SendAT()
{    
    u32 destip = ntohl(inet_addr(ARDRONE_IP_ADDR));
    u16 destport = AT_PORT; 
    //printf("%s\n", command);
    return control_p->Send((unsigned char *)command, strlen(command), destip, destport);
}

// enable 0 to make ardrone into hovering mode
// roll : left-right tilt. negative to fly leftward, positive to fly rightward 
// pitch: front-back tilt. negative to fly forward, postive to fly backward
// gaz  : vertical speed.  negative to go down, postive to go rise 
// yaw  : angular speed.   negative to spini left. postive to spin right. 
int CControlThread::SendPcmd(int enable, float roll, float pitch, float gaz, float yaw)
{
    CPThreadMutexLocker lock(mutex_p);
    // the float must be changed to int
    strcpy(action.instruction, "AT*PCMD=");
    sprintf(action.parameters, "%d,%d,%d,%d,%d\r", 
            enable, 
            floatToIntBits(roll), 
            floatToIntBits(pitch), 
            floatToIntBits(gaz), 
            floatToIntBits(yaw)); 
    action.input = true;
    // perform the command immediate
    condition_p->Signal();
}

int CControlThread::Takeoff()
{
    CPThreadMutexLocker lock(mutex_p);
    int instruction = 290718208;
    Trim();
    printf("command to takeoff\n");
    strcpy(action.instruction, "AT*REF=");
    sprintf(action.parameters, "%d", instruction);
    action.input = true;
    takeoff = true;
    return 0; 
}   

int CControlThread::Landing()
{
    CPThreadMutexLocker lock(mutex_p);
    int instruction = 290717696;
    printf("command to landing\n");
    strcpy(action.instruction, "AT*REF=");
    sprintf(action.parameters, "%d", instruction);
    action.input = true;
    landing_flag = true;
    takeoff = false;
    condition_p->Signal();
    return 0;     
} 

int CControlThread::Emergency()
{
    CPThreadMutexLocker lock(mutex_p);
    int instruction = 290717952;
    printf("command to emergency\n");
    takeoff = false;
    strcpy(action.instruction, "AT*REF=");
    sprintf(action.parameters, "%d", instruction);
    action.input = true;
    takeoff = true;
    condition_p->Signal();
    return 0; 
}   

int CControlThread::StopMoving()
{
    printf("stop to wait further command\n");
    sprintf(command, "AT*PCMD=%d,0,0,0,0,0\r", GetSeq());
    SendAT();
    return 0;
}

int CControlThread::Hovering()
{
    printf("command to hovering\n");
    SendPcmd(0, 0, 0, 0, 0);
    return 0;     
} 

// This command will cause drone reset its internal sensors
// at horizontal plane. not doing so before taking off will cause
// ardrone not being able to stablize itself when flying.
// Actually it is used to tell ardrone it is lying horizontally.
int CControlThread::Trim()
{
    printf("command to trim\n");
    sprintf(command, "AT*FTRIM=%d\r", GetSeq());
    SendAT();  
    return 0;     
}

// roll+
int CControlThread::RollRight()
{
    printf("command to fly right\n");
    SendPcmd(1, speed, 0, 0, 0);
    return 0;        
}

// roll-
int CControlThread::RollLeft()
{
    printf("command to fly left\n");
    SendPcmd(1, -speed, 0, 0, 0);
    return 0;        
}

// pitch+
int CControlThread::PitchForward()
{
    printf("command to pitch forward\n");
    SendPcmd(1, 0, -speed, 0, 0);
    return 0;        
}

// pitch-
int CControlThread::PitchBackward()
{
    printf("command to pitch backward\n");
    SendPcmd(1, 0, speed, 0, 0);
    return 0;        
}

// gaz+
int CControlThread::GoUp()
{
    printf("command to go up\n");
    SendPcmd(1, 0, 0, speed, 0);
    return 0;        
}

// gaz-
int CControlThread::GoDown()
{
    printf("command to go down\n");
    SendPcmd(1, 0, 0, -speed, 0);
    return 0;        
}

// yaw-
int CControlThread::RotateLeft()
{
    printf("command to rotate left\n");
    SendPcmd(1, 0, 0, 0, -speed);
    return 0;        
}

// yaw+
int CControlThread::RotateRight()
{
    printf("command to rotate right\n");
    SendPcmd(1, 0, 0, 0, speed);
    return 0;        
}

int CControlThread::Speedup()
{
    CPThreadMutexLocker lock(mutex_p);
    if (speed > 1)
    {
        printf("speed too fast, ignore\n");
        return -1;
    }
    speed += 0.1;
    printf("command to speed up to %f\n", speed);
    return 0;
}

int CControlThread::Speeddown()
{
    CPThreadMutexLocker lock(mutex_p);
    if (speed < 0.2)
    {
        printf("the speed is already down to slowest %f\n", speed);
        return -1;
    }
    speed -= 0.1;
    printf("command to speed down to %f\n", speed);
    return 0;
}

