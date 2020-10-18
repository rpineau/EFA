//
//  efa.h
//  PlaneWave EFA X2 plugin
//
//  Created by Rodolphe Pineau on 3/25/2018.

#ifndef __EFA__
#define __EFA__
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <memory.h>
#ifdef SB_MAC_BUILD
#include <unistd.h>
#endif
#ifdef SB_WIN_BUILD
#include <time.h>
#endif


#include <string>
#include <vector>
#include <sstream>
#include <iostream>
#include <exception>
#include <typeinfo>
#include <stdexcept>

#include "../../licensedinterfaces/sberrorx.h"
#include "../../licensedinterfaces/serxinterface.h"
#include "../../licensedinterfaces/loggerinterface.h"
#include "../../licensedinterfaces/sleeperinterface.h"

#define PLUGIN_DEBUG 2
#define DRIVER_VERSION      1.0


#define SERIAL_BUFFER_SIZE 256
#define MAX_TIMEOUT 1000
#define LOG_BUFFER_SIZE 256

enum EFA_Errors     {PLUGIN_OK = 0, NOT_CONNECTED, PLUGIN_CANT_CONNECT, PLUGIN_BAD_CMD_RESPONSE, COMMAND_FAILED};
enum MotorDir       {NORMAL = 0 , REVERSE};
enum MotorStatus    {IDLE = 0, MOVING};

#define SOM         0x3B
#define PC          0x20
#define HC          0x0D
#define FOC_TEMP    0x12
#define ROT_FAN     0x13
#define DELTA_T     0x32

#define NUM 1
#define SRC 2
#define RCV 3
#define CMD 4


// temps source
#define PRIMARY 0
#define AMBIANT 1
#define SECONDARY 2

#define MTR_GET_POS                 0x01
#define MTR_GOTO_POS2               0x17
#define MTR_OFFSET_CNT              0x04
#define MTR_GOTO_OVER               0x13
#define MTR_PTRACK                  0x06
#define MTR_NTRACK                  0x07
#define MTR_SLEWLIMITMIN            0x1A
#define MTR_SLEWLIMITMAX            0x1B
#define MTR_SLEWLIMITGETMIN         0x1C
#define MTR_SLEWLIMITGETMAX         0x1D
#define MTR_PMSLEW_RATE             0x24
#define MTR_NMSLEW_RATE             0x25
#define TEMP_GET                    0x26
#define FANS_SET                    0x27
#define FANS_GET                    0x28
#define MTR_GET_CALIBRATION_STATE   0x30
#define MTR_SET_CALIBRATION_STATE   0x31
#define MTR_GET_STOP_DETECT         0xEE
#define MTR_STOP_DETECT             0xEF
#define MTR_GET_APPROACH_DIRECTION  0xFC
#define MTR_APPROACH_DIRECTION      0xFD
#define GET_VERSION                 0xFE

class CEFAController
{
public:
    CEFAController();
    ~CEFAController();

    int         Connect(const char *pszPort);
    void        Disconnect(void);
    bool        IsConnected(void) { return m_bIsConnected; };

    void        SetSerxPointer(SerXInterface *p) { m_pSerx = p; };
    void        setLogger(LoggerInterface *pLogger) { m_pLogger = pLogger; };
    void        setSleeper(SleeperInterface *pSleeper) { m_pSleeper = pSleeper; };

    // move commands
    int         haltFocuser();
    int         gotoPosition(int nPos);
    int         moveRelativeToPosision(int nSteps);

    // command complete functions
    int         isGoToComplete(bool &bComplete);
    int         isMotorMoving(bool &bMoving);

    // getter and setter
    void        setDebugLog(bool bEnable) {m_bDebugLog = bEnable; };

    int         getFirmwareVersion(char *pszVersion, int nStrMaxLen);
    int         getTemperature(double &dTemperature);
    int         getPosition(int &nPosition);
    int         syncMotorPosition(int nPos);
    int         getPosLimitMin(int &nPosLimit);
    int         getPosLimitMax(int &nPosLimit);
    int         setPosLimitMin(int nLimit);
    int         setPosLimitMax(int nLimit);

    int         setPositiveMotorSlewRate(int nRate);
    int         setNegativeMotorSlewRate(int nRate);
    int         trackPositiveMotorRate(int nRate);
    int         trackNegativeMotorRate(int nRate);
    int         trackAtMotorRate(int nRate);
    int         setFan(bool bOn);
    int         getFan(bool &bOn);
    int         setCalibrationState(bool bCalbrated);
    int         getCalibrationState(bool &bCalbrated);
    int         setStopDetect(bool bEnable);
    int         getStopDetect(bool &bEnable);
    int         setApproachDir(int nDir);
    int         getApproachDir(int &nDir);


protected:
    int             takeEFABus();
    int             releaseEFABus();
    
    int             EFACommand(const unsigned char *pszCmd, unsigned char *pszResult, int nResultMaxLen);
    int             readResponse(unsigned char *pszRespBuffer, int nBufferLen);
    unsigned char   checksum(const unsigned char *cMessage, int nLen);
    void            hexdump(const unsigned char* pszInputBuffer, unsigned char *pszOutputBuffer, int nInputBufferSize, int nOutpuBufferSize);
    
    
    bool            isClearToSendSerx(SerXInterface* pSerX);
    bool            isRequestToSendSerx(SerXInterface* pSerX);
    bool            setRequestToSendSerx(SerXInterface* pSerX, bool bSet);
    
    
    SerXInterface   *m_pSerx;
    LoggerInterface *m_pLogger;
    SleeperInterface    *m_pSleeper;

    bool            m_bDebugLog;
    bool            m_bIsConnected;
    char            m_szFirmwareVersion[SERIAL_BUFFER_SIZE];
    char            m_szLogBuffer[LOG_BUFFER_SIZE];

    int             m_nCurPos;
    int             m_nTargetPos;
    int             m_nPosLimitMin;
    int             m_nPosLimitMax;
    bool            m_bPosLimitEnabled;
    bool            m_bMoving;
    bool            m_bCalibrated;
    
#ifdef PLUGIN_DEBUG
    std::string m_sLogfilePath;
	// timestamp for logs
	char *timestamp;
	time_t ltime;
	FILE *Logfile;	  // LogFile
#endif

};

#endif //__EFA__
