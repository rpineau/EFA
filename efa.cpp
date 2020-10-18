//
//  efa.cpp
//  PlaneWave EFA X2 plugin
//
//  Created by Rodolphe Pineau on 3/25/2018.


#include "efa.h"


CEFAController::CEFAController()
{

    m_pSerx = NULL;
    m_pLogger = NULL;


    m_bDebugLog = false;
    m_bIsConnected = false;

    m_nCurPos = 0;
    m_nTargetPos = 0;
    m_nPosLimitMin = 0;
    m_nPosLimitMax = 0;
    m_bMoving = false;


#ifdef PLUGIN_DEBUG
#if defined(SB_WIN_BUILD)
    m_sLogfilePath = getenv("HOMEDRIVE");
    m_sLogfilePath += getenv("HOMEPATH");
    m_sLogfilePath += "\\EFALog.txt";
#elif defined(SB_LINUX_BUILD)
    m_sLogfilePath = getenv("HOME");
    m_sLogfilePath += "/EFALog.txt";
#elif defined(SB_MAC_BUILD)
    m_sLogfilePath = getenv("HOME");
    m_sLogfilePath += "/EFALog.txt";
#endif
    Logfile = fopen(m_sLogfilePath.c_str(), "w");
#endif

    
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CEFAController New Constructor Called\n", timestamp);
    fprintf(Logfile, "[%s] [CEFAController::CEFAController] version %3.2f build 2020_08_26_1350.\n", timestamp, DRIVER_VERSION);
    fflush(Logfile);
#endif
}

CEFAController::~CEFAController()
{
#ifdef	PLUGIN_DEBUG
    // Close LogFile
    if (Logfile) fclose(Logfile);
#endif
}

int CEFAController::Connect(const char *pszPort)
{
    int nErr = PLUGIN_OK;
    
    if(!m_pSerx)
        return ERR_COMMNOLINK;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] [CEFAController::Connect] Called with port %s\n", timestamp, pszPort);
	fflush(Logfile);
#endif

    // 19200 8N1
    nErr = m_pSerx->open(pszPort, 19200, SerXInterface::B_NOPARITY, "-DTR_CONTROL 1");
    if(nErr) {
        m_bIsConnected = false;
        return nErr;
    }
    m_bIsConnected = true;
    releaseEFABus();
    
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CEFAController::Connect] connected to %s\n", timestamp, pszPort);
    fprintf(Logfile, "[%s] [CEFAController::Connect] Getting Firmware.\n", timestamp);
    fflush(Logfile);
#endif
    
    nErr = getFirmwareVersion(m_szFirmwareVersion, SERIAL_BUFFER_SIZE);
    if(nErr) {
        Disconnect();
        nErr = ERR_COMMNOLINK;
    }
    return nErr;
}

void CEFAController::Disconnect()
{
    if(m_bIsConnected && m_pSerx)
        m_pSerx->close();
 
	m_bIsConnected = false;
}

#pragma mark move commands
int CEFAController::haltFocuser()
{
    int nErr = PLUGIN_OK;

    if(!m_bIsConnected)
		return ERR_COMMNOLINK;

    nErr = setPositiveMotorSlewRate(0); // stop
    m_nTargetPos = m_nCurPos;

    return nErr;
}

int CEFAController::gotoPosition(int nPos)
{
    int nErr = PLUGIN_OK;
    unsigned char szCmd[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    memset(szCmd,0, SERIAL_BUFFER_SIZE);
    szCmd[0] = SOM;
    szCmd[NUM] = 6;
    szCmd[SRC] = PC;
    szCmd[RCV] = FOC_TEMP;
    szCmd[CMD] = MTR_GOTO_POS2;
    szCmd[5] = (nPos & 0x00FF0000)>>16;
    szCmd[6] = (nPos & 0x0000FF00)>>8;
    szCmd[7] = (nPos & 0x000000FF);
    szCmd[8] = checksum(szCmd+1, szCmd[NUM]+1);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CEFAController::gotoPosition goto position  : %d\n", timestamp, nPos);
    fflush(Logfile);
#endif

    nErr = EFACommand(szCmd, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    m_nTargetPos = nPos;

    return nErr;
}

int CEFAController::moveRelativeToPosision(int nSteps)
{
    int nErr;

	if(!m_bIsConnected)
		return ERR_COMMNOLINK;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CEFAController::gotoPosition] goto relative position  : %d\n", timestamp, nSteps);
    fflush(Logfile);
#endif

    m_nTargetPos = m_nCurPos + nSteps;
    nErr = gotoPosition(m_nTargetPos);
    return nErr;
}

#pragma mark command complete functions

int CEFAController::isGoToComplete(bool &bComplete)
{
    int nErr = PLUGIN_OK;
	
	if(!m_bIsConnected)
		return ERR_COMMNOLINK;

    nErr = getPosition(m_nCurPos);
    if(nErr)
        return nErr;

    if(m_nCurPos == m_nTargetPos)
        bComplete = true;
    else
        bComplete = false;
    return nErr;
}

int CEFAController::isMotorMoving(bool &bMoving)
{
    int nErr = PLUGIN_OK;
    unsigned char szCmd[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    memset(szCmd,0, SERIAL_BUFFER_SIZE);
    // Goto Position = 0x140000 = 1310720
    szCmd[0] = SOM;
    szCmd[NUM] = 3;
    szCmd[SRC] = PC;
    szCmd[RCV] = FOC_TEMP;
    szCmd[CMD] = MTR_GOTO_OVER;
    szCmd[5] = checksum(szCmd+1, szCmd[NUM]+1);

    nErr = EFACommand(szCmd, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    if(szResp[5] == 0xFF) // FF = goto is over.
        m_bMoving = false;
    else
        m_bMoving = true;

    bMoving = m_bMoving;

    return nErr;
}

#pragma mark getters and setters

int CEFAController::getFirmwareVersion(char *pszVersion, int nStrMaxLen)
{
    int nErr = PLUGIN_OK;
    unsigned char szCmd[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];

	if(!m_bIsConnected)
		return ERR_COMMNOLINK;

    memset(szCmd,0, SERIAL_BUFFER_SIZE);
    // 3B 03 20 12 FE CD
    szCmd[0] = SOM;
    szCmd[NUM] = 3;
    szCmd[SRC] = PC;
    szCmd[RCV] = FOC_TEMP;
    szCmd[CMD] = GET_VERSION;
    szCmd[5] = checksum(szCmd+1, szCmd[NUM]+1);

    nErr = EFACommand(szCmd, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CEFAController::getFirmwareVersion] version : %d.%d\n", timestamp, szResp[5] , szResp[6]);
    fflush(Logfile);
#endif

    snprintf(pszVersion, nStrMaxLen, "%d.%d", szResp[5] , szResp[6]);

    return nErr;
}

int CEFAController::getTemperature(double &dTemperature)
{
    int nErr = PLUGIN_OK;
    unsigned char szCmd[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];
    int rawTemp;

    if(!m_bIsConnected)
		return ERR_COMMNOLINK;

    memset(szCmd,0, SERIAL_BUFFER_SIZE);
    szCmd[0] = SOM;
    szCmd[NUM] = 4;
    szCmd[SRC] = PC;
    szCmd[RCV] = FOC_TEMP;
    szCmd[CMD] = TEMP_GET;
    szCmd[5] = AMBIANT;
    szCmd[6] = checksum(szCmd+1, szCmd[NUM]+1);

    nErr = EFACommand(szCmd, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    rawTemp = szResp[5] + szResp[6]*256;
    if(rawTemp & 0x8000)
        rawTemp = rawTemp - 0x10000;
    
    dTemperature = double(rawTemp)/16.0;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CEFAController::getTemperature] szResp[5] : %02x\n", timestamp, szResp[5]);
    fprintf(Logfile, "[%s] [CEFAController::getTemperature] szResp[6] : %02x\n", timestamp, szResp[6]);
    fprintf(Logfile, "[%s] [CEFAController::getTemperature] dTemperature : %5.2f\n", timestamp, dTemperature);
    fflush(Logfile);
#endif

    return nErr;
}

int CEFAController::getPosition(int &nPosition)
{
    int nErr = PLUGIN_OK;
    unsigned char szCmd[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    memset(szCmd,0, SERIAL_BUFFER_SIZE);
    szCmd[0] = SOM;
    szCmd[NUM] = 3;
    szCmd[SRC] = PC;
    szCmd[RCV] = FOC_TEMP;
    szCmd[CMD] = MTR_GET_POS;
    szCmd[5] = checksum(szCmd+1, szCmd[NUM]+1);

    nErr = EFACommand(szCmd, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert response
    nPosition = (szResp[5]<<16) + (szResp[6]<<8) + szResp[7];
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CEFAController::getPosition] nPosition = %d\n", timestamp, nPosition);
    fflush(Logfile);
#endif

    m_nCurPos = nPosition;

    return nErr;
}


int CEFAController::syncMotorPosition(int nPos)
{
    int nErr = PLUGIN_OK;
    unsigned char szCmd[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    memset(szCmd,0, SERIAL_BUFFER_SIZE);
    szCmd[0] = SOM;
    szCmd[NUM] = 6;
    szCmd[SRC] = PC;
    szCmd[RCV] = FOC_TEMP;
    szCmd[CMD] = MTR_OFFSET_CNT;
    szCmd[5] = (nPos & 0x00FF0000)>>16;
    szCmd[6] = (nPos & 0x0000FF00)>>8;
    szCmd[7] = (nPos & 0x000000FF);
    szCmd[8] = checksum(szCmd+1, szCmd[NUM]+1);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CEFAController::syncMotorPosition] new position  : %d\n", timestamp, nPos);
    fflush(Logfile);
#endif

    nErr = EFACommand(szCmd, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    m_nCurPos = nPos;
    return nErr;
}

int CEFAController::getPosLimitMin(int &nPosLimit)
{
    int nErr = PLUGIN_OK;
    unsigned char szCmd[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    memset(szCmd,0, SERIAL_BUFFER_SIZE);
    szCmd[0] = SOM;
    szCmd[NUM] = 3;
    szCmd[SRC] = PC;
    szCmd[RCV] = FOC_TEMP;
    szCmd[CMD] = MTR_SLEWLIMITGETMIN;
    szCmd[5] = checksum(szCmd+1, szCmd[NUM]+1);

    nErr = EFACommand(szCmd, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert response
    m_nPosLimitMin = (szResp[5]<<16) + (szResp[6]<<8) + szResp[7];
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CEFAController::getPosLimitMin] m_nPosLimitMin = %d\n", timestamp, m_nPosLimitMin);
    fflush(Logfile);
#endif
    nPosLimit = m_nPosLimitMin;
    return nErr;
}

int CEFAController::getPosLimitMax(int &nPosLimit)
{
    int nErr = PLUGIN_OK;
    unsigned char szCmd[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    memset(szCmd,0, SERIAL_BUFFER_SIZE);
    szCmd[0] = SOM;
    szCmd[NUM] = 3;
    szCmd[SRC] = PC;
    szCmd[RCV] = FOC_TEMP;
    szCmd[CMD] = MTR_SLEWLIMITGETMAX;
    szCmd[5] = checksum(szCmd+1, szCmd[NUM]+1);

    nErr = EFACommand(szCmd, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert response
    m_nPosLimitMax = (szResp[5]<<16) + (szResp[6]<<8) + szResp[7];
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CEFAController::getPosLimitMax] m_nPosLimitMax = %d\n", timestamp, m_nPosLimitMax);
    fflush(Logfile);
#endif
    nPosLimit = m_nPosLimitMax;
    return nErr;
}


int CEFAController::setPosLimitMin(int nLimit)
{
    int nErr = PLUGIN_OK;
    unsigned char szCmd[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    memset(szCmd,0, SERIAL_BUFFER_SIZE);
    szCmd[0] = SOM;
    szCmd[NUM] = 6;
    szCmd[SRC] = PC;
    szCmd[RCV] = FOC_TEMP;
    szCmd[CMD] = MTR_SLEWLIMITMIN;
    szCmd[5] = (nLimit & 0x00FF0000)>>16;
    szCmd[6] = (nLimit & 0x0000FF00)>>8;
    szCmd[7] = (nLimit & 0x000000FF);
    szCmd[8] = checksum(szCmd+1, szCmd[NUM]+1);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CEFAController::setPosLimitMin] new limit : %d\n", timestamp, nLimit);
    fflush(Logfile);
#endif

    nErr = EFACommand(szCmd, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    m_nPosLimitMin = nLimit;
    return nErr;
}

int CEFAController::setPosLimitMax(int nLimit)
{
    int nErr = PLUGIN_OK;
    unsigned char szCmd[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    memset(szCmd,0, SERIAL_BUFFER_SIZE);
    szCmd[0] = SOM;
    szCmd[NUM] = 6;
    szCmd[SRC] = PC;
    szCmd[RCV] = FOC_TEMP;
    szCmd[CMD] = MTR_SLEWLIMITMAX;
    szCmd[5] = (nLimit & 0x00FF0000)>>16;
    szCmd[6] = (nLimit & 0x0000FF00)>>8;
    szCmd[7] = (nLimit & 0x000000FF);
    szCmd[8] = checksum(szCmd+1, szCmd[NUM]+1);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CEFAController::setPosLimitMax] new limit : %d\n", timestamp, nLimit);
    fflush(Logfile);
#endif

    nErr = EFACommand(szCmd, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    m_nPosLimitMax = nLimit;
    return nErr;
}

int CEFAController::setPositiveMotorSlewRate(int nRate)
{
    int nErr = PLUGIN_OK;
    unsigned char szCmd[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    if(nRate > 9)
        return ERR_CMDFAILED;

    memset(szCmd,0, SERIAL_BUFFER_SIZE);

    szCmd[0] = SOM;
    szCmd[NUM] = 4;
    szCmd[SRC] = PC;
    szCmd[RCV] = FOC_TEMP;
    szCmd[CMD] = MTR_PMSLEW_RATE;
    szCmd[5] = (unsigned char) nRate; // 0 to 9
    szCmd[6] = checksum(szCmd+1, szCmd[NUM]+1);

    nErr = EFACommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int CEFAController::setNegativeMotorSlewRate(int nRate)
{
    int nErr = PLUGIN_OK;
    unsigned char szCmd[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    memset(szCmd,0, SERIAL_BUFFER_SIZE);

    szCmd[0] = SOM;
    szCmd[NUM] = 4;
    szCmd[SRC] = PC;
    szCmd[RCV] = FOC_TEMP;
    szCmd[CMD] = MTR_NMSLEW_RATE;
    szCmd[5] = (unsigned char) nRate; // 0 to 9
    szCmd[6] = checksum(szCmd+1, szCmd[NUM]+1);

    nErr = EFACommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int CEFAController::setFan(bool bOn)
{
    int nErr = PLUGIN_OK;
    unsigned char szCmd[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    memset(szCmd,0, SERIAL_BUFFER_SIZE);

    szCmd[0] = SOM;
    szCmd[NUM] = 4;
    szCmd[SRC] = PC;
    szCmd[RCV] = ROT_FAN;
    szCmd[CMD] = FANS_SET;
    szCmd[5] = bOn?0x01:0x00;
    szCmd[6] = checksum(szCmd+1, szCmd[NUM]+1);

    nErr = EFACommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int CEFAController::getFan(bool &bOn)
{
    int nErr = PLUGIN_OK;
    unsigned char szCmd[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    bOn = false;
    memset(szCmd,0, SERIAL_BUFFER_SIZE);

    szCmd[0] = SOM;
    szCmd[NUM] = 3;
    szCmd[SRC] = PC;
    szCmd[RCV] = ROT_FAN;
    szCmd[CMD] = FANS_GET;
    szCmd[5] = checksum(szCmd+1, szCmd[NUM]+1);

    nErr = EFACommand(szCmd, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    if(szResp[5] == 0)
        bOn = true;

    return nErr;
}

int CEFAController::setCalibrationState(bool bCalbrated)
{
    int nErr = PLUGIN_OK;
    unsigned char szCmd[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    memset(szCmd,0, SERIAL_BUFFER_SIZE);

    szCmd[0] = SOM;
    szCmd[NUM] = 5;
    szCmd[SRC] = PC;
    szCmd[RCV] = FOC_TEMP;
    szCmd[CMD] = MTR_SET_CALIBRATION_STATE;
    szCmd[5] = 0x40;
    szCmd[6] = bCalbrated?0x01:0x00;
    szCmd[7] = checksum(szCmd+1, szCmd[NUM]+1);

    nErr = EFACommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int CEFAController::getCalibrationState(bool &bCalbrated)
{
    int nErr = PLUGIN_OK;
    unsigned char szCmd[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    bCalbrated = false;

    memset(szCmd,0, SERIAL_BUFFER_SIZE);

    szCmd[0] = SOM;
    szCmd[NUM] = 4;
    szCmd[SRC] = PC;
    szCmd[RCV] = FOC_TEMP;
    szCmd[CMD] = MTR_GET_CALIBRATION_STATE;
    szCmd[5] = 0x40;
    szCmd[6] = checksum(szCmd+1, szCmd[NUM]+1);

    nErr = EFACommand(szCmd, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    if(szResp[5] == 0x01)
        bCalbrated = true;

    return nErr;
}

int CEFAController::setStopDetect(bool bEnable)
{
    int nErr = PLUGIN_OK;
    unsigned char szCmd[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    memset(szCmd,0, SERIAL_BUFFER_SIZE);

    szCmd[0] = SOM;
    szCmd[NUM] = 4;
    szCmd[SRC] = PC;
    szCmd[RCV] = FOC_TEMP;
    szCmd[CMD] = MTR_STOP_DETECT;
    szCmd[5] = bEnable?0x01:0x00;
    szCmd[6] = checksum(szCmd+1, szCmd[NUM]+1);

    nErr = EFACommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int CEFAController::getStopDetect(bool &bEnable)
{
    int nErr = PLUGIN_OK;
    unsigned char szCmd[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    bEnable = false;
    memset(szCmd,0, SERIAL_BUFFER_SIZE);

    szCmd[0] = SOM;
    szCmd[NUM] = 3;
    szCmd[SRC] = PC;
    szCmd[RCV] = FOC_TEMP;
    szCmd[CMD] = MTR_GET_STOP_DETECT;
    szCmd[5] = checksum(szCmd+1, szCmd[NUM]+1);

    nErr = EFACommand(szCmd, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    if(szResp[5] == 0x01)
        bEnable = true;

    return nErr;
}

int CEFAController::setApproachDir(int nDir)
{
    int nErr = PLUGIN_OK;
    unsigned char szCmd[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    if(nDir>1)
        return ERR_CMDFAILED;

    memset(szCmd,0, SERIAL_BUFFER_SIZE);

    szCmd[0] = SOM;
    szCmd[NUM] = 4;
    szCmd[SRC] = PC;
    szCmd[RCV] = FOC_TEMP;
    szCmd[CMD] = MTR_APPROACH_DIRECTION;
    szCmd[5] = (unsigned char)nDir;
    szCmd[6] = checksum(szCmd+1, szCmd[NUM]+1);

    nErr = EFACommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int CEFAController::getApproachDir(int &nDir)
{
    int nErr = PLUGIN_OK;
    unsigned char szCmd[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    memset(szCmd,0, SERIAL_BUFFER_SIZE);

    szCmd[0] = SOM;
    szCmd[NUM] = 3;
    szCmd[SRC] = PC;
    szCmd[RCV] = FOC_TEMP;
    szCmd[CMD] = MTR_GET_APPROACH_DIRECTION;
    szCmd[5] = checksum(szCmd+1, szCmd[NUM]+1);

    nErr = EFACommand(szCmd, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    nDir = szResp[5];

    return nErr;
}


#pragma mark command and response functions

int CEFAController::takeEFABus()
{
    int nErr = PLUGIN_OK;
    int nTimeout = 0;
    bool bCTS;

    #if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CEFAController::takeEFABus] taking the bus\n", timestamp);
            fflush(Logfile);
    #endif

    // wait for CTS to be clear
    while(true) {
        bCTS = isClearToSendSerx(m_pSerx);
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CEFAController::takeEFABus] bCTS  = %s\n", timestamp, bCTS?"True":"False");
        fflush(Logfile);
#endif
        if(!bCTS) // aka if CTS is false we can take the bus
            break;
        nTimeout++;
        if(nTimeout>100) {
            return ERR_CMDFAILED;
        }
        m_pSleeper->sleep(100);
    }
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CEFAController::takeEFABus] bCTS  = %s\n", timestamp, bCTS?"True":"False");
        fflush(Logfile);
#endif

    // set RTS to true -> we're taking the bus
    nErr = setRequestToSendSerx(m_pSerx, true);
    return nErr;
}

int CEFAController::releaseEFABus()
{
    int nErr = PLUGIN_OK;
    #if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CEFAController::releaseEFABus] releasing the bus\n", timestamp);
            fflush(Logfile);
    #endif

    // set RTS to false -> we're releasing the bus
    nErr = setRequestToSendSerx(m_pSerx, false);
    return nErr;
}


int CEFAController::EFACommand(const unsigned char *pszCmd, unsigned char *pszResult, int nResultMaxLen)
{
    int nErr = PLUGIN_OK;
    unsigned char szResp[SERIAL_BUFFER_SIZE];
    unsigned long  ulBytesWrite;
    unsigned char cHexMessage[LOG_BUFFER_SIZE];
    int i;
    
    if(!m_bIsConnected)
		return ERR_COMMNOLINK;

    nErr = takeEFABus();
    m_pSerx->purgeTxRx();
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
    hexdump(pszCmd, cHexMessage, pszCmd[NUM]+3, LOG_BUFFER_SIZE);
    fprintf(Logfile, "[%s] [CEFAController::EFACommand] Sending %s\n", timestamp, cHexMessage);
	fflush(Logfile);
#endif
    // write packet
    nErr = m_pSerx->writeFile((void *)pszCmd, pszCmd[NUM]+3, ulBytesWrite);
    m_pSerx->flushTx();

    if(nErr) {
        // set 0 to false -> we're releasing the bus
        releaseEFABus();
        return nErr;
    }

    // read command echo
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CEFAController::EFACommand] reading command echo.\n", timestamp);
        fflush(Logfile);
#endif
    nErr = readResponse(szResp, SERIAL_BUFFER_SIZE);
    releaseEFABus();

    
    // The EFA always respond even if no data is expected
    i = 0;
    while(true) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CEFAController::EFACommand] [%d] waiting for response.\n", timestamp, i++);
        fflush(Logfile);
#endif
        nErr = readResponse(szResp, SERIAL_BUFFER_SIZE);
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        hexdump(szResp, cHexMessage, szResp[1]+3, LOG_BUFFER_SIZE);
        fprintf(Logfile, "[%s] [CEFAController::EFACommand] response \"%s\"\n", timestamp, cHexMessage);
        fflush(Logfile);
#endif
        if(nErr) {
            return nErr;
        }
        // if we  expect a response and get a packet but we're not the receicer .. try to read another response, 3 times max
        if(pszResult && szResp[RCV] != PC && i<3) {
            continue;
        }
        else
            break;
    }

    if(pszResult) {
        memset(pszResult,0, nResultMaxLen);
        memcpy(pszResult, szResp, szResp[1]+3);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        hexdump(pszResult, cHexMessage, pszResult[1]+3, LOG_BUFFER_SIZE);
        fprintf(Logfile, "[%s] [CEFAController::EFACommand] response copied to pszResult : \"%s\"\n", timestamp, cHexMessage);
        fflush(Logfile);
#endif
        }
    return nErr;
}

int CEFAController::readResponse(unsigned char *pszRespBuffer, int nBufferLen)
{
    int nErr = PLUGIN_OK;
    unsigned long ulBytesRead = 0;
    int nLen = SERIAL_BUFFER_SIZE;
    unsigned char cHexMessage[LOG_BUFFER_SIZE];
    unsigned char cChecksum;

    if(!m_bIsConnected)
		return ERR_COMMNOLINK;

    memset(pszRespBuffer, 0, (size_t) nBufferLen);

    // Look for a SOM starting character, until timeout occurs
    while (*pszRespBuffer != SOM && nErr == PLUGIN_OK) {
        nErr = m_pSerx->readFile(pszRespBuffer, 1, ulBytesRead, MAX_TIMEOUT);
        if (ulBytesRead !=1) // timeout
            nErr = PLUGIN_BAD_CMD_RESPONSE;
    }

    if(*pszRespBuffer != SOM || nErr != PLUGIN_OK)
        return ERR_CMDFAILED;

    // Read message length
    nErr = m_pSerx->readFile(pszRespBuffer + 1, 1, ulBytesRead, MAX_TIMEOUT);
    if (nErr != PLUGIN_OK || ulBytesRead!=1)
        return ERR_CMDFAILED;

    nLen = pszRespBuffer[1];
    if(!nLen) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CEFAController::readResponse] Error reading response (no data from EFA), nLen =  %d\n", timestamp, nLen);
#endif
        return ERR_CMDFAILED;
    }

    // Read the rest of the message
    nErr = m_pSerx->readFile(pszRespBuffer + 2, nLen + 1, ulBytesRead, MAX_TIMEOUT); // the +1 on nLen is to also read the checksum
    if(nErr || (ulBytesRead != nLen+1)) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        hexdump(pszRespBuffer, cHexMessage, pszRespBuffer[1]+3, LOG_BUFFER_SIZE);
        fprintf(Logfile, "[%s] [CEFAController::readResponse] error\n", timestamp);
        fprintf(Logfile, "[%s] [CEFAController::readResponse] ulBytesRead = %lu\n", timestamp, ulBytesRead);
        fprintf(Logfile, "[%s] [CEFAController::readResponse] nLen =  %d\n", timestamp, nLen);
        fprintf(Logfile, "[%s] [CEFAController::readResponse] got %s\n", timestamp, cHexMessage);
        fflush(Logfile);
#endif
        return ERR_CMDFAILED;
    }

    // verify checksum
    cChecksum = checksum(pszRespBuffer+1, nLen+1);
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    hexdump(pszRespBuffer, cHexMessage, pszRespBuffer[1]+3, LOG_BUFFER_SIZE);
    fprintf(Logfile, "[%s] [CEFAController::readResponse] got %s\n", timestamp, cHexMessage);
    fprintf(Logfile, "[%s] [CEFAController::readResponse] nLen =  %d\n", timestamp, nLen);
    fprintf(Logfile, "[%s] [CEFAController::readResponse] Checksum : calculated checksum is %02X, message checksum is %02X\n", timestamp, cChecksum, *(pszRespBuffer+nLen+2) );
    fflush(Logfile);
#endif

    if (cChecksum  != *(pszRespBuffer+nLen+2) && *(pszRespBuffer+nLen+2) != 0x00) { // echoed packet have a checksum of 00 apparently.
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        hexdump(pszRespBuffer, cHexMessage, pszRespBuffer[1]+3, LOG_BUFFER_SIZE);
        fprintf(Logfile, "[%s] [CEFAController::readResponse] WTF !!! Checksum error : calculated checksum is %02X, message checksum is %02X\n", timestamp, cChecksum, *(pszRespBuffer+nLen+2) );
        fflush(Logfile);
#endif
        nErr = ERR_CMDFAILED;
    }
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    hexdump(pszRespBuffer, cHexMessage, pszRespBuffer[1]+3, LOG_BUFFER_SIZE);
    fprintf(Logfile, "[%s] [CEFAController::readResponse] response \"%s\"\n", timestamp, cHexMessage);
    fflush(Logfile);
#endif

    return nErr;
}

unsigned char CEFAController::checksum(const unsigned char *cMessage, int nLen)
{
    int nIdx;
    char cChecksum = 0;

    for (nIdx = 0; nIdx < nLen && nIdx < SERIAL_BUFFER_SIZE; nIdx++) {
        cChecksum -= cMessage[nIdx];
    }
    return (unsigned char)cChecksum;
}

void CEFAController::hexdump(const unsigned char* pszInputBuffer, unsigned char *pszOutputBuffer, int nInputBufferSize, int nOutpuBufferSize)
{
    unsigned char *pszBuf = pszOutputBuffer;
    int nIdx=0;

    memset(pszOutputBuffer, 0, nOutpuBufferSize);
    for(nIdx=0; nIdx < nInputBufferSize && pszBuf < (pszOutputBuffer + nOutpuBufferSize -3); nIdx++){
        snprintf((char *)pszBuf,4,"%02X ", pszInputBuffer[nIdx]);
        pszBuf+=3;
    }
}



/*
The isRequestToSendSerx() and setRequestToSendSerx() and isClearToSend() methods provide a means to read and set the "request to send" pin on a serial port high and subsequently
low using the SerXInterface provided to x2 drivers as well as query the state of the clear to send pin.

The isRequestToSendSerx() returns immedately if the RTS line is high or low, returning 1 or 0 respectively.

The isClearToSendSerx() returns immedately if the CTS line is high or low, returning 1 or 0 respectively.

The setRequestToSendSerx() method returns immedately if the request succeeded or failed, returning 1 or 0 respectively.

The serial port must be already be open.

These methods are for hardware (like the Planewave EFA) that uses the RTS/CTS line in a non conventional manner to request access the their shared internal serial bus (they us an internal I2C bus).

The one who calls setRequestToSendSerx() is responsible for first looping (and perhaps eventually timing out) while reading isClearToSendSerx() until it is high to gain access to the shared port.
Beware, because the set is not atomic and exclusive, there will surely be extreme cases where two clients "get in" and the serial send/receive will be wrong/mixed/garbled/etc.  At minimum, programming with retries is a must.

Important note, TheSky build 12669 or later is required in order for isRequestToSend() and setRequestToSendSerx() to work.
*/

bool CEFAController::isClearToSendSerx(SerXInterface* pSerX)
{
    int nSpecialNumber = -5002;

    if (NULL == pSerX)
        return false;

    if (!pSerX->isConnected())
        return false;

    return pSerX->waitForBytesRx(nSpecialNumber, 0);//Work around to KISS, hi jack the existing method waitForBytesRx() to call the internal implemenation of isClearToSend diffentiated by nSpecialNumber
}

bool CEFAController::isRequestToSendSerx(SerXInterface* pSerX)
{
    int nSpecialNumber = -5001;

    if (NULL == pSerX)
        return false;

    if (!pSerX->isConnected())
        return false;

    return pSerX->waitForBytesRx(nSpecialNumber, 0);//Work around to KISS, hi jack the existing method waitForBytesRx() to call the internal implemenation of isRequestToSend diffentiated by nSpecialNumber
}

bool CEFAController::setRequestToSendSerx(SerXInterface* pSerX, bool bSet)
{
    int nSpecialNumber = -5000;

    if (NULL == pSerX)
        return false;

    if (!pSerX->isConnected())
        return false;

    return pSerX->waitForBytesRx(nSpecialNumber, bSet);//Work around to KISS, hi jack the existing method waitForBytesRx() to call the internal implemenation of setRequestToSend diffentiated by nSpecialNumber
}
