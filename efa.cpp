//
//  nexdome.cpp
//  NexDome X2 plugin
//
//  Created by Rodolphe Pineau on 6/11/2016.


#include "efa.h"
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <memory.h>
#include <string.h>
#ifdef SB_MAC_BUILD
#include <unistd.h>
#endif
#ifdef SB_WIN_BUILD
#include <time.h>
#endif


CEFAController::CEFAController()
{

    m_pSerx = NULL;
    m_pLogger = NULL;


    m_bDebugLog = false;
    m_bIsConnected = false;

    m_nCurPos = 0;
    m_nTargetPos = 0;
    m_nPosLimit = 0;
    m_bMoving = false;


#ifdef EFA_DEBUG
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

#if defined EFA_DEBUG && EFA_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CEFAController Constructor Called\n", timestamp);
    fflush(Logfile);
#endif

}

CEFAController::~CEFAController()
{
#ifdef	EFA_DEBUG
    // Close LogFile
    if (Logfile) fclose(Logfile);
#endif
}

int CEFAController::Connect(const char *pszPort)
{
    int nErr = EFA_OK;

    if(!m_pSerx)
        return ERR_COMMNOLINK;

#if defined EFA_DEBUG && EFA_DEBUG >= 2
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] CEFAController::Connect Called %s\n", timestamp, pszPort);
	fflush(Logfile);
#endif

    // 19200 8N1
    if(m_pSerx->open(pszPort, 19200, SerXInterface::B_NOPARITY, "-DTR_CONTROL 1 -RTS_CONTROL 1") == 0)
        m_bIsConnected = true;
    else
        m_bIsConnected = false;

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    m_pSleeper->sleep(2000);

#if defined EFA_DEBUG && EFA_DEBUG >= 2
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] CEFAController::Connect connected to %s\n", timestamp, pszPort);
	fflush(Logfile);
#endif
	
    if (m_bDebugLog && m_pLogger) {
        snprintf(m_szLogBuffer,LOG_BUFFER_SIZE,"[CEFAController::Connect] Connected.\n");
        m_pLogger->out(m_szLogBuffer);

        snprintf(m_szLogBuffer,LOG_BUFFER_SIZE,"[CEFAController::Connect] Getting Firmware.\n");
        m_pLogger->out(m_szLogBuffer);
    }

    nErr = getFirmwareVersion(m_szFirmwareVersion, SERIAL_BUFFER_SIZE);
    if(nErr)
        nErr = ERR_COMMNOLINK;
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
    int nErr = EFA_OK;
    unsigned char szResp[SERIAL_BUFFER_SIZE];
    unsigned char szTmpBuf[SERIAL_BUFFER_SIZE];

	if(!m_bIsConnected)
		return ERR_COMMNOLINK;

    // nErr = EFACommand("H#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    // parse output to update m_curPos
    // m_nCurPos = atoi(szTmpBuf);
    m_nTargetPos = m_nCurPos;

    return nErr;
}

int CEFAController::gotoPosition(int nPos)
{
    int nErr = EFA_OK;
    unsigned char szCmd[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];
    unsigned char cHexMessage[LOG_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    memset(szCmd,0, SERIAL_BUFFER_SIZE);
    szCmd[0] = SOM;
    szCmd[1] = 6;
    szCmd[2] = PC;
    szCmd[3] = FOC;
    szCmd[4] = MTR_GOTO_POS2;
    szCmd[5] = (nPos & 0x00FF0000)>>16;
    szCmd[6] = (nPos & 0x0000FF00)>>8;
    szCmd[7] = (nPos & 0x000000FF);
    szCmd[8] = checksum(szCmd+1, szCmd[1]+1);

#if defined EFA_DEBUG && EFA_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    hexdump(szCmd, cHexMessage, szCmd[1]+2, LOG_BUFFER_SIZE);
    fprintf(Logfile, "[%s] CEFAController::gotoPosition goto position : %d\n", timestamp, nPos);
    fprintf(Logfile, "[%s] CEFAController::gotoPosition Sending       : %s\n", timestamp, cHexMessage);
    fflush(Logfile);
#endif

#if defined EFA_DEBUG && EFA_DEBUG >= 2
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

#if defined EFA_DEBUG && EFA_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CEFAController::gotoPosition goto relative position  : %d\n", timestamp, nSteps);
    fflush(Logfile);
#endif

    m_nTargetPos = m_nCurPos + nSteps;
    nErr = gotoPosition(m_nTargetPos);
    return nErr;
}

#pragma mark command complete functions

int CEFAController::isGoToComplete(bool &bComplete)
{
    int nErr = EFA_OK;
	
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
    int nErr = EFA_OK;
    unsigned char szCmd[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    memset(szCmd,0, SERIAL_BUFFER_SIZE);
    // Goto Position = 0x140000 = 1310720
    szCmd[0] = SOM;
    szCmd[1] = 3;
    szCmd[2] = PC;
    szCmd[3] = FOC;
    szCmd[4] = MTR_GOTO_OVER;
    szCmd[5] = checksum(szCmd+1, szCmd[1]+1);

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
    int nErr = EFA_OK;
    unsigned char szCmd[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];
    unsigned char cHexMessage[LOG_BUFFER_SIZE];

	if(!m_bIsConnected)
		return ERR_COMMNOLINK;

    memset(szCmd,0, SERIAL_BUFFER_SIZE);
    // 3B 03 20 12 FE CD
    szCmd[0] = SOM;
    szCmd[1] = 3;
    szCmd[2] = PC;
    szCmd[3] = FOC;
    szCmd[4] = GET_VERSION;
    szCmd[5] = checksum(szCmd+1, szCmd[1]+1);

#if defined EFA_DEBUG && EFA_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    hexdump(szCmd, cHexMessage, szCmd[1]+3, LOG_BUFFER_SIZE);
    fprintf(Logfile, "[%s] CEFAController::getFirmwareVersion Sending %s\n", timestamp, cHexMessage);
    fflush(Logfile);
#endif

    nErr = EFACommand(szCmd, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

#if defined EFA_DEBUG && EFA_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CEFAController::getFirmwareVersion szResp : %s\n", timestamp, szResp);
    fflush(Logfile);
#endif

    snprintf(pszVersion, nStrMaxLen,"%d.%d", szResp[5] , szResp[6]);
    return nErr;
}

int CEFAController::getTemperature(double &dTemperature)
{
    int nErr = EFA_OK;
    unsigned char szCmd[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];
    int rawTemp;
    bool tempIsNeg;
    int intPart;
    int fractionDigit;

	if(!m_bIsConnected)
		return ERR_COMMNOLINK;

    memset(szCmd,0, SERIAL_BUFFER_SIZE);
    szCmd[0] = SOM;
    szCmd[1] = 4;
    szCmd[2] = PC;
    szCmd[3] = FOC;
    szCmd[4] = TEMP_GET;
    szCmd[5] = AMBIANT;
    szCmd[6] = checksum(szCmd+1, szCmd[1]+1);

#if defined EFA_DEBUG && EFA_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CEFAController::getTemperature Sending %s\n", timestamp, szCmd);
    fflush(Logfile);
#endif

    nErr = EFACommand(szCmd, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    rawTemp = szResp[5]*256 + szResp[6];
    tempIsNeg = false;
    if(rawTemp > 32768){
        tempIsNeg = true;
        rawTemp = 65536 - rawTemp;
    }
    intPart = rawTemp / 16;
    fractionDigit = (rawTemp - intPart) * 625 / 1000;
    dTemperature = intPart + fractionDigit / 10;
    if(tempIsNeg)
        dTemperature = -dTemperature;

    return nErr;
}

int CEFAController::getPosition(int &nPosition)
{
    int nErr = EFA_OK;
    unsigned char szCmd[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    memset(szCmd,0, SERIAL_BUFFER_SIZE);
    szCmd[0] = SOM;
    szCmd[1] = 3;
    szCmd[2] = PC;
    szCmd[3] = FOC;
    szCmd[4] = MTR_GET_POS;
    szCmd[5] = checksum(szCmd+1, szCmd[1]+1);

#if defined EFA_DEBUG && EFA_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CEFAController::getPosition Sending %s\n", timestamp, szCmd);
    fflush(Logfile);
#endif

    nErr = EFACommand(szCmd, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert response
    nPosition = (szResp[5]<<16) + (szResp[6]<<8) + szResp[7];
#if defined EFA_DEBUG && EFA_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CEFAController::getPosition nPosition = %d\n", timestamp, nPosition);
    fflush(Logfile);
#endif

    m_nCurPos = nPosition;

    return nErr;
}


int CEFAController::syncMotorPosition(int nPos)
{
    int nErr = EFA_OK;
    unsigned char szCmd[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    memset(szCmd,0, SERIAL_BUFFER_SIZE);
    szCmd[0] = SOM;
    szCmd[1] = 6;
    szCmd[2] = PC;
    szCmd[3] = FOC;
    szCmd[4] = MTR_OFFSET_CNT;
    szCmd[5] = (nPos & 0x00FF0000)>>16;
    szCmd[6] = (nPos & 0x0000FF00)>>8;
    szCmd[7] = (nPos & 0x000000FF);
    szCmd[8] = checksum(szCmd+1, szCmd[1]+1);

#if defined EFA_DEBUG && EFA_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CEFAController::syncMotorPosition new position : %d\n", timestamp, nPos);
    fprintf(Logfile, "[%s] CEFAController::syncMotorPosition Sending      : %s\n", timestamp, szCmd);
    fflush(Logfile);
#endif

#if defined EFA_DEBUG && EFA_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CEFAController::syncMotorPosition new position  : %d\n", timestamp, nPos);
    fflush(Logfile);
#endif

    nErr = EFACommand(szCmd, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    m_nCurPos = nPos;
    return nErr;
}



int CEFAController::getPosLimit(int &nPosLimit)
{
    int nErr = EFA_OK;
    unsigned char szCmd[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    memset(szCmd,0, SERIAL_BUFFER_SIZE);
    szCmd[0] = SOM;
    szCmd[1] = 3;
    szCmd[2] = PC;
    szCmd[3] = FOC;
    szCmd[4] = MTR_SLEWLIMITGETMAX;
    szCmd[5] = checksum(szCmd+1, szCmd[1]+1);

#if defined EFA_DEBUG && EFA_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CEFAController::getPosLimit Sending %s\n", timestamp, szCmd);
    fflush(Logfile);
#endif

    nErr = EFACommand(szCmd, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // convert response
    m_nPosLimit = (szResp[5]<<16) + (szResp[6]<<8) + szResp[7];
#if defined EFA_DEBUG && EFA_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CEFAController::getPosLimit m_nPosLimit = %d\n", timestamp, m_nPosLimit);
    fflush(Logfile);
#endif
    nPosLimit = m_nPosLimit;
    return nErr;
}

int CEFAController::setPosLimit(int nLimit)
{
    int nErr = EFA_OK;
    unsigned char szCmd[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    memset(szCmd,0, SERIAL_BUFFER_SIZE);
    szCmd[0] = SOM;
    szCmd[1] = 6;
    szCmd[2] = PC;
    szCmd[3] = FOC;
    szCmd[4] = MTR_SLEWLIMITMAX;
    szCmd[5] = (nLimit & 0x00FF0000)>>16;
    szCmd[6] = (nLimit & 0x0000FF00)>>8;
    szCmd[7] = (nLimit & 0x000000FF);
    szCmd[8] = checksum(szCmd+1, szCmd[1]+1);

#if defined EFA_DEBUG && EFA_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CEFAController::syncMotorPosition new limit : %d\n", timestamp, nLimit);
    fprintf(Logfile, "[%s] CEFAController::syncMotorPosition Sending   : %s\n", timestamp, szCmd);
    fflush(Logfile);
#endif

    nErr = EFACommand(szCmd, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    m_nPosLimit = nLimit;
    return nErr;
}

int CEFAController::setPositiveMotorSlewRate(int nRate)
{
    int nErr = EFA_OK;
    unsigned char szCmd[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    if(nRate > 9)
        return ERR_CMDFAILED;

    memset(szCmd,0, SERIAL_BUFFER_SIZE);

    szCmd[0] = SOM;
    szCmd[1] = 4;
    szCmd[2] = PC;
    szCmd[3] = FOC;
    szCmd[4] = MTR_PMSLEW_RATE;
    szCmd[5] = (unsigned char) nRate; // 0 to 9
    szCmd[6] = checksum(szCmd+1, szCmd[1]+1);

    nErr = EFACommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int CEFAController::setNegativeMotorSlewRate(int nRate)
{
    int nErr = EFA_OK;
    unsigned char szCmd[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    memset(szCmd,0, SERIAL_BUFFER_SIZE);

    szCmd[0] = SOM;
    szCmd[1] = 4;
    szCmd[2] = PC;
    szCmd[3] = FOC;
    szCmd[4] = MTR_NMSLEW_RATE;
    szCmd[5] = (unsigned char) nRate; // 0 to 9
    szCmd[6] = checksum(szCmd+1, szCmd[1]+1);

    nErr = EFACommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int CEFAController::setFan(bool bOn)
{
    int nErr = EFA_OK;
    unsigned char szCmd[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    memset(szCmd,0, SERIAL_BUFFER_SIZE);

    szCmd[0] = SOM;
    szCmd[1] = 4;
    szCmd[2] = PC;
    szCmd[3] = FAN;
    szCmd[4] = FANS_SET;
    szCmd[5] = bOn?0x01:0x00;
    szCmd[6] = checksum(szCmd+1, szCmd[1]+1);

    nErr = EFACommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int CEFAController::getFan(bool &bOn)
{
    int nErr = EFA_OK;
    unsigned char szCmd[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    bOn = false;
    memset(szCmd,0, SERIAL_BUFFER_SIZE);

    szCmd[0] = SOM;
    szCmd[1] = 3;
    szCmd[2] = PC;
    szCmd[3] = FAN;
    szCmd[4] = FANS_GET;
    szCmd[5] = checksum(szCmd+1, szCmd[1]+1);

    nErr = EFACommand(szCmd, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    if(szResp[5] == 0)
        bOn = true;

    return nErr;
}

int CEFAController::setCalibrationState(bool bCalbrated)
{
    int nErr = EFA_OK;
    unsigned char szCmd[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    memset(szCmd,0, SERIAL_BUFFER_SIZE);

    szCmd[0] = SOM;
    szCmd[1] = 5;
    szCmd[2] = PC;
    szCmd[3] = FOC;
    szCmd[4] = MTR_SET_CALIBRATION_STATE;
    szCmd[5] = 0x40;
    szCmd[6] = bCalbrated?0x01:0x00;
    szCmd[7] = checksum(szCmd+1, szCmd[1]+1);

    nErr = EFACommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int CEFAController::getCalibrationState(bool &bCalbrated)
{
    int nErr = EFA_OK;
    unsigned char szCmd[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    bCalbrated = false;

    memset(szCmd,0, SERIAL_BUFFER_SIZE);

    szCmd[0] = SOM;
    szCmd[1] = 4;
    szCmd[2] = PC;
    szCmd[3] = FOC;
    szCmd[4] = MTR_GET_CALIBRATION_STATE;
    szCmd[5] = 0x40;
    szCmd[6] = checksum(szCmd+1, szCmd[1]+1);

    nErr = EFACommand(szCmd, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    if(szResp[5] == 0x01)
        bCalbrated = true;

    return nErr;
}

int CEFAController::setStopDetect(bool bEnable)
{
    int nErr = EFA_OK;
    unsigned char szCmd[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    memset(szCmd,0, SERIAL_BUFFER_SIZE);

    szCmd[0] = SOM;
    szCmd[1] = 4;
    szCmd[2] = PC;
    szCmd[3] = FOC;
    szCmd[4] = MTR_STOP_DETECT;
    szCmd[5] = bEnable?0x01:0x00;
    szCmd[6] = checksum(szCmd+1, szCmd[1]+1);

    nErr = EFACommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int CEFAController::getStopDetect(bool &bEnable)
{
    int nErr = EFA_OK;
    unsigned char szCmd[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    bEnable = false;
    memset(szCmd,0, SERIAL_BUFFER_SIZE);

    szCmd[0] = SOM;
    szCmd[1] = 3;
    szCmd[2] = PC;
    szCmd[3] = FOC;
    szCmd[4] = MTR_GET_STOP_DETECT;
    szCmd[5] = checksum(szCmd+1, szCmd[1]+1);

    nErr = EFACommand(szCmd, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    if(szResp[5] == 0x01)
        bEnable = true;

    return nErr;
}

int CEFAController::setApproachDir(int nDir)
{
    int nErr = EFA_OK;
    unsigned char szCmd[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    if(nDir>1)
        return ERR_CMDFAILED;

    memset(szCmd,0, SERIAL_BUFFER_SIZE);

    szCmd[0] = SOM;
    szCmd[1] = 4;
    szCmd[2] = PC;
    szCmd[3] = FOC;
    szCmd[4] = MTR_APPROACH_DIRECTION;
    szCmd[5] = (unsigned char)nDir;
    szCmd[6] = checksum(szCmd+1, szCmd[1]+1);

    nErr = EFACommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int CEFAController::getApproachDir(int &nDir)
{
    int nErr = EFA_OK;
    unsigned char szCmd[SERIAL_BUFFER_SIZE];
    unsigned char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    memset(szCmd,0, SERIAL_BUFFER_SIZE);

    szCmd[0] = SOM;
    szCmd[1] = 3;
    szCmd[2] = PC;
    szCmd[3] = FOC;
    szCmd[4] = MTR_GET_APPROACH_DIRECTION;
    szCmd[5] = checksum(szCmd+1, szCmd[1]+1);

    nErr = EFACommand(szCmd, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    nDir = szResp[5];

    return nErr;
}


#pragma mark command and response functions

int CEFAController::EFACommand(const unsigned char *pszCmd, unsigned char *pszResult, int nResultMaxLen)
{
    int nErr = EFA_OK;
    unsigned char szResp[SERIAL_BUFFER_SIZE];
    unsigned long  ulBytesWrite;
    unsigned char cHexMessage[LOG_BUFFER_SIZE];

	if(!m_bIsConnected)
		return ERR_COMMNOLINK;

    m_pSerx->purgeTxRx();
#if defined EFA_DEBUG && EFA_DEBUG >= 2
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
    hexdump(pszCmd, cHexMessage, pszCmd[1]+3, LOG_BUFFER_SIZE);
    fprintf(Logfile, "[%s] CEFAController::EFACommand Sending %s\n", timestamp, cHexMessage);
	fflush(Logfile);
#endif
    nErr = m_pSerx->writeFile((void *)pszCmd, pszCmd[1]+3, ulBytesWrite);
    m_pSerx->flushTx();

    if(nErr)
        return nErr;

    if(pszResult) {
        // read response
        nErr = readResponse(szResp, SERIAL_BUFFER_SIZE);
#if defined EFA_DEBUG && EFA_DEBUG >= 2
		ltime = time(NULL);
		timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
        hexdump(szResp, cHexMessage, szResp[1]+3, LOG_BUFFER_SIZE);
		fprintf(Logfile, "[%s] CEFAController::EFACommand response \"%s\"\n", timestamp, cHexMessage);
		fflush(Logfile);
#endif
        if(nErr)
            return nErr;

        memset(pszResult,0, nResultMaxLen);
        memcpy(pszResult, szResp, szResp[1]+3);

#if defined EFA_DEBUG && EFA_DEBUG >= 2
		ltime = time(NULL);
		timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
        hexdump(pszResult, cHexMessage, pszResult[1]+3, LOG_BUFFER_SIZE);
		fprintf(Logfile, "[%s] CEFAController::EFACommand response copied to pszResult : \"%s\"\n", timestamp, cHexMessage);
		fflush(Logfile);
#endif
    }
    return nErr;
}

int CEFAController::readResponse(unsigned char *pszRespBuffer, int nBufferLen)
{
    int nErr = EFA_OK;
    unsigned long ulBytesRead = 0;
    int nLen = SERIAL_BUFFER_SIZE;
    unsigned char cHexMessage[LOG_BUFFER_SIZE];
    char cChecksum;

    if(!m_bIsConnected)
		return ERR_COMMNOLINK;

    memset(pszRespBuffer, 0, (size_t) nBufferLen);

    // Look for a 0x01 starting character, until timeout occurs
    while (*pszRespBuffer != SOM && nErr == EFA_OK) {
        nErr = m_pSerx->readFile(pszRespBuffer, 1, ulBytesRead, MAX_TIMEOUT);
        if (ulBytesRead !=1) // timeout
            nErr = EFA_BAD_CMD_RESPONSE;
    }

    if(*pszRespBuffer != SOM || nErr != EFA_OK)
        return ERR_CMDFAILED;

    // Read message length
    nErr = m_pSerx->readFile(pszRespBuffer + 1, 1, ulBytesRead, MAX_TIMEOUT);
    if (nErr != EFA_OK || ulBytesRead!=1)
        return ERR_CMDFAILED;

    nLen = pszRespBuffer[1];

    // Read the rest of the message
    nErr = m_pSerx->readFile(pszRespBuffer + 2, nLen, ulBytesRead, MAX_TIMEOUT);
    if(nErr || ulBytesRead != nLen) {
#ifdef EFA_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        hexdump(pszRespBuffer, cHexMessage, pszRespBuffer[1]+2, LOG_BUFFER_SIZE);
        fprintf(Logfile, "[%s] CEFAController::readResponse error\n", timestamp);
        fprintf(Logfile, "[%s] CEFAController::readResponse got %s\n", timestamp, cHexMessage);
        fflush(Logfile);
#endif
        return ERR_CMDFAILED;
    }

    // verify checksum
    cChecksum = checksum(pszRespBuffer+1, nLen+1);
    if (cChecksum != *(pszRespBuffer+nLen+2)) {
#ifdef EFA_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] CEFAController::readResponse Checksum error : calculated checksume is %02x, message checksum is %02X\n", timestamp, cChecksum, *(pszRespBuffer+nLen+2));
        fflush(Logfile);
#endif
        nErr = ERR_CMDFAILED;
    }
    return nErr;
}

signed char CEFAController::checksum(const unsigned char *cMessage, int nLen)
{
    int nIdx;
    char cChecksum = 0;

    for (nIdx = 0; nIdx < nLen && nIdx < SERIAL_BUFFER_SIZE; nIdx++) {
        cChecksum -= cMessage[nIdx];
    }
    return cChecksum;
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
