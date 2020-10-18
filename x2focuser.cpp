#include <stdio.h>
#include <string.h>
#include "x2focuser.h"

#include "../../licensedinterfaces/theskyxfacadefordriversinterface.h"
#include "../../licensedinterfaces/sleeperinterface.h"
#include "../../licensedinterfaces/loggerinterface.h"
#include "../../licensedinterfaces/basiciniutilinterface.h"
#include "../../licensedinterfaces/mutexinterface.h"
#include "../../licensedinterfaces/basicstringinterface.h"
#include "../../licensedinterfaces/tickcountinterface.h"
#include "../../licensedinterfaces/serxinterface.h"
#include "../../licensedinterfaces/sberrorx.h"
#include "../../licensedinterfaces/serialportparams2interface.h"

X2Focuser::X2Focuser(const char* pszDisplayName, 
												const int& nInstanceIndex,
												SerXInterface						* pSerXIn, 
												TheSkyXFacadeForDriversInterface	* pTheSkyXIn, 
												SleeperInterface					* pSleeperIn,
												BasicIniUtilInterface				* pIniUtilIn,
												LoggerInterface						* pLoggerIn,
												MutexInterface						* pIOMutexIn,
												TickCountInterface					* pTickCountIn)

{
	m_pSerX							= pSerXIn;		
	m_pTheSkyXForMounts				= pTheSkyXIn;
	m_pSleeper						= pSleeperIn;
	m_pIniUtil						= pIniUtilIn;
	m_pLogger						= pLoggerIn;	
	m_pIOMutex						= pIOMutexIn;
	m_pTickCount					= pTickCountIn;
	
	m_bLinked = false;
	m_nPosition = 0;
    m_fLastTemp = -273.15f; // aboslute zero :)

	m_EFAController.SetSerxPointer(m_pSerX);
	m_EFAController.setLogger(m_pLogger);
    m_EFAController.setSleeper(m_pSleeper);
}

X2Focuser::~X2Focuser()
{
    //Delete objects used through composition
	if (GetSerX())
		delete GetSerX();
	if (GetTheSkyXFacadeForDrivers())
		delete GetTheSkyXFacadeForDrivers();
	if (GetSleeper())
		delete GetSleeper();
	if (GetSimpleIniUtil())
		delete GetSimpleIniUtil();
	if (GetLogger())
		delete GetLogger();
	if (GetMutex())
		delete GetMutex();

}

#pragma mark - DriverRootInterface

int	X2Focuser::queryAbstraction(const char* pszName, void** ppVal)
{
    *ppVal = NULL;

    if (!strcmp(pszName, LinkInterface_Name))
        *ppVal = (LinkInterface*)this;

    else if (!strcmp(pszName, FocuserGotoInterface2_Name))
        *ppVal = (FocuserGotoInterface2*)this;

    else if (!strcmp(pszName, ModalSettingsDialogInterface_Name))
        *ppVal = dynamic_cast<ModalSettingsDialogInterface*>(this);

    else if (!strcmp(pszName, X2GUIEventInterface_Name))
        *ppVal = dynamic_cast<X2GUIEventInterface*>(this);

    else if (!strcmp(pszName, FocuserTemperatureInterface_Name))
        *ppVal = dynamic_cast<FocuserTemperatureInterface*>(this);

    else if (!strcmp(pszName, LoggerInterface_Name))
        *ppVal = GetLogger();

    else if (!strcmp(pszName, ModalSettingsDialogInterface_Name))
        *ppVal = dynamic_cast<ModalSettingsDialogInterface*>(this);

    else if (!strcmp(pszName, SerialPortParams2Interface_Name))
        *ppVal = dynamic_cast<SerialPortParams2Interface*>(this);

    return SB_OK;
}

#pragma mark - DriverInfoInterface
void X2Focuser::driverInfoDetailedInfo(BasicStringInterface& str) const
{
        str = "EFA Focuser X2 plugin by Rodolphe Pineau";
}

double X2Focuser::driverInfoVersion(void) const							
{
	return DRIVER_VERSION;
}

void X2Focuser::deviceInfoNameShort(BasicStringInterface& str) const
{
    str="EFA";
}

void X2Focuser::deviceInfoNameLong(BasicStringInterface& str) const				
{
    deviceInfoNameShort(str);
}

void X2Focuser::deviceInfoDetailedDescription(BasicStringInterface& str) const		
{
	str = "EFA Controller";
}

void X2Focuser::deviceInfoFirmwareVersion(BasicStringInterface& str)				
{
    int nErr = SB_OK;
    if(!m_bLinked) {
        str="";
    }
    else {
        X2MutexLocker ml(GetMutex());
        // get firmware version
        char cFirmware[SERIAL_BUFFER_SIZE];
        nErr = m_EFAController.getFirmwareVersion(cFirmware, SERIAL_BUFFER_SIZE);
        if(nErr)
            str = "";
        else
            str = cFirmware;
    }
}

void X2Focuser::deviceInfoModel(BasicStringInterface& str)							
{
    str="EFA";
}

#pragma mark - LinkInterface
int	X2Focuser::establishLink(void)
{
    char szPort[DRIVER_MAX_STRING];
    int nErr;

    X2MutexLocker ml(GetMutex());
    // get serial port device name
    portNameOnToCharPtr(szPort,DRIVER_MAX_STRING);
    nErr = m_EFAController.Connect(szPort);
    if(nErr)
        m_bLinked = false;
    else
        m_bLinked = true;

    return nErr;
}

int	X2Focuser::terminateLink(void)
{
    if(!m_bLinked)
        return SB_OK;

    X2MutexLocker ml(GetMutex());
    m_EFAController.haltFocuser();
    m_EFAController.Disconnect();
    m_bLinked = false;

	return SB_OK;
}

bool X2Focuser::isLinked(void) const
{
	return m_bLinked;
}

#pragma mark - ModalSettingsDialogInterface
int	X2Focuser::initModalSettingsDialog(void)
{
    return SB_OK;
}

int	X2Focuser::execModalSettingsDialog(void)
{
    int nErr = SB_OK;
    X2ModalUIUtil uiutil(this, GetTheSkyXFacadeForDrivers());
    X2GUIInterface*					ui = uiutil.X2UI();
    X2GUIExchangeInterface*			dx = NULL;//Comes after ui is loaded
    bool bPressedOK = false;
    int nPosition = 0;
    int nPosLimitMin = 0;
    int nPosLimitMax = 0;
    int nApproachDir = 0;
    bool bFanOn;
    bool bStopDetect;
    bool bCalibrated;

    mUiEnabled = false;

    if (NULL == ui)
        return ERR_POINTER;

    if ((nErr = ui->loadUserInterface("efa.ui", deviceType(), m_nPrivateMulitInstanceIndex)))
        return nErr;

    if (NULL == (dx = uiutil.X2DX()))
        return ERR_POINTER;

    X2MutexLocker ml(GetMutex());
	// set controls values
    if(m_bLinked) {
        // new position (set to current )
        nErr = m_EFAController.getPosition(nPosition);
        if(nErr)
            return nErr;
        nErr = m_EFAController.getPosLimitMin(nPosLimitMin);
        if(nErr)
            return nErr;

        nErr = m_EFAController.getPosLimitMax(nPosLimitMax);
        if(nErr)
            return nErr;

        nErr = m_EFAController.getApproachDir(nApproachDir);
        if(nErr)
            return nErr;
        nErr = m_EFAController.getFan(bFanOn);
        if(nErr)
            return nErr;
        m_bFanOn = bFanOn;

        nErr = m_EFAController.getStopDetect(bStopDetect);
        if(nErr)
            return nErr;
        m_bStopDetect = bStopDetect;

        nErr = m_EFAController.getCalibrationState(bCalibrated);
        if(nErr)
            return nErr;
        m_bCalibrated = bCalibrated;
        
        dx->setEnabled("newPos", true);
        dx->setEnabled("pushButton", true);
        dx->setPropertyInt("newPos", "value", nPosition);
        dx->setEnabled("posLimit", true);
        dx->setEnabled("pushButton_2", true);
        dx->setPropertyInt("posLimitMin", "value", nPosLimitMin);
        dx->setPropertyInt("posLimitMax", "value", nPosLimitMax);

        dx->setEnabled("radioButtonAppPos", true);
        dx->setEnabled("radioButtonAppNeg", true);
        if(nApproachDir == 1) {
            dx->setChecked("radioButtonAppPos",true);
            dx->setChecked("radioButtonAppNeg",false);
        }
        else {
            dx->setChecked("radioButtonAppPos",false);
            dx->setChecked("radioButtonAppNeg",true);
        }
        dx->setEnabled("isFanOn", true);
        dx->setChecked("isFanOn", bFanOn);

        dx->setEnabled("isStopDetect", true);
        dx->setChecked("isStopDetect", bStopDetect);

        dx->setEnabled("isCalibrated", true);
        dx->setChecked("isCalibrated", bCalibrated);

    }
    else {
        // disable all controls
        dx->setEnabled("newPos", false);
        dx->setPropertyInt("newPos", "value", 0);
        dx->setEnabled("posLimit", false);
        dx->setPropertyInt("posLimit", "value", 0);
        dx->setEnabled("pushButton", false);
        dx->setEnabled("pushButton_2", false);
        dx->setEnabled("radioButtonAppPos", false);
        dx->setEnabled("radioButtonAppNeg", false);
        dx->setEnabled("isFanOn", false);
        dx->setEnabled("isStopDetect", false);
        dx->setEnabled("isCalibrated", false);
    }

    //Display the user interface
    mUiEnabled = true;
    if ((nErr = ui->exec(bPressedOK)))
        return nErr;
    mUiEnabled = false;

    //Retreive values from the user interface
    if (bPressedOK) {
        nErr = SB_OK;
        // get limit option
        dx->propertyInt("posLimitMin", "value", nPosLimitMin);
        m_EFAController.setPosLimitMin(nPosLimitMin);
        dx->propertyInt("posLimitMax", "value", nPosLimitMax);
        m_EFAController.setPosLimitMax(nPosLimitMax);
    }
    return nErr;
}

void X2Focuser::uiEvent(X2GUIExchangeInterface* uiex, const char* pszEvent)
{
    int nErr = SB_OK;
    int nTmpVal;
    char szErrorMessage[LOG_BUFFER_SIZE];

    // new position
    if (!strcmp(pszEvent, "on_pushButton_clicked")) {
        uiex->propertyInt("newPos", "value", nTmpVal);
        nErr = m_EFAController.syncMotorPosition(nTmpVal);
        if(nErr) {
            snprintf(szErrorMessage, LOG_BUFFER_SIZE, "Error setting new position : Error %d", nErr);
            uiex->messageBox("Set New Position", szErrorMessage);
            return;
        }
    }

    // new limit
    if (!strcmp(pszEvent, "on_pushButton_2_clicked")) {
        uiex->propertyInt("posLimitMin", "value", nTmpVal);
        nErr = m_EFAController.setPosLimitMin(nTmpVal);
        if(nErr) {
            snprintf(szErrorMessage, LOG_BUFFER_SIZE, "Error setting new limit : Error %d", nErr);
            uiex->messageBox("Set New Limit", szErrorMessage);
            return;
        }
    }

    if (!strcmp(pszEvent, "on_pushButton_3_clicked")) {
        uiex->propertyInt("posLimitMax", "value", nTmpVal);
        nErr = m_EFAController.setPosLimitMax(nTmpVal);
        if(nErr) {
            snprintf(szErrorMessage, LOG_BUFFER_SIZE, "Error setting new limit : Error %d", nErr);
            uiex->messageBox("Set New Limit", szErrorMessage);
            return;
        }
    }

    if (!strcmp(pszEvent, "on_timer")) {
        if(uiex->isChecked("isFanOn")) {
            if(!m_bFanOn){ // only change state if needed
                m_EFAController.setFan(true);
                m_bFanOn = true;
            }
        } else {
            if(m_bFanOn){ // only change state if needed
                m_EFAController.setFan(false);
                m_bFanOn = false;
            }
        }

        if(uiex->isChecked("isStopDetect")) {
            if(!m_bStopDetect){ // only change state if needed
                m_EFAController.setStopDetect(true);
                m_bStopDetect = true;
            }
        } else {
            if(m_bStopDetect){ // only change state if needed
                m_EFAController.setStopDetect(false);
                m_bStopDetect = false;
            }
        }

        if(uiex->isChecked("isCalibrated")) {
            if(!m_bCalibrated){ // only change state if needed
                m_EFAController.setCalibrationState(true);
                m_bCalibrated = true;
            }
        } else {
            if(m_bCalibrated){ // only change state if needed
                m_EFAController.setCalibrationState(false);
                m_bCalibrated = false;
            }
        }
    }
}

#pragma mark - FocuserGotoInterface2
int	X2Focuser::focPosition(int& nPosition)
{
    int nErr;

    if(!m_bLinked)
        return NOT_CONNECTED;

    X2MutexLocker ml(GetMutex());

    nErr = m_EFAController.getPosition(nPosition);
    m_nPosition = nPosition;
    return nErr;
}

int	X2Focuser::focMinimumLimit(int& nMinLimit) 		
{
    int nErr = SB_OK;

    if(!m_bLinked)
        return NOT_CONNECTED;

    X2MutexLocker ml(GetMutex());
    nErr = m_EFAController.getPosLimitMin(nMinLimit);
    return nErr;

}

int	X2Focuser::focMaximumLimit(int& nMaxLimit)
{
    int nErr = SB_OK;

    if(!m_bLinked)
        return NOT_CONNECTED;

	X2MutexLocker ml(GetMutex());
    nErr = m_EFAController.getPosLimitMax(nMaxLimit);

    return nErr;
}

int	X2Focuser::focAbort()								
{   int nErr;

    if(!m_bLinked)
        return NOT_CONNECTED;

    X2MutexLocker ml(GetMutex());
    nErr = m_EFAController.haltFocuser();
    return nErr;
}

int	X2Focuser::startFocGoto(const int& nRelativeOffset)	
{
    if(!m_bLinked)
        return NOT_CONNECTED;

    X2MutexLocker ml(GetMutex());
    m_EFAController.moveRelativeToPosision(nRelativeOffset);
    return SB_OK;
}

int	X2Focuser::isCompleteFocGoto(bool& bComplete) const
{
    int nErr;

    if(!m_bLinked)
        return NOT_CONNECTED;

    X2Focuser* pMe = (X2Focuser*)this;
    X2MutexLocker ml(pMe->GetMutex());
	nErr = pMe->m_EFAController.isGoToComplete(bComplete);

    return nErr;
}

int	X2Focuser::endFocGoto(void)
{
    int nErr;
    if(!m_bLinked)
        return NOT_CONNECTED;

    X2MutexLocker ml(GetMutex());
    nErr = m_EFAController.getPosition(m_nPosition);
    return nErr;
}

int X2Focuser::amountCountFocGoto(void) const					
{ 
	return 3;
}

int	X2Focuser::amountNameFromIndexFocGoto(const int& nZeroBasedIndex, BasicStringInterface& strDisplayName, int& nAmount)
{
	switch (nZeroBasedIndex)
	{
		default:
		case 0: strDisplayName="10 steps"; nAmount=10;break;
		case 1: strDisplayName="100 steps"; nAmount=100;break;
		case 2: strDisplayName="1000 steps"; nAmount=1000;break;
	}
	return SB_OK;
}

int	X2Focuser::amountIndexFocGoto(void)
{
	return 0;
}

#pragma mark - FocuserTemperatureInterface
int X2Focuser::focTemperature(double &dTemperature)
{
    int nErr = SB_OK;

    if(!m_bLinked) {
        dTemperature = -100.0;
        return NOT_CONNECTED;
    }
    X2MutexLocker ml(GetMutex());

    // Taken from Richard's Robofocus plugin code.
    // this prevent us from asking the temperature too often
    static CStopWatch timer;

    if(timer.GetElapsedSeconds() > 30.0f || m_fLastTemp < -99.0f) {
        nErr = m_EFAController.getTemperature(m_fLastTemp);
        timer.Reset();
    }

    dTemperature = m_fLastTemp;

    return nErr;
}

#pragma mark - SerialPortParams2Interface

void X2Focuser::portName(BasicStringInterface& str) const
{
    char szPortName[DRIVER_MAX_STRING];

    portNameOnToCharPtr(szPortName, DRIVER_MAX_STRING);

    str = szPortName;

}

void X2Focuser::setPortName(const char* pszPort)
{
    if (m_pIniUtil)
        m_pIniUtil->writeString(PARENT_KEY, CHILD_KEY_PORTNAME, pszPort);

}


void X2Focuser::portNameOnToCharPtr(char* pszPort, const int& nMaxSize) const
{
    if (NULL == pszPort)
        return;

    snprintf(pszPort, nMaxSize, DEF_PORT_NAME);

    if (m_pIniUtil)
        m_pIniUtil->readString(PARENT_KEY, CHILD_KEY_PORTNAME, pszPort, pszPort, nMaxSize);
    
}




