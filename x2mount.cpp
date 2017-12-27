#include "x2mount.h"

X2Mount::X2Mount(const char* pszDriverSelection,
				 const int& nInstanceIndex,
				 SerXInterface					* pSerX,
				 TheSkyXFacadeForDriversInterface	* pTheSkyX,
				 SleeperInterface					* pSleeper,
				 BasicIniUtilInterface			* pIniUtil,
				 LoggerInterface					* pLogger,
				 MutexInterface					* pIOMutex,
				 TickCountInterface				* pTickCount)
{
	// Variables for HEQ5
	
	m_nPrivateMulitInstanceIndex	= nInstanceIndex;
	m_pSerX							= pSerX;
	m_pTheSkyXForMounts				= pTheSkyX;
	m_pSleeper						= pSleeper;
	m_pIniUtil						= pIniUtil;
	m_pLogger						= pLogger;
	m_pIOMutex						= pIOMutex;
	m_pTickCount					= pTickCount;
	
#ifdef ATCS_X2_DEBUG
	LogFile = fopen(HEQ5_LOGFILENAME, "w");
#endif
	
	
	m_bSynced = false;
	m_bParked = false;
    m_bLinked = false;

    mATCS.setSerxPointer(m_pSerX);
    mATCS.setTSX(m_pTheSkyXForMounts);
    mATCS.setSleeper(m_pSleeper);
    mATCS.setLogger(m_pLogger);


	// Read the current stored values for the settings
	if (m_pIniUtil)
	{
	}

}

X2Mount::~X2Mount()
{
	// Write the stored values

    if(m_bLinked)
        mATCS.Disconnect();
    
    if (m_pSerX)
		delete m_pSerX;
	if (m_pTheSkyXForMounts)
		delete m_pTheSkyXForMounts;
	if (m_pSleeper)
		delete m_pSleeper;
	if (m_pIniUtil)
		delete m_pIniUtil;
	if (m_pLogger)
		delete m_pLogger;
	if (m_pIOMutex)
		delete m_pIOMutex;
	if (m_pTickCount)
		delete m_pTickCount;
	
#ifdef ATCS_X2_DEBUG
	// Close LogFile
	if (LogFile) {
		fclose(LogFile);
	}
#endif
	
}

int X2Mount::queryAbstraction(const char* pszName, void** ppVal)
{
	*ppVal = NULL;
	
	if (!strcmp(pszName, SyncMountInterface_Name))
	    *ppVal = dynamic_cast<SyncMountInterface*>(this);
	if (!strcmp(pszName, SlewToInterface_Name))
		*ppVal = dynamic_cast<SlewToInterface*>(this);
	else if (!strcmp(pszName, AsymmetricalEquatorialInterface_Name))
		*ppVal = dynamic_cast<AsymmetricalEquatorialInterface*>(this);
	else if (!strcmp(pszName, OpenLoopMoveInterface_Name))
		*ppVal = dynamic_cast<OpenLoopMoveInterface*>(this);
	else if (!strcmp(pszName, NeedsRefractionInterface_Name))
		*ppVal = dynamic_cast<NeedsRefractionInterface*>(this);
	else if (!strcmp(pszName, ModalSettingsDialogInterface_Name))
		*ppVal = dynamic_cast<ModalSettingsDialogInterface*>(this);
	else if (!strcmp(pszName, X2GUIEventInterface_Name))
		*ppVal = dynamic_cast<X2GUIEventInterface*>(this);
	else if (!strcmp(pszName, TrackingRatesInterface_Name))
		*ppVal = dynamic_cast<TrackingRatesInterface*>(this);
	else if (!strcmp(pszName, ParkInterface_Name))
		*ppVal = dynamic_cast<ParkInterface*>(this);
	else if (!strcmp(pszName, UnparkInterface_Name))
		*ppVal = dynamic_cast<UnparkInterface*>(this);
	else if (!strcmp(pszName, LoggerInterface_Name))
        *ppVal = dynamic_cast<UnparkInterface*>(this);
    else if (!strcmp(pszName, SerialPortParams2Interface_Name))
        *ppVal = dynamic_cast<SerialPortParams2Interface*>(this);
	return SB_OK;
}

//OpenLoopMoveInterface
int X2Mount::startOpenLoopMove(const MountDriverInterface::MoveDir& Dir, const int& nRateIndex)
{
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

	m_CurrentRateIndex = nRateIndex;
#ifdef ATCS_X2_DEBUG
	if (LogFile) {
		time_t ltime = time(NULL);
		char *timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
		fprintf(LogFile, "[%s] startOpenLoopMove called %d %d\n", timestamp, Dir, nRateIndex);
        fflush(LogFile);
	}
#endif
    // mATCS.StartOpenSlew(Dir, SlewSpeeds[nRateIndex]);
    // do stuff

    return SB_OK;
}

int X2Mount::endOpenLoopMove(void)
{
	int nErr = SB_OK;
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

#ifdef ATCS_X2_DEBUG
	if (LogFile){
		time_t ltime = time(NULL);
		char *timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
		fprintf(LogFile, "[%s] endOpenLoopMove Called\n", timestamp);
        fflush(LogFile);
	}
#endif
    // do stuff
    return nErr;
}

int X2Mount::rateCountOpenLoopMove(void) const
{
    X2Mount* pMe = (X2Mount*)this;
	return pMe->mATCS.getNbSlewRates();
}

int X2Mount::rateNameFromIndexOpenLoopMove(const int& nZeroBasedIndex, char* pszOut, const int& nOutMaxSize)
{
    mATCS.getRateName(nZeroBasedIndex, pszOut, nOutMaxSize);

    return SB_OK;
}

int X2Mount::rateIndexOpenLoopMove(void)
{
	return m_CurrentRateIndex;
}

#pragma mark - UI binding

int X2Mount::execModalSettingsDialog(void)
{
	int nErr = SB_OK;
	X2ModalUIUtil uiutil(this, m_pTheSkyXForMounts);
	X2GUIInterface*					ui = uiutil.X2UI();
	X2GUIExchangeInterface*			dx = NULL;//Comes after ui is loaded
	bool bPressedOK = false;
	int i;
	
#ifdef ATCS_X2_DEBUG
	time_t ltime;
	char *timestamp;
	if (LogFile) {
		ltime = time(NULL);
		timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
		fprintf(LogFile, "[%s] execModelSettingsDialog called %d\n", timestamp, m_bPolarisHomeAlignmentSet);
        fflush(LogFile);
	}
#endif
	
	if (NULL == ui) return ERR_POINTER;
	
	if ((nErr = ui->loadUserInterface("ATCS.ui", deviceType(), m_nPrivateMulitInstanceIndex)))
		return nErr;
	
	if (NULL == (dx = uiutil.X2DX())) {
		return ERR_POINTER;
	}
	
	// Set values in the userinterface

	//Display the user interface
	if ((nErr = ui->exec(bPressedOK)))
		return nErr;
	
	//Retreive values from the user interface
	if (bPressedOK)
	{
	}
	return nErr;
}

void X2Mount::uiEvent(X2GUIExchangeInterface* uiex, const char* pszEvent)
{

	X2MutexLocker ml(GetMutex());
	
#ifdef ATCS_X2_DEBUG
	time_t ltime;
	char *timestamp;
	if (LogFile) {
		ltime = time(NULL);
		timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
		fprintf(LogFile, "[%s] uievent %s\n", timestamp, pszEvent);
        fflush(LogFile);
	}
#endif
	if (!strcmp(pszEvent, "on_timer")) {
	}

	return;
}

//LinkInterface
int X2Mount::establishLink(void)
{
    int nErr;
    char szPort[DRIVER_MAX_STRING];

	X2MutexLocker ml(GetMutex());
	// get serial port device name
    portNameOnToCharPtr(szPort,DRIVER_MAX_STRING);

	nErr =  mATCS.Connect(szPort);
    if(nErr) {
        m_bLinked = false;
    }
    else {
        m_bLinked = true;
    }
    return nErr;
}

int X2Mount::terminateLink(void)
{
    int nErr = SB_OK;

	X2MutexLocker ml(GetMutex());

    nErr = mATCS.Disconnect();
    m_bLinked = false;

    return nErr;
}

bool X2Mount::isLinked(void) const
{

	return mATCS.isConnected();;
}

bool X2Mount::isEstablishLinkAbortable(void) const
{
    return false;
}

//AbstractDriverInfo

void	X2Mount::driverInfoDetailedInfo(BasicStringInterface& str) const
{
	str = "ATCS X2 plugin by Rodolphe Pineau";
}

double	X2Mount::driverInfoVersion(void) const
{
	return DRIVER_VERSION;
}

//AbstractDeviceInfo
void X2Mount::deviceInfoNameShort(BasicStringInterface& str) const
{
    if(m_bLinked) {
        X2Mount* pMe = (X2Mount*)this;
        X2MutexLocker ml(pMe->GetMutex());
        char cModel[SERIAL_BUFFER_SIZE];
        pMe->mATCS.getModel(cModel, SERIAL_BUFFER_SIZE);
        str = cModel;
    }
    else
        str = "Not connected";
}
void X2Mount::deviceInfoNameLong(BasicStringInterface& str) const
{
	str = "ATCS Mount";
	
}
void X2Mount::deviceInfoDetailedDescription(BasicStringInterface& str) const
{
	str = "Astrometric Instruments Telescope Control System";
	
}
void X2Mount::deviceInfoFirmwareVersion(BasicStringInterface& str)
{
    if(m_bLinked) {
        char cFirmware[SERIAL_BUFFER_SIZE];
        X2MutexLocker ml(GetMutex());
        mATCS.getFirmwareVersion(cFirmware, SERIAL_BUFFER_SIZE);
        str = cFirmware;
    }
    else
        str = "Not connected";
}
void X2Mount::deviceInfoModel(BasicStringInterface& str)
{
    if(m_bLinked) {
        char cModel[SERIAL_BUFFER_SIZE];
        X2MutexLocker ml(GetMutex());
        mATCS.getModel(cModel, SERIAL_BUFFER_SIZE);
        str = cModel;
    }
    else
        str = "Not connected";
}

//Common Mount specifics
int X2Mount::raDec(double& ra, double& dec, const bool& bCached)
{
	int nErr = 0;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());
	
	// Get the RA and DEC from the mount
	nErr = mATCS.getRaAndDec(ra, dec);
    if(nErr)
        nErr = ERR_CMDFAILED;

	return nErr;
}

int X2Mount::abort(void)
{
    int nErr = SB_OK;
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

#ifdef ATCS_X2_DEBUG
	if (LogFile) {
		time_t ltime = time(NULL);
		char *timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
		fprintf(LogFile, "[%s] abort Called\n", timestamp);
        fflush(LogFile);
	}
#endif

    nErr = mATCS.Abort();
    if(nErr)
        nErr = ERR_CMDFAILED;

    return nErr;
}

int X2Mount::startSlewTo(const double& dRa, const double& dDec)
{
	int nErr = SB_OK;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

#ifdef ATCS_X2_DEBUG
	if (LogFile) {
		time_t ltime = time(NULL);
		char *timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
		fprintf(LogFile, "[%s] startSlewTo Called %f %f\n", timestamp, dRa, dDec);
        fflush(LogFile);
	}
#endif

    // nErr = mATCS.StartSlewTo(dRa, dDec);
	m_wasslewing = true;
	return nErr;
}

int X2Mount::isCompleteSlewTo(bool& bComplete) const
{
    if(!m_bLinked)
        return ERR_NOLINK;

    X2Mount* pMe = (X2Mount*)this;
    X2MutexLocker ml(pMe->GetMutex());

    // bComplete = mATCS.GetIsNotGoto();
	
    bComplete = true; // FIXME
	return SB_OK;
}

int X2Mount::endSlewTo(void)
{
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());
    // do stuff
    return SB_OK;
}


int X2Mount::syncMount(const double& ra, const double& dec)
{
	int nErr = SB_OK;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

#ifdef ATCS_X2_DEBUG
    if (LogFile) {
        time_t ltime = time(NULL);
        char *timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] syncMount Called : %f\t%f\n", timestamp, ra, dec);
        fflush(LogFile);
    }
#endif

    nErr = mATCS.syncTo(ra, dec);
    if(nErr)
        nErr = ERR_CMDFAILED;

	return nErr;
}

bool X2Mount::isSynced(void)
{
    int nErr;
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());
    nErr = mATCS.isSynced(m_bSynced);
    if(nErr)
        nErr = ERR_CMDFAILED;

	return m_bSynced;
}

int X2Mount::setTrackingRates(const bool& bTrackingOn, const bool& bIgnoreRates, const double& dRaRateArcSecPerSec, const double& dDecRateArcSecPerSec)
{
    int nErr = SB_OK;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());
#ifdef ATCS_X2_DEBUG
	if (LogFile) {
		time_t ltime = time(NULL);
		char *timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
		fprintf(LogFile, "[%s] setTrackingRates Called Tracking On %d Ignore Rates%d\n", timestamp, bTrackingOn, bIgnoreRates);
        fflush(LogFile);
	}
#endif
    // nErr = mATCS.SetTrackingRates(bTrackingOn, bIgnoreRates, dRaRateArcSecPerSec, dDecRateArcSecPerSec);
    return nErr;
	
}

int X2Mount::trackingRates(bool& bTrackingOn, double& dRaRateArcSecPerSec, double& dDecRateArcSecPerSec)
{
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());
	bTrackingOn = false;
	dRaRateArcSecPerSec = -1000.0;
	dDecRateArcSecPerSec = -1000.0;
	
	return SB_OK;
}

//NeedsRefractionInterface
bool X2Mount::needsRefactionAdjustments(void)
{
	return true;	
}

/* Parking Interface */
bool X2Mount::isParked(void) {

    mATCS.getAtPark(m_bParked);
	return m_bParked;
}

int X2Mount::startPark(const double& dAz, const double& dAlt)
{
	double dRa, dDec;
	int nErr = SB_OK;

    if(!m_bLinked)
        return ERR_NOLINK;
	
	X2MutexLocker ml(GetMutex());
	nErr = m_pTheSkyXForMounts->HzToEq(dAz, dAlt, dRa, dDec);
    if (nErr) {
        return nErr;
    }

#ifdef ATCS_X2_DEBUG
	if (LogFile) {
		time_t ltime = time(NULL);
		char *timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
		fprintf(LogFile, "[%s] startPark Called Alt %f Az %f Ra %f Dec %f\n", timestamp, dAz, dAlt, dRa, dDec);
        fflush(LogFile);
	}
#endif

	return nErr;
}


int X2Mount::isCompletePark(bool& bComplete) const
{
    int nErr = SB_OK;

    if(!m_bLinked)
        return ERR_NOLINK;
    X2Mount* pMe = (X2Mount*)this;

#ifdef ATCS_X2_DEBUG
	if (LogFile) {
		time_t ltime = time(NULL);
		char *timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
		fprintf(LogFile, "[%s] isCompletePark Called\n", timestamp);
        fflush(LogFile);
	}
#endif
    nErr = pMe->mATCS.getAtPark(bComplete);
    if(nErr)
        nErr = ERR_CMDFAILED;

	return nErr;
}

int X2Mount::endPark(void)
{
    return SB_OK;
}

int		X2Mount::startUnpark(void)
{
	return SB_OK;
}
/*!Called to monitor the unpark process.  \param bComplete Set to true if the unpark is complete, otherwise set to false.*/
int X2Mount::isCompleteUnpark(bool& bComplete) const
{
    int nErr;
    bool bIsPArked;
    if(!m_bLinked)
        return ERR_NOLINK;

    X2Mount* pMe = (X2Mount*)this;
    bComplete = true;
    nErr = pMe->mATCS.getAtPark(bIsPArked);
    if(nErr)
        nErr = ERR_CMDFAILED;

    if(bIsPArked)
        bComplete = false;

    pMe->m_bParked = false;
	return SB_OK;
}

/*!Called once the unpark is complete.	This is called once for every corresponding startUnpark() allowing software implementations of unpark.*/
int		X2Mount::endUnpark(void)
{
	return SB_OK;
}

int X2Mount::beyondThePole(bool& bYes) {
	// bYes = mATCS.GetIsBeyondThePole();
	return SB_OK;
}


// Leave the two functions below as virtual functions since we're not setting them explicitly

/* 
double X2Mount::flipHourAngle() {
#ifdef ATCS_X2_DEBUG
	if (LogFile) {
		time_t ltime = time(NULL);
		char *timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
		fprintf(LogFile, "[%s] flipHourAngle called\n", timestamp);
	}
#endif

	return 0.0;
}


int X2Mount::gemLimits(double& dHoursEast, double& dHoursWest)
{
#ifdef ATCS_X2_DEBUG
	if (LogFile) {
		time_t ltime = time(NULL);
		char *timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
		fprintf(LogFile, "[%s] gemLimits called\n", timestamp);
	}
#endif

	dHoursEast = 0.0;
	dHoursWest = 0.0;
	return SB_OK;
}
*/

//
// SerialPortParams2Interface
//
#pragma mark - SerialPortParams2Interface

void X2Mount::portName(BasicStringInterface& str) const
{
    char szPortName[DRIVER_MAX_STRING];

    portNameOnToCharPtr(szPortName, DRIVER_MAX_STRING);

    str = szPortName;

}

void X2Mount::setPortName(const char* pszPort)
{
    if (m_pIniUtil)
        m_pIniUtil->writeString(PARENT_KEY, CHILD_KEY_PORT_NAME, pszPort);

}


void X2Mount::portNameOnToCharPtr(char* pszPort, const int& nMaxSize) const
{
    if (NULL == pszPort)
        return;

    snprintf(pszPort, nMaxSize,DEF_PORT_NAME);

    if (m_pIniUtil)
        m_pIniUtil->readString(PARENT_KEY, CHILD_KEY_PORT_NAME, pszPort, pszPort, nMaxSize);

}



