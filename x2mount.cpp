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

	m_nPrivateMulitInstanceIndex	= nInstanceIndex;
	m_pSerX							= pSerX;
	m_pTheSkyXForMounts				= pTheSkyX;
	m_pSleeper						= pSleeper;
	m_pIniUtil						= pIniUtil;
	m_pLogger						= pLogger;
	m_pIOMutex						= pIOMutex;
	m_pTickCount					= pTickCount;
	
#ifdef ATCS_X2_DEBUG
#if defined(SB_WIN_BUILD)
    m_sLogfilePath = getenv("HOMEDRIVE");
    m_sLogfilePath += getenv("HOMEPATH");
    m_sLogfilePath += "\\ATCS_X2_Logfile.txt";
#elif defined(SB_LINUX_BUILD)
    m_sLogfilePath = getenv("HOME");
    m_sLogfilePath += "/ATCS_X2_Logfile.txt";
#elif defined(SB_MAC_BUILD)
    m_sLogfilePath = getenv("HOME");
    m_sLogfilePath += "/ATCS_X2_Logfile.txt";
#endif
	LogFile = fopen(m_sLogfilePath.c_str(), "w");
#endif
	
	
	m_bSynced = false;
	m_bParked = false;
    m_bLinked = false;

    mATCS.setSerxPointer(m_pSerX);
    mATCS.setTSX(m_pTheSkyXForMounts);
    mATCS.setSleeper(m_pSleeper);

    m_CurrentRateIndex = 0;

	// Read the current stored values for the settings
	if (m_pIniUtil)
	{
	}

    // set mount alignement type and meridian avoidance mode.
    if(strstr(pszDriverSelection,"Fork")) {
        mATCS.setMountMode(MountTypeInterface::Symmetrical_Equatorial);
    }
    else if(strstr(pszDriverSelection,"Equatorial")) {
         mATCS.setMountMode(MountTypeInterface::Asymmetrical_Equatorial);
     }
     else {
         mATCS.setMountMode(MountTypeInterface::AltAz);
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
        fflush(LogFile);
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
        *ppVal = GetLogger();
    else if (!strcmp(pszName, SerialPortParams2Interface_Name))
        *ppVal = dynamic_cast<SerialPortParams2Interface*>(this);
    else if (!strcmp(pszName, DriverSlewsToParkPositionInterface_Name))
        *ppVal = dynamic_cast<DriverSlewsToParkPositionInterface*>(this);

    return SB_OK;
}

#pragma mark - OpenLoopMoveInterface

int X2Mount::startOpenLoopMove(const MountDriverInterface::MoveDir& Dir, const int& nRateIndex)
{
    int nErr = SB_OK;
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

	m_CurrentRateIndex = nRateIndex;
#ifdef ATCS_X2_DEBUG
	if (LogFile) {
		time_t ltime = time(NULL);
		char *timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] startOpenLoopMove called Dir: %d , Rate: %d\n", timestamp, Dir, nRateIndex);
        fflush(LogFile);
	}
#endif

    nErr = mATCS.startOpenSlew(Dir, nRateIndex);
    if(nErr) {
#ifdef ATCS_X2_DEBUG
        if (LogFile) {
            time_t ltime = time(NULL);
            char *timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(LogFile, "[%s] startOpenLoopMove ERROR %d\n", timestamp, nErr);
            fflush(LogFile);
        }
#endif
        m_pLogger->out("startOpenLoopMove ERROR");
        return ERR_CMDFAILED;
    }
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

    nErr = mATCS.stopOpenLoopMove();
    if(nErr) {
#ifdef ATCS_X2_DEBUG
        if (LogFile) {
            time_t ltime = time(NULL);
            char *timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(LogFile, "[%s] endOpenLoopMove ERROR %d\n", timestamp, nErr);
            fflush(LogFile);
        }
#endif
        m_pLogger->out("endOpenLoopMove ERROR");
        return ERR_CMDFAILED;
    }
    return nErr;
}

int X2Mount::rateCountOpenLoopMove(void) const
{
    X2Mount* pMe = (X2Mount*)this;

    X2MutexLocker ml(pMe->GetMutex());
	return pMe->mATCS.getNbSlewRates();
}

int X2Mount::rateNameFromIndexOpenLoopMove(const int& nZeroBasedIndex, char* pszOut, const int& nOutMaxSize)
{
    int nErr = SB_OK;
    std::string sTmp;
    
    nErr = mATCS.getRateName(nZeroBasedIndex, sTmp);
    if(nErr) {
#ifdef ATCS_X2_DEBUG
        if (LogFile) {
            time_t ltime = time(NULL);
            char *timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(LogFile, "[%s] rateNameFromIndexOpenLoopMove ERROR %d\n", timestamp, nErr);
            fflush(LogFile);
        }
#endif
        m_pLogger->out("rateNameFromIndexOpenLoopMove ERROR");
        return ERR_CMDFAILED;
    }
    strncpy(pszOut, sTmp.c_str(), nOutMaxSize);
    return nErr;
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
    std::string sTmp;
    std::string sTime;
    std::string sDate;
    std::string sLongitude;
    std::string sLatitude;
    std::string sTimeZone;
	if (NULL == ui) return ERR_POINTER;
	
	if ((nErr = ui->loadUserInterface("ATCS.ui", deviceType(), m_nPrivateMulitInstanceIndex)))
		return nErr;
	
	if (NULL == (dx = uiutil.X2DX())) {
		return ERR_POINTER;
	}

    X2MutexLocker ml(GetMutex());

	// Set values in the userinterface
    if(m_bLinked) {
        dx->setEnabled("pushButton",true);
        dx->setEnabled("pushButton_2",true);
        dx->setEnabled("pushButton_3",true);
        dx->setEnabled("alignmentType",true);
        dx->setEnabled("pushButton_4",true);

        nErr = mATCS.getSiteName(sTmp);
        if(!nErr)
            dx->setText("siteName", sTmp.c_str());

        nErr = mATCS.getSiteData(sLongitude, sLatitude, sTimeZone);
        if(!nErr) {
            dx->setText("longitude", sLongitude.c_str());
            dx->setText("latitude", sLatitude.c_str());
            dx->setText("timezone", sTimeZone.c_str());
        }
        else {
            dx->setText("longitude", "");
            dx->setText("latitude", "");
            dx->setText("timezone", "");
        }

        nErr = mATCS.getStandardTime(sTime);
        nErr |= mATCS.getStandardDate(sDate);
        if(!nErr) {
            sTmp =sDate + " - " + sTime;
            dx->setText("time_date", sTmp.c_str());
        }

        nErr = mATCS.getTopActiveFault(sTmp);
        if(!nErr)
            dx->setText("activeFault", sTmp.c_str());
    }
    else {
        dx->setText("time_date", "");
        dx->setText("siteName", "");
        dx->setText("longitude", "");
        dx->setText("latitude", "");
        dx->setText("timezone", "");
        dx->setEnabled("pushButton",false);
        dx->setEnabled("pushButton_2",false);
        dx->setEnabled("pushButton_3",false);
        dx->setEnabled("alignmentType",false);
        dx->setEnabled("pushButton_4",false);
    }
	//Display the user interface
	if ((nErr = ui->exec(bPressedOK)))
		return nErr;
	
	//Retreive values from the user interface
	if (bPressedOK) {
	}
	return nErr;
}

void X2Mount::uiEvent(X2GUIExchangeInterface* uiex, const char* pszEvent)
{
    int nErr;
    std::string sTmpBuf;
    std::string sTime;
    std::string sDate;

    if(!m_bLinked)
        return ;

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
        nErr = mATCS.getStandardTime(sTime);
        nErr |= mATCS.getStandardDate(sDate);
        if(!nErr) {
            sTmpBuf = sDate + " - " + sTime;
            uiex->setText("time_date", sTmpBuf.c_str());
        }
        nErr = mATCS.getTopActiveFault(sTmpBuf);
        if(!nErr)
            uiex->setText("activeFault", sTmpBuf.c_str());

	}

    if (!strcmp(pszEvent, "on_pushButton_clicked")) {
        // TSX longitude is + going west and - going east, so passing the opposite
        mATCS.setSiteData( - m_pTheSkyXForMounts->longitude(),
                          m_pTheSkyXForMounts->latitude(),
                          m_pTheSkyXForMounts->timeZone());
    }

    if (!strcmp(pszEvent, "on_pushButton_2_clicked")) {
        nErr = mATCS.syncTime();
        nErr |= mATCS.syncDate();
        nErr |= mATCS.getStandardTime(sTime);
        nErr |= mATCS.getStandardDate(sDate);
        if(!nErr) {
            sTmpBuf = sDate + " - " + sTime;
            uiex->setPropertyString("time_date", "text", sTmpBuf.c_str());
        }
    }

    if (!strcmp(pszEvent, "on_pushButton_3_clicked")) {
        mATCS.markParkPosition();
    }

	return;
}

#pragma mark - LinkInterface
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

#pragma mark - AbstractDriverInfo

void	X2Mount::driverInfoDetailedInfo(BasicStringInterface& str) const
{
	str = "ATCS X2 plugin by Rodolphe Pineau";
}

double	X2Mount::driverInfoVersion(void) const
{
	return PLUGIN_VERSION;
}

void X2Mount::deviceInfoNameShort(BasicStringInterface& str) const
{
    if(m_bLinked) {
        X2Mount* pMe = (X2Mount*)this;
        X2MutexLocker ml(pMe->GetMutex());
        std::string sModel;
        pMe->mATCS.getModel(sModel);
        str = sModel.c_str();
    }
    else
        str = "Not connected1";
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
        std::string sFirmware;
        X2MutexLocker ml(GetMutex());
        mATCS.getFirmwareVersion(sFirmware);
        str = sFirmware.c_str();
    }
    else
        str = "Not connected";
}
void X2Mount::deviceInfoModel(BasicStringInterface& str)
{
    if(m_bLinked) {
        X2MutexLocker ml(GetMutex());
        std::string sModel;
        mATCS.getModel(sModel);
        str = sModel.c_str();
    }
    else
        str = "Not connected";
}

#pragma mark - Common Mount specifics
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

#ifdef ATCS_X2_DEBUG
    if (LogFile) {
        time_t ltime = time(NULL);
        char *timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] raDec Called. Ra : %f , Dec : %f \n", timestamp, ra, dec);
        fprintf(LogFile, "[%s] nErr = %d \n", timestamp, nErr);
        fflush(LogFile);
    }
#endif

	return nErr;
}

int X2Mount::abort()
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

#ifdef ATCS_X2_DEBUG
    if (LogFile) {
        time_t ltime = time(NULL);
        char *timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] Abort nErr = %d \n", timestamp, nErr);
        fflush(LogFile);
    }
#endif

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
    nErr = mATCS.startSlewTo(dRa, dDec);
    if(nErr) {
#ifdef ATCS_X2_DEBUG
        if (LogFile) {
            time_t ltime = time(NULL);
            char *timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(LogFile, "[%s] startSlewTo nErr = %d \n", timestamp, nErr);
            fflush(LogFile);
        }
#endif
        m_pLogger->out("startSlewTo ERROR");
        return ERR_CMDFAILED;
    }

    return nErr;
}

int X2Mount::isCompleteSlewTo(bool& bComplete) const
{
    int nErr = SB_OK;
    if(!m_bLinked)
        return ERR_NOLINK;

    X2Mount* pMe = (X2Mount*)this;
    X2MutexLocker ml(pMe->GetMutex());

    nErr = pMe->mATCS.isSlewToComplete(bComplete);
    if(nErr)
        return ERR_CMDFAILED;

#ifdef ATCS_X2_DEBUG
    if (LogFile) {
        time_t ltime = time(NULL);
        char *timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] isCompleteSlewTo nErr = %d \n", timestamp, nErr);
        fflush(LogFile);
    }
#endif

	return nErr;
}

int X2Mount::endSlewTo(void)
{
#ifdef ATCS_X2_DEBUG
    if (LogFile) {
        time_t ltime = time(NULL);
        char *timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] endSlewTo Called\n", timestamp);
        fflush(LogFile);
    }
#endif
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

#ifdef ATCS_X2_DEBUG
    if (LogFile) {
        time_t ltime = time(NULL);
        char *timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] syncMount nErr = %d \n", timestamp, nErr);
        fflush(LogFile);
    }
#endif

    return nErr;
}

bool X2Mount::isSynced(void)
{
    int nErr;

    if(!m_bLinked)
        return false;

    X2MutexLocker ml(GetMutex());

   nErr = mATCS.isAligned(m_bSynced);

#ifdef ATCS_X2_DEBUG
    if (LogFile) {
        time_t ltime = time(NULL);
        char *timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] isSynced Called : m_bSynced = %s\n", timestamp, m_bSynced?"true":"false");
        fprintf(LogFile, "[%s] isSynced nErr = %d \n", timestamp, nErr);
        fflush(LogFile);
    }
#endif

    return m_bSynced;
}

#pragma mark - TrackingRatesInterface
int X2Mount::setTrackingRates(const bool& bTrackingOn, const bool& bIgnoreRates, const double& dRaRateArcSecPerSec, const double& dDecRateArcSecPerSec)
{
    int nErr = SB_OK;
    double dTrackRaArcSecPerHr;
    double dTrackDecArcSecPerHr;
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    dTrackRaArcSecPerHr = dRaRateArcSecPerSec * 3600;
    dTrackDecArcSecPerHr = dDecRateArcSecPerSec * 3600;

    nErr = mATCS.setTrackingRates(bTrackingOn, bIgnoreRates, dTrackRaArcSecPerHr, dTrackDecArcSecPerHr);
#ifdef ATCS_X2_DEBUG
    if (LogFile) {
        time_t ltime = time(NULL);
        char *timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] setTrackingRates Called. Tracking On: %s , Ra rate : %f , Dec rate: %f\n", timestamp, bTrackingOn?"true":"false", dRaRateArcSecPerSec, dDecRateArcSecPerSec);
        fflush(LogFile);
    }
#endif
    if(nErr)
        return ERR_CMDFAILED;

#ifdef ATCS_X2_DEBUG
    if (LogFile) {
        time_t ltime = time(NULL);
        char *timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] setTrackingRates nErr = %d \n", timestamp, nErr);
        fflush(LogFile);
    }
#endif

    return nErr;
	
}

int X2Mount::trackingRates(bool& bTrackingOn, double& dRaRateArcSecPerSec, double& dDecRateArcSecPerSec)
{
    int nErr = SB_OK;
    double dTrackRaArcSecPerHr;
    double dTrackDecArcSecPerHr;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    nErr = mATCS.getTrackRates(bTrackingOn, dTrackRaArcSecPerHr, dTrackDecArcSecPerHr);
    if(nErr) {
#ifdef ATCS_X2_DEBUG
        if (LogFile) {
            time_t ltime = time(NULL);
            char *timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(LogFile, "[%s] trackingRates  mATCS.getTrackRates nErr = %d \n", timestamp, nErr);
            fflush(LogFile);
        }
#endif
        return ERR_CMDFAILED;
    }
    dRaRateArcSecPerSec = dTrackRaArcSecPerHr / 3600;
    dDecRateArcSecPerSec = dTrackDecArcSecPerHr / 3600;

#ifdef ATCS_X2_DEBUG
    if (LogFile) {
        time_t ltime = time(NULL);
        char *timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] trackingRates Called. Tracking On: %d , Ra rate : %f , Dec rate: %f\n", timestamp, bTrackingOn, dRaRateArcSecPerSec, dDecRateArcSecPerSec);
        fflush(LogFile);
    }
#endif

	return nErr;
}

int X2Mount::siderealTrackingOn()
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
        fprintf(LogFile, "[%s] siderealTrackingOn Called \n", timestamp);
        fflush(LogFile);
    }
#endif

    nErr = setTrackingRates( true, true, 0.0, 0.0);
    if(nErr)
        return ERR_CMDFAILED;

#ifdef ATCS_X2_DEBUG
    if (LogFile) {
        time_t ltime = time(NULL);
        char *timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] siderealTrackingOn nErr = %d \n", timestamp, nErr);
        fflush(LogFile);
    }
#endif

    return nErr;
}

int X2Mount::trackingOff()
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
        fprintf(LogFile, "[%s] trackingOff Called \n", timestamp);
        fflush(LogFile);
    }
#endif
    nErr = setTrackingRates( false, true, 0.0, 0.0);
    if(nErr)
        nErr = ERR_CMDFAILED;

#ifdef ATCS_X2_DEBUG
    if (LogFile) {
        time_t ltime = time(NULL);
        char *timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] trackingOff nErr = %d \n", timestamp, nErr);
        fflush(LogFile);
    }
#endif

    return nErr;
}

#pragma mark - NeedsRefractionInterface
bool X2Mount::needsRefactionAdjustments(void)
{
    bool bEnabled;
    int nErr;

    if(!m_bLinked)
        return false;

    X2MutexLocker ml(GetMutex());

    // check if ATCS refraction adjustment is on.
    nErr = mATCS.getRefractionCorrEnabled(bEnabled);

#ifdef ATCS_X2_DEBUG
    if (LogFile) {
        time_t ltime = time(NULL);
        char *timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] getRefractionCorrEnabled nErr = %d \n", timestamp, nErr);
        fprintf(LogFile, "[%s] getRefractionCorrEnabled bEnabled = %s\n", timestamp, bEnabled?"true":"false");
        fflush(LogFile);
    }
#endif

    return !bEnabled; // if enabled in ATCS, don't ask TSX to do it.
}

#pragma mark - Parking Interface
bool X2Mount::isParked(void)
{
    int nErr;
    bool bTrackingOn;
    bool bIsPArked;
    double dTrackRaArcSecPerHr, dTrackDecArcSecPerHr;

    if(!m_bLinked)
        return false;

    X2MutexLocker ml(GetMutex());

    nErr = mATCS.getAtPark(bIsPArked);
    if(nErr) {
#ifdef ATCS_X2_DEBUG
        if (LogFile) {
            time_t ltime = time(NULL);
            char *timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(LogFile, "[%s] isParked mATCS.getAtPark nErr = %d \n", timestamp, nErr);
            fflush(LogFile);
        }
#endif
        return false;
    }
    if(!bIsPArked) // not parked
        return false;

    // get tracking state.
    nErr = mATCS.getTrackRates(bTrackingOn, dTrackRaArcSecPerHr, dTrackDecArcSecPerHr);
    if(nErr) {
#ifdef ATCS_X2_DEBUG
        if (LogFile) {
            time_t ltime = time(NULL);
            char *timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(LogFile, "[%s] isParked mATCS.getTrackRates nErr = %d \n", timestamp, nErr);
            fflush(LogFile);
        }
#endif
        return false;
    }
    // if AtPark and tracking is off, then we're parked, if not then we're unparked.
    if(bIsPArked && !bTrackingOn)
        m_bParked = true;
    else
        m_bParked = false;
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
#ifdef ATCS_X2_DEBUG
        if (LogFile) {
            time_t ltime = time(NULL);
            char *timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(LogFile, "[%s] startPark  m_pTheSkyXForMounts->HzToEq nErr = %d \n", timestamp, nErr);
            fflush(LogFile);
        }
#endif
        return nErr;
    }

#ifdef ATCS_X2_DEBUG
	if (LogFile) {
		time_t ltime = time(NULL);
		char *timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] startPark Called. Alt: %f , Az: %f [ Ra: %f , Dec: %f]\n", timestamp, dAz, dAlt, dRa, dDec);
        fflush(LogFile);
	}
#endif
    // goto park
    nErr = mATCS.gotoPark(dRa, dDec);
    if(nErr)
        nErr = ERR_CMDFAILED;

#ifdef ATCS_X2_DEBUG
    if (LogFile) {
        time_t ltime = time(NULL);
        char *timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] startPark  mATCS.gotoPark nErr = %d \n", timestamp, nErr);
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

    X2MutexLocker ml(pMe ->GetMutex());

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

#ifdef ATCS_X2_DEBUG
    if (LogFile) {
        time_t ltime = time(NULL);
        char *timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] isCompletePark  mATCS.getAtPark nErr = %d \n", timestamp, nErr);
        fflush(LogFile);
    }
#endif

	return nErr;
}

int X2Mount::endPark(void)
{
    return SB_OK;
}

int X2Mount::startUnpark(void)
{
    int nErr = SB_OK;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    nErr = mATCS.unPark();
    if(nErr) {
#ifdef ATCS_X2_DEBUG
        if (LogFile) {
            time_t ltime = time(NULL);
            char *timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(LogFile, "[%s] startUnpark : mATCS.unPark() failled !\n", timestamp);
            fflush(LogFile);
        }
#endif
        nErr = ERR_CMDFAILED;
    }
    m_bParked = false;
    return nErr;
}

/*!Called to monitor the unpark process.
 \param bComplete Set to true if the unpark is complete, otherwise set to false.
*/
int X2Mount::isCompleteUnpark(bool& bComplete) const
{
    int nErr;
    bool bIsParked;
    bool bTrackingOn;
    double dTrackRaArcSecPerHr, dTrackDecArcSecPerHr;

    if(!m_bLinked)
        return ERR_NOLINK;

    X2Mount* pMe = (X2Mount*)this;

    X2MutexLocker ml(pMe ->GetMutex());

    bComplete = false;

    nErr = pMe->mATCS.getAtPark(bIsParked);
    if(nErr) {
#ifdef ATCS_X2_DEBUG
        if (LogFile) {
            time_t ltime = time(NULL);
            char *timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(LogFile, "[%s] isCompleteUnpark  mATCS.getAtPark nErr = %d \n", timestamp, nErr);
            fflush(LogFile);
        }
#endif
        nErr = ERR_CMDFAILED;
    }
    if(!bIsParked) { // no longer parked.
        bComplete = true;
        pMe->m_bParked = false;
        return nErr;
    }

    // if we're still at the park position
    // get tracking state. If tracking is off, then we're parked, if not then we're unparked.
    nErr = pMe->mATCS.getTrackRates(bTrackingOn, dTrackRaArcSecPerHr, dTrackDecArcSecPerHr);
    if(nErr)
        nErr = ERR_CMDFAILED;
#ifdef ATCS_X2_DEBUG
    if (LogFile) {
        time_t ltime = time(NULL);
        char *timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] isCompleteUnpark  mATCS.getTrackRates nErr = %d \n", timestamp, nErr);
        fflush(LogFile);
    }
#endif

    if(bTrackingOn) {
        bComplete = true;
        pMe->m_bParked = false;
    }
    else {
        bComplete = false;
        pMe->m_bParked = true;
    }
	return SB_OK;
}

/*!Called once the unpark is complete.
 This is called once for every corresponding startUnpark() allowing software implementations of unpark.
 */
int		X2Mount::endUnpark(void)
{
	return SB_OK;
}

#pragma mark - AsymmetricalEquatorialInterface

bool X2Mount::knowsBeyondThePole()
{
    return true;
}

int X2Mount::beyondThePole(bool& bYes) {
	// bYes = mATCS.GetIsBeyondThePole();
	return SB_OK;
}


double X2Mount::flipHourAngle() {
#ifdef ATCS_X2_DEBUG
	if (LogFile) {
		time_t ltime = time(NULL);
		char *timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
		// fprintf(LogFile, "[%s] flipHourAngle called\n", timestamp);
        fflush(LogFile);
	}
#endif

	return 0.0;
}


int X2Mount::gemLimits(double& dHoursEast, double& dHoursWest)
{
    int nErr = SB_OK;
    if(!m_bLinked)
        return ERR_NOLINK;

    X2MutexLocker ml(GetMutex());

    nErr = mATCS.getLimits(dHoursEast, dHoursWest);

#ifdef ATCS_X2_DEBUG
    if (LogFile) {
        time_t ltime = time(NULL);
        char *timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(LogFile, "[%s] gemLimits mATCS.getLimits nErr = %d\n", timestamp, nErr);
        fprintf(LogFile, "[%s] gemLimits dHoursEast = %f\n", timestamp, dHoursEast);
        fprintf(LogFile, "[%s] gemLimits dHoursWest = %f\n", timestamp, dHoursWest);
        fflush(LogFile);
    }
#endif
    // temp debugging.
	dHoursEast = 0.0;
	dHoursWest = 0.0;
	return SB_OK;
}

MountTypeInterface::Type X2Mount::mountType()
{
    return  mATCS.mountType();
}


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


void X2Mount::portNameOnToCharPtr(char* pszPort, const unsigned int& nMaxSize) const
{
    if (NULL == pszPort)
        return;

    snprintf(pszPort, nMaxSize,DEF_PORT_NAME);

    if (m_pIniUtil)
        m_pIniUtil->readString(PARENT_KEY, CHILD_KEY_PORT_NAME, pszPort, pszPort, nMaxSize);

}




