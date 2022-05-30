#include "ATCS.h"

// Constructor for ATCS
ATCS::ATCS()
{

	m_bIsConnected = false;

    m_bDebugLog = true;
    m_bJNOW = false;

    m_b24h = false;
    m_bDdMmYy = false;
    m_bTimeSetOnce = false;
    m_bLimitCached = false;

#ifdef PLUGIN_DEBUG
#if defined(SB_WIN_BUILD)
    m_sLogfilePath = getenv("HOMEDRIVE");
    m_sLogfilePath += getenv("HOMEPATH");
    m_sLogfilePath += "\\ATCSLog.txt";
#elif defined(SB_LINUX_BUILD)
    m_sLogfilePath = getenv("HOME");
    m_sLogfilePath += "/ATCSLog.txt";
#elif defined(SB_MAC_BUILD)
    m_sLogfilePath = getenv("HOME");
    m_sLogfilePath += "/ATCSLog.txt";
#endif
    m_sLogFile.open(m_sLogfilePath, std::ios::out |std::ios::trunc);
#endif

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [ATCS] Version " << std::fixed << std::setprecision(2) << PLUGIN_VERSION << " build " << __DATE__ << " " << __TIME__ << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [ATCS] Constructor Called." << std::endl;
    m_sLogFile.flush();
#endif

}


ATCS::~ATCS(void)
{
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [~ATCS] ATCS Destructor Called" << std::endl;
    m_sLogFile.flush();
#endif

#ifdef    PLUGIN_DEBUG
    // Close LogFile
    if(m_sLogFile.is_open())
        m_sLogFile.close();
#endif
}

int ATCS::Connect(char *pszPort)
{
    int nErr = PLUGIN_OK;
    bool bIsParked;
    bool bIsAligned;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Connect] Connect Called." << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Connect] Trying to connect to port " << pszPort<< std::endl;
    m_sLogFile.flush();
#endif

    // 19200 8N1
    if(m_pSerx->open(pszPort, 19200, SerXInterface::B_NOPARITY, "-DTR_CONTROL 1") == 0)
        m_bIsConnected = true;
    else
        m_bIsConnected = false;

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;
    timer.Reset();
    while(true) {
        nErr = atclEnter();
        if(!nErr)
            break;
        if(timer.GetElapsedSeconds()>3) {
            // if we can't get an answer after 5 seconds
            // we might not be physicaly connected or the controller is off
            m_bIsConnected = false;
            return ERR_NOLINK;
        }
    }
    setAsyncUpdateEnabled(false);
    disablePacketSeqChecking();
	disableStaticStatusChangeNotification();
    m_pSerx->purgeTxRx();

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Connect] m_mountType " << m_mountType << std::endl;
    m_sLogFile.flush();
#endif

    // set mount type
    switch(m_mountType) {
        case MountTypeInterface::Symmetrical_Equatorial:
            setAlignementType("Polar");
            setMeridianAvoidMethod("Lower");
            break;

        case MountTypeInterface::Asymmetrical_Equatorial :
            setAlignementType("Polar");
            setMeridianAvoidMethod("Full(GEM)");
            break;

        case MountTypeInterface::AltAz :
            setAlignementType("AltAz");
            setMeridianAvoidMethod("Lower");
            break;
            
        default :
            break;
    }
    m_bJNOW = true;
    setEpochOfEntry("Now");

    nErr = getLocalTimeFormat(m_b24h);
    nErr = getDateFormat(m_bDdMmYy);
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Connect] Time format : " << (m_b24h?"24H":"12H") << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Connect] Date format : " << (m_bDdMmYy?"D/M/Y":"M/D/Y") << std::endl;
    m_sLogFile.flush();
#endif

    // do we need to set the time ?
    nErr = checkSiteTimeDateSetOnce(m_bTimeSetOnce);
    if(!nErr) {
        if(!m_bTimeSetOnce) {
            nErr = syncTime();
            nErr = syncDate();
        }
    }
    else
    {
        m_bIsConnected = false;
        return ERR_CMDFAILED;
    }

    // are we parked ?
    getAtPark(bIsParked);
    if(!bIsParked) {
        // are we aligned ?
        isAligned(bIsAligned);
        if(bIsAligned) { // if aligned, resume tracking
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Connect] Not parked but aligned, resuming sidereal tracking." << std::endl;
            m_sLogFile.flush();
#endif
            setTrackingRates(true, true, 0, 0);
        }
    }
    m_bLimitCached = false;

    return SB_OK;
}


int ATCS::Disconnect(void)
{
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Disconnect] Disconnect called." << std::endl;
    m_sLogFile.flush();
#endif

    if (m_bIsConnected) {
        if(m_pSerx){
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Disconnect] closing serial port." << std::endl;
            m_sLogFile.flush();
#endif
            m_pSerx->flushTx();
            m_pSerx->purgeTxRx();
            m_pSerx->close();
        }
    }
	m_bIsConnected = false;
    m_bLimitCached = false;

	return SB_OK;
}


int ATCS::getNbSlewRates()
{
    return ATCS_NB_SLEW_SPEEDS;
}

// returns "Slew", "ViewVel4", "ViewVel3", "ViewVel2", "ViewVel1"
int ATCS::getRateName(int nZeroBasedIndex, std::string &sOut)
{
    if (nZeroBasedIndex > ATCS_NB_SLEW_SPEEDS)
        return ATCS_ERROR;

    sOut.assign(m_svSlewRateNames[nZeroBasedIndex]);
    return PLUGIN_OK;
}

#pragma mark - ATCS communication

int ATCS::ATCSSendCommand(const std::string sCmd, std::string &sResp, int nTimeout)
{
    int nErr = PLUGIN_OK;
    unsigned long  ulBytesWrite;
    bool resp_ok = false;

    sResp.clear();
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [ATCSSendCommand std::string] sending " << sCmd << std::endl;
    m_sLogFile.flush();
#endif

    nErr = m_pSerx->writeFile((void *)sCmd.c_str(), sCmd.size(), ulBytesWrite);
    m_pSerx->flushTx();
    if(nErr)
        return nErr;
    // read response
    while(!resp_ok) {
        nErr = ATCSreadResponse(sResp, nTimeout);
        if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            if(sResp[0] == char(ATCL_NACK ))
                m_sLogFile << "["<<getTimeStamp()<<"]"<< " [ATCSSendCommand std::string] ERROR reading response , ATCL_NACK received " << std::uppercase << std::setfill('0') << std::setw(2) << std::hex << (unsigned int)((unsigned char)sResp[0]) << std::endl;
            else if(sResp[0] == char(ATCL_ACK ))
                m_sLogFile << "["<<getTimeStamp()<<"]"<< " [ATCSSendCommand std::string] ERROR reading response , ATCL_ACK received " << std::uppercase << std::setfill('0') << std::setw(2) << std::hex << (unsigned int)((unsigned char)sResp[0]) << std::endl;
            else
                m_sLogFile << "["<<getTimeStamp()<<"]"<< " [ATCSSendCommand std::string] ERROR " << nErr<< " reading response :" << sResp << std::endl;
            m_sLogFile.flush();
            m_sLogFile << std::dec;
#endif
            return nErr;
        }


        // filter out async status and log them in debug mode
        if(sResp.size()) {
            if(sResp[0] == char(ATCL_STATUS) ||
               sResp[0]== char(ATCL_WARNING) ||
               sResp[0] == char(ATCL_ALERT) ||
               sResp[0] == char(ATCL_INTERNAL_ERROR) ||
               sResp[0] == char(ATCL_IDC_ASYNCH)
               ) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
                m_sLogFile << "["<<getTimeStamp()<<"]"<< " [ATCSSendCommand std::string] Async message :  " << sResp.substr(1, sResp.length()) << std::endl;
                m_sLogFile.flush();
#endif
            }
            else {
                if(sResp[0] == char(ATCL_SYNTAX_ERROR)) { // not async but we need to log it
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
                    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [ATCSSendCommand std::string] Async message :  " << sResp.substr(1, sResp.length()) << std::endl;
                    m_sLogFile.flush();
#endif
                }
                resp_ok = true;
            }
        }
    } // end while(!resp_ok)
    if(sResp.size()) {
        sResp = rtrim(sResp, "%");
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        if(sResp[0] == char(ATCL_ACK) )
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [ATCSSendCommand std::string]  got ATCL_ACK : " << std::uppercase << std::setfill('0') << std::setw(2) << std::hex << (unsigned int)((unsigned char)sResp[0]) << std::endl;
        else if(sResp[0] == char(ATCL_NACK) )
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [ATCSSendCommand std::string]  got ATCL_NACK : " << std::uppercase << std::setfill('0') << std::setw(2) << std::hex << (unsigned int)((unsigned char)sResp[0]) << std::endl;
        else {
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [ATCSSendCommand std::string]  got response : " << sResp << std::endl;
        }
        m_sLogFile.flush();
        m_sLogFile << std::dec;
#endif

    }

    return nErr;
}


int ATCS::ATCSreadResponse(std::string &sResp, int nTimeout)
{
    int nErr = PLUGIN_OK;
    char pszBuf[SERIAL_BUFFER_SIZE];
    unsigned long ulBytesRead = 0;
    unsigned long ulTotalBytesRead = 0;
    char *pszBufPtr;
    int nBytesWaiting = 0 ;
    int nbTimeouts = 0;

    memset(pszBuf, 0, SERIAL_BUFFER_SIZE);
    pszBufPtr = pszBuf;

    do {
        nErr = m_pSerx->bytesWaitingRx(nBytesWaiting);
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse std::string] nBytesWaiting      : " << nBytesWaiting << std::endl;
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse std::string] nBytesWaiting nErr : " << nErr << std::endl;
        m_sLogFile.flush();
#endif
        if(!nBytesWaiting) {
            nbTimeouts += MAX_READ_WAIT_TIMEOUT;
            if(nbTimeouts >= nTimeout) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
                m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse std::string] bytesWaitingRx timeout, no data for " << nbTimeouts << " ms"<< std::endl;
                m_sLogFile.flush();
#endif
                nErr = COMMAND_TIMEOUT;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(MAX_READ_WAIT_TIMEOUT));
            continue;
        }
        nbTimeouts = 0;
        if(ulTotalBytesRead + nBytesWaiting <= SERIAL_BUFFER_SIZE) {
            nErr = m_pSerx->readFile(pszBufPtr, nBytesWaiting, ulBytesRead, nTimeout);
            // check for  errors or single ACK
            if(*pszBufPtr == char(ATCL_NACK)) {
#if defined PLUGIN_DEBUG
                m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse std::string] ATCL_NACK received." << std::endl;
                m_sLogFile.flush();
#endif
                ulTotalBytesRead += ulBytesRead;
                nErr = ATCS_BAD_CMD_RESPONSE;
                break;
            }

            if(*pszBufPtr == char(ATCL_ACK)) {
#if defined PLUGIN_DEBUG
                m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse std::string] ATCL_ACK received." << std::endl;
                m_sLogFile.flush();
#endif
                ulTotalBytesRead += ulBytesRead;
                nErr = PLUGIN_OK;
                break;
            }

        }
        else {
            nErr = ERR_RXTIMEOUT;
            break; // buffer is full.. there is a problem !!
        }
        if(nErr) {
#if defined PLUGIN_DEBUG
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse std::string] readFile error : " << nErr << std::endl;
            m_sLogFile.flush();
#endif
            return nErr;
        }

        if (ulBytesRead != nBytesWaiting) { // timeout
#if defined PLUGIN_DEBUG
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse std::string] readFile Timeout Error." << std::endl;
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse std::string] readFile nBytesWaiting : " << nBytesWaiting << std::endl;
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse std::string] readFile ulBytesRead   : " << ulBytesRead << std::endl;
            m_sLogFile.flush();
#endif
        }

        ulTotalBytesRead += ulBytesRead;
        pszBufPtr+=ulBytesRead;
    }  while (ulTotalBytesRead < SERIAL_BUFFER_SIZE  && *(pszBufPtr-1) != ';');


#if defined PLUGIN_DEBUG
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse std::string] ulTotalBytesRead = " << ulTotalBytesRead << std::endl;
    m_sLogFile.flush();
#endif

    if(!ulTotalBytesRead)
        nErr = COMMAND_TIMEOUT; // we didn't get an answer.. so timeout
    else if(*(pszBufPtr-1) == ';')
        *(pszBufPtr-1) = 0; //remove the ';'

    sResp.assign(pszBuf);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [readResponse std::string] sResp : " << sResp << std::endl;
    m_sLogFile.flush();
#endif

    return nErr;
}


int ATCS::atclEnter()
{
    int nErr = PLUGIN_OK;
    std::stringstream ssCmd;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [atclEnter] called." << std::endl;
    m_sLogFile.flush();
#endif

    ssCmd<<char(ATCL_ENTER);
    nErr = ATCSSendCommand(ssCmd.str(), sResp);

    nErr |= ATCSSendCommand("!QDcn;", sResp);

    return nErr;
}


#pragma mark - dome controller informations

int ATCS::getFirmwareVersion(std::string &sFirmware)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getFirmwareVersion] called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = ATCSSendCommand("!HGfv;", sResp);

    if(nErr)
        return nErr;

    sFirmware.assign(sResp);
    m_sFirmwareVersion.assign(sResp);
    return nErr;
}

int ATCS::getModel(std::string &sModel)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getModel] called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = ATCSSendCommand("!HGsm;", sResp);
    if(nErr)
        return nErr;

    sModel.assign(sResp);
    m_sHardwareModel.assign(sResp);
    return nErr;
}


#pragma mark - Mount Coordinates
void ATCS::setMountMode(MountTypeInterface::Type mountType)
{
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setMountMode]  mountType : " << mountType << std::endl;
    m_sLogFile.flush();
#endif

    m_mountType = mountType;
}

MountTypeInterface::Type ATCS::mountType()
{
    return m_mountType;
}


int ATCS::getRaAndDec(double &dRa, double &dDec)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRaAndDec] called." << std::endl;
    m_sLogFile.flush();
#endif

    // get RA
    nErr = ATCSSendCommand("!CGra;", sResp);
    if(nErr) {
        return nErr;
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRaAndDec]  sResp : " << sResp << std::endl;
    m_sLogFile.flush();
#endif

    // if not aligned we have no coordinates.
    if(sResp.find("N/A") != -1) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRaAndDec]  Not aligned yet." << std::endl;
        m_sLogFile.flush();
#endif
        dRa = 0.0f;
        dDec = 0.0f;
        return nErr;
    }

    nErr = convertHHMMSStToRa(sResp, dRa);
    if(nErr)
        return nErr;

    // get DEC
    nErr = ATCSSendCommand("!CGde;", sResp);
    if(nErr)
        return nErr;
    // even if RA was ok, we need to test Dec as we might have reach park between the 2 calls
    if(sResp.find("N/A") != -1) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRaAndDec]  Not aligned yet." << std::endl;
        m_sLogFile.flush();
#endif
        dDec = 0.0f;
        return nErr;
    }
    nErr = convertDDMMSSToDecDeg(sResp, dDec);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRaAndDec] dRa  : " << dRa << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRaAndDec] dDec : " << dDec << std::endl;
    m_sLogFile.flush();
#endif

    return nErr;
}

int ATCS::setTarget(double dRa, double dDec)
{
    int nErr;

    std::stringstream ssTmp;
    std::string sCmd;
    std::string sResp;
    std::string sTemp;
    char cSign;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTarget]  Ra  : " << std::fixed << std::setprecision(12) << dRa << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTarget]  Dec : " << std::fixed << std::setprecision(12) << dDec << std::endl;
    m_sLogFile.flush();
#endif

    // convert Ra value to HH:MM:SS.T before passing them to the ATCS
    convertRaToHHMMSSt(dRa, sTemp);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTarget]  Ra  : " << sTemp << std::endl;
    m_sLogFile.flush();
#endif
    // set target Ra
    sCmd = "!CStr" + sTemp + ";";
    nErr = ATCSSendCommand(sCmd, sResp);

    convertDecDegToDDMMSS(dDec, sTemp, cSign);
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTarget]  Dec : " << cSign << sTemp << std::endl;
    m_sLogFile.flush();
#endif
    // set target dec

    ssTmp << "!CStd" << cSign << sTemp << ";";
    nErr = ATCSSendCommand(ssTmp.str(), sResp);

    return nErr;
}

#pragma mark - Sync and Cal
int ATCS::syncTo(double dRa, double dDec)
{
    int nErr = PLUGIN_OK;
    bool bAligned;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [syncTo]  Ra  : " << std::fixed << std::setprecision(12) << dRa << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [syncTo]  Dec : " << std::fixed << std::setprecision(12) << dDec << std::endl;
    m_sLogFile.flush();
#endif

    nErr = isAligned(bAligned);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [syncTo] isAligned nErr = " << nErr << std::endl;
        m_sLogFile.flush();
#endif
        return nErr;
    }
    // set sync target coordinate
    nErr = setTarget(dRa, dDec);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [syncTo] setTarget nErr = " << nErr << std::endl;
        m_sLogFile.flush();
#endif
        return nErr;
    }
    // Sync
    if(!bAligned){
        nErr = alignFromTargetRA_DecCalcSideEpochNow();
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [syncTo] alignFromTargetRA_DecCalcSideEpochNow nErr = " << nErr << std::endl;
        m_sLogFile.flush();
#endif
    }
    else {
        nErr = calFromTargetRA_DecEpochNow();
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [syncTo] calFromTargetRA_DecEpochNow nErr = " << nErr << std::endl;
        m_sLogFile.flush();
#endif
    }
    return nErr;
}

int ATCS::calFromTargetRA_DecEpochNow()
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [calFromTargetRA_DecEpochNow] called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = ATCSSendCommand("!ACrn;", sResp);
    return nErr;
}

int ATCS::calFromTargetRA_Dec()
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [calFromTargetRA_Dec] called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = ATCSSendCommand("!ACrd;", sResp);
    return nErr;
}


int ATCS::isAligned(bool &bAligned)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isAligned] called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = ATCSSendCommand("!AGas;", sResp);
    if(nErr)
        return nErr;
    if(sResp.find("NotAligned") != -1) {
        bAligned = false;
    }
    else if(sResp.find("Preliminary") != -1) {
        bAligned = false;
    }
    else if(sResp.find("Complete") != -1) {
        bAligned = true;
    }
    else
        bAligned = false;

    return nErr;
}

int ATCS::getAlignementType(std::string &sType)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getAlignementType] called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = ATCSSendCommand("!NGat;", sResp);
    if(nErr)
        return nErr;

    sType.assign(sResp);
    return nErr;
}

int ATCS::setAlignementType(std::string sType)
{
    int nErr = PLUGIN_OK;
    std::string sCmd;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setAlignementType] called." << std::endl;
    m_sLogFile.flush();
#endif

    sCmd = "!NSat" + sType + ";";
    nErr = ATCSSendCommand(sCmd, sResp);
    return nErr;
}


int ATCS::setMeridianAvoidMethod(std::string sType)
{
    int nErr = PLUGIN_OK;
    std::string sCmd;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setMeridianAvoidMethod] called." << std::endl;
    m_sLogFile.flush();
#endif

    sCmd = "!NSam" + sType + ";";
    nErr = ATCSSendCommand(sCmd, sResp);

    return nErr;
}

int ATCS::getMeridianAvoidMethod(std::string &sType)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getMeridianAvoidMethod] called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = ATCSSendCommand("!NGam;", sResp);
    if(nErr)
        return nErr;

    sType.assign(sResp);
    return nErr;
}


#pragma mark - tracking rates
int ATCS::setTrackingRates(bool bTrackingOn, bool bIgnoreRates, double dTrackRaArcSecPerHr, double dTrackDecArcSecPerHr)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTrackingRates] called." << std::endl;
    m_sLogFile.flush();
#endif

    if(!bTrackingOn) { // stop tracking
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTrackingRates] setting to Drift." << std::endl;
        m_sLogFile.flush();
#endif
        nErr = ATCSSendCommand("!RStrDrift;", sResp);

    }
    else if(bTrackingOn && bIgnoreRates) { // sidereal
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTrackingRates] setting to Sidereal." << std::endl;
        m_sLogFile.flush();
#endif
        nErr = ATCSSendCommand("!RStrSidereal;", sResp);
    }
    else { // custom rate
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTrackingRates] setting to Custom." << std::endl;
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTrackingRates] dTrackRaArcSecPerHr  : " << std::fixed << std::setprecision(12) << dTrackRaArcSecPerHr << std::endl;
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setTrackingRates] dTrackDecArcSecPerHr : " << std::fixed << std::setprecision(12) << dTrackDecArcSecPerHr << std::endl;
        m_sLogFile.flush();
#endif
        nErr = setCustomTRateOffsetRA(dTrackRaArcSecPerHr);
        nErr |= setCustomTRateOffsetDec(dTrackDecArcSecPerHr);
        if(nErr) {
            return nErr; // if we cant set the rate no need to switch to custom.
        }
        nErr = ATCSSendCommand("!RStrCustom;", sResp);
    }
    return nErr;
}

int ATCS::getTrackRates(bool &bTrackingOn, double &dTrackRaArcSecPerHr, double &dTrackDecArcSecPerHr)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getTrackRates] called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = ATCSSendCommand("!RGtr;", sResp);
    bTrackingOn = true;
    if(sResp.find("Drift") != -1) {
        bTrackingOn = false;
    }
    nErr = getCustomTRateOffsetRA(dTrackRaArcSecPerHr);
    nErr |= getCustomTRateOffsetDec(dTrackDecArcSecPerHr);

    return nErr;
}

int ATCS::setCustomTRateOffsetRA(double dRa)
{
    int nErr;
    std::string sResp;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setCustomTRateOffsetRA] called." << std::endl;
    m_sLogFile.flush();
#endif

    ssTmp << "!RSor" << std::fixed << std::setprecision(2) << dRa << ";";
    nErr = ATCSSendCommand(ssTmp.str(), sResp);
    if(nErr) {
#if defined PLUGIN_DEBUG
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setCustomTRateOffsetRA] Error setting Ra tracking rate to " << std::fixed << std::setprecision(12) <<  dRa << std::endl;
        m_sLogFile.flush();
#endif
    }
    return nErr;
}

int ATCS::setCustomTRateOffsetDec(double dDec)
{
    int nErr;
    std::string sResp;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setCustomTRateOffsetDec] called." << std::endl;
    m_sLogFile.flush();
#endif

    ssTmp << "!RSod" << std::fixed << std::setprecision(2) << dDec << ";";
    nErr = ATCSSendCommand(ssTmp.str(), sResp);
    if(nErr) {
#if defined PLUGIN_DEBUG
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setCustomTRateOffsetDec] Error setting Dec tracking rate to " << std::fixed << std::setprecision(12) <<  dDec << std::endl;
        m_sLogFile.flush();
#endif
    }
    return nErr;
}

int ATCS::getCustomTRateOffsetRA(double &dTrackRaArcSecPerHr)
{
    int nErr;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getCustomTRateOffsetRA] called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = ATCSSendCommand("!RGor;", sResp);
    if(nErr)
        return nErr;
    dTrackRaArcSecPerHr = std::stof(sResp);

    return nErr;
}

int ATCS::getCustomTRateOffsetDec(double &dTrackDecArcSecPerHr)
{
    int nErr;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getCustomTRateOffsetDec] called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = ATCSSendCommand("!RGod;", sResp);
    if(nErr)
        return nErr;
    dTrackDecArcSecPerHr = std::stof(sResp);

    return nErr;
}

#pragma mark - Limis
int ATCS::getLimits(double &dHoursEast, double &dHoursWest)
{
    int nErr = PLUGIN_OK;
    double dEast, dWest;

    if(m_bLimitCached) {
        dHoursEast = m_dHoursEast;
        dHoursWest = m_dHoursWest;
        return nErr;
    }

    nErr = getSoftLimitEastAngle(dEast);
    if(nErr)
        return nErr;

    nErr = getSoftLimitWestAngle(dWest);
    if(nErr)
        return nErr;

    dHoursEast = m_pTsx->hourAngle(dEast);
    dHoursWest = m_pTsx->hourAngle(dWest);

    m_bLimitCached = true;
    m_dHoursEast = dHoursEast;
    m_dHoursWest = dHoursWest;
    return nErr;
}

int ATCS::getSoftLimitEastAngle(double &dAngle)
{
    int nErr;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getSoftLimitEastAngle] called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = ATCSSendCommand("!NGle;", sResp);
    if(nErr)
        return nErr;
    dAngle = std::stof(sResp);

    return nErr;
}

int ATCS::getSoftLimitWestAngle(double &dAngle)
{
    int nErr;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getSoftLimitWestAngle] called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = ATCSSendCommand("!NGlw;", sResp);
    if(nErr)
        return nErr;
    dAngle = std::stof(sResp);

    return nErr;
}

#pragma mark - Slew

int ATCS::startSlewTo(double dRa, double dDec)
{
    int nErr = PLUGIN_OK;
    bool bAligned;

    nErr = isAligned(bAligned);
    if(nErr)
        return nErr;

    // set sync target coordinate
    nErr = setTarget(dRa, dDec);
    if(nErr)
        return nErr;

    slewTargetRA_DecEpochNow();
    return nErr;
}

int ATCS::slewTargetRA_DecEpochNow()
{
    int nErr;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [slewTargetRA_DecEpochNow] called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = ATCSSendCommand("!GTrn;", sResp);
    timer.Reset();
    return nErr;

}

int ATCS::startOpenSlew(const MountDriverInterface::MoveDir Dir, unsigned int nRate)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;


    m_nOpenLoopDir = Dir;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [startOpenSlew] setting Dir to " << Dir << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [startOpenSlew] setting rate to " << nRate << std::endl;
    m_sLogFile.flush();
#endif

    // select rate
    if(nRate == 4) { // "Slew"
        nErr = ATCSSendCommand("!KSsl;", sResp);
    }
    else {
        // clear slew
        nErr = ATCSSendCommand("!KCsl;", sResp);
        // select rate
        // KScv + 1,2 3 or 4 for ViewVel 1,2,3,4, 'ViewVel 1' is index 0 so nRate+1
        ssTmp << "!KScv" << (nRate+1) << ";";
        nErr = ATCSSendCommand(ssTmp.str(), sResp);
    }
    // figure out direction
    switch(Dir){
        case MountDriverInterface::MD_NORTH:
            nErr = ATCSSendCommand("!KSpu100;", sResp);
            break;
        case MountDriverInterface::MD_SOUTH:
            nErr = ATCSSendCommand("!KSpd100;", sResp);
            break;
        case MountDriverInterface::MD_EAST:
            nErr = ATCSSendCommand("!KSpl100;", sResp);
            break;
        case MountDriverInterface::MD_WEST:
            nErr = ATCSSendCommand("!KSsr100;", sResp);
            break;
    }

    return nErr;
}

int ATCS::stopOpenLoopMove()
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [stopOpenLoopMove] dir was " << m_nOpenLoopDir << std::endl;
    m_sLogFile.flush();
#endif

    switch(m_nOpenLoopDir){
        case MountDriverInterface::MD_NORTH:
        case MountDriverInterface::MD_SOUTH:
            nErr = ATCSSendCommand("!XXud;", sResp);
            break;
        case MountDriverInterface::MD_EAST:
        case MountDriverInterface::MD_WEST:
            nErr = ATCSSendCommand("!XXlr;", sResp);
            break;
    }

    return nErr;
}

int ATCS::isSlewToComplete(bool &bComplete)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    int nPrecentRemaining;

    bComplete = false;

    if(timer.GetElapsedSeconds()<2) {
        // we're checking for comletion to quickly, assume it's moving for now
        return nErr;
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isSlewToComplete] called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = ATCSSendCommand("!GGgr;", sResp);
    if(nErr)
        return nErr;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isSlewToComplete] szResp : " << sResp << std::endl;
    m_sLogFile.flush();
#endif

    // remove the %
    sResp = rtrim(sResp, "%");

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isSlewToComplete] szResp : " << sResp << std::endl;
    m_sLogFile.flush();
#endif

    nPrecentRemaining = std::stoi(sResp);
    if(nPrecentRemaining == 0)
        bComplete = true;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [isSlewToComplete] Slew is finished  : " << (bComplete?"Yes":"No") << std::endl;
    m_sLogFile.flush();
#endif

    return nErr;
}

int ATCS::gotoPark(double dRa, double dDec)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [gotoPark] called." << std::endl;
    m_sLogFile.flush();
#endif

    // goto park
    nErr = ATCSSendCommand("!GTop;", sResp);

    return nErr;
}

int ATCS::markParkPosition()
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [markParkPosition] called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = ATCSSendCommand("!AMpp;", sResp);

    return nErr;

}

int ATCS::getAtPark(bool &bParked)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getAtPark] called." << std::endl;
    m_sLogFile.flush();
#endif

    bParked = false;
    nErr = ATCSSendCommand("!AGak;", sResp);
    if(sResp.find("Yes") != -1) {
        bParked = true;
    }
    return nErr;
}

int ATCS::unPark()
{
    int nErr = PLUGIN_OK;
    bool bAligned;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [unPark] called." << std::endl;
    m_sLogFile.flush();
#endif

    // are we aligned ?
    nErr = isAligned(bAligned);
    if(nErr) {
#ifdef PLUGIN_DEBUG
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [unPark] Error getting alignement status." << std::endl;
        m_sLogFile.flush();
#endif
        return nErr;
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [unPark] bAligned = " << (bAligned?"true":"false") << std::endl;
    m_sLogFile.flush();
#endif

    // if not
    if(!bAligned) {
        nErr = alignFromLastPosition();
        if(nErr) {
#ifdef PLUGIN_DEBUG
            m_sLogFile << "["<<getTimeStamp()<<"]"<< " [unPark] error aligning to last position." << std::endl;
            m_sLogFile.flush();
#endif
        }
    }
    nErr = setTrackingRates(true, true, 0, 0);
    if(nErr) {
#ifdef PLUGIN_DEBUG
        m_sLogFile << "["<<getTimeStamp()<<"]"<< " [unPark] Error setting track rate to Sidereal." << std::endl;
        m_sLogFile.flush();
#endif
    }
    return nErr;
}


int ATCS::getRefractionCorrEnabled(bool &bEnabled)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getRefractionCorrEnabled] called." << std::endl;
    m_sLogFile.flush();
#endif

    bEnabled = false;
    nErr = ATCSSendCommand("!PGre;", sResp);
    if(strncmp(szResp,"Yes",SERIAL_BUFFER_SIZE) == 0) {
        bEnabled = true;
    }
    return nErr;
}

int ATCS::setRefractionCorrEnabled(bool bEnable)
{
    int nErr = PLUGIN_OK;
    std::string sCmd;
    std::string sResp;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setRefractionCorrEnabled] called." << std::endl;
    m_sLogFile.flush();
#endif

    if(bEnable) {
        sCmd = "!PSreYes;";
    }
    else  {
        sCmd = "!PSreNo;";
    }
    nErr = ATCSSendCommand(sCmd, sResp);

    return nErr;
}


int ATCS::Abort()
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [Abort] called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = ATCSSendCommand("!XXxx;", sResp);
    return nErr;
}

#pragma mark - time and site methods
int ATCS::syncTime()
{
    int nErr = PLUGIN_OK;
    int yy, mm, dd, h, min, dst;
    double sec;
    int n12h_time;

    std::string sCmd;
    std::string sResp;
    std::stringstream ssTmp;
    std::string sAmPm;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [syncTime] called." << std::endl;
    m_sLogFile.flush();
#endif

    m_pTsx->localDateTime(yy, mm, dd, h, min, sec, dst);
    if(!m_b24h) {
        n12h_time = h%12;
        if(h<12)
            sAmPm = "AM";
        else
            sAmPm = "PM";

        ssTmp << std::setfill('0') << std::setw(2) << n12h_time << ":" << std::setfill('0') << std::setw(2) << min << ":" << std::setfill('0') << std::setw(5) << std::fixed << std::setprecision(2) << sec << sAmPm;
    }
    else {
        ssTmp << std::setfill('0') << std::setw(2) << h << ":" << std::setfill('0') << std::setw(2) << min << ":" << std::setfill('0') << std::setw(5) << std::fixed << std::setprecision(2) << sec;
    }

    sCmd = "!TSst" + ssTmp.str() + ";";
#ifdef PLUGIN_DEBUG
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [syncTime] setting time to : " << ssTmp.str() << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [syncTime] Command is  : " << sCmd << std::endl;
    m_sLogFile.flush();
#endif

    nErr = ATCSSendCommand(sCmd, sResp);
    getStandardTime(m_sTime);

    return nErr;
}


int ATCS::syncDate()
{
    int nErr = PLUGIN_OK;
    int yy, mm, dd, h, min, dst;
    double sec;

    std::string sCmd;
    std::string sResp;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [syncDate] called." << std::endl;
    m_sLogFile.flush();
#endif

    m_pTsx->localDateTime(yy, mm, dd, h, min, sec, dst);
    // yy is actually yyyy, need conversion to yy, 2017 -> 17
    yy = yy - (int(yy / 1000) * 1000);

    if(!m_bDdMmYy) {
        ssTmp << std::setfill('0') << std::setw(2) << mm << "/" << std::setfill('0') << std::setw(2) << dd << "/" << std::setfill('0') << std::setw(2) << yy;
    }
    else {
        ssTmp << std::setfill('0') << std::setw(2) << dd << "/" << std::setfill('0') << std::setw(2) << mm << "/" << std::setfill('0') << std::setw(2) << yy;
    }

    sCmd = "!TSsd" + ssTmp.str() + ";";

#ifdef PLUGIN_DEBUG
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [syncDate] setting time to : " << ssTmp.str() << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [syncDate] Command is  : " << sCmd << std::endl;
    m_sLogFile.flush();
#endif

    nErr = ATCSSendCommand(sCmd, sResp);

    getStandardDate(m_sDate);
    return nErr;
}

int ATCS::getSiteName(std::string &sSiteName)
{
    int nErr = PLUGIN_OK;
    int nSiteNb;

    nErr = getUsingSiteNumber(nSiteNb);
    if(nErr)
        return nErr;

    nErr = getUsingSiteName(nSiteNb, sSiteName);

    return nErr;
}

int ATCS::setSiteData(double dLongitude, double dLatitute, double dTimeZone)
{
    int nErr = PLUGIN_OK;
    char cSignLong;
    char cSignLat;
    std::string sLong;
    std::string sLat;
    std::string sTimeZone;
    std::string sHH;
    std::string sMM;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteData] dLongitude " << std::fixed << std::setprecision(12) << dLongitude << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteData] dLatitute " << std::fixed << std::setprecision(12) << dLatitute << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteData] dTimeZone " << std::fixed << std::setprecision(12) << dTimeZone << std::endl;
    m_sLogFile.flush();
#endif

    convertDecDegToDDMMSS(dLongitude, sLong, cSignLong);
    convertDecDegToDDMMSS(dLatitute, sLat, cSignLat);
    ssTmp<< std::setfill('0') << std::setw(2) << int(fabs(dTimeZone));
    sHH = ssTmp.str();
    std::stringstream().swap(ssTmp);
    ssTmp<< std::setfill('0') << std::setw(2) << (int((fabs(dTimeZone) - int(fabs(dTimeZone)))) * 100);
    sMM = ssTmp.str();

    if(dTimeZone<0) {
        sTimeZone = sHH + ":" + sMM + "W";
    }
    else if (dTimeZone>0) {
        sTimeZone = sHH + ":" + sMM + "E";
    }
    else
        sTimeZone = "00:00";

    // Set the W/E
    if(dLongitude<0) {
        sLong += "W";
    }
    else {
        sLong += "E";
    }

    // convert signed latitude to N/S
    if(dLatitute>=0) {
        sLat += "N";
    }
    else {
        sLat += "S";
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteData] dLongitude " << sLong << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteData] dLatitute " << sLat << std::endl;
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteData] dTimeZone " << sTimeZone << std::endl;
    m_sLogFile.flush();
#endif

    nErr = setSiteLongitude(m_nSiteNumber, sLong);
    nErr |= setSiteLatitude(m_nSiteNumber, sLat);
    nErr |= setSiteTimezone(m_nSiteNumber, sTimeZone);

    return nErr;
}
int ATCS::getSiteData(std::string &sLongitude, std::string &sLatitude, std::string &sTimeZone)
{
    int nErr = PLUGIN_OK;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getSiteData] called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = getSiteLongitude(m_nSiteNumber, sLongitude);
    nErr |= getSiteLatitude(m_nSiteNumber, sLatitude);
    nErr |= getSiteTZ(m_nSiteNumber, sTimeZone);
    return nErr;
}

#pragma mark - Special commands & functions

int ATCS::getTopActiveFault(std::string &sFault)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getTopActiveFault] called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = ATCSSendCommand("!HGtf;", sResp);
    if(nErr)
        return nErr;
    sFault.assign(sResp);
    return nErr;
}

int ATCS::disablePacketSeqChecking()
{
    int nErr;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [disablePacketSeqChecking] called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = ATCSSendCommand("!QDps;", sResp);

    return nErr;
}

int ATCS::disableStaticStatusChangeNotification()
{
    int nErr;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [disableStaticStatusChangeNotification] called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = ATCSSendCommand("!QDcn;", sResp);
    return nErr;
}

#pragma mark - site data
int ATCS::checkSiteTimeDateSetOnce(bool &bSet)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [checkSiteTimeDateSetOnce] called." << std::endl;
    m_sLogFile.flush();
#endif

    bSet = false;
    nErr = ATCSSendCommand("!ACst;", sResp);
    if(sResp.find("Yes") != -1) {
        bSet = true;
    }
    return nErr;
}

int ATCS::getUsingSiteNumber(int &nSiteNb)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getUsingSiteNumber] called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = ATCSSendCommand("!SGuu;", sResp);
    if(nErr)
        return nErr;

    nSiteNb = std::stoi(sResp);
    m_nSiteNumber = nSiteNb;
    return nErr;

}

int ATCS::getUsingSiteName(int nSiteNb, std::string &sSiteName)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getUsingSiteName] called." << std::endl;
    m_sLogFile.flush();
#endif

    ssTmp << "!SGun" << nSiteNb <<";";
    nErr = ATCSSendCommand(ssTmp.str(), sResp);
    if(nErr)
        return nErr;
    sSiteName.assign(sResp);
    return nErr;
}

int ATCS::setSiteLongitude(int nSiteNb, const std::string sLongitude)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteLongitude] called." << std::endl;
    m_sLogFile.flush();
#endif

    ssTmp << "!SSo" << nSiteNb << sLongitude <<";";
    nErr = ATCSSendCommand(ssTmp.str(), sResp);

    return nErr;
}

int ATCS::setSiteLatitude(int nSiteNb, const std::string sLatitude)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteLatitude] called." << std::endl;
    m_sLogFile.flush();
#endif

    ssTmp << "!SSa" << nSiteNb << sLatitude <<";";
    nErr = ATCSSendCommand(ssTmp.str(), sResp);

    return nErr;
}

int ATCS::setSiteTimezone(int nSiteNb, const std::string sTimezone)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setSiteTimezone] called." << std::endl;
    m_sLogFile.flush();
#endif

    ssTmp << "!SSz" << nSiteNb << sTimezone <<";";
    nErr = ATCSSendCommand(ssTmp.str(), sResp);

    return nErr;
}

int ATCS::getSiteLongitude(int nSiteNb, std::string &sLongitude)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getSiteLongitude] called." << std::endl;
    m_sLogFile.flush();
#endif

    ssTmp << "!SGo" << nSiteNb <<";";
    nErr = ATCSSendCommand(ssTmp.str(), sResp);
    if(!nErr) {
        sLongitude.assign(sResp);
    }
    return nErr;
}

int ATCS::getSiteLatitude(int nSiteNb, std::string &sLatitude)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getSiteLatitude] called." << std::endl;
    m_sLogFile.flush();
#endif

    ssTmp << "!SGa" << nSiteNb <<";";
    nErr = ATCSSendCommand(ssTmp.str(), sResp);
    if(!nErr) {
        sLatitude.assign(sResp);
    }

    return nErr;
}

int ATCS::getSiteTZ(int nSiteNb, std::string &sTimeZone)
{
    int nErr = PLUGIN_OK;
    std::string sResp;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getSiteTZ] called." << std::endl;
    m_sLogFile.flush();
#endif

    ssTmp << "!SGz" << nSiteNb <<";";
    nErr = ATCSSendCommand(ssTmp.str(), sResp);
    if(!nErr) {
        sTimeZone.assign(sResp);
    }

    return nErr;
}


#pragma mark  - Time and Date

int ATCS::getLocalTimeFormat(bool &b24h)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getLocalTimeFormat] called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = ATCSSendCommand("!TGlf;", sResp);

    if(nErr)
        return nErr;
    b24h = false;
    if(sResp.find("24hr") != -1) {
        b24h = true;
    }

    return nErr;
}

int ATCS::getDateFormat(bool &bDdMmYy )
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getDateFormat] called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = ATCSSendCommand("!TGdf;", sResp);
    if(nErr)
        return nErr;
    bDdMmYy = false;
    if(sResp.find("dd/mm/yy") != -1) {
        bDdMmYy = true;
    }
    return nErr;
}

int ATCS::getStandardTime(std::string &sTime)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getStandardTime] called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = ATCSSendCommand("!TGst;", sResp);
    if(nErr)
        return nErr;
    sTime.assign(sResp);
    return nErr;
}

int ATCS::getStandardDate(std::string &sDate)
{
    int nErr = PLUGIN_OK;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [getStandardDate] called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = ATCSSendCommand("!TGsd;", sResp);
    if(nErr)
        return nErr;

    sDate.assign(sResp);
    return nErr;
}


int ATCS::setAsyncUpdateEnabled(bool bEnable)
{
    int nErr = PLUGIN_OK;
    std::string sCmd;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setAsyncUpdateEnabled] called." << std::endl;
    m_sLogFile.flush();
#endif

    if(bEnable) {
        sCmd = "!QSauYes;";
    }
    else  {
        sCmd = "!QSauNo;";
    }
    nErr = ATCSSendCommand(sCmd, sResp);

    return nErr;
}

int ATCS::setEpochOfEntry(const char *szEpoch)
{
    int nErr;
    std::string sResp;
    std::stringstream ssTmp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [setEpochOfEntry] called." << std::endl;
    m_sLogFile.flush();
#endif

    ssTmp << "!PSep" << szEpoch <<";";
    nErr = ATCSSendCommand(ssTmp.str(), sResp);
    return nErr;
}

int ATCS::alignFromTargetRA_DecCalcSide()
{
    int nErr;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [alignFromTargetRA_DecCalcSide] called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = ATCSSendCommand("!AFcs;", sResp);

    return nErr;
}

int ATCS::alignFromTargetRA_DecCalcSideEpochNow()
{
    int nErr;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [alignFromTargetRA_DecCalcSideEpochNow] called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = ATCSSendCommand("!AFcn;", sResp);

    return nErr;
}

int ATCS::alignFromLastPosition()
{
    int nErr;
    std::string sResp;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [alignFromLastPosition] called." << std::endl;
    m_sLogFile.flush();
#endif

    nErr = ATCSSendCommand("!AFlp;", sResp);

    return nErr;
}


void ATCS::convertDecDegToDDMMSS(double dDeg, std::string  &sResult, char &cSign)
{
    int DD, MM, SS;
    double mm, ss;
    std::stringstream ssTmp;

    // convert dDeg decimal value to sDD:MM:SS

    cSign = dDeg>=0?'+':'-';
    dDeg = fabs(dDeg);
    DD = int(dDeg);
    mm = dDeg - DD;
    MM = int(mm*60);
    ss = (mm*60) - MM;
    SS = int(std::roundf(ss*60));

    ssTmp <<  std::setfill('0') << std::setw(2) << DD << ":" << std::setfill('0') << std::setw(2) << MM << ":" << std::setfill('0') << std::setw(2) << SS;
    sResult.assign(ssTmp.str());
}


int ATCS::convertDDMMSSToDecDeg(const std::string StrDeg, double &dDecDeg)
{
    int nErr = PLUGIN_OK;
    std::vector<std::string> vFieldsData;

    dDecDeg = 0;

    nErr = parseFields(StrDeg, vFieldsData, ':');
    if(nErr)
        return nErr;

    if(vFieldsData.size() >= 3) {
        dDecDeg = std::stod(vFieldsData[0]);
        if(dDecDeg <0) {
            dDecDeg = dDecDeg - std::stod(vFieldsData[1])/60.0 - std::stod(vFieldsData[2])/3600.0;
        }
        else {
            dDecDeg = dDecDeg + std::stod(vFieldsData[1])/60.0 + std::stod(vFieldsData[2])/3600.0;
        }
    }
    else
        nErr = ERR_PARSE;

    return nErr;
}


void ATCS::convertRaToHHMMSSt(double dRa, std::string &sResult)
{
    int HH, MM;
    double hh, mm, SSt;
    std::stringstream ssTmp;

    sResult.clear();
    // convert Ra value to HH:MM:SS.S
    HH = int(dRa);
    hh = dRa - HH;
    MM = int(hh*60);
    mm = (hh*60) - MM;
    SSt = mm * 60;

    ssTmp << std::setfill('0') << std::setw(2) << HH << ":" << std::setfill('0') << std::setw(2) << MM << ":" << std::setfill('0') << std::setw(4) << std::fixed << std::setprecision(1) << SSt;
    sResult.assign(ssTmp.str());

}


int ATCS::convertHHMMSStToRa(const std::string StrRa, double &dRa)
{
    int nErr = PLUGIN_OK;
    std::vector<std::string> vFieldsData;

    dRa = 0;

    nErr = parseFields(StrRa, vFieldsData, ':');
    if(nErr)
        return nErr;

    if(vFieldsData.size() >= 3) {
        dRa = atof(vFieldsData[0].c_str()) + atof(vFieldsData[1].c_str())/60.0 + atof(vFieldsData[2].c_str())/3600.0;
    }
    else
        nErr = ERR_PARSE;

    return nErr;
}

int ATCS::parseFields(const std::string szIn, std::vector<std::string> &svFields, char cSeparator)
{
    int nErr = PLUGIN_OK;
    std::string sSegment;
    std::stringstream ssTmp(szIn);

    svFields.clear();
    // split the string into vector elements
    while(std::getline(ssTmp, sSegment, cSeparator))
    {
        svFields.push_back(sSegment);
    }

    if(svFields.size()==0) {
        nErr = ERR_PARSE;
    }
    return nErr;
}

std::string& ATCS::trim(std::string &str, const std::string& filter )
{
    return ltrim(rtrim(str, filter), filter);
}

std::string& ATCS::ltrim(std::string& str, const std::string& filter)
{
    str.erase(0, str.find_first_not_of(filter));
    return str;
}

std::string& ATCS::rtrim(std::string& str, const std::string& filter)
{
    str.erase(str.find_last_not_of(filter) + 1);
    return str;
}




#ifdef PLUGIN_DEBUG
void ATCS::log(std::string sLogEntry)
{
    m_sLogFile << "["<<getTimeStamp()<<"]"<< " [log] " << sLogEntry << std::endl;
    m_sLogFile.flush();

}

const std::string ATCS::getTimeStamp()
{
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    std::strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

    return buf;
}
#endif

