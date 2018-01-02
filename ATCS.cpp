#include "ATCS.h"

// Constructor for ATCS
ATCS::ATCS()
{

	m_bIsConnected = false;
	m_bGotoInProgress = false;
	m_bParkInProgress = false;

    m_bDebugLog = true;
    m_bJNOW = false;
    m_bAligned = false;

    m_b24h = false;
    m_bDdMmYy = false;
    m_bTimeSetOnce = false;


#ifdef	ATCS_DEBUG
	Logfile = fopen(ATCS_LOGFILENAME, "w");
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] ATCS New Constructor Called\n", timestamp);
    fflush(Logfile);
#endif

}


ATCS::~ATCS(void)
{
#ifdef ATCS_DEBUG
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] ATCS Destructor Called\n", timestamp );
    fflush(Logfile);
	// Close LogFile
	if (Logfile) fclose(Logfile);
#endif
}

int ATCS::Connect(char *pszPort)
{
    int nErr = ATCS_OK;

#ifdef ATCS_DEBUG
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] ATCS::Connect Called %s\n", timestamp, pszPort);
    fflush(Logfile);
#endif

    // 19200 8N1
    if(m_pSerx->open(pszPort, 19200, SerXInterface::B_NOPARITY, "-DTR_CONTROL 1") == 0)
        m_bIsConnected = true;
    else
        m_bIsConnected = false;

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    atclEnter();
    disablePacketSeqChecking();

    // disable any async message from the controller
    setAsyncUpdateEnabled(false);
    m_bJNOW = true;
    setEpochOfEntry("Now");

    // do we need to set the time ?
    nErr = checkSiteTimeDateSetOnce(m_bTimeSetOnce);
    nErr = getLocalTimeFormat(m_b24h);
    nErr = getDateFormat(m_bDdMmYy);

    // debug
    getStandardTime(m_szTime, SERIAL_BUFFER_SIZE);
    getStandardDate(m_szDate, SERIAL_BUFFER_SIZE);

    if(!m_bTimeSetOnce) {
        nErr = syncTime();
        nErr = syncDate();
    }

    // do we need to set the location Long. Lat. ?
    // 
    // do we need to set the timezone ?
    // read park location
    
    return SB_OK;
}


int ATCS::Disconnect(void)
{
#ifdef ATCS_DEBUG
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] ATCS::Disconnect Called\n", timestamp);
    fflush(Logfile);
#endif
	if (m_bIsConnected) {
        if(m_pSerx){
#ifdef ATCS_DEBUG
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] ATCS::Disconnect closing serial port\n", timestamp);
            fflush(Logfile);
#endif
            m_pSerx->flushTx();
            m_pSerx->purgeTxRx();
            m_pSerx->close();
        }
    }
	m_bIsConnected = false;
	return SB_OK;
}


int ATCS::getNbSlewRates()
{
    return ATCS_NB_SLEW_SPEEDS;
}

// returns "Slew", "ViewVel4", "ViewVel3", "ViewVel2", "ViewVel1"
int ATCS::getRateName(int nZeroBasedIndex, char *pszOut, int nOutMaxSize)
{
    if (nZeroBasedIndex > ATCS_NB_SLEW_SPEEDS)
        return ATCS_ERROR;

    strncpy(pszOut, m_aszSlewRateNames[nZeroBasedIndex], nOutMaxSize);

    return ATCS_OK;
}

#pragma mark - ATCS communication

int ATCS::ATCSSendCommand(const char *pszCmd, char *pszResult, int nResultMaxLen)
{
    int nErr = ATCS_OK;
    unsigned char szResp[SERIAL_BUFFER_SIZE];
    unsigned long ulBytesWrite;

    m_pSerx->purgeTxRx();
    if (m_bDebugLog) {
        snprintf(m_szLogBuffer,ATCS_LOG_BUFFER_SIZE,"[ATCS::ATCSSendCommand] Sending %s\n",pszCmd);
        m_pLogger->out(m_szLogBuffer);
    }

#ifdef ATCS_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [ATCS::domeCommand] Sending %s\n", timestamp, pszCmd);
    fflush(Logfile);
#endif

    nErr = m_pSerx->writeFile((void *)pszCmd, strlen(pszCmd), ulBytesWrite);
    m_pSerx->flushTx();
    if(nErr)
        return nErr;
    // read response
    if (m_bDebugLog) {
        snprintf(m_szLogBuffer,ATCS_LOG_BUFFER_SIZE,"[ATCS::ATCSSendCommand] Getting response.\n");
        m_pLogger->out(m_szLogBuffer);
    }
    nErr = ATCSreadResponse(szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {

#ifdef ATCS_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        if(szResp[0] == ATCL_NACK )
            fprintf(Logfile, "[%s] [ATCS::domeCommand] ERROR reading response , ATCL_NACK received\n", timestamp);
        else
            fprintf(Logfile, "[%s] [ATCS::domeCommand] error %d reading response : %s\n", timestamp, nErr, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
    if(pszResult)
        strncpy(pszResult, (const char *)szResp, nResultMaxLen);

#ifdef ATCS_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    if(szResp[0] == ATCL_ACK )
        fprintf(Logfile, "[%s] [ATCS::domeCommand] got ATCL_ACK (%02X) \n", timestamp, szResp[0]);
    else if(szResp[0] == ATCL_NACK )
        fprintf(Logfile, "[%s] [ATCS::domeCommand] got ATCL_NACK (%02X) \n", timestamp, szResp[0]);
    else
        fprintf(Logfile, "[%s] [ATCS::domeCommand] got response : '%s'\n", timestamp, szResp);
    fflush(Logfile);
#endif

    return nErr;

}


int ATCS::ATCSreadResponse(unsigned char *pszRespBuffer, int nBufferLen)
{
    int nErr = ATCS_OK;
    unsigned long ulBytesRead = 0;
    unsigned long ulTotalBytesRead = 0;
    unsigned char *pszBufPtr;

    memset(pszRespBuffer, 0, (size_t) nBufferLen);
    pszBufPtr = pszRespBuffer;

    do {
        nErr = m_pSerx->readFile(pszBufPtr, 1, ulBytesRead, MAX_TIMEOUT);
        if(nErr) {
            if (m_bDebugLog) {
                snprintf(m_szLogBuffer,ATCS_LOG_BUFFER_SIZE,"[ATCS::ATCSreadResponse] readFile error.\n");
                m_pLogger->out(m_szLogBuffer);
            }
            return nErr;
        }
/*
#ifdef ATCS_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [ATCS::readResponse] *pszBufPtr = 0x%02X\n", timestamp, *pszBufPtr);
        fflush(Logfile);
#endif
*/
        if (ulBytesRead !=1) {// timeout
            if (m_bDebugLog) {
                snprintf(m_szLogBuffer,ATCS_LOG_BUFFER_SIZE,"[ATCS::ATCSreadResponse] readFile Timeout.\n");
                m_pLogger->out(m_szLogBuffer);
            }
            nErr = ATCS_BAD_CMD_RESPONSE;
            break;
        }
        ulTotalBytesRead += ulBytesRead;
        if (m_bDebugLog) {
            snprintf(m_szLogBuffer,ATCS_LOG_BUFFER_SIZE,"[ATCS::ATCSreadResponse] nBytesRead = %lu\n",ulBytesRead);
            m_pLogger->out(m_szLogBuffer);
        }
        // check for  errors or single ACK
        if(*pszBufPtr == ATCL_NACK) {
            nErr = ATCS_BAD_CMD_RESPONSE;
            break;
        }

        if(*pszBufPtr == ATCL_ACK) {
            nErr = ATCS_OK;
            break;
        }


    } while (*pszBufPtr++ != ';' && ulTotalBytesRead < nBufferLen );

    if(ulTotalBytesRead && *(pszBufPtr-1) == ';')
        *(pszBufPtr-1) = 0; //remove the ; to zero terminate the string

    return nErr;
}

int ATCS::atclEnter()
{
    int nErr = ATCS_OK;
    char szCmd[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];

    snprintf(szCmd,SERIAL_BUFFER_SIZE, "%c", ATCL_ENTER);
    nErr = ATCSSendCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}


#pragma mark - dome controller informations

int ATCS::getFirmwareVersion(char *pszVersion, int nStrMaxLen)
{
    int nErr = ATCS_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = ATCSSendCommand("!HGfv;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    strncpy(pszVersion, szResp, nStrMaxLen);
    strncpy(m_szFirmwareVersion, szResp, nStrMaxLen);
    return nErr;
}

int ATCS::getModel(char *pszModel, int nStrMaxLen)
{
    int nErr = ATCS_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = ATCSSendCommand("!HGsm;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    snprintf(pszModel, nStrMaxLen, "%s", szResp);
    snprintf(m_szHardwareModel, nStrMaxLen, "%s", szResp);

    return nErr;
}


#pragma mark - Mount Coordinates

int ATCS::getRaAndDec(double &dRa, double &dDec)
{
    int nErr = ATCS_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    // get RA
    nErr = ATCSSendCommand("!CGra;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
        return nErr;
    }
#ifdef ATCS_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [ATCS::getRaAndDec] szResp = %s\n", timestamp, szResp);
    fflush(Logfile);
#endif

    // if not yet synced we have no coordinates.
    if(strncmp(szResp,"N/A",SERIAL_BUFFER_SIZE) == 0) {
#ifdef ATCS_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [ATCS::getRaAndDec] Not synced yet\n", timestamp);
        fflush(Logfile);
#endif
        dRa = 0.0f;
        dDec = 0.0f;
        return nErr;
    }

    nErr = convertHHMMSStToRa(szResp, dRa);
    if(nErr)
        return nErr;

    // get DEC
    nErr = ATCSSendCommand("!CGde;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    nErr = convertDDMMSSToDecDeg(szResp, dDec);
    return nErr;
}

int ATCS::setTarget(double dRa, double dDec)
{
    int nErr;
    char szCmd[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];
    char szTemp[SERIAL_BUFFER_SIZE];
    char cSign;

#ifdef ATCS_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [ATCS::syncTo] Ra : %f\n", timestamp, dRa);
    fprintf(Logfile, "[%s] [ATCS::syncTo] Dec : %f\n", timestamp, dDec);
    fflush(Logfile);
#endif

    // convert Ra value to HH:MM:SS.T before passing them to the ATCS
    convertRaToHHMMSSt(dRa, szTemp, SERIAL_BUFFER_SIZE);

#ifdef ATCS_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [ATCS::syncTo] Ra : %s\n", timestamp, szTemp);
    fflush(Logfile);
#endif
    // set target Ra
    snprintf(szCmd, SERIAL_BUFFER_SIZE, "!CStr%s;", szTemp);
    nErr = ATCSSendCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    convertDecDegToDDMMSS(dDec, szTemp, cSign, SERIAL_BUFFER_SIZE);
#ifdef ATCS_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [ATCS::syncTo] Dec : %c%s\n", timestamp, cSign, szTemp);
    fflush(Logfile);
#endif
    // set target dec
    snprintf(szCmd, SERIAL_BUFFER_SIZE, "!CStd%c%s;", cSign,szTemp);
    nErr = ATCSSendCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

#pragma mark - Sync and Cal
int ATCS::syncTo(double dRa, double dDec)
{
    int nErr = ATCS_OK;
    bool bAligned;

    nErr = isSynced(bAligned);
    if(nErr)
        return nErr;

    // set sync target coordinate
    nErr = setTarget(dRa, dDec);
    if(nErr)
        return nErr;

    // Sync
    if(!bAligned){
        nErr = alignFromTargetRA_DecCalcSideEpochNow();
    }
    else {
        nErr = calFromTargetRA_DecEpochNow();
    }
    return nErr;
}

int ATCS::calFromTargetRA_DecEpochNow()
{
    int nErr = ATCS_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    nErr = ATCSSendCommand("!ACrn;", szResp, SERIAL_BUFFER_SIZE);
    return nErr;
}

int ATCS::calFromTargetRA_Dec()
{
    int nErr = ATCS_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    nErr = ATCSSendCommand("!ACrd;", szResp, SERIAL_BUFFER_SIZE);
    return nErr;
}


int ATCS::isSynced(bool &bSyncked)
{
    int nErr = ATCS_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(m_bAligned) {
        bSyncked = m_bAligned;
        return nErr;
    }

    nErr = ATCSSendCommand("!AGas;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    if(strncmp(szResp,"NotAligned",SERIAL_BUFFER_SIZE) == 0)
        bSyncked = false;
    else if(strncmp(szResp,"Preliminary",SERIAL_BUFFER_SIZE) == 0)
        bSyncked = false;
    else if(strncmp(szResp,"Complete",SERIAL_BUFFER_SIZE) == 0)
        bSyncked = true;
    else
        bSyncked = false;

    m_bAligned = bSyncked;
    return nErr;
}

int ATCS::getAlignementType(char *szType, int nMaxLEn)
{
    int nErr = ATCS_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = ATCSSendCommand("!NGat;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    strncpy(szType, szResp, nMaxLEn);
    return nErr;
}

int ATCS::setAlignementType(char *szType)
{
    int nErr = ATCS_OK;
    char szCmd[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];

    snprintf(szCmd, SERIAL_BUFFER_SIZE, "!NSat%s;", szType);
    nErr = ATCSSendCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int ATCS::getNbAlignementType()
{
    return ATCS_NB_ALIGNEMENT_TYPE;
}

int ATCS::getAlignementTypeName(int nZeroBasedIndex, char *pszOut, int nOutMaxSize)
{
    if (nZeroBasedIndex > ATCS_NB_ALIGNEMENT_TYPE)
        return ATCS_ERROR;

    strncpy(pszOut, m_szAlignmentType[nZeroBasedIndex], nOutMaxSize);

    return ATCS_OK;

}

#pragma mark - tracking rates
int ATCS::setTrackingRates(bool bTrackingOn, bool bIgnoreRates, double dTrackRaArcSecPerHr, double dTrackDecArcSecPerHr)
{
    int nErr = ATCS_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!bTrackingOn) { // stop tracking
#ifdef ATCS_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [ATCS::setTrackingRates] setting to Drift\n", timestamp);
        fflush(Logfile);
#endif
        nErr = ATCSSendCommand("!RStrDrift;", szResp, SERIAL_BUFFER_SIZE);
    }
    else if(bTrackingOn && bIgnoreRates) { // sidereal
#ifdef ATCS_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [ATCS::setTrackingRates] setting to Sidereal\n", timestamp);
        fflush(Logfile);
#endif
        nErr = ATCSSendCommand("!RStrSidereal;", szResp, SERIAL_BUFFER_SIZE);
    }
    else { // custom rate
#ifdef ATCS_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [ATCS::setTrackingRates] setting to Custom\n", timestamp);
        fprintf(Logfile, "[%s] [ATCS::setTrackingRates] dTrackRaArcSecPerHr = %f\n", timestamp, dTrackRaArcSecPerHr);
        fprintf(Logfile, "[%s] [ATCS::setTrackingRates] dTrackDecArcSecPerHr = %f\n", timestamp, dTrackDecArcSecPerHr);
        fflush(Logfile);
#endif
        nErr = setCustomTRateOffsetRA(dTrackRaArcSecPerHr);
        nErr |= setCustomTRateOffsetDec(dTrackDecArcSecPerHr);
        if(nErr)
            return nErr; // if we cant set the rate no need to switch to custom.
        nErr = ATCSSendCommand("!RStrCustom;", szResp, SERIAL_BUFFER_SIZE);
    }
    return nErr;
}

int ATCS::getTrackRates(bool &bTrackingOn, double &dTrackRaArcSecPerHr, double &dTrackDecArcSecPerHr)
{
    int nErr = ATCS_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = ATCSSendCommand("!RGtr;", szResp, SERIAL_BUFFER_SIZE);
    bTrackingOn = true;
    if(strncmp(szResp, "Drift", SERIAL_BUFFER_SIZE)==0) {
        bTrackingOn = false;
    }
    nErr = getCustomTRateOffsetRA(dTrackRaArcSecPerHr);
    nErr |= getCustomTRateOffsetDec(dTrackDecArcSecPerHr);

    return nErr;
}

int ATCS::setCustomTRateOffsetRA(double dRa)
{
    int nErr;
    char szCmd[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];

    snprintf(szCmd, SERIAL_BUFFER_SIZE, "!RSor%.2f;", dRa);
    nErr = ATCSSendCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int ATCS::setCustomTRateOffsetDec(double dDec)
{
    int nErr;
    char szCmd[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];

    snprintf(szCmd, SERIAL_BUFFER_SIZE, "!RSod%.2f;", dDec);
    nErr = ATCSSendCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int ATCS::getCustomTRateOffsetRA(double &dTrackRaArcSecPerHr)
{
    int nErr;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = ATCSSendCommand("!RGor;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    dTrackRaArcSecPerHr = atof(szResp);

    return nErr;
}

int ATCS::getCustomTRateOffsetDec(double &dTrackDecArcSecPerHr)
{
    int nErr;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = ATCSSendCommand("!RGod;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    dTrackDecArcSecPerHr = atof(szResp);

    return nErr;
}


#pragma mark - slew

int ATCS::startSlewTo(double dRa, double dDec)
{
    int nErr = ATCS_OK;
    bool bAligned;

    nErr = isSynced(bAligned);
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
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = ATCSSendCommand("!GTrn;", szResp, SERIAL_BUFFER_SIZE);

    return nErr;

}

int ATCS::startOpenSlew(const MountDriverInterface::MoveDir Dir, int nRate)
{
    int nErr = ATCS_OK;
    char szCmd[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];

    m_nOpenLoopDir = Dir;

#ifdef ATCS_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [ATCS::startOpenSlew] setting to Dir %d\n", timestamp, Dir);
    fprintf(Logfile, "[%s] [ATCS::startOpenSlew] Setting rate to %d\n", timestamp, nRate);
    fflush(Logfile);
#endif

    // select rate
    if(nRate == 4) { // "Slew"
        nErr = ATCSSendCommand("!KSsl;", szResp, SERIAL_BUFFER_SIZE);
    }
    else {
        // clear slew
        nErr = ATCSSendCommand("!KCsl;", szResp, SERIAL_BUFFER_SIZE);
        // select rate
        // KScv + 1,2 3 or 4 for ViewVel 1,2,3,4, 'ViewVel 1' is index 0 so nRate+1
        snprintf(szCmd, SERIAL_BUFFER_SIZE, "!KScv%d;", nRate+1);
    }
    // figure out direction
    switch(Dir){
        case MountDriverInterface::MD_NORTH:
            nErr = ATCSSendCommand("!KSpu100;", szResp, SERIAL_BUFFER_SIZE);
            break;
        case MountDriverInterface::MD_SOUTH:
            nErr = ATCSSendCommand("!KSpd100;", szResp, SERIAL_BUFFER_SIZE);
            break;
        case MountDriverInterface::MD_EAST:
            nErr = ATCSSendCommand("!KSpl100;", szResp, SERIAL_BUFFER_SIZE);
            break;
        case MountDriverInterface::MD_WEST:
            nErr = ATCSSendCommand("!KSsr100;", szResp, SERIAL_BUFFER_SIZE);
            break;
    }

    return nErr;
}

int ATCS::stopOpenLoopMove()
{
    int nErr = ATCS_OK;
    char szResp[SERIAL_BUFFER_SIZE];

#ifdef ATCS_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [ATCS::stopOpenLoopMove] Dir was %d\n", timestamp, m_nOpenLoopDir);
    fflush(Logfile);
#endif

    switch(m_nOpenLoopDir){
        case MountDriverInterface::MD_NORTH:
        case MountDriverInterface::MD_SOUTH:
            nErr = ATCSSendCommand("!XXud;", szResp, SERIAL_BUFFER_SIZE);
            break;
        case MountDriverInterface::MD_EAST:
        case MountDriverInterface::MD_WEST:
            nErr = ATCSSendCommand("!XXlr;", szResp, SERIAL_BUFFER_SIZE);
            break;
    }

    return nErr;
}

int ATCS::isSlewToComplete(bool &bComplete)
{
    int nErr = ATCS_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    int nPrecentRemaining;

    bComplete = false;

    nErr = ATCSSendCommand("!GGgr;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
#ifdef ATCS_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [ATCS::isSlewToComplete] szResp : %s\n", timestamp, szResp);
    fflush(Logfile);
#endif

    // remove the %
    szResp[strlen(szResp) -1 ] = 0;
    
#ifdef ATCS_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [ATCS::isSlewToComplete] szResp : %s\n", timestamp, szResp);
    fflush(Logfile);
#endif

    nPrecentRemaining = atoi(szResp);
    if(nPrecentRemaining == 0)
        bComplete = true;

    return nErr;
}

int ATCS::gotoPark(double dRa, double dDEc)
{
    int nErr = ATCS_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    // set park position ?
    // or goto ?
    // goto park
    nErr = ATCSSendCommand("!GTop;", szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int ATCS::markParkPosition()
{
    int nErr = ATCS_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = ATCSSendCommand("!AMpp;", szResp, SERIAL_BUFFER_SIZE);

    return nErr;

}

int ATCS::getAtPark(bool &bParked)
{
    int nErr = ATCS_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    bParked = false;
    nErr = ATCSSendCommand("!AGak;", szResp, SERIAL_BUFFER_SIZE);
    if(strncmp(szResp,"Yes",SERIAL_BUFFER_SIZE) == 0) {
        bParked = true;
    }
    return nErr;
}

int ATCS::unPark()
{
    int nErr = ATCS_OK;
    nErr = alignFromLastPosition();
    return nErr;
}


int ATCS::getRefractionCorrEnabled(bool &bEnabled)
{
    int nErr = ATCS_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    bEnabled = false;
    nErr = ATCSSendCommand("!PGre;", szResp, SERIAL_BUFFER_SIZE);
    if(strncmp(szResp,"Yes",SERIAL_BUFFER_SIZE) == 0) {
        bEnabled = true;
    }
    return nErr;
}

int ATCS::setRefractionCorrEnabled(bool bEnable)
{
    int nErr = ATCS_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;
    if(bEnable) {
        snprintf(szCmd, SERIAL_BUFFER_SIZE, "!PSreYes;");
    }
    else  {
        snprintf(szCmd, SERIAL_BUFFER_SIZE, "!PSreNo;");
    }
    nErr = ATCSSendCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}


int ATCS::Abort()
{
    int nErr = ATCS_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = ATCSSendCommand("!XXxx;", szResp, SERIAL_BUFFER_SIZE);
    return nErr;
}

#pragma mark - time and site methods
int ATCS::syncTime()
{
    int nErr = ATCS_OK;
    int yy, mm, dd, h, min, dst;
    double sec;
    int n12h_time;

    char szCmd[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];
    char szTemp[SERIAL_BUFFER_SIZE];
    char szAmPm[4];

    m_pTsx->localDateTime(yy, mm, dd, h, min, sec, dst);
    if(!m_b24h) {
        n12h_time = h%12;
        if(h<12)
            strncpy(szAmPm,"AM",4);
        else
            strncpy(szAmPm,"PM",4);
        snprintf(szTemp,SERIAL_BUFFER_SIZE, "%02d:%02d:%02.2f%s", n12h_time, min, sec, szAmPm);
    }
    else {
        snprintf(szTemp,SERIAL_BUFFER_SIZE, "%02d:%02d:%02.2f", h, min, sec);
    }

    snprintf(szCmd, SERIAL_BUFFER_SIZE, "!TSst%s;", szTemp);
    nErr = ATCSSendCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);
    getStandardTime(m_szTime, SERIAL_BUFFER_SIZE);
    m_bAligned = false; // changing time void alignement

    return nErr;
}


int ATCS::syncDate()
{
    int nErr = ATCS_OK;
    int yy, mm, dd, h, min, dst;
    double sec;

    char szCmd[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];
    char szTemp[SERIAL_BUFFER_SIZE];

    m_pTsx->localDateTime(yy, mm, dd, h, min, sec, dst);
    // yy is actually yyyy, need conversion to yy, 2017 -> 17
    yy = yy - (int(yy / 1000) * 1000);

    if(!m_bDdMmYy) {
        snprintf(szTemp,SERIAL_BUFFER_SIZE, "%02d/%02d/%02d", mm, dd, yy);
    }
    else {
        snprintf(szTemp,SERIAL_BUFFER_SIZE, "%02d/%02d/%02d", dd, mm, yy);
    }

    snprintf(szCmd, SERIAL_BUFFER_SIZE, "!TSsd%s;", szTemp);
    nErr = ATCSSendCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    getStandardDate(m_szDate, SERIAL_BUFFER_SIZE);
    m_bAligned = false; // changing date void alignement
    return nErr;
}

int ATCS::getSiteName(char *szSiteName, int nMaxSize)
{
    int nErr = ATCS_OK;
    int nSiteNb;

    nErr = getUsingSiteNumber(nSiteNb);
    if(nErr)
        return nErr;

    nErr = getUsingSiteName(nSiteNb, szSiteName, nMaxSize);

    return nErr;
}

int ATCS::setSiteData(double dLongitude, double dLatitute, double dTimeZone)
{
    int nErr = ATCS_OK;
    char szLong[SERIAL_BUFFER_SIZE];
    char szLat[SERIAL_BUFFER_SIZE];
    char szTimeZone[SERIAL_BUFFER_SIZE];
    char szHH[3], szMM[3];
    char cSignLong;
    char cSignLat;
#ifdef ATCS_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [ATCS::setSiteData] dLongitude : %f\n", timestamp, dLongitude);
    fprintf(Logfile, "[%s] [ATCS::setSiteData] dLatitute : %f\n", timestamp, dLatitute);
    fprintf(Logfile, "[%s] [ATCS::setSiteData] szTimeZone : %f\n", timestamp, dTimeZone);
    fflush(Logfile);
#endif

    convertDecDegToDDMMSS(dLongitude, szLong, cSignLong, SERIAL_BUFFER_SIZE);
    convertDecDegToDDMMSS(dLatitute, szLat, cSignLat, SERIAL_BUFFER_SIZE);
    snprintf(szHH,3,"%02d", int(fabs(dTimeZone)));
    snprintf(szMM,3,"%02d", int((fabs(dTimeZone) - int(fabs(dTimeZone)))) * 100);
    if(dTimeZone<0) {
        snprintf(szTimeZone, SERIAL_BUFFER_SIZE, "%s:%sW", szHH, szMM);
    }
    else if (dTimeZone>0) {
        snprintf(szTimeZone, SERIAL_BUFFER_SIZE, "%s:%sE", szHH, szMM);
    }
    else
        snprintf(szTimeZone, SERIAL_BUFFER_SIZE, "00:00");

    // Set the W/E
    if(dLongitude<0) {
        snprintf(szLong, SERIAL_BUFFER_SIZE, "%sW", szLong);
    }
    else {
        snprintf(szLong, SERIAL_BUFFER_SIZE, "%sE", szLong);
    }
    // convert signed latitude to N/S
    if(dLatitute>=0) {
        snprintf(szLat, SERIAL_BUFFER_SIZE, "%sN", szLat);
    }
    else {
        snprintf(szLat, SERIAL_BUFFER_SIZE, "%sS", szLat);
    }

#ifdef ATCS_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [ATCS::setSiteData] szLong : %s\n", timestamp, szLong);
    fprintf(Logfile, "[%s] [ATCS::setSiteData] szLat : %s\n", timestamp, szLat);
    fprintf(Logfile, "[%s] [ATCS::setSiteData] szTimeZone : %s\n", timestamp, szTimeZone);
    fflush(Logfile);
#endif

    setSiteLongitude(m_nSiteNumber, szLong);
    setSiteLatitude(m_nSiteNumber, szLat);
    setSiteTimezone(m_nSiteNumber, szTimeZone);

    return nErr;
}

#pragma mark - Special commands & functions

int ATCS::GetTopActiveFault(char *szFault, int nMaxLen)
{
    int nErr = ATCS_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = ATCSSendCommand("!HGtf;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    strncpy(szFault, szResp, nMaxLen);

    return nErr;
}

int ATCS::disablePacketSeqChecking()
{
    int nErr;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = ATCSSendCommand("!QDps;", szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

#pragma mark - site data
int ATCS::checkSiteTimeDateSetOnce(bool &bSet)
{
    int nErr = ATCS_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    bSet = false;
    nErr = ATCSSendCommand("!ACst;", szResp, SERIAL_BUFFER_SIZE);
    if(strncmp(szResp,"Yes",SERIAL_BUFFER_SIZE) == 0) {
        bSet = true;
    }
    return nErr;
}

int ATCS::getUsingSiteNumber(int &nSiteNb)
{
    int nErr = ATCS_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = ATCSSendCommand("!SGuu;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    nSiteNb = atoi(szResp);
    m_nSiteNumber = nSiteNb;
    return nErr;

}

int ATCS::getUsingSiteName(int nSiteNb, char *szSiteName, int nMaxSize)
{
    int nErr = ATCS_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

    snprintf(szCmd, SERIAL_BUFFER_SIZE, "!SGun%d;", nSiteNb);
    nErr = ATCSSendCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    strncpy(szSiteName, szResp, nMaxSize);
    return nErr;
}

int ATCS::setSiteLongitude(int nSiteNb, const char *szLongitude)
{
    int nErr = ATCS_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

    snprintf(szCmd, SERIAL_BUFFER_SIZE, "!SSo%d%s;", nSiteNb, szLongitude);
    nErr = ATCSSendCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int ATCS::setSiteLatitude(int nSiteNb, const char *szLatitude)
{
    int nErr = ATCS_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

    snprintf(szCmd, SERIAL_BUFFER_SIZE, "!SSa%d%s;", nSiteNb, szLatitude);
    nErr = ATCSSendCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int ATCS::setSiteTimezone(int nSiteNb, const char *szTimezone)
{
    int nErr = ATCS_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

    snprintf(szCmd, SERIAL_BUFFER_SIZE, "!SSz%d%s;", nSiteNb, szTimezone);
    nErr = ATCSSendCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

#pragma mark  - Time and Date

int ATCS::getLocalTimeFormat(bool &b24h)
{
    int nErr = ATCS_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = ATCSSendCommand("!TGlf;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    b24h = false;
    if(strncmp(szResp,"24hr",SERIAL_BUFFER_SIZE) == 0) {
        b24h = true;
    }

    return nErr;
}

int ATCS::getDateFormat(bool &bDdMmYy )
{
    int nErr = ATCS_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = ATCSSendCommand("!TGdf;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    bDdMmYy = false;
    if(strncmp(szResp,"dd/mm/yy",SERIAL_BUFFER_SIZE) == 0) {
        bDdMmYy = true;
    }
    return nErr;
}

int ATCS::getStandardTime(char *szTime, int nMaxLen)
{
    int nErr = ATCS_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = ATCSSendCommand("!TGst;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    strncpy(szTime, szResp, nMaxLen);
    return nErr;
}

int ATCS::getStandardDate(char *szDate, int nMaxLen)
{
    int nErr = ATCS_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = ATCSSendCommand("!TGsd;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    strncpy(szDate, szResp, nMaxLen);
    return nErr;
}


int ATCS::setAsyncUpdateEnabled(bool bEnable)
{
    int nErr = ATCS_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;
    if(bEnable) {
        snprintf(szCmd, SERIAL_BUFFER_SIZE, "!QSauYes;");
    }
    else  {
        snprintf(szCmd, SERIAL_BUFFER_SIZE, "!QSauNo;");
    }
    nErr = ATCSSendCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int ATCS::setEpochOfEntry(const char *szEpoch)
{
    int nErr;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

    snprintf(szCmd, SERIAL_BUFFER_SIZE, "!PSep%s;", szEpoch);
    nErr = ATCSSendCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int ATCS::alignFromTargetRA_DecCalcSide()
{
    int nErr;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = ATCSSendCommand("!AFcs;", szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int ATCS::alignFromTargetRA_DecCalcSideEpochNow()
{
    int nErr;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = ATCSSendCommand("!AFcn;", szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int ATCS::alignFromLastPosition()
{
    int nErr;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = ATCSSendCommand("!AFlp;", szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

void ATCS::convertDecDegToDDMMSS(double dDeg, char *szResult, char &cSign, int size)
{
    int DD, MM, SS;
    float mm, ss;

    // convert dDeg decimal value to sDD:MM:SS

    cSign = dDeg>=0?'+':'-';
    dDeg = fabs(dDeg);
    DD = int(dDeg);
    mm = dDeg - DD;
    MM = int(mm*60);
    ss = (mm*60) - MM;
    SS = int(ceil(ss*60));
    snprintf(szResult, size, "%02d:%02d:%02d", DD, MM, SS);
}

int ATCS::convertDDMMSSToDecDeg(const char *szStrDeg, double &dDecDeg)
{
    int nErr = ATCS_OK;
    std::vector<std::string> vFieldsData;

    dDecDeg = 0;

    nErr = parseFields(szStrDeg, vFieldsData, ':');
    if(nErr)
        return nErr;

    if(vFieldsData.size() >= 3) {
        dDecDeg = atof(vFieldsData[0].c_str()) + atof(vFieldsData[1].c_str())/60 + atof(vFieldsData[1].c_str())/3600;
    }
    else
        nErr = ERR_PARSE;

    return nErr;
}

void ATCS::convertRaToHHMMSSt(double dRa, char *szResult, int size)
{
    int HH, MM;
    float hh, mm, SSt;

    // convert Ra value to HH:MM:SS.T before passing them to the ATCS
    HH = int(dRa);
    hh = dRa - HH;
    MM = int(hh*60);
    mm = (hh*60) - MM;
    SSt = mm * 60;
    snprintf(szResult,SERIAL_BUFFER_SIZE, "%02d:%02d:%02.1f", HH, MM, SSt);
}

int ATCS::convertHHMMSStToRa(const char *szStrRa, double &dRa)
{
    int nErr = ATCS_OK;
    std::vector<std::string> vFieldsData;

    dRa = 0;

    nErr = parseFields(szStrRa, vFieldsData, ':');
    if(nErr)
        return nErr;

    if(vFieldsData.size() >= 3) {
        dRa = atof(vFieldsData[0].c_str()) + atof(vFieldsData[1].c_str())/60 + atof(vFieldsData[1].c_str())/3600;
    }
    else
        nErr = ERR_PARSE;

    return nErr;
}


int ATCS::parseFields(const char *pszIn, std::vector<std::string> &svFields, char cSeparator)
{
    int nErr = ATCS_OK;
    std::string sSegment;
    std::stringstream ssTmp(pszIn);

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

