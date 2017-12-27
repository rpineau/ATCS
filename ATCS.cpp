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

    // do we need to set the location Long. Lat. ?
    // do we need to set the timezone ?
    // do we need to set the time ?
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

#ifdef ATCS_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [ATCS::readResponse] *pszBufPtr = 0x%02X\n", timestamp, *pszBufPtr);
        fflush(Logfile);
#endif

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

    unsigned long ulBytesWrite;

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

    convertDecDegToDDMMSS(dDec, szTemp, SERIAL_BUFFER_SIZE);
#ifdef ATCS_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [ATCS::syncTo] Dec : %s\n", timestamp, szTemp);
    fflush(Logfile);
#endif
    // set target dec
    snprintf(szCmd, SERIAL_BUFFER_SIZE, "!CStd%s;", szTemp);
    nErr = ATCSSendCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

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

int ATCS::gotoPark(double dRa, double dDEc)
{
    int nErr = ATCS_OK;
    char szResp[SERIAL_BUFFER_SIZE];

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


int ATCS::Abort()
{
    int nErr = ATCS_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = ATCSSendCommand("!XXxx;", szResp, SERIAL_BUFFER_SIZE);
    return nErr;
}

#pragma mark - Special commands & functions
int ATCS::disablePacketSeqChecking()
{
    int nErr;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = ATCSSendCommand("!QDps;", szResp, SERIAL_BUFFER_SIZE);

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

void ATCS::convertDecDegToDDMMSS(double dDeg, char *szResult, int size)
{
    char cSign;
    int DD, MM, SS;
    float mm, ss;

    // convert dDeg decimal value to sDD:MM:SS

    cSign = dDeg>=0?'+':'-';
    DD = int(dDeg);
    mm = dDeg - DD;
    MM = int(mm*60);
    ss = (mm*60) - MM;
    SS = int(ceil(ss*60));
    snprintf(szResult, size, "%c%02d:%02d:%02d", cSign, DD, MM, SS);
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

