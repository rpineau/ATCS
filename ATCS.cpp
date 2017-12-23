#include "ATCS.h"

// Constructor for ATCS
ATCS::ATCS()
{

	m_bIsConnected = false;
	m_bGotoInProgress = false;
	m_bParkInProgress = false;

    m_bDebugLog = true;
    
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

    // disable any async message from the controller
    setAsyncUpdateEnabled(false);
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

#pragma mark - Special commands.

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
        snprintf(szCmd, SERIAL_BUFFER_SIZE, "!QSauYes;");
    }
    nErr = ATCSSendCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

#pragma mark - Mount Coordinates

int ATCS::getRaAndDec(double &dRa, double &dDec)
{
    int nErr = ATCS_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    // get RA
    nErr = ATCSSendCommand("!CGra;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    // if not yet synced we have no coordinates.
    if(strncmp(szResp,"N/A",SERIAL_BUFFER_SIZE)) {
        dRa = 0.0f;
        dDec = 0.0f;
        return nErr;
    }

    dRa = atof(szResp);

    // get DEC
    nErr = ATCSSendCommand("!CGde;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    dDec = atof(szResp);

    return nErr;
}

int ATCS::syncTo(double dRa, double dDec)
{
    int nErr = ATCS_OK;
    double HA=0;
    char szResp[SERIAL_BUFFER_SIZE];
    char szCmd[SERIAL_BUFFER_SIZE];

#ifdef ATCS_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [ATCS::syncTo] Ra : %f\n", timestamp, dRa);
    fprintf(Logfile, "[%s] [ATCS::syncTo] Dec : %f\n", timestamp, dDec);
    fflush(Logfile);
#endif

    // Determine HA from RA
    HA = m_pTsx->hourAngle(dRa);

    snprintf(szCmd, SERIAL_BUFFER_SIZE, "!CStr%3.2f;", HA);
    nErr = ATCSSendCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);

    snprintf(szCmd, SERIAL_BUFFER_SIZE, "!CStd3.2%f;", dDec);
    nErr = ATCSSendCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);


    // ACrn
    nErr = ATCSSendCommand("!ACrn;", szResp, SERIAL_BUFFER_SIZE);
    return nErr;
}

int ATCS::isSynced(bool bSyncked)
{
    int nErr = ATCS_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    nErr = ATCSSendCommand("!AGas;", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    if(strncmp(szResp,"NotAligned",SERIAL_BUFFER_SIZE))
        bSyncked = false;
    else if(strncmp(szResp,"Preliminary",SERIAL_BUFFER_SIZE))
        bSyncked = false;
    else if(strncmp(szResp,"Complete",SERIAL_BUFFER_SIZE))
        bSyncked = true;
    else
        bSyncked = false;

    return nErr;
}
