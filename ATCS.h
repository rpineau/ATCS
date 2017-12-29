#pragma once
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <memory.h>
#include <string.h>
#include <time.h>
#ifdef SB_MAC_BUILD
#include <unistd.h>
#endif

// C++ includes
#include <string>
#include <vector>
#include <sstream>
#include <iostream>

#include "../../licensedinterfaces/sberrorx.h"
#include "../../licensedinterfaces/theskyxfacadefordriversinterface.h"
#include "../../licensedinterfaces/sleeperinterface.h"
#include "../../licensedinterfaces/serxinterface.h"
#include "../../licensedinterfaces/loggerinterface.h"

#define ATCS_DEBUG 1   // define this to have log files

#ifdef ATCS_DEBUG
#if defined(SB_WIN_BUILD)
#define ATCS_LOGFILENAME "C:\\ATCSLog.txt"
#elif defined(SB_LINUX_BUILD)
#define ATCS_LOGFILENAME "/tmp/ATCSLog.txt"
#elif defined(SB_MAC_BUILD)
#define ATCS_LOGFILENAME "/tmp/ATCSLog.txt"
#endif
#endif


// Next turns string charcter representing a HEX code into a number
#define HEX(c) (((c) < 'A')?((c)-'0'):((c) - 'A') + 10)
enum ATCSErrors {ATCS_OK=0, NOT_CONNECTED, ATCS_CANT_CONNECT, ATCS_BAD_CMD_RESPONSE, COMMAND_FAILED, ATCS_ERROR};

#define SERIAL_BUFFER_SIZE 256
#define MAX_TIMEOUT 5000
#define ATCS_LOG_BUFFER_SIZE 256
#define ERR_PARSE   1

/// ATCL response code
#define ATCL_ENTER  0xB1
#define ATCL_ACK    0x8F
#define ATCL_NACK   0xA5
#define ATCS_NB_SLEW_SPEEDS 5
#define ATCS_SLEW_NAME_LENGHT 12




// Define Class for ATCS
class ATCS
{
public:
	ATCS();
	~ATCS();
	
	int Connect(char *pszPort);
	int Disconnect();
	bool isConnected() const { return m_bIsConnected; }

    void setSerxPointer(SerXInterface *p) { m_pSerx = p; }
    void setLogger(LoggerInterface *pLogger) { m_pLogger = pLogger; };
    void setTSX(TheSkyXFacadeForDriversInterface *pTSX) { m_pTsx = pTSX;};
    void setSleeper(SleeperInterface *pSleeper) { m_pSleeper = pSleeper;};

    int getNbSlewRates();
    int getRateName(int nZeroBasedIndex, char *pszOut, int nOutMaxSize);

    
    int getFirmwareVersion(char *version, int strMaxLen);
    int getModel(char *model, int strMaxLen);

    int getRaAndDec(double &dRa, double &dDec);
    int syncTo(double dRa, double dDec);
    int isSynced(bool &bSyncked);

    int startSlewTo(double dRa, double dDec);
    int isSlewToComplete(bool &bComplete);
    int slewToPark(double dRa, double dDEc);
    int unPark();
    int getAtPark(bool &bParked);


    int Abort();

    int getLocalTimeFormat(bool &b24h);
    int getDateFormat(bool &bDdMmYy);
    int getStandardTime(char *szTime, int nMaxLen);
    int getStandardDate(char *szDate, int nMaxLen);
    int syncTime();
    int syncDate();


private:

    SerXInterface                       *m_pSerx;
    LoggerInterface                     *m_pLogger;
    TheSkyXFacadeForDriversInterface    *m_pTsx;
    SleeperInterface                    *m_pSleeper;

    bool    m_bDebugLog;
    char    m_szLogBuffer[ATCS_LOG_BUFFER_SIZE];

	bool    m_bIsConnected;                               // Connected to the mount?
    char    m_szFirmwareVersion[SERIAL_BUFFER_SIZE];

    char    m_szHardwareModel[SERIAL_BUFFER_SIZE];
    char    m_szTime[SERIAL_BUFFER_SIZE];
    char    m_szDate[SERIAL_BUFFER_SIZE];

	double  m_dGotoRATarget;						  // Current Target RA;
	double  m_dGotoDECTarget;                      // Current Goto Target Dec;
	bool    m_bGotoInProgress;						  // Is GOTO in progress?
	bool    m_bParkInProgress;						  // Is a park in progress?
	
    bool    m_bJNOW;
    bool    m_bAligned;
    bool    m_b24h;
    bool    m_bDdMmYy;
    bool    m_bTimeSetOnce;


    int     ATCSSendCommand(const char *pszCmd, char *pszResult, int nResultMaxLen);
    int     ATCSreadResponse(unsigned char *pszRespBuffer, int bufferLen);
    int     atclEnter();
    int     disablePacketSeqChecking();
    int     checkSiteTimeDateSetOnce(bool &bSet);

    int     setTarget(double dRa, double dDec);
    int     setAsyncUpdateEnabled(bool bEnable);
    int     setEpochOfEntry(const char *szEpoch);
    int     alignFromTargetRA_DecCalcSide();
    int     alignFromTargetRA_DecCalcSideEpochNow();
    int     alignFromLastPosition();
    int     calFromTargetRA_DecEpochNow();
    int     calFromTargetRA_Dec();

    int     slewTargetRA_DecEpochNow();

    void    convertDecDegToDDMMSS(double dDeg, char *szResult, int size);
    int     convertDDMMSSToDecDeg(const char *szStrDeg, double &dDecDeg);
    
    void    convertRaToHHMMSSt(double dRa, char *szResult, int size);
    int     convertHHMMSStToRa(const char *szStrRa, double &dRa);

    int     parseFields(const char *pszIn, std::vector<std::string> &svFields, char cSeparator);

    const char m_aszSlewRateNames[ATCS_NB_SLEW_SPEEDS][ATCS_SLEW_NAME_LENGHT] = { "Slew", "ViewVel 1", "ViewVel 2", "ViewVel 3", "ViewVel 4"};

#ifdef ATCS_DEBUG
	// timestamp for logs
    char *timestamp;
	time_t ltime;
	FILE *Logfile;	  // LogFile
#endif
	
};


