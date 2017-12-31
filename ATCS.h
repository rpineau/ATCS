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
#include "../../licensedinterfaces/mountdriverinterface.h"


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
#define ATCS_NB_ALIGNEMENT_TYPE 4
#define ATCS_ALIGNEMENT_NAME_LENGHT 12




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
    int getAlignementType(char *szType, int nMaxLEn);
    int setAlignementType(char *szType);
    int getNbAlignementType();
    int getAlignementTypeName(int nZeroBasedIndex, char *pszOut, int nOutMaxSize);

    int setTrackingRates(bool bTrackingOn, bool bIgnoreRates, double dTrackRaArcSecPerHr, double dTrackDecArcSecPerHr);
    int getTrackRates(bool &bTrackingOn, double &dTrackRaArcSecPerHr, double &dTrackDecArcSecPerHr);

    int startSlewTo(double dRa, double dDec);
    int isSlewToComplete(bool &bComplete);

    int startOpenSlew(const MountDriverInterface::MoveDir Dir, int nRate);
    int stopOpenLoopMove();

    int gotoPark(double dRa, double dDEc);
    int markParkPosition();
    int unPark();
    int getAtPark(bool &bParked);


    int Abort();

    int getLocalTimeFormat(bool &b24h);
    int getDateFormat(bool &bDdMmYy);
    int getStandardTime(char *szTime, int nMaxLen);
    int getStandardDate(char *szDate, int nMaxLen);
    int syncTime();
    int syncDate();

    int GetTopActiveFault(char *szFault, int nMaxLen);

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
    MountDriverInterface::MoveDir      m_nOpenLoopDir;

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

    int     setCustomTRateOffsetRA(double dRa);
    int     setCustomTRateOffsetDec(double dDec);
    int     getCustomTRateOffsetRA(double &dTrackRaArcSecPerHr);
    int     getCustomTRateOffsetDec(double &dTrackDecArcSecPerHr);

    void    convertDecDegToDDMMSS(double dDeg, char *szResult, int size);
    int     convertDDMMSSToDecDeg(const char *szStrDeg, double &dDecDeg);
    
    void    convertRaToHHMMSSt(double dRa, char *szResult, int size);
    int     convertHHMMSStToRa(const char *szStrRa, double &dRa);

    int     parseFields(const char *pszIn, std::vector<std::string> &svFields, char cSeparator);

    const char m_aszSlewRateNames[ATCS_NB_SLEW_SPEEDS][ATCS_SLEW_NAME_LENGHT] = { "ViewVel 1", "ViewVel 2", "ViewVel 3", "ViewVel 4",  "Slew"};
    const char m_szAlignmentType[ATCS_NB_ALIGNEMENT_TYPE][ATCS_ALIGNEMENT_NAME_LENGHT] = { "Polar", "AltAz", "NearlyPolar", "NearlyAltAz"};

#ifdef ATCS_DEBUG
	// timestamp for logs
    char *timestamp;
	time_t ltime;
	FILE *Logfile;	  // LogFile
#endif
	
};


