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
#include <cstring>
#include <vector>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <chrono>
#include <thread>
#include <ctime>
#include <cmath>
#include <iomanip>
#include <algorithm>

#include "../../licensedinterfaces/sberrorx.h"
#include "../../licensedinterfaces/theskyxfacadefordriversinterface.h"
#include "../../licensedinterfaces/sleeperinterface.h"
#include "../../licensedinterfaces/serxinterface.h"
#include "../../licensedinterfaces/loggerinterface.h"
#include "../../licensedinterfaces/mountdriverinterface.h"
#include "../../licensedinterfaces/mount/asymmetricalequatorialinterface.h"

#include "StopWatch.h"

// #define PLUGIN_DEBUG 2   // define this to have log files, 1 = bad stuff only, 2 and up.. full debug
#define PLUGIN_VERSION 1.6

enum ATCSErrors {PLUGIN_OK=0, NOT_CONNECTED, ATCS_CANT_CONNECT, ATCS_BAD_CMD_RESPONSE, COMMAND_FAILED, COMMAND_TIMEOUT, ATCS_ERROR};

#define SERIAL_BUFFER_SIZE 1024
#define MAX_TIMEOUT 1000
#define ERR_PARSE   1

#define MAX_READ_WAIT_TIMEOUT 25
#define NB_RX_WAIT 10

/// ATCL response code
#define ATCL_ENTER  0xB1
#define ATCL_ACK    0x8F
#define ATCL_NACK   0xA5

#define ATCL_STATUS         0x9A
#define ATCL_WARNING        0x9B
#define ATCL_ALERT          0x9C
#define ATCL_INTERNAL_ERROR 0x9D
#define ATCL_SYNTAX_ERROR   0x9E
#define ATCL_IDC_ASYNCH     0x9F

#define ATCS_NB_SLEW_SPEEDS 5
#define ATCS_SLEW_NAME_LENGHT 12
#define ATCS_NB_ALIGNEMENT_TYPE 4
#define ATCS_ALIGNEMENT_NAME_LENGHT 12


// Define Class for Astrometric Instruments ATCS controller.
class ATCS
{
public:
	ATCS();
	~ATCS();
	
	int Connect(char *pszPort);
	int Disconnect();
	bool isConnected() const { return m_bIsConnected; }

    void setSerxPointer(SerXInterface *p) { m_pSerx = p; }
    void setTSX(TheSkyXFacadeForDriversInterface *pTSX) { m_pTsx = pTSX;};
    void setSleeper(SleeperInterface *pSleeper) { m_pSleeper = pSleeper;};

    int getNbSlewRates();
    int getRateName(int nZeroBasedIndex, std::string &sOut);
    
    int getFirmwareVersion(std::string &sFirmware);
    int getModel(std::string &sModel);

    void    setMountMode(MountTypeInterface::Type mountType);
    MountTypeInterface::Type mountType();

    int getRaAndDec(double &dRa, double &dDec);
    int syncTo(double dRa, double dDec);
    int isAligned(bool &bAligned);
    int getAlignementType(std::string &sType);
    int setAlignementType(std::string sType);

    int setMeridianAvoidMethod(std::string sType);
    int getMeridianAvoidMethod(std::string &sType);
    
    int setTrackingRates(bool bTrackingOn, bool bIgnoreRates, double dTrackRaArcSecPerHr, double dTrackDecArcSecPerHr);
    int getTrackRates(bool &bTrackingOn, double &dTrackRaArcSecPerHr, double &dTrackDecArcSecPerHr);

    int startSlewTo(double dRa, double dDec);
    int isSlewToComplete(bool &bComplete);

    int startOpenSlew(const MountDriverInterface::MoveDir Dir, unsigned int nRate);
    int stopOpenLoopMove();

    int gotoPark(double dRa, double dDEc);
    int markParkPosition();
    int getAtPark(bool &bParked);
    int unPark();

    int getRefractionCorrEnabled(bool &bEnabled);
    int setRefractionCorrEnabled(bool bEnable);

    int getLimits(double &dHoursEast, double &dHoursWest);

    int Abort();

    int getLocalTimeFormat(bool &b24h);
    int getDateFormat(bool &bDdMmYy);
    int getStandardTime(std::string &sTime);
    int getStandardDate(std::string &sDate);
    int syncTime();
    int syncDate();
    int getSiteName(std::string &sSiteName);
    int setSiteData(double dLongitude, double dLatitute, double dTimeZone);
    int getSiteData(std::string &sLongitude, std::string &sLatitude, std::string &sTimeZone); // assume all buffers have the same size
    int getTopActiveFault(std::string &sFault);

#ifdef PLUGIN_DEBUG
    void log(std::string sLogEntry);
#endif

private:

    SerXInterface                       *m_pSerx;
    TheSkyXFacadeForDriversInterface    *m_pTsx;
    SleeperInterface                    *m_pSleeper;

    bool    m_bDebugLog;
	bool    m_bIsConnected;                               // Connected to the mount?
    std::string m_sFirmwareVersion;

    MountTypeInterface::Type    m_mountType;
    
    std::string     m_sHardwareModel;
    std::string     m_sTime;
    std::string     m_sDate;
    int     m_nSiteNumber;

	double  m_dGotoRATarget;						  // Current Target RA;
	double  m_dGotoDECTarget;                      // Current Goto Target Dec;
	
    bool    m_bJNOW;
    bool    m_b24h;
    bool    m_bDdMmYy;
    bool    m_bTimeSetOnce;
    MountDriverInterface::MoveDir      m_nOpenLoopDir;

    // limits don't change mid-course so we cache them
    bool    m_bLimitCached;
    double  m_dHoursEast;
    double  m_dHoursWest;
    
    int     ATCSSendCommand(const std::string sCmd, std::string &sResp, int nTimeout = MAX_TIMEOUT);
    int     ATCSreadResponse(std::string &sResult, int nTimeout = MAX_TIMEOUT);

    int     atclEnter();
    int     disablePacketSeqChecking();
    int     disableStaticStatusChangeNotification();
    int     checkSiteTimeDateSetOnce(bool &bSet);

    int     getUsingSiteNumber(int &nSiteNb);
    int     getUsingSiteName(int nSiteNb, std::string &sSiteName);
    int     setSiteLongitude(int nSiteNb, const std::string sLongitude);
    int     setSiteLatitude(int nSiteNb, const std::string sLatitude);
    int     setSiteTimezone(int nSiteNb, const std::string sTimezone);

    int     getSiteLongitude(int nSiteNb, std::string &sLongitude);
    int     getSiteLatitude(int nSiteNb, std::string &sLatitude);
    int     getSiteTZ(int nSiteNb, std::string &sTimeZone);

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

    int     getSoftLimitEastAngle(double &dAngle);
    int     getSoftLimitWestAngle(double &dAngle);

    void    convertDecDegToDDMMSS(double dDeg, std::string  &sResult, char &cSign);

    int     convertDDMMSSToDecDeg(const std::string StrDeg, double &dDecDeg);

    void    convertRaToHHMMSSt(double dRa, std::string &sResult);

    int     convertHHMMSStToRa(const std::string StrRa, double &dRa);
    int     parseFields(const std::string szIn, std::vector<std::string> &svFields, char cSeparator);

    std::vector<std::string>    m_svSlewRateNames = { "ViewVel 1", "ViewVel 2", "ViewVel 3", "ViewVel 4",  "Slew"};
    CStopWatch      timer;


    std::string&    trim(std::string &str, const std::string &filter );
    std::string&    ltrim(std::string &str, const std::string &filter);
    std::string&    rtrim(std::string &str, const std::string &filter);


#ifdef PLUGIN_DEBUG
    const std::string getTimeStamp();
    std::ofstream m_sLogFile;
    std::string m_sLogfilePath;
#endif
	
};


