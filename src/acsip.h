#pragma once

#include <Arduino.h>


// #define DEBUG_PORT          Serial

#ifdef DEBUG_PORT
#define DEBUG(x)            DEBUG_PORT.print(x);
#define DEBUGLN(x)          DEBUG_PORT.println(x);
#else
#define DEBUG(x)
#define DEBUGLN(x)
#endif


// #define DEBUG_SERIAL_HEX


#define DEFAULT_SERIAL_TIMEOUT          10000

#ifndef INPUT
#define INPUT             0x01
#endif

#ifndef OUTPUT
#define OUTPUT            0x02
#endif

#define ACSIP_CHECK_ERROR(ret)                                          \
                do{                                                     \
                    if(ret != S7XG_OK){                                 \
                        Serial.printf("[%lu]:%s %d line failed,error code:%s\n",millis(),__FILE__, __LINE__,Acsip::errrToString(ret).c_str());         \
                        while (1);                                      \
                    }                                                   \
                }while(0)


#define STRNCMP_FULLSTRING(str)   (strncmp(ptr, str, strlen(str)) == 0)


enum S7XG_Error {
    S7XG_OK,
    S7XG_FAILED,
    S7XG_TIMEROUT,
    S7XG_JOINED,
    S7XG_UNJOINED,
    S7XG_INVALD,
    S7XG_ALREADY_JOINED,
    S7XG_BUSY,
    S7XG_INVALD_LEN,
    S7XG_GPS_NOT_INIT,
    S7XG_GPS_NOT_POSITIONING,
    S7XG_GPS_SUCCESS,
    S7XG_GPS_ERROR,
    S7XG_COMMAND_ERROR,
    S7XG_UNKONW,
};

enum GPIOGroup {
    S7XG_GPIO_GROUP_A,
    S7XG_GPIO_GROUP_B,
    S7XG_GPIO_GROUP_C,
    S7XG_GPIO_GROUP_D,
    S7XG_GPIO_GROUP_E,
    S7XG_GPIO_GROUP_F,
    S7XG_GPIO_GROUP_H,
};

enum MACAutoJoinMode {
    S7XG_JOIN_OTAA,
    S7XG_JOIN_ABP,
    S7XG_JOIN_OFF
};


enum GPSSentence {
    S7XG_CGA,   /*Global Positioning System Fix Data*/
    S7XG_GLL,   /*Geographic Position – Latitude/Longitude*/
    S7XG_GNS,   /*GNSS Fix Data*/
    S7XG_GSA,   /*GNSS DOP and Active Satellites*/
    S7XG_GSV,   /*GNSS Satellites in View*/
    S7XG_RMC,   /*Recommended Minimum Specific GNSS Data*/
    S7XG_VTG,   /*Course Over Ground & Ground Speed*/
    S7XG_ZDA    /*Time & Date*/
} ;

enum GPSUplinkFormat {
    S7XG_GPS_FORMAT_RAW,
    S7XG_GPS_FORMAT_IPSO,
    S7XG_GPS_FORMAT_KIWI,
    S7XG_GPS_FORMAT_UTC_POS,
    S7XG_GPS_FORMAT_MAX
} ;

enum GPSMode {
    S7XG_GPS_MODE_AUTO,
    S7XG_GPS_MODE_MANUAL,
    S7XG_GPS_MODE_IDLE,
    S7XG_GPS_MODE_MAX
} ;

enum GPSSystem {
    S7XG_GPS_SYS_GPS,
    S7XG_GPS_SYS_HYBRID,
    S7XG_GPS_SYS_MAX
};

enum GPSDataType {
    S7XG_GPS_DATA_RAW,
    S7XG_GPS_DATA_DD,
    S7XG_GPS_DATA_DMS,
    S7XG_GPS_DATA_MAX
} ;

enum GPSSatelliteSys {
    S7XG_SATELLITE_GPS,
    S7XG_SATELLITE_GPS_GLONASS,
    S7XG_SATELLITE_MAX
} ;

enum GPSStartMode {
    S7XG_GPS_START_HOT,
    S7XG_GPS_START_WARM,
    S7XG_GPS_START_COLD,
    S7XG_GPS_START_MAX
} ;

enum TxMode {
    S7XG_TX_MODE_CYCLE,
    S7XG_TX_MODE_NO_CYCLE,
};

enum MacJoin {
    S7XG_MAC_OTAA,
    S7XG_MAC_ABP,
    S7XG_MAC_MAX
};

enum AcsipFwVer {
    ACSIP_FW_VERSION_V165G9,
    ACSIP_FW_VERSION_V166G11,
};

struct GPSModeStruct {
    GPSMode mode;
    GPSStartMode start;
    uint8_t port;
    uint32_t cycle;
    GPSUplinkFormat format;
    GPSSatelliteSys sys;
    bool pps;
};

struct GPSDateTime {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
};

struct GPS_DD {
    struct GPSDateTime datetime;
    double lat;
    double lng;
};

struct GPS_DMS {
    struct GPSDateTime datetime;
    struct  {
        int dd;
        int mm;
        double ss;
    } lat;
    struct  {
        int dd;
        int mm;
        double ss;
    } lng;
};

struct GPS_RAW {
    struct GPSDateTime datetime;
    double lat;
    double lng;
};

struct GPSDataStruct {
    union {
        struct GPS_DMS dms;
        struct GPS_DD dd;
        struct GPS_RAW raw;
    };
    bool isValid;
    float second;
};

struct ChannelParameter {
    uint32_t uplinkFreq;
    uint32_t downLinkFreq;
    uint8_t minDataRate;
    uint8_t maxDataRate;
    uint8_t bandId;
};

typedef void (*rf_callback)(const char *data, int rssi, int snr);

class Acsip
{
public:
    static String errrToString(int err_code);


    bool begin(HardwareSerial &port);

    /*****************************************
     *          SIP FUNCTION
     ****************************************/

    void reset();
    const char *getModel();
    const char *factoryReset();
    const char *getVersion();

    int setEcho(bool on);
    int setLog(int level);

    const char *getHardWareVer();
    const char *getUUID();

    int sleep(uint32_t second, bool uratWake);

    int setBaudRate(uint32_t baud, const char *password);
    int setStorage(uint8_t *buffer, uint32_t len);
    int getStorage(uint8_t *buffer, uint32_t &len);

    int setGPIOMode(GPIOGroup group, int pin, int mode);
    int setGPIOValue(GPIOGroup group, int pin, uint8_t val);
    bool getGPIOValue(GPIOGroup group, int pin);


    int setBatteryResistor(uint32_t r1, uint32_t r2);
    int getBatteryResistor(uint32_t &r1, uint32_t &r2);
    int getBatteryVoltage(uint16_t &volt);

    /*****************************************
     *          MAC FUNCTION
     ****************************************/
    int joinOTAA();
    int joinABP();
    int join(const char *type);
    int send(uint8_t port, uint8_t *data, size_t len, uint8_t type = 1);
    int getAutoJoin();
    const char getClass();
    int setClass(char type);
    bool isJoin();
    int getPower(int &pwr);
    int setDevEui(const char *deveui);
    int setAppEui(const char *appeui);
    int setAppKey(const char *appkey);
    int setDevAddr(const char *addr);
    int setNetworkSessionKey(const char *key);
    int setAppSessionKey(const char *key);
    int setChannelFreq(uint8_t channel, uint32_t freq);


    int setDataRate(uint8_t rate);


    int setPower(int dBm);
    int getBand(int &band);
    int getAdr(bool &isOn);

    // it can be from 0 to 255
    int getTxRetry(uint8_t &count);
    int getRxDelay(uint32_t &rx1, uint32_t &rx2);
    int getDataRate(int &dr);
    int getSync(uint8_t &syncWord);

    int getRx2(uint8_t &datarate, uint32_t &freq);
    int getChannelParameter(uint8_t channel, ChannelParameter &param);
    int getChannelStatus(uint8_t channel, bool &isOn);
    int getDutyCycleSwitch(bool &isOn);
    int getDutyCycleBand(uint8_t id, uint32_t &dutyCycle);

    int getAppKey(String &key);
    int getAppSessionKey(String &key);
    int getNetworkSessionKey(String &key);
    int getAppEui(String &eui);
    int getDevEui(String &eui);
    int getDevAddr(String &addr);



    int getJoinChannel();
    int getUplinkCounter(uint32_t &count);
    int getDownLinkCounter(uint32_t &count);
    int setTxMode(TxMode mode);
    int getTxMode(TxMode &mode);
    int setBatteryIndication(uint8_t level);
    int getBatteryIndication(uint8_t &level);
    int setTxConfirm(bool on);
    int getTxConfirm(bool &on);

    int setLBT(bool on);
    int getLBT(bool &on);
    int setUplinkDwell(bool on);
    int getUplinkDwell(bool &on);
    int setDownlinkDwell(bool on);
    int getDownlinkDwell(bool &on);
    int setMaxEIRP(uint8_t index);
    int getMaxEIRP(uint8_t &index);
    int setChannelCount(uint8_t channelCount, uint16_t bw);
    int getChannelCount(uint8_t &count);
    int setKeys(const char *devAddr, const char *devEui, const char *appEui, const char *appKey, const char *appsKey,
                const char *nwksKey);
    int setTxInterval(uint32_t ms);
    int getTxInterval(uint32_t &ms);
    int setRx1Freq(uint32_t freqBegin, uint32_t step, uint8_t count);
    int getRx1Freq(uint32_t &freqBegin, uint32_t &step, uint8_t &count);
    int setAutoJoin(bool on, MacJoin type, uint8_t count);
    int getAutoJoin(bool &on, MacJoin &type, uint8_t &count);
    int setPowerIndex(uint8_t index);
    int getPowerIndex(uint8_t index);

    /*****************************************
     *          RF FUNCTION
     ****************************************/
    //Representing communication frequency in Hz,
    // it can be values from 862000000 to 932000000 (S76S); 137000000 to 525000000 (S78S).
    int setRfFreq(uint32_t freq);

    ////Representing transmitting power in dBm, it can be from 2 to 20.
    int setRfPower(uint8_t dBm);

    //Representing spreading factor used for communication, it can be: 7, 8, 9,10, 11 and 12.
    int setRfSpreadingFactor(uint8_t factor);

    //Save p2p configuration parameters to EEPROM.
    int setRfSave();

    //Representing signal bandwidth in kHz, it can be: 125, 250, 500.
    int setRfBandWitdth(uint16_t bw);

    //Representing coding rate, can be: 5, 6, 7, 8.
    int setRfCodingRate(uint8_t r);

    int setRfPreambleLength(uint16_t pl);

    //Representing whether the CRC header is on or off.
    int setRfCRC(bool en);

    int setRfIQInvert(bool en);

    //A hexadecimal string representing sync word, it can be from 00 to FF.
    int setRfSyncWord(uint8_t sw);

    int setRfFreqDeviation(uint16_t dev);

    int setReceiveContinuous(bool en);

    int RfSend(char *hexData);

    int RfSendString(const char *str);

    int getRfFreq(uint32_t &freq);

    int getRfPower(uint8_t &dBm);

    int getRfSpreadingFactor(uint8_t &factor);

    /*****************************************
     *          GPS FUNCTION
     ****************************************/
    //! low level api
    int setLevelShift(bool on);
    //it only can be rmc.
    int setNmea(GPSSentence type = S7XG_RMC);
    int setPortUplink(uint8_t port);
    int setFormatUplink(GPSUplinkFormat format);
    int setPositioningCycle(uint32_t cycle);
    int setMode(GPSMode mode);
    int setSatelliteSystem(GPSSatelliteSys type);
    int setStart(GPSStartMode type);

    int getMode(GPSModeStruct &data);
    int getData( GPSDataStruct &data, GPSDataType type = S7XG_GPS_DATA_DD);
    int getTtff(float &second);

    int gpsReset();
    int gpsSleep();
    int gpsDeepSleep();
    int gpsWakeup();


    //! high level api
    int GPSStart(GPSStartMode type = S7XG_GPS_START_HOT,
                 GPSMode mode = S7XG_GPS_MODE_MANUAL, GPSSatelliteSys satellite = S7XG_SATELLITE_GPS, uint32_t cycle = 5000);
    int GPSStop();


    struct GPSDataStruct GPSData;

    /*****************************************
     *          OTHER FUNCTION
     ****************************************/
    void setTimeout(uint32_t cycle);
    int  service();
    void setRFCallback(rf_callback cb);

private:

    bool rf_check_available(const char *ptr);
    int getArgs(const char *cmd, const char *format, ...);
    template <typename T> int getUnit(const char *cmd, T &value);
    int checkOnOff(const char *cmd, bool &isOn);
    int universalSendConnamd(const char *format, ...);
    int snedConnamd(const char *format, ...);
    int universalSendCmd(const char *cmd);
    inline bool cmpstr(const char *str) __attribute__((always_inline));
    inline void sendCmd(const char *cmd) __attribute__((always_inline));

    int waitForAck(char *ack, uint32_t timeout = 0);

    char            buffer[256];
    HardwareSerial *_port;
    bool            isHardwareSerial = false;

    uint32_t        _timeout;
    rf_callback     _rf_callback = nullptr;
    int          version;

};
