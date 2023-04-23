#include "acsip.h"
#include <stdarg.h>

#define GPS_DD_FORMAT           "DD UTC( %hu/%hhu/%hhu %hhu:%hhu:%hhu ) LAT( %lf N ) LONG( %lf E ) POSITIONING( %fs )"
#define GPS_RAW_FORMAT          "RAW UTC( %hu/%hhu/%hhu %hhu:%hhu:%hhu ) LAT( %lf N ) LONG( %lf E ) POSITIONING( %fs )"
#define GPS_DMS_FORMAT          "DMS UTC( %hu/%hhu/%hhu %hhu:%hhu:%hhu ) LAT( %d*%d'%lf\" N ) LONG( %d * %d'%lf\" E ) POSITIONING( %fs )"

static const char *rxTxMode[2] = {"cycle", "no_cycle"};


String Acsip::errrToString(int err_code)
{
    switch (err_code) {
    case S7XG_OK:
        return "OK";
    case S7XG_FAILED:
        return "Failed";
    case S7XG_TIMEROUT:
        return "Timeout";
    case S7XG_JOINED:
        return "Joined";
    case S7XG_UNJOINED:
        return "UnJoined";
    case S7XG_INVALD:
        return "Command invald";
    case S7XG_ALREADY_JOINED:
        return "Already Joined";
    case S7XG_BUSY:
        return "Busy";
    case S7XG_INVALD_LEN:
        return "Invald len";
    case S7XG_GPS_NOT_INIT:
        return "Not init";
    case S7XG_GPS_NOT_POSITIONING:
        return "Positioning";
    case S7XG_GPS_SUCCESS:
        return "Success";
    case S7XG_GPS_ERROR:
        return "GPS Connect Error";
    case S7XG_COMMAND_ERROR:
        return "Unknown command!";
    default:
        break;
    }
    return "Unkonw";
}


static int hexToString(const char *src, uint8_t *dst, size_t srcLen, size_t &dstLen)
{
    if (src == NULL) {
        return -1;
    }
    dstLen = 0;
    if (*src == 0) {
        return 0;
    }
    for (int i = 0; src[i]; i++) {
        if (isxdigit(src[i])) {
            dstLen++;
        }
    }
    if (dstLen & 0x01) {
        dstLen++;
    }
    dstLen /= 2;
    if (dstLen > srcLen) {
        return -1;
    }
    dstLen = 0;

    int has = 0;
    for (int i = 0; src[i]; i++) {
        if (!isxdigit(src[i])) {
            continue;
        }
        uint8_t val = src[i] - (isdigit(src[i]) ? 0x30 : (isupper(src[i]) ? 0x37 : 0x57));
        if (has == 0) {
            dst[dstLen] = val << 4;
            has++;
        } else {
            dst[dstLen] |= val;
            has = 0;
            dstLen++;
        }
    }
    return 0;
}


template <typename T> int Acsip::getUnit(const char *cmd, T &value)
{
    sendCmd(cmd);
    if (waitForAck(buffer) != S7XG_OK) {
        return S7XG_TIMEROUT;
    }
    if (std::is_same<T, float>::value) {
        value = atof(buffer);
    }
    if (std::is_same<T, unsigned int>::value || std::is_same<T, unsigned char>::value
            || std::is_same<T, int>::value ) {
        value = atoi(buffer);
    }
    return S7XG_OK;
}

int Acsip::getArgs(const char *cmd, const char *format, ...)
{
    int err = 0;
    sendCmd(cmd);
    if (waitForAck(buffer) != S7XG_OK) {
        return S7XG_TIMEROUT;
    }
    va_list args;
    va_start(args, format);
    err = vsscanf(buffer, format, args);
    va_end(args);
    return err < 0 ? S7XG_FAILED : S7XG_OK;
}

int Acsip::checkOnOff(const char *cmd, bool &isOn)
{
    isOn = false;
    sendCmd(cmd);
    if (waitForAck(buffer) != S7XG_OK) {
        return S7XG_TIMEROUT;
    }
    if (cmpstr("on")) {
        isOn = true;
    } /*else if (cmpstr("off")) {
    }*/
    return S7XG_OK;
}

int Acsip::snedConnamd(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    int err = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    if (err < 0) {
        return S7XG_FAILED;
    }
    _port->print(buffer);
    return S7XG_OK;
}

int Acsip::universalSendConnamd(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    int err = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    if (err < 0) {
        return S7XG_FAILED;
    }
    _port->print(buffer);
    if (waitForAck(buffer) != S7XG_OK) {
        return S7XG_TIMEROUT;
    }
    if (cmpstr("Ok")) {
        return S7XG_OK;
    } else if (cmpstr("Invalid")) {
        return S7XG_INVALD;
    }
    return S7XG_FAILED;
}

int Acsip::universalSendCmd(const char *cmd)
{
    sendCmd(cmd);
    if (waitForAck(buffer) != S7XG_OK) {
        return S7XG_TIMEROUT;
    }
    if (cmpstr("Ok")) {
        return S7XG_OK;
    } else if (cmpstr("Invalid")) {
        return S7XG_INVALD;
    } else if (cmpstr("Please disconnect UART4 TX/RX")) {
        return S7XG_GPS_ERROR;
    } else if (cmpstr("Unknown command!")) {
        return S7XG_COMMAND_ERROR;
    }


    return S7XG_UNKONW;
}

void Acsip::setTimeout(uint32_t cycle)
{
    _timeout = cycle;
}

bool Acsip::cmpstr(const char *str)
{
    return (bool)(strcmp(buffer, str) == 0);
}

void Acsip::sendCmd(const char *cmd)
{
    _port->print(cmd);
    DEBUGLN(cmd);
}

int Acsip::waitForAck(char *ack, uint32_t timeout)
{
    uint8_t serial_buffer[256] = {0};
    int i = 0;
    uint32_t utimerStart;
    utimerStart = millis();

    while (1) {
        if (_port->available()) {
            //Preventing buffer overflows
            if (i >= sizeof(buffer))return S7XG_FAILED;
            serial_buffer[i] = _port->read();

#ifdef DEBUG_SERIAL_HEX
#ifdef DEBUG_PORT
            DEBUG("0x");
            DEBUG_PORT.print(serial_buffer[i], HEX);
            DEBUG(",");
#endif
#endif
            //respone command prefix [\n\r>>]
            //respone command suffix [\n]
            if (serial_buffer[0] == 0xA && serial_buffer[1] == 0xD &&
                    serial_buffer[2] == 0x3E && serial_buffer[3] == 0x3E
                    && serial_buffer[i] == 0xA) {
                memcpy(ack, &serial_buffer[5], i - 5);
                ack[i - 5] = '\0';
#ifdef DEBUG_PORT
                DEBUG_PORT.printf("Recvicer Done .. [size:%d] -> %s\n", i - 5, ack);
#endif
                return S7XG_OK;
            }
            i++;
        }
        if (millis() - utimerStart > timeout != 0 ? timeout :  _timeout) {
            DEBUGLN(".");
#ifdef DEBUG_PORT
            DEBUG_PORT.println("Time out flush uart!");
            _port->flush();
            while (_port->available()) {
                char c = _port->read();
                DEBUG_PORT.print(c);
            }
            DEBUG_PORT.println();
#endif
            break;
        }
    }
    return S7XG_TIMEROUT;
}


bool Acsip::begin(HardwareSerial &port)
{
    isHardwareSerial = true;
    _port = &port;
    _timeout = DEFAULT_SERIAL_TIMEOUT;

    sendCmd("rf rx_con off");
    waitForAck(buffer, 2000);

    sendCmd("rf lora_tx_stop");
    waitForAck(buffer, 2000);

    sendCmd("rf lora_rx_stop");
    waitForAck(buffer, 2000);
    waitForAck(buffer, 2000);

    reset();

    String model = getModel();
    if (model == "S76G" || model == "S78G") {
        getVersion();
        if (cmpstr("v1.6.6-g11")) {
            version = ACSIP_FW_VERSION_V166G11;
        } else if (cmpstr("v1.6.5-g9")) {
            version = ACSIP_FW_VERSION_V165G9;
        }
        return true;
    }
    return false;
}

/*****************************************
 *          SIP FUNCTION
 ****************************************/
void Acsip::reset()
{
    sendCmd("sip reset");
    delay(1000);
    while (_port->available()) {
        _port->read();
    }
    _port->flush();
    _port->flush();
}

const char *Acsip::getModel()
{
    sendCmd("sip get_hw_model");
    if (waitForAck(buffer) == S7XG_OK) {
        if (cmpstr("S76G")) {
            return "S76G";
        } else if (cmpstr("S78G")) {
            return "S78G";
        }
    }
    return "Unkonw";
}

const char *Acsip::factoryReset()
{
    sendCmd("sip factory_reset");
    if (waitForAck(buffer) == S7XG_OK) {
        return buffer;
    }
    return "Unkonw";
}

const char *Acsip::getVersion()
{
    sendCmd("sip get_ver");
    if (waitForAck(buffer) == S7XG_OK) {
        return buffer;
    }
    return "Unkonw";
}

int Acsip::setEcho(bool on)
{
    snprintf(buffer, sizeof(buffer), "sip set_echo %s", on ? "on" : "off");
    return universalSendCmd(buffer);
}

int Acsip::setLog(int level)
{
    if (level >= 2 )    return S7XG_INVALD;
    const char *log[2] = {"debug", "info"};
    snprintf(buffer, sizeof(buffer), "sip set_log %s", log[level]);
    return universalSendCmd(buffer);
}

/**
 * @brief  sleep
 * @note
 * @param  second: Input value must be the multiple of 10.
 * @param  uratWake: Means it can be interrupted (waked up) by UART.
 * @retval
 */
int Acsip::sleep(uint32_t second, bool uratWake)
{
    if (second % 10) return S7XG_INVALD;
    snprintf(buffer, sizeof(buffer), "sip sleep %u %s", second, uratWake ? "uart_on" : "uart_off");
    sendCmd(buffer);
    if (waitForAck(buffer) != S7XG_OK) {
        return S7XG_FAILED;
    }
    if (strncmp(buffer, "sleep", 5) == 0) {
        return S7XG_OK;
    }
    return S7XG_FAILED;
}

int Acsip::setBaudRate(uint32_t baud, const char *password)
{
    snprintf(buffer, sizeof(buffer), "sip set_baudrate %u %s", baud, password);
    return universalSendCmd(buffer);
}

const char *Acsip::getHardWareVer()
{
    sendCmd("sip get_hw_model_ver");
    if (waitForAck(buffer) == S7XG_OK) {
        return buffer;
    }
    return "Unkonw";
}


static const char grp[7] = {'A', 'B', 'C', 'D', 'E', 'F', 'H'};

int Acsip::setGPIOMode(GPIOGroup group, int pin, int mode)
{
    if (pin <= 0 && pin > 16) return S7XG_INVALD;
    char m = (mode == INPUT) ? '0' : '1';
    snprintf(buffer, sizeof(buffer), "sip set_gpio_mode %c %d %c", grp[group], pin, m);
    return universalSendCmd(buffer);
}

int Acsip::setGPIOValue(GPIOGroup group, int pin, uint8_t val)
{
    if (pin <= 0 && pin > 16) return S7XG_INVALD;
    snprintf(buffer, sizeof(buffer), "sip set_gpio %c %d %u", grp[group], pin, val);
    return universalSendCmd(buffer);
}

bool Acsip::getGPIOValue(GPIOGroup group, int pin)
{
    if (pin <= 0 && pin > 16) return S7XG_INVALD;
    snprintf(buffer, sizeof(buffer), "sip get_gpio %c %d", grp[group], pin);
    sendCmd(buffer);
    if (waitForAck(buffer) == S7XG_OK) {
        if (cmpstr("1")) {
            return true;
        }
    }
    return false;
}

const char *Acsip::getUUID()
{
    sendCmd("sip get_uuid");
    if (waitForAck(buffer) == S7XG_OK) {
        const char *uuid = strchr(buffer, '=');
        if ( uuid == 0) {
            return "Unkonw";
        }
        return uuid + 1;
    }
    return "Unkonw";
}

int Acsip::setStorage(uint8_t *buffer, uint32_t len)
{
    // TODO :
    return S7XG_OK;
}

int Acsip::getStorage(uint8_t *buffer, uint32_t &len)
{
    // TODO :
    return S7XG_OK;
}

int Acsip::setBatteryResistor(uint32_t r1, uint32_t r2)
{
    snprintf(buffer, sizeof(buffer), "sip set_batt_resistor %u %u", r1, r2);
    return universalSendCmd(buffer);
}

int Acsip::getBatteryResistor(uint32_t &r1, uint32_t &r2)
{
    return getArgs("sip get_batt_resistor", "%u %u", &r1, &r2);
}

int Acsip::getBatteryVoltage(uint16_t &volt)
{
    char *ch = NULL;
    char strr[32] = {0};
    sendCmd("sip get_batt_volt");

    if (waitForAck(buffer) != S7XG_OK) {
        return S7XG_TIMEROUT;
    }

    if (strncmp(buffer, "adc volt", strlen("adc volt")) == 0) {
        if (waitForAck(buffer) != S7XG_OK) {
            return S7XG_TIMEROUT;
        }
        if (strncmp(buffer, "battery volt", strlen("battery volt")) == 0) {
            char *ptr = buffer + strlen("battery volt") + 1;
            ch = strchr(ptr, ' ');
            if (ch == NULL)return S7XG_FAILED;
            memcpy(strr, ptr, ch - ptr);
            volt = atoi(strr);
            return S7XG_OK;
        }
    } else if (strncmp(buffer, "battery volt", strlen("battery volt")) == 0) {
        char *ptr = buffer + strlen("battery volt") + 1;
        ch = strchr(ptr, ' ');
        if (ch == NULL)return S7XG_FAILED;
        memcpy(strr, ptr, ch - ptr);
        volt = atoi(strr);
        return S7XG_OK;
    }


    return S7XG_FAILED;
}

/*****************************************
 *          MAC FUNCTION
 ****************************************/

int Acsip::join(const char *type)
{
    snprintf(buffer, sizeof(buffer), "mac join %s", type);
    sendCmd(buffer);
    if (waitForAck(buffer) != S7XG_OK) {
        return S7XG_TIMEROUT;
    }
    if (cmpstr("Ok")) {
        if (waitForAck(buffer) != S7XG_OK) {
            return S7XG_TIMEROUT;
        }
        if (cmpstr("accepted")) {
            return S7XG_OK;
        }
        return S7XG_TIMEROUT;
    } else if (cmpstr("Invalid")) {
        return S7XG_FAILED;
    } else if (cmpstr("keys_not_init")) {
        return S7XG_FAILED;
    } else if (cmpstr("no_free_ch")) {
        return S7XG_FAILED;
    } else if (cmpstr("busy")) {
        return S7XG_FAILED;
    }
    return S7XG_OK;
}


int Acsip::joinOTAA()
{
    return join("otaa");
}

int Acsip::joinABP()
{
    return join("abp");
}


int Acsip::send(uint8_t port, uint8_t *data, size_t len, uint8_t type)
{
    if (port < 1 || port > 223)return S7XG_INVALD;
    //! FORMAT WARNING ... DONT'T EDIT
    snprintf(buffer, sizeof(buffer), "mac tx %s %d ", type ? "ucnf" : "cnf", port);
    DEBUG("Send->  ");
    DEBUG(buffer);
    _port->print(buffer);
    for (size_t i = 0; i < len; i++) {
        if ((data[i] & 0xF0) == 0) {
            DEBUG('0');
            _port->print('0');
        }

#ifdef DEBUG_PORT
        DEBUG_PORT.print(data[i], HEX);
#endif
        _port->print(data[i], HEX);

    }
    DEBUGLN();
    DEBUGLN();

    if (waitForAck(buffer) != S7XG_OK) {
        return S7XG_TIMEROUT;
    }
    if (cmpstr("Ok")) {
        if (waitForAck(buffer) != S7XG_OK) {
            return S7XG_TIMEROUT;
        }
        if (cmpstr("tx_ok")) {
            DEBUGLN("TX_OK");
            return S7XG_OK;
        } else if (cmpstr("err")) {
            DEBUGLN("TX_ERR");
            return S7XG_FAILED;
        }
    } else if (cmpstr("Invalid")) {
        return S7XG_INVALD;
    } else if (cmpstr("not_joined")) {
        return S7XG_UNJOINED;
    } else if (cmpstr("busy")) {
        return S7XG_BUSY;
    } else if (cmpstr("invalid_data_length")) {
        return S7XG_INVALD_LEN;
    } else if (cmpstr("exceeded_data_length")) {
        return S7XG_INVALD_LEN;
    }
    return S7XG_OK;
}


int Acsip::getAutoJoin()
{
    sendCmd("mac get_auto_join");
    if (waitForAck(buffer) != S7XG_OK) {
        return S7XG_TIMEROUT;
    }
    if (buffer[0] == 'o') {
        if (cmpstr("off")) {
            return  S7XG_JOIN_OFF;
        } else if (strncmp(buffer, "otta", 4) == 0) {
            return  S7XG_JOIN_OTAA;
        }
    } else if (cmpstr("abp")) {
        return  S7XG_JOIN_ABP;
    }
    return S7XG_FAILED;
}

const char Acsip::getClass()
{
    sendCmd("mac get_class");
    if (waitForAck(buffer) != S7XG_OK) {
        return S7XG_TIMEROUT;
    }
    return buffer[0];
}

int Acsip::setClass(char type)
{
    if (type == 'A') {
        sendCmd("mac set_class A");
    } else if (type == 'C') {
        sendCmd("mac set_class C");
    } else {
        return S7XG_INVALD;
    }
    if (waitForAck(buffer) == S7XG_OK) {
        if (cmpstr("already_joined")) {
            return S7XG_ALREADY_JOINED;
        } else if (cmpstr("Invalid")) {
            return S7XG_INVALD;
        } else if (cmpstr("Ok")) {
            return S7XG_OK;
        }
    }
    return S7XG_TIMEROUT;
}

bool Acsip::isJoin()
{
    sendCmd("mac get_join_status");
    if (waitForAck(buffer) == S7XG_OK) {
        if (cmpstr("joined")) {
            return true;
        } else if (cmpstr("unjoined")) {
            return false;
        }
    }
    return false;
}

int Acsip::getPower(int &pwr)
{
    return getUnit("mac get_power", pwr);
}


int Acsip::setDevEui(const char *deveui)
{
    snprintf(buffer, sizeof(buffer), "mac set_deveui %s", deveui);
    return universalSendCmd(buffer);
}

int Acsip::setAppEui(const char *appeui)
{
    snprintf(buffer, sizeof(buffer), "mac set_appeui %s", appeui);
    return universalSendCmd(buffer);
}

int Acsip::setAppKey(const char *appkey)
{
    snprintf(buffer, sizeof(buffer), "mac set_appkey %s", appkey);
    return universalSendCmd(buffer);
}


int Acsip::setDevAddr(const char *addr)
{
    snprintf(buffer, sizeof(buffer), "mac set_devaddr %s", addr);
    return universalSendCmd(buffer);
}

int Acsip::setNetworkSessionKey(const char *key)
{
    snprintf(buffer, sizeof(buffer), "mac set_nwkskey %s", key);
    return universalSendCmd(buffer);
}

int Acsip::setAppSessionKey(const char *key)
{
    snprintf(buffer, sizeof(buffer), "mac set_appskey %s", key);
    return universalSendCmd(buffer);
}


int Acsip::setChannelFreq(uint8_t channel, uint32_t freq)
{
    snprintf(buffer, sizeof(buffer), "mac set_ch_freq %d %u", channel, freq);
    return universalSendCmd(buffer);
}

/**
 * @brief  setDataRate
 * @note   it can be from 0 to 6. US915 is limited from 0 to 4 ; CN470 is limited from 0 to 5
 * @param  rate: [0 - 6]
 * @retval status code , see 'S7XG_Error' enum
 */
int Acsip::setDataRate(uint8_t rate)
{
    snprintf(buffer, sizeof(buffer), "mac set_dr %u", rate);
    return universalSendCmd(buffer);
}


/**
 * @brief  setPower
 * @note
 * @param  dBm: (non - 915 band):2, 5, 8, 11, 14, 20 ,
 *              (915 band):30, 28, 26, 24, 22, 20, 18, 16, 14, 12, 10 ;
 *              (470band):17, 16, 14, 12, 10, 7, 5, 2
 * @retval status code , see 'S7XG_Error' enum
 */
int Acsip::setPower(int dBm)
{
    snprintf(buffer, sizeof(buffer), "mac set_power %u", dBm);
    return universalSendCmd(buffer);
}

int Acsip::getBand(int &band)
{
    return getUnit("mac get_band", band);
}

int Acsip::getAdr(bool &isOn)
{
    return checkOnOff("mac get_adr", isOn);
}

int Acsip::getTxRetry(uint8_t &count)
{
    return getUnit("mac get_txretry", count);
}

int Acsip::getRxDelay(uint32_t &rx1, uint32_t &rx2)
{
    return getArgs("mac get_rxdelay", "%u %u", &rx1, &rx2);
}

int Acsip::getDataRate(int &dr)
{
    return getUnit("mac get_dr", dr);
}

int Acsip::getRx2(uint8_t &datarate, uint32_t &freq)
{
    return getArgs("mac get_rx2", "%hhu %u", &datarate, &freq);
}

int Acsip::getSync(uint8_t &syncWord)
{
    return getArgs("mac get_sync", "%hhu", &syncWord);
}

int Acsip::getChannelParameter(uint8_t channel, ChannelParameter &param)
{
    snprintf(buffer, sizeof(buffer), "mac get_ch_para %hhu", channel);

    return getArgs(buffer, "%u %hhu %hhu %hhu %u",
                   &param.uplinkFreq,
                   &param.minDataRate,
                   &param.maxDataRate,
                   &param.bandId,
                   &param.downLinkFreq
                  );
}

int Acsip::getChannelStatus(uint8_t channel, bool &isOn)
{
    snprintf(buffer, sizeof(buffer), "mac get_ch_status %u", channel);
    return checkOnOff(buffer, isOn);
}


int Acsip::getDutyCycleSwitch(bool &isOn)
{
    return checkOnOff("mac get_dc_ctl", isOn);
}

int Acsip::getDutyCycleBand(uint8_t id, uint32_t &dutyCycle)
{
    snprintf(buffer, sizeof(buffer), "mac get_dc_band %u", id);
    return getArgs(buffer, "%hhu", &dutyCycle);
}

int Acsip::getAppKey(String &key)
{
    sendCmd("mac get_appkey");
    if (waitForAck(buffer) != S7XG_OK) {
        return S7XG_TIMEROUT;
    }
    key = buffer;
    return S7XG_OK;
}

int Acsip::getAppSessionKey(String &key)
{
    sendCmd("mac get_appskey");
    if (waitForAck(buffer) != S7XG_OK) {
        return S7XG_TIMEROUT;
    }
    key = buffer;
    return S7XG_OK;
}

int Acsip::getNetworkSessionKey(String &key)
{
    sendCmd("mac get_nwkskey");
    if (waitForAck(buffer) != S7XG_OK) {
        return S7XG_TIMEROUT;
    }
    key = buffer;
    return S7XG_OK;
}

int Acsip::getAppEui(String &eui)
{
    sendCmd("mac get_appeui");
    if (waitForAck(buffer) != S7XG_OK) {
        return S7XG_TIMEROUT;
    }
    eui = buffer;
    return S7XG_OK;
}

int Acsip::getDevEui(String &eui)
{
    sendCmd("mac get_deveui");
    if (waitForAck(buffer) != S7XG_OK) {
        return S7XG_TIMEROUT;
    }
    eui = buffer;
    return S7XG_OK;
}

int Acsip::getDevAddr(String &addr)
{
    sendCmd("mac get_devaddr");
    if (waitForAck(buffer) != S7XG_OK) {
        return S7XG_TIMEROUT;
    }
    addr = buffer;
    return S7XG_OK;
}

int Acsip::getJoinChannel()
{
    //TODO:
    return S7XG_OK;
}

int Acsip::getUplinkCounter(uint32_t &count)
{
    return getUnit("mac get_upcnt", count);
}

int Acsip::getDownLinkCounter(uint32_t &count)
{
    return getUnit("mac get_downcnt", count);
}


int Acsip::setTxMode(TxMode mode)
{
    snprintf(buffer, sizeof(buffer), "mac set_tx_mode %s", rxTxMode[mode]);
    return universalSendCmd(buffer);
}

int Acsip::getTxMode(TxMode &mode)
{
    sendCmd("mac get_tx_mode");
    if (waitForAck(buffer) != S7XG_OK) {
        return S7XG_TIMEROUT;
    }
    if (cmpstr("cycle")) {
        mode = S7XG_TX_MODE_CYCLE;
    } else if (cmpstr("no_cycle")) {
        mode = S7XG_TX_MODE_NO_CYCLE;
    }
    return S7XG_OK;
}

int Acsip::setBatteryIndication(uint8_t level)
{
    snprintf(buffer, sizeof(buffer), "mac set_batt %u", level);
    return universalSendCmd(buffer);
}

int Acsip::getBatteryIndication(uint8_t &level)
{
    return getUnit("mac get_batt", level);
}

int Acsip::setTxConfirm(bool on)
{
    snprintf(buffer, sizeof(buffer), "mac set_tx_confirm %s", on ? "on" : "off");
    return universalSendCmd(buffer);
}

int Acsip::getTxConfirm(bool &on)
{
    return checkOnOff("mac get_tx_confirm", on);
}


int Acsip::setLBT(bool on)
{
    snprintf(buffer, sizeof(buffer), "mac set_lbt %s", on ? "on" : "off");
    return universalSendCmd(buffer);
}

int Acsip::getLBT(bool &on)
{
    return checkOnOff("mac get_lbt", on);
}

int Acsip::setUplinkDwell(bool on)
{
    snprintf(buffer, sizeof(buffer), "mac set_uplink_dwell %s", on ? "on" : "off");
    return universalSendCmd(buffer);
}

int Acsip::getUplinkDwell(bool &on)
{
    return checkOnOff("mac get_uplink_dwell", on);
}

int Acsip::setDownlinkDwell(bool on)
{
    snprintf(buffer, sizeof(buffer), "mac set_downlink_dwell %s", on ? "on" : "off");
    return universalSendCmd(buffer);
}

int Acsip::getDownlinkDwell(bool &on)
{
    return checkOnOff("mac get_downlink_dwell", on);
}

//A decimal string representing MaxEIRP index defined in LoRaWAN TM v1.0.2, it can be 0 to 15.
int Acsip::setMaxEIRP(uint8_t index)
{
    snprintf(buffer, sizeof(buffer), "mac set_max_eirp %u", index);
    return universalSendCmd(buffer);
}

int Acsip::getMaxEIRP(uint8_t &index)
{
    return getUnit("mac get_max_eirp", index);
}

/**
 * @brief  setChannelCount
 * @note
 * @param  channelCount: a decimal string representing channel count, it can only be 0~8, 16, 32, 48, 64, 80 and 96.
 * @param  bw: a decimal string representing which channels group different from bandwidth, it can only be 125 or 500.
 * @retval
 */
int Acsip::setChannelCount(uint8_t channelCount, uint16_t bw)
{
    snprintf(buffer, sizeof(buffer), "mac set_ch_count %u %u", channelCount, bw);
    return universalSendCmd(buffer);
}

int Acsip::getChannelCount(uint8_t &count)
{
    return getUnit("mac get_ch_count", count);
}

int Acsip::setKeys(const char *devAddr,
                   const char *devEui,
                   const char *appEui,
                   const char *appKey,
                   const char *appsKey,
                   const char *nwksKey)
{
    snprintf(buffer, sizeof(buffer), "mac set_keys %s %s %s %s %s %s", devAddr, devEui, appEui, appKey, appsKey, nwksKey);
    return universalSendCmd(buffer);
}

int Acsip::setTxInterval(uint32_t ms)
{
    snprintf(buffer, sizeof(buffer), "mac set_tx_interval %u", ms);
    return universalSendCmd(buffer);
}

int Acsip::getTxInterval(uint32_t &ms)
{
    return getUnit("mac get_tx_interval", ms);
}

int Acsip::setRx1Freq(uint32_t freqBegin, uint32_t step, uint8_t count)
{
    snprintf(buffer, sizeof(buffer), "mac set_rx1_freq %u %u %u", freqBegin, step, count);
    return universalSendCmd(buffer);
}

int Acsip::getRx1Freq(uint32_t &freqBegin, uint32_t &step, uint8_t &count)
{
    sendCmd("mac get_rx1_freq");
    if (waitForAck(buffer) != S7XG_OK) {
        return S7XG_TIMEROUT;
    }
    //TODO:
    return S7XG_OK;
}

int Acsip::setAutoJoin(bool on, MacJoin type, uint8_t count)
{
    if (type >= S7XG_MAC_MAX)return S7XG_INVALD;
    snprintf(buffer, sizeof(buffer), "mac set_auto_join %s %s %d", on ? "on" : "off", type == S7XG_MAC_OTAA ? "otaa" : "abp", count);
    return universalSendCmd(buffer);
}

int Acsip::getAutoJoin(bool &on, MacJoin &type, uint8_t &count)
{
    sendCmd("mac get_auto_join");
    if (waitForAck(buffer) != S7XG_OK) {
        return S7XG_TIMEROUT;
    }
    //TODO:
    return S7XG_OK;
}

int Acsip::setPowerIndex(uint8_t index)
{
    snprintf(buffer, sizeof(buffer), "mac set_power_index %u", index);
    return universalSendCmd(buffer);
}

int Acsip::getPowerIndex(uint8_t index)
{
    return getUnit("mac get_power_index", index);
}

/*****************************************
 *          RF FUNCTION
 ****************************************/

int Acsip::setRfFreq(uint32_t freq)
{
    snprintf(buffer, sizeof(buffer), "rf set_freq %u", freq);
    return universalSendCmd(buffer);
}

int Acsip::setRfPower(uint8_t dBm)
{
    snprintf(buffer, sizeof(buffer), "rf set_pwr %u", dBm);
    return universalSendCmd(buffer);
}

int Acsip::setRfSpreadingFactor(uint8_t factor)
{
    snprintf(buffer, sizeof(buffer), "rf set_sf %u", factor);
    return universalSendCmd(buffer);
}

int Acsip::setRfSave()
{
    return universalSendCmd("rf save");
}

int Acsip::setRfBandWitdth(uint16_t bw)
{
    snprintf(buffer, sizeof(buffer), "rf set_bw %u", bw);
    return universalSendCmd(buffer);
}

int Acsip::setRfCodingRate(uint8_t r)
{
    snprintf(buffer, sizeof(buffer), "rf set_cr 4/%u", r);
    return universalSendCmd(buffer);
}

int Acsip::setRfPreambleLength(uint16_t pl)
{
    snprintf(buffer, sizeof(buffer), "rf set_prlen %u", pl);
    return universalSendCmd(buffer);
}

int Acsip::setRfCRC(bool en)
{
    snprintf(buffer, sizeof(buffer), "rf set_crc %s", en ? "on" : "off");
    return universalSendCmd(buffer);
}

int Acsip::setRfIQInvert(bool en)
{
    snprintf(buffer, sizeof(buffer), "rf set_iqi %s", en ? "on" : "off");
    return universalSendCmd(buffer);
}

int Acsip::setRfSyncWord(uint8_t sw)
{
    snprintf(buffer, sizeof(buffer), "rf set_sync %x", sw);
    return universalSendCmd(buffer);
}

int Acsip::setRfFreqDeviation(uint16_t dev)
{
    snprintf(buffer, sizeof(buffer), "rf set_fdev %u", dev);
    return universalSendCmd(buffer);
}

int Acsip::setReceiveContinuous(bool en)
{
    snprintf(buffer, sizeof(buffer), "rf rx_con %s", en ? "on" : "off");
    return universalSendCmd(buffer);
}

int Acsip::RfSendString(const char *str)
{
    int i = 0;
    char hex[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
    char *ascii = (char *)calloc(strlen(str) * 3 + 1, sizeof(char));
    if (ascii == NULL) {
        return S7XG_FAILED;
    }
    while ( i < strlen(str) ) {
        int c = str[i] & 0x000000ff;
        ascii[i * 2] = hex[c / 16] ;
        ascii[i * 2 + 1] = hex[c % 16] ;
        ++i;
    }
    int ret = RfSend(ascii);
    free(ascii);
    return ret;
}

int Acsip::RfSend(char *hexData)
{
    snprintf(buffer, sizeof(buffer), "rf tx %s", hexData);
    sendCmd(buffer);
    if (waitForAck(buffer) != S7XG_OK) {
        return S7XG_TIMEROUT;
    }
    if (cmpstr("Ok")) {
        if (waitForAck(buffer) != S7XG_OK) {
            return S7XG_TIMEROUT;
        }
        if (cmpstr("radio_tx_ok")) {
            return S7XG_OK;
        }
        return S7XG_OK;
    }
    return S7XG_FAILED;
}

int Acsip::getRfFreq(uint32_t &freq)
{
    return getUnit("rf get_freq", freq);
}

int Acsip::getRfPower(uint8_t &dBm)
{
    return getUnit("rf get_pwr", dBm);
}

int Acsip::getRfSpreadingFactor(uint8_t &factor)
{
    return getUnit("rf get_sf", factor);
}


/*****************************************
 *          GPS FUNCTION
 ****************************************/

int Acsip::setLevelShift(bool on)
{
    snprintf(buffer, sizeof(buffer), "gps set_level_shift %s", on ? "on" : "off");
    return universalSendCmd(buffer);
}

int Acsip::setMode(GPSMode mode)
{
    static const char *gpsMode[] = {
        "auto", "manual", "idle"
    };
    if (mode >= S7XG_GPS_MODE_MAX)return S7XG_INVALD;
    snprintf(buffer, sizeof(buffer), "gps set_mode %s", gpsMode[mode]);
    return universalSendCmd(buffer);
}

int Acsip::setPortUplink(uint8_t port)
{
    if (port < 1 || port > 223) return S7XG_INVALD;
    snprintf(buffer, sizeof(buffer), "gps set_port_uplink %u", port);
    return universalSendCmd(buffer);
}

int Acsip::setFormatUplink(GPSUplinkFormat format)
{
    static const char *uplinkFormat[] = {
        "raw", "ipso", "kiwi", "utc_pos"
    };
    if (format >= S7XG_GPS_FORMAT_MAX)return S7XG_INVALD;
    snprintf(buffer, sizeof(buffer), "gps set_format_uplink %s", uplinkFormat[format]);
    return universalSendCmd(buffer);
}

int Acsip::setPositioningCycle(uint32_t cycle)
{
    if (cycle < 1000 || cycle > 600000) return S7XG_INVALD;
    snprintf(buffer, sizeof(buffer), "gps set_positioning_cycle %u", cycle);
    return universalSendCmd(buffer);
}

int Acsip::setSatelliteSystem(GPSSatelliteSys type)
{
    static const char *system[] = {
        "gps", "hybrid"
    };
    if (type >= S7XG_SATELLITE_MAX)return S7XG_INVALD;
    snprintf(buffer, sizeof(buffer), "gps set_satellite_system %s", system[type]);
    return universalSendCmd(buffer);
}

int Acsip::setStart(GPSStartMode type)
{
    static const char *start[] = {
        "hot", "warm", "cold"
    };
    if (type >= S7XG_GPS_START_MAX)return S7XG_INVALD;
    snprintf(buffer, sizeof(buffer), "gps set_start %s", start[type]);
    return universalSendCmd(buffer);
}

int Acsip::setNmea(GPSSentence type)
{
    (void)type;
    //it only can be rmc.
    return universalSendCmd("gps set_nmea rmc");
}

int Acsip::getMode(GPSModeStruct &data)
{
    sendCmd("gps get_mode");
    if (waitForAck(buffer) != S7XG_OK) {
        return S7XG_TIMEROUT;
    }
    //TODO:
    // <Mode>: it follows “gps set_mode” command input format. (off/auto/manual)
    // <Start_Type>: it follows “gps set_start” command input format. (hot/cold/warm)
    // <Port>: it follows “gps set_port_uplink” command input format.
    // <Time>: it follows “gps set_positioning_cycle” command input format.
    // <Format>: it follows “gps set_format_uplink” command input format. (raw/ipso/kiwi/utc_pos)
    // <Satellite_System>: it follows “gps set_satellite_system” command input format. (gps/hybrid)

    // manual hot 20 1000 ipso gps 1PPS_on
    // off hot 1 0 raw gps 1PPS_off

    char *ptr = buffer;

    //! Mode
    if (strncmp(ptr, "auto", strlen("auto")) == 0) {
        DEBUGLN("auto");
        ptr += strlen("auto") + 1;
        data.mode = S7XG_GPS_MODE_AUTO;
    } else if (strncmp(ptr, "manual", strlen("manual")) == 0) {
        DEBUGLN("manual");
        ptr += strlen("manual") + 1;
        data.mode = S7XG_GPS_MODE_MANUAL;
    } else if (strncmp(ptr, "off", strlen("off")) == 0) {
        DEBUGLN("off");
        ptr += strlen("off") + 1;
        data.mode = S7XG_GPS_MODE_IDLE;
    } else if (strncmp(ptr, "idle", strlen("idle")) == 0) {
        DEBUGLN("idle");
        ptr += strlen("idle") + 1;
        data.mode = S7XG_GPS_MODE_IDLE;
    } else {
        data.mode = S7XG_GPS_MODE_MAX;
    }

    //! Start_Type
    if (strncmp(ptr, "hot", strlen("hot")) == 0) {
        DEBUGLN("hot");
        ptr += strlen("hot") + 1;
        data.start = S7XG_GPS_START_HOT;

    } else if (strncmp(ptr, "warm", strlen("warm")) == 0) {
        DEBUGLN("warm");
        ptr += strlen("warm") + 1;
        data.start = S7XG_GPS_START_WARM;
    } else if (strncmp(ptr, "cold", strlen("cold")) == 0) {
        DEBUGLN("cold");
        ptr += strlen("cold") + 1;
        data.start = S7XG_GPS_START_COLD;
    }

    //! Port ,cycle
    if (sscanf(ptr, "%hhu %u", &data.port, &data.cycle) < 2) {
        return S7XG_FAILED;
    }

    int c = 2;
    while (*ptr != '\0') {
        if (*ptr == ' ') {
            if (--c == 0) {
                ptr++;
                break;
            }
        }
        ptr++;
    }

    //! Format
    if (strncmp(ptr, "raw", strlen("raw")) == 0) {
        DEBUGLN("raw");
        ptr += strlen("raw") + 1;
        data.format = S7XG_GPS_FORMAT_RAW;
    } else if (strncmp(ptr, "ipso", strlen("ipso")) == 0) {
        DEBUGLN("ipso");
        ptr += strlen("ipso") + 1;
        data.format = S7XG_GPS_FORMAT_IPSO;
    } else if (strncmp(ptr, "kiwi", strlen("kiwi")) == 0) {
        DEBUGLN("kiwi");
        ptr += strlen("kiwi") + 1;
        data.format = S7XG_GPS_FORMAT_KIWI;
    } else if (strncmp(ptr, "utc_pos", strlen("utc_pos")) == 0) {
        DEBUGLN("utc_pos");
        ptr += strlen("utc_pos") + 1;
        data.format = S7XG_GPS_FORMAT_UTC_POS;
    }

    //! Satellite_System
    if (strncmp(ptr, "gps", strlen("gps")) == 0) {
        DEBUGLN("gps");
        ptr += strlen("gps") + 1;
        data.sys = S7XG_SATELLITE_GPS;
    } else if (strncmp(ptr, "hybrid", strlen("hybrid")) == 0) {
        DEBUGLN("hybrid");
        ptr += strlen("hybrid") + 1;
        data.sys = S7XG_SATELLITE_GPS_GLONASS;
    }

    //! 1PPS
    if (strncmp(ptr, "1PPS_off", strlen("1PPS_off")) == 0) {
        DEBUGLN("1PPS_off");
        data.pps = true;
    } else if (strncmp(ptr, "1PPS_on", strlen("1PPS_on")) == 0) {
        DEBUGLN("1PPS_on");
        data.pps = false;
    }

    return S7XG_OK;
}

int Acsip::getTtff(float &second)
{
    return getArgs("gps get_mode", "%fs", &second);
}

int Acsip::gpsReset()
{
    return universalSendCmd("gps reset");
}

int Acsip::gpsSleep()
{
    return universalSendCmd("gps sleep on 0");
}

int Acsip::gpsDeepSleep()
{
    return universalSendCmd("gps sleep on 1");
}

int Acsip::gpsWakeup()
{
    return universalSendCmd("gps sleep off");
}

int Acsip::getData( GPSDataStruct &data, GPSDataType type)
{
    int ret = 0;
    static const char *gpsType[] = {
        "raw", "dd", "dms"
    };
    data.isValid = false;

    if (type >= S7XG_GPS_DATA_MAX)return S7XG_INVALD;
    snprintf(buffer, sizeof(buffer), "gps get_data %s", gpsType[type]);
    sendCmd(buffer);
    if (waitForAck(buffer) != S7XG_OK) {
        return S7XG_TIMEROUT;
    }
    //TODO:
    if (strncmp(buffer, "POSITIONING", strlen("POSITIONING")) == 0) {
        const char *c = strchr(buffer, '(');
        if (c != NULL) {
            sscanf(buffer, "POSITIONING ( %fs )", &data.second);
        }
        return S7XG_OK;
    } else if (cmpstr("gps_not_init") ) {
        DEBUGLN("gps_not_init");
        return S7XG_GPS_NOT_INIT;
    } else if (cmpstr("gps_in_idle")) {
        DEBUGLN("gps_in_idle");
        return S7XG_FAILED;
    } else if (cmpstr("gps_not_positioning")) {
        return S7XG_FAILED;
    }

    switch (type) {
    case S7XG_GPS_DATA_RAW:
        ret = sscanf(buffer, GPS_RAW_FORMAT,
                     &data.raw.datetime.year,
                     &data.raw.datetime.month,
                     &data.raw.datetime.day,
                     &data.raw.datetime.hour,
                     &data.raw.datetime.minute,
                     &data.raw.datetime.second,
                     &data.raw.lat, &data.raw.lng, &data.second);
        data.isValid = ret == 9 ? true : false;
#ifdef DEBUG_PORT
        DEBUG_PORT.printf("ret:%d %u/%hhu/%hhu %hhu:%hhu:%hhu lat:%f lng:%f [%f]\n",
                          ret,
                          data.raw.datetime.year,
                          data.raw.datetime.month,
                          data.raw.datetime.day,
                          data.raw.datetime.hour,
                          data.raw.datetime.minute,
                          data.raw.datetime.second,
                          data.raw.lat,
                          data.raw.lng,
                          data.second);
#endif
        break;
    case S7XG_GPS_DATA_DD:
        ret = sscanf(buffer, GPS_DD_FORMAT,
                     &data.dd.datetime.year,
                     &data.dd.datetime.month,
                     &data.dd.datetime.day,
                     &data.dd.datetime.hour,
                     &data.dd.datetime.minute,
                     &data.dd.datetime.second,
                     &data.dd.lat, &data.dd.lng, &data.second);

        data.isValid = ret == 9 ? true : false;
#ifdef DEBUG_PORT
        DEBUG_PORT.printf("ret:%d %hu/%hhu/%hhu %hhu:%hhu:%hhu lat:%f lng:%f [%f]\n",
                          ret,
                          data.dd.datetime.year,
                          data.dd.datetime.month,
                          data.dd.datetime.day,
                          data.dd.datetime.hour,
                          data.dd.datetime.minute,
                          data.dd.datetime.second,
                          data.dd.lat,
                          data.dd.lng,
                          data.second);
#endif
        break;
    case S7XG_GPS_DATA_DMS:
        ret = sscanf(buffer, GPS_DMS_FORMAT,
                     &data.dms.datetime.year,
                     &data.dms.datetime.month,
                     &data.dms.datetime.day,
                     &data.dms.datetime.hour,
                     &data.dms.datetime.minute,
                     &data.dms.datetime.second,
                     &data.dms.lat.dd, &data.dms.lat.mm, &data.dms.lat.ss,
                     &data.dms.lng.dd, &data.dms.lng.mm, &data.dms.lng.ss,
                     &data.second);

        data.isValid = ret == 13 ? true : false;
#ifdef DEBUG_PORT
        DEBUG_PORT.printf("ret:%d  %hu/%hhu/%hhu %hhu:%hhu:%hhu dd:%d mm:%d ss:%lf dd:%d mm:%d ss:%lf [%f]\n",
                          ret,
                          data.dms.datetime.year,
                          data.dms.datetime.month,
                          data.dms.datetime.day,
                          data.dms.datetime.hour,
                          data.dms.datetime.minute,
                          data.dms.datetime.second,
                          data.dms.lat.dd, data.dms.lat.mm, data.dms.lat.ss,
                          data.dms.lng.dd, data.dms.lng.mm, data.dms.lng.ss,
                          data.second);
#endif
        break;
    default:
        return S7XG_INVALD;
        break;
    }
    return S7XG_OK;
}

int Acsip::GPSStart(GPSStartMode type, GPSMode mode, GPSSatelliteSys satellite, uint32_t cycle)
{
    int ret  = 0;
    ret = setLevelShift(true);
    if (ret != S7XG_OK) {
        return ret;
    }
    ret = setStart(type);
    if (ret != S7XG_OK) {
        return ret;
    }
    ret = setSatelliteSystem(satellite);
    if (ret != S7XG_OK) {
        return ret;
    }
    ret = setPositioningCycle(cycle);
    if (ret != S7XG_OK) {
        return ret;
    }
    ret = setMode(mode);
    if (ret != S7XG_OK) {
        return ret;
    }
    return ret;
}

int Acsip::GPSStop()
{
    setMode(S7XG_GPS_MODE_IDLE);
    setLevelShift(false);
    return S7XG_OK;
}


/*****************************************
 *          SERVICE FUNCTION
 ****************************************/
bool Acsip::rf_check_available(const char *ptr)
{
    return (strncmp(ptr, "radio_rx", strlen("radio_rx")) == 0);
}

void Acsip::setRFCallback(rf_callback cb)
{
    _rf_callback = cb;
}

int Acsip::service()
{
    int rssi = 0, snr = 0;
    bool rxdone = false;
    char *ptr = NULL;
    char *str = NULL;
    uint8_t serial_buffer[256] = {0};
    int buffer_size = sizeof(serial_buffer);
    int i = 0;
    int offset = 0;

    while (_port->available() && i < buffer_size) {
        serial_buffer[i] = _port->read();
        if (serial_buffer[0] == 0xA && serial_buffer[1] == 0xD &&
                serial_buffer[2] == 0x3E && serial_buffer[3] == 0x3E
                && serial_buffer[i] == 0xA) {
            ptr = strndup((const char *)&serial_buffer[5], i - 6);
            // Serial.printf("Recvicer Done .. [size:%d] -> %s\n", i - 6, ptr);
            rxdone = true;
            break;
        }
        ++i;
    }

    if (!rxdone) {
        return 0;
    }

    if (rf_check_available(ptr) && _rf_callback) {

        char *rf_prt = ptr + 9;
        str = strrchr(rf_prt, 0x20);
        if (str == NULL) {
            goto exit;
        }
        offset = str - rf_prt;
        snr = atoi(str + 1);
        rf_prt[offset] = 0;

        str = strrchr(rf_prt, 0x20);
        if (str == NULL) {
            goto exit;
        }
        offset = str - rf_prt;
        rssi = atoi(str + 1 );
        rf_prt[offset] = 0;

        size_t len;
        hexToString(rf_prt, serial_buffer, strlen(rf_prt), len);
        serial_buffer[len] = 0;
        _rf_callback((char *)serial_buffer, rssi, snr);
    }

exit:
    if (ptr) {
        free(ptr);
    }
    return 0;
}

