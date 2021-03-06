/*-
 * Copyright (c) 2012 Darran Hunt (darran [at] hunt dot net dot nz)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL
 * THE CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* Release history
 *
 * Version  Date         Description
 * 0.1      25-Mar-2012  First release.
 * 0.2      09-Apr-2012  Added features to support http servers.
 *                       - added an httpserver.ino example.
 *                       - added sendChunk() and sendChunkln() to send chunked HTTP bodies.
 *                       - added terminal() method for simple terminal access via debug stream
 *                       - replaced getFreeMemory() with simpler version that works with 0 bytes
 *                       - turned peek buffer into a circular buffer to fix bug with detecting
 *                         *CLOS* and *OPEN* after a partial match.
 *                       - Added new TCP connection detection via *OPEN* match from available().
 *                         isConnected() can now be polled until a client connects.
 *                       - made the match() function public, handy for matching text in a stream.
 *                       - Added a getProtocol() function to get current set of protocols.
 * 0.3      21-Apr-2012  Added createAdhocNetwork() to create an Ad Hoc WiFi network.
 * 			 Optimised the setopt() and getopt() function so they handle
 * 			 integer conversions and refactored all of the set and get functions.
 * 			 Added a multMatch_P() function to match serial data against multiple
 * 			 progmem strings.
 * 			 Added failure detection to the join() function to quickly detect
 * 			 a failure rather than relying on a timeout.
 * 			 Added setJoin() and getJoin() function for access to the wlan join parameter.
 * 			 Refactored getres() to use the new multiMatch_P() function.
 * 0.4		20-Dec-2012 By Felix Bonowski. Added an easy setup function. Lots of minor changes.
 *
 */

#ifndef _WIFLYHQ_H_
#define _WIFLYHQ_H_

#include <Arduino.h>
#include <Stream.h>
#include <avr/pgmspace.h>
#include <IPAddress.h>

/* IP Protocol bits */
#define WIFLY_PROTOCOL_UDP		0x01
#define WIFLY_PROTOCOL_TCP		0x02
#define WIFLY_PROTOCOL_SECURE		0x04
#define WIFLY_PROTOCOL_TCP_CLIENT	0x08
#define WIFLY_PROTOCOL_HTTP		0x10	/* HTTP Client mode */
#define WIFLY_PROTOCOL_RAW		0x20
#define WIFLY_PROTOCOL_SMTP		0x40

/* IP Flag bits */
#define WIFLY_FLAG_TCP_KEEP		0x01	/* Keep TCP connection alive when wifi lost */
#define WIFLY_FLAG_TCP_NODELAY		0x02
#define WIFLY_FLAG_TCP_RETRY		0x04
#define WIFLY_FLAG_UDP_RETRY		0x08
#define WIFLY_FLAG_DNS_CACHING		0x10
#define WIFLY_FLAG_ARP_CACHING		0x20
#define WIFLY_FLAG_UDP_AUTO_PAIR	0x40
#define WIFLY_FLAG_ADD_TIMESTAMP	0x80

/* UART mode bits */
#define WIFLY_UART_MODE_NOECHO		0x01
#define WIFLY_UART_MODE_DATA_TRIGGER	0x02
#define WIFLY_UART_MODE_SLEEP_RX_BREAK	0x08
#define WIFLY_UART_MODE_RX_BUFFER	0x10

/* DHCP modes */
#define WIFLY_DHCP_MODE_OFF		0x00	/* No DHCP, static IP mode */
#define WIFLY_DHCP_MODE_ON		0x01	/* get IP, Gateway, and DNS from AP */
#define WIFLY_DHCP_MODE_AUTOIP		0x02	/* Used with Adhoc networks */
#define WIFLY_DHCP_MODE_CACHE		0x03	/* Use previous DHCP address based on lease */
#define WIFLY_DHCP_MODE_SERVER		0x04	/* Server DHCP IP addresses? */

/* WLAN Join modes */
#define WIFLY_WLAN_JOIN_MANUAL		0x00	/* Don't auto-join a network */
#define WIFLY_WLAN_JOIN_AUTO		0x01	/* Auto-join network set in SSID, passkey, and channel. */
#define WIFLY_WLAN_JOIN_ANY		0x02	/* Ignore SSID and join strongest network using passkey. */
#define WIFLY_WLAN_JOIN_ADHOC		0x04	/* Create an Adhoc network using SSID, Channel, IP and NetMask */

#define WIFLY_DEFAULT_TIMEOUT		500	/* 500 milliseconds */

#define WIFLY_MODE_WPA			0	
#define WIFLY_MODE_WEP			1

//Debug out

#ifndef TRACE
#define TRACE(x) do { if (DEBUG) Serial.print( x); } while (0)
#endif
#ifndef TRACELN
#define TRACELN(x) do { if (DEBUG) Serial.println( x); } while (0)
#endif


class WFDebug : public Stream {
public:
    WFDebug();
    void begin(Stream *debugPrint);

    virtual size_t write(uint8_t byte);
    virtual int read() { return debug->read(); }
    virtual int available() { return debug->available(); }
    virtual void flush() { return debug->flush(); }
    virtual int peek() { return debug->peek(); }

    using Print::write;
private:
    Stream *debug;
};

class WiFly : public Stream {
public:
    WiFly();
	/// a complete setup of all functionality for UDP. Templated with the type of the Serial you want to use.
	template <class serialType> void setupForUDP(
		serialType* wiFlySerial,
		const uint32_t newSerialSpeed,
		const bool tryOtherSpeeds,	///< should we try some other baudrates if the currently selected one fails?
		const char* SSID,
		const char* password,
		const char* deviceID,      ///< for identifacation in the network
		const char* localIP,       ///< a string with numbers, if 0, we will use dhcp to get an ip
		const uint16_t localPort,
		const char* remoteHost,
		const uint16_t remotePort,
		const bool printTrace	///< show debug information on Serial
	){
		bool DEBUG=printTrace;
	  TRACE((F("Free memory: ")));
	  TRACELN((this->getFreeMemory()));

	  boolean saveAndReboot=false;
	  char buf[32];

	  //try out some different serial speeds until we find one that is working...
	  const uint32_t serialSpeeds[]={
		newSerialSpeed,9600, 19200, 38400, 57600, 115200        };

	  int speedIndex=0;
	  int maxSpeedIndex;
	  if(tryOtherSpeeds){
		maxSpeedIndex=6;
	  }
	  else{ 
		maxSpeedIndex=1;
	  }; 
	  boolean baudrateFound=false;
	  TRACELN(F("Trying to connect to this"));

	  for (speedIndex=0;speedIndex<maxSpeedIndex;speedIndex++){
		TRACE(F("Connecting to this using Baudrate:"));
		TRACELN(serialSpeeds[speedIndex]);
		wiFlySerial->begin(serialSpeeds[speedIndex]);
		if(this->begin(wiFlySerial, &Serial)){
		  baudrateFound=true;
		  break;
		}
	  }

	  if(!baudrateFound){
		TRACELN(F("Could not find working Baud rate to connect to this"));
		return;
	  }
	  else{
		TRACE(F("Working Baud rate is "));
		TRACELN(serialSpeeds[speedIndex]);
	  }

	  this->startCommand();  //made this public to avoid the annoing waiting times
	  if(serialSpeeds[speedIndex]!=newSerialSpeed){
		TRACE(F("Setting new baud rate to "));
		TRACELN(newSerialSpeed);
		//set the new this baud rate
		this->setBaud(newSerialSpeed);
		TRACE(F("Saving this config."));
		this->save();
		TRACE(F("Rebooting this->.."));
		this->reboot();
		wiFlySerial->begin(newSerialSpeed);
		saveAndReboot=false;
	  }
	  //set

	  this->getDeviceID(buf, sizeof(buf));
	  if(strcmp(buf, deviceID) != 0){
		TRACE(F("Changing device ID to "));
		TRACELN(deviceID);
		this->setDeviceID(deviceID);
		saveAndReboot=true;
	  }

	  //setup dhcp or an ip..
	  if(localIP==0){
		TRACELN(F("No fixed IP provided"));
		if(this->getDHCPMode()!=WIFLY_DHCP_MODE_ON){
		  TRACELN(F("Enabling DHCP"));

		  this->enableDHCP();
		  saveAndReboot=true;
		}
	  }
	  else{
		if(this->getDHCPMode()!=WIFLY_DHCP_MODE_OFF){
		  TRACE(F("Disabling DHCP; setting IP to "));
		  TRACELN(localIP);
		  this->disableDHCP();
		  this->setIP(localIP);
		  saveAndReboot=true;
		}
		//set ip only if necessary...
		this->getIP(buf,sizeof(buf));  
		if(strcmp(buf, localIP) != 0){
		  TRACE(F("Setting IP to "));
		  TRACELN(localIP);
		  this->setIP(localIP);
		  saveAndReboot=true;
		}
	  }

	  //receive packets at this port...
	  if(this->getPort()!=localPort){
		TRACE(F("Setting listen Port to "));
		TRACELN(localPort);
		this->setPort( localPort);	// Send UPD packets to this server and port
		saveAndReboot=true;
	  }

	  //2 millis seems to be the minimum for 9600 baud
	  if (this->getFlushTimeout() != 2) {
		TRACE(F("Setting Flush timeout to 2ms "));
		this->setFlushTimeout(2);
		saveAndReboot=true;
	  }

	  //set SSID only if necessary...
	  this->getSSID(buf,sizeof(buf));  
	  if(strcmp(buf, SSID) != 0){
		TRACE(F("Setting SSID to "));
		TRACELN((SSID));
		this->setSSID(SSID);

		TRACE(F("Setting Wifi Passwd to "));
		TRACELN((password));
		this->setPassphrase(password);
		saveAndReboot=true;
	  }

	  this->save();
	  this->finishCommand();
	  if(saveAndReboot==true){
		this->reboot();
		wiFlySerial->begin(newSerialSpeed);
	  }


	  this->startCommand();
	  //set SSID only if necessary...
	  this->getSSID(buf,sizeof(buf));  
	  if(strcmp(buf, SSID) != 0){
		this->setSSID(SSID);
		this->setPassphrase(password);
	  }


	  /* Join wifi network if not already associated */
	  if (!this->isAssociated()) {
		//  if(true){
		/* Setup the this to connect to a wifi network */
		TRACELN(F("Joining network"));
		this->setSSID(SSID);

		if (this->join()) {
		  //  this->setopt("set w a ", 1);
		  //if (this->join(SSID, password, localIP==0, this_MODE_WEP)) {
		  TRACELN(F("Joined wifi network"));
		} 
		else {
		  TRACELN(F("Failed to join wifi network"));
		}
	  } 
	  else {
		TRACELN(F("Already joined network"));
	  }

	  //enable auto join
	  this->setJoin(WIFLY_WLAN_JOIN_AUTO);
	  /* Setup for UDP packets, sent automatically */
	  this->setIpProtocol(WIFLY_PROTOCOL_UDP);
	  this->setHost(remoteHost, remotePort);	// Send UPD packets to this server and port

		TRACE("MAC: ");
	  TRACELN(this->getMAC(buf, sizeof(buf)));
	  TRACE("IP: ");
	  TRACELN(this->getIP(buf, sizeof(buf)));
	  TRACE("Netmask: ");
	  TRACELN(this->getNetmask(buf, sizeof(buf)));
	  TRACE("Gateway: ");
	  TRACELN(this->getGateway(buf, sizeof(buf)));
	  TRACE("Listen Port:");
	  TRACELN(this->getPort());
	  this->finishCommand();
	  TRACELN(F("this setup finished"));

	};
	
	void printStatusInfo(){
	  this->startCommand();
	  char buf[32];
	  /* Ping the gateway */
	  this->getGateway(buf, sizeof(buf));

	  Serial.print("ping ");
	  Serial.print(buf);
	  Serial.print(" ... ");
	  if (this->ping(buf)) {
		Serial.println("ok");
	  } 
	  else {
		Serial.println("failed");
	  }

	  Serial.print("ping google.com ... ");
	  if (this->ping("google.com")) {
		Serial.println("ok");
	  } 
	  else {
		Serial.println("failed");
	  }

	  Serial.print("MAC: ");
	  Serial.println(this->getMAC(buf, sizeof(buf)));
	  Serial.print("IP: ");
	  Serial.println(this->getIP(buf, sizeof(buf)));
	  Serial.print("Netmask: ");
	  Serial.println(this->getNetmask(buf, sizeof(buf)));
	  Serial.print("Gateway: ");
	  Serial.println(this->getGateway(buf, sizeof(buf)));
	  Serial.print("Listen Port:");
	  Serial.println(this->getPort());
	  this->finishCommand();

	}
	
	
    boolean begin(Stream *serialdev, Stream *debugPrint = NULL);
    
    char *getSSID(char *buf, int size);
    uint8_t getJoin();
    char *getDeviceID(char *buf, int size);    
    char *getIP(char *buf, int size);
    uint16_t getPort();
    char *getNetmask(char *buf, int size);
    char *getGateway(char *buf, int size);
    char *getDNS(char *buf, int size);
    char *getMAC(char *buf, int size);
    int8_t getDHCPMode();
    uint32_t getRate();
    uint8_t getTxPower();

    uint16_t getConnection();
    int8_t getRSSI();

    bool setJoin(uint8_t join);
    boolean setDeviceID(const char *buf);
    boolean setBaud(uint32_t baud);
    uint32_t getBaud();
    uint8_t getUartMode();
    uint8_t getIpFlags();
    uint8_t getProtocol();

    uint8_t getFlushChar();
    uint16_t getFlushSize();
    uint16_t getFlushTimeout();

    char *getHostIP(char *buf, int size);
    uint16_t getHostPort();

    boolean setSSID(const char *buf);
    boolean setIP(const char *buf);
    boolean setIP(const __FlashStringHelper *buf);
    boolean setPort(const uint16_t port);
    boolean setNetmask(const char *buf);
    boolean setNetmask(const __FlashStringHelper *buf);
    boolean setGateway(const char *buf);
    boolean setDNS(const char *buf);
    boolean setChannel(uint8_t channel);
    boolean setKey(const char *buf);
    boolean setPassphrase(const char *buf);
    boolean setSpaceReplace(const char *buf);
    boolean setDHCP(const uint8_t mode);
    boolean setRate(uint32_t rate);
    boolean setTxPower(uint8_t dBm);

    boolean setHostIP(const char *buf);
    boolean setHostIP(const __FlashStringHelper *buf);
    boolean setHostPort(const uint16_t port);
    boolean setHost(const char *buf, uint16_t port);

    boolean setProtocol(const uint8_t protocol);
    boolean setIpProtocol(const uint8_t protocol);	/* obsolete */
    boolean setIpFlags(const uint8_t flags);
    boolean setUartMode(const uint8_t mode);

    boolean setBroadcastInterval(const uint8_t seconds);

    boolean setTimeAddress(const char *buf);
    boolean setTimePort(const uint16_t port);
    boolean setTimezone(const uint8_t zone);
    boolean setTimeEnable(const uint16_t enable);

    boolean setAdhocBeacon(const uint16_t msecs);
    boolean setAdhocProbe(const uint16_t secs);
    uint16_t getAdhocBeacon();
    uint16_t getAdhocProbe();
    uint16_t getAdhocReboot();

    boolean setFlushTimeout(const uint16_t timeout);
    boolean setFlushChar(const char flushChar);
    boolean setFlushSize(uint16_t size);
    boolean enableDataTrigger(const uint16_t flushtime=10, const char flushChar=0, const uint16_t flushSize=64);
    boolean disableDataTrigger();
    boolean enableUdpAutoPair();
    boolean disableUdpAutoPair();

    boolean setIOFunc(const uint8_t func);

    char *getTime(char *buf, int size);
    uint32_t getUptime();
    uint8_t getTimezone();
    uint32_t getRTC();

    bool getHostByName(const char *hostname, char *buf, int size);
    boolean ping(const char *host);

    boolean enableDHCP();
    boolean disableDHCP();
    
    boolean createAdhocNetwork(const char *ssid, uint8_t channel);
    boolean join(const char *ssid, uint16_t timeout=20000);
    boolean join(uint16_t timeout=20000);
    boolean join(const char *ssid, const char *password, bool dhcp=true, uint8_t mode=WIFLY_MODE_WPA, uint16_t timeout=20000);
    boolean leave();
    boolean isAssociated();

    boolean save();
    boolean reboot();
    boolean factoryRestore();

    boolean sendto(const uint8_t *data, uint16_t size, const char *host, uint16_t port);
    boolean sendto(const uint8_t *data, uint16_t size, IPAddress host, uint16_t port);
    boolean sendto(const char *data, const char *host, uint16_t port);
    boolean sendto(const char *data, IPAddress host, uint16_t port);
    boolean sendto(const __FlashStringHelper *data, const char *host, uint16_t port);
    boolean sendto(const __FlashStringHelper *data, IPAddress host, uint16_t port);

    void enableHostRestore();
    void disableHostRestore();

    boolean open(const char *addr, int port=80, boolean block=true);
    boolean open(IPAddress addr, int port=80, boolean block=true);
    boolean close();
    boolean openComplete();
    boolean isConnected();
    boolean isInCommandMode();
    
    virtual size_t write(uint8_t byte);
    virtual int read();
    virtual int readBufTimeout(char* buf, int size, uint16_t timeout=WIFLY_DEFAULT_TIMEOUT);	///< read data into the buffer until it is full or the next character takes more than timeout millis to arrive
    virtual int available();
    virtual void flush();
    virtual int peek();

    char *iptoa(IPAddress addr, char *buf, int size);
    IPAddress atoip(char *buf);
    boolean isDotQuad(const char *addr);

    void sendChunk(const char *str);
    void sendChunk(const __FlashStringHelper *str);
    void sendChunkln(const char *str);
    void sendChunkln(const __FlashStringHelper *str);
    void sendChunkln(void);

    int getFreeMemory();
    void terminal();
  
    using Print::write;

    void dbgBegin(int size=256);
    void dbgDump();
    void dbgEnd();
    boolean debugOn;

    boolean match(const char *str, uint16_t timeout=WIFLY_DEFAULT_TIMEOUT);
    boolean match(const __FlashStringHelper *str, uint16_t timeout=WIFLY_DEFAULT_TIMEOUT);
    int multiMatch_P(uint16_t timeout, uint8_t count, ...);
    int gets(char *buf, int size, uint16_t timeout=WIFLY_DEFAULT_TIMEOUT);
    int getsTerm(char *buf, int size, char term, uint16_t timeout=WIFLY_DEFAULT_TIMEOUT);
    void flushRx(int timeout=WIFLY_DEFAULT_TIMEOUT);

    boolean setFtpDefaults(void);
    boolean setFtpAddress(const char *addr);
    boolean setFtpPort(uint16_t port);
    boolean setFtpDirectory(const char *dir);
    boolean setFtpUser(const char *user);
    boolean setFtpPassword(const char *password);
    boolean setFtpFilename(const char *filename);
    boolean setFtpTimer(uint16_t msecs);
    boolean setFtpMode(uint8_t mode);

    boolean ftpGet(
	const char *addr,
	const char *dir,
	const char *user,
	const char *password,
	const char *filename);
    boolean startCommand();
    boolean finishCommand();
	boolean setopt(const prog_char *cmd, const char *buf=NULL, const __FlashStringHelper *buf_P=NULL);
    boolean setopt(const prog_char *opt, const uint32_t value, uint8_t base=DEC);
  private:
    void init(void);

    void dump(const char *str);

    boolean sendto(
	const uint8_t *data,
	uint16_t size,
	const __FlashStringHelper *flashData,
	const char *host,
	uint16_t port);

    boolean match_P(const prog_char *str, uint16_t timeout=WIFLY_DEFAULT_TIMEOUT);
    int8_t multiMatch_P(const prog_char *str[], uint8_t count, uint16_t timeout=WIFLY_DEFAULT_TIMEOUT);

    void send_P(const prog_char *str);
    void send(const char *str);
    void send(const char ch);
    boolean enterCommandMode();
    boolean exitCommandMode();
    boolean setPrompt();
    boolean getPrompt(uint16_t timeout=WIFLY_DEFAULT_TIMEOUT);
    boolean checkPrompt(const char *str);
    int getResponse(char *buf, int size, uint16_t timeout=WIFLY_DEFAULT_TIMEOUT);
    boolean readTimeout(char *ch, uint16_t timeout=WIFLY_DEFAULT_TIMEOUT);

    char *getopt(int opt, char *buf, int size);
    uint32_t getopt(int opt, uint8_t base=DEC);

    boolean getres(char *buf, int size);

    boolean checkStream(const prog_char *str, boolean peeked);
    boolean checkClose(boolean peeked);
    boolean checkOpen(boolean peeked);

    boolean hide();

    boolean inCommandMode;
    int  exitCommand;
    boolean dhcp;
    bool restoreHost;
    bool restoreHostStored;
    char lastHost[32];
    uint16_t lastPort;

    boolean tcpMode;
    boolean udpAutoPair;

    boolean connected;
    boolean connecting;
    struct {
	uint8_t tcp;
	uint8_t assoc;
	uint8_t authen;
	uint8_t dnsServer;
	uint8_t dnsFound;
	uint8_t channel;
    } status;

    Stream *serial;	/* Serial interface to WiFly */
    
    WFDebug debug;	/* Internal debug channel. */

    /*  for dbgDump() */
    char *dbgBuf;
    int dbgInd;
    int dbgMax;
};

#endif
