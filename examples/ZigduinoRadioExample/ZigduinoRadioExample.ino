/*
Run this sketch on two Zigduinos, open the serial monitor at 9600 baud, and type in stuff
Watch the Rx Zigduino output what you've input into the serial port of the Tx Zigduino
*/

#include <ZigduinoRadio.h>
#define NODE_ID 0x0001  // node id of this node. change it with different boards
#define CHANNEL 26      // check correspond frequency in SpectrumAnalyzer
#define TX_TRY_TIMES 5  // if TX_RETRY is set, pkt_Tx() will try x times before success
#define TX_DO_CARRIER_SENSE 1
#define TX_SOFT_ACK 1
#define TX_SOFT_FCS 1
#define TX_CHECKSUM 1
#define TX_RETRY 1     // pkt_Tx() retransmit packets if failed. TODO: Don't retry 
#define TX_BACKOFF 100  // sleep time of tx retransmission, in ms
#define TX_HEADER_LEN 10
#define TX_INFO_LEN 22
#define TX_CHECKSUM 1
#define RETRY_INTERVAL 1
#define PACKET_TYPE_DATA 0x00
#define PACKET_TYPE_SOFTERR 0xff
#define PACKET_TYPE_RTS 0x10
#define PACKET_TYPE_CTS 0x11
#define PACKET_TYPE_UNDEFINED 0xaa
#define TX_STATUS_IDLE 0x01
#define TX_STATUS_WAIT_CTS 0x02
#define TX_STATUS_SEND_DATA 0x03
#define CTS_TIME_LIMIT 100
#define ERROR_TABLE_SIZE 10
#define ROUTING_MAP_LEN 16
#define ROUTING_MAP_START 10
#define PACKET_TYPE_LOC 9
#define TIME_START 26
#define TIME_LEN 4

uint8_t TxBuffer[128]; // can be used as header and full pkt.
uint8_t RxBuffer[128];
uint8_t softError[10];
uint8_t softACK[5];
char teststr[] = "test x";
uint8_t TX_available; // set to 1 if need a packet delivery, and use need_TX() to check  its value
// here are internal variables, please do not modify them.
uint8_t retry_c;
uint8_t RX_available; // use has_RX() to check its value
uint8_t RX_pkt_len;
uint8_t fcs_failed,check_sum_failed;
uint8_t routingMap[ROUTING_MAP_LEN];
int softErrorCounter = 0;
uint8_t txStatus = TX_STATUS_IDLE;
unsigned long RTSTime = 0;

uint8_t errors[10][3]; 

uint8_t seqCounter = 1;

uint8_t timeTable[256];

uint16_t RTSaddr;
uint8_t doRTS = 0;

unsigned long time;

class softError{
public:
	softError(uint16_t _add, uint8_t _seq){
		address = _add;
		seqNum = _seq;
	}

	uint16_t getAddr(){
		return address;
	}

	uint8_t getSeq(){
		return seqNum;
	}
private:
	uint16_t address;
	uint8_t seqNum;
};
//Error class


// the setup() function is called when Zigduino staets or reset
//header format 0~8 packet type 9 original 10~26 routing map 27 
void setup()
{
	init_header();
	retry_c = 0;
	TX_available = 1;
	RX_available = 0;
	fcs_failed = 0;
	check_sum_failed= 0;
	ZigduinoRadio.begin(CHANNEL,TxBuffer);
	ZigduinoRadio.setParam(phyPanId,(uint16_t)0xABCD );
	ZigduinoRadio.setParam(phyShortAddr,(uint16_t)NODE_ID );
	ZigduinoRadio.setParam(phyCCAMode, (uint8_t)3);
	Serial.begin(9600);

	// register event handlers
	ZigduinoRadio.attachError(errHandle);
	ZigduinoRadio.attachTxDone(onXmitDone);
	ZigduinoRadio.attachReceiveFrame(pkt_Rx);

	for(int i=0;i<ROUTING_MAP_LEN;i++){
		routingMap[i] = 0xff;
	}

}

// the function is always running
void loop()
{
	uint8_t inbyte;
	uint8_t inhigh;
	uint8_t inlow;
	uint8_t tx_suc;
	if(need_TX()){
		if(txStatus == TX_STATUS_IDLE) {
			int rssi = ZigduinoRadio.getRssiNow();
			if(rssi == -91) {
				//        Serial.print("Channel is clear now.");
				txStatus = TX_STATUS_WAIT_CTS;
				RTSTime = millis();
				tx_suc = pkt_Tx(0x0002, teststr, 0, PACKET_TYPE_RTS, 0x01);
			}
			else {
				Serial.print("Channel is too crowd.");
				delay(random(50, 150));
			}
		}
		else if(txStatus == TX_STATUS_SEND_DATA) {
			Serial.println("DATA Sending");
			delay(1000);
			tx_suc = pkt_Tx(0x0002, teststr, seqCounter, PACKET_TYPE_DATA, 0x00);
			seqCounter++;
			txStatus = TX_STATUS_IDLE;
		}
		TX_available = 1;
	}

	/** this is from the original example
*  it reads bytes from your serial input, then transmit it
* if (Serial.available()){
* ZigduinoRadio.beginTransmission();
* Serial.println();
* Serial.print("Tx: ");
* while(Serial.available())
* {
* char c = Serial.read();
* Serial.write(c);
* ZigduinoRadio.write(c);
* }
* Serial.println();
* ZigduinoRadio.endTransmission((uint16_t)0x0001); //0xffff for broadcast
* 
* }
*/

	/** William Added, to handle if cannot get CTS */
	if(txStatus == TX_STATUS_WAIT_CTS) {
		//      Serial.println("Waiting for CTS");
		int time = calcWaitingTime();
		//      Serial.print("time: ");
		//      Serial.println(time);
		if(time > CTS_TIME_LIMIT) {
			txStatus = TX_STATUS_IDLE;
			delay(random(50, 150));
		}
	}

	if(doRTS) {
		tx_suc = pkt_Tx(RTSaddr, teststr, 0, PACKET_TYPE_CTS, 0x00);
		doRTS = 0;
	}

	if(has_RX()){
		Serial.println("Rx packet info");  
		for(uint8_t i=0;i<RX_pkt_len;i++){
			Serial.print("Rx[");
			Serial.print(i);
			Serial.print("]: ");
			Serial.println(RxBuffer[i], HEX);
		}


		if(fcs_failed){
			Serial.println();
			Serial.print("FCS error");
			Serial.println();
		}
		else if(check_sum_failed && TX_CHECKSUM){
			Serial.println();
			Serial.print("Check_sum error");
			Serial.println();
		}
		else{
			Serial.println();
			Serial.print("Rx: ");
			for(uint8_t i=0;i<RX_pkt_len;i++){
				inbyte = RxBuffer[i];
				if(printable(inbyte)){
					Serial.write(inbyte);
				}
				else if(check_sum_failed && TX_CHECKSUM){
					Serial.println();
					Serial.print("Check_sum error");
					Serial.println();
				}
				else{
					Serial.print(".");
				}

				/* for printing bytes in hex
		Serial.print("[");
		inhigh = inbyte/16;
		inlow = inbyte%16;
		Serial.write(inhigh>9?'A'+inhigh-10:'0'+inhigh);
		Serial.write(inlow>9?'A'+inlow-10:'0'+inlow);
		Serial.print("] ");
		*/
			}
			Serial.println();
			Serial.print("LQI: ");
			Serial.print(ZigduinoRadio.getLqi(), 10);
			Serial.print(", RSSI: ");
			Serial.print(ZigduinoRadio.getLastRssi(), 10);
			Serial.print(" dBm, ED: ");
			Serial.print(ZigduinoRadio.getLastEd(), 10);
			Serial.println("dBm");


		}
	}
	//TODO
	softErrorCounter++;
	if(softErrorCounter >= RETRY_INTERVAL){
		softErrorCounter = 0;
		sendSoftError();
	}
	delay(100);
}

void init_header(){
	if(1){ //TX_SOFT_ACK){
		TxBuffer[0] = 0x61; // ack required
	}
	else{
		TxBuffer[0] = 0x41; // no ack required
	}
	TxBuffer[1] = 0x88;
	TxBuffer[2] = 0;    // seqence number
	TxBuffer[3] = 0xCD;
	TxBuffer[4] = 0xAB; //Pan Id
	TxBuffer[5] = 0x01; //dest address low byte
	TxBuffer[6] = 0x00; //dest address hight byte
	TxBuffer[7] = NODE_ID & 0xff; //source address low byte
	TxBuffer[8] = NODE_ID >> 8; //source address hight byre
	// TxBuffer[PACKET_TYPE_LOC] = PACKET_TYPE_DATA;
	TxBuffer[PACKET_TYPE_LOC] = PACKET_TYPE_UNDEFINED;

	softACK[0] = 0x42;
	softACK[1] = 0x88;

	//Define softerror
	softError[0] = 0x41;
	softError[1] = 0x88;
	softError[2] = 0;    // seqence number
	softError[3] = 0xCD;
	softError[4] = 0xAB; //Pan Id

	for(int i = ROUTING_MAP_START;i<ROUTING_MAP_START+ROUTING_MAP_LEN;i++){
		TxBuffer[i] = 0xff;
	}
}

/* the packet transfer function
* parameters :
*     dst_addr : destnation address, set 0xffff for broadcast. Pkt will be dropped if  NODE_ID != dst addr in packet header.
*     msg : a null-terminated string to be sent. This function won't send the '\0'. Pkt  header will be paded in this function.
* return values : this function returns 0 if success, not 0 if problems (no ACK after  some retries).
*
* This function set the headers to original msg then transmit it according your policy  setting.
* The most important function is ZigduinoRadio.txFrame(TxBuffer, pkt_len) which transmit  pkt_len bytes from TxBuffer, where TxBuffer includes header.
* Note that onXmitDone(radio_tx_done_t x) is right called after txFrame(), you can do  some status checking at there.
*
* Feel free to modify this function if needed.
*/

uint8_t pkt_Tx(uint16_t dst_addr, char* msg, uint8_t seqNum, uint8_t pktType, uint8_t needTime){
	uint16_t fcs,check_sum;
	uint8_t i;
	uint8_t pkt_len;
	uint8_t tmp_byte;
	radio_cca_t cca = RADIO_CCA_FREE;
	int8_t rssi;

	// process the dst addr, 0xffff for broadcast
	TxBuffer[5] = dst_addr & 0x00ff;
	TxBuffer[6] = dst_addr >> 8;
	tmp_byte = TxBuffer[0];
	if(dst_addr == 0xffff){ // broadcast, no ACK required
		TxBuffer[0] = 0x41;
	}
	TxBuffer[2] = seqNum;
	TxBuffer[PACKET_TYPE_LOC] = pktType;

	if(pktType == PACKET_TYPE_DATA) {
		// Serial.println("Routeing code");
		// delay(1000);
		for(i = 0;i<ROUTING_MAP_LEN;i++){
			TxBuffer[i+ROUTING_MAP_START] = routingMap[i];
		}
		if(needTime == 0x01){
			time = millis();
			TxBuffer[TIME_START] = time >> 24;
			TxBuffer[TIME_START+1] = (time >> 16) & 0x000000ff;
			TxBuffer[TIME_START+2] = (time >> 8) & 0x000000ff;
			TxBuffer[TIME_START+3] = time & 0x000000ff;
		}

		// fill the payload
		for(i = 0; msg[i] != '\0'; i++){
			TxBuffer[TX_HEADER_LEN + TX_INFO_LEN + i] = msg[i];
		}
		pkt_len = TX_HEADER_LEN + TX_INFO_LEN + i;
	}
	else {
		pkt_len = TX_HEADER_LEN;
	}


	/*
Serial.println("");
Serial.print("pktType: ");
Serial.println(pktType);
*/


	// fill the software fcs
	if(TX_SOFT_FCS){
		fcs = cal_fcs(TxBuffer, pkt_len);
		TxBuffer[pkt_len++] = fcs & 0xff;
		TxBuffer[pkt_len++] = fcs >> 8;
	}
	// fill the check_sum in TX_CHECKSUM_POS
	if(TX_CHECKSUM){
		if(pkt_len%2==1){
			pkt_len ++;
		}
		check_sum = cal_check_sum(TxBuffer, pkt_len);
		TxBuffer[pkt_len++] = check_sum & 0xff;
		TxBuffer[pkt_len++] = check_sum >> 8;
	}
	// hardware fcs, no use
	pkt_len += 2;
	// transmit the packet
	// retry_c will be set to RETRY_TIMES by onXmitDone() if packet send successfully
	if(TX_RETRY){
		for(retry_c = 0; retry_c < TX_TRY_TIMES; retry_c++){
			if(TX_DO_CARRIER_SENSE){
				//        cca = ZigduinoRadio.doCca();
				rssi = ZigduinoRadio.getRssiNow();
				//        if(cca == RADIO_CCA_FREE)
				if(rssi == -91){
					ZigduinoRadio.txFrame(TxBuffer, pkt_len);

					Serial.println();  
					for(i=0; i<pkt_len; i++) {
						Serial.print("Tx[");
						Serial.print(i);
						Serial.print("]: ");
						Serial.println(TxBuffer[i], HEX);
					}
					// delay(999999);
				}
				else{
					Serial.print("ca fail with rssi = ");
					Serial.println(rssi);
				}
			}
			else{
				ZigduinoRadio.txFrame(TxBuffer, pkt_len);

				Serial.println();  
				for(i=0; i<pkt_len; i++) {
					Serial.print("Tx[");
					Serial.print(i);
					Serial.print("]: ");
					Serial.println(TxBuffer[i], HEX);
				}
				// delay(999999);
			}
			delay(TX_BACKOFF);
		}
		retry_c--; // extra 1 by for loop, if tx success retry_c == TX_TRY_TIMES
	}
	else{
		if(TX_DO_CARRIER_SENSE){
			//      cca = ZigduinoRadio.doCca();
			rssi = ZigduinoRadio.getRssiNow();
			//      if(cca == RADIO_CCA_FREE)
			if(rssi == -91){
				ZigduinoRadio.txFrame(TxBuffer, pkt_len);
				Serial.println();  
				for(i=0; i<pkt_len; i++) {
					Serial.print("Tx[");
					Serial.print(i);
					Serial.print("]: ");
					Serial.println(TxBuffer[i], HEX);
				}

			}
			else{
				Serial.print("ca fail with rssi = ");
				Serial.println(rssi);
			}
		}
		else{
			ZigduinoRadio.txFrame(TxBuffer, pkt_len);
			Serial.println();  
			for(i=0; i<pkt_len; i++) {
				Serial.print("Tx[");
				Serial.print(i);
				Serial.print("]: ");
				Serial.println(TxBuffer[i], HEX);
			}
		}
	}
	TxBuffer[0] = tmp_byte;

	return retry_c == TX_TRY_TIMES;
}

/* the event handler which is called when Zigduino got a new packet
* don't call this function yourself
* do sanity checks in this function, and set RX_available to 1 at the end
* the crc_fail parameter is a fake, please ignore it
*/
uint8_t* pkt_Rx(uint8_t len, uint8_t* frm, uint8_t lqi, uint8_t crc_fail){
	uint16_t fcs,check_sum;

	/** William added, for debug purpose */
	RX_pkt_len = len;
	for(uint8_t i=0; i < len; i++){
		RxBuffer[i] = frm[i];
	}
	// This function set RX_available = 1 at the end of this function.
	// You can use has_RX() to check if has packet received.

	// Software packet filter :
	// Check pkt_len, dst_addr, FCS. Drop pkt if fails
	if(len < TX_HEADER_LEN){
		return RxBuffer;
	}
	// keep the pkt only if broadcast or dst==me
	if( (frm[5] != NODE_ID&0xff) || (frm[6] != NODE_ID>>8) ){
		if(frm[5] != 0xff || frm[6] != 0xff){
			return RxBuffer;
		}
	}

	// check fcs first, drop pkt if failed
	if(TX_SOFT_FCS){
		if(len%2 == 0){
			fcs = cal_fcs(frm, len-4);
		}
		else{
			fcs = cal_fcs(frm, len-5);
		}

		//TODO: add checksum
		if(fcs != 0x0000){
			fcs_failed = 1;
			RX_available = 1;
			//TODO: if the packet is wrong push the info to error vector
			for(int i = 0;i<ERROR_TABLE_SIZE;i++){
				if(errors[i][0] == 0xff){
					errors[i][0] = frm[5];
					errors[i][1] = frm[6];
					errors[i][2] = frm[2];
				}
			}
			//errors.add(new errors(((uint16_t)frm[6] << 8) | frm[5]), frm[2]);

			return RxBuffer;
		}
		else{
			fcs_failed = 0;
		}
	}
	if(TX_CHECKSUM){
		check_sum = cal_check_sum(frm, len-2);
		if(check_sum!= 0x0000){
			//Serial.println("Check_sum Failed");
			check_sum_failed = 1;
			RX_available = 1;
			for(int i = 0;i<ERROR_TABLE_SIZE;i++){
				if(errors[i][0] == 0xff){
					errors[i][0] = frm[5];
					errors[i][1] = frm[6];
					errors[i][2] = frm[2];
				}
			}
			return RxBuffer;
		}
		else{
			check_sum_failed = 0;
			//Serial.println("Check_sum Success");
		}
	}

	//TODO: load routing map
	if(frm[PACKET_TYPE_LOC] == 0x00){
		for(int i = ROUTING_MAP_START;i<ROUTING_MAP_LEN;i++){
			routingMap[i] = frm[i]; 
			//TODO: if get correct packet, erase it from error vector
			//and renew time info
			for(int j =0;j<ERROR_TABLE_SIZE;j++){
				if(frm[2] == errors[j][2])
					errors[j][0] = 0xff;
			}


		}
	}
	if(frm[PACKET_TYPE_LOC] == 0xff){
		if(need_TX()){
			pkt_Tx(((uint16_t)frm[8] << 8) | frm[7], teststr, frm[2], PACKET_TYPE_SOFTERR, 0x00);
			TX_available = 1;
		}
	}

	if(frm[PACKET_TYPE_LOC] == PACKET_TYPE_CTS) {
		if(txStatus == TX_STATUS_WAIT_CTS) {
			int time = calcWaitingTime();
			if(time > CTS_TIME_LIMIT) {
				txStatus = TX_STATUS_IDLE;
				delay(random(50, 150));
			}
			else {
				txStatus = TX_STATUS_SEND_DATA;
			}
		}
	}
	else if(RxBuffer[PACKET_TYPE_LOC] == PACKET_TYPE_RTS) {
		RTSaddr = 0;
		uint16_t high = RxBuffer[7];
		uint16_t low = RxBuffer[8];
		RTSaddr |= high << 8;
		RTSaddr |= low;
		doRTS = 1;
		//pkt_Tx(addr, teststr, 0, PACKET_TYPE_CTS, 0x00);
	}

	if(len >= TX_HEADER_LEN+TX_INFO_LEN){//TODO: forwarding
		for(int i = ROUTING_MAP_START ;i<ROUTING_MAP_START+ROUTING_MAP_LEN;i++){
			if(frm[i] == 0xff){
				frm[i] = NODE_ID & 0xff;
				break;
			}
		}
	}

	// send software ack
	if(0){ // frm[0] & 0x20){
		softACK[2] = frm[2];
		ZigduinoRadio.txFrame(softACK, 5);
	}
	//TODO: if get correct packet, erase it from error vector
	//and renew time info
	for(int j =0;j<ERROR_TABLE_SIZE;j++){
		if(frm[2] == errors[j][2])
		errors[j][0] = 0xff;
	}

	// now all checks are passed, copy out the received packet
	for(uint8_t i=0; i < len; i++){
		RxBuffer[i] = frm[i];
	}

	RX_pkt_len = len;
	RX_available = 1;

	return RxBuffer;
}

//TODO: send softerror to every error source
int sendSoftError(){
	for(int i = 0;i<ERROR_TABLE_SIZE;i++){
		if(errors[i][2] != 0xff)
		continue;
		softError[2] = errors[i][2];
		softError[5] = errors[i][0];
		softError[6] = errors[i][1];
		softError[7] = NODE_ID & 0xff; //source address low byte
		softError[8] = NODE_ID >> 8; //source address hight byre
		softError[9] = 0xff;
		ZigduinoRadio.txFrame(softError, 10);
	}
}

uint16_t cal_check_sum(uint8_t* frm, uint8_t len){
	uint16_t sum = 0x00,to_add;
	for(uint8_t i = 0; i < len; i += 2){
		to_add = (uint16_t)frm[i];
		if(i+1<len) 
		to_add += (uint16_t)(frm[i+1]<<8);
		sum = sum_two(sum,to_add);
	}
	return ~sum;  // return the one's complement, add it with the sum and get 0 if the packet is not corrupted
}

// this function returns TX_available and reset it to 0
uint8_t need_TX(){
	if(TX_available){
		TX_available = 0;
		return 1;
	}
	return 0;
}


// this function returns RX_available and reset it to 0
uint8_t has_RX(){
	if(RX_available){
		RX_available = 0;
		return 1;
	}
	return 0;
}

// calculate error detecting code
// choose an algorithm for it
uint16_t cal_fcs(uint8_t* frm, uint8_t len){
	Serial.print("FCS Len: ");
	Serial.println(len);
	uint16_t fcs = frm[0];
	for(uint8_t i = 1; i < len; i += 1){
		fcs ^= frm[i];
	}
	return fcs;
}

uint8_t printable(uint8_t in){
	if(32 <= in && in <= 126){
		return 1;
	}
	return 0;
}

void errHandle(radio_error_t err)
{
	Serial.println();
	Serial.print("Error: ");
	Serial.print((uint8_t)err, 10);
	Serial.println();
}

// this function is called after the packet transmit function
void onXmitDone(radio_tx_done_t x)
{
	Serial.println();
	Serial.print("TxDone: ");
	Serial.print((uint8_t)x, 10);
	if(x==TX_NO_ACK){
		Serial.print(" NO ACK ");
	}
	else if(x==TX_OK){
		Serial.print("(OK)");
		retry_c = TX_TRY_TIMES;
	}
	else if(x==TX_CCA_FAIL){ // not implemented
		Serial.print("(CS busy)");
	}
	Serial.println();
}

unsigned long calcWaitingTime() {
	return millis() - RTSTime;
}

uint16_t sum_two(uint16_t a, uint16_t b){
	uint32_t sum = (uint32_t)a + (uint32_t)b;
	uint32_t carry = sum>>16;
	if(carry == 0x01){	// case that there's carry bit
		sum = sum_two(sum,carry);
	}
	return (uint16_t) sum & 0xffff;
}


