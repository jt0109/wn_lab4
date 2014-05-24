/*

Run this sketch on two Zigduinos, open the serial monitor at 9600 baud, and type in stuff
Watch the Rx Zigduino output what you've input into the serial port of the Tx Zigduino

*/

#include <ZigduinoRadio.h>

#define NODE_ID 0x0018  // node id of this node. change it with different boards
#define CHANNEL 26      // check correspond frequency in SpectrumAnalyzer
#define TX_TRY_TIMES 5  // if TX_RETRY is set, pkt_Tx() will try x times before success
#define TX_DO_CARRIER_SENSE 1
#define TX_SOFT_ACK 1   // only affect RX part(send ACK by hw/sw). TX still check ACK by  hardware in this code. modify libraries if necessary.
#define TX_SOFT_FCS 1
#define TX_RETRY 1      // pkt_Tx() retransmit packets if failed.
#define TX_BACKOFF 100  // sleep time in ms
#define TX_HEADER_LEN 9
uint8_t TxBuffer[128]; // can be used as header and full pkt.
uint8_t RxBuffer[128];
uint8_t softACK[8];
char teststr[] = "TT ?";

uint8_t TX_available; // set to 1 if need a packet delivery, and use need_TX() to check  its value
// here are internal variables, please do not modify them.
uint8_t retry_c;
uint8_t RX_available; // use has_RX() to check its value
uint8_t RX_pkt_len;

// the setup() function is called when Zigduino staets or reset
void setup()
{
  teststr[3] = '0' + NODE_ID;
  init_header();
  retry_c = 0;
  TX_available = 0;
  RX_available = 1;
  ZigduinoRadio.begin(CHANNEL,TxBuffer);
  ZigduinoRadio.setParam(phyPanId,(uint16_t)0xABCD );
  ZigduinoRadio.setParam(phyShortAddr,(uint16_t)NODE_ID );
  ZigduinoRadio.setParam(phyCCAMode, (uint8_t)3);
  Serial.begin(9600);

  // register event handlers
  ZigduinoRadio.attachError(errHandle);
  ZigduinoRadio.attachTxDone(onXmitDone);
  ZigduinoRadio.attachReceiveFrame(pkt_Rx);
}

// the function is always running
void loop()
{
  uint8_t inbyte;
  uint8_t inhigh;
  uint8_t inlow;
  uint8_t tx_suc;

  if(need_TX()){
    delay(TX_BACKOFF);
    tx_suc = pkt_Tx(0x0001, teststr);
    TX_available = 1;
  }

/** this is from the original example
 *  it reads bytes from your serial input, then transmit it
  if (Serial.available()){
    ZigduinoRadio.beginTransmission();
    Serial.println();
    Serial.print("Tx: ");
    while(Serial.available())
    {
      char c = Serial.read();
      Serial.write(c);
      ZigduinoRadio.write(c);
    }
    Serial.println();
    ZigduinoRadio.endTransmission((uint16_t)0x0001); //0xffff for broadcast

    }
 */

if(has_RX())
  {
    Serial.println();
    Serial.print("Rx: ");
    for(uint8_t i=0;i<RX_pkt_len;i++){
      inbyte = RxBuffer[i];
      if(printable(inbyte)){
        Serial.write(inbyte);
      }else{
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
  delay(100);
}

void init_header(){
  if(TX_SOFT_ACK){
    TxBuffer[0] = 0x61; // ack required
  }else{
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
  softACK[0] = 0x42;
  softACK[1] = 0x88;
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
uint8_t pkt_Tx(uint16_t dst_addr, char* msg){
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
  // fill the payload
  for(i = 0; msg[i] != '\0'; i++){
    TxBuffer[TX_HEADER_LEN + i] = msg[i];
  }
  pkt_len = TX_HEADER_LEN + i;
  // fill the software fcs
  if(TX_SOFT_FCS){
    fcs = cal_fcs(TxBuffer, pkt_len);
    TxBuffer[pkt_len++] = fcs & 0xff;
    TxBuffer[pkt_len++] = fcs >> 8;
  }
  // fill the check_sum in TX_CHECKSUM_POS
  if(TX_SOFT_FCS){
	if(pkt_len%2==1){
		pkt_len ++;
	}
	check_sum = cal_check_sum(TxBuffer, pkt_len);
    TxBuffer[pkt_len++] = fcs & 0xff;
    TxBuffer[pkt_len++] = fcs >> 8;
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
        }else{
          Serial.print("ca fail with rssi = ");
          Serial.println(rssi);
        }
      }else{
        ZigduinoRadio.txFrame(TxBuffer, pkt_len);
      }
      delay(TX_BACKOFF);
    }
    retry_c--; // extra 1 by for loop, if tx success retry_c == TX_TRY_TIMES
  }else{
    if(TX_DO_CARRIER_SENSE){
//      cca = ZigduinoRadio.doCca();
      rssi = ZigduinoRadio.getRssiNow();
//      if(cca == RADIO_CCA_FREE)
      if(rssi == -91){
        ZigduinoRadio.txFrame(TxBuffer, pkt_len);
      }else{
        Serial.print("ca fail with rssi = ");
        Serial.println(rssi);
      }
    }else{
      ZigduinoRadio.txFrame(TxBuffer, pkt_len);
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
  uint16_t fcs;
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
		fcs = cal_fcs(frm, len-2);
	}else{
		fcs = cal_fcs(frm, len-3);
	}
    if(fcs != 0x0000){
      return RxBuffer;
    }
  }
  // send software ack
  if(frm[0] & 0x20){
    softACK[2] = frm[2];
    ZigduinoRadio.txFrame(softACK, 5);
  }
  // now all checks are passed, copy out the received packet
  for(uint8_t i=0; i < len; i++){
    RxBuffer[i] = frm[i];
  }
  RX_pkt_len = len;
  RX_available = 1;
  return RxBuffer;
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
  uint16_t fcs = frm[0];
  for(uint8_t i = 1; i < len; i += 1){
    fcs ^= frm[i];
  }
  return fcs;
}
// Check_sum 
/*
// Here's the code from CN of johntsai. lol
short check_sum(int size,char* data){		 //1 or 0
	char sum[2];
	int some_int;
	pkt pac;
	memcpy(&pac, &data, size);
	some_int = add_sum(size, data, sum);
	if(strncmp(sum,pac.chk_sum,2))
		return 0;
	else
		return some_int;
}
int add_sum(int size,char* data, char* dest){
	int sum=0,new;
	int i;
	char buffer[4];
	for(i=0;i<size;i+=2){
		new=(int)(data[i]&0xFF);
		new+=(int)(data[i+1]&0xFF)*256;
		sum+=new;
		memcpy(buffer,&sum,4);
		while(buffer[2]==0x01){
			sum-=256*256;
			sum+=1;
			memcpy(buffer,&sum,4);
		}
	}
	memcpy(buffer,&sum,4);
	dest[0]=~buffer[0];
	dest[1]=~buffer[1];
//	printf("%d %d\n",buffer[0]&0xFF,buffer[1]&0xFF);
//	printf("%d\n",(buffer[0]&0xFF)&(buffer[1]&0xFF));
	return (buffer[0]&0xFF)&(buffer[1]&0xFF)==255;
}
*/
/** For TX: Call it to get check_sum byte[2]
	For RX: Send all packets (as long as check_sum bytes) will return 0xff if the packet is not corrupted
*/
uint16_t cal_check_sum(uint8_t* frm, uint8_t len){
  uint16_t sum = 0x00,to_add;
  for(uint8_t i = 0; i < len; i += 2){
	to_add = (uint16_t)frm[i];
	if(i+1<len) 
		to_add += (uint16_t)(frm[i+1]<<8);
	sum = sum_two(sum,to_add);
  }
  return ~sum;	// return the one's complement, add it with the sum and get 0 if the packet is not corrupted
}
uint16_t sum_two(uint16_t a, uint16_t b){
  uint32_t sum = (uint32_t)a + (uint32_t)b;
  uint32_t carry = sum>>16;
	if(carry == 0x01){			// case that there's carry bit
		sum = sum_two(sum,carry); 
	}
  return (uint16_t) sum & 0xffff;
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
  }else if(x==TX_OK){
    Serial.print("(OK)");
    retry_c = TX_TRY_TIMES;
  }else if(x==TX_CCA_FAIL){ // not implemented
    Serial.print("(CS busy)");
  }
  Serial.println();
}
