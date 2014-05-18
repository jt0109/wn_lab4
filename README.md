wn_lab4
=======
LAB4 of wn2014, team7

There are some pde codes:
	examples/ZigduinoRadioExample/rx/rx.ino		# for testing receiving packets
	examples/ZigduinoRadioExample/tx/tx.ino		# for testing transmitting packets 
																						# not yet implemented
	examples/ZigduinoRadioExample/ZigduinoRadioExample.ino		# Original code
	
Plans and todos:

====== Channel changing ?
1? Use SpectrumAnalyzer to find good channel 
2? Check corresponding frequency in SpectrumAnalyzer

JT: ... OK I really don't know what the fuck we're going to do about this.

====== MAC
1. Design header (following IEEE802.15.4)

2. CSMA/CA
	Check this page
	http://en.wikipedia.org/wiki/Carrier_sense_multiple_access_with_collision_avoidance

	Tools:
		ZigduinoRadio.getRssiNow()	-> Get Rssi, which represents the power of signal -> -91 means channel clear/power low
		cZigduinoRadio::doCca()			-> Check if the channel is clear (Clear Channel Axxxxx)

3. RTS/CTS
	http://en.wikipedia.org/wiki/IEEE_802.11_RTS/CTS

	I'm not sure how tell them apart using informations in pkts. (How to send/recv and aware of it.)

4. ACK
	4.1.	Same problem with previous one.
	4.2.	The hw ACK have no sender information.	-> What the heck. We need to close hardware one and write our own ACK?

5. Error Detecting (checksum/ CRC)
	5.1. Hardware FCS is broken -> we need our own one.
	5.2. unit16_t calc_fcs(...)
		Simple. But not good in many situations.
		

====== Routing

====== 
	