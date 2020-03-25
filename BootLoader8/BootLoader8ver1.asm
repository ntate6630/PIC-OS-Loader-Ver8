;**** Z80 Bootloader ****
; Written by Nicholas Tate  02/02/2019
;****

	LIST 		p=16F877A
	#include	<P16F877A.inc>

	__config _HS_OSC & _WDT_OFF & _PWRTE_ON & _CP_OFF & _BODEN_ON & _LVP_ON & _CPD_OFF & _DEBUG_OFF

	errorlevel -302					;Suppress message 302 from list file

#define	WR_CONTROL		b'10100000'	;Control byte for I2C write operations
#define	RD_CONTROL	 	b'10100001'	;Control byte for I2C read operations

;Variables
		cblock 0x20				
		temp						;Temporary register.				
		addrH						;High byte of EEPROM address.
		addrL						;Low byte of EEPROM address.
		outAddrH					;High byte of address on output ports.
		outAddrL					;Low byte of address on output ports.
		data_out
		data_in
		tx_buffer
		rx_buffer
		counterH				
		counterL
		pollcount					;Holds the number of tries when ACK polling.
		delay
		delay1
		delay2
		endc			

Bank0		MACRO					;Macro to select data RAM bank 0
		bcf		STATUS,RP0
		bcf		STATUS,RP1
		ENDM

Bank1		MACRO					;Macro to select data RAM bank 1
		bsf		STATUS,RP0
		bcf		STATUS,RP1
		ENDM

Bank2		MACRO					;Macro to select data RAM bank 2
		bcf		STATUS,RP0
		bsf		STATUS,RP1
		ENDM

Bank3		MACRO					;Macro to select data RAM bank 3
		bsf		STATUS,RP0
		bsf		STATUS,RP1
		ENDM

		ORG     0x0000				;Place code at reset vector

ResetCode	
		clrf    PCLATH				;Select program memory page 0
  		goto    Main				;Go to beginning of program

;**** This code executes when an interrupt occurs.
		ORG	0x0004					;Place code at interrupt vector
InterruptCode						;Do interrupts here
		retfie						;Return from interrupt

Main
		btfss	STATUS, 3			;Test the "PD" bit to determine if /RESET occurred on power up or if reset was performed manually after power up.
									;The "PD" bit will be set to '1' after power up in which case the bootLoader functions will run. The sleep instruction at the end will set the "PD" to '0'.
									;Once the "PD" is set to '0' all subsequent resets will perform a fastReset.
		goto	fastReset
bootLoader
		call 	PortSetup			;Configure ports  Set /BUSREQ low so target CPU is suspended. 
		call	i2c_init			;Configure I2C bus pins and setup MSSP.
		call	initializeAddressPointers	;Setup address pointers for EEPROM and target system RAM.
		call	dataTransfer		;Perform the transfer of data from I2C EEPROM to the RAM. 0000h to 17FFh.

		call	releaseBus			;Change port configuration to all inputs except PORTA bit 0, 4 and 5, then set /BUSREQ and /RESET Low for 0.5 seconds then high again high so target CPU is free to run.
		goto	wait
fastReset
		call	doFastReset
wait
		sleep						
		goto	wait

doFastReset							;Reset only function.
		banksel	PORTB				 
		clrf	PORTB
		movlw	0xFF				;Setup PORTB as inputs.
		banksel	TRISB				;PORTB bit B3 must remain set as input and tied to ground through 1K resistor.
		movwf	TRISB
		movwf	TRISC
		movwf	TRISD
		movlw	0x07
		movwf	TRISE

		banksel	PORTA
		CLRF 	PORTA				
		Bank1
		movlw 	0x07				;Configure all pins as digital input/outputs.
		movwf 	ADCON1 				;
		movlw 	0xCF 				;
		movwf 	TRISA 				;Set PORTA bit A0 to input as /BUSREQ not needed. A4 to control CPU reset and A5 as output for status LED. All other bits as inputs. B'11001111'
		banksel	PORTA
		movlw	b'00000001'			;/RESET = 0. to reset the CPU. Turn OFF status LED.
		movwf	PORTA
		call	ms_delay			
		call	ms_delay
		call	ms_delay			
		movlw	b'00110001'			;/RESET = 1. Reset goes high. CPU is running again.
		movwf	PORTA				;Turn ON status LED attached to PORTA bit 5 to indicate everything went OK.
		return

PortSetup
		banksel	PORTB				 
		clrf	PORTB
		movlw	0xFF				;Setup PORTB B0 as input for checking /BUSACK from target CPU. Set B1 - B2 and B4 - B7 as inputs. B'11111111'
		banksel	TRISB				;PORTB bit B3 must remain set as input and tied to ground through 1K resistor.
		movwf	TRISB

		banksel	PORTA
		CLRF 	PORTA 				
		Bank1
		movlw 	0x07				;Configure all pins as digital input/outputs.
		movwf 	ADCON1 				;
		movlw 	0xCE 				;
		movwf 	TRISA 				;Set PORTA bit A0 to output to control target CPU /BUSREQ line. A4 to control CPU reset and A5 as output for status LED. All other bits as inputs. B'11001110'
		banksel	PORTA
		movlw	b'00000000'			;/RESET the CPU and set /BUSREQ low so when the CPU comes out of reset it immediately follows /BUSREQ and tristates the busses. Set the status LED OFF. 
		movwf	PORTA
		call	ms_delay
		call	ms_delay
		call	ms_delay
		bsf		PORTA, 4			;RESET is released to allow CPU to perform bus request.				
busack_high
		btfsc	PORTB, 0			;Test bit B0 to see when target CPU has set /BUSACK LOW and bus is now tristate.
		goto	busack_high
busack_low	
		nop			
		clrf	PORTE
		banksel	TRISE		
		movlw	0x00
		movwf	TRISE				;Set PORTE to outputs.
		banksel	PORTE
		movlw	0x7F				;E2 - /CS, E1 - /WR, E0 - /RD all inactive high.
		movwf	PORTE

		banksel	TRISA
		movlw	0x00				;Keeping /BUSREQ low and RESET high and status LED OFF, set all the other PORTA bits to outputs as they are used as address lines A12, A3 and A4.
		movwf	TRISA
		banksel	PORTA
		movlw	0x10				;PORTA bit 4 is used for target CPU reset. Keep this line high. NOTE: A pull up resistor is required as it is open drain output.
		movwf	PORTA
         	
        clrf    PORTC
        movlw   0x18				;Set PORTC C0 to C2 to output. C3 to C4 to input for I2C bus, and C5 to C7 to output.  B'00011000'
        banksel TRISC				
        movwf   TRISC   			
		
		movlw	0x09				;Setup PORTB B4 - B7 as outputs for address lines A8 - A11. B0 stays an input for /BUSACK signal from target CPU and B3 stays an input because its tied down with 1k resistor and not used.
		movwf	TRISB				;00001001
		banksel	PORTB	
		movlw	0X00
		movwf	PORTB						

		banksel	PORTD				
		clrf	PORTD
		movlw	0x00				;Set PORTD for outputs - Data bus lines D0 - D7.
		banksel	TRISD
		movwf	TRISD				
		return
   
initializeAddressPointers
		banksel	PORTA	
		movlw	0x00				;Setup address counters for output ports.
		movwf	outAddrL
		movwf	outAddrH
		movwf	PORTC				;Set address A0 to A2 and A5 to A7 to 0. Pins 3 and 4 are used for I2C bus. 
		movwf	PORTB				;Set address A8 - A11 to 0. Set A13 and A14 to 0.
		bcf		PORTA, 1			;Set address A12 to 0.
		bcf		PORTA, 2			;Set address A3 to 0.
		bcf		PORTA, 3			;Set address A4 to 0.
		movlw	0x00 				;Setup I2C EEPROM memory address pointers. Starting at 0x0000h.
		movwf	addrL
		movlw	0x00
		movwf	addrH
		return
	
;This routine performs reading the I2C EEPROM and writing each byte to the target systems RAM.
dataTransfer
		movlw	0x00				;Set counters to 0x1800h
		movwf	counterL
		movlw	0x20
		movwf	counterH
doNextByte
		call	ByteRead		;Read a byte of data from I2C EEPROM.
outputAddress
		Bank0
		movf	outAddrL, W		;Output address to the address bus PORTC and PORTB.
		movwf	PORTC
remapA3
		btfsc	outAddrL, 3		;Test bit 3 and re-map it to PORTA pin A2
		bsf		PORTA, 2
		btfss	outAddrL, 3
		bcf		PORTA, 2
remapA4
		btfsc	outAddrL, 4		;Test bit 4 and re-map it to PORTA pin A3
		bsf		PORTA, 3
		btfss	outAddrL, 4
		bcf		PORTA, 3

		movf	outAddrH, W
		movwf	temp
		swapf	temp, W			;Swap the high and low nibbles of outAddrH and send to PORTB. only the top 4 bits are used.
		andlw	0xF0			;Mask off the top four bits.
		movwf	PORTB
		btfsc	outAddrH, 4		;Test bit 4 of outAddrH for address line A12.
		bsf		PORTA, 1		;Set PORTA bit 1 when address 0x1000h is reached.
		btfss	outAddrH, 4		;Reset PORTA bit 1 when this address line is 0.
		bcf		PORTA, 1 		
outputData
		movf	rx_buffer, W		;
		movwf	PORTD
		movlw	0x7B	
		movwf	PORTE			;Bit E0 - Enable /CE  b'01111011'
		movlw	0x79
		movwf	PORTE			;Bit E1 - Enable /WR  b'01111001'
;		nop
		movlw	0x7B
		bsf		PORTE, 1		;Bit E1 - Disable /WR b'01111011'
;		nop
		movlw	0x7F
		movwf	PORTE			;Bit E0 - Disable /CE b'01111111'
incAddresses	
		incf	outAddrL, F		
		incf	addrL, F
		decfsz	counterL, F
		goto	doNextByte
		incf	outAddrH, F
		incf	addrH, F
		decfsz	counterH, F
		goto	doNextByte
		return

releaseBus						;Set all bus lines to inputs so the target CPU can be ready to take over.			
		banksel	TRISD
		movlw	0xFF			;Set ports to inputs so they do not interfere with bus when target CPU is running.
		movwf	TRISD			
		movwf	TRISC
		movwf	TRISB
		movlw	0x07
		movwf	TRISE
		movlw	b'11001110'			;Leave A0, A4, A5 as outputs for the /BUSREQ, /RESET, and the status LED.	
		movwf	TRISA
		banksel	PORTA
		bsf		PORTA, 0			;/BUSREQ = 1. /BUSREQ goes high to allow target CPU to take over the busses.
busack_low2
		btfss	PORTB, 0			;Wait for /BUSACK to go high again.
		goto	busack_low2
busack_high2						;Target CPU has now taken over busses.
		nop
		bcf		PORTA, 4			;/RESET = 0. Reset goes low to reset the target CPU so it starts executing code from 0000h.
		call	ms_delay			;Allow 117 milliseconds for reset.
		call	ms_delay
		call	ms_delay
		movlw	b'00110001'			;/RESET = 1. Reset goes high. CPU is running again. /BUSREQ = 1. Turn ON status LED attached to PORTA bit 5 to indicate everything went OK.
		movwf	PORTA				
		return

;This routine initializes the MSSP module for I2C Master mode, with 100Khz clock
i2c_init
    bcf     STATUS,RP1          ; Select Bank 01
    bsf     STATUS,RP0
	movlw   0x18				;Set PORTC address lines C0 to C2 to output. C3 to C4 to input. C5 to C7 to output.  B'00011000'
    movwf   TRISC
	clrf	SSPSTAT				;Disable SMBus	inputs
	bsf		SSPSTAT,SMP			;Disable slew rate control
	movlw	0x04				;Set the bit rate to 100Khz 0x32
	movwf	SSPADD				;Setup 100Khz I2C Clock
	clrf	SSPCON2				;Clear control bits
	bcf		STATUS,RP0			;Bank 00
	movlw	b'00101000'	
	movwf	SSPCON				;Enable SSP, select I2C Master mode
	bcf		PIR1,SSPIF			;Clear SSP interupt flag
	bcf		PIR2,BCLIF			;Clear bit collision flag
	retlw	0

;This routine will write 1 byte to the I2C EEPROM at the specified by addrH and addrL.
ByteWrite
	Call	BSTART				;Generate start condition
								;Send control byte
	bcf		STATUS,RP0			;Select bank 00
	movlw	WR_CONTROL			;Load control byte for write
	movwf	data_out			;Copy to datao for output
	call	TX					;Send control byte to EEPROM device
	
								;Send address high byte
	bcf		STATUS,RP0			;Select bank 00
	movf	addrH, W			;Address location
	movwf	data_out			;Copy to datao for output
	call	TX					;Send word address byte to device

								;Send address low byte
	bcf		STATUS,RP0			;Select bank 00
	movf	addrL, W			;Address location
	movwf	data_out			;Copy to datao for output
	call	TX					;Send word address byte to device

								;Send data byte
	bcf		STATUS,RP0			;Select bank 00
	movf	tx_buffer, W
	movwf	data_out
	call	TX					;Send data byte to device

	call	BSTOP				;Generate stop condition
	call	Poll				;Poll for write completion
	retlw	0

;This routine will read a byte of data from the I2C EEPROM at address specified by addrH and addrL.
ByteRead
	call	BSTART				;Generate start condition
								;Send control byte
	bcf		STATUS,RP0			;Select bank 00
	movlw	WR_CONTROL			;Load control byte for write
	movwf	data_out			;Copy to 'data_out' for output
	call	TX					;Send control byte to device

								;Send word address high byte
	bcf		STATUS,RP0			;Select bank 00
	movf	addrH, W			;Address high byte
	movwf	data_out				
	call	TX					;Send high byte to device
		
								;Send word address low byte
	bcf		STATUS,RP0			;Select bank 00
	movf	addrL, W			;Address low byte
	movwf	data_out
	call	TX					;Send low byte to device

	call	BRESTART			;Generate restart condition

								;Send control byte
	bcf		STATUS,RP0			;Select bank 00
	movlw	RD_CONTROL			;Load control byte for read
	movwf	data_out
	call	TX					;Send control byte

								;Read data byte
	bsf		STATUS,RP0			;Select bank 01
	bsf		SSPCON2,ACKDT		;Select to send NO ACK bit
	call	RX					;Read data byte from EEPROM device
	movf	data_in, W
	movwf	rx_buffer			;Save data byte to rx buffer
	
	call	BSTOP				;Generate stop condition
	retlw	0

;This routine reads in one byte of data from the EEPROM and stores it in 'rx_buffer'.
;It then responds with either an ACK or a NO ACK bit, depending on the value of ACKDT in SSPCON2.
RX
	bcf		STATUS,RP1
	bcf		STATUS,RP0			;Select bank 00
	bcf		PIR1,SSPIF			;Clear SSP interupt flag
	bsf		STATUS,RP0			;Select bank 01
	bsf		SSPCON2,RCEN		;Initiate reception of byte
	bcf		STATUS,RP0			;Select bank 00
rx_wait
	btfss	PIR1,SSPIF			;Check if operation completed
	goto	rx_wait				;If not keep checking
	movf	SSPBUF,W			;Copy received data byte to W
	movwf	data_in				;Copy W to 'datai'
	bcf		PIR1,SSPIF			;Clear SSP interupt flag
	bsf		STATUS,RP0			;Select bank 01
	bsf		SSPCON2,ACKEN		;Generate ACK / NO ACK bit
	bcf		STATUS,RP0			;Select bank 01
rx_wait2
	btfss	PIR1,SSPIF			;Check if Operation completed
	goto 	rx_wait2			;If not keep checking
	retlw	0

; This routine transmits the byte of data stored in 'tx_buffer' to the I2C EEPROM.
; Instructions are in place to check  for an ACK bit if needed.
; Replace the 'goto' instruction, or create an 'ackfailed' label, to provide the functionality.
TX
	bcf		STATUS,RP1
	bcf		STATUS,RP0			;Select bank 00
	bcf		PIR1,SSPIF			;Clear SSP interupt flag
	movf	data_out,W			;Output a byte of data previously loaded into tx_buffer before calling this routine
	movwf	SSPBUF				;Write byte out to device.
tx_wait
	btfss	PIR1,SSPIF			;Check if operation completed
	goto	tx_wait				;If not keep checking
	bsf		STATUS,RP0			;Select bank01
	btfsc	SSPCON2,ACKSTAT		;Check if ACK bit was received
	goto	ackfailed			;This executes if no ACK received
	retlw	0

ackfailed
	banksel	PORTA
	bsf		PORTA, 5
	call	long_delay
	bcf		PORTA, 5
	call	long_delay
	goto 	ackfailed

;This routine generates a start condition high to low transition of SDA while SCL is still high.
BSTART
	bcf		STATUS,RP1
	bcf		STATUS,RP0			;Select bank 00
	bcf		PIR1,SSPIF			;Clear SSP interupt flag
	bsf		STATUS,RP0			;Select bank 01
	bsf		SSPCON2,SEN			;Generate start condition
	bcf		STATUS,RP0			;Select bank 00
bstart_wait
	btfss	PIR1,SSPIF			;Check if operation completed
	goto	bstart_wait			;if not keep checking
	retlw	0

;This routine polls the EEPROM for an ACK bit, which indicates the internal write cycle has completed.
;Code is in place for a write timeout routine, just uncomment the 'goto TimedOut' line, and provide a 'TimedOut' label
;and add some code to handle a timedout situation.
Poll
	bcf		STATUS,RP0			;Select bank00
	movlw	.40
	movwf	pollcount			;Set max polling times to 40
polling
	call	BRESTART			;Generate start bit
	bcf		STATUS,RP0			;Select bank 00
	movlw	WR_CONTROL			;Now send the control byte
	movwf	data_out			;Copy control byte to buffer
	call	TX					;Output control byte to device
	bsf		STATUS,RP0			;Select bank 01
	btfss	SSPCON2,ACKSTAT		;Was the ACK bit low?
	goto	exitpoll			;If yes then stop polling
								;If no check if polled 40 times
	bcf		STATUS,RP0			;Select bank 00
	decfsz	pollcount,F			;Is pollcnt down to zero?
	goto	polling				;If NO, goto polling
;	goto	TimedOut			;If yes, part didnt respond in time
								;so add some code to handle this situation
exitpoll
	call	BSTOP				;Generate stop bit
	retlw	0	

TimedOut
	banksel	PORTA
	bsf		PORTA, 5
	call	long_delay
	call	long_delay
	call	long_delay
	call	long_delay
	bcf		PORTA, 5
	call	long_delay
	call	long_delay
	call	long_delay
	call	long_delay
	goto 	TimedOut

;This routine generates a repeated start condition high to low transition of SDA while SCL is still high.
BRESTART
	bcf		STATUS,RP1
	bcf		STATUS,RP0			;Select bank 00
	bcf		PIR1,SSPIF			;Clear SSP interupt flag
	bsf		STATUS,RP0			;Select bank 01
	bsf		SSPCON2,RSEN		;Generate restart condition
	bcf		STATUS,RP0			;Select bank 00
brestart_wait
	btfss	PIR1,SSPIF			;Chech if operation completed
	goto	brestart_wait		;if not keep checking
	retlw	0

;This routine generates a stop condition low to high transition of SDA while SCL is still high.
BSTOP
	bcf		STATUS,RP1
	bcf		STATUS,RP0			;Select bank 00
	bcf		PIR1,SSPIF			;Clear SSP interupt flag
	bsf		STATUS,RP0			;Select bank 01
	bsf		SSPCON2,PEN			;Generate stop condition
	bcf		STATUS,RP0			;Select bank 00
bstop_wait
	btfss 	PIR1,SSPIF			;Check if operation completed
	goto	bstop_wait			;If not keep checking
	retlw	0

long_delay
		movlw	0x08
		movwf	delay2
d_loop2
		call	ms_delay
		decfsz	delay2,F
		goto	d_loop2
		return

ms_delay
		movlw	0xFF
		movwf	delay
		movlw	0xFF
		movwf	delay1
d_loop	decfsz	delay, F
		goto	d_loop
		decfsz	delay1, F
		goto	d_loop
		return

		END