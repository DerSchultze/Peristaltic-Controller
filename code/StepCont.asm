#include "P12F675.INC" ; Include header file
#DEFINE OSCCAL_1K 1
	__CONFIG	_INTRC_OSC_NOCLKOUT & _WDT_ON & _PWRTE_ON & _CP_OFF & _MCLRE_OFF

;EEPROM Memory map:
;0x00	Program data
;..
;0x7A	Program length
;0x7B	Program reentry point
;0x7C	Checksum
;0x7D	Reserved (Time multiplier)
;0x7E	Speed (DelayVal)
;0x7F	Mode bits
;	x... ....	Run mode: 0=Manual, 1=Auto
;	.x.. ....	Startup state in manual mode 0=Running, 1=Stopped
;	.... xyzq	Set liquid flags, y=wait when break, z=wait when dispense, q=invert liquid

;macros
#define     Bank0       BCF 	STATUS,RP0		; Select bank0
#define     Bank1       BSF 	STATUS,RP0		; Select bank0
#define     SerialIn    GPIO,GP3
#define     SerialOut   GPIO,GP5

;default values
#define     InitialDelay    0x17
#define     OuterDelay      0x10
#define     ChecksumAddr    0x7C
#define     ModeBitsAddr    0x7E
#define     DelayValAddr    0x7F
#define     DebugAddr       0x05

;variables
DelayVal        EQU     0x20
DelayCount      EQU     0x21
DelayCount2     EQU     0x22
RecvBuf         EQU     0x23
#define     SerialReady     0x24,0
DataCount       EQU     0x25
#define     ModeBits        0x26
#define     InvertLiquid    0x26,0
#define     WaitDispense    0x26,1
#define     WaitBreak       0x26,2
#define     AutoBit         0x26,6
#define     RunBit          0x26,7
SendBuffer     EQU     0x27

	ORG 0x000
		GOTO Start
		NOP
		NOP
		NOP

Interrupt
        BSF     SerialReady
		BCF     INTCON,GIE	; Disable interrupts
        BCF     INTCON,GPIF
        RETFIE

Start
        Bank0
        CLRF    GPIO
        MOVLW   0x07                ; Init GPIO
        MOVWF   CMCON

        Bank1
        call    3FFh                ;Get the cal value
        movwf   OSCCAL              ;Calibrate

        CLRF    ANSEL
;        MOVLW   b'00101000'         ; Init TRISIO
        MOVLW   b'00001000'         ; Init TRISIO
        MOVWF   TRISIO

;        MOVLW   b'00001000'         ; Mask in serial input IOC
;        MOVWF   IOC

        ; Read EEPROM data
        MOVLW   ModeBitsAddr
        MOVWF   EEADR
        BSF     EECON1,RD
        MOVFW   EEDATA
        MOVWF   ModeBits
        MOVLW   DelayValAddr
        MOVWF   EEADR
        BSF     EECON1,RD
        MOVFW   EEDATA
        MOVWF   DelayVal
        MOVLW   ChecksumAddr        ; We use RecvBuf for temp storage of checksum
        MOVWF   EEADR
        BSF     EECON1,RD
        MOVFW   EEDATA
        MOVWF   RecvBuf

        CLRW                        ; Calc checksum
        XORWF   ModeBits,W
        XORWF   DelayVal,W
        XORWF   RecvBuf,W           ; Shuld be 0
        BTFSC   STATUS,Z
        GOTO    ChecksumOk

        CLRF    ModeBits            ; Default values
        MOVLW   InitialDelay
        MOVWF   DelayVal

ChecksumOk
        Bank0
        BCF     SerialReady
;		BSF     INTCON,GIE	; Enable interrupts

        MOVLW   '#'
        MOVWF   SendBuffer
        CALL    SendSerial
        CLRF    RecvBuf

;        MOVLW   b'00001000'         ; Enable IOC on GPIO
;        MOVWF   INTCON

LoopStart
        CLRWDT
        BTFSC   SerialIn
        GOTO    HandleSerial
        BTFSC   RunBit
        GOTO    LoopStart

        BTFSC   SerialReady
        GOTO    HandleSerial
        MOVLW   b'00010001'
        MOVWF   GPIO
        CALL    Delay

        BTFSC   SerialReady
        GOTO    HandleSerial
        MOVLW   b'00000011'
        MOVWF   GPIO
        CALL    Delay

        BTFSC   SerialReady
        GOTO    HandleSerial
        MOVLW   b'00000110'
        MOVWF   GPIO
        CALL    Delay

        BTFSC   SerialReady
        GOTO    HandleSerial
        MOVLW   b'00010100'
        MOVWF   GPIO
        CALL    Delay

        GOTO    LoopStart

HandleSerial
        CALL    ReceiveSerial

        MOVFW   RecvBuf
        SUBLW   '1'
        BTFSS   STATUS,Z
        GOTO    NotSpeed1
        MOVLW   0x70        ; 100mL / hr
        MOVWF   DelayVal
        GOTO    EndHandleSerial
NotSpeed1
        MOVFW   RecvBuf
        SUBLW   '2'
        BTFSS   STATUS,Z
        GOTO    NotSpeed2
        MOVLW   0x38        ; 200mL / hr
        MOVWF   DelayVal
        GOTO    EndHandleSerial
NotSpeed2
        MOVFW   RecvBuf
        SUBLW   '3'
        BTFSS   STATUS,Z
        GOTO    NotSpeed3
        MOVLW   0x25        ; 300mL / hr
        MOVWF   DelayVal
        GOTO    EndHandleSerial
NotSpeed3
        MOVFW   RecvBuf
        SUBLW   '4'
        BTFSS   STATUS,Z
        GOTO    NotSpeed4
        MOVLW   0x1C        ; 400mL / hr
        MOVWF   DelayVal
        GOTO    EndHandleSerial
NotSpeed4
        MOVFW   RecvBuf
        SUBLW   '5'
        BTFSS   STATUS,Z
        GOTO    NotSpeed5
        MOVLW   0x17        ; 500mL / hr
        MOVWF   DelayVal
        GOTO    EndHandleSerial
NotSpeed5
        MOVFW   RecvBuf
        SUBLW   '6'
        BTFSS   STATUS,Z
        GOTO    NotSpeed6
        MOVLW   0x13        ; 600mL / hr
        MOVWF   DelayVal
        GOTO    EndHandleSerial
NotSpeed6
        MOVFW   RecvBuf
        SUBLW   '7'
        BTFSS   STATUS,Z
        GOTO    NotSpeed7
        MOVLW   0x10        ; 700mL / hr
        MOVWF   DelayVal
        GOTO    EndHandleSerial
NotSpeed7
        MOVFW   RecvBuf
        SUBLW   '8'
        BTFSS   STATUS,Z
        GOTO    NotSpeed8
        MOVLW   0x0E        ; 800mL / hr
        MOVWF   DelayVal
        GOTO    EndHandleSerial
NotSpeed8
        MOVFW   RecvBuf
        SUBLW   '9'
        BTFSS   STATUS,Z
        GOTO    NotSpeed9
        MOVLW   0x0C        ; 900mL / hr
        MOVWF   DelayVal
        GOTO    EndHandleSerial
NotSpeed9
        MOVFW   RecvBuf
        SUBLW   '0'
        BTFSS   STATUS,Z
        GOTO    NotSpeed10
        MOVLW   0x0B        ; 1000mL / hr
        MOVWF   DelayVal
        GOTO    EndHandleSerial
NotSpeed10
        MOVFW   RecvBuf
        SUBLW   'R'
        BTFSS   STATUS,Z
        GOTO    NotRun
        BCF     RunBit
        GOTO    EndHandleSerial
NotRun
        MOVFW   RecvBuf
        SUBLW   'S'
        BTFSS   STATUS,Z
        GOTO    NotStop

        BSF     RunBit
        CLRW
        MOVWF   GPIO
        GOTO    EndHandleSerial
NotStop
        MOVFW   RecvBuf
        SUBLW   'W'
        BTFSS   STATUS,Z
        GOTO    NotWrite

        Bank1
        MOVFW   ModeBits
        MOVWF   EEDATA
        MOVLW   ModeBitsAddr
        MOVWF   EEADR
        CALL    WriteEEPROM

        Bank1
        MOVFW   DelayVal
        MOVWF   EEDATA
        MOVLW   DelayValAddr
        MOVWF   EEADR
        CALL    WriteEEPROM

        Bank1                   ; Write a simple checksum
        CLRW
        XORWF   ModeBits,W
        XORWF   DelayVal,W
        MOVWF   EEDATA
        MOVLW   ChecksumAddr
        MOVWF   EEADR
        CALL    WriteEEPROM

        GOTO    EndHandleSerial
NotWrite

EndHandleSerial
        MOVFW   RecvBuf
        MOVWF   SendBuffer
        CALL    SendSerial

        BCF     SerialReady
;		BSF     INTCON,GIE	; Enable interrupts
        GOTO    LoopStart

WriteEEPROM
        Bank1
        BSF     EECON1,WREN     ; Remove protection
        BCF     INTCON,GIE
        MOVLW   0x55            ; Secret knock
        MOVWF   EECON2
        MOVLW   0xAA
        MOVWF   EECON2
        BSF     EECON1,WR
;        BSF     INTCON,GIE      ; Enable interrupts
WaitWrite                       ; Wait for write completion
        BTFSC   EECON1,WR
        GOTO    WaitWrite

        BCF     EECON1,WREN     ; Write protect the EEPROM
        Bank0                   ; Return clean
        RETURN

Delay
        MOVLW   OuterDelay
        MOVWF   DelayCount2

DelayInnerLoop
		MOVFW	DelayVal        ; Wait a while
		MOVWF	DelayCount      ; they assure that the channel selector stabilizes
        CLRWDT
DelayLoop
        BTFSC   SerialIn
        BSF     SerialReady
        BTFSC   SerialReady     ; Exit if there is any serial data
        RETURN

		DECFSZ	DelayCount,1
		GOTO	DelayLoop

		DECFSZ	DelayCount2,1
		GOTO	DelayInnerLoop
        RETURN

ReceiveSerial
StartWait
        CLRWDT
		BTFSS	SerialIn	; Wait for start bit ´0´ High on the line
		GOTO	StartWait

		MOVLW	0x0F		; Wait a while approx 70uS (was 0x15)
		MOVWF	DelayCount
StartWait1
        CLRWDT
		DECFSZ	DelayCount,1
		GOTO	StartWait1

		MOVLW	0x08		; We want to get 8 bits
		MOVWF	DataCount
RecvBit
		MOVLW	0x17		; Wait aprox 104uS (0x1E)
		MOVWF	DelayCount
RecvBitWait
        CLRWDT
		DECFSZ	DelayCount,1	; Wait for next bit
		GOTO	RecvBitWait

		BSF     STATUS,C
		BTFSC	SerialIn	; Test the waiting bit
		BCF     STATUS,C
		RRF     RecvBuf,1

		DECFSZ	DataCount,1	; Do we have 8 bits?
		GOTO	RecvBit

StopWaitEnd
		BTFSC	SerialIn	; Wait for serial line to go low after the last bit
		GOTO	StopWaitEnd

		RETURN

SendSerial
		MOVLW	0x08			; We want to Send 8 bits
		MOVWF	DataCount

SendStart
		BSF		SerialOut		; Send a start bit

		MOVLW	0x20			; Wait a while approx 104uS
		MOVWF	DelayCount
StartWaitSend
		DECFSZ	DelayCount,1
		GOTO	StartWaitSend
		NOP

SendBit
		RRF		SendBuffer,1
		BTFSS	STATUS,C		; Shall we send 0
		BSF		SerialOut		; Send a bit

		BTFSC	STATUS,C		; Shall we send 1
		BCF		SerialOut		; Send a bit

		NOP

		MOVLW	0x20			; Bit timing approx 104uS
		MOVWF	DelayCount

SendBitWait
		DECFSZ	DelayCount,1	; Wait until next bit
		GOTO	SendBitWait

		DECFSZ	DataCount,1		; Have we send 6 bits?
		GOTO	SendBit

		BCF		SerialOut		; Send the stop bit

		MOVLW	0x20			; Wait a while approx 104uS
		MOVWF	DelayCount
StopWait
		DECFSZ	DelayCount,1
		GOTO	StopWait

		RETURN

    END
