# Timer1 base time for 1 second.result displayed in decimal format in USART,works fine
# Timer2 external mode2 ,Default mapping (CH1/ETR/PD4, CH2/PD3,CH3/PC0, CH4/PD7). 
#BRR = BAUD  #13.6=38400,#4.4 = 115200,#52.1 = 9600,#8.7 =  57600
#USARTx_TX Full-duplex mode Push-pull multiplexed outputs,#USARTx_RX Full-duplex mode Floating input or pull-up input
#PD5 -tx
#PD6 -rx
#PD4 - ETR pin for measuring frequency

fclk = 24000000   # 24Mhz RCO internal , AHB =8Mhz by default
state = 0x2000000C # located in SRAM
result1 = 0x20000010 # 0x20000010 to 0x20000018 is used for storing result in decimal format
scratch = 0x2000001C
include CH32V003_reg1.asm

vtable:
	j reset_handler		#  longs 0x00000000 # RESERVED 0
align 4
  longs   0x00000000 # RESERVED 1
  longs   0x00000000 #pack <l longs NMI_IRQhandler
  longs   0x00000000 #pack <l HardFault_IRQhandler
  longs   0x00000000 # RESERVED 4
  longs   0x00000000 # RESERVED 5
  longs   0x00000000 # RESERVED 6
  longs   0x00000000 # RESERVED 7
  longs   0x00000000 # RESERVED 8
  longs   0x00000000 # RESERVED 9
  longs   0x00000000 # RESERVED 10
  longs   0x00000000 # RESERVED 11
  longs   0x00000000 # pack <l SysTick_IRQhandler	#; place the address of the mtime ISR subroutine in the vector table position 7,assembler will store isr address here, longs 0x00000000 # RESERVED 12	
  longs   0x00000000 # RESERVED 13
  longs   0x00000000 #pack <l SW_Software_IRQhandler
  longs   0x00000000 # RESERVED 15
  longs   0x00000000 #pack <l WWDG_IRQhandler
  longs   0x00000000 #pack <l PVD_IRQhandler
  longs   0x00000000 #pack <l FLASH_IRQhandler
  longs   0x00000000 #pack <l RCC_IRQhandler
  longs   0x00000000 #pack <l EXTI7_0_IRQhandler
  longs   0x00000000 #pack <l AWU_IRQhandler
  longs   0x00000000 #pack <l DMA1_CH1_IRQhandler
  longs   0x00000000 #pack <l DMA1_CH2_IRQhandler
  longs   0x00000000 #pack <l DMA1_CH3_IRQhandler
  longs   0x00000000 #pack <l DMA1_CH4_IRQhandler
  longs   0x00000000 #pack <l DMA1_CH5_IRQhandler
  longs   0x00000000 #pack <l DMA1_CH6_IRQhandler
  longs   0x00000000 #pack <l DMA1_CH7_IRQhandler
  longs   0x00000000 #pack <l ADC1_IRQhandler
  longs   0x00000000 #pack <l I2C1_EV_IRQhandler
  longs   0x00000000 #pack <l I2C1_ER_IRQhandler
  longs   0x00000000 #pack <l USART1_IRQhandler
  longs   0x00000000 #pack <l SPI1_IRQhandler
  longs   0x00000000 #pack <l TIM1BRK_IRQhandler
pack <l TIM1UP_IRQhandler
  longs   0x00000000 #pack <l TIM1TRG_COM_IRQhandler
  longs   0x00000000 #pack <l TIM1CC_IRQhandler
pack <l TIM2_IRQhandler

reset_handler:


    	li sp, STACK			# load stack pointer with stack end address
	 
    	li t0, vtable			#BASEADDR[31:2],The interrupt vector table base address,which needs to be 1KB aligned
    	ori t0, t0, 3			#BASEADDR[31:2],1: Identify by absolute address,1: Address offset based on interrupt number *4
    	#csrrw zero,t0, mtvec		# write to mtvec
	longs 0x30529073  
    
   	li t0,main
	longs 0x34129073          	#csrw	mepc,t0 :mepc updated with address of main
	longs 0x30200073		# mret ( return from interrupt)	.
  
	align 4
main:
	nop


#enable periphrel clocks
	li x10,R32_RCC_APB2PCENR	# load address of APB2PCENR register to x10 ,for enabling GPIO A,D,C peripherals
	lw x11,0(x10)			# load contents from peripheral register R32_RCC_APB2PCENR pointed by x10
	li x7,((1<<2)|(1<<4)|(1<<5)|(1<<14)|(1<<11)|(1<<0))	# 1<<IOPA_EN,1<<IOPC_EN,1<<IOPD_EN,1<<USART_EN,1<<TIM1_EN,1<<AF_EN
	or x11,x11,x7			# or values 
	sw x11,0(10)			# store modified enable values in R32_RCC_APB2PCENR
	li x10,R32_RCC_APB1PCENR
	lw x11,0(x10)
	ori,x11,x11,(1<<0)		# timer2 clock enable
	sw x11,0(x10)

#configure GPIO 
	li x10,R32_GPIOD_CFGLR		# load pointer x10 with address of R32_GPIOD_CFGLR , GPIO configuration register
	lw x11,0(x10)			# load contents from register pointed by x10
	li x7,~((0xf<<20)|(0xf<<24)|(0xf<<16))	#clear pd4,pd5,pd6. we need to setup PD5 & PD6 for usart tx and rx and pd4 for ETR
	and x11,x11,x7			# clear pd4,pd5,pd6 mode and cnf bits for selected pin D4,D5,D6
	li x7,((0x8<<24)|(0xB<<20)|(0x4<<16))	# pd6 = input with PU/PD,pd5= multiplex pushpull output 50mhz,pd4 floating input for ETR
	or x11,x11,x7			# OR value to register
	sw x11,0(x10)			# store in R32_GPIOD_CFGLR

#enable pull up for input	
	li x10,R32_GPIOD_OUTDR		# enable pullup resistor by setting OUTDR register
	lw x11,0(x10)			# this setting of GPIO_OUTDR for pullup resistor effects if corresonding pin is selected as input
	li x7,(1<<6)			#when PD6 is input with resistor selected 1= pullup and 0 = pulldown
	or x11,x11,x7
	sw x11,0(x0)

#configure USART baud
	li x10,R32_USART_BRR		# USART BAUD setting
	lw x11,0(x10)			# copy R32_USART_BRR to x11
	li x7,((52<<4)|(1<<0))		# 52.1 in BRR =9600
	or x11,x11,x7			# or registers
	sw x11,0(x10)			# store in R32_USART_BRR

#setup UART control and enable	
	li x10,R32_USART_CTLR1		# load x10 with R32_USART_CTLR1 address
	lw x11,0(x10)			# load to x11 contents
	li x7,(1<<13)|(1<<3)|(1<<2)	# enable USART UE, TX,RX bits		# UE 
	or x11,x11,x7
	sw x11,0(x10)			# store back new values

timer1_for_1s:
	li x10,R16_TIM1_PSC
	li x11,(7999)			# fck_PSC/PSC[15:0]+1,8MHz/7+1 =1MHz
	sw x11,0(x10)
	li x10,R16_TIM1_ATRLR
	li x11,(999)			# max reload
	sw x11,0(x10)	
	li x10,R16_TIM1_DMAINTENR
	lw x11,0(x10)
	li x7,(1<<0)			# enable update interrupt
	or x11,x11,x7
	sw x11,0(x10)
	li x10,R16_TIM1_CTLR1
	lw x11,0(x10)
	li x7,((0<<1)|(1<<2)|(1<<7))	# UDIS =0 ,URS =1,ARPE=1,
	or x11,x11,x7
	sw x11,0(x10)
	li x10,R16_TIM1_SWEVGR
	lw x11,0(x10)
	li x7,(1<<0)			# 1<<UG, generate update event to load all values immediately
	or x11,x11,x7
	sw x11,0(x10)
	li x10,R16_TIM1_INTFR
	lw x11,0(x10)
	li x7,0xFFFFFFFA		# 0<<UIF  clear update interrupt flag
	and x11,x11,x7
	sw x11,0(x10)
	li x10,R16_TIM1_CTLR1
	lw x11,0(x10)
	ori x11,x11,(1<<0)		# enable timer 2
	sw x11,0(x10)

timer2_ETR:
	li x10,R16_TIM2_SMCFGR
	lw x11,0(x10)
	li x7((0<<15)|(1<<14))		# external clock mode 2
	or x11,x11,x7
	sw x11,0(x10)


	li x10,R16_TIM2_CCER
	lw x11,0(x10)
	li x7,1<<0			# capture enable
	or x11,x11,x7
	sw x11,0(x10)

	li x10,R16_TIM2_DMAINTENR
	lw x11,0(x10)
	ori x11,x11,1			# 1<<UIE enable update interrupt
	sw x11,0(x10)

	li x10,R16_TIM2_INTFR
	lw x11,0(x10)
	li x7,~1			# 0<<UIF ,clear update interrupt flag
	and x11,x11,x7
	sw x11,0(x10)
	li x10,R16_TIM2_PSC
	li x11,(0)			# fck_PSC/PSC[15:0]+1,8MHz/7+1 =1MHz
	sw x11,0(x10)
	li x10,R16_TIM2_ATRLR
	li x11,(0xffffffff)		# max reload
	sw x11,0(x10)
	li x10,R16_TIM2_CTLR1
	lw x11,0(x10)
	ori x11,x11,(1<<0)		# enable timer 2
	sw x11,0(x10)	


PFIC_CONFIG:
	li x10,R32_PFIC_CFGR		# reset core PFIC register for interrupts
	lw x11,0(x10)
	li x7,((PFIC_KEY3<<16)|(1<<7))	# key3  and SYSRESET , reference manual tells to do it
	or x11,x11,x7
	sw x11,0(x10)			# store back new values

	li x10,R32_PFIC_IENR2		# PFIC Interrupt Enable in core PFIC
	lw x11,0(x10)
	li x7,((1<<6)|(1<<3))		# enabled TIM2 interrupt & TIM1 update interrupts in PFIC
	or x11,x11,x7
	sw x11,0(x10)			# store back new values

# enabling GLOBAL INTERRUPTS
	li t0, 0x88			# load MPIE and MIE bits , 1<<MIE in mstatus is enabling GLOBAL INTERRUPTS
	longs 0x30029073        	#csrw	mstatus,t0 ,manually assembled opcade to do csrrw the values in t0,  


	li x10,state			# point x10 to state register in sram	
	sw zero,0(x10)			# store 0 in state else overflow will be reported in timer1 IRQ and stuck for random value loop inside the IRQ

#main endless loop for uart transmit
example:

	li x10,name			# load address of label "name" to x10, string to be transmitted
string_loop:
	lb x8,0(x10)			# load 1 byte from 0 offset of "name"
	beqz x8,finish			# if byte in x8 null branch to label "finish"
	call USART_TX			# call subroutine USART_TX to transmit byte
	addi x10,x10,1			# increase pointer by 1 byte
	j string_loop			# jump back to label string_loop until null is encountered
		

	li x10,R16_TIM2_CNT
	lw x11,0(x10)
	li x7,0
	sw x7,0(x10)
	li x10,result1
	sw x11,0(x10)
	li x7,0
	sw x7,0(x10)
	
finish:
	
	j finish
	
#####################################################################
#
#####################################################################	
	
USART_TX:
	addi sp,sp,-16			# add space in stack
	sw ra,0(sp)			# push ra
	sw x7,4(sp)			# push x7
	sw x10,8(sp)			# push x10
	sw x11,12(sp)			# push x11

	li x10,R32_USART_STATR		# load address of usart status register
	lw x11,0(x10)			# load contents of status register in x11
	andi x11,x11,(1<<7)		# mask out 7th bit, transmit buffer empty flag
	beqz x11,USART_TX		# if 0 transmit buffer full, wait until bit is set
	#li x8,0x30
	mv x7,x8			# move byte in x8 to x7
	li x10,R32_USART_DATAR		# x10 has the address of data register
	sb x7,0(x10)			#store byte in x7 to data register
TC_check:
	li x10,R32_USART_STATR		# get contents of status register again
	lw x11,0(x10)
	andi x11,x11,(1<<6)		# check transmit complete bit
	beqz x11,TC_check		# wait if bit is 0 , when transmit complete = 1
		
	lw x11,12(sp)			# pop x11
	lw x10,8(sp)			# pop x10
	lw x7,4(sp)			# pop x7
	lw ra,0(sp)			# pop ra
	addi sp,sp,16			# set SP back 16 bytes
	ret				# return to caller

###################################################

###################################################
delay:	
	addi sp,sp,-8			# move sp 2 words
	sw ra,0(sp)			# push ra
	sw x6,4(sp)			# push x6
	li x6,2000000			# load an arbitarary value 20000000 to t1 register		
dloop:
	addi x6,x6,-1			# subtract 1 from t1
	bne x6,zero,dloop		# if t1 not equal to 0 branch to label loop
	lw x6,4(sp)			# pop x6
	lw ra,0(sp)			# pop ra
	addi sp,sp,8			# sp back 2 words
	ret				# return to caller
###########################################################
name:
string SAJEEV SANKARAN CH32V003 UART
eol:
bytes 0x0d,0x0a,0x00
##########################################################
TIM2_IRQhandler:
	#clear exti interrupt flag
	addi sp,sp,-60    		# push all registers
	sw x15,56(sp)
	sw x14,52(sp)
	sw x13,48(sp)
	sw x12,44(sp)
	sw x11,40(sp)
	sw x10,36(sp)
	sw x9,32(sp)
	sw x8,28(sp)
	sw x7,24(sp)
	sw x6,20(sp)
	sw x5,16(sp)
	sw x4,12(sp)
	sw x3,8(sp)
	sw x2,4(sp)
	sw x1,0(sp)
	
###########
	li x10,R16_TIM2_INTFR
	lw x11,0(x10)
	li x7,~1			# 0<<UIF
	and x11,x11,x7
	sw x11,0(x10)
	li x10,state
	lw x11,0(x10)
	addi x11,x11,1
	sw x11,0(x10)
	
############
	lw x1,0(sp)
	lw x2,4(sp)
	lw x3,8(sp)
	lw x4,12(sp)
	lw x5,16(sp)
	lw x6,20(sp)
	lw x7,24(sp)
	lw x8,28(sp)
	lw x9,32(sp)
	lw x10,36(sp)
	lw x11,40(sp)
	lw x12,44(sp)
	lw x13,48(sp)
	lw x14,52(sp)
	lw x15,56(sp)
	addi sp,sp,60
	longs 0x30200073		# mret (manually assembled opcode for mret as per RISCV spec)
	
#################

#################

TIM1UP_IRQhandler:
	#clear exti interrupt flag
	addi sp,sp,-60    		# push all registers
	sw x15,56(sp)
	sw x14,52(sp)
	sw x13,48(sp)
	sw x12,44(sp)
	sw x11,40(sp)
	sw x10,36(sp)
	sw x9,32(sp)
	sw x8,28(sp)
	sw x7,24(sp)
	sw x6,20(sp)
	sw x5,16(sp)
	sw x4,12(sp)
	sw x3,8(sp)
	sw x2,4(sp)
	sw x1,0(sp)
	
###########

	li x10,R16_TIM1_INTFR		# x10 points to timer1 inerrupt flag register
	lw x11,0(x10)
	li x7,~1			# 0<<UIF
	and x11,x11,x7			# clear timer1 update interrupt flag
	sw x11,0(x10)
	
	li x10,R16_TIM2_CNT		# point x10 to timer2 counter which has frequency count
	lw x11,0(x10)			# copy value to x11
	li x7,0				# load x7 with 0
	sw x7,0(x10)			# clear timer2 counter

	li x10,state			# point x10 to state
	lw x7,0(x10)			# copy state , how many times timer2 overflowed 
	li x8,0xffff			# load x8 with 65535,full count of timer2 before overflow
	beqz x7,no_overflow		# if x7(state) is 0, no overflow , frequency measured is less than 65535Hz, branch to no_overflow


# this block of code prints out number of external pulse overflows(which happens if signal crosses 65535Hz) recorded in state register,x10 pointing state
########li x10,result1			# load address of result1 which holds the period between 1st and 2nd time stamp
	li t1,4				# t1 counter = 4
	
readloop2:
	lb t2,3(x10)			# loads 1 byte to t2 from result
	call bin_to_ascii		# convert bin to ascii
	addi x10,x10,-1			# decrease pointer
	addi t1,t1,-1			# decrease counter
	bnez t1,readloop2		# loop till counter is 0
	li x8,0x0d			# line feed
	call USART_TX			# call uart
	li x8,0x0a			# carriage feed
	call USART_TX			# call uart

add_overflow:
	add x11,x11,x8			# add 65535 to current count in x11
	addi x7,x7,-1			# decrease x7 (total oevrflows recorded in state) by 1
	bnez x7,add_overflow		# if x7 not 0 loop to label add_overflow till all overflows are added to get total pulses counted in 1 second
no_overflow:
	li x10,result1			# point x10 to SRAM result1 register
	sw x11,0(x10)			# save final result based on 0 or non 0 overflow

	li x10,state			# point x10 to state register	
	sw zero,0(x10)			# clear state register 

XXXX:	
	call D_ASCII			# call D_ASCII subroutine to convert result1 value to decimal ASCII. uses 10 bytes of space
	call print			# prints value to USART

############		

	lw x1,0(sp)
	lw x2,4(sp)
	lw x3,8(sp)
	lw x4,12(sp)
	lw x5,16(sp)
	lw x6,20(sp)
	lw x7,24(sp)
	lw x8,28(sp)
	lw x9,32(sp)
	lw x10,36(sp)
	lw x11,40(sp)
	lw x12,44(sp)
	lw x13,48(sp)
	lw x14,52(sp)
	lw x15,56(sp)
	addi sp,sp,60
	longs 0x30200073		# mret (manually assembled opcode for mret as per RISCV spec)
###############################################
###############################################
# Prints 10 bytes from result1 to USART
###############################################
print:
	addi sp,sp,-20
	sw ra,16(sp)
	sw x11,12(sp)
	sw x10,8(sp)
	sw x8,4(sp)
	sw t1,0(sp)


	li x10,result1		# point to address mem ,top byte stored in mem0 lowest byte in mem+9, need to print top byte 1st
	li t1,0			# byte counter loaded with 10 , total 10 bytes to be printed
	li x15,10		# max count of 10bytes in x15

Z:
	lb x8,0(x10)		# load byte from result1
	
print1:
	addi x10,x10,1		# increase the address by 1 byte
	addi t1,t1,1		# decrease the byte counter
	call USART_TX		# call uart
	bne t1,x15,Z
	li x8,' '
	call USART_TX		# call uart
	li x8,'H'
	call USART_TX		# call uart
	li x8,'Z'
	call USART_TX		# call uart
	li x8,0x0d		# line feed
	call USART_TX		# call uart
	li x8,0x0a		# carriage feed
	call USART_TX	
	lw t1,0(sp)
	lw x8,4(sp)
	lw x10,8(sp)
	lw x11,12(sp)
	lw ra,16(sp)
	addi sp,sp,20
	ret
#########################################################################
# D_ASCII subroutine for converting binary in result1 to DECIMAL (ASCII)
#########################################################################

D_ASCII:			
	addi sp,sp,-32
	sw ra,28(sp)
	sw x15,24(sp)
	sw x11,20(sp)
	sw x8,16(sp)
	sw x7,12(sp)
	sw x5,8(sp)
	sw x4,4(sp)
	sw t1,0(sp)
	li x4,0			# clear register
	li x5,0			# clear register
	li x7,0			# clear register
	li x8,0			# clear register
	li x15,0		# clear register
	
	li x10,0x20000010	# result1
	lw x4,0(x10)		# copy result1 to x4
#	li x4,0xffffffff	# 32bit word to be converted into ascii chars
	li x7,1000000000	# divisor
Y1:
	sub x8,x4,x7		# subtract divisor from word to be converted
	sltu x5,x4,x7		# if result negative set x5 to indicate cannot be divided
	bnez x5,X1		# if result negative(not divisible) branch to X1
	mv x4,x8		# else move remainder to x4 for further division/subtraction
	addi x15,x15,1		# increase result by 1
	j Y1			# jump to label Y1 till not divisible
X1:
	addi x15,x15,0x30	# add ascii 0 to result to convert to ASCII char
	li x10,result1		# set pointer x10 to SRAM register mem to store the byte
	sb x15,0(x10)		# store byte in mem+0
	li x15,0		# clear result
	li x7,100000000		# load x7 with new divisor
Y2:
	sub x8,x4,x7		# subtract divisor from word to be converted
	sltu x5,x4,x7		# if result negative set x5 to indicate cannot be divided
	bnez x5,X2		# if result negative(not divisible) branch to X2
	mv x4,x8		# else move remainder to x4 for further division/subtraction
	addi x15,x15,1		# increase result by 1
	j Y2			# jump to label Y2 till not divisible
X2:
	addi x15,x15,0x30	# add ascii 0 to result to convert to ASCII char
	addi x10,x10,1		# set pointer x10 to SRAM register mem+1 to store the byte
	sb x15,0(x10)		# store byte in mem+1
	li x15,0		# clear result
	li x7,10000000		# load x7 with new divisor
Y3:
	sub x8,x4,x7		# subtract divisor from word to be converted
	sltu x5,x4,x7		# if result negative set x5 to indicate cannot be divided
	bnez x5,X3		# if result negative(not divisible) branch to X3
	mv x4,x8		# else move remainder to x4 for further division/subtraction
	addi x15,x15,1		# increase result by 1
	j Y3			# jump to label Y3 till not divisible
X3:
	addi x15,x15,0x30	# add ascii 0 to result to convert to ASCII char
	addi x10,x10,1		# set pointer x10 to SRAM register mem+2 to store the byte
	sb x15,0(x10)		# store byte in mem+2
	li x15,0		# clear result
	li x7,1000000		# load x7 with new divisor
Y4:
	sub x8,x4,x7		# subtract divisor from word to be converted
	sltu x5,x4,x7		# if result negative set x5 to indicate cannot be divided
	bnez x5,X4		# if result negative(not divisible) branch to X4
	mv x4,x8		# else move remainder to x4 for further division/subtraction
	addi x15,x15,1		# increase result by 1
	j Y4			# jump to label Y4 till not divisible
X4:
	addi x15,x15,0x30	# add ascii 0 to result to convert to ASCII char
	addi x10,x10,1		# set pointer x10 to SRAM register mem+3		
	sb x15,0(x10)		# store byte in mem+3
	li x15,0		# clear result
	li x7,100000		# load x7 with new divisor
Y5:
	sub x8,x4,x7		# subtract divisor from word to be converted
	sltu x5,x4,x7		# if result negative set x5 to indicate cannot be divided
	bnez x5,X5		# if result negative(not divisible) branch to X5
	mv x4,x8		# else move remainder to x4 for further division/subtraction
	addi x15,x15,1		# increase result by 1
	j Y5			# jump to label Y5 till not divisible
X5:
	addi x15,x15,0x30	# add ascii 0 to result to convert to ASCII char
	addi x10,x10,1		# set pointer x10 to SRAM register mem+4 to store the byte
	sb x15,0(x10)		# store byte in mem+4
	li x15,0		# clear result
	li x7,10000		# load x7 with new divisor
Y6:
	sub x8,x4,x7		# subtract divisor from word to be converted
	sltu x5,x4,x7		# if result negative set x5 to indicate cannot be divided
	bnez x5,X6		# if result negative(not divisible) branch to X6
	mv x4,x8		# else move remainder to x4 for further division/subtraction
	addi x15,x15,1		# increase result by 1
	j Y6			# jump to label Y6 till not divisible
X6:
	addi x15,x15,0x30	# add ascii 0 to result to convert to ASCII char
	addi x10,x10,1		# set pointer x10 to SRAM register mem+5 to store the byte
	sb x15,0(x10)		# store byte in mem+5
	li x15,0		# clear result
	li x7,1000		# load x7 with new divisor
Y7:
	sub x8,x4,x7		# subtract divisor from word to be converted
	sltu x5,x4,x7		# if result negative set x5 to indicate cannot be divided
	bnez x5,X7		# if result negative(not divisible) branch to X7
	mv x4,x8		# else move remainder to x4 for further division/subtraction
	addi x15,x15,1		# increase result by 1
	j Y7			# jump to label Y7 till not divisible
X7:
	addi x15,x15,0x30	# add ascii 0 to result to convert to ASCII char
	addi x10,x10,1		# set pointer x10 to SRAM register mem+6 to store the byte
	sb x15,0(x10)		# store byte in mem+6
	li x15,0		# clear result
	li x7,100		# load x7 with new divisor
Y8:
	sub x8,x4,x7		# subtract divisor from word to be converted
	sltu x5,x4,x7		# if result negative set x5 to indicate cannot be divided
	bnez x5,X8		# if result negative(not divisible) branch to X8
	mv x4,x8		# else move remainder to x4 for further division/subtraction
	addi x15,x15,1		# increase result by 1
	j Y8			# jump to label Y8 till not divisible
X8:
	addi x15,x15,0x30	# add ascii 0 to result to convert to ASCII char
	addi x10,x10,1		# set pointer x10 to SRAM register mem+7 to store the byte
	sb x15,0(x10)		# store byte in mem+7
	li x15,0		# clear result
	li x7,10		# load x7 with new divisor
Y9:
	sub x8,x4,x7		# subtract divisor from word to be converted
	sltu x5,x4,x7		# if result negative set x5 to indicate cannot be divided
	bnez x5,X9		# if result negative(not divisible) branch to X9
	mv x4,x8		# else move remainder to x4 for further division/subtraction
	addi x15,x15,1		# increase result by 1
	j Y9			# jump to label Y9 till not divisible
X9:
	addi x15,x15,0x30	# add ascii 0 to result to convert to ASCII char
	addi x10,x10,1		# set pointer x10 to SRAM register mem+8 to store the byte
	sb x15,0(x10)		# store byte in mem+8
	li x15,0		# clear result
	mv x15,x4
X10:
	addi x15,x15,0x30	# add ascii 0 to result to convert to ASCII char
	addi x10,x10,1		# set pointer x10 to SRAM register mem+9 to store the byte	
	sb x15,0(x10)		# store byte in mem+9

	lw t1,0(sp)
	lw x4,4(sp)
	lw x5,8(sp)
	lw x7,12(sp)
	lw x8,16(sp)
	lw x11,20(sp)
	lw x15,24(sp)
	lw ra,28(sp)
	addi sp,sp,32
	ret
######################################################
##########################################################################################################
# converts 1 byte into ASCII represented hexadecimal value
##########################################################################################################
bin_to_ascii:
	addi sp,sp,-4
	sw ra,0(sp)
	mv a3,t2
	andi a3,a3,0xf0
	srli a3,a3,4
	slti a4,a3,10			# set a4 to 1 if a3 is less than 10 ,10and higher a4=0
	beqz a4 ,letter1
	ori a3,a3,0x30
	#mv a0,a3
	mv x8,a3
	call USART_TX
	j low_nibble
letter1:
	addi a3,a3,0x37
	#mv a0,a3
	mv x8,a3
	call USART_TX
low_nibble:
	mv a3,t2
	andi a3,a3,0x0f
	slti a4,a3,10			# set a4 to 1 if a3 is less than 10 ,10and higher a4=0
	beqz a4 ,letter2
	ori a3,a3,0x30
	#mv a0,a3
	mv x8,a3
	call USART_TX
	j exit_bin_to_ascii
letter2:
	addi a3,a3,0x37
	#mv a0,a3
	mv x8,a3
	call USART_TX
exit_bin_to_ascii:
	lw ra,0(sp)
	addi sp,sp,4
	ret
###############################################