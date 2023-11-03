# works fine but output is hex in terminal-period and frequency
#An example of channel 1 to illustrate the steps to use the input capture mode is as follows.
#1) Configure the CCxS domain to select the source of the ICx signal. For example, set it to 10b and select TI1FP1 as the source of IC1, not using the default setting, the CCxS domain defaults to making the comparison capture module the output channel.
#2) Configure the ICxF domain to set the digital filter for the TI signal. The digital filter will sample the signal at a determined frequency, a determined number of times, and then output a hop. This sampling
#frequency and number of times is determined by ICxF.
#3) Configure the CCxP bit to set the polarity of the TIxFPx. For example, keeping the CC1P bit low and selecting rising edge jumps.
#4) Configure the ICxPS domain to set the ICx signal to be the crossover factor between ICxPS. For example, keeping ICxPS at 00b, without crossover.
#5) Configure the CCxE bit to allow capturing the value of the core counter (CNT) into the compare capture register. Set the CC1E bit.
#6) Configure the CCxIE and CCxDE bits as needed to determine whether to allow enable interrupts or DMA.This completes the comparison capture channel configuration.
#52.1 = 9600
#Default mapping (CH1/ETR/PD4, CH2/PD3,CH3/PC0, CH4/PD7).
fclk = 24000000   # 24Mhz RCO internal , AHB =8Mhz by default
state = 0x2000000C
result1 = 0x20000010
result2 = 0x20000014
result_high = 0x20000018 
result_low = 0x2000001C
#USARTx_TX Full-duplex mode Push-pull multiplexed outputs
#USARTx_RX Full-duplex mode Floating input or pull-up input
#PD5 -tx
#PD6 -rx
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
  longs   0x00000000 #pack <l TIM1UP_IRQhandler
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
	li x7,((1<<2)|(1<<4)|(1<<5)|(1<<14)|(1<<0))	# 1<<IOPA_EN,1<<IOPC_EN,1<<IOPD_EN,1<<USART_EN
	or x11,x11,x7			# or values 
	sw x11,0(10)			# store modified enable values in R32_RCC_APB2PCENR
	li x10,R32_RCC_APB1PCENR
	lw x11,0(x10)
	ori,x11,x11,(1<<0)		# timer2 clock enable
	sw x11,0(x10)

#configure GPIO 
	li x10,R32_GPIOD_CFGLR		# load pointer x10 with address of R32_GPIOD_CFGLR , GPIO configuration register
	lw x11,0(x10)			# load contents from register pointed by x10
	li x7,~((0xf<<20)|(0xf<<24)|(0xf<<16)|(0xf<<12))	#clear pd3,pd4,pd5,pd6. we need to setup PD5 & PD6 for usart tx and rx and pd4 for led & pd3 for input capture
	and x11,x11,x7			# clear pd3,pd4,pd5,pd6 mode and cnf bits for selected pin D4,D5,D6
	li x7,((0x8<<24)|(0xB<<20)|(0x3<<16)|(0x4<<12))	# pd6 = input with PU/PD,pd5= multiplex pushpull output 50mhz,pd4 pushpull,pd3 floating input for input capture
	or x11,x11,x7			# OR value to register
	sw x11,0(x10)			# store in R32_GPIOD_CFGLR

	li x10,R32_GPIOD_BSHR		# R32_GPIOD_BSHR register sets and resets GPIOD pins, load address into pointer x10
	lw x11,0(x10)			# load contents to x11
	li x7,(1<<4)			# set pd4 by shifting 1 to bit position 4
	or x11,x11,x7			# OR with x11
	sw x11,0(x10)			# store x11 to R32_GPIOD_BSHR
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


	li x10,state
	li x11,1
	sw x11,0(x10)
timer_for_1s:
	li x10,R16_TIM2_PSC
	li x11,(7999)			# fck_PSC/PSC[15:0]+1,8MHz/7+1 =1MHz , 1ms resolution for each click
	sw x11,0(x10)
	li x10,R16_TIM2_ATRLR		# auto reload register to reload the initialization value for the CNT at the end of each counting cycle
	li x11,(999)			# max reload
	sw x11,0(x10)
	li x10,R16_TIM2_DMAINTENR
	lw x11,0(x10)
	li x7,(1<<2)			# enable CC2IE interrupt,ch2 compare
	or x11,x11,x7
	sw x11,0(x10)
	li x10,R16_TIM2_CHCTLR1
	lw x11,0(x10)
	li x7,((1<<8))			# comparison capture channel 2 is configured as an input and IC2 is mapped onTI1
	or x11,x11,x7
	sw x11,0(x10)
	li x10,R16_TIM2_CCER
	lw x11,0(x10)
	li x7,(1<<4)			# enable cc2 input capture
	or x11,x11,x7
	sw x11,0(x10)
	li x10,R16_TIM2_INTFR
	lw x11,0(x10)
	li x7,0xFFFFFFFA		# 0<<UIF ,clear update interrupt flag
	and x11,x11,x7
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
	li x7,(1<<6 )			# enabled  TIM2 interrupts
	or x11,x11,x7
	sw x11,0(x10)			# store back new values

# enabling GLOBAL INTERRUPTS
	li t0, 0x88			# load MPIE and MIE bits , 1<<MIE in mstatus is enabling GLOBAL INTERRUPTS
	longs 0x30029073        	# csrw	mstatus,t0 ,manually assembled opcade to do csrrw the values in t0,  

	li x10,state
	li x7,0
	sw x7,0(x10)

#main endless loop for uart transmit
example:

	li x10,name			# load address of label "name" to x10, string to be transmitted
string_loop:
	lb x8,0(x10)			# load 1 byte from 0 offset of "name"
	beqz x8,finish			# if byte in x8 null branch to label "finish"
	call USART_TX			# call subroutine USART_TX to transmit byte
	addi x10,x10,1			# increase pointer by 1 byte
	j string_loop			# jump back to label string_loop until null is encountered
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

########################################
# Blinks LED on pd4, used for debugging
########################################
PD4_ON:
	addi sp,sp,-16			# move sp 16 bytes downward(4 words)
	sw ra,0(sp)			# push ra
	sw x7,4(sp)			# push x7
	sw x10,8(sp)			# push x10
	sw x11,12(sp)			# push x11
	li x10,R32_GPIOD_BSHR		# R32_GPIOD_BSHR register sets and resets GPIOD pins, load address into pointer x10
	lw x11,0(x10)			# load contents to x11
	li x7,1<<20			# reset pd4 by shifting 1 into bit position 20 of R32_GPIOD_BSHR
	or x11,x11,x7			# OR with x11
	sw x11,0(x10)			# store x11 to R32_GPIOD_BSHR
	

	call delay			# delay subroutine

PD4_OFF:
	li x10,R32_GPIOD_BSHR		# R32_GPIOD_BSHR register sets and resets GPIOD pins, load address into pointer x10
	lw x11,0(x10)			# load contents to x11
	li x7,(1<<4)			# set pd4 by shifting 1 to bit position 4
	or x11,x11,x7			# OR with x11
	sw x11,0(x10)			# store x11 to R32_GPIOD_BSHR

	

	call delay			# delay subroutine

	lw x11,12(sp)			# pop x11
	lw x10,8(sp)			# pop x10
	lw x7,4(sp)			# pop x7
	lw ra,0(sp)			# pop ra
	addi sp,sp,16			# move sp back 4 words
	ret				# return to caller
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
###########################################################
name:
string SAJEEV SANKARAN CH32V003 UART
eol:
bytes 0x0d,0x0a,0x00
##########################################################
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
	li x10,R16_TIM2_INTFR		# load address of timer2 interrupt flag
	lw x11,0(x10)			# copy word from flag register
	li x7,0xFFFFFBFD		# 0<<CC2OF|0<<UIF| 0<<CC2IF
	and x11,x11,x7			# write to flag register 0
	sw x11,0(x10)			# store back
	li x10,R16_TIM2_CH2CVR		# comparison capture registers (CHxCVR) that support comparison with the main counter (CNT) to output pulses
	lw x11,0(x10)			# read compare capture value
	li x10,state			# laod x10 with address of state register
	lw x7,0(x10)			# copy state register to x7
	beqz x7,result1str		# branch to result1str if x7 is 0
	li x10,result2			# if state register is not 0 load address of result2 as the current interrupt is 2nd time stamp
	sw x11,0(x10)			# store capture register value(2nd time stamp) in result2
	li x10,result1			# load address of result1 in x10
	lw x7,0(x10)			# copy result1 to x7
	sub x11,x11,x7			# subtract x7(result1) from x11(result2) which gives period
	sw x11,0(x10)			# difference in result1
	li x10,state			# load pointer x10 with address of state register
	li x7,0				# load 0 in x7
	sw x7,0(x10)			# store 0 in state
	j out				# jump to label out
result1str:
	li x10,result1			# point x10 to result1
	sw x11,0(x10)			# x11 has the 1st time stamp, store in result1
	li x10,state			# load x10 pointer with address of state register
	li x7,1				# load x7 with 1
	sw x7,0(x10)			# store 1 in state register so that next interrupt will be counted as 2nd time stamp
	j nexttime			# exit ISR by jumping to label nexttime
out:	
	li t2,'I'			# reach here only after reading the 2nd time stamp, load ascii I in t2
	call bin_to_ascii		# call asccii_to_bin to convert binary value in t2 to ascii value to be displayed in terminal
	li x8,0x0d			# new line
	call USART_TX			# call uart
	li x8,0x0a			# carriage return
	call USART_TX			# call uart

	li x10,result1			# load address of result1 which holds the period between 1st and 2nd time stamp
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

	call division			# call division routine to convert period to frequency and store in result1. 1000ms/period in result1 = frequency store back in result1
	li x10,result1			# pointer x10 points to result1 for printing frequency in terminal
	li t1,4				# initialize counter to 4, 4 bytes to be printed
	
readloop1:
	lb t2,3(x10)			# load 3rd byte
	call bin_to_ascii		# convert to ascii
	addi x10,x10,-1			# decrease pointer
	addi t1,t1,-1			# decrease counter
	bnez t1,readloop1		# loop till counter is 0
	li x8,0x0d			# line feed
	call USART_TX			# call uart
	li x8,0x0a			# carriage feed
	call USART_TX			# call uart
nexttime:
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

# a1 = nimber1 # low
# a2 = number2 # high
# a5 = workreg
# a3= low register = a2:a1
# a4 =hi register = a4:a3
# t1 carry

division:
	addi sp,sp,-24
	sw a1,20(sp)
	sw a2,16(sp)
	sw a3,12(sp)
	sw a4,8(sp)
	sw a5,4(sp)
	sw t1,0(sp)
	li t1,0				# initialize t1 to 0
	li t2,0				# initialize t2 to 0
	li a2,0				# initialize a2 to 0
	li a4,0				# initialize a4 to 0
	li a5,0				# initialize a5 to 0 
	li a1,1000			# divident x11
	li x10,result1			# divisor in result1,x10 points to sram result1
	lw a3,0(x10)			# load word in result1 to x13(divisor)
X:
	sub a5,a1,a3			# subtract divisor from dividend and store remainder in a5
	sltu t1,a1,a3			# set t1 if a1 is less than a3
	bnez t1,carry			# if t1 not equal 0 branch to carry
	mv a1,a5			# move remainder to a1 from a5
	sub a5,a2,a4			# subtract the high registers a4 from a2
	addi t2,t2,1			# increase t2 by 1 for each successful subtraction (result)
	J X				# loop to X till t1 is set
carry:
	li t0, result1			# load address of result1 in t0
	sw t2,0(t0)			# store result in t2 in result1
	li t0, result2			# load address of result2 in t0
	sw a1,0(t0)			# store a1 high byte of result in result2 = 0
	lw t1,0(sp)
	lw a5,4(sp)
	lw a4,8(sp)
	lw a3,12(sp)
	lw a2,16(sp)
	lw a1,20(sp)
	addi sp,sp,24
	ret