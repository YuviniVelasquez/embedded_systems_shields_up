# Embedded Systems - Shields Up
## An analysis, debugging, and managment of software errors in embedded systems.

# Table of Content
1. [Introduction](#introduction)
2. [Setup](#setup)
3. [Debug Approach](#debug)
    1. [Fault 1: SETPOINT_HIGH](#debug1)
      1. [Analysis](#debug1_1)
      2. [Fault Debugging Approach](#debug1_2)
      3. [Evaluation of Effectiveness](#debug1_3)
    2. [Fault 2: PID_FX_GAINS](#debug2)
      1. [Analysis](#debug2_1)
      2. [Fault Debugging Approach](#debug2_2)
      3. [Evaluation of Effectiveness](#debug2_3)     
    3. [Fault 3: SETPOINT_HIGH](#debug3)
      1. [Analysis](#debug3_1)
      2. [Fault Debugging Approach](#debug3_2)
      3. [Evaluation of Effectiveness](#debug3_3)
4. [Conclusions](#conclusions)

## 1. Introduction <a name="introduction"></a>

There are many errors and faults that need to be prevented and managed when creating and designing a robust Embedded System. These error can be internal or external faults such as exploit attacks, device failures, bad programming practices, or even change of bits because of solar flare. The following project tackles some of the most common errors and provides a solution and analysis of this solution.

In this project, I modified the code provided that controlled a Freedom Developing board with NXP Kinetis processor with an LCD display and I handled common software errors in Embedded Systems. I layed out the debugging process, the tools I used, I provided with videos or oscilloscope screenshots to view the bugs, and I analized the efficiency of the solution I proposed.

<table>
  <tr style="text-align:center">
    <td>Freedom Development Board</td>
     <td>Shield PCB with Debugging Setup</td>
     <td>Freedom board with PCB, LCD, and AD2 as oscilloscope</td>
  </tr>
  <tr>
    <td><img src="https://github.com/YuviniVelasquez/embedded_systems_shields_up/blob/main/Images/freedom_developmen_board.jpg" width=450></td>
    <td><img src="https://github.com/YuviniVelasquez/embedded_systems_shields_up/blob/main/Images/shield_pcb_debugging_setup.jpg" width=450 ></td>
    <td><img src="https://github.com/YuviniVelasquez/embedded_systems_shields_up/blob/main/Images/board_with_LDC.jpg" width=270 ></td>
  </tr>
 </table>

To solve any of these errors, it was necessary to understand the architecture of the MCU and the implemented code. This allowed to have a better debugging approach.

<div style="text-align:center"><img src="https://github.com/YuviniVelasquez/embedded_systems_shields_up/blob/main/Images/Flowchart.jpg" /></div>
<div style="text-align:center">Architecture and Diagram with Perpherals</div>

## 2. Setup <a name="setup"></a>

This project runs on the [Freedom Development Board](https://www.digikey.com/en/products/detail/freescale-semiconductor-nxp/FRDM-KL25Z/3529594) which has a ARM Cortex-M0+ 32-Bit MCU.

To view the behavior of the board, I used an oscilloscope by using the [Analog Discovery 2](https://store.digilentinc.com/analog-discovery-2-100msps-usb-oscilloscope-logic-analyzer-and-variable-power-supply/) which allowed to view the behavior of signals and current through the LED via pins and the [PCB Shield](PDFs_and_Manuals/Freedom-KL25Z-Shield-v12-Hardware-Manual.pdf). The Analog Discovery board uses [Waveforms](https://reference.digilentinc.com/reference/software/waveforms/waveforms-3/start) as the virtual oscillloscope and waveform generator.

 I also used the [Keil uVision IDE](www.keil.com/mdk5/install) to code and debug the MCU board and it also it supports the debugging peripheral in the board called OpenSDA. With the OpenSDA platform, I could watch the Stack and I could add variables in the Watch List.

## 3. Debugging Approach <a name="debug"></a>

To manage these faults included creating a system that stores and verifies previous values, adding maximum and minimum value verifiers to keep the threshold, storing and verifying configuration periodically, and adding a mechanism to restart the system such as a Watchdog timer. 

## 3.1 Fault 1: SETPOINT_HIGH <a name="debug1"></a>

### 3.1.1 Analysis <a name="debug1_1"></a>
<table>

  <tr>
    <td><img src="https://github.com/YuviniVelasquez/embedded_systems_shields_up/blob/main/Images/1.1.jpg" width=500></td>
    <td><img src="https://github.com/YuviniVelasquez/embedded_systems_shields_up/blob/main/Images/1.2.jpg" width=500 ></td>
  </tr>
    <tr style="text-align:center">
    <td>Figure 1.1. Current goes unexpectedly high, there is no verification process where the set current is verified with its previous value, leaving room for error. </td>
     <td>Figure 1.2. Code implemented to review and correct unexpected current changes by comparing them to a previous recorded current.</td>
  </tr>
 </table>

The fault set_point_high manually changes the current setpoint which make the current go very high. On Figure 0.1 we can see how the current goes high for 1.745 ms. This simulates a case when the current is set to high by an unexpected event. After some current comparison in the **Control_HBLED** we can correct an unexpected high value.

### 3.1.2 Fault Debugging Approach <a name="debug1_2"></a>
To be able to solve this problem, I first needed to figure out the function that is updating the **g_set_current** value. This function happened to be [Update_Set_Current](Source/control.c#L295).

In here I created a copy of the set current called **g_set_current_copy** nonvolatile so it won’t be modified unexpectedly. This will help in the comparison of the g_set_current with the saved copy. The function **Update_Set_Current** is called by the [Thread_Buck_Update_Setpoint](Source/threads.c#L89) thread. 

**Thread_Buck_Update_Setpoint** is updated every four times the **ADC0_IRQHandler** runs. So, it is smarter to add verification of the code in **ADC0_IRQHandler**, but specifically in [Control_HBLED](Source/control.c#L98). This allows to find an unexpected current change faster which reduces the time where another handler can set the incorrect current. 

Additionally, a backup check is added in **Update_Set_Current** function where there is a comparison between the current **g_set_current** and the recorded current from last function call named **g_set_current_copy** to verify if the current is the value previously recorded. If that is not the case, then the **g_set_current_copy** recorded value is copied instead of the **g_set_current** value.


```cpp
volatile int g_set_current = 0;     //Default LED current value
int g_set_current_copy = 0;         //Default LED copy

//Setup and Verification of g_set_current_copy in Update_Set_Current 
void Update_Set_Current(void) {
	// Ramp curent up and down 
	static volatile int t=0;
	
	if (g_enable_flash){
		t++;
		if(g_set_current != g_set_current_copy){
			g_set_current = g_set_current_copy;
		}
    //...
		Set_DAC_mA(g_set_current);
		if (t >= g_flash_period)
			t = 0;
	}
	g_set_current_copy = g_set_current; //Saving current from present period
}

//Verification of g_set_current_copy in Control_HBLED
void Control_HBLED(void) {
	//verification if current has unesxpectedly changed
	if(g_set_current != g_set_current_copy){
			g_set_current = g_set_current_copy;
		}
	//...
}
```

### 3.1.3 Evaluation of Effectiveness <a name="debug1_3"></a>
As shown in Figure 1.2  this process eliminates non expected current values and there isn’t a spike in any of the currents. Other solutions were first tested but there was still a spike in the current. There is no visible signal on the oscilloscope of the high current output which shows that there is a good timing response for this solution. This is even with an oscilloscope visualization of 15uS per division.  

##  3.2 Fault 2: PID_FX_GAINS <a name="debug2"></a>

### 3.2.1 Analysis <a name="debug2_1"></a>

<table>
  <tr>
    <td><img src="https://github.com/YuviniVelasquez/embedded_systems_shields_up/blob/main/Images/2.1.jpg" width=500></td>
    <td><img src="https://github.com/YuviniVelasquez/embedded_systems_shields_up/blob/main/Images/2.2.jpg" width=500 ></td>
  </tr>
    <tr style="text-align:center">
    <td width= "50%">Figure 2.1. After the present fault, the gain of the current is off.  </td>
     <td width= "50%">Figure 2.2 Implemented a verification method that compares a previous PID value used in the last setup with the current value.</td>
  </tr>
 </table>

The fault **PID_FX_GAIN** changes the current’s gain. On Figure 2.1 we can see how the current starts incrementally changing its gain until the pattern is not as expected. This simulates a case when the SPid, which contains proportional gain, integral gain, and derivative gain, changes by an unexpected event. 

###  3.2.2 Fault Management Approach  <a name="debug2_2"></a>

To be able to solve this problem, I first needed to figure out where the values for gain are stored. I found out that values are stored in a struct type SPIDFX instantiated as **plantPID_FX** which contains the proportional gain, integral gain, and derivative gain among other values. These values are used in the function **UpdatePID_FX** to calculate the proportional, integral, and derivative terms with respect with the error_FX which is the now current error of this term.

A verification statement has been set in **UpdatePID_FX** in case **plantPID_FX** is unexpectedly modified. First, we needed to have some variables that will store the previous set up **plantPID_FX** value after the values have been used in **UpdatePID_FX**.   

These values are stored at the end of the **UpdatePID_FX** function to be read in the next call function. When the program runs for the first time, these values used to compare are set to zero, therefore we need to set up a first-time update. This was done using an IF statement with three OR statements which check if values are not zero. If values are zero, then the function runs as it’s supposed to for the first run, otherwise it verifies if the values stored are equal to the ones currently set in **plantPID_FX**. If not, then the stored values are given to plantPID_FX.   

This modification is implemented every time **Control_HBLED** is called and control mode is set to **PID_FX**. In **PID_FX** the function **UpdatePID_FX** is called and the verification that was implemented is used. 


```cpp
FX16_16 pGain_store = 0;	//Storing proportional gain as reference
FX16_16 pGain_store = 0;	//Storing integral gain as reference
FX16_16 pGain_store = 0;	//Storing derivative gain as reference

FX16_16 UpdatePID_FX(SPidFX * pid, FX16_16 error_FX, FX16_16 position_FX){

	FX16_16 pTerm, dTerm, iTerm, diff, ret_val;
//Verification if not initial default values
if (pGain_Store || iGain_store || dGain_Store){
    if(pGain_Store != pid->pGain){
        pid->pGain = pGain_Store; 
    }
    if(iGain_Store != pid->iGain){
        pid->iGain = iGain_Store; 
    }
    if(dGain_Store != pid->dGain){
        pid->dGain = dGain_Store; 
    }
}
//...
	ret_val = Add_FX(pTerm, iTerm);
	ret_val = Subtract_FX(ret_val, dTerm);
	//Storing Gain Values
  pGain_Store = pid->pGain;
  iGain_Store = pid->iGain;
  dGain_Store = pid->dGain;
  return ret_val;
}
```

###  3.2.3 Evaluation of Effectiveness  <a name="debug2_3"></a>

As shown in Figure 2.2 this process eliminates any unwanted change in the gain of current. This solution does not require a correction time as seen in the orange current. There is no visible signal on the oscilloscope of where the problem is, which shows that there is a good timing response for this solution. This is even with an oscilloscope visualization of 20mS per division.  

##  3.3 Fault 3: LCD_MUTEX  <a name="debug3"></a>

### 3.3.1 Analysis  <a name="debug3_1"></a>

 [Video Link 3.1](https://youtu.be/-7olpZ1rwAU)

 Getting into infinite loop because osMutexAdquire receives LCD_mutex where it gets stuck.

 [Video link 3.2: ](https://youtu.be/ak2Ju7Ihu5I)

The fault **TR_LCD_mutex** acquires the **LDC_mutex** and does not return it. On **Video link 3.1** we can see how the LCD gets stuck. This simulates a case when the mutex is stuck in an infinite loop. 

### 3.3.2 Fault Management Approach  <a name="debug3_2"></a>

To be able to solve this problem, I first needed to figure out in which mutex the code will get stuck. I noticed that it was the LCD that was stuck, and I also analyzed the fault code where it was stuck in. A WatchDog timer was implemented to avoid getting the MCU stuck at the LCD mutex. 

First, I included the code for a Watchdog timer in the program files called **COPT_WDT.c** and **COP_WDT.h** where the initialization and service of the WatchDog timer is located. After this, I disabled the function that by default disables the WatchDog by defining **DISABLE_WDOG** to 0.

To service the WatchDog timer, I found the function that was called the most by the LCD process by adding breakpoints to the functions used in the LCD. With this process I found that the function **LCD_Text_PrintChar** gets called more times than other LCD functions and I decided to service the timer there. 

```cpp
#ifndef COP_WDT_H
#define COP_WDT_H

void Init_COP_WDT(void);
void Service_COP_WDT(void);
void Flash_Reset_Cause(void);

#endif // COP_WDT_H

void Init_COP_WDT(void) {
	// Select 1 kHz clock and 1024 cycle time-out
	SIM->COPC = SIM_COPC_COPT(3) & ~SIM_COPC_COPCLKS_MASK & ~SIM_COPC_COPW_MASK; 
	}

void Service_COP_WDT(void) {
	SIM->SRVCOP = 0x55;
	SIM->SRVCOP = 0xaa;
}

#ifndef DISABLE_WDOG
  #define DISABLE_WDOG                 0
#endif

void SystemInit (void) {
#if (DISABLE_WDOG)
  /* SIM_COPC: COPT=0,COPCLKS=0,COPW=0 */
  SIM->COPC = (uint32_t)0x00u;
#endif /* (DISABLE_WDOG) */

void LCD_Text_PrintChar(PT_T * pos, char ch) {
	uint8_t glyph_index_entry;
	const uint8_t * glyph_data; // start of the data
#if BITS_PER_PIXEL == 1					// Copy bitmap byte directly
	PT_T cur_pos;
#endif
```

### 3.3.3 Evaluation of Effectiveness <a name="debug3_3"></a>

To visualize the reset, I set the oscilloscope which recorded the reset process. From the time the infinite loop starts to the reset there is a 605 ms time. This solution may reset and the MCU to which may cause the loss of current processes, but it is a great solution since most likely the OS was in a non-recoverable state.

<div style="text-align:center"><img src="https://github.com/YuviniVelasquez/embedded_systems_shields_up/blob/main/Images/3.1.jpg" /></div>
<div style="text-align:center">Restart process visualization with oscilloscope</div>

## 4. Conclusion and retrospective <a name="conclusions"></a>

This project was very interesting and engaging. I was able to use the embedded system concepts I learned in the debugging. I did not have any hardware, driver, or debuggger setup problems as I did in onther projects but, I did have to spend some time setting up the Analog Discovery 2 and the oscilloscope.
