/*
 * adc_timer_main.c
 *
 *  Created on: May 1, 2023
 *      Author: hemanth
 */


/*
 * cbmain.c
 *
 *  Created on: May 1, 2023
 *      Author: hemanth
 */


/* FreeRTOS 10 Tiva Demo
 *
 * main.c
 *
 * Andy Kobyljanec
 *
 * This is a simple demonstration project of FreeRTOS 8.2 on the Tiva Launchpad
 * EK-TM4C1294XL.  TivaWare driverlib sourcecode is included.
 */

#if 1
#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "drivers/pinout.h"
#include "utils/uartstdio.h"
#include "driverlib/pin_map.h"
#include "gpio.h"
#include "uart.h"

// TivaWare includes
#include "driverlib/sysctl.h"
#include "driverlib/debug.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/timer.h"
#include "inc/hw_emac.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/emac.h"
// FreeRTOS includes
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "FreeRTOS_IP.h"
#include "task.h"
#include "queue.h"
#include "FreeRTOS_sockets.h"
#include "adc.h"
// Demo Task declarations
void demoLEDTask(void *pvParameters);
void demoSerialTask(void *pvParameters);
//*****************************************************************************
//
// A set of flags.  The flag bits are defined as follows:
//
//     0 -> An indicator that a SysTick interrupt has occurred.
//     1 -> An RX Packet has been received.
//     2 -> A TX packet DMA transfer is pending.
//     3 -> A RX packet DMA transfer is pending.
//
//*****************************************************************************
#define FLAG_SYSTICK            0
#define FLAG_RXPKT              1
#define FLAG_TXPKT              2
#define FLAG_RXPKTPEND          3
static volatile uint32_t g_ui32Flags;

/* Define the network addressing.  These parameters will be used if either
ipconfigUDE_DHCP is 0 or if ipconfigUSE_DHCP is 1 but DHCP auto configuration
failed. */
static const uint8_t ucIPAddress[ 4 ] = { 172, 17, 0, 5 };
static const uint8_t ucNetMask[ 4 ] = { 255, 0, 0, 0 };
static const uint8_t ucGatewayAddress[ 4 ] = { 172, 17, 0, 1 };
const uint8_t ucMACAddress[6]={0x0,0x1a,0xb6,0x3,0x35,0x99};
xSemaphoreHandle test_sem1,test_sem2,shared,adc_ready;
//*****************************************************************************
//
// The interrupt handler for the Ethernet interrupt.
//
//*****************************************************************************
int udp_acq=0,udp_rel=0;
int adc_acq=0,adc_rel=0;
int timer_a_ticks=0;
void
Timer0AIntHandler(void) // Timer runs at 8 Khz
{
    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    static uint32_t i=0;
    i++;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if(i%8==0) // Trigger at 1 KHz
    {
        xSemaphoreGiveFromISR(test_sem2,&xHigherPriorityTaskWoken);
        udp_rel++;
    }
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}
//uint16_t buff[2][8];
uint8_t cbuff[64];
int rd=0,wr=0;
int samples=0;


uint32_t adc_start,adc_end,adc_diff;
void ADCIntHandler()
{
    uint32_t res;

    ADCIntClear(ADC0_BASE, 3);

    //uint8_t *ptr=(char*)cbuff;

    ADCSequenceDataGet(ADC0_BASE, 3, &res);

    //cbuff[wr]=(uint16_t)res;
    res=res<<2;
    cbuff[wr]=g711_encode(res);
    wr=(wr+1)&63;
    samples++;

}

TaskHandle_t ADChandle;
static void vUDPSendUsingStandardInterface( void *pvParameters )
{
Socket_t xSocket;
struct freertos_sockaddr xDestinationAddress;


   /* Send strings to port 1234 on IP address 192.168.0.50. */
   xDestinationAddress.sin_addr = FreeRTOS_inet_addr( "172.17.0.6" );
   xDestinationAddress.sin_port = FreeRTOS_htons( 1234 );

   FreeRTOS_IPInit( ucIPAddress,
                    ucNetMask,
                    ucGatewayAddress,
                    0,
                    ucMACAddress );
   /* Create the socket. */
   xSocket = FreeRTOS_socket( FREERTOS_AF_INET,
                              FREERTOS_SOCK_DGRAM,/*FREERTOS_SOCK_DGRAM for UDP.*/
                              FREERTOS_IPPROTO_UDP );

   /* Check the socket was created. */
   configASSERT( xSocket != FREERTOS_INVALID_SOCKET );

   /* NOTE: FreeRTOS_bind() is not called.  This will only work if
   ipconfigALLOW_SOCKET_SEND_WITHOUT_BIND is set to 1 in FreeRTOSIPConfig.h. */

   uint8_t rdbuff[8];
   uint32_t start,end,ret,diff;
   uint8_t *ptr = (char*)cbuff;
#if 1
   for( ;; )
   {

       xSemaphoreTake(test_sem2,portMAX_DELAY );

       start=TimerValueGet(TIMER0_BASE, TIMER_A);

       memcpy(rdbuff,&cbuff[rd],8);
       rd=(rd+8)&63;

       ret=FreeRTOS_sendto( xSocket,
                         //buff[not_in_use],
                        rdbuff,
                        8,
                        0,
                        &xDestinationAddress,
                        sizeof( xDestinationAddress ) );

       if(ret<0)
           while(1);
       end=TimerValueGet(TIMER0_BASE, TIMER_A);;

       diff=(end-start)&(0xFFFFFFFF);

   }
#endif
}

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

uint32_t ulApplicationGetNextSequenceNumber(uint32_t a, uint16_t b, uint32_t c,uint16_t d)
{
    return a ^ b ^ c ^ d;
}

BaseType_t xApplicationGetRandomNumber(uint32_t *p) {
   *p = 0;
   return 1;
}

void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                    signed char *pcTaskName )
{
    while(1);
}
// Main function

void init_ADC()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
    //
    // Wait for the ADC0 module to be ready.
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0))
    {
    }
    //
    // Enable the first sample sequencer to capture the value of channel 0 when
    // the processor trigger occurs.
    //
    ADCClockConfigSet(ADC0_BASE,ADC_CLOCK_SRC_PIOSC|ADC_CLOCK_RATE_FULL,1);
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_TIMER, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0,
    ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH0);
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE, 3);
    IntEnable(INT_ADC0SS3);
    // Trigger the sample sequence.
    ADCIntEnable(ADC0_BASE,3);

}
int main(void)
{
    // Initialize system clock to 120 MHz
    test_sem1=xSemaphoreCreateBinary();
    test_sem2=xSemaphoreCreateBinary();
    shared=xSemaphoreCreateMutex();

    ConfigureUART();
    init_ADC();
    uint32_t output_clock_rate_hz;
    output_clock_rate_hz = SysCtlClockFreqSet(
                               (SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                                SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
                               SYSTEM_CLOCK);
    ASSERT(output_clock_rate_hz == SYSTEM_CLOCK);
    IntMasterEnable();

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerClockSourceSet(TIMER0_BASE,TIMER_CLOCK_SYSTEM);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, 2000);
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    TimerControlTrigger(TIMER0_BASE, TIMER_A, true);

    TimerIntRegister(TIMER0_BASE,TIMER_A,Timer0AIntHandler);
    TimerControlStall(TIMER0_BASE, TIMER_A, true);
    TimerEnable(TIMER0_BASE, TIMER_A);

    // Initialize the GPIO pins for the Launchpad
    PinoutSet(false, false);

    xTaskCreate(vUDPSendUsingStandardInterface,(const portCHAR *)"Udp",
                configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+2, NULL);

    vTaskStartScheduler();

    // Code should never reach this point
    return 0;
}


/*  ASSERT() Error function
 *
 *  failed ASSERTS() from driverlib/debug.h are executed in this function
 */
void __error__(char *pcFilename, uint32_t ui32Line)
{
    // Place a breakpoint here to capture errors until logging routine is finished
    while (1)
    {
    }
}
#endif
