/* FreeRTOS 10 Tiva Demo
 *
 * main.c
 *
 * Andy Kobyljanec
 *
 * This is a simple demonstration project of FreeRTOS 8.2 on the Tiva Launchpad
 * EK-TM4C1294XL.  TivaWare driverlib sourcecode is included.
 */
#if 0
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
static const uint8_t ucIPAddress[ 4 ] = { 172, 17, 0, 2 };
static const uint8_t ucNetMask[ 4 ] = { 255, 0, 0, 0 };
static const uint8_t ucGatewayAddress[ 4 ] = { 172, 17, 0, 1 };
const uint8_t ucMACAddress[6]={0x0,0x1a,0xb6,0x3,0x35,0x99};
xSemaphoreHandle test_sem1,test_sem2,shared,adc_ready;
//*****************************************************************************
//
// The interrupt handler for the Ethernet interrupt.
//
//*****************************************************************************

void
Timer0AIntHandler(void) // Timer runs at 16 Khz
{
    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    static uint32_t i=0;
    i++;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if(i%2==0)  // Trigger at 8 Khz
    {
        xSemaphoreGiveFromISR(test_sem1,&xHigherPriorityTaskWoken);
    }

    if(i%16==0) // Trigger at 1 KHz
    {
        xSemaphoreGiveFromISR(test_sem2,&xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}
uint16_t buff[2][8];
uint16_t cbuff[16];
int rd=0,wr=0;
int buff_idx=0;
int in_use=1;
int ready=1;
void ADCIntHandler()
{
    uint32_t res;
    static uint32_t cntr=0;
    ADCIntClear(ADC0_BASE, 3);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    //
    // Read the value from the ADC.
    //
    xSemaphoreGiveFromISR(adc_ready,&xHigherPriorityTaskWoken);
    /*
    ADCSequenceDataGet(ADC0_BASE, 3, &res);
    if(buff_idx==0)
    {
        in_use = (in_use+1)&1;
        //xSemaphoreGiveFromISR(test_sem2,&xHigherPriorityTaskWoken);
    }
    cntr=(cntr+1)%26;
    */
   // buff[in_use][buff_idx]='A'+cntr;
   // buff_idx=(buff_idx+1)&7;
    //cbuff[wr]='A'+cntr;
    //wr=(wr+1)&15;
}

TaskHandle_t ADChandle;
static void vUDPSendUsingStandardInterface( void *pvParameters )
{
Socket_t xSocket;
struct freertos_sockaddr xDestinationAddress;
uint8_t cString[ 120 ];
uint32_t start,end,diff;
uint32_t not_in_use,ret;
TaskStatus_t xTaskDetails;
const TickType_t x1000ms = 1000UL / portTICK_PERIOD_MS;

   /* Send strings to port 10000 on IP address 192.168.0.50. */
   xDestinationAddress.sin_addr = FreeRTOS_inet_addr( "172.17.0.6" );
   xDestinationAddress.sin_port = FreeRTOS_htons( 10000 );

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
   int cntr;
   uint16_t rdbuff[8];
   for( ;; )
   {
       /* Create the string that is sent. */
       cntr=0;

       xSemaphoreTake(test_sem2,16 );
       start=xTaskGetTickCount();
       //sprintf( cString,
       //         "Standard send message number %lurn",
       //         ulCount );

       /* Send the string to the UDP socket.  ulFlags is set to 0, so the standard
       semantics are used.  That means the data from cString[] is copied
       into a network buffer inside FreeRTOS_sendto(), and cString[] can be
       reused as soon as FreeRTOS_sendto() has returned. */

       if(xSemaphoreTake(shared,portMAX_DELAY )==1)
       {
       not_in_use = (in_use+1)&1;
       xSemaphoreGive(shared);
       }
       /*
       while(cntr<8)
       {
           rdbuff[cntr]=cbuff[rd];
           rd=(rd+1)&15;
           cntr++;
       }*/
       ret=FreeRTOS_sendto( xSocket,
                         buff[not_in_use],
                        //rdbuff,
                        16,
                        0,
                        &xDestinationAddress,
                        sizeof( xDestinationAddress ) );


  //     end=xTaskGetTickCount();

//       diff=end-start;

       //ulCount++;

       /* Wait until it is time to send again. */
       //vTaskDelay( 10 );
   }
}

void ADCsample( void *pvParameters)
{//

    uint16_t res;
    static uint32_t cntr=0;
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
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0,
    ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH0);
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE, 3);
    IntEnable(INT_ADC0SS3);
    // Trigger the sample sequence.
    ADCIntEnable(ADC0_BASE,3);

    for( ;; )
    {
        if(xSemaphoreTake(test_sem1,2 )==1)
        {
        //ready=0;
        ADCProcessorTrigger(ADC0_BASE, 3);

        xSemaphoreTake(adc_ready,portMAX_DELAY );
        ADCSequenceDataGet(ADC0_BASE, 3, &res);
         if(buff_idx==0)
         {
             if(xSemaphoreTake(shared,portMAX_DELAY )==1)
             {
             in_use = (in_use+1)&1;
             xSemaphoreGive(shared );
             }
             //xSemaphoreGiveFromISR(test_sem2,&xHigherPriorityTaskWoken);
         }
         cntr=(cntr+1)%26;
         buff[in_use][buff_idx]='A'+cntr;
         buff_idx=(buff_idx+1)&7;

        //UARTprintf("a\r\n");
        // Wait until the sample sequence has completed.
        //
        }
        else
        {
            UARTprintf("Failed to get sema\r\n");
        }

       // for(i=0;i<8;i++)
       // UARTprintf("ADC %u\r\n",buff[0]);
    }
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
int main(void)
{
    // Initialize system clock to 120 MHz
    test_sem1=xSemaphoreCreateBinary();
    test_sem2=xSemaphoreCreateBinary();
    shared=xSemaphoreCreateMutex();
    adc_ready=xSemaphoreCreateBinary();
    ConfigureUART();
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
    TimerLoadSet(TIMER0_BASE, TIMER_A, 1000);
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER0_BASE, TIMER_A);
    TimerIntRegister(TIMER0_BASE,TIMER_A,Timer0AIntHandler);

    //BaseType_t res=_ethernet_mac_get(ucMACAddress);
    //if(res!=pdPASS)
      //  UARTprintf("get mac addr failed\r\n");


    // Initialize the GPIO pins for the Launchpad
    PinoutSet(false, false);

    // Create demo tasks
    /*
    xTaskCreate(demoLEDTask, (const portCHAR *)"LEDs",
                configMINIMAL_STACK_SIZE, NULL, 1, NULL);

    xTaskCreate(demoSerialTask, (const portCHAR *)"Serial",
                configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    */
    xTaskCreate(vUDPSendUsingStandardInterface,(const portCHAR *)"Udp",
                configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, NULL);
    xTaskCreate(ADCsample,(const portCHAR *)"adc",
                configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+2, &ADChandle);
    TimerEnable(TIMER0_BASE, TIMER_A);
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
