/*
	Antonio Cashiola
	SR04 Ultrasonic Sensor
	todo:  when position of the device in front of the sonic sensor changes an arbitrary value is read.

*/

#include <ets_sys.h>
#include <osapi.h>
#include <os_type.h>
#include <gpio.h>
#include "driver/uart.h"
//#include "driver/esp_system.h"
//#include "c_types.h"
//#include "user_interface.h"


// see eagle_soc.h for these definitions
#define LED_GPIO 2
#define LED_GPIO_MUX PERIPHS_IO_MUX_GPIO2_U
#define LED_GPIO_FUNC FUNC_GPIO2

#define TRIGGER_GPIO5 5
#define SONIC_GPIO_MUX5 PERIPHS_IO_MUX_GPIO5_U
#define GPIO_CHARGE_SONIC FUNC_GPIO5

#define ECHO_GPIO4 4
#define SONIC_GPIO_MUX4 PERIPHS_IO_MUX_GPIO4_U
#define GPIO_READ_SONIC FUNC_GPIO4

#define CHARGEDELAY 100
#define RTC_MAGIC 0x55aaaa55

LOCAL os_timer_t CHARGE_timer;

uint32 gpio_status;
uint32 gpio_status2;
uint32 old_time;
uint32 new_time;
uint32 total_time;
uint32 last_distance;
uint32 last_timer ;
uint32 difference;
uint32 new_distance;
uint32 new_timer;


uint32 real_time_clock_old,real_time_clock_new;   // real time clock old and new
uint32 st1,st2;        							  // system time 1 and 2
uint32 cal1, cal2;								  // tick time 1 and 2


extern int ets_uart_printf(const char *fmt, ...);
void user_init(void);
void charge_ultra (void);
void read_sonicSensor (void);

typedef struct {
	  uint64 time_acc;
	  uint32 magic;
	  uint32 time_base;
	} RTC_TIMER_DEMO;


void ICACHE_FLASH_ATTR user_rf_pre_init(void)
{
}

void ICACHE_FLASH_ATTR charge_ultra (void)  // called every 1000 millesecond by CHARGETIMER
{

	gpio_output_set(BIT5, 0, BIT5, 0);                 // write GPIO5 high
	os_delay_us(30);				                   // delay let Ultrasonic Charge
	gpio_output_set(0, BIT5, BIT5, 0);                 // write GPIO5 Low
	real_time_clock_old = system_get_rtc_time();       // Get System REAL TIME CLOCK time (old)
	st1 = system_get_time();			               // get system Time
	GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS,BIT(4));   // clear interrupt
	ETS_GPIO_INTR_ENABLE(); // Enable gpio interrupts  // enable interrupts


}

void ICACHE_FLASH_ATTR interupt_sonic (void)
{

	ETS_GPIO_INTR_DISABLE();                           // Disable interrupts
	real_time_clock_new = system_get_rtc_time();       // get System REAL TIME CLOCK time (new)
	st2 = system_get_time();						   // get system Time
	cal1 = system_rtc_clock_cali_proc();			   // get tick time (5.9~ us)
	read_sonicSensor();								   // call read_sonicSensor to calc Info
}

void ICACHE_FLASH_ATTR read_sonicSensor (void)
{
		cal2 =  (uint64)((cal1*1000)>>12);
	    old_time =(uint32)( real_time_clock_new-real_time_clock_old);  // new time - old time
		new_distance = old_time * cal2;                                // distance = clock cycles * clock tick time.
		// distance = speed * TIME/2  --> speed of sound at sea level is 343m/s or 34300cm/s
		uint32 confirmed_distance = (343/100 * (new_distance/200000));
		ets_uart_printf("confirmed distance = %d \r\n", confirmed_distance);
		last_distance = new_distance;  // remember old time
}

void ICACHE_FLASH_ATTR user_init(void){

	// Configure the UART
	uart_init(BIT_RATE_115200, BIT_RATE_115200);
	// init GPIO
	 gpio_init();

	PIN_FUNC_SELECT(LED_GPIO_MUX, LED_GPIO_FUNC );           // SET GPIO 2 TO OUTPUT ( LED )
	PIN_FUNC_SELECT(SONIC_GPIO_MUX4, GPIO_READ_SONIC );      // SET GPIO 4 TO INPUT (GPIO_READ_SONIC )
	gpio_output_set(0, 0, 0, GPIO_ID_PIN(4));                // Set GPIO0 as input

	PIN_FUNC_SELECT(SONIC_GPIO_MUX5, GPIO_CHARGE_SONIC);     // SET GPIO 5 TO OUTPUT (CHARGE  SONIC SENSOR)
	PIN_PULLUP_EN(PERIPHS_IO_MUX_GPIO4_U);                   //enable pin pullup  ?? needed?

	 // Set up a timer to CHARGE THE SONIC SENSOR
	os_timer_disarm(&CHARGE_timer);						     //disarm the timer
	os_timer_setfn(&CHARGE_timer, (os_timer_func_t *)charge_ultra, (void *)0);	//set function to call with timer flag
	os_timer_arm(&CHARGE_timer, CHARGEDELAY, 1);            // set timer length

	//setup interrupt for echo
	 ETS_GPIO_INTR_DISABLE(); 								// Disable gpio interrupts
	 ETS_GPIO_INTR_ATTACH(interupt_sonic , 4);			    // GPIO4 interrupt handler
	 GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, BIT(0));      // clear interput status
	 gpio_pin_intr_state_set(GPIO_ID_PIN(4),2);             // Interrupt on any GPIO4 on neg-edge.  lolevel doesnt work
}








