#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/hrtimer.h>
#include <linux/math64.h>

//GPIO Defination
#define FRONT_LED_PIN 66
#define BACK_LED_PIN 65
#define FL_BTN_PIN 67
#define S_TRIG_PIN 69
#define FRONT_ECHO_PIN 68
#define BACK_ECHO_PIN 46
#define T_BTN_PIN 45
#define R_BTN_PIN 26

//LED FLASH Speed
#define INTERVAL 125


enum mode {
    OFFLINE,
    SENSING,
    MANUAL
};
enum status { 
    ALERT,
    SAFE
};

//Virable Declarations
static enum mode current_mode = MANUAL;  //Initial mode is MODE2, which is LED off
static enum status front_alert_state = SAFE;	//Initial alert status is SAFE
static enum status back_alert_state = SAFE;	//Initial alert status is SAFE
static struct task_struct *blink_thread;
static int fl_btn_irq, t_btn_irq, r_btn_irq;
static int choose_sensor=1; 	//Choose which sensor, initial=0
static bool front_sensor_active = false;  //Sensor initial mode is inactive
static bool back_sensor_active = false;  //Sensor initial mode is inactive

//LED Flashing Control
static int led_blink(void *data) 
{
    while (!kthread_should_stop()) 
    {	      
	switch (current_mode) 
        {
            case OFFLINE: //OFFLINE mode all LEDs off
		{
	    gpio_set_value(FRONT_LED_PIN, 0);
	    gpio_set_value(BACK_LED_PIN, 0);
            msleep(INTERVAL);
		break;
		}
	    case SENSING:  //SENSING mode both sensors working
		{
	if((front_alert_state == ALERT) && (back_alert_state == ALERT))
	{            
            gpio_set_value(FRONT_LED_PIN, 1);
	    gpio_set_value(BACK_LED_PIN, 1);
            msleep(INTERVAL);
            gpio_set_value(FRONT_LED_PIN, 0);
	    gpio_set_value(BACK_LED_PIN, 0);
            msleep(INTERVAL);
	}
	if((front_alert_state == ALERT) && (back_alert_state == SAFE))
	{
            gpio_set_value(FRONT_LED_PIN, 1);
            gpio_set_value(BACK_LED_PIN, 0);
            msleep(INTERVAL);
            gpio_set_value(FRONT_LED_PIN, 0);
            gpio_set_value(BACK_LED_PIN, 0);
            msleep(INTERVAL);
	}
	if((front_alert_state == SAFE) && (back_alert_state == ALERT))
	{
            gpio_set_value(FRONT_LED_PIN, 0);
            gpio_set_value(BACK_LED_PIN, 1);
            msleep(INTERVAL);
            gpio_set_value(FRONT_LED_PIN, 0);
            gpio_set_value(BACK_LED_PIN, 0);
            msleep(INTERVAL);
	}
	if((front_alert_state == SAFE) && (back_alert_state == SAFE))
	{
	    gpio_set_value(FRONT_LED_PIN, 0);
	    gpio_set_value(BACK_LED_PIN, 0);
            msleep(INTERVAL);
	}
	break;
	}
	    case MANUAL:  //MANUAL mode both LED on
		{
	    gpio_set_value(BACK_LED_PIN, 1);
            gpio_set_value(FRONT_LED_PIN, 1);
            msleep(INTERVAL);
		break;
		}
	}
    }
    return 0;
}



//Flash-Button Pressed Handler
static irqreturn_t fl_btn_isr(int irq, void *data) 
{
    //if current is SENSING mode, then turn to MANUAL.
	current_mode=(current_mode==OFFLINE)?MANUAL:OFFLINE;

    return IRQ_HANDLED;
}

//Sense Button Pressed Handler
static irqreturn_t t_btn_isr(int irq, void *data) 
{
    //if sensor is on, switch off; if sensor off, switch on
switch (choose_sensor) //rotating choose sensor
        { 
case 0:
	{
front_sensor_active = false;
back_sensor_active = false;
current_mode=SENSING;
choose_sensor=1;
break; 
}
case 1:
	{
front_sensor_active = true;
back_sensor_active = false;
current_mode=SENSING;
choose_sensor=2;
break; 
}
case 2:
	{
front_sensor_active = false;
back_sensor_active = true;
current_mode=SENSING;
choose_sensor=3;
break; 
}
case 3:
	{
front_sensor_active = true;
back_sensor_active = true;
current_mode=SENSING;
choose_sensor=4;
break; 
}
case 4:
	{
front_sensor_active = false;
back_sensor_active = false;
current_mode=OFFLINE;
choose_sensor=1;
break; 
}
}

    return IRQ_HANDLED;
}

//Read button Pressed Handler
static irqreturn_t r_btn_isr(int irq, void *data) 
{
    //Get current mode and print out
    switch (current_mode) 
    {
        case OFFLINE:
            printk(KERN_INFO "System OFFLINE\n");break;           
        case SENSING:
            printk(KERN_INFO "SENSING mode is on\n");break;           
        case MANUAL:
            printk(KERN_INFO "MANUAL mode is on\n");break;                 
    }
    return IRQ_HANDLED;
}

//High Resolution Timer Callback Handler
static enum hrtimer_restart sensor_timer_callback(struct hrtimer *timer)
{
    ktime_t interval = ktime_set(0, 10000000); //Set interval to 0s 10ms
    long front_echo_start = 0, front_echo_end = 0;
    long back_echo_start = 0, back_echo_end = 0;
    long front_time_difference, front_distance;
    long back_time_difference, back_distance;
    unsigned long timeout;

    //if both sensor is off
    if (!front_sensor_active && !back_sensor_active)
    {
        //Update timer to expire in interval time
        hrtimer_forward_now(timer, interval);
        return HRTIMER_RESTART;
    }
if(front_sensor_active)
{
    // Handle front sensor
    gpio_set_value(S_TRIG_PIN, 1);
    udelay(10); //10us High Level Trigger Signal
    gpio_set_value(S_TRIG_PIN, 0);
    timeout = jiffies + msecs_to_jiffies(10);
    while (!gpio_get_value(FRONT_ECHO_PIN) && time_before(jiffies, timeout))
        front_echo_start = ktime_get_ns();
    while (gpio_get_value(FRONT_ECHO_PIN) && time_before(jiffies, timeout))
        front_echo_end = ktime_get_ns();
    front_time_difference = front_echo_end - front_echo_start;
    front_distance = (front_time_difference * 343) / 2000000;
    if (front_distance>-30)
    {
        front_alert_state = ALERT;
    }
    else
    {
        front_alert_state = SAFE;
    }
}
if(back_sensor_active)
{
    // Handle back sensor
    gpio_set_value(S_TRIG_PIN, 1);
    udelay(10); //10us High Level Trigger Signal
    gpio_set_value(S_TRIG_PIN, 0);
    timeout = jiffies + msecs_to_jiffies(10);
    while (!gpio_get_value(BACK_ECHO_PIN) && time_before(jiffies, timeout))
        back_echo_start = ktime_get_ns();
    while (gpio_get_value(BACK_ECHO_PIN) && time_before(jiffies, timeout))
        back_echo_end = ktime_get_ns();
    back_time_difference = back_echo_end - back_echo_start;
    back_distance = (back_time_difference * 343) / 2000000;
    if (back_distance >-30)
    {
        back_alert_state = ALERT;
    }
    else
    {
        back_alert_state = SAFE;
    }
}
    //update timer with current time
    hrtimer_forward_now(timer, interval);
    return HRTIMER_RESTART;
}


static struct hrtimer sensor_timer;

static int __init security_door_init(void) {
    int ret;
    //High Resolution Timer Initial and seting
    hrtimer_init(&sensor_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    sensor_timer.function = sensor_timer_callback;
    hrtimer_start(&sensor_timer, ktime_set(0, 10000000), HRTIMER_MODE_REL);

    //GPIO front_led Setting
    gpio_request(FRONT_LED_PIN, "front_led");
    gpio_direction_output(FRONT_LED_PIN, 0);
    //GPIO back_led Setting
    gpio_request(BACK_LED_PIN, "back_led");
    gpio_direction_output(BACK_LED_PIN, 0);
    //GPIO fl_btn Setting and Interrupt setting
    gpio_request(FL_BTN_PIN, "fl_btn");
    gpio_direction_input(FL_BTN_PIN);
    fl_btn_irq = gpio_to_irq(FL_BTN_PIN);
    ret = request_irq(fl_btn_irq, fl_btn_isr, IRQF_TRIGGER_RISING, "fl_btn_irq", NULL);
    if (ret) {
        printk(KERN_ERR "Failed to request FL_BTN IRQ\n");
        return ret;
        }

    gpio_request(S_TRIG_PIN, "s_trig");
    gpio_direction_output(S_TRIG_PIN, 0);

    gpio_request(FRONT_ECHO_PIN, "front_echo");
    gpio_direction_input(FRONT_ECHO_PIN);
    gpio_request(BACK_ECHO_PIN, "back_echo");
    gpio_direction_input(BACK_ECHO_PIN);

    gpio_request(T_BTN_PIN, "t_btn");
    gpio_direction_input(T_BTN_PIN);
    t_btn_irq = gpio_to_irq(T_BTN_PIN);
    ret = request_irq(t_btn_irq, t_btn_isr, IRQF_TRIGGER_RISING, "t_btn_irq", NULL);
    if (ret) {
        printk(KERN_ERR "Failed to request T_BTN IRQ\n");
        free_irq(fl_btn_irq, NULL);
        return ret;
        }

    gpio_request(R_BTN_PIN, "r_btn");
    gpio_direction_input(R_BTN_PIN);
    r_btn_irq = gpio_to_irq(R_BTN_PIN);
    ret = request_irq(r_btn_irq, r_btn_isr, IRQF_TRIGGER_RISING, "r_btn_irq", NULL);
    if (ret) {
        printk(KERN_ERR "Failed to request R_BTN IRQ\n");
        free_irq(t_btn_irq, NULL);
        free_irq(fl_btn_irq, NULL);
        return ret;
        }
    //Create another thread for LED Flashing
    blink_thread = kthread_run(led_blink, NULL, "blink_thread");
    if (IS_ERR(blink_thread)) {
        printk(KERN_ERR "Failed to create blink thread\n");
        free_irq(r_btn_irq, NULL);
        free_irq(t_btn_irq, NULL);
        free_irq(fl_btn_irq, NULL);
        return PTR_ERR(blink_thread);
        }

    printk(KERN_INFO "Security door module loaded\n");
    return 0;
    }

//Exit Function
static void __exit security_door_exit(void) {
    kthread_stop(blink_thread);
    hrtimer_cancel(&sensor_timer);
    free_irq(fl_btn_irq, NULL);
    gpio_free(FL_BTN_PIN);
    gpio_free(S_TRIG_PIN);
    gpio_free(FRONT_ECHO_PIN);
    gpio_free(BACK_ECHO_PIN);
    free_irq(t_btn_irq, NULL);
    gpio_free(T_BTN_PIN);
    free_irq(r_btn_irq, NULL);
    gpio_free(R_BTN_PIN);
    gpio_set_value(FRONT_LED_PIN, 0);
    gpio_free(FRONT_LED_PIN);
    gpio_set_value(BACK_LED_PIN, 0);
    gpio_free(BACK_LED_PIN);
    printk(KERN_INFO "Security door module unloaded\n");
}

module_init(security_door_init);
module_exit(security_door_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("yg csc");



















