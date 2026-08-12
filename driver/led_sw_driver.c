/* ============================================================================
 *  led_sw_driver.c  --  RPi GPIO LED x3 & Switch x2 통합 캐릭터 디바이스 드라이버
 * ----------------------------------------------------------------------------
 *  단일 디바이스 드라이버 모듈로 구현된 LED 및 스위치 제어 드라이버 (/dev/led_sw)
 *
 *  하드웨어 구성:
 *    - LED_초록 (Green)  : 명령코드 중 제어코드 동작 중 (GPIO 27 기본값, Pin 11에 연결)
 *    - LED_노랑 (Yellow) : 명령 대기 중                 (GPIO 17 기본값, Pin 13에 연결)
 *    - LED_빨강 (Red)    : 에러(코드) 발생              (GPIO 22 기본값)
 *    - 스위치_scan_start : 스캔 시작 CMD_SCAN_START     (GPIO 23 기본값, IRQ)
 *    - 스위치_ems        : 즉시 정지 CMD_DISARM         (GPIO 24 기본값, IRQ)
 *
 *  기능:
 *    - Platform Driver + DeviceTree (adts,led-sw) 매칭 및 수동 insmod 폴백 지원
 *    - ioctl() 로 LED 개별/일괄 제어 및 현재 상태 조회
 *    - Interrupt + Timer Debounce 기반 스위치 입력 감지
 *    - kfifo + poll() + read() 로 스위치 눌림 이벤트를 유저 데몬에 비동기 전달
 * ==========================================================================*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/kfifo.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/version.h>
#include <linux/kthread.h>
#include <linux/delay.h>

#include "../shared/led_sw.h"

/* 커널 6.15+ 타이머 함수 이름 변경 호환성 매크로 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 15, 0)
  #define led_sw_del_timer_sync(t) timer_delete_sync(t)
#else
  #define led_sw_del_timer_sync(t) del_timer_sync(t)
#endif

MODULE_LICENSE("GPL");
MODULE_AUTHOR("VEDA Oppenheimer Team");
MODULE_DESCRIPTION("RPi Integrated LED & Switch Character Driver");
MODULE_VERSION("1.1");

/* 모듈 파라미터 기본 핀 정의 (RPi4 BCM GPIO 번호) */
static int gpio_green      = 27;
static int gpio_yellow     = 17;
static int gpio_red        = 22;
static int gpio_buzzer     = 26;
static int gpio_scan_start = 23;
static int gpio_ems        = 24;

module_param(gpio_green, int, 0444);
MODULE_PARM_DESC(gpio_green, "GPIO pin for Green LED (default: 27)");

module_param(gpio_yellow, int, 0444);
MODULE_PARM_DESC(gpio_yellow, "GPIO pin for Yellow LED (default: 17)");

module_param(gpio_red, int, 0444);
MODULE_PARM_DESC(gpio_red, "GPIO pin for Red LED (default: 22)");

module_param(gpio_scan_start, int, 0444);
MODULE_PARM_DESC(gpio_scan_start, "GPIO pin for Scan Start Switch (default: 23)");

module_param(gpio_ems, int, 0444);
MODULE_PARM_DESC(gpio_ems, "GPIO pin for EMS Switch (default: 24)");

#define EVENT_FIFO_SIZE 64
#define DEBOUNCE_DELAY_MS 50

struct led_sw_dev {
	struct miscdevice misc;
	struct mutex lock;
	wait_queue_head_t wq;

	/* GPIO 번호 */
	int pin_led_green;
	int pin_led_yellow;
	int pin_led_red;
	int pin_buzzer;
	int pin_sw_scan_start;
	int pin_sw_ems;

	/* 폴링 타이머 (인터럽트 대체) */
	struct timer_list poll_timer;

	/* 수동 부저(Passive Buzzer)용 커널 스레드 (소프트웨어 PWM) */
	struct task_struct *buzzer_thread;
	int buzzer_toggle;

	/* LED 및 스위치 상태 캐시 */
	u8 led_state[LED_MAX];
	u8 sw_state[SW_MAX];

	/* 이벤트 FIFO (read/poll) */
	DECLARE_KFIFO(fifo, struct led_sw_event, EVENT_FIFO_SIZE);
};

static struct led_sw_dev *g_led_sw = NULL;
static struct platform_device *g_plat_dev = NULL;

/* ---------------------------------------------------------------------------
 *  스위치 디바운스 타이머 콜백
 * ------------------------------------------------------------------------- */
/* ---------------------------------------------------------------------------
 *  스위치 폴링 타이머 (50ms 주기로 상태 확인)
 * ------------------------------------------------------------------------- */
static void sw_poll_timer_handler(struct timer_list *t)
{
	struct led_sw_dev *dev = container_of(t, struct led_sw_dev, poll_timer);
	/* 스위치는 pull-up + active-low 라 눌리면 0 이다. 핀이 없으면 안 눌린
	 * 것으로 본다 — 없는 스위치가 계속 눌린 상태로 보이면 EMS 가 상시 발동한다. */
	u8 pressed_scan = 0, pressed_ems = 0;

	if (gpio_is_valid(dev->pin_sw_scan_start))
		pressed_scan = (gpio_get_value(dev->pin_sw_scan_start) == 0) ? 1 : 0;

	if (gpio_is_valid(dev->pin_sw_ems))
		pressed_ems = (gpio_get_value(dev->pin_sw_ems) == 0) ? 1 : 0;


	if (pressed_scan != dev->sw_state[SW_SCAN_START]) {
		dev->sw_state[SW_SCAN_START] = pressed_scan;
		struct led_sw_event evt;
		evt.sw_id = SW_SCAN_START;
		evt.state = pressed_scan;
		evt.timestamp_ms = jiffies_to_msecs(jiffies);

		if (kfifo_put(&dev->fifo, evt))
			wake_up_interruptible(&dev->wq);
		pr_info("led_sw: SW_SCAN_START %s\n", pressed_scan ? "pressed" : "released");
	}

	if (pressed_ems != dev->sw_state[SW_EMS]) {
		dev->sw_state[SW_EMS] = pressed_ems;
		struct led_sw_event evt;
		evt.sw_id = SW_EMS;
		evt.state = pressed_ems;
		evt.timestamp_ms = jiffies_to_msecs(jiffies);

		if (kfifo_put(&dev->fifo, evt))
			wake_up_interruptible(&dev->wq);
		pr_info("led_sw: SW_EMS %s\n", pressed_ems ? "pressed" : "released");
	}

	mod_timer(&dev->poll_timer, jiffies + msecs_to_jiffies(DEBOUNCE_DELAY_MS));
}

/* ---------------------------------------------------------------------------
 *  수동 부저(Passive Buzzer) 커널 스레드 (소프트웨어 PWM)
 * ------------------------------------------------------------------------- */
static int buzzer_kthread_func(void *data)
{
	struct led_sw_dev *dev = data;
	unsigned long last_print = jiffies;

	while (!kthread_should_stop()) {
		if (READ_ONCE(dev->led_state[LED_BUZZER])) {
			dev->buzzer_toggle = !dev->buzzer_toggle;
			gpio_set_value(dev->pin_buzzer, dev->buzzer_toggle);
			
			/* pr_info 였는데 낮춘다. 울리는 동안 초당 1줄이 쌓이는데,
			 * 커널 링버퍼가 주기 로그로 차면 정작 봐야 할 메시지(다른
			 * 드라이버의 probe 실패 등)가 밀려난다 — turret 드라이버의
			 * TX 덤프가 IMU 진단을 가렸던 것과 같은 문제다.
			 *   echo 'file led_sw_driver.c +p' > \
			 *     /sys/kernel/debug/dynamic_debug/control */
			if (time_after(jiffies, last_print + HZ)) {
				pr_debug("led_sw: buzzer active, toggling pin %d\n",
					 dev->pin_buzzer);
				last_print = jiffies;
			}
			/* 패시브 피에조 부저 공진 주파수 (~2.7kHz, 반주기 185us).
			 * 기존 1.25ms(400Hz)는 공진 범위를 벗어나 소리가 안 들리거나 극히 작았음.
			 * test_buzzer.py 의 4kHz~2.7kHz 공진 대역에 맞춤. */
			usleep_range(180, 200);

		} else {
			/* 꺼져 있을 때는 CPU 점유율을 낮추기 위해 대기 */
			msleep(20);
			last_print = jiffies;
		}
	}
	return 0;
}

/* ---------------------------------------------------------------------------
 *  하드웨어 제어 유틸리티
 * ------------------------------------------------------------------------- */
static void set_led_hw(struct led_sw_dev *dev, enum led_channel ch, u8 on)
{
	int pin = -1;
	on = !!on;

	switch (ch) {
	case LED_GREEN:
		pin = dev->pin_led_green;
		break;
	case LED_YELLOW:
		pin = dev->pin_led_yellow;
		break;
	case LED_RED:
		pin = dev->pin_led_red;
		break;
	case LED_BUZZER:
		pin = dev->pin_buzzer;
		break;

	default:
		return;
	}

	if (gpio_is_valid(pin)) {
		gpio_set_value(pin, on);
		dev->led_state[ch] = on;
	}
}

/* ---------------------------------------------------------------------------
 *  File Operations (open, read, poll, ioctl)
 * ------------------------------------------------------------------------- */
static int led_sw_open(struct inode *inode, struct file *file)
{
	(void)inode;
	file->private_data = g_led_sw;
	return 0;
}

/* cppcheck-suppress constParameterCallback ; file_operations read 콜백 ABI */
static ssize_t led_sw_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	struct led_sw_dev *dev = file->private_data;
	struct led_sw_event evt;

	(void)ppos;
	if (count < sizeof(struct led_sw_event))
		return -EINVAL;

	if (!dev)
		return -ENODEV;

	if (kfifo_is_empty(&dev->fifo)) {
		int ret;

		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;

		ret = wait_event_interruptible(dev->wq, !kfifo_is_empty(&dev->fifo));
		if (ret)
			return ret;
	}

	if (!kfifo_get(&dev->fifo, &evt))
		return -EAGAIN;

	if (copy_to_user(buf, &evt, sizeof(struct led_sw_event)))
		return -EFAULT;

	return sizeof(struct led_sw_event);
}

static __poll_t led_sw_poll(struct file *file, poll_table *wait)
{
	struct led_sw_dev *dev = file->private_data;
	__poll_t mask = 0;

	if (!dev)
		return EPOLLERR;

	poll_wait(file, &dev->wq, wait);

	if (!kfifo_is_empty(&dev->fifo))
		mask |= (EPOLLIN | EPOLLRDNORM);

	return mask;
}

/* cppcheck-suppress constParameterCallback ; file_operations unlocked_ioctl 콜백 ABI */
static long led_sw_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct led_sw_dev *dev = file->private_data;

	if (!dev)
		return -ENODEV;

	switch (cmd) {
	case LED_SW_SET_LEDS: {
		struct led_sw_ctrl ctrl;

		if (copy_from_user(&ctrl, (void __user *)arg, sizeof(ctrl)))
			return -EFAULT;

		mutex_lock(&dev->lock);
		set_led_hw(dev, LED_GREEN,  ctrl.green);
		set_led_hw(dev, LED_YELLOW, ctrl.yellow);
		set_led_hw(dev, LED_RED,    ctrl.red);
		set_led_hw(dev, LED_BUZZER, ctrl.buzzer);
		mutex_unlock(&dev->lock);
		break;
	}
	case LED_SW_SET_SINGLE: {
		u32 val = (u32)arg;
		u16 ch = (val >> 16) & 0xFFFF;
		u8 on = val & 0xFF;

		if (ch >= LED_MAX)
			return -EINVAL;

		mutex_lock(&dev->lock);
		set_led_hw(dev, (enum led_channel)ch, on);
		mutex_unlock(&dev->lock);
		break;
	}
	case LED_SW_GET_STATE: {
		struct led_sw_state st;

		mutex_lock(&dev->lock);
		st.leds[LED_GREEN]  = dev->led_state[LED_GREEN];
		st.leds[LED_YELLOW] = dev->led_state[LED_YELLOW];
		st.leds[LED_RED]    = dev->led_state[LED_RED];
		st.leds[LED_BUZZER] = dev->led_state[LED_BUZZER];

		st.sw[SW_SCAN_START] = dev->sw_state[SW_SCAN_START];
		st.sw[SW_EMS]        = dev->sw_state[SW_EMS];
		mutex_unlock(&dev->lock);

		if (copy_to_user((void __user *)arg, &st, sizeof(st)))
			return -EFAULT;
		break;
	}
	default:
		return -ENOTTY;
	}

	return 0;
}

static struct file_operations led_sw_fops = {
	.owner          = THIS_MODULE,
	.open           = led_sw_open,
	.read           = led_sw_read,
	.poll           = led_sw_poll,
	.unlocked_ioctl = led_sw_ioctl,
	.llseek         = noop_llseek,
};

/* ---------------------------------------------------------------------------
 *  GPIO 요청 헬퍼 (재시도 및 에러 처리)
 * ------------------------------------------------------------------------- */
static int request_gpio_safe(int pin, unsigned long flags, const char *label)
{
	int ret;

	if (!gpio_is_valid(pin))
		return -EINVAL;

	ret = gpio_request_one(pin, flags, label);

	/* ⚠️ -EPROBE_DEFER 는 여기서 되돌리지 않는다.
	 *
	 *   그 코드의 뜻은 "내가 기대는 리소스(gpiochip 등)가 아직 준비되지
	 *   않았으니 **나중에 다시 불러 달라**" 이다. 즉시 재요청해도 상황이
	 *   바뀔 리가 없고, 오히려 커널이 준비된 뒤 probe 를 다시 불러줄
	 *   기회를 없애 deferral 자체를 무력화한다. 그대로 올려보낸다.
	 *
	 * ⚠️ -EBUSY 에서 gpio_free 로 되찾는 것은 남겨둔다. 실기에서 실제로
	 *   필요했던 우회이고, 재현할 보드 없이 빼면 probe 가 실패할 수 있다.
	 *   다만 이건 **남이 쥔 GPIO 를 강제로 뺏는** 동작이라 안전하지 않다.
	 *   원인 후보:
	 *     ① 이전 insmod 의 정리 누락으로 우리 자신의 낡은 요청이 남음
	 *        (remove 경로 결함이 있었으므로 유력하다 — 정리 수정 후
	 *         -EBUSY 가 아직도 나는지 다시 볼 것)
	 *     ② 오버레이의 brcm,pins pinctrl 이 같은 핀을 이미 잡음
	 *   ②라면 descriptor API(devm_gpiod_get)로 옮기면 깔끔히 풀린다. */
	if (ret == -EBUSY) {
		pr_warn("led_sw: gpio %d busy — 강제 회수 후 재시도 (%s)\n",
			pin, label);
		gpio_free(pin);
		ret = gpio_request_one(pin, flags, label);
	}
	return ret;
}

/* ---------------------------------------------------------------------------
 *  Platform Probe & Remove
 * ------------------------------------------------------------------------- */
/* GPIO·타이머·스레드 정리. **misc device 는 여기서 건드리지 않는다.**
 *
 * 이 함수는 두 곳에서 불리는데 misc 상태가 서로 다르기 때문이다:
 *   probe 실패 경로 — misc_register 가 실패했으므로 등록된 것이 없다
 *   remove 경로     — 등록돼 있으므로 반드시 deregister 해야 한다
 * 여기에 misc_deregister 를 넣으면 실패 경로가 등록도 안 된 걸 해제하려 들고,
 * 빼면 remove 경로에서 문자 디바이스가 남는다. 그래서 **호출자가** 책임진다.
 *
 * ⚠️ 남겨두면 어떻게 되나: rmmod 로 g_led_sw(devm 할당)가 해제된 뒤에도
 *   /dev/led_sw 가 등록된 채라 fops 가 사라진 모듈을 가리킨다. 누가 열면
 *   use-after-free 다. 재적재 시 같은 이름으로 misc_register 가 또 불려
 *   실패할 수도 있다. */
static void led_sw_teardown(void)
{
	if (g_led_sw) {
		if (g_led_sw->buzzer_thread) {
			kthread_stop(g_led_sw->buzzer_thread);
			g_led_sw->buzzer_thread = NULL;
		}

		led_sw_del_timer_sync(&g_led_sw->poll_timer);

		/* LED 및 부저 소등 후 해제 */
		set_led_hw(g_led_sw, LED_GREEN, 0);
		set_led_hw(g_led_sw, LED_YELLOW, 0);
		set_led_hw(g_led_sw, LED_RED, 0);
		set_led_hw(g_led_sw, LED_BUZZER, 0);

		if (gpio_is_valid(g_led_sw->pin_sw_ems))
			gpio_free(g_led_sw->pin_sw_ems);
		if (gpio_is_valid(g_led_sw->pin_sw_scan_start))
			gpio_free(g_led_sw->pin_sw_scan_start);
		if (gpio_is_valid(g_led_sw->pin_buzzer))
			gpio_free(g_led_sw->pin_buzzer);
		if (gpio_is_valid(g_led_sw->pin_led_red))
			gpio_free(g_led_sw->pin_led_red);
		if (gpio_is_valid(g_led_sw->pin_led_yellow))
			gpio_free(g_led_sw->pin_led_yellow);
		if (gpio_is_valid(g_led_sw->pin_led_green))
			gpio_free(g_led_sw->pin_led_green);

		g_led_sw = NULL;
	}
}

static int led_sw_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int ret;

	if (g_led_sw)
		return 0; /* 이미 초기화됨 */

	g_led_sw = devm_kzalloc(&pdev->dev, sizeof(*g_led_sw), GFP_KERNEL);
	if (!g_led_sw)
		return -ENOMEM;

	mutex_init(&g_led_sw->lock);
	init_waitqueue_head(&g_led_sw->wq);
	INIT_KFIFO(g_led_sw->fifo);

	/* DeviceTree 노드 속성 해석 (없으면 모듈 파라미터 기본값 적용) */
	g_led_sw->pin_led_green     = gpio_green;
	g_led_sw->pin_led_yellow    = gpio_yellow;
	g_led_sw->pin_led_red       = gpio_red;
	g_led_sw->pin_buzzer        = gpio_buzzer;
	g_led_sw->pin_sw_scan_start = gpio_scan_start;
	g_led_sw->pin_sw_ems        = gpio_ems;

	if (np) {
		int gpio_tmp;

		gpio_tmp = of_get_named_gpio(np, "gpios-led-green", 0);
		if (gpio_tmp >= 0)
			g_led_sw->pin_led_green = gpio_tmp;

		gpio_tmp = of_get_named_gpio(np, "gpios-led-yellow", 0);
		if (gpio_tmp >= 0)
			g_led_sw->pin_led_yellow = gpio_tmp;

		gpio_tmp = of_get_named_gpio(np, "gpios-led-red", 0);
		if (gpio_tmp >= 0)
			g_led_sw->pin_led_red = gpio_tmp;

		gpio_tmp = of_get_named_gpio(np, "gpios-sw-scan-start", 0);
		if (gpio_tmp >= 0)
			g_led_sw->pin_sw_scan_start = gpio_tmp;

		gpio_tmp = of_get_named_gpio(np, "gpios-sw-ems", 0);
		if (gpio_tmp >= 0)
			g_led_sw->pin_sw_ems = gpio_tmp;

		gpio_tmp = of_get_named_gpio(np, "gpios-buzzer", 0);
		if (gpio_tmp >= 0)
			g_led_sw->pin_buzzer = gpio_tmp;
	}

	/* GPIO 요청 - LED */
	ret = request_gpio_safe(g_led_sw->pin_led_green, GPIOF_OUT_INIT_LOW, "led_green");
	if (ret) {
		pr_err("led_sw: gpio green (%d) request failed: %d\n", g_led_sw->pin_led_green, ret);
		return ret;
	}

	ret = request_gpio_safe(g_led_sw->pin_led_yellow, GPIOF_OUT_INIT_LOW, "led_yellow");
	if (ret) {
		pr_err("led_sw: gpio yellow (%d) request failed: %d\n", g_led_sw->pin_led_yellow, ret);
		return ret;
	}

	ret = request_gpio_safe(g_led_sw->pin_led_red, GPIOF_OUT_INIT_LOW, "led_red");
	if (ret) {
		pr_err("led_sw: gpio red (%d) request failed: %d\n", g_led_sw->pin_led_red, ret);
		return ret;
	}

	ret = request_gpio_safe(g_led_sw->pin_buzzer, GPIOF_OUT_INIT_LOW, "buzzer");
	if (ret) {
		pr_err("led_sw: gpio buzzer (%d) request failed: %d\n", g_led_sw->pin_buzzer, ret);
		return ret;
	}

	/* GPIO 요청 - Switches */
	ret = request_gpio_safe(g_led_sw->pin_sw_scan_start, GPIOF_IN, "sw_scan_start");
	if (ret) {
		pr_err("led_sw: gpio scan_start (%d) request failed: %d\n", g_led_sw->pin_sw_scan_start, ret);
		return ret;
	}

	ret = request_gpio_safe(g_led_sw->pin_sw_ems, GPIOF_IN, "sw_ems");
	if (ret) {
		pr_err("led_sw: gpio ems (%d) request failed: %d\n", g_led_sw->pin_sw_ems, ret);
		return ret;
	}

	/* 폴링 타이머 초기화 및 시작 (50ms) */
	timer_setup(&g_led_sw->poll_timer, sw_poll_timer_handler, 0);
	mod_timer(&g_led_sw->poll_timer, jiffies + msecs_to_jiffies(DEBOUNCE_DELAY_MS));

	/* 수동 부저(Passive Buzzer)용 커널 스레드 시작 */
	g_led_sw->buzzer_thread = kthread_run(buzzer_kthread_func, g_led_sw, "buzzer_pwm_thread");
	if (IS_ERR(g_led_sw->buzzer_thread)) {
		pr_warn("led_sw: Failed to create buzzer kthread\n");
		g_led_sw->buzzer_thread = NULL;
	}

	/* misc device 등록 (/dev/led_sw) */
	g_led_sw->misc.minor    = MISC_DYNAMIC_MINOR;
	g_led_sw->misc.name     = LED_SW_DEV_NAME;
	g_led_sw->misc.fops     = &led_sw_fops;
	g_led_sw->misc.mode     = 0666;
	g_led_sw->misc.nodename = LED_SW_DEV_NAME;

	ret = misc_register(&g_led_sw->misc);
	if (ret) {
		/* ⚠️ 예전엔 로그만 남기고 0 을 반환했다. 그러면 /dev/led_sw 가 없는데도
		 *   insmod 가 성공하고 probe 성공 로그까지 찍혀서, 데몬이 "open 실패 →
		 *   degraded" 로 조용히 넘어간 이유를 찾기 어려웠다. 실패는 실패로 알린다. */
		pr_err("led_sw: misc_register failed: %d\n", ret);
		led_sw_teardown();
		return ret;
	}

	platform_set_drvdata(pdev, g_led_sw);
	pr_info("led_sw: driver probed & registered successfully (/dev/%s)\n", LED_SW_DEV_NAME);
	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
static void led_sw_remove(struct platform_device *pdev)
{
	(void)pdev;
	if (g_led_sw)
		misc_deregister(&g_led_sw->misc);
	led_sw_teardown();
	pr_info("led_sw: driver removed\n");
}
#else
static int led_sw_remove(struct platform_device *pdev)
{
	(void)pdev;
	if (g_led_sw)
		misc_deregister(&g_led_sw->misc);
	led_sw_teardown();
	pr_info("led_sw: driver removed\n");
	return 0;
}
#endif

static const struct of_device_id led_sw_of_match[] = {
	{ .compatible = "adts,led-sw" },
	{ }
};
MODULE_DEVICE_TABLE(of, led_sw_of_match);

static struct platform_driver led_sw_platform_driver = {
	.probe  = led_sw_probe,
	.remove = led_sw_remove,
	.driver = {
		.name           = "led_sw_custom",
		.of_match_table = led_sw_of_match,
		.owner          = THIS_MODULE,
	},
};

/* ---------------------------------------------------------------------------
 *  Init & Exit
 * ------------------------------------------------------------------------- */
static int __init led_sw_init(void)
{
	(void)platform_driver_register(&led_sw_platform_driver);

	/* DT 오버레이가 미로딩되었거나 probe 수동 호출 필요 시 폴백 생성 */
	if (!g_led_sw) {
		g_plat_dev = platform_device_register_simple("led_sw_custom", -1, NULL, 0);
		if (IS_ERR(g_plat_dev)) {
			g_plat_dev = NULL;
		}
	}

	return 0;
}

static void __exit led_sw_exit(void)
{
	if (g_plat_dev) {
		platform_device_unregister(g_plat_dev);
		g_plat_dev = NULL;
	}
	platform_driver_unregister(&led_sw_platform_driver);
	pr_info("led_sw: module exit completed\n");
}

module_init(led_sw_init);
module_exit(led_sw_exit);
