/* ============================================================================
 *  led_sw_driver.c  --  RPi GPIO LED x3 & Switch x2 통합 캐릭터 디바이스 드라이버
 * ----------------------------------------------------------------------------
 *  단일 디바이스 드라이버 모듈로 구현된 LED 및 스위치 제어 드라이버 (/dev/led_sw)
 *
 *  하드웨어 구성:
 *    - LED_초록 (Green)  : 명령코드 중 제어코드 동작 중 (GPIO 17 기본값)
 *    - LED_노랑 (Yellow) : 명령 대기 중                 (GPIO 27 기본값)
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

#include "../shared/led_sw.h"

/* 커널 6.15+ 타이머 삭제 함수 이름 변경 호환성 매크로 */
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
static int gpio_green      = 17;
static int gpio_yellow     = 27;
static int gpio_red        = 22;
static int gpio_scan_start = 23;
static int gpio_ems        = 24;

module_param(gpio_green, int, 0444);
MODULE_PARM_DESC(gpio_green, "GPIO pin for Green LED (default: 17)");

module_param(gpio_yellow, int, 0444);
MODULE_PARM_DESC(gpio_yellow, "GPIO pin for Yellow LED (default: 27)");

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
	int pin_sw_scan_start;
	int pin_sw_ems;

	/* IRQ 번호 */
	int irq_scan_start;
	int irq_ems;

	/* 디바운스 타이머 */
	struct timer_list timer_scan_start;
	struct timer_list timer_ems;

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
static void scan_start_timer_handler(struct timer_list *t)
{
	struct led_sw_dev *dev = container_of(t, struct led_sw_dev, timer_scan_start);
	int val = gpio_get_value(dev->pin_sw_scan_start);
	u8 pressed = (val == 0) ? 1 : 0; /* Active Low */

	if (pressed != dev->sw_state[SW_SCAN_START]) {
		dev->sw_state[SW_SCAN_START] = pressed;
		if (pressed) {
			struct led_sw_event evt;

			evt.sw_id = SW_SCAN_START;
			evt.state = 1;
			evt.timestamp_ms = jiffies_to_msecs(jiffies);

			if (kfifo_put(&dev->fifo, evt))
				wake_up_interruptible(&dev->wq);
			pr_info("led_sw: SW_SCAN_START pressed\n");
		}
	}
}

static void ems_timer_handler(struct timer_list *t)
{
	struct led_sw_dev *dev = container_of(t, struct led_sw_dev, timer_ems);
	int val = gpio_get_value(dev->pin_sw_ems);
	u8 pressed = (val == 0) ? 1 : 0; /* Active Low */

	if (pressed != dev->sw_state[SW_EMS]) {
		dev->sw_state[SW_EMS] = pressed;
		if (pressed) {
			struct led_sw_event evt;

			evt.sw_id = SW_EMS;
			evt.state = 1;
			evt.timestamp_ms = jiffies_to_msecs(jiffies);

			if (kfifo_put(&dev->fifo, evt))
				wake_up_interruptible(&dev->wq);
			pr_info("led_sw: SW_EMS pressed\n");
		}
	}
}

/* ---------------------------------------------------------------------------
 *  스위치 IRQ 핸들러
 * ------------------------------------------------------------------------- */
static irqreturn_t scan_start_irq_handler(int irq, void *dev_id)
{
	struct led_sw_dev *dev = (struct led_sw_dev *)dev_id;
	(void)irq;
	mod_timer(&dev->timer_scan_start, jiffies + msecs_to_jiffies(DEBOUNCE_DELAY_MS));
	return IRQ_HANDLED;
}

static irqreturn_t ems_irq_handler(int irq, void *dev_id)
{
	struct led_sw_dev *dev = (struct led_sw_dev *)dev_id;
	(void)irq;
	mod_timer(&dev->timer_ems, jiffies + msecs_to_jiffies(DEBOUNCE_DELAY_MS));
	return IRQ_HANDLED;
}

/* ---------------------------------------------------------------------------
 *  LED 출력 제어 헬퍼
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
		set_led_hw(dev, LED_GREEN, ctrl.green);
		set_led_hw(dev, LED_YELLOW, ctrl.yellow);
		set_led_hw(dev, LED_RED, ctrl.red);
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
	if (ret == -EBUSY || ret == -EPROBE_DEFER) {
		gpio_free(pin);
		ret = gpio_request_one(pin, flags, label);
	}
	return ret;
}

/* ---------------------------------------------------------------------------
 *  Platform Probe & Remove
 * ------------------------------------------------------------------------- */
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
	}

	/* GPIO 요청 - LED */
	ret = request_gpio_safe(g_led_sw->pin_led_green, GPIOF_OUT_INIT_LOW, "led_green");
	if (ret)
		pr_warn("led_sw: gpio green (%d) request result: %d\n", g_led_sw->pin_led_green, ret);

	ret = request_gpio_safe(g_led_sw->pin_led_yellow, GPIOF_OUT_INIT_LOW, "led_yellow");
	if (ret)
		pr_warn("led_sw: gpio yellow (%d) request result: %d\n", g_led_sw->pin_led_yellow, ret);

	ret = request_gpio_safe(g_led_sw->pin_led_red, GPIOF_OUT_INIT_LOW, "led_red");
	if (ret)
		pr_warn("led_sw: gpio red (%d) request result: %d\n", g_led_sw->pin_led_red, ret);

	/* GPIO 요청 - Switches */
	ret = request_gpio_safe(g_led_sw->pin_sw_scan_start, GPIOF_IN, "sw_scan_start");
	if (ret)
		pr_warn("led_sw: gpio scan_start (%d) request result: %d\n", g_led_sw->pin_sw_scan_start, ret);

	ret = request_gpio_safe(g_led_sw->pin_sw_ems, GPIOF_IN, "sw_ems");
	if (ret)
		pr_warn("led_sw: gpio ems (%d) request result: %d\n", g_led_sw->pin_sw_ems, ret);

	/* 타이머 초기화 */
	timer_setup(&g_led_sw->timer_scan_start, scan_start_timer_handler, 0);
	timer_setup(&g_led_sw->timer_ems, ems_timer_handler, 0);

	/* IRQ 등록 */
	if (gpio_is_valid(g_led_sw->pin_sw_scan_start)) {
		g_led_sw->irq_scan_start = gpio_to_irq(g_led_sw->pin_sw_scan_start);
		if (g_led_sw->irq_scan_start >= 0) {
			ret = request_irq(g_led_sw->irq_scan_start, scan_start_irq_handler,
					  IRQF_TRIGGER_FALLING, "sw_scan_start_irq", g_led_sw);
			if (ret)
				pr_warn("led_sw: request_irq scan_start failed: %d\n", ret);
		}
	} else {
		g_led_sw->irq_scan_start = -1;
	}

	if (gpio_is_valid(g_led_sw->pin_sw_ems)) {
		g_led_sw->irq_ems = gpio_to_irq(g_led_sw->pin_sw_ems);
		if (g_led_sw->irq_ems >= 0) {
			ret = request_irq(g_led_sw->irq_ems, ems_irq_handler,
					  IRQF_TRIGGER_FALLING, "sw_ems_irq", g_led_sw);
			if (ret)
				pr_warn("led_sw: request_irq ems failed: %d\n", ret);
		}
	} else {
		g_led_sw->irq_ems = -1;
	}

	/* misc device 등록 (/dev/led_sw) */
	g_led_sw->misc.minor    = MISC_DYNAMIC_MINOR;
	g_led_sw->misc.name     = LED_SW_DEV_NAME;
	g_led_sw->misc.fops     = &led_sw_fops;
	g_led_sw->misc.mode     = 0666;
	g_led_sw->misc.nodename = LED_SW_DEV_NAME;

	ret = misc_register(&g_led_sw->misc);
	if (ret) {
		pr_err("led_sw: misc_register result: %d\n", ret);
	}

	platform_set_drvdata(pdev, g_led_sw);
	pr_info("led_sw: driver probed & registered successfully (/dev/%s)\n", LED_SW_DEV_NAME);
	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
static void led_sw_remove(struct platform_device *pdev)
#else
static int led_sw_remove(struct platform_device *pdev)
#endif
{
	(void)pdev;
	if (g_led_sw) {
		misc_deregister(&g_led_sw->misc);

		if (g_led_sw->irq_ems >= 0)
			free_irq(g_led_sw->irq_ems, g_led_sw);
		if (g_led_sw->irq_scan_start >= 0)
			free_irq(g_led_sw->irq_scan_start, g_led_sw);

		led_sw_del_timer_sync(&g_led_sw->timer_ems);
		led_sw_del_timer_sync(&g_led_sw->timer_scan_start);

		/* LED 소등 후 해제 */
		set_led_hw(g_led_sw, LED_GREEN, 0);
		set_led_hw(g_led_sw, LED_YELLOW, 0);
		set_led_hw(g_led_sw, LED_RED, 0);

		if (gpio_is_valid(g_led_sw->pin_sw_ems))
			gpio_free(g_led_sw->pin_sw_ems);
		if (gpio_is_valid(g_led_sw->pin_sw_scan_start))
			gpio_free(g_led_sw->pin_sw_scan_start);
		if (gpio_is_valid(g_led_sw->pin_led_red))
			gpio_free(g_led_sw->pin_led_red);
		if (gpio_is_valid(g_led_sw->pin_led_yellow))
			gpio_free(g_led_sw->pin_led_yellow);
		if (gpio_is_valid(g_led_sw->pin_led_green))
			gpio_free(g_led_sw->pin_led_green);

		g_led_sw = NULL;
		pr_info("led_sw: driver removed\n");
	}
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
	return 0;
#endif
}

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
