/*
 * turret_driver.c — RPi serdev 커널 모듈 (/dev/turret)  [protocol.h v4 스캐너]
 *
 * protocol.h(PROTO_VERSION=4) 프레임으로 STM32(USART1)와 UART 통신.
 * 유저 데몬은 /dev/turret 에
 *   - ioctl 로 제어 명령(HOME / SCAN_START / SCAN_STOP / DISARM / PING)을 내리고,
 *   - read()/poll() 로 스캔 점 스트림(CMD_SCAN_DATA)을 배치 수신한다.
 * 드라이버는 [SOF|CMD|LEN|PAYLOAD|CRC16] 프레임으로 조립해 UART TX 하고,
 * 상행 프레임을 파싱해 스캔 점은 kfifo 로, 통지는 turret_link_state 캐시로 노출한다.
 *
 * 설계 메모:
 *   - HOME / SCAN_START / SCAN_STOP / DISARM / PING 은 STM 이 즉시 ACK 하지 않는
 *     fire-and-forget. 프레임을 UART 로 흘려보내고 즉시 반환한다.
 *     (완료/상태는 CMD_HOMED / CMD_SCAN_DONE / CMD_STATUS 로 STM 이 비동기 통지)
 *   - 스캔 점(CMD_SCAN_DATA)은 수만 개라 ioctl 이 아니라 kfifo + read()/poll()
 *     스트림으로 전달한다. 생산자=serdev RX 콜백, 소비자=read() 의 SPSC 구조라
 *     kfifo 자체 락으로 충분(별도 lock 불필요).
 *   - heartbeat: PONG 수신 시 pong_seq 만 증가시킨다. 300ms link_dead 판정은
 *     데몬(자기 CLOCK_MONOTONIC + pong_seq 증가 감시) 소유(§8 얇은 드라이버).
 *     따라서 커널은 link_dead 타이머를 돌리지 않는다. link_alive 는 raw 표시.
 *
 * 빌드: RPi에서  make   (Makefile 참조, -I 로 ../shared/protocol.h)
 * 사용: sudo dtoverlay turret-overlay.dtbo → sudo insmod turret_driver.ko
 *       → /dev/turret 생성
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/serdev.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/kfifo.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/string.h>
#include <linux/of.h>

/* 커널 include 시 __KERNEL__ 이 정의되어 proto_u8=__u8 등으로 매핑,
 * TURRET_* ioctl 매크로/구조체도 활성화된다. */
#include "protocol.h"

/* 스캔 점 링버퍼 깊이. proto_scan_point(6B) × 1024 = 6KB.
 * 라이다 100Hz(10ms/점) 대비 데몬 read 주기가 넉넉해 오버플로 여유 큼. */
#define SCAN_FIFO_POINTS 1024

/* ── 디바이스 컨텍스트 ────────────────────────────────── */
struct turret_dev {
	struct serdev_device *serdev;
	struct miscdevice     misc;
	struct mutex          lock;        /* ioctl 직렬화(TX 프레임 원자성) */
	wait_queue_head_t     rx_wq;       /* read()/poll() 대기            */

	/* 스캔 점 스트림 (생산자=RX 콜백, 소비자=read(), SPSC) */
	DECLARE_KFIFO(scan_fifo, struct proto_scan_point, SCAN_FIFO_POINTS);

	/* 유저에게 노출하는 종합 상태(캐시) */
	struct turret_link_state st;
	u32  last_point_count;             /* 최근 CMD_SCAN_DONE 총 점 수    */
	unsigned int notify_pending;       /* HOMED/DONE/STATUS/ERROR 통지   */

	/* 수신 파서 상태 */
	u8 rx_buf[PROTO_MAX_FRAME];
	u8 rx_idx;
	u8 rx_need;    /* 프레임 완성까지 필요한 총 바이트 */
};

/* 단일 인스턴스(연습 단축). 실제 다중화 시 container_of 사용. */
static struct turret_dev *g_dev;

/* ═══════════════════════════════════════════════════════
 *  프레임 빌드 + serdev 전송  (fire-and-forget)
 * ═══════════════════════════════════════════════════════ */
static int turret_send_frame(struct turret_dev *dev, u8 cmd,
			     const void *payload, u8 plen)
{
	u8  frame[PROTO_MAX_FRAME];
	u16 crc;
	int total, ret;

	if (plen > PROTO_MAX_PAYLOAD)          /* CWE-120 경계검사 */
		return -EINVAL;

	frame[0] = PROTO_SOF;
	frame[1] = cmd;
	frame[2] = plen;
	if (plen && payload)
		memcpy(&frame[PROTO_HEADER_LEN], payload, plen);

	total = PROTO_HEADER_LEN + plen;
	crc   = proto_crc16(frame, (proto_u16)total);
	frame[total]     = crc & 0xFF;          /* 리틀엔디언 */
	frame[total + 1] = (crc >> 8) & 0xFF;
	total += PROTO_CRC_LEN;

	print_hex_dump(KERN_INFO, "turret TX: ",
		       DUMP_PREFIX_NONE, 16, 1, frame, total, false);

	/* 블로킹 전송(1초 타임아웃). serdev 가 TX FIFO 로 밀어냄. */
	ret = serdev_device_write(dev->serdev, frame, total, HZ);
	return (ret == total) ? 0 : (ret < 0 ? ret : -EIO);
}

/* 통지(HOMED/DONE/STATUS/ERROR) 도착 → 데몬이 GET_STATE 로 확인하도록 깨움 */
static void turret_notify(struct turret_dev *dev)
{
	WRITE_ONCE(dev->notify_pending, 1);
	wake_up_interruptible(&dev->rx_wq);
}

/* ═══════════════════════════════════════════════════════
 *  serdev 수신 콜백 — STM->RPi 통지/스캔 파싱
 * ═══════════════════════════════════════════════════════ */
static size_t turret_rx_callback(struct serdev_device *serdev,
				 const unsigned char *buf, size_t count)
{
	struct turret_dev *dev = serdev_device_get_drvdata(serdev);
	size_t i;

	for (i = 0; i < count; i++) {
		u8 b = buf[i];

		/* ① SOF 탐색 */
		if (dev->rx_idx == 0) {
			if (b != PROTO_SOF)
				continue;
			dev->rx_buf[0] = b;
			dev->rx_idx    = 1;
			dev->rx_need   = 0;
			continue;
		}

		/* 오버플로우 방지(CWE-120) */
		if (dev->rx_idx >= PROTO_MAX_FRAME) {
			dev->rx_idx = 0;
			continue;
		}

		dev->rx_buf[dev->rx_idx++] = b;

		/* ② 헤더 완성 → 전체 프레임 길이 확정 */
		if (dev->rx_idx == PROTO_HEADER_LEN) {
			u8 len = dev->rx_buf[2];

			if (len > PROTO_MAX_PAYLOAD) {      /* CWE-120 */
				pr_warn("turret: bad LEN %u\n", len);
				dev->rx_idx = 0;
				continue;
			}
			dev->rx_need = PROTO_HEADER_LEN + len + PROTO_CRC_LEN;
		}

		/* ③ 프레임 완성 → CRC 검증 → CMD 디스패치 */
		if (dev->rx_need && dev->rx_idx >= dev->rx_need) {
			u8  len      = dev->rx_buf[2];
			u16 data_len = PROTO_HEADER_LEN + len;
			u16 rx_crc   = dev->rx_buf[dev->rx_need - 2]
				     | ((u16)dev->rx_buf[dev->rx_need - 1] << 8);
			u16 calc     = proto_crc16(dev->rx_buf,
						   (proto_u16)data_len);

			if (rx_crc == calc) {
				u8 cmd = dev->rx_buf[1];
				const u8 *pl = &dev->rx_buf[PROTO_HEADER_LEN];

				switch (cmd) {
				case CMD_PONG:
					/* heartbeat: 판정은 데몬. 여기선 카운터만. */
					dev->st.pong_seq++;
					dev->st.link_alive = 1;
					break;
				case CMD_HOMED:
					dev->st.flags |= STF_HOMED;
					turret_notify(dev);
					break;
				case CMD_STATUS:
					if (len == sizeof(struct proto_status)) {
						struct proto_status s;

						memcpy(&s, pl, sizeof(s));
						dev->st.cur_theta_ddeg = s.cur_theta_ddeg;
						dev->st.cur_phi_ddeg   = s.cur_phi_ddeg;
						dev->st.flags          = s.flags;
						turret_notify(dev);
					}
					break;
				case CMD_SCAN_DATA:
					if (len == sizeof(struct proto_scan_point)) {
						struct proto_scan_point pt;

						memcpy(&pt, pl, sizeof(pt));
						/* SPSC: 별도 락 없이 kfifo 로 밀어넣음 */
						if (!kfifo_put(&dev->scan_fifo, pt))
							pr_warn_ratelimited(
								"turret: scan fifo full, point dropped\n");
						wake_up_interruptible(&dev->rx_wq);
					}
					break;
				case CMD_SCAN_DONE:
					if (len == sizeof(struct proto_scan_done)) {
						struct proto_scan_done d;

						memcpy(&d, pl, sizeof(d));
						dev->last_point_count = d.point_count;
						dev->st.flags &= ~STF_SCANNING;
						turret_notify(dev);
					}
					break;
				case CMD_ERROR:
					if (len == sizeof(struct proto_err)) {
						struct proto_err e;

						memcpy(&e, pl, sizeof(e));
						dev->st.last_err = e.code;
						pr_warn("turret: STM ERROR code=%u\n",
							e.code);
						turret_notify(dev);
					}
					break;
				default:
					pr_info("turret: unknown cmd 0x%02X\n", cmd);
					break;
				}
			} else {
				pr_warn("turret: CRC FAIL rx=0x%04X calc=0x%04X\n",
					rx_crc, calc);
			}

			dev->rx_idx  = 0;
			dev->rx_need = 0;
		}
	}

	return count;   /* 전부 소비 */
}

static const struct serdev_device_ops turret_serdev_ops = {
	.receive_buf  = turret_rx_callback,
	.write_wakeup = serdev_device_write_wakeup,
};

/* ═══════════════════════════════════════════════════════
 *  read() — 스캔 점 스트림 (proto_scan_point 배치)
 * ═══════════════════════════════════════════════════════ */
static ssize_t turret_read(struct file *f, char __user *ubuf,
			   size_t count, loff_t *ppos)
{
	struct turret_dev *dev = g_dev;
	unsigned int copied = 0;
	size_t max;
	int ret;

	if (!dev)
		return -ENODEV;

	/* proto_scan_point 정수 배수만 전달(부분 점 금지) */
	max = count - (count % sizeof(struct proto_scan_point));
	if (max == 0)
		return -EINVAL;      /* 버퍼가 1점보다 작음 */

	if (kfifo_is_empty(&dev->scan_fifo)) {
		if (f->f_flags & O_NONBLOCK)
			return -EAGAIN;
		ret = wait_event_interruptible(dev->rx_wq,
					       !kfifo_is_empty(&dev->scan_fifo));
		if (ret)
			return ret;  /* -ERESTARTSYS */
	}

	ret = kfifo_to_user(&dev->scan_fifo, ubuf, max, &copied);
	if (ret)
		return ret;
	return copied;
}

/* ═══════════════════════════════════════════════════════
 *  poll() — 스캔 점/통지 도착, link_dead 대기
 * ═══════════════════════════════════════════════════════ */
static __poll_t turret_poll(struct file *f, struct poll_table_struct *wait)
{
	struct turret_dev *dev = g_dev;
	__poll_t mask = 0;

	if (!dev)
		return EPOLLERR;

	poll_wait(f, &dev->rx_wq, wait);

	if (!kfifo_is_empty(&dev->scan_fifo))
		mask |= EPOLLIN | EPOLLRDNORM;      /* 스캔 점 있음 */
	if (READ_ONCE(dev->notify_pending))
		mask |= EPOLLIN;                    /* 통지 → GET_STATE 확인 */

	return mask;
}

/* ═══════════════════════════════════════════════════════
 *  ioctl — 유저 데몬/테스트앱 제어 진입점
 * ═══════════════════════════════════════════════════════ */
static long turret_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	struct turret_dev *dev = g_dev;
	void __user *uarg = (void __user *)arg;
	int ret;

	if (!dev)
		return -ENODEV;

	switch (cmd) {
	case TURRET_HOME:
		mutex_lock(&dev->lock);
		ret = turret_send_frame(dev, CMD_HOME, NULL, 0);
		mutex_unlock(&dev->lock);
		return ret;

	case TURRET_SCAN_START: {
		struct proto_scan_start ss;

		if (copy_from_user(&ss, uarg, sizeof(ss)))
			return -EFAULT;
		/* 각도 범위 검증: θ 0~3599, φ -900~+900, step>0 */
		if (ss.theta_start_ddeg < THETA_MIN ||
		    ss.theta_start_ddeg > THETA_MAX ||
		    ss.theta_end_ddeg   < THETA_MIN ||
		    ss.theta_end_ddeg   > THETA_MAX ||
		    ss.phi_start_ddeg   < PHI_MIN   ||
		    ss.phi_start_ddeg   > PHI_MAX   ||
		    ss.phi_end_ddeg     < PHI_MIN   ||
		    ss.phi_end_ddeg     > PHI_MAX   ||
		    ss.step_ddeg == 0)
			return -EINVAL;

		mutex_lock(&dev->lock);
		ret = turret_send_frame(dev, CMD_SCAN_START, &ss, sizeof(ss));
		mutex_unlock(&dev->lock);
		return ret;
	}

	case TURRET_SCAN_STOP:
		mutex_lock(&dev->lock);
		ret = turret_send_frame(dev, CMD_SCAN_STOP, NULL, 0);
		mutex_unlock(&dev->lock);
		return ret;

	case TURRET_DISARM:
		mutex_lock(&dev->lock);
		ret = turret_send_frame(dev, CMD_DISARM, NULL, 0);
		mutex_unlock(&dev->lock);
		return ret;

	case TURRET_GET_STATE:
		WRITE_ONCE(dev->notify_pending, 0);   /* 통지 소비 */
		if (copy_to_user(uarg, &dev->st, sizeof(dev->st)))
			return -EFAULT;
		return 0;

	case TURRET_PING:
		/* 데몬이 100ms tick 마다 호출. 응답 PONG 은 pong_seq 로 감지 */
		mutex_lock(&dev->lock);
		ret = turret_send_frame(dev, CMD_PING, NULL, 0);
		mutex_unlock(&dev->lock);
		return ret;

	default:
		return -ENOTTY;
	}
}

static const struct file_operations turret_fops = {
	.owner          = THIS_MODULE,
	.unlocked_ioctl = turret_ioctl,
	.read           = turret_read,
	.poll           = turret_poll,
};

/* ═══════════════════════════════════════════════════════
 *  serdev probe / remove
 * ═══════════════════════════════════════════════════════ */
static int turret_probe(struct serdev_device *serdev)
{
	struct turret_dev *dev;
	int ret;

	dev = devm_kzalloc(&serdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->serdev = serdev;
	mutex_init(&dev->lock);
	init_waitqueue_head(&dev->rx_wq);
	INIT_KFIFO(dev->scan_fifo);

	serdev_device_set_drvdata(serdev, dev);
	serdev_device_set_client_ops(serdev, &turret_serdev_ops);

	ret = serdev_device_open(serdev);
	if (ret) {
		dev_err(&serdev->dev, "serdev open failed: %d\n", ret);
		return ret;
	}

	/* 115200 8N1, 흐름제어 없음 — STM32 USART1 설정과 일치 */
	serdev_device_set_baudrate(serdev, 115200);
	serdev_device_set_flow_control(serdev, false);
	serdev_device_set_parity(serdev, SERDEV_PARITY_NONE);

	dev->misc.minor = MISC_DYNAMIC_MINOR;
	dev->misc.name  = "turret";
	dev->misc.fops  = &turret_fops;

	ret = misc_register(&dev->misc);
	if (ret) {
		dev_err(&serdev->dev, "misc_register failed: %d\n", ret);
		serdev_device_close(serdev);
		return ret;
	}

	g_dev = dev;
	dev_info(&serdev->dev, "turret probed → /dev/turret ready (proto v%u)\n",
		 PROTO_VERSION);
	return 0;
}

static void turret_remove(struct serdev_device *serdev)
{
	struct turret_dev *dev = serdev_device_get_drvdata(serdev);

	misc_deregister(&dev->misc);
	serdev_device_close(serdev);
	g_dev = NULL;
	dev_info(&serdev->dev, "turret removed\n");
}

/* ═══════════════════════════════════════════════════════
 *  Device Tree 매칭 + 모듈 등록
 * ═══════════════════════════════════════════════════════ */
static const struct of_device_id turret_dt_ids[] = {
	{ .compatible = "adts,turret" },
	{ }
};
MODULE_DEVICE_TABLE(of, turret_dt_ids);

static struct serdev_device_driver turret_drv = {
	.driver = {
		.name           = "turret",
		.of_match_table = turret_dt_ids,
	},
	.probe  = turret_probe,
	.remove = turret_remove,
};
module_serdev_device_driver(turret_drv);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("이현우");
MODULE_DESCRIPTION("/dev/turret — RPi<->STM32 UART LiDAR scanner protocol driver (v4)");
