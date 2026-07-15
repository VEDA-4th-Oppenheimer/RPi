/*
 * turret_driver.c — RPi serdev 커널 모듈 (/dev/turret)
 *
 * protocol.h(PROTO_VERSION=3) 프레임으로 STM32(USART1)와 UART 통신.
 * 유저 데몬/테스트앱은 /dev/turret 에 ioctl 로 명령을 내리고,
 * 드라이버가 [SOF|CMD|LEN|PAYLOAD|CRC16] 프레임으로 조립해 UART TX 한다.
 *
 * 설계 메모:
 *   - SET_TARGET / HOME / SET_MODE / DISARM / QUERY_DIST 는
 *     STM32 가 즉시 ACK 하지 않는 fire-and-forget 명령이다. (완료는
 *     CMD_ALIGNED / CMD_HOMED 등으로 STM 이 비동기 통지)
 *   - 따라서 위 ioctl 들은 프레임을 UART 로 흘려보내고 즉시 반환한다.
 *   - GET_DISTANCE 만 CMD_QUERY_DIST 송신 후 CMD_DISTANCE 상행을
 *     completion 으로 대기한다. (STM 측 미구현이면 -ETIMEDOUT)
 *   - RX 콜백은 STM->RPi 통지(PONG/HOMED/ALIGNED/STATUS/DISTANCE/ERROR)를
 *     파싱해 turret_link_state 캐시를 갱신한다.
 *
 * 빌드: RPi에서  make   (Makefile 참조, -I 로 프로젝트 루트 protocol.h)
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
#include <linux/completion.h>
#include <linux/string.h>
#include <linux/of.h>

/* 커널 include 시 __KERNEL__ 이 정의되어 proto_u8=__u8 등으로 매핑,
 * TURRET_* ioctl 매크로/구조체도 활성화된다. */
#include "protocol.h"


/* ── 디바이스 컨텍스트 ────────────────────────────────── */
struct turret_dev {
	struct serdev_device *serdev;
	struct miscdevice     misc;
	struct mutex          lock;        /* ioctl 직렬화(TX 프레임 원자성) */
	struct completion     dist_recv;   /* CMD_DISTANCE 상행 대기         */

	/* 유저에게 노출하는 종합 상태(캐시) */
	struct turret_link_state st;
	struct proto_distance    last_dist;

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

/* ═══════════════════════════════════════════════════════
 *  serdev 수신 콜백 — STM->RPi 통지 파싱
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

				print_hex_dump(KERN_INFO, "turret RX: ",
					       DUMP_PREFIX_NONE, 16, 1,
					       dev->rx_buf, dev->rx_need, false);

				switch (cmd) {
				case CMD_PONG:
					dev->st.link_alive = 1;
					break;
				case CMD_HOMED:
					dev->st.flags |= STF_HOMED;
					break;
				case CMD_ALIGNED:
					dev->st.flags |= STF_ALIGNED;
					break;
				case CMD_STATUS:
					if (len == sizeof(struct proto_status)) {
						struct proto_status s;

						memcpy(&s, pl, sizeof(s));
						dev->st.cur_theta_ddeg = s.cur_theta_ddeg;
						dev->st.cur_phi_ddeg   = s.cur_phi_ddeg;
						dev->st.flags          = s.flags;
					}
					break;
				case CMD_DISTANCE:
					if (len == sizeof(struct proto_distance)) {
						memcpy(&dev->last_dist, pl,
						       sizeof(dev->last_dist));
						complete(&dev->dist_recv);
					}
					break;
				case CMD_ERROR:
					if (len == sizeof(struct proto_err)) {
						struct proto_err e;

						memcpy(&e, pl, sizeof(e));
						dev->st.last_err = e.code;
						pr_warn("turret: STM ERROR code=%u\n",
							e.code);
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
 *  ioctl — 유저 데몬/테스트앱 진입점
 * ═══════════════════════════════════════════════════════ */
static long turret_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	struct turret_dev *dev = g_dev;
	void __user *uarg = (void __user *)arg;
	int ret;

	if (!dev)
		return -ENODEV;

	switch (cmd) {
	case TURRET_SET_TARGET: {
		struct proto_target t;

		if (copy_from_user(&t, uarg, sizeof(t)))
			return -EFAULT;
		if (t.theta_ddeg < THETA_MIN || t.theta_ddeg > THETA_MAX ||
		    t.phi_ddeg   < PHI_MIN   || t.phi_ddeg   > PHI_MAX)
			return -EINVAL;

		mutex_lock(&dev->lock);
		ret = turret_send_frame(dev, CMD_SET_TARGET, &t, sizeof(t));
		mutex_unlock(&dev->lock);
		return ret;
	}

	case TURRET_HOME:
		mutex_lock(&dev->lock);
		ret = turret_send_frame(dev, CMD_HOME, NULL, 0);
		mutex_unlock(&dev->lock);
		return ret;

	case TURRET_SET_MODE: {
		struct proto_mode m;

		if (copy_from_user(&m, uarg, sizeof(m)))
			return -EFAULT;
		if (m.mode > MODE_TRACK)
			return -EINVAL;

		mutex_lock(&dev->lock);
		ret = turret_send_frame(dev, CMD_SET_MODE, &m, sizeof(m));
		mutex_unlock(&dev->lock);
		return ret;
	}

	case TURRET_DISARM:
		mutex_lock(&dev->lock);
		ret = turret_send_frame(dev, CMD_DISARM, NULL, 0);
		mutex_unlock(&dev->lock);
		return ret;

	case TURRET_GET_STATE:
		if (copy_to_user(uarg, &dev->st, sizeof(dev->st)))
			return -EFAULT;
		return 0;

	case TURRET_GET_DISTANCE:
		mutex_lock(&dev->lock);
		reinit_completion(&dev->dist_recv);
		ret = turret_send_frame(dev, CMD_QUERY_DIST, NULL, 0);
		if (ret < 0) {
			mutex_unlock(&dev->lock);
			return ret;
		}
		/* CMD_DISTANCE 상행 대기(500ms) */
		if (!wait_for_completion_timeout(&dev->dist_recv,
						 msecs_to_jiffies(500))) {
			mutex_unlock(&dev->lock);
			pr_warn("turret: DISTANCE timeout (STM 미응답)\n");
			return -ETIMEDOUT;
		}
		ret = copy_to_user(uarg, &dev->last_dist,
				   sizeof(dev->last_dist)) ? -EFAULT : 0;
		mutex_unlock(&dev->lock);
		return ret;

	default:
		return -ENOTTY;
	}
}

static const struct file_operations turret_fops = {
	.owner          = THIS_MODULE,
	.unlocked_ioctl = turret_ioctl,
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
	init_completion(&dev->dist_recv);

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
	dev_info(&serdev->dev, "turret probed → /dev/turret ready\n");
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
MODULE_DESCRIPTION("/dev/turret — RPi<->STM32 UART turret protocol driver");
