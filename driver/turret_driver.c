/*
 * turret_driver.c — RPi serdev 커널 모듈 (/dev/turret)  [protocol.h v5 스캐너]
 *
 * protocol.h(PROTO_VERSION=6) 프레임으로 STM32(USART1)와 UART 통신.
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
 *   - 드라이버는 좌표 변환을 하지 않는다. STM 이 올린 기구각(pan/tilt)과 거리를
 *     그대로 통과시키고, 계약 좌표계(x,y,z) 변환은 데몬이 맡는다.
 *
 * v4 -> v5 변경:
 *   - proto_scan_point 6B -> 18B (signal_strength / dis_status / 라이다·STM
 *     타임스탬프 추가). kfifo 원소 크기가 커진 것 외에 파싱 로직은 동일 —
 *     len == sizeof() 비교라 자동으로 새 크기를 따른다.
 *   - CMD_HOMED 가 proto_homed(8B) payload 를 싣는다. 아래 RX 파서에서 받아
 *     turret_link_state 의 home_* 필드에 캐시한다.
 *   - TURRET_SCAN_START 가 이전 스캔 잔여물을 비운다(아래 ioctl 주석 참조).
 *
 * 빌드: RPi에서  make   (Makefile 참조, -I 로 ../shared/protocol.h)
 * 사용: sudo dtoverlay overlays/turret-overlay.dtbo → sudo insmod turret_driver.ko
 *       → /dev/turret 생성
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/serdev.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/kfifo.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/string.h>
#include <linux/of.h>

/* 커널 include 시 __KERNEL__ 이 정의되어 proto_u8=__u8 등으로 매핑,
 * TURRET_* ioctl 매크로/구조체도 활성화된다. */
#include "protocol.h"

/* 스캔 점 링버퍼 깊이. v5 에서 proto_scan_point 가 6B -> 18B 로 커져
 * 18B × 1024 = 18KB 다. struct turret_dev 안에 인라인이라 probe 의
 * devm_kzalloc 이 그만큼 커지지만(≈20KB, kmalloc 상한 128KB 이내) probe 는
 * 비원자 컨텍스트라 문제 없다.
 * 깊이 1024 = 라이다 100Hz 기준 약 10초치. 데몬이 그보다 훨씬 자주 read 하므로
 * 스케줄링이 크게 밀려도 점을 잃지 않는다. */
#define SCAN_FIFO_POINTS 1024

/* ── 디바이스 컨텍스트 ────────────────────────────────── */
struct turret_dev {
	struct serdev_device *serdev;
	struct miscdevice     misc;
	struct mutex          lock;        /* ioctl 직렬화(TX 프레임 원자성) */
	/* st 전용 스핀락.
	 *
	 * 주의: dev->lock(뮤텍스)으로는 못 막는다. RX 콜백은 tty flip 워크에서
	 *   불리는데 거기서 뮤텍스를 잡으면 수신이 멈춰 프레임을 놓친다. 그래서
	 *   RX 는 락 없이 st 를 갱신했고, ioctl 은 그걸 통째로 copy_to_user 했다
	 *   — 갱신 도중에 읽으면 **서로 다른 프레임의 값이 섞인 스냅샷**이
	 *   나간다(예: 새 flags 와 옛 cur_pan_ddeg). 임계구역이 몇 바이트
	 *   복사뿐이라 스핀락이 맞다. copy_to_user 는 잠들 수 있으므로 반드시
	 *   락 밖에서 한다. */
	spinlock_t            st_lock;
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

	/* 주의: KERN_INFO 였는데 KERN_DEBUG 로 낮춘다. 데몬이 100ms 마다 PING 을
	 * 보내므로 초당 10줄이 무조건 쌓이고, 커널 링버퍼가 그걸로 가득 차서
	 * 정작 봐야 할 메시지(다른 드라이버의 probe 실패 등)가 밀려난다.
	 * 실제로 IMU 고장 로그를 이 홍수 때문에 못 봤다.
	 *
	 * 필요할 때만 켠다:
	 *   echo 'file turret_driver.c +p' > /sys/kernel/debug/dynamic_debug/control
	 * (CONFIG_DYNAMIC_DEBUG 가 없으면 모듈을 -DDEBUG 로 빌드) */
	print_hex_dump_debug("turret TX: ",
			     DUMP_PREFIX_NONE, 16, 1, frame, total, false);

	/* 블로킹 전송(1초 타임아웃). serdev 가 TX FIFO 로 밀어냄. */
	ret = serdev_device_write(dev->serdev, frame, total, HZ);
	return (ret == total) ? 0 : (ret < 0 ? ret : -EIO);
}

/* 통지(HOMED/DONE/STATUS/ERROR) 도착 → 데몬이 GET_STATE 로 확인하도록 깨움 */
/* st.flags 를 st_lock 아래에서 갱신한다. ioctl 이 dev->lock 을 쥐고 있어도
 * RX 콜백은 그 뮤텍스를 안 잡으므로, flags 읽기-수정-쓰기가 RX 의 갱신과
 * 겹칠 수 있다. 둘 다 같은 스핀락을 거쳐야 의미가 있다. */
static void st_flags_update(struct turret_dev *dev, u8 set, u8 clear)
{
	unsigned long fl;

	spin_lock_irqsave(&dev->st_lock, fl);
	dev->st.flags = (u8)((dev->st.flags | set) & (u8)~clear);
	spin_unlock_irqrestore(&dev->st_lock, fl);
}

/* payload 길이가 기대와 다르면 알린다.
 *
 * 주의: 예전에는 `if (len == sizeof(...))` 가 아니면 **아무 말 없이 버렸다.**
 *   그래서 펌웨어와 헤더 버전이 어긋나면 오류 통지 같은 것이 통째로 사라지는데,
 *   증상은 "STM 이 아무 말도 안 한다" 로만 보여 링크 문제로 오해하게 된다.
 *   protocol.h 4개 사본은 CI 의 drift-check 가 지키지만, **보드에 옛 펌웨어가
 *   그대로 올라가 있는 경우**는 그것으로 못 잡는다 — 이 경고가 그걸 잡는다.
 *   버전 바이트를 와이어에 싣는 것보다 넓다(버전을 안 올리고 구조체만 바꾼
 *   경우까지 걸린다). */
static void turret_bad_len(u8 cmd, u8 len, size_t want)
{
	pr_warn_ratelimited("turret: payload 길이 불일치 cmd=0x%02X len=%u (기대 %zu)"
			    " — 펌웨어 버전 확인\n", cmd, len, want);
}

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
				unsigned long fl;

				/* st 갱신 전체를 한 임계구역으로 묶는다. 프레임 하나가
				 * 여러 필드를 함께 바꾸므로(예: CMD_STATUS 는 각도와
				 * flags 를 같이), 필드별로 잠그면 스냅샷이 여전히 섞인다. */
				spin_lock_irqsave(&dev->st_lock, fl);
				switch (cmd) {
				case CMD_PONG:
					/* heartbeat: 판정은 데몬. 여기선 카운터만. */
					dev->st.pong_seq++;
					dev->st.link_alive = 1;
					break;
				case CMD_HOMED:
					/* v5: 홈 결과(엔코더 raw + 각도)를 싣고 온다.
					 * 구버전 STM 은 payload 없이(len=0) 보내므로
					 * 크기가 맞을 때만 캐시하고 플래그는 항상 세운다
					 * — 펌웨어를 먼저 올리든 나중에 올리든 홈 자체는
					 * 동작해야 하기 때문. */
					if (len == sizeof(struct proto_homed)) {
						struct proto_homed h;

						memcpy(&h, pl, sizeof(h));
						dev->st.home_pan_encoder_raw  =
							h.pan_encoder_raw;
						dev->st.home_tilt_encoder_raw =
							h.tilt_encoder_raw;
						dev->st.home_pan_ddeg  = h.pan_ddeg;
						dev->st.home_tilt_ddeg = h.tilt_ddeg;
					} else if (len != 0) {
						pr_warn("turret: HOMED bad len %u (want %zu)\n",
							len, sizeof(struct proto_homed));
					}
					dev->st.flags |= STF_HOMED;
					turret_notify(dev);
					break;
				case CMD_STATUS:
					/* v6: STM 이 1초 주기로 실제 보낸다. v5 까지는 이
					 * 프레임이 한 번도 오지 않아 STF_HOMED 가 갱신될 일이
					 * 없었고, STM 을 리셋하면 캐시만 참으로 남았다.
					 *
					 * turret_notify 는 부르지 않는다 — 데몬은 100ms 틱마다
					 * GET_STATE 로 어차피 읽으므로, 깨우면 poll() 이 초당
					 * 한 번 헛되이 깨어날 뿐이다(예전에는 불렀는데, 그때는
					 * 이 프레임이 실제로 오지 않아 드러나지 않았다). */
					if (len == sizeof(struct proto_status)) {
						struct proto_status ps;

						memcpy(&ps, pl, sizeof(ps));
						dev->st.cur_pan_ddeg  = ps.cur_pan_ddeg;
						dev->st.cur_tilt_ddeg = ps.cur_tilt_ddeg;
						dev->st.flags         = ps.flags;
						dev->st.tx_fail       = ps.tx_fail;
						dev->st.rx_ovf        = ps.rx_ovf;
						dev->st.enc_retry     = ps.enc_retry;
						dev->st.lidar_drop    = ps.lidar_drop;
						dev->st.reject_busy   = ps.reject_busy;
						dev->st.status_seen   = 1;
					} else {
						turret_bad_len(cmd, len,
							       sizeof(struct proto_status));
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
						dev->st.last_err      = e.code;
						dev->st.last_err_axis = e.axis;
						pr_warn("turret: STM ERROR code=%u axis=%u\n",
							e.code, e.axis);
						turret_notify(dev);
					} else {
						turret_bad_len(cmd, len,
							       sizeof(struct proto_err));
					}
					break;
				default:
					pr_info("turret: unknown cmd 0x%02X\n", cmd);
					break;
				}
				spin_unlock_irqrestore(&dev->st_lock, fl);
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
/* cppcheck-suppress constParameterCallback  // fops.read ABI: struct file* 비const 고정 */
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
		/* 요청 시점에 STF_HOMED 를 내린다.
		 *
		 * 이게 없으면 플래그가 "예전에 한 번이라도 홈했음" 이라는 뜻이 되어
		 * 영영 참으로 남는다. STM 은 CMD_STATUS 를 주기적으로 보내지 않아
		 * 갱신될 일도 없으므로, STM 을 리셋/재플래시하면 STM 의 홈 상태는
		 * 사라졌는데 드라이버 캐시만 참으로 남는다. 그 상태로 유저가 홈을
		 * 건너뛰면 SCAN_START 가 매번 ERR_NOT_HOMED 로 거절된다(실기 발생).
		 *
		 * 내려두면 플래그가 "직전 HOME 이후 완료됨" 을 뜻하게 되어,
		 * 유저가 이 플래그를 기다리는 것이 실제로 의미를 갖는다. */
		st_flags_update(dev, 0, STF_HOMED);
		ret = turret_send_frame(dev, CMD_HOME, NULL, 0);
		mutex_unlock(&dev->lock);
		return ret;

	case TURRET_SCAN_START: {
		struct proto_scan_start ss;

		if (copy_from_user(&ss, uarg, sizeof(ss)))
			return -EFAULT;
		/* 각도 범위 검증: θ 0~3599, φ -900~+900, step>0 */
		if (ss.pan_start_ddeg < PAN_MIN ||
		    ss.pan_start_ddeg > PAN_MAX ||
		    ss.pan_end_ddeg   < PAN_MIN ||
		    ss.pan_end_ddeg   > PAN_MAX ||
		    ss.tilt_start_ddeg   < TILT_MIN   ||
		    ss.tilt_start_ddeg   > TILT_MAX   ||
		    ss.tilt_end_ddeg     < TILT_MIN   ||
		    ss.tilt_end_ddeg     > TILT_MAX   ||
		    ss.step_ddeg == 0)
			return -EINVAL;

		mutex_lock(&dev->lock);

		/* 이전 스캔 잔여물 제거.
		 *
		 * 왜 필요한가: 1축 브링업에서 스캔을 연속으로 돌리면 산출물에 이전
		 * 스캔의 점이 섞여 들어왔다(격자 셀 충돌 → 중복 카운트). 중단된
		 * 스캔이나 데몬이 다 읽지 않고 종료한 경우 fifo 에 점이 남기 때문.
		 *
		 * kfifo_reset() 이 아니라 kfifo_reset_out() 을 쓴다. 전자는 in/out
		 * 포인터를 모두 건드려 생산자(RX 콜백)와 동시 접근하면 깨진다.
		 * 후자는 소비자측 포인터만 옮기므로 "단일 소비자"만 지키면 안전한데,
		 * 소비자는 read()/ioctl 뿐이고 이 구간은 dev->lock 아래다.
		 *
		 * 잔여 창(window): 여기서 비운 뒤 SCAN_START 가 STM 에 닿기 전까지
		 * 이미 UART 에 실려 있던 점이 들어올 수 있다. 실제로는 직전 스캔이
		 * SCAN_DONE 으로 끝난 뒤 호출되므로 비어 있고, 남더라도 데몬이
		 * 각도로 격자에 배치하므로 새 스캔 범위 밖이면 걸러진다. */
		kfifo_reset_out(&dev->scan_fifo);
		dev->last_point_count = 0;
		{
			unsigned long fl;

			spin_lock_irqsave(&dev->st_lock, fl);
			dev->st.last_err = ERR_NONE;
			spin_unlock_irqrestore(&dev->st_lock, fl);
		}

		/* STF_SCANNING 을 여기서 세운다. STM 의 CMD_STATUS 를 기다리면
		 * 그 사이 데몬이 "스캔 안 도는데?" 로 오판한다. 전송 실패 시
		 * 아래에서 되돌린다. */
		st_flags_update(dev, STF_SCANNING, 0);

		ret = turret_send_frame(dev, CMD_SCAN_START, &ss, sizeof(ss));
		if (ret < 0)
			st_flags_update(dev, 0, STF_SCANNING);   /* 롤백 */

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

	case TURRET_GET_STATE: {
		struct turret_link_state snap;
		unsigned long fl;

		WRITE_ONCE(dev->notify_pending, 0);   /* 통지 소비 */
		/* 락 안에서는 복사만. copy_to_user 는 페이지 폴트로 잠들 수 있어
		 * 스핀락 아래에서 부르면 안 된다. */
		spin_lock_irqsave(&dev->st_lock, fl);
		snap = dev->st;
		spin_unlock_irqrestore(&dev->st_lock, fl);

		if (copy_to_user(uarg, &snap, sizeof(snap)))
			return -EFAULT;
		return 0;
	}

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
	spin_lock_init(&dev->st_lock);
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
	dev->misc.mode = 0666;

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
