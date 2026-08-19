#!/usr/bin/env python3
"""
fake_camera.py — 카메라 수신측(lidar_json_receiver) 대역

카메라 없이 데몬의 업로드 경로를 시험한다. mTLS 핸드셰이크와 와이어
프로토콜만 흉내내고, 받은 파일은 검증만 하고 버린다(--save 로 저장).

  사용:
      python3 fake_camera.py --certs /etc/adts/certs [--port 2222] [--once]

  주의: 이 스크립트는 **카메라 인증서(adts-camera.crt/.key)** 로 서버를 연다.
    발급:  sudo bash broker/gen-certs.sh --server adts-camera

  수신측(CV5 앱)이 갖춰야 할 것을 이 파일이 그대로 보여준다:
    ① adts-camera.crt / adts-camera.key 로 TLS 서버를 연다
    ② ca.crt 로 **클라이언트 인증서를 검증한다**(CERT_REQUIRED)
       — 이게 없으면 mTLS 가 아니라 그냥 TLS 다. 아무나 올릴 수 있다.
    ③ 프레이밍은 평문일 때와 동일하다. TLS 는 그 아래에만 들어간다.
"""
import argparse
import hashlib
import socket
import ssl
import struct
import sys


def recv_exact(sock, n):
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise EOFError(f"연결이 끊겼다 ({len(buf)}/{n} B)")
        buf += chunk
    return buf


def serve_one(tls_sock, save):
    peer = tls_sock.getpeercert()
    cn = dict(x[0] for x in peer["subject"]).get("commonName", "?")
    print(f"  클라이언트 CN={cn}  cipher={tls_sock.cipher()[0]}", flush=True)

    # 주의: 신원 확인은 여기서 한 번 더 한다. 인증서가 우리 CA 로 서명됐다는 것과
    #   그것이 **데몬**이라는 것은 다른 얘기다. 같은 CA 가 Qt 콘솔 인증서도
    #   발급했으므로, CN 을 안 보면 Qt 인증서로도 파일을 밀어넣을 수 있다.
    if cn != "adts-daemon":
        print(f"   거부 — 기대한 CN 은 adts-daemon", flush=True)
        tls_sock.sendall(b'{"result":"error","reason":"unauthorized"}\n')
        return

    name_len = struct.unpack("!H", recv_exact(tls_sock, 2))[0]
    name = recv_exact(tls_sock, name_len).decode("ascii")
    file_len = struct.unpack("!Q", recv_exact(tls_sock, 8))[0]

    got, digest = 0, hashlib.sha256()
    out = open(name, "wb") if save else None
    while got < file_len:
        chunk = tls_sock.recv(min(262144, file_len - got))
        if not chunk:
            raise EOFError(f"본문 중단 {got}/{file_len} B")
        digest.update(chunk)
        if out:
            out.write(chunk)
        got += len(chunk)
    if out:
        out.close()

    print(f"   {name}  {file_len:,} B  sha256={digest.hexdigest()[:16]}"
          + ("  (저장함)" if save else ""), flush=True)
    tls_sock.sendall(b'{"result":"ok","file":"' + name.encode() + b'"}\n')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--certs", default="/etc/adts/certs")
    ap.add_argument("--port", type=int, default=2222)
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--name", default="adts-camera", help="서버 인증서 이름")
    ap.add_argument("--save", action="store_true", help="받은 파일을 저장한다")
    ap.add_argument("--once", action="store_true", help="한 번 받고 끝낸다")
    a = ap.parse_args()

    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.minimum_version = ssl.TLSVersion.TLSv1_2
    ctx.load_cert_chain(f"{a.certs}/{a.name}.crt", f"{a.certs}/{a.name}.key")
    ctx.load_verify_locations(f"{a.certs}/ca.crt")
    ctx.verify_mode = ssl.CERT_REQUIRED          # 핵심: mTLS

    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((a.bind, a.port))
    srv.listen(4)
    print(f"fake-camera: {a.bind}:{a.port} 대기 (인증서 {a.name}, mTLS 필수)",
          flush=True)

    while True:
        raw, addr = srv.accept()
        print(f"접속: {addr[0]}:{addr[1]}", flush=True)
        try:
            with ctx.wrap_socket(raw, server_side=True) as tls:
                serve_one(tls, a.save)
        except ssl.SSLError as e:
            # 여기로 오는 대부분은 클라이언트 인증서 문제다.
            print(f"   TLS 실패: {e}", flush=True)
        except (EOFError, OSError) as e:
            print(f"   전송 실패: {e}", flush=True)
        finally:
            raw.close()
        if a.once:
            return 0


if __name__ == "__main__":
    sys.exit(main())
