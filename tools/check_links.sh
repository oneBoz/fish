#!/bin/sh
# Field check for Maelstrom: is the laptop on the hotspot, do the boards answer, does the camera give a frame?
#   tools/check_links.sh [fin-ip] [cam-ip]
FIN=${1:-172.20.10.12}; CAM=${2:-172.20.10.13}
echo "laptop: $(ifconfig | awk '/inet 172\.20\.10\./{print $2}' | head -1)"
for ip in $FIN $CAM; do ping -c 1 -W 800 -q $ip >/dev/null 2>&1 && echo "$ip  ping ok" || echo "$ip  NO PING (board off, wrong hotspot, or wrong IP)"; done
PY=$(dirname "$0")/../.venv/bin/python; [ -x "$PY" ] || PY=python3
"$PY" - "$FIN" <<'EOF'
import socket, sys, time
ip=sys.argv[1]; s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.settimeout(0.5); s.bind(("0.0.0.0",0))
s.sendto(b"0.0,0.0\n",(ip,4210)); t0=time.time(); ack=imu=batt=0   # neutral fins: pid_fins.ino ignores PING
while time.time()-t0<2:
    try: d,_=s.recvfrom(256)
    except socket.timeout: continue
    for l in d.decode(errors="ignore").splitlines():
        ack+=l.startswith("ACK"); imu+=l.startswith("IMU,"); batt+=l.startswith("BATT")
print("fin board: %s ACK, %d IMU lines in 2 s%s" % ("got" if ack else "NO", imu, "" if imu else "  -> no IMU stream (pid_fins.ino, or fish_node.ino could not find the ICM20948; check its serial monitor)"))
EOF
out=$(mktemp); code=$(curl -s -m 10 -o "$out" -w "%{http_code}" "http://$CAM:81/capture"); size=$(wc -c < "$out" | tr -d ' ')
if [ "$code" = "200" ] && [ "$size" -gt 1000 ]; then echo "camera: capture ok ($size bytes) -> stream should work"; else echo "camera: NO FRAME (http $code, $size bytes) -> power-cycle the ESP32-CAM, make sure nothing else is viewing its stream, then run this again"; fi
rm -f "$out"
