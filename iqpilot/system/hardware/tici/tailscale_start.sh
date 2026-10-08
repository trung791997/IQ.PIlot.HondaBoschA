#!/usr/bin/env bash
# Start Tailscale from /data/tailscale when it is installed there.
#
# The root filesystem is read-only, so Tailscale's own Linux installer (apt + a systemd unit) cannot run, and a unit
# added to /etc/systemd/system by remounting rw is lost whenever the OS image is reflashed (IQ.OS vs AGNOS). /data
# survives both, so the static arm64 binaries and the node state live there and this launcher starts them on every
# boot. Userspace networking needs no tun device and changes no routes. Best-effort: never fails the launch.
#
# Install once, on the device:
#   mkdir -p /data/tailscale && cd /data/tailscale
#   curl -fsSL https://pkgs.tailscale.com/stable/tailscale_<ver>_arm64.tgz | tar xz --strip-components=1 \
#     tailscale_<ver>_arm64/tailscale tailscale_<ver>_arm64/tailscaled
#   (after the next boot) sudo /data/tailscale/tailscale --socket=/data/tailscale/tailscaled.sock up --ssh

TS_DIR="${TS_DIR:-/data/tailscale}"
TAILSCALED="$TS_DIR/tailscaled"

[ -x "$TAILSCALED" ] || exit 0

# already running: the stock-AGNOS systemd unit, or an earlier launch
if pgrep -f "^$TAILSCALED( |\$)" > /dev/null 2>&1; then
  exit 0
fi

mkdir -p "$TS_DIR/state" 2> /dev/null

# root, like the systemd unit, so Tailscale SSH can log in as the comma user
SUDO=""
if [ "$(id -u)" -ne 0 ] && sudo -n true 2> /dev/null; then
  SUDO="sudo -n"
fi

# restart on exit, like the unit's Restart=on-failure
nohup $SUDO bash -c "while true; do
  '$TAILSCALED' --tun=userspace-networking --socks5-server=localhost:1055 \
    --state='$TS_DIR/state/tailscaled.state' --socket='$TS_DIR/tailscaled.sock' --statedir='$TS_DIR/state'
  sleep 5
done" > "$TS_DIR/tailscaled.log" 2>&1 < /dev/null &

exit 0
