#!/bin/bash
# Setup script for RPi 5 running Ubuntu 24 LTS
# Run as root or with sudo: sudo bash setup_rpi.sh [hostname]

set -e

HOSTNAME="${1:-qbot}"

echo "=== RPi 5 Setup Script ==="
echo "Hostname will be set to: $HOSTNAME"
echo ""

# Update package list
echo "[1/4] Updating package list..."
apt-get update -qq

# Install avahi-daemon for mDNS (hostname.local resolution)
echo "[2/4] Installing avahi-daemon..."
apt-get install -y avahi-daemon avahi-utils libnss-mdns

# Set hostname
echo "[3/4] Setting hostname to '$HOSTNAME'..."
hostnamectl set-hostname "$HOSTNAME"

# Update /etc/hosts to reflect new hostname
if grep -q "127.0.1.1" /etc/hosts; then
    sed -i "s/^127\.0\.1\.1.*/127.0.1.1\t$HOSTNAME/" /etc/hosts
else
    echo "127.0.1.1	$HOSTNAME" >> /etc/hosts
fi

# Configure avahi to advertise on all interfaces
echo "[4/4] Configuring and enabling avahi-daemon..."
cat > /etc/avahi/avahi-daemon.conf << 'EOF'
[server]
host-name-from-machine-id=no
browse-domains=
use-ipv4=yes
use-ipv6=yes
ratelimit-interval-usec=1000000
ratelimit-burst=1000

[wide-area]
enable-wide-area=yes

[publish]
publish-addresses=yes
publish-hinfo=yes
publish-workstation=yes
publish-domain=yes

[reflector]
enable-reflector=no

[rlimits]
EOF

# Enable and start avahi-daemon
systemctl enable avahi-daemon
systemctl restart avahi-daemon

echo ""
echo "=== Setup complete! ==="
echo ""
echo "Your Pi is now reachable at: ${HOSTNAME}.local"
echo "Test from another machine:   ping ${HOSTNAME}.local"
echo "SSH into it:                 ssh <user>@${HOSTNAME}.local"
echo ""
echo "Note: The connecting machine also needs mDNS support:"
echo "  Linux  -> install avahi-daemon (same as above)"
echo "  macOS  -> works out of the box (Bonjour)"
echo "  Windows -> install Bonjour (comes with iTunes) or enable 'mDNS' in Windows settings"
