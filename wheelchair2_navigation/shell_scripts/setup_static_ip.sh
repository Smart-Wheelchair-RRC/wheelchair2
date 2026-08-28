#!/usr/bin/env bash

# Exit immediately if any command fails
set -e

# Default variables
CONNECTION_NAME="wheelchair@rrc"
STATIC_IP="192.168.0.50/24"
GATEWAY="192.168.0.1"
DNS="8.8.8.8,1.1.1.1"

# Print banner
echo "============================================="
echo "   Wheelchair Jetson Static IP Setup Tool"
echo "============================================="

# Ensure script is run with sudo
if [ "$EUID" -ne 0 ]; then
  echo "❌ Error: Please run this script with sudo."
  exit 1
fi

# Ask for input or use defaults
read -p "Enter Wi-Fi Connection Name [\$CONNECTION_NAME]: " user_conn
CONNECTION_NAME=${user_conn:-$CONNECTION_NAME}

read -p "Enter Desired Static IP with CIDR [\$STATIC_IP]: " user_ip
STATIC_IP=${user_ip:-$STATIC_IP}

read -p "Enter Gateway IP [\$GATEWAY]: " user_gateway
GATEWAY=${user_gateway:-$GATEWAY}

read -p "Enter DNS Servers [\$DNS]: " user_dns
DNS=${user_dns:-$DNS}

echo ""
echo "Configuration summary:"
echo "----------------------"
echo "Connection:  \$CONNECTION_NAME"
echo "Static IP:   \$STATIC_IP"
echo "Gateway:     \$GATEWAY"
echo "DNS:         \$DNS"
echo "----------------------"
read -p "Do you want to apply this configuration? (y/n) [y]: " confirm
confirm=${confirm:-y}

if [[ ! "$confirm" =~ ^[Yy]$ ]]; then
  echo "Aborted."
  exit 0
fi

# Apply configuration via nmcli
echo "🔧 Configuring NetworkManager connection '\$CONNECTION_NAME'..."
nmcli connection modify "$CONNECTION_NAME" \
  ipv4.method manual \
  ipv4.addresses "$STATIC_IP" \
  ipv4.gateway "$GATEWAY" \
  ipv4.dns "$DNS" \
  connection.autoconnect yes

echo "🔄 Reloading network connections..."
nmcli connection reload

echo "⚡ Applying configuration (bringing connection up)..."
nmcli connection up "$CONNECTION_NAME"

echo "✅ Success! Static IP configured and activated."
