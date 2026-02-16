# ESP32 micro-ROS WiFi Transport Test

This test verifies micro-ROS communication over WiFi UDP between an ESP32 and a micro-ROS agent running on the same network.

## How It Works

1. ESP32 connects to a WiFi network using credentials from `platformio.ini`
2. micro-ROS agent runs on a machine on the same network (e.g., Raspberry Pi or laptop)
3. ESP32 sends/receives DDS-XRCE messages over UDP to the agent's IP:port
4. The agent bridges these to the ROS2 network as normal topics/services

## Network Setup

You need the ESP32 and the micro-ROS agent machine on the same network. Two common setups:

### Option A: Existing WiFi Router

Both the ESP32 and the agent machine (Pi/laptop) connect to an existing WiFi router.

1. Connect the agent machine to the router's WiFi or Ethernet
2. Assign the agent machine a **static IP** (e.g., `192.168.1.100`):
   - Via router DHCP reservation (recommended), or
   - Manually with `nmcli`:
     ```bash
     nmcli con mod "YourConnection" ipv4.addresses 192.168.1.100/24
     nmcli con mod "YourConnection" ipv4.method manual
     nmcli con mod "YourConnection" ipv4.gateway 192.168.1.1
     nmcli con up "YourConnection"
     ```
3. Ensure both devices are on the same subnet (`192.168.1.x/24`)
4. Allow UDP port 8888 through the firewall:
   ```bash
   sudo ufw allow 8888/udp
   ```

### Option B: Raspberry Pi as Access Point

The Pi creates its own WiFi network. Useful in the field without a router.

1. Install `hostapd` and `dnsmasq`:
   ```bash
   sudo apt install hostapd dnsmasq
   ```

2. Configure `/etc/hostapd/hostapd.conf`:
   ```
   interface=wlan0
   driver=nl80211
   ssid=SEC26_Robot
   hw_mode=g
   channel=6
   wmm_enabled=0
   auth_algs=1
   wpa=2
   wpa_passphrase=sec26wifi
   wpa_key_mgmt=WPA-PSK
   rsn_pairwise=CCMP
   ```

3. Configure `/etc/dnsmasq.conf`:
   ```
   interface=wlan0
   dhcp-range=192.168.1.150,192.168.1.250,255.255.255.0,24h
   ```

4. Set Pi's static IP on `wlan0` (e.g., via `/etc/dhcpcd.conf` or netplan):
   ```
   interface wlan0
   static ip_address=192.168.1.100/24
   nohook wpa_supplicant
   ```

5. Enable and start services:
   ```bash
   sudo systemctl unmask hostapd
   sudo systemctl enable hostapd dnsmasq
   sudo systemctl start hostapd dnsmasq
   ```

6. (Optional) Enable IP forwarding for internet access:
   ```bash
   echo 1 | sudo tee /proc/sys/net/ipv4/ip_forward
   sudo iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE
   ```

## ESP32 Configuration

Edit the `[esp32_microros_wifi]` section in `mcu_ws/platformio.ini`:

```ini
build_flags =
    ${esp32_base.build_flags}
    -DMICRO_ROS_TRANSPORT_ARDUINO_WIFI
    '-DAGENT_IP={ 192, 168, 1, 100 }'     ; IP of the machine running the agent
    '-DAGENT_PORT=8888'
    '-DWIFI_SSID="YourNetworkName"'        ; WiFi SSID (must be 2.4GHz)
    '-DWIFI_PASSWORD="YourPassword"'       ; WiFi password
    '-DLOCAL_MAC={ 0xAA, 0xBB, 0xCC, 0xEE, 0xDD, 0xFF }'
    '-DLOCAL_IP={ 192, 168, 1, 200 }'     ; Static IP for the ESP32
```

**Important**: `WIFI_SSID` and `WIFI_PASSWORD` must match your network. `AGENT_IP` must point to the machine running the micro-ROS agent.

## Build, Flash, and Test

All commands run inside the Docker container.

### 1. Build

```bash
pio run -e esp32-test-microros-wifi
```

### 2. Flash

```bash
pio run -e esp32-test-microros-wifi --target upload
```

### 3. Start the micro-ROS agent

On the agent machine (Pi or laptop), with ROS2 sourced:

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

### 4. Monitor serial output

```bash
pio device monitor -e esp32-test-microros-wifi
```

Expected output:
```
=== ESP32 micro-ROS WiFi Transport Test ===
Agent: 192.168.1.100:8888
SSID:  YourNetworkName
Setup complete. Waiting for agent...
[..] Waiting for agent connection...
[..] Waiting for agent connection...
[OK] Agent connected - publishing heartbeat
[OK] Agent connected - publishing heartbeat
```

### 5. Verify on the ROS2 side

```bash
ros2 topic list
# Should show: /example_subsystem/status

ros2 topic echo /example_subsystem/status std_msgs/msg/String
# Should show: data: 'OK' every second
```

## Troubleshooting

### WiFi not connecting

- **Wrong credentials**: Double-check `WIFI_SSID` and `WIFI_PASSWORD` in `platformio.ini`
- **5GHz network**: ESP32 only supports **2.4GHz WiFi**. Make sure your router/AP is broadcasting on 2.4GHz
- **Hidden SSID**: ESP32 may not connect to hidden networks without additional configuration
- **Too far from AP**: Move the ESP32 closer to the access point

### Agent not found

- **IP mismatch**: Verify `AGENT_IP` matches the actual IP of the machine running the agent (`ip addr` or `ifconfig`)
- **Firewall blocking UDP**: Allow port 8888: `sudo ufw allow 8888/udp`
- **Wrong port**: Ensure `AGENT_PORT` matches the `--port` flag passed to the agent
- **Different subnet**: Both devices must be on the same subnet (e.g., both `192.168.1.x`)
- **Agent not running**: Confirm the agent process is active

### Frequent disconnects

- **Weak signal**: Check RSSI in serial output; move ESP32 closer to AP
- **Power supply**: ESP32 WiFi draws significant current; use a stable 5V supply, not just USB from a laptop
- **Network congestion**: Reduce other traffic on the network, or use a dedicated AP (Option B)
- **Agent crashed**: Check the agent terminal for errors; restart if needed
