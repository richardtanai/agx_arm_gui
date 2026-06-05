# USB CAN Adapter Setup on Jetson Orin Nano

Connecting the Agilex Piper arm to the Jetson Orin Nano dev board via a
candleLight USB-CAN adapter (Geschwister Schneider, VID `1d50` / PID `606f`).

---

## Hardware

| Item | Detail |
|------|--------|
| Board | Jetson Orin Nano Developer Kit |
| Kernel | `5.15.148-tegra` |
| CAN adapter | candleLight / bytewerk.org USB-CAN (`1d50:606f`) |
| Arm | Agilex Piper — exposes **CANH** (yellow) and **CANL** (blue) |

### Why the onboard CAN (`can0`) did not work

The onboard mttcan controller (`c310000.mttcan`) exposes logic-level CAN TX/RX
pins that require an external SN65HVD230 transceiver to drive the CANH/CANL
differential bus. Without the transceiver the interface shows `NO-CARRIER`
indefinitely. The USB CAN adapter has the transceiver built in, so only two
wires (CANH/CANL) are needed.

---

## One-time setup (do this once after a fresh install)

### 1 — Compile and install the `gs_usb` kernel module

The `gs_usb` driver is **not** compiled into the Tegra kernel
(`CONFIG_CAN_GS_USB is not set`). Build it as an out-of-tree module:

```bash
mkdir -p ~/gs_usb_build && cd ~/gs_usb_build

# Download the driver source matching kernel 5.15
curl -fsSL \
  "https://raw.githubusercontent.com/torvalds/linux/v5.15/drivers/net/can/usb/gs_usb.c" \
  -o gs_usb.c

# Minimal Makefile
cat > Makefile << 'EOF'
obj-m += gs_usb.o
KDIR := /lib/modules/5.15.148-tegra/build
all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules
clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
EOF

make

# Install
sudo cp gs_usb.ko /lib/modules/5.15.148-tegra/updates/
sudo depmod -a
sudo modprobe gs_usb
```

Verify the adapter appeared:
```bash
ip link show type can
# Expected: can0 (onboard mttcan) + can1 (USB adapter)
```

### 2 — Load `gs_usb` automatically on boot

```bash
echo 'gs_usb' | sudo tee -a /etc/modules
```

### 3 — Auto-configure `can1` on plug-in (udev rule)

```bash
sudo tee /etc/udev/rules.d/80-can1.rules << 'EOF'
SUBSYSTEM=="net", ACTION=="add", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f", \
  RUN+="/sbin/ip link set %k type can bitrate 1000000", \
  RUN+="/sbin/ip link set %k up"
EOF
sudo udevadm control --reload-rules
```

`%k` is the kernel-assigned interface name (typically `can1`). Avoiding a fixed
`NAME="can1"` prevents conflicts if the onboard `can0` is also present.

### 4 — Passwordless sudo for CAN commands

`headless.sh` calls `ip` and `ethtool` with sudo. Add a sudoers drop-in:

```bash
echo 'dmat2 ALL=(ALL) NOPASSWD: /usr/sbin/ip, /usr/sbin/ethtool, /sbin/modprobe' \
  | sudo tee /etc/sudoers.d/agx_arm_can
sudo chmod 440 /etc/sudoers.d/agx_arm_can
```

---

## Boot sequence (after setup)

```
Boot
 └─ /etc/modules loads gs_usb
     └─ USB adapter enumerated → can1 created
         └─ udev rule fires: bitrate=1Mbit/s, link up
             └─ headless.sh --hivemq runs successfully
```

---

## headless.sh changes required

`headless.sh` needed two changes to support the USB adapter alongside the
onboard CAN:

**1. Use `can1` instead of `can0`** (lines 22–25):
```bash
CAN_IFACE="can1"
CAN_PORT="can1"
```

**2. Pass the USB hardware address to `can_activate.sh`** so it can identify
the correct interface when multiple CAN interfaces are present:
```bash
CAN_USB_ADDR="$(sudo ethtool -i "${CAN_IFACE}" 2>/dev/null | awk '/bus-info/{print $2}')"
bash "${CAN_SCRIPT}" "${CAN_IFACE}" 1000000 ${CAN_USB_ADDR} \
    || die "can_activate.sh failed — check that the USB-CAN adapter is plugged in."
```

---

## Wiring (CANH / CANL only)

The Piper arm exposes only two CAN wires:

| Wire colour | Signal | Connect to |
|-------------|--------|-----------|
| Yellow | CANH | USB adapter CANH |
| Blue | CANL | USB adapter CANL |

No transceiver, no termination resistor needed — the candleLight adapter
includes both internally.

---

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| `modprobe: FATAL: Module gs_usb not found` | Module not compiled/installed | Redo Step 1 |
| Only `can0` visible after plugging adapter | `gs_usb` not loaded | `sudo modprobe gs_usb` |
| `can1` visible but `NOARP` (not UP) | udev rule not applied | Unplug/replug adapter, or run `ip link set can1 type can bitrate 1000000 && ip link set can1 up` manually |
| `can_activate.sh` error: number of CAN modules ≠ 1 | Two interfaces detected, no USB address passed | Ensure the `ethtool` auto-detect change is in `headless.sh` |
| `Failed to get firmware version` in arm controller | `can1` is UP but arm is off or cable unplugged | Power on arm, check CANH/CANL wiring |
