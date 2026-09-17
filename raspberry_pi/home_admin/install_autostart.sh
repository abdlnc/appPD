#!/usr/bin/env bash
# B.I.T.S. AGV auto-start INSTALLER
# Patakbuhin sa Pi pagkatapos mong i-WinSCP ang 3 files sa ~/:
#     agv_autostart.launch.py   start_agv.sh   agv.service
# Tapos:   sudo bash ~/install_autostart.sh

set -e

# Service file ay naka-set sa User=admin, kaya admin home ang target.
# (Kung ibang username ang Pi mo, palitan ang USER_HOME dito AT ang User=/Environment=HOME= sa agv.service.)
USER_HOME="/home/admin"

if [ "$EUID" -ne 0 ]; then
    echo "!! Patakbuhin nang may sudo:  sudo bash ~/install_autostart.sh" >&2
    exit 1
fi

# 1. tiyaking nandiyan ang 3 files
missing=0
for f in agv_autostart.launch.py start_agv.sh agv.service; do
    if [ ! -f "$USER_HOME/$f" ]; then
        echo "!! KULANG: $USER_HOME/$f  — i-WinSCP muna ito sa ~/." >&2
        missing=1
    fi
done
[ "$missing" -eq 1 ] && exit 1

# 2. install
chmod +x "$USER_HOME/start_agv.sh"
cp "$USER_HOME/agv.service" /etc/systemd/system/agv.service
systemctl daemon-reload

echo ""
echo "=== OK. Na-install ang agv.service. Hindi pa naka-enable sa boot (sinadya). ==="
echo ""
echo "--- STEP 1: TEST muna (hindi kailangang mag-reboot) ---"
echo "    sudo systemctl start agv.service"
echo "    journalctl -u agv.service -f        # panoorin: aakyat dapat lahat ng nodes, walang crash-loop"
echo "    ( Ctrl+C para ihinto ang panonood ng log )"
echo ""
echo "--- STEP 2: buksan ang app, i-connect sa Pi IP — dapat WALANG TYPING sa Pi ---"
echo ""
echo "--- STEP 3: kapag malinis na, saka i-enable sa boot + i-confirm ---"
echo "    sudo systemctl enable agv.service"
echo "    sudo reboot                          # dapat kusang umakyat ang stack pagkabukas"
echo ""
echo "--- Para sa manual rehearsal / hardware test mamaya: i-stop muna (kinukuha nito ang GPIO+Lidar) ---"
echo "    sudo systemctl stop agv.service"
echo ""
