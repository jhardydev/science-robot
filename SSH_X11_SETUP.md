# SSH and X11 Display Setup Guide

This guide explains how to configure SSH and X11 forwarding to view the robot's display output remotely.

## Overview

There are **two main approaches** to view the robot's display:

1. **X11 Forwarding** - Forward the container's X11 display through SSH to your local machine
2. **VNC** - Use VNC to view a virtual display running in the container

## Option 1: X11 Forwarding (Recommended for macOS/Linux)

X11 forwarding allows you to display GUI applications from the robot on your local machine.

### Prerequisites

**On your local machine (macOS):**
- Install [XQuartz](https://www.xquartz.org/) (X11 server for macOS)
- Restart your computer after installing XQuartz

**On your local machine (Linux):**
- X11 server is typically already installed
- If not: `sudo apt-get install xorg`

**On your local machine (Windows):**
- Install [VcXsrv](https://sourceforge.net/projects/vcxsrv/) or [Xming](https://sourceforge.net/projects/xming/)
- Or use WSL2 with X11 forwarding

### SSH Server Configuration (on Robot)

1. **Edit SSH server config:**
   ```bash
   sudo nano /etc/ssh/sshd_config
   ```

2. **Ensure these lines are present and uncommented:**
   ```bash
   X11Forwarding yes
   X11DisplayOffset 10
   X11UseLocalhost yes
   ```

3. **Restart SSH service:**
   ```bash
   sudo systemctl restart sshd
   # Or on older systems:
   sudo service ssh restart
   ```

4. **Install X11 packages on robot (if not already installed):**
   ```bash
   sudo apt-get update
   sudo apt-get install xauth x11-apps
   ```

### SSH Client Configuration (on Your Machine)

**1. Start X11 server on your local machine:**

**macOS:**
- Open XQuartz (Applications > Utilities > XQuartz)
- In XQuartz preferences, go to "Security" tab
- Check "Allow connections from network clients"
- Restart XQuartz
- You may need to restart your computer after first installing XQuartz

**Linux:**
- X11 server should already be running
- If not: `sudo systemctl start display-manager` (or your display manager)

**2. Connect with X11 forwarding:**
```bash
ssh -X username@robot-ip
# Or with trusted forwarding (less secure but more compatible):
ssh -Y username@robot-ip
```

**3. Verify DISPLAY is set:**
```bash
# After SSH'd into robot, check:
echo $DISPLAY
# Should show something like: localhost:10.0 or localhost:11.0
```

**4. Test X11 forwarding:**
```bash
# If DISPLAY is not set, try:
export DISPLAY=localhost:10.0
# Or check what it should be:
echo $DISPLAY

# Then test:
xeyes
# Or:
xclock
```

If you see a window appear on your local machine, X11 forwarding is working!

**5. If DISPLAY is still not set:**
```bash
# Check if X11 forwarding is enabled in SSH connection:
ssh -v -X username@robot-ip
# Look for lines like:
# debug1: Requesting X11 forwarding with authentication spoofing.
# debug1: Requesting authentication agent forwarding.

# Check SSH server config on robot:
grep -i x11 /etc/ssh/sshd_config
# Should show:
# X11Forwarding yes
```

### Running the Container with X11 Forwarding

1. **SSH into the robot with X11 forwarding:**
   ```bash
   ssh -X username@robot-ip
   ```

2. **On the robot, set DISPLAY to forward through SSH:**
   ```bash
   export DISPLAY=localhost:10.0
   # Or check what DISPLAY is set to:
   echo $DISPLAY
   ```

3. **Run the container with X11 forwarding:**
   ```bash
   cd ~/science-robot
   ./docker-run.sh --display-output
   ```

   The container will use the X11 display forwarded through SSH.

### Troubleshooting X11 Forwarding

**"Can't open display:" or DISPLAY not set:**
1. **Check if DISPLAY is set:**
   ```bash
   echo $DISPLAY
   # If empty, try:
   export DISPLAY=localhost:10.0
   # Or check what SSH set:
   ssh -v -X username@robot-ip 2>&1 | grep -i display
   ```

2. **Verify XQuartz is running (macOS):**
   - Open XQuartz application
   - Check XQuartz preferences > Security > "Allow connections from network clients"
   - Restart XQuartz and try again

3. **Check SSH X11 forwarding is working:**
   ```bash
   # On your local machine, before SSH:
   echo $DISPLAY
   # Should show your local display (e.g., /private/tmp/com.apple.launchd.xxx/org.xquartz:0)
   
   # After SSH with -X:
   ssh -X username@robot-ip
   echo $DISPLAY
   # Should show something like localhost:10.0
   ```

4. **Verify SSH server config:**
   ```bash
   # On robot:
   sudo grep -i x11 /etc/ssh/sshd_config
   # Should show:
   # X11Forwarding yes
   # X11DisplayOffset 10
   # X11UseLocalhost yes
   
   # If not, edit and restart:
   sudo nano /etc/ssh/sshd_config
   sudo systemctl restart sshd
   ```

5. **Check xauth is installed:**
   ```bash
   # On robot:
   which xauth
   # If not found:
   sudo apt-get install xauth
   ```

6. **Try trusted forwarding:**
   ```bash
   ssh -Y username@robot-ip
   # -Y is less secure but more permissive
   ```

**"Cannot connect to X server" error:**
- Make sure XQuartz/X server is running on your local machine
- Check `echo $DISPLAY` - should be something like `localhost:10.0`
- Try `xauth list` to see if authentication is set up
- Restart XQuartz and reconnect via SSH

**"X11 connection rejected" error:**
- Check SSH server config has `X11Forwarding yes`
- Restart SSH service on robot: `sudo systemctl restart sshd`
- Try `ssh -Y` instead of `ssh -X` (trusted forwarding)
- Check XQuartz security settings allow network connections

**"Bad display name" error:**
- Set `export DISPLAY=localhost:10.0` before running container
- Or use `-e DISPLAY=$DISPLAY` in docker-run.sh
- Verify with: `xhost +localhost` (less secure, for testing only)

**"No protocol specified" error:**
- X11 authentication issue
- Try: `xauth list` to see available auth entries
- May need to restart SSH connection
- On macOS, ensure XQuartz security allows network clients

## Option 2: VNC (Virtual Display)

VNC allows you to view a virtual display running inside the container via a VNC client.

### Prerequisites

**On your local machine:**
- Install a VNC client:
  - macOS: [TigerVNC Viewer](https://github.com/TigerVNC/tigervnc/releases) or [RealVNC Viewer](https://www.realvnc.com/download/viewer/)
  - Linux: `sudo apt-get install tigervnc-viewer` or `sudo apt-get install remmina`
  - Windows: [TigerVNC Viewer](https://github.com/TigerVNC/tigervnc/releases) or [RealVNC Viewer](https://www.realvnc.com/download/viewer/)

### Running Container with VNC

1. **Run container with virtual display and VNC enabled:**
   ```bash
   cd ~/science-robot
   ENABLE_VNC=true VNC_PORT=5900 ./docker-run.sh --virtual-display
   ```

   Or modify `docker-run.sh` to add:
   ```bash
   -e ENABLE_VNC=true \
   -e VNC_PORT=5900 \
   -p 5900:5900 \
   ```

2. **Connect with VNC client:**
   - Open your VNC client
   - Connect to: `robot-ip:5900`
   - No password required (for testing - add password in production!)

### SSH Tunnel for VNC (More Secure)

Instead of exposing VNC port directly, tunnel it through SSH:

1. **On your local machine, create SSH tunnel:**
   ```bash
   ssh -L 5900:localhost:5900 username@robot-ip
   ```

2. **In another terminal, connect VNC client to:**
   ```
   localhost:5900
   ```

3. **Run container with VNC (no port mapping needed):**
   ```bash
   ENABLE_VNC=true VNC_PORT=5900 ./docker-run.sh --virtual-display
   ```

### Updating docker-run.sh for VNC

Add VNC port mapping when `ENABLE_VNC` is set:

```bash
# In docker-run.sh, add to docker run command:
if [ "$ENABLE_VNC" = "true" ]; then
    VNC_PORT=${VNC_PORT:-5900}
    docker run ... -p ${VNC_PORT}:${VNC_PORT} ...
fi
```

## Quick Reference

### X11 Forwarding Setup
```bash
# 1. Install XQuartz (macOS) or ensure X11 is running (Linux)
# 2. SSH with X11 forwarding:
ssh -X username@robot-ip

# 3. Run container:
./docker-run.sh --display-output
```

### VNC Setup
```bash
# 1. Run container with VNC:
ENABLE_VNC=true ./docker-run.sh --virtual-display

# 2. Connect VNC client to robot-ip:5900
# Or use SSH tunnel:
ssh -L 5900:localhost:5900 username@robot-ip
# Then connect VNC to localhost:5900
```

### Testing Display

**Test X11 forwarding:**
```bash
# After SSH with -X, run:
xeyes
```

**Test VNC:**
```bash
# In container:
DISPLAY=:99 xeyes
# View via VNC client
```

## Security Notes

- **X11 Forwarding**: Generally secure when using SSH, but be cautious with trusted forwarding (`-Y`)
- **VNC**: By default, our setup has no password. For production:
  - Set a VNC password: `x11vnc -storepasswd /path/to/passwd`
  - Use SSH tunnel instead of exposing port directly
  - Consider using `-localhost` flag to only allow local connections

## Common Issues

**"No display" errors:**
- Check `echo $DISPLAY` is set correctly
- For X11: Should be `localhost:10.0` or similar
- For VNC: Should be `:99` (virtual display)

**"Permission denied" for X11:**
- Run `xhost +localhost` on your local machine (less secure)
- Or properly configure X11 authentication

**VNC connection refused:**
- Check firewall: `sudo ufw allow 5900/tcp`
- Verify container is running: `docker ps`
- Check VNC is started: `docker logs science-robot | grep VNC`

