# Quick X11 Forwarding Troubleshooting

## Problem: "Can't open display:" error

This means the `DISPLAY` environment variable is not set.

## Quick Fix Steps

### 1. Check DISPLAY variable
```bash
# After SSH with -X, check:
echo $DISPLAY
```

If empty, try:
```bash
export DISPLAY=localhost:10.0
# Or try:
export DISPLAY=localhost:11.0
```

### 2. Verify X11 forwarding is enabled in SSH

**On your local machine (before SSH):**
```bash
# Check verbose SSH output:
ssh -v -X username@robot-ip 2>&1 | grep -i x11
```

You should see:
```
debug1: Requesting X11 forwarding with authentication spoofing.
```

**On robot (after SSH):**
```bash
# Check if xauth is working:
xauth list
# Should show entries like:
# robot1/unix:10  MIT-MAGIC-COOKIE-1  xxxxxx
```

### 3. Check SSH server configuration

**On robot:**
```bash
# Check SSH server config:
sudo grep -i x11 /etc/ssh/sshd_config
```

Should show:
```
X11Forwarding yes
X11DisplayOffset 10
X11UseLocalhost yes
```

If not, edit:
```bash
sudo nano /etc/ssh/sshd_config
# Add or uncomment:
X11Forwarding yes
X11DisplayOffset 10
X11UseLocalhost yes

# Save and restart:
sudo systemctl restart sshd
```

### 4. Install required packages on robot

```bash
# Install xauth and X11 test apps:
sudo apt-get update
sudo apt-get install xauth x11-apps
```

### 5. macOS XQuartz setup

**On your Mac:**
1. Open XQuartz (Applications > Utilities > XQuartz)
2. Go to XQuartz > Preferences > Security
3. Check "Allow connections from network clients"
4. **Restart XQuartz** (quit and reopen)
5. **Restart your Mac** (after first install)

### 6. Test X11 forwarding

**On robot (after SSH with -X):**
```bash
# Set DISPLAY if needed:
export DISPLAY=localhost:10.0

# Test:
xeyes
# Or:
xclock
```

If a window appears on your Mac, X11 forwarding is working!

### 7. Alternative: Use trusted forwarding

If `-X` doesn't work, try `-Y` (less secure but more permissive):
```bash
ssh -Y username@robot-ip
```

### 8. Check your local DISPLAY

**On your Mac (before SSH):**
```bash
echo $DISPLAY
# Should show something like:
# /private/tmp/com.apple.launchd.xxx/org.xquartz:0
```

**After SSH with -X:**
```bash
echo $DISPLAY
# Should show:
# localhost:10.0
```

## Common Issues

### Issue: DISPLAY is empty after SSH
**Solution:** 
- Verify SSH server has `X11Forwarding yes`
- Check `xauth` is installed on robot
- Try `ssh -Y` instead of `ssh -X`
- Restart SSH service on robot

### Issue: "X11 connection refused"
**Solution:**
- XQuartz security settings - allow network clients
- Restart XQuartz
- Check firewall isn't blocking X11

### Issue: Works locally but not in Docker
**Solution:**
- Pass DISPLAY to container: `-e DISPLAY=$DISPLAY`
- Mount X11 socket: `-v /tmp/.X11-unix:/tmp/.X11-unix:rw`
- Mount Xauthority: `-v ~/.Xauthority:/root/.Xauthority:rw`

## Quick Test Script

Save this as `test-x11.sh` on robot:

```bash
#!/bin/bash
echo "DISPLAY: $DISPLAY"
echo "Xauth entries:"
xauth list
echo ""
echo "Testing xeyes (should open window on your Mac)..."
xeyes
```

Run: `chmod +x test-x11.sh && ./test-x11.sh`

