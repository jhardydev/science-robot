# Quick X11 Forwarding Troubleshooting

## Problem: "Can't open display:" error

This means the `DISPLAY` environment variable is not set.

## Problem: "X11 forwarding requested but DISPLAY not set"

This means your **local machine** (Mac) doesn't have DISPLAY set. SSH needs this to forward X11.

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

### 5. macOS XQuartz setup (CRITICAL)

**On your Mac:**

1. **Open XQuartz** (Applications > Utilities > XQuartz)
   - If it's not installed: `brew install --cask xquartz`
   - Or download from: https://www.xquartz.org/

2. **Configure XQuartz:**
   - XQuartz > Preferences > Security
   - ✅ Check "Allow connections from network clients"
   - Close preferences

3. **Restart XQuartz:**
   - Quit XQuartz completely (Cmd+Q)
   - Reopen XQuartz

4. **Check DISPLAY is set on your Mac:**
   ```bash
   # In a NEW terminal window (after XQuartz is running):
   echo $DISPLAY
   ```
   
   **If DISPLAY is empty**, you need to set it:
   ```bash
   # Find your XQuartz display:
   ls -la /tmp/.X11-unix
   # Should show files like X0, X1, etc.
   
   # Set DISPLAY (adjust X0 to match what you see):
   export DISPLAY=:0
   # Or try:
   export DISPLAY=/private/tmp/com.apple.launchd.*/org.xquartz:0
   
   # Make it permanent in your shell profile:
   echo 'export DISPLAY=:0' >> ~/.zshrc  # or ~/.bash_profile
   source ~/.zshrc
   ```

5. **Restart your Mac** (after first installing XQuartz)

6. **Verify XQuartz is running:**
   ```bash
   # Check if XQuartz process is running:
   ps aux | grep -i xquartz
   # Should show Xquartz process
   ```

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

## Fix: "X11 forwarding requested but DISPLAY not set"

**This error means your Mac doesn't have DISPLAY set.**

### Step-by-step fix:

1. **On your Mac, check if DISPLAY is set:**
   ```bash
   echo $DISPLAY
   ```

2. **If empty, start XQuartz:**
   - Open XQuartz application
   - Make sure it's running (check Dock or Activity Monitor)

3. **Set DISPLAY on your Mac:**
   ```bash
   # Try these in order:
   export DISPLAY=:0
   # Or:
   export DISPLAY=/private/tmp/com.apple.launchd.*/org.xquartz:0
   # Or find it:
   echo $DISPLAY
   ```

4. **Make it permanent:**
   ```bash
   # Add to your shell profile:
   echo 'export DISPLAY=:0' >> ~/.zshrc
   source ~/.zshrc
   ```

5. **Now try SSH again:**
   ```bash
   ssh -X duckie@robot1
   # Check verbose output:
   ssh -v -X duckie@robot1 2>&1 | grep -i display
   # Should NOT show "DISPLAY not set"
   ```

6. **On robot, verify:**
   ```bash
   echo $DISPLAY
   # Should show: localhost:10.0
   xeyes
   ```

