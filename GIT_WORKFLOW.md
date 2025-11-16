# Git Workflow Guide

This guide covers common git operations for working with the science-robot repository.

## Updating Your Local Clone

### Basic Update (Recommended)

```bash
# Navigate to repository
cd ~/science-robot

# Pull latest changes from remote
git pull

# Or specify remote and branch explicitly
git pull origin main
```

### Safe Update (Check Status First)

```bash
# Check current status
git status

# See what changes are on remote
git fetch

# Compare local vs remote
git log HEAD..origin/main --oneline

# Pull changes
git pull origin main
```

### Update with Local Changes

If you have local uncommitted changes:

```bash
# Option 1: Stash changes, pull, then reapply
git stash
git pull
git stash pop

# Option 2: Commit your changes first
git add .
git commit -m "Your local changes"
git pull

# Option 3: See what would conflict
git fetch
git diff HEAD origin/main
```

## Common Git Operations

### Check Current Status

```bash
# See what branch you're on and status
git status

# See recent commits
git log --oneline -10

# See what's different from remote
git fetch
git log HEAD..origin/main --oneline
```

### Update from Remote (Multiple Methods)

**Method 1: Pull (fetch + merge)**
```bash
git pull origin main
```

**Method 2: Fetch then merge**
```bash
git fetch origin
git merge origin/main
```

**Method 3: Fetch then rebase (cleaner history)**
```bash
git fetch origin
git rebase origin/main
```

### Switch Branches

```bash
# List branches
git branch -a

# Switch to main branch
git checkout main
# or (newer syntax)
git switch main

# Create and switch to new branch
git checkout -b feature-branch
# or
git switch -c feature-branch
```

### View Changes

```bash
# See what changed locally
git diff

# See what changed on remote
git fetch
git diff HEAD origin/main

# See commit history
git log --oneline --graph --all
```

## Troubleshooting

### Merge Conflicts

If `git pull` results in conflicts:

```bash
# See conflicted files
git status

# Edit conflicted files to resolve conflicts
# Look for <<<<<<< HEAD markers

# After resolving, stage and commit
git add .
git commit -m "Merge remote changes"
```

### Discard Local Changes

**Warning: This will lose uncommitted changes!**

```bash
# Discard changes to tracked files
git checkout -- .

# Discard all changes including untracked files
git clean -fd
git reset --hard HEAD

# Reset to match remote exactly
git fetch origin
git reset --hard origin/main
```

### Update After Clone

If you just cloned and want to ensure you're up to date:

```bash
cd ~/science-robot
git pull origin main
```

## Workflow Examples

### Daily Development Workflow

```bash
# Start of day: Update from remote
cd ~/science-robot
git pull

# Make your changes
# ... edit files ...

# Check what changed
git status
git diff

# Commit changes
git add .
git commit -m "Description of changes"

# Push to remote (if you have write access)
git push origin main
```

### Update Before Building

```bash
# Always good to update before building
cd ~/science-robot
git pull
./docker-run.sh --build
```

### Update on Robot

```bash
# SSH to robot
ssh duckiebot@robot1

# Navigate to repository
cd ~/science-robot

# Update from GitHub
git pull origin main

# Rebuild if needed
./docker-run.sh --build
```

## Remote Repository Information

```bash
# See remote URL
git remote -v

# Should show:
# origin  https://github.com/jhardydev/science-robot.git (fetch)
# origin  https://github.com/jhardydev/science-robot.git (push)

# Change remote URL (if needed)
git remote set-url origin https://github.com/jhardydev/science-robot.git
```

## Quick Reference

| Command | Description |
|---------|-------------|
| `git pull` | Update from remote (fetch + merge) |
| `git fetch` | Download changes without merging |
| `git status` | Check current status |
| `git log --oneline -10` | See last 10 commits |
| `git diff` | See local changes |
| `git stash` | Temporarily save changes |
| `git stash pop` | Restore stashed changes |
| `git reset --hard HEAD` | Discard all local changes |

