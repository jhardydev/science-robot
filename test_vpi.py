#!/usr/bin/env python3
"""
Simple VPI verification script
Tests if VPI is available and shows available backends
"""
import sys

print("Testing VPI installation...")
print("-" * 50)

try:
    import vpi
    print("✓ VPI module imported successfully")
except ImportError as e:
    print(f"✗ VPI module not available: {e}")
    print("\nTo install VPI:")
    print("  sudo apt-get update")
    print("  sudo apt-get install libnvvpi4 python3-vpi4")
    sys.exit(1)

# Check available backends
print("\nChecking available VPI backends...")
available_backends = []

backend_names = ['CPU', 'CUDA', 'VIC', 'PVA', 'OFA']
for name in backend_names:
    try:
        backend_attr = getattr(vpi.Backend, name, None)
        if backend_attr is not None:
            available_backends.append(name)
            print(f"  ✓ {name}: Available")
        else:
            print(f"  ✗ {name}: Not available")
    except AttributeError:
        print(f"  ✗ {name}: Not available")

if not available_backends:
    print("  ⚠ No backends found - VPI may not be properly installed")

print(f"\nAvailable backends: {', '.join(available_backends) if available_backends else 'None'}")

# Test basic VPI functionality
print("\nTesting basic VPI functionality...")
try:
    import numpy as np
    
    # Create a test image
    test_img = np.zeros((100, 100, 3), dtype=np.uint8)
    
    # Try to create a VPI image
    vpi_img = vpi.asimage(test_img)
    print("✓ VPI image creation: OK")
    
    # Try resize operation
    resized = vpi.resize(vpi_img, (50, 50), interp=vpi.Interp.LINEAR)
    print("✓ VPI resize operation: OK")
    
    # Convert back to numpy
    result = resized.cpu()
    print("✓ VPI to numpy conversion: OK")
    
    print("\n✓ VPI is fully functional!")
    
except Exception as e:
    print(f"✗ VPI functionality test failed: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

print("-" * 50)
print("VPI verification complete!")

