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
    
    # Try resize operation - VPI Image has rescale method
    resized = None
    try:
        # Use rescale method on Image object - backend is required
        resized = vpi_img.rescale((50, 50), interp=vpi.Interp.LINEAR, backend=vpi.Backend.CUDA)
        print("✓ VPI resize operation (rescale): OK")
    except (AttributeError, TypeError) as e1:
        try:
            # Try with scale factors instead of target size
            scale_x = 50 / vpi_img.width
            scale_y = 50 / vpi_img.height
            resized = vpi_img.rescale((scale_x, scale_y), interp=vpi.Interp.LINEAR, backend=vpi.Backend.CUDA)
            print("✓ VPI resize operation (rescale with scale factors): OK")
        except Exception as e2:
            print(f"✗ VPI resize operation failed: {e1}, {e2}")
            print("  Note: VPI resize may use a different API")
            raise
    
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

