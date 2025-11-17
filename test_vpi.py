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
    
    # Check image format
    print(f"  Image format: {vpi_img.format}")
    print(f"  Image size: {vpi_img.width}x{vpi_img.height}")
    
    # Try resize operation - VPI Image has rescale method
    # Note: rescale() may require specific formats or backends
    resized = None
    try:
        # Method 1: Try CUDA backend with original format
        resized = vpi_img.rescale((50, 50), interp=vpi.Interp.LINEAR, backend=vpi.Backend.CUDA)
        print("✓ VPI resize operation (rescale with CUDA): OK")
    except (ValueError, AttributeError, TypeError) as e1:
        print(f"  CUDA backend failed: {e1}")
        try:
            # Method 2: Try CPU backend (supports more formats)
            resized = vpi_img.rescale((50, 50), interp=vpi.Interp.LINEAR, backend=vpi.Backend.CPU)
            print("✓ VPI resize operation (rescale with CPU): OK")
        except Exception as e2:
            print(f"  CPU backend failed: {e2}")
            try:
                # Method 3: Convert format first, then rescale
                if vpi_img.format != vpi.BGR8:
                    print(f"  Converting format from {vpi_img.format} to BGR8...")
                    converted_img = vpi_img.convert(vpi.BGR8, backend=vpi.Backend.CUDA)
                    resized = converted_img.rescale((50, 50), interp=vpi.Interp.LINEAR, backend=vpi.Backend.CUDA)
                else:
                    # Format is BGR8, try different backend
                    resized = vpi_img.rescale((50, 50), interp=vpi.Interp.LINEAR, backend=vpi.Backend.VIC)
                print("✓ VPI resize operation (with format conversion): OK")
            except Exception as e3:
                print(f"✗ VPI resize operation failed: {e1}, {e2}, {e3}")
                print("  Note: VPI rescale() may not support this image format on available backends")
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

