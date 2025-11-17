#!/usr/bin/env python3
"""
Discover VPI API structure - helpful for debugging API availability
"""
import sys

print("Discovering VPI API structure...")
print("-" * 50)

try:
    import vpi
    print("✓ VPI module imported successfully")
except ImportError as e:
    print(f"✗ VPI module not available: {e}")
    sys.exit(1)

# List top-level VPI attributes
print("\nTop-level VPI attributes:")
vpi_attrs = [attr for attr in dir(vpi) if not attr.startswith('_')]
for attr in sorted(vpi_attrs):
    try:
        obj = getattr(vpi, attr)
        obj_type = type(obj).__name__
        print(f"  {attr}: {obj_type}")
    except:
        print(f"  {attr}: <error accessing>")

# Check for resize/Scale operations
print("\nChecking for resize/scale operations:")
resize_candidates = ['resize', 'Resize', 'Scale', 'scale']
for name in resize_candidates:
    attr = getattr(vpi, name, None)
    if attr is not None:
        print(f"  ✓ {name}: {type(attr).__name__}")

# Check Interp enum
print("\nChecking Interp enum:")
try:
    interp_attrs = [attr for attr in dir(vpi.Interp) if not attr.startswith('_')]
    print(f"  Available interpolation methods: {', '.join(interp_attrs)}")
except AttributeError:
    print("  ✗ vpi.Interp not available")

# Check Backend enum
print("\nChecking Backend enum:")
try:
    backend_attrs = [attr for attr in dir(vpi.Backend) if not attr.startswith('_')]
    print(f"  Available backends: {', '.join(backend_attrs)}")
except AttributeError:
    print("  ✗ vpi.Backend not available")

# Try to create a test image and see what operations are available
print("\nTesting VPI image operations:")
try:
    import numpy as np
    test_img = np.zeros((100, 100, 3), dtype=np.uint8)
    vpi_img = vpi.asimage(test_img)
    print("  ✓ vpi.asimage() works")
    print(f"  VPI image type: {type(vpi_img).__name__}")
    
    # Check what methods are available on VPI image
    img_methods = [m for m in dir(vpi_img) if not m.startswith('_')]
    print(f"  VPI image methods: {', '.join(img_methods[:10])}...")  # Show first 10
except Exception as e:
    print(f"  ✗ Error testing VPI image: {e}")

print("-" * 50)
print("Discovery complete!")

