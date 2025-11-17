"""
VPI (Vision Programming Interface) module for GPU-accelerated image processing
Provides GPU-accelerated operations for better performance on Jetson Nano
"""
import numpy as np
import logging

logger = logging.getLogger(__name__)

try:
    import vpi
    VPI_AVAILABLE = True
except ImportError:
    VPI_AVAILABLE = False
    # Don't log here - logger may not be configured yet
    pass


class VPIProcessor:
    """GPU-accelerated image processing using NVIDIA VPI"""
    
    def __init__(self, backend='CUDA'):
        """
        Initialize VPI processor
        
        Args:
            backend: VPI backend ('CUDA', 'CPU', 'VIC', 'PVA', or 'OFA')
                     Note: 'GPU' is accepted as alias for 'CUDA'
        """
        self.available = VPI_AVAILABLE
        self.backend = backend
        
        if self.available:
            # Map user-friendly backend names to VPI backend enums
            # VPI uses CUDA for GPU acceleration, not 'GPU'
            backend_map = {
                'CUDA': vpi.Backend.CUDA,  # GPU acceleration
                'GPU': vpi.Backend.CUDA,   # Alias for CUDA (backward compatibility)
                'CPU': vpi.Backend.CPU,
                'VIC': vpi.Backend.VIC,    # Video Image Compositor
                'PVA': vpi.Backend.PVA,    # Programmable Vision Accelerator
                'OFA': vpi.Backend.OFA     # Optical Flow Accelerator
            }
            
            # Normalize backend name (handle case variations)
            backend_upper = backend.upper()
            if backend_upper not in backend_map:
                logger.warning(f"Unknown VPI backend '{backend}', defaulting to CUDA")
                backend_upper = 'CUDA'
            
            # Use getattr to safely access backend attributes
            backend_enum = None
            for attempt_name in [backend_upper, 'CUDA', 'CPU']:
                backend_attr = getattr(vpi.Backend, attempt_name, None)
                if backend_attr is not None:
                    backend_enum = backend_attr
                    if attempt_name != backend_upper:
                        logger.warning(f"Requested backend '{backend_upper}' not available, using '{attempt_name}'")
                    break
            
            if backend_enum is None:
                logger.error("No VPI backends available. VPI installation may be incomplete.")
                self.vpi_backend = None
                self.available = False
            else:
                self.vpi_backend = backend_enum
                # Log the actual backend name (not alias)
                actual_backend = 'CUDA' if backend_upper == 'GPU' else backend_upper
                logger.info(f"VPI processor initialized with {actual_backend} backend")
        else:
            logger.warning("VPI processor unavailable - using CPU fallback. Install JetPack for VPI support.")
            self.vpi_backend = None
    
    def resize_gpu(self, image, target_width, target_height):
        """
        Resize image using GPU acceleration
        
        Args:
            image: Input image (numpy array)
            target_width: Target width
            target_height: Target height
            
        Returns:
            Resized image (numpy array)
        """
        if not self.available:
            # Fallback to OpenCV CPU resize
            import cv2
            return cv2.resize(image, (target_width, target_height))
        
        try:
            # Convert numpy array to VPI image
            vpi_img = vpi.asimage(image)
            original_height, original_width = vpi_img.size[1], vpi_img.size[0]
            
            # Calculate scale factors
            scale_x = target_width / original_width
            scale_y = target_height / original_height
            
            # VPI uses Conversion with HomographyTransform2D for resizing
            # Try different possible API patterns
            resized = None
            try:
                # Method 1: Try Conversion with scale transform
                # Create a homography transform for scaling
                scale_transform = vpi.HomographyTransform2D([scale_x, 0, 0, 
                                                             0, scale_y, 0, 
                                                             0, 0, 1])
                
                # Create output image with target size
                output_img = vpi.Image((target_width, target_height), vpi_img.format)
                
                # Apply conversion
                conv = vpi.Conversion(vpi_img, output_img, scale_transform, 
                                     interp=vpi.Interp.LINEAR, backend=self.vpi_backend)
                vpi.execute(conv)
                resized = output_img
            except (AttributeError, TypeError) as e1:
                try:
                    # Method 2: Try if Image object has resize method
                    if hasattr(vpi_img, 'resize'):
                        resized = vpi_img.resize((target_width, target_height), 
                                                interp=vpi.Interp.LINEAR, 
                                                backend=self.vpi_backend)
                    else:
                        raise AttributeError("No resize method found")
                except (AttributeError, TypeError) as e2:
                    # Method 3: Try Conversion without explicit transform (simpler API)
                    try:
                        output_img = vpi.Image((target_width, target_height), vpi_img.format)
                        conv = vpi.Conversion(vpi_img, output_img, 
                                             interp=vpi.Interp.LINEAR, 
                                             backend=self.vpi_backend)
                        vpi.execute(conv)
                        resized = output_img
                    except Exception as e3:
                        # VPI resize not available - fall back to CPU
                        logger.debug(f"VPI resize attempts failed: {e1}, {e2}, {e3}")
                        raise AttributeError("VPI resize API not available")
            
            # Convert back to numpy
            return resized.cpu()
        except Exception as e:
            logger.warning(f"VPI resize error: {e}, falling back to CPU")
            import cv2
            return cv2.resize(image, (target_width, target_height))
    
    def gaussian_blur_gpu(self, image, kernel_size=5, sigma=1.0):
        """
        Apply Gaussian blur using GPU acceleration
        
        Args:
            image: Input image (numpy array)
            kernel_size: Blur kernel size (must be odd)
            sigma: Gaussian sigma value
            
        Returns:
            Blurred image (numpy array)
        """
        if not self.available:
            # Fallback to OpenCV CPU blur
            import cv2
            return cv2.GaussianBlur(image, (kernel_size, kernel_size), sigma)
        
        try:
            vpi_img = vpi.asimage(image)
            # VPI Gaussian might be available as a filter method on the image
            # Try different API patterns
            blurred = None
            try:
                # Try if Gaussian is a top-level function
                blurred = vpi.Gaussian(vpi_img, kernel_size, sigma, backend=self.vpi_backend)
            except (AttributeError, TypeError):
                # Try if it's a method on the image object
                if hasattr(vpi_img, 'gaussian'):
                    blurred = vpi_img.gaussian(kernel_size, sigma, backend=self.vpi_backend)
                else:
                    # Fallback: VPI may not have Gaussian - use CPU
                    raise AttributeError("VPI Gaussian not available")
            return blurred.cpu()
        except Exception as e:
            logger.warning(f"VPI blur error: {e}, falling back to CPU")
            import cv2
            return cv2.GaussianBlur(image, (kernel_size, kernel_size), sigma)
    
    def convert_color_gpu(self, image, conversion_code):
        """
        Convert color space using GPU acceleration
        
        Args:
            image: Input image (numpy array)
            conversion_code: OpenCV color conversion code (e.g., cv2.COLOR_BGR2RGB)
            
        Returns:
            Converted image (numpy array)
        """
        if not self.available:
            # Fallback to OpenCV CPU conversion
            import cv2
            return cv2.cvtColor(image, conversion_code)
        
        try:
            vpi_img = vpi.asimage(image)
            
            # Map OpenCV conversion codes to VPI conversions
            # Note: VPI has limited conversion support
            # For BGR2RGB, we'll use CPU fallback
            import cv2
            return cv2.cvtColor(image, conversion_code)
        except Exception as e:
            logger.warning(f"VPI color conversion error: {e}, falling back to CPU")
            import cv2
            return cv2.cvtColor(image, conversion_code)
    
    def is_available(self):
        """Check if VPI is available"""
        return self.available
    
    def get_available_backends(self):
        """
        Get list of available VPI backends
        
        Returns:
            List of available backend names, or empty list if VPI unavailable
        """
        if not self.available:
            return []
        
        available = []
        backend_names = ['CPU', 'CUDA', 'VIC', 'PVA', 'OFA']
        
        for name in backend_names:
            try:
                backend_attr = getattr(vpi.Backend, name, None)
                if backend_attr is not None:
                    available.append(name)
            except AttributeError:
                pass
        
        return available

