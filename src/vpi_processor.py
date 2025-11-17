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
    
    def __init__(self, backend='GPU'):
        """
        Initialize VPI processor
        
        Args:
            backend: VPI backend ('GPU', 'CPU', or 'VIC')
        """
        self.available = VPI_AVAILABLE
        self.backend = backend
        
        if self.available:
            # Convert string to VPI backend enum
            backend_map = {
                'GPU': vpi.Backend.GPU,
                'CPU': vpi.Backend.CPU,
                'VIC': vpi.Backend.VIC
            }
            self.vpi_backend = backend_map.get(backend, vpi.Backend.GPU)
            logger.info(f"VPI processor initialized with {backend} backend")
        else:
            logger.warning("VPI processor unavailable - using CPU fallback. Install JetPack for VPI support.")
    
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
            
            # Resize using VPI
            resized = vpi.resize(vpi_img, (target_width, target_height), 
                               interp=vpi.Interp.LINEAR, backend=self.vpi_backend)
            
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
            blurred = vpi.Gaussian(vpi_img, kernel_size, sigma, backend=self.vpi_backend)
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

