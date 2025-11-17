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

