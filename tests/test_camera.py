"""
Unit tests for camera module
"""
import unittest
from unittest.mock import Mock, patch
from src.camera import Camera


class TestCamera(unittest.TestCase):
    """Test camera initialization and frame capture"""
    
    @patch('cv2.VideoCapture')
    def test_camera_initialization(self, mock_video_capture):
        """Test camera initializes correctly"""
        mock_cap = Mock()
        mock_cap.isOpened.return_value = True
        mock_cap.get.side_effect = lambda prop: {
            3: 640,  # CAP_PROP_FRAME_WIDTH
            4: 480,  # CAP_PROP_FRAME_HEIGHT
        }.get(prop, 0)
        mock_video_capture.return_value = mock_cap
        
        camera = Camera()
        result = camera.initialize()
        
        self.assertTrue(result)
        mock_cap.set.assert_called()
    
    @patch('cv2.VideoCapture')
    def test_camera_read_frame(self, mock_video_capture):
        """Test frame reading"""
        mock_cap = Mock()
        mock_cap.isOpened.return_value = True
        mock_cap.read.return_value = (True, Mock())
        mock_video_capture.return_value = mock_cap
        
        camera = Camera()
        camera.cap = mock_cap
        
        success, frame = camera.read_frame()
        self.assertTrue(success)
        self.assertIsNotNone(frame)


if __name__ == '__main__':
    unittest.main()

