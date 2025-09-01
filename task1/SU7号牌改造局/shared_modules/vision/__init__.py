"""
视觉模块
提供二维码识别、图像处理、统一识别等功能
"""

# 导入现有模块
try:
    from .qr_recognition import QRRecognitionModule
except ImportError:
    QRRecognitionModule = None

try:
    from .unified_recognition import UnifiedRecognition
except ImportError:
    UnifiedRecognition = None

from .unified_recognition_simple import SimpleUnifiedRecognition

# 导入新增模块
try:
    from .qrcode_recognition import QRCodeRecognizer, recognize_qr_warehouse_code
    _qrcode_available = True
except ImportError as e:
    print(f"QR码识别模块导入失败: {e}")
    QRCodeRecognizer = None
    recognize_qr_warehouse_code = None
    _qrcode_available = False

try:
    from .doubao_api import DoubaoAPI, recognize_warehouse_with_doubao
    _doubao_available = True
except ImportError as e:
    print(f"豆包API模块导入失败: {e}")
    DoubaoAPI = None
    recognize_warehouse_with_doubao = None
    _doubao_available = False

# 构建导出列表
__all__ = ['SimpleUnifiedRecognition']

if QRRecognitionModule:
    __all__.append('QRRecognitionModule')
if UnifiedRecognition:
    __all__.append('UnifiedRecognition')
if _qrcode_available:
    __all__.extend(['QRCodeRecognizer', 'recognize_qr_warehouse_code'])
if _doubao_available:
    __all__.extend(['DoubaoAPI', 'recognize_warehouse_with_doubao'])
