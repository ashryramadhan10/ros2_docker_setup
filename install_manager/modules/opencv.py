"""
OpenCV installer module
"""

import subprocess
from pathlib import Path
import sys
sys.path.append(str(Path(__file__).parent.parent))

from core.installer import BaseInstaller

class OpenCVInstaller(BaseInstaller):
    """OpenCV installer"""
    
    def detect(self) -> bool:
        """Check if OpenCV is installed"""
        try:
            result = self.run_command("pkg-config --exists opencv4", check=False)
            return result.returncode == 0
        except Exception:
            return False
    
    def install(self) -> bool:
        """Install OpenCV"""
        try:
            self.logger.info("Installing OpenCV...")
            
            self.run_command("apt-get update")
            self.run_command("apt-get install -y libopencv-dev python3-opencv")
            
            self.logger.info("OpenCV installation completed")
            return True
            
        except Exception as e:
            self.logger.error(f"OpenCV installation failed: {e}")
            return False
    
    def verify(self) -> bool:
        """Verify OpenCV installation"""
        if not self.detect():
            return False
        
        try:
            # Test CMake find_package
            result = self.run_command(
                "cmake --find-package -DNAME=OpenCV -DCOMPILER_ID=GNU -DLANGUAGE=CXX -DMODE=EXIST",
                check=False
            )
            return result.returncode == 0
        except Exception:
            return False
    
    def uninstall(self) -> bool:
        """Uninstall OpenCV"""
        try:
            self.run_command("apt-get remove -y libopencv-dev python3-opencv")
            self.run_command("apt-get autoremove -y")
            return True
        except Exception as e:
            self.logger.error(f"OpenCV uninstall failed: {e}")
            return False