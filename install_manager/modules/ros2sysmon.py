"""
ROS2 System Monitor installer module
"""

import os
from pathlib import Path
import sys
sys.path.append(str(Path(__file__).parent.parent))

from core.installer import BaseInstaller

class ROS2SysmonInstaller(BaseInstaller):
    """ROS2 System Monitor installer"""
    
    def __init__(self, name: str, config: dict = None):
        super().__init__(name, config)
        self.workspace_dir = config.get('workspace_dir', '/workspace') if config else '/workspace'
        self.src_dir = Path(self.workspace_dir) / 'src'
        self.package_dir = self.src_dir / 'ros2sysmon'
    
    def detect(self) -> bool:
        """Check if ros2sysmon is installed"""
        return (self.package_dir.exists() and 
                Path(self.workspace_dir, 'install', 'ros2sysmon').exists())
    
    def install(self) -> bool:
        """Install ros2sysmon"""
        try:
            self.logger.info("Installing ros2sysmon...")
            
            # Install Python dependency
            self.run_command("python3 -m pip install textual")
            
            # Create workspace structure
            self.src_dir.mkdir(parents=True, exist_ok=True)
            
            # Clone repository if not exists
            if not self.package_dir.exists():
                os.chdir(self.src_dir)
                self.run_command("git clone https://github.com/pitosalas/ros2sysmon.git")
            
            # Build the package
            os.chdir(self.workspace_dir)
            self.run_command("source /opt/ros/humble/setup.bash && colcon build --packages-select ros2sysmon")
            
            self.logger.info("ros2sysmon installation completed")
            return True
            
        except Exception as e:
            self.logger.error(f"ros2sysmon installation failed: {e}")
            return False
    
    def verify(self) -> bool:
        """Verify ros2sysmon installation"""
        if not self.detect():
            return False
        
        try:
            # Check if package can be found by ROS2
            env = os.environ.copy()
            env['ROS_PACKAGE_PATH'] = f"{self.workspace_dir}/install/share"
            
            result = self.run_command(
                f"source {self.workspace_dir}/install/setup.bash && ros2 pkg list | grep ros2sysmon",
                check=False
            )
            return result.returncode == 0
        except Exception:
            return False
    
    def uninstall(self) -> bool:
        """Uninstall ros2sysmon"""
        try:
            if self.package_dir.exists():
                self.run_command(f"rm -rf {self.package_dir}")
            
            install_dir = Path(self.workspace_dir) / 'install' / 'ros2sysmon'
            if install_dir.exists():
                self.run_command(f"rm -rf {install_dir}")
            
            build_dir = Path(self.workspace_dir) / 'build' / 'ros2sysmon'
            if build_dir.exists():
                self.run_command(f"rm -rf {build_dir}")
            
            return True
        except Exception as e:
            self.logger.error(f"ros2sysmon uninstall failed: {e}")
            return False
    
    def get_dependencies(self) -> list:
        """ros2sysmon has no dependencies in our system"""
        return []