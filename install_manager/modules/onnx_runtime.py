"""
ONNX Runtime installer module
"""

import os
import tempfile
from pathlib import Path
import sys
sys.path.append(str(Path(__file__).parent.parent))

from core.installer import BaseInstaller

class ONNXRuntimeInstaller(BaseInstaller):
    """ONNX Runtime 1.22.1 installer"""
    
    def __init__(self, name: str, config: dict = None):
        super().__init__(name, config)
        self.version = config.get('version', '1.22.1') if config else '1.22.1'
        self.lib_path = '/usr/local/lib/libonnxruntime.so'
    
    def detect(self) -> bool:
        """Check if ONNX Runtime is installed"""
        return Path(self.lib_path).exists()
    
    def install(self) -> bool:
        """Install ONNX Runtime"""
        try:
            self.logger.info(f"Installing ONNX Runtime {self.version}...")
            
            # Install required tools
            self.run_command("apt-get update && apt-get install -y wget unzip")
            
            with tempfile.TemporaryDirectory() as temp_dir:
                os.chdir(temp_dir)
                
                # Download and extract
                url = f"https://www.nuget.org/api/v2/package/Microsoft.ML.OnnxRuntime/{self.version}"
                self.run_command(f"wget -O onnx_archive.nupkg {url}")
                self.run_command("unzip onnx_archive.nupkg")
                
                # Install libraries
                native_dir = Path(temp_dir) / "runtimes/linux-x64/native"
                os.chdir(native_dir)
                
                self.run_command(f"ln -s libonnxruntime.so libonnxruntime.so.{self.version}")
                self.run_command("cp libonnxruntime.so /usr/local/lib/")
                self.run_command(f"cp libonnxruntime.so.{self.version} /usr/local/lib/")
                
                # Install headers
                include_dir = Path("/usr/local/include/onnxruntime")
                include_dir.mkdir(parents=True, exist_ok=True)
                
                build_include = Path(temp_dir) / "build/native/include"
                if build_include.exists():
                    self.run_command(f"cp -r {build_include}/* {include_dir}/")
                
                # Install CMake config
                self._install_cmake_config()
                
                # Update library cache
                self.run_command("ldconfig")
            
            self.logger.info("ONNX Runtime installation completed")
            return True
            
        except Exception as e:
            self.logger.error(f"ONNX Runtime installation failed: {e}")
            return False
    
    def _install_cmake_config(self):
        """Install CMake configuration files"""
        cmake_dir = Path("/usr/local/share/cmake/onnxruntime")
        cmake_dir.mkdir(parents=True, exist_ok=True)
        
        # Version config
        version_config = f'''set(PACKAGE_VERSION "{self.version}")
if("${{PACKAGE_VERSION}}" VERSION_LESS "${{PACKAGE_FIND_VERSION}}")
  set(PACKAGE_VERSION_COMPATIBLE FALSE)
else()
  set(PACKAGE_VERSION_COMPATIBLE TRUE)
  if("${{PACKAGE_VERSION}}" VERSION_EQUAL "${{PACKAGE_FIND_VERSION}}")
    set(PACKAGE_VERSION_EXACT TRUE)
  endif()
endif()'''
        
        with open(cmake_dir / "onnxruntimeVersion.cmake", 'w') as f:
            f.write(version_config)
        
        # Main config
        main_config = '''include(FindPackageHandleStandardArgs)
get_filename_component(CMAKE_CURRENT_LIST_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
get_filename_component(onnxruntime_INSTALL_PREFIX "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)
set(onnxruntime_INCLUDE_DIRS ${onnxruntime_INSTALL_PREFIX}/include/onnxruntime/include)
set(onnxruntime_LIBRARIES onnxruntime)
set(onnxruntime_CXX_FLAGS "")
find_library(onnxruntime_LIBRARY onnxruntime PATHS "${onnxruntime_INSTALL_PREFIX}/lib")
add_library(onnxruntime SHARED IMPORTED)
set_property(TARGET onnxruntime PROPERTY IMPORTED_LOCATION "${onnxruntime_LIBRARY}")
set_property(TARGET onnxruntime PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${onnxruntime_INCLUDE_DIRS}")
set_property(TARGET onnxruntime PROPERTY INTERFACE_COMPILE_OPTIONS "${onnxruntime_CXX_FLAGS}")
find_package_handle_standard_args(onnxruntime DEFAULT_MSG onnxruntime_LIBRARY onnxruntime_INCLUDE_DIRS)'''
        
        with open(cmake_dir / "onnxruntimeConfig.cmake", 'w') as f:
            f.write(main_config)
    
    def verify(self) -> bool:
        """Verify ONNX Runtime installation"""
        if not self.detect():
            return False
        
        try:
            # Test CMake find_package
            result = self.run_command(
                "cmake --find-package -DNAME=onnxruntime -DCOMPILER_ID=GNU -DLANGUAGE=CXX -DMODE=EXIST",
                check=False
            )
            return result.returncode == 0
        except Exception:
            return False
    
    def uninstall(self) -> bool:
        """Uninstall ONNX Runtime"""
        try:
            files_to_remove = [
                "/usr/local/lib/libonnxruntime.so",
                f"/usr/local/lib/libonnxruntime.so.{self.version}",
                "/usr/local/include/onnxruntime",
                "/usr/local/share/cmake/onnxruntime"
            ]
            
            for file_path in files_to_remove:
                if Path(file_path).exists():
                    self.run_command(f"rm -rf {file_path}")
            
            self.run_command("ldconfig")
            return True
            
        except Exception as e:
            self.logger.error(f"ONNX Runtime uninstall failed: {e}")
            return False