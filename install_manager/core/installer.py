"""
Base installer classes and installation management
"""

import os
import subprocess
import logging
from abc import ABC, abstractmethod
from typing import List, Dict, Optional, Set
from pathlib import Path
import importlib
import importlib.util
import yaml

class BaseInstaller(ABC):
    """Base class for all component installers"""
    
    def __init__(self, name: str, config: dict = None):
        self.name = name
        self.config = config or {}
        self.logger = logging.getLogger(f"installer.{name}")
    
    @abstractmethod
    def detect(self) -> bool:
        """Check if component is already installed"""
        pass
    
    @abstractmethod
    def install(self) -> bool:
        """Install the component"""
        pass
    
    def verify(self) -> bool:
        """Verify installation (default: same as detect)"""
        return self.detect()
    
    def uninstall(self) -> bool:
        """Uninstall the component (optional)"""
        self.logger.warning(f"Uninstall not implemented for {self.name}")
        return False
    
    def get_dependencies(self) -> List[str]:
        """Return list of required dependencies"""
        return []
    
    def run_command(self, cmd: str, check: bool = True) -> subprocess.CompletedProcess:
        """Run shell command with logging"""
        self.logger.debug(f"Running: {cmd}")
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        
        if result.stdout:
            self.logger.debug(f"stdout: {result.stdout}")
        if result.stderr:
            self.logger.debug(f"stderr: {result.stderr}")
        
        if check and result.returncode != 0:
            raise subprocess.CalledProcessError(result.returncode, cmd, result.stdout, result.stderr)
        
        return result

class InstallationManager:
    """Manages component installation and dependencies"""
    
    def __init__(self, config, dry_run: bool = False):
        self.config = config
        self.dry_run = dry_run
        self.logger = logging.getLogger("manager")
        self.installers: Dict[str, BaseInstaller] = {}
        self._load_installers()
    
    def _load_installers(self):
        """Load all available installer modules"""
        modules_dir = Path(__file__).parent.parent / "modules"
        
        for module_file in modules_dir.glob("*.py"):
            if module_file.name.startswith("__"):
                continue
            
            module_name = module_file.stem
            try:
                spec = importlib.util.spec_from_file_location(module_name, module_file)
                module = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(module)
                
                # Look for installer class
                for attr_name in dir(module):
                    attr = getattr(module, attr_name)
                    if (isinstance(attr, type) and 
                        issubclass(attr, BaseInstaller) and 
                        attr != BaseInstaller):
                        installer = attr(module_name, self.config.get_component_config(module_name))
                        self.installers[module_name] = installer
                        self.logger.debug(f"Loaded installer: {module_name}")
                        break
                        
            except Exception as e:
                self.logger.error(f"Failed to load installer {module_name}: {e}")
    
    def get_available_components(self) -> List[str]:
        """Get list of available components"""
        return list(self.installers.keys())
    
    def get_installed_components(self) -> List[str]:
        """Get list of currently installed components"""
        installed = []
        for name, installer in self.installers.items():
            try:
                if installer.detect():
                    installed.append(name)
            except Exception as e:
                self.logger.error(f"Error checking {name}: {e}")
        return installed
    
    def resolve_dependencies(self, components: List[str]) -> List[str]:
        """Resolve component dependencies and return installation order"""
        resolved = []
        visited = set()
        visiting = set()
        
        def visit(component: str):
            if component in visiting:
                raise ValueError(f"Circular dependency detected: {component}")
            if component in visited:
                return
            
            if component not in self.installers:
                raise ValueError(f"Unknown component: {component}")
            
            visiting.add(component)
            
            for dep in self.installers[component].get_dependencies():
                visit(dep)
            
            visiting.remove(component)
            visited.add(component)
            resolved.append(component)
        
        for component in components:
            visit(component)
        
        return resolved
    
    def install_components(self, components: List[str]) -> int:
        """Install specified components with dependency resolution"""
        try:
            ordered_components = self.resolve_dependencies(components)
            self.logger.info(f"Installation order: {ordered_components}")
            
            if self.dry_run:
                print("DRY RUN - Would install:")
                for comp in ordered_components:
                    status = "✓" if self.installers[comp].detect() else "○"
                    print(f"  {status} {comp}")
                return 0
            
            failed = []
            for component in ordered_components:
                installer = self.installers[component]
                
                if installer.detect():
                    self.logger.info(f"{component} already installed")
                    continue
                
                self.logger.info(f"Installing {component}...")
                try:
                    if installer.install():
                        if installer.verify():
                            self.logger.info(f"✓ {component} installed successfully")
                        else:
                            self.logger.error(f"✗ {component} installation verification failed")
                            failed.append(component)
                    else:
                        self.logger.error(f"✗ {component} installation failed")
                        failed.append(component)
                except Exception as e:
                    self.logger.error(f"✗ {component} installation error: {e}")
                    failed.append(component)
            
            if failed:
                self.logger.error(f"Failed to install: {failed}")
                return 1
            
            self.logger.info("All components installed successfully!")
            return 0
            
        except Exception as e:
            self.logger.error(f"Installation failed: {e}")
            return 1
    
    def uninstall_components(self, components: List[str]) -> int:
        """Uninstall specified components"""
        if self.dry_run:
            print("DRY RUN - Would uninstall:")
            for comp in components:
                print(f"  - {comp}")
            return 0
        
        failed = []
        for component in components:
            if component not in self.installers:
                self.logger.error(f"Unknown component: {component}")
                failed.append(component)
                continue
            
            installer = self.installers[component]
            if not installer.detect():
                self.logger.info(f"{component} not installed")
                continue
            
            self.logger.info(f"Uninstalling {component}...")
            try:
                if installer.uninstall():
                    self.logger.info(f"✓ {component} uninstalled")
                else:
                    self.logger.error(f"✗ {component} uninstall failed")
                    failed.append(component)
            except Exception as e:
                self.logger.error(f"✗ {component} uninstall error: {e}")
                failed.append(component)
        
        return 1 if failed else 0
    
    def show_status(self):
        """Show installation status of all components"""
        installed = self.get_installed_components()
        available = self.get_available_components()
        
        print("\n=== Installation Status ===")
        for component in available:
            status = "✓ installed" if component in installed else "○ not installed"
            print(f"  {component}: {status}")
        print()
    
    def load_profile(self, profile_name: str) -> List[str]:
        """Load components from profile file"""
        profile_path = Path(__file__).parent.parent / "profiles" / f"{profile_name}.yaml"
        
        if not profile_path.exists():
            raise FileNotFoundError(f"Profile not found: {profile_name}")
        
        with open(profile_path) as f:
            profile = yaml.safe_load(f)
        
        return profile.get('components', [])