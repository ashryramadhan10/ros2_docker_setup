"""
Configuration management for the dependency manager
"""

import yaml
from pathlib import Path
from typing import Dict, Any, Optional

class Config:
    """Configuration manager for installation settings"""
    
    def __init__(self, config_file: Optional[str] = None):
        self.config_file = config_file or "install_config.yaml"
        self.config = self._load_config()
    
    def _load_config(self) -> Dict[str, Any]:
        """Load configuration from file or return defaults"""
        config_path = Path(self.config_file)
        
        if config_path.exists():
            with open(config_path) as f:
                return yaml.safe_load(f) or {}
        
        # Default configuration
        return {
            'workspace_dir': '/workspace',
            'temp_dir': '/tmp',
            'parallel_install': False,
            'verify_after_install': True,
            'components': {}
        }
    
    def get_component_config(self, component: str) -> Dict[str, Any]:
        """Get configuration for specific component"""
        return self.config.get('components', {}).get(component, {})
    
    def get(self, key: str, default: Any = None) -> Any:
        """Get configuration value"""
        return self.config.get(key, default)
    
    def save(self):
        """Save current configuration to file"""
        with open(self.config_file, 'w') as f:
            yaml.dump(self.config, f, default_flow_style=False)