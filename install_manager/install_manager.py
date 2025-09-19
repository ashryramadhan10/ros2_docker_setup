#!/usr/bin/env python3
"""
ROS2 Dependency Manager
A modular installation system for ROS2 development dependencies
"""

import argparse
import sys
import os
from pathlib import Path
from typing import List, Dict, Optional
import yaml
import logging

# Add current directory to path for imports
sys.path.insert(0, str(Path(__file__).parent))

from core.installer import InstallationManager
from core.config import Config

def setup_logging(verbose: bool = False):
    """Setup logging configuration"""
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format='%(asctime)s - %(levelname)s - %(message)s',
        handlers=[
            logging.StreamHandler(),
            logging.FileHandler('/tmp/install_manager.log')
        ]
    )

def interactive_mode(manager: InstallationManager) -> List[str]:
    """Interactive component selection"""
    available = manager.get_available_components()
    installed = manager.get_installed_components()
    
    print("\n=== ROS2 Dependency Manager ===")
    print("Available components:")
    
    for i, component in enumerate(available, 1):
        status = "✓ installed" if component in installed else "○ not installed"
        print(f"  {i}. {component} ({status})")
    
    print("\nEnter component numbers to install (e.g., 1,3,5) or 'all' for everything:")
    selection = input("> ").strip()
    
    if selection.lower() == 'all':
        return [c for c in available if c not in installed]
    
    try:
        indices = [int(x.strip()) - 1 for x in selection.split(',')]
        return [available[i] for i in indices if 0 <= i < len(available)]
    except (ValueError, IndexError):
        print("Invalid selection")
        return []

def main():
    parser = argparse.ArgumentParser(description='ROS2 Dependency Manager')
    parser.add_argument('--install', nargs='+', help='Install specific components')
    parser.add_argument('--uninstall', nargs='+', help='Uninstall components')
    parser.add_argument('--profile', help='Install from profile')
    parser.add_argument('--interactive', action='store_true', help='Interactive mode')
    parser.add_argument('--status', action='store_true', help='Show installation status')
    parser.add_argument('--dry-run', action='store_true', help='Preview without installing')
    parser.add_argument('--update', action='store_true', help='Update existing installations')
    parser.add_argument('--verbose', '-v', action='store_true', help='Verbose output')
    parser.add_argument('--list', action='store_true', help='List available components')
    
    args = parser.parse_args()
    
    setup_logging(args.verbose)
    
    try:
        config = Config()
        manager = InstallationManager(config, dry_run=args.dry_run)
        
        if args.list:
            components = manager.get_available_components()
            print("Available components:")
            for comp in components:
                print(f"  - {comp}")
            return 0
        
        if args.status:
            manager.show_status()
            return 0
        
        components_to_install = []
        
        if args.interactive:
            components_to_install = interactive_mode(manager)
        elif args.install:
            components_to_install = args.install
        elif args.profile:
            components_to_install = manager.load_profile(args.profile)
        elif args.update:
            components_to_install = manager.get_installed_components()
        elif args.uninstall:
            return manager.uninstall_components(args.uninstall)
        else:
            parser.print_help()
            return 1
        
        if components_to_install:
            return manager.install_components(components_to_install)
        
        return 0
        
    except KeyboardInterrupt:
        print("\nInstallation cancelled by user")
        return 1
    except Exception as e:
        logging.error(f"Installation failed: {e}")
        return 1

if __name__ == '__main__':
    sys.exit(main())