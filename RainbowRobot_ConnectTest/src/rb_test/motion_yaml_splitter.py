#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Motion YAML Splitter for RB10-1300E
Splits large YAML motion files into chunks of 50 waypoints or less
"""

import yaml
import sys
import os
from typing import Dict, List, Any
from datetime import datetime
import argparse


class MotionYAMLSplitter:
    """Split large motion YAML files into manageable chunks."""
    
    MAX_WAYPOINTS = 50  # RB10-1300E limit
    
    def __init__(self, max_waypoints: int = 50):
        self.max_waypoints = max_waypoints
    
    def load_yaml(self, file_path: str) -> Dict[str, Any]:
        """Load YAML file."""
        with open(file_path, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)
    
    def save_yaml(self, data: Dict[str, Any], file_path: str):
        """Save YAML file."""
        with open(file_path, 'w', encoding='utf-8') as f:
            yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
    
    def split_motion(self, motion_data: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Split motion into chunks."""
        chunks = []
        
        for motion in motion_data.get('motions', []):
            waypoints = motion.get('waypoints', [])
            total_waypoints = len(waypoints)
            
            if total_waypoints <= self.max_waypoints:
                # No need to split
                chunks.append(motion_data)
                continue
            
            # Split into chunks
            num_chunks = (total_waypoints + self.max_waypoints - 1) // self.max_waypoints
            
            for chunk_idx in range(num_chunks):
                start_idx = chunk_idx * self.max_waypoints
                end_idx = min(start_idx + self.max_waypoints, total_waypoints)
                
                chunk_motion = {
                    'name': f"{motion.get('name', 'motion')}_chunk_{chunk_idx + 1}",
                    'motion_type': motion.get('motion_type', 'joint_path'),
                    'execution_mode': motion.get('execution_mode', 'j_add'),
                    'coordinate_frame': motion.get('coordinate_frame', 'robot_base_frame'),
                    'parameters': motion.get('parameters', {}),
                    'waypoints': waypoints[start_idx:end_idx]
                }
                
                # Renumber IDs for consistency
                for i, wp in enumerate(chunk_motion['waypoints']):
                    wp['id'] = i
                
                chunk_data = {
                    'motions': [chunk_motion],
                    'metadata': {
                        'original_file': motion_data.get('metadata', {}).get('original_file', 'unknown'),
                        'chunk': chunk_idx + 1,
                        'total_chunks': num_chunks,
                        'waypoint_range': f"{start_idx}-{end_idx-1}",
                        'created': datetime.now().isoformat()
                    }
                }
                chunks.append(chunk_data)
        
        return chunks
    
    def split_file(self, input_file: str, output_dir: str = None) -> List[str]:
        """Split a YAML file and save chunks."""
        if not os.path.exists(input_file):
            raise FileNotFoundError(f"Input file not found: {input_file}")
        
        # Default output directory
        if output_dir is None:
            output_dir = os.path.dirname(input_file)
        
        os.makedirs(output_dir, exist_ok=True)
        
        # Load original file
        data = self.load_yaml(input_file)
        
        # Add metadata
        if 'metadata' not in data:
            data['metadata'] = {}
        data['metadata']['original_file'] = os.path.basename(input_file)
        
        # Split into chunks
        chunks = self.split_motion(data)
        
        # Save chunks
        output_files = []
        base_name = os.path.splitext(os.path.basename(input_file))[0]
        
        if len(chunks) == 1:
            print(f"✅ File has {len(data['motions'][0]['waypoints'])} waypoints - no splitting needed")
            return [input_file]
        
        for i, chunk in enumerate(chunks):
            output_file = os.path.join(output_dir, f"{base_name}_chunk_{i+1:03d}.yaml")
            self.save_yaml(chunk, output_file)
            output_files.append(output_file)
            
            waypoint_count = len(chunk['motions'][0]['waypoints'])
            print(f"📁 Created: {output_file} ({waypoint_count} waypoints)")
        
        return output_files
    
    def create_sequence_file(self, chunk_files: List[str], output_file: str):
        """Create a sequence file that references all chunks."""
        sequence = {
            'sequence': {
                'name': 'chunked_motion_sequence',
                'chunks': chunk_files,
                'execution': 'sequential',
                'delay_between_chunks_ms': 100
            }
        }
        
        self.save_yaml(sequence, output_file)
        print(f"📋 Created sequence file: {output_file}")


def main():
    """Command-line interface for the splitter."""
    parser = argparse.ArgumentParser(
        description='Split large motion YAML files for RB10-1300E robot'
    )
    parser.add_argument(
        'input_file',
        help='Input YAML file to split'
    )
    parser.add_argument(
        '-o', '--output-dir',
        help='Output directory for chunks (default: same as input)',
        default=None
    )
    parser.add_argument(
        '-m', '--max-waypoints',
        type=int,
        default=50,
        help='Maximum waypoints per chunk (default: 50)'
    )
    parser.add_argument(
        '-s', '--create-sequence',
        action='store_true',
        help='Create a sequence file for all chunks'
    )
    
    args = parser.parse_args()
    
    # Create splitter
    splitter = MotionYAMLSplitter(max_waypoints=args.max_waypoints)
    
    try:
        # Split the file
        print(f"\n🔧 Splitting {args.input_file} with max {args.max_waypoints} waypoints per chunk...")
        chunk_files = splitter.split_file(args.input_file, args.output_dir)
        
        # Create sequence file if requested
        if args.create_sequence and len(chunk_files) > 1:
            output_dir = args.output_dir or os.path.dirname(args.input_file)
            base_name = os.path.splitext(os.path.basename(args.input_file))[0]
            sequence_file = os.path.join(output_dir, f"{base_name}_sequence.yaml")
            splitter.create_sequence_file(chunk_files, sequence_file)
        
        print(f"\n✅ Successfully created {len(chunk_files)} chunk(s)")
        
    except Exception as e:
        print(f"\n❌ Error: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()