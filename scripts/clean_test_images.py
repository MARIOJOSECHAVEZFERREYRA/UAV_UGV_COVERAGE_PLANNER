#!/usr/bin/env python3
import os
import shutil
import sys

def clean_results():
    """Removes all files from data/test_results"""
    # Determine Path relative to this script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir) # Assuming scripts/ is at root level or src/scripts
    
    # Check if we are in src/scripts or scripts/
    # If project structure is project/scripts, then root is one level up.
    # If project structure is project/src/scripts, then root is two levels up.
    # Let's verify by looking for 'data' folder
    
    candidate_root = os.path.dirname(script_dir)
    if not os.path.exists(os.path.join(candidate_root, 'data')):
        # Try going up one more level
        candidate_root = os.path.dirname(candidate_root)
        
    results_dir = os.path.join(candidate_root, 'data', 'test_results')
    
    print(f"Cleaning PNG images in: {results_dir}")
    
    if not os.path.exists(results_dir):
        print("Directory does not exist. Nothing to clean.")
        return

    count = 0
    # Recursive walk to find only .png files
    for root, dirs, files in os.walk(results_dir):
        for filename in files:
            if filename.lower().endswith('.png'):
                file_path = os.path.join(root, filename)
                try:
                    os.unlink(file_path)
                    count += 1
                except Exception as e:
                    print(f"Failed to delete {file_path}. Reason: {e}")
            
    print(f"Done. Removed {count} PNG images.")

if __name__ == "__main__":
    clean_results()
