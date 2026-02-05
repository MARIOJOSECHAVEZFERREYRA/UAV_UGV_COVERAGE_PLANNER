import unittest
import sys
import os
import time
from datetime import datetime

def run_all_tests():
    #Setup
    start_time = time.time()
    loader = unittest.TestLoader()
    
    #Determine Project Paths
    current_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(current_dir)
    results_dir = os.path.join(project_root, 'data', 'test_results')
    log_file_path = os.path.join(results_dir, 'test_run.log')
    
    # Create results directory if needed
    if not os.path.exists(results_dir):
        os.makedirs(results_dir)
    
    # Create subdirectories for organized PNG storage
    for subdir in ['basic', 'stress_tests', 'organic', 'unit']:
        subdir_path = os.path.join(results_dir, subdir)
        if not os.path.exists(subdir_path):
            os.makedirs(subdir_path)

    
    #Clean Previous Results (keep subdirs, remove files)
    import shutil
    for root, dirs, files in os.walk(results_dir):
        for file in files:
            # Only delete PNG files, keep logs and other data
            if file.lower().endswith('.png'):
                file_path = os.path.join(root, file)
                try:
                    os.unlink(file_path)
                except Exception:
                    pass
    
    # Open log file and redirect all output to it
    log_file = open(log_file_path, 'w')
    original_stdout = sys.stdout
    original_stderr = sys.stderr
    sys.stdout = log_file
    sys.stderr = log_file
    
    # Write header to log
    print(f"Test Run - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"Test Directory: {current_dir}")
    print(f"Results Directory: {results_dir}")
    print("-" * 50)
    
    # Discover Tests (including subdirectories)
    suite = loader.discover(current_dir, pattern='test_*.py', top_level_dir=current_dir)
    
    if suite.countTestCases() == 0:
        print("No tests found!")
        log_file.close()
        sys.stdout = original_stdout
        sys.stderr = original_stderr
        print("No tests found!")
        return

    # Run tests (output goes to log)
    runner = unittest.TextTestRunner(verbosity=2, stream=sys.stdout)
    result = runner.run(suite)
    
    # Close log and restore stdout
    log_file.close()
    sys.stdout = original_stdout
    sys.stderr = original_stderr
    
    # Report to console
    end_time = time.time()
    duration = end_time - start_time
    
    print("-" * 50)
    print(f"Test Summary")
    print(f"   Total Tests:         {result.testsRun}")
    print(f"   Errors:              {len(result.errors)}")
    print(f"   Failures:            {len(result.failures)}")
    print(f"   Duration:            {duration:.2f} seconds")
    
    if result.wasSuccessful():
        print("\nALL TESTS PASSED! System is stable.")
        exit_code = 0
    else:
        print("\nSOME TESTS FAILED. review log in /home/mario/programacion/tesis/agriswarm_planner/data/test_results/test_run.log.")
        exit_code = 1
    
    sys.exit(exit_code)

if __name__ == "__main__":
    run_all_tests()
