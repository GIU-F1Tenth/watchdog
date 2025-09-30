#!/usr/bin/env python3

"""
Comprehensive test runner for the F1TENTH watchdog sanity checking system.

This script runs all test suites and provides a summary of results.
It can be used for continuous integration and development testing.

Author: F1TENTH Watchdog Team
License: MIT
"""

import unittest
import sys
import os
import time
import importlib.util
from io import StringIO

# Add the watchdog module to the path for testing
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


class ColoredTestResult(unittest.TextTestResult):
    """Test result class with colored output."""
    
    def __init__(self, stream, descriptions, verbosity):
        super().__init__(stream, descriptions, verbosity)
        self.success_count = 0
    
    def addSuccess(self, test):
        super().addSuccess(test)
        self.success_count += 1
        if self.verbosity > 1:
            self.stream.write(f"✓ {test._testMethodName}\n")
    
    def addError(self, test, err):
        super().addError(test, err)
        if self.verbosity > 1:
            self.stream.write(f"✗ {test._testMethodName} (ERROR)\n")
    
    def addFailure(self, test, err):
        super().addFailure(test, err)
        if self.verbosity > 1:
            self.stream.write(f"✗ {test._testMethodName} (FAILED)\n")
    
    def addSkip(self, test, reason):
        super().addSkip(test, reason)
        if self.verbosity > 1:
            self.stream.write(f"- {test._testMethodName} (SKIPPED: {reason})\n")


def run_test_suite(test_module_name, test_file_path):
    """Run a specific test suite and return results."""
    print(f"\n{'='*60}")
    print(f"Running {test_module_name}")
    print(f"{'='*60}")
    
    # Check if test file exists
    if not os.path.exists(test_file_path):
        print(f" Test file not found: {test_file_path}")
        return False, 0, 0, 0, 0
    
    # Import and run the test module
    try:
        # Load the test module
        spec = importlib.util.spec_from_file_location(test_module_name, test_file_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        
        # Create test suite
        loader = unittest.TestLoader()
        suite = loader.loadTestsFromModule(module)
        
        # Run tests with custom result class
        stream = StringIO()
        runner = unittest.TextTestRunner(
            stream=stream,
            verbosity=2,
            resultclass=ColoredTestResult
        )
        
        start_time = time.time()
        result = runner.run(suite)
        end_time = time.time()
        
        # Print results
        output = stream.getvalue()
        if output:
            print(output)
        
        # Summary
        total_tests = result.testsRun
        failures = len(result.failures)
        errors = len(result.errors)
        skipped = len(result.skipped)
        successes = total_tests - failures - errors - skipped
        
        print(f"\n Test Summary for {test_module_name}:")
        print(f"   Total: {total_tests}")
        print(f"   Passed: {successes}")
        print(f"   Failed: {failures}")
        print(f"   Errors: {errors}")
        print(f"   Skipped: {skipped}")
        print(f"   Time: {end_time - start_time:.2f}s")
        
        success = (failures == 0 and errors == 0)
        if success:
            print(f"    {test_module_name} PASSED")
        else:
            print(f"    {test_module_name} FAILED")
        
        return success, total_tests, successes, failures, errors
        
    except Exception as e:
        print(f" Error running {test_module_name}: {e}")
        import traceback
        traceback.print_exc()
        return False, 0, 0, 1, 0


def print_detailed_failures(test_file_path):
    """Print detailed failure information."""
    try:
        # Re-run tests to get detailed failure info
        spec = importlib.util.spec_from_file_location("temp_module", test_file_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        
        loader = unittest.TestLoader()
        suite = loader.loadTestsFromModule(module)
        
        # Run with detailed output
        runner = unittest.TextTestRunner(verbosity=2)
        result = runner.run(suite)
        
        # Print failure details
        if result.failures:
            print("\n Detailed Failure Information:")
            for test, traceback in result.failures:
                print(f"\n FAILURE: {test}")
                print("-" * 40)
                print(traceback)
        
        if result.errors:
            print("\n Detailed Error Information:")
            for test, traceback in result.errors:
                print(f"\n ERROR: {test}")
                print("-" * 40)
                print(traceback)
    except Exception as e:
        print(f"Error getting detailed failure info: {e}")


def main():
    """Main test runner function."""
    print(" Starting F1TENTH Watchdog Sanity Checking Test Suite")
    print("=" * 80)
    
    # Test configuration
    test_dir = os.path.dirname(__file__)
    test_suites = [
        ("Validator Unit Tests", os.path.join(test_dir, "test_validators.py")),
        ("SanityChecker Integration Tests", os.path.join(test_dir, "test_sanity_checker.py")),
        ("Watchdog Node Integration Tests", os.path.join(test_dir, "test_watchdog_integration.py")),
    ]
    
    # Run all test suites
    overall_success = True
    total_tests = 0
    total_successes = 0
    total_failures = 0
    total_errors = 0
    
    failed_suites = []
    
    for suite_name, test_file in test_suites:
        success, tests, successes, failures, errors = run_test_suite(suite_name, test_file)
        
        if not success:
            overall_success = False
            failed_suites.append((suite_name, test_file))
        
        total_tests += tests
        total_successes += successes
        total_failures += failures
        total_errors += errors
    
    # Print overall summary
    print(f"\n{'='*80}")
    print(" OVERALL TEST SUMMARY")
    print(f"{'='*80}")
    print(f"Total Test Suites: {len(test_suites)}")
    print(f"Total Tests: {total_tests}")
    print(f" Passed: {total_successes}")
    print(f" Failed: {total_failures}")
    print(f" Errors: {total_errors}")
    
    if overall_success:
        print(f"\n ALL TESTS PASSED! ")
        print("The sanity checking system is working correctly.")
    else:
        print(f"\n SOME TESTS FAILED! ")
        print(f"Failed test suites: {len(failed_suites)}")
        
        # Print detailed failure information for failed suites
        for suite_name, test_file in failed_suites:
            print(f"\n Getting detailed info for {suite_name}...")
            print_detailed_failures(test_file)
    
    # Exit with appropriate code
    sys.exit(0 if overall_success else 1)


if __name__ == '__main__':
    main()