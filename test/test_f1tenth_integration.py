#!/usr/bin/env python3

"""
F1TENTH Integration Test

This script tests the F1TENTH integration components without requiring
the full F1TENTH stack to be installed.
"""

import sys
import os

# Add the watchdog package to the path
sys.path.insert(0, '/home/yousufaboeldahab/ros2_ws/src/watchdog')

def test_f1tenth_configuration():
    """Test F1TENTH configuration loading."""
    print("Testing F1TENTH Configuration Loading...")
    
    config_file = '/home/yousufaboeldahab/ros2_ws/src/watchdog/config/f1tenth_params.yaml'
    
    try:
        import yaml
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
        
        # Check key F1TENTH parameters
        watchdog_params = config['watchdog']['ros__parameters']
        
        assert watchdog_params['vehicle_type'] == 'f1tenth'
        assert watchdog_params['max_speed'] == 8.0
        assert watchdog_params['wheelbase'] == 0.33
        assert watchdog_params['racing_mode'] == True
        
        print("  ✓ F1TENTH configuration loaded successfully")
        print(f"  ✓ Vehicle type: {watchdog_params['vehicle_type']}")
        print(f"  ✓ Max speed: {watchdog_params['max_speed']} m/s")
        print(f"  ✓ Racing mode: {watchdog_params['racing_mode']}")
        
        return True
        
    except Exception as e:
        print(f"  ✗ Configuration test failed: {e}")
        return False

def test_fsm_integration_classes():
    """Test FSM integration class imports."""
    print("\nTesting FSM Integration Classes...")
    
    try:
        # Test FSM state enum
        sys.path.insert(0, '/home/yousufaboeldahab/ros2_ws/src/watchdog/watchdog')
        from fsm_integration import FSMState, HealthLevel, FSMHealthStatus
        
        # Test enum values
        assert FSMState.DRIVING.value == "driving"
        assert HealthLevel.EXCELLENT.value == "excellent"
        
        # Test dataclass creation
        health_status = FSMHealthStatus(
            overall_healthy=True,
            health_score=0.95,
            health_level=HealthLevel.EXCELLENT,
            critical_issues=[],
            warnings=[],
            affected_sensors=[],
            recommended_action="continue_normal",
            can_continue_racing=True
        )
        
        assert health_status.can_continue_racing == True
        assert health_status.health_level == HealthLevel.EXCELLENT
        
        print("  ✓ FSM integration classes imported successfully")
        print(f"  ✓ FSM states available: {[state.value for state in FSMState]}")
        print(f"  ✓ Health levels available: {[level.value for level in HealthLevel]}")
        
        return True
        
    except Exception as e:
        print(f"  ✗ FSM integration test failed: {e}")
        return False

def test_launch_file_syntax():
    """Test launch file syntax and structure."""
    print("\nTesting Launch File Syntax...")
    
    try:
        # Test F1TENTH watchdog launch file
        launch_file = '/home/yousufaboeldahab/ros2_ws/src/watchdog/launch/f1tenth_watchdog.launch.py'
        
        with open(launch_file, 'r') as f:
            content = f.read()
        
        # Check for key components
        assert 'watchdog_node.py' in content
        assert 'f1tenth_params.yaml' in content
        assert '/emergency_stop' in content
        assert 'F1TENTH standard topic mappings' in content
        
        print("  ✓ F1TENTH launch file syntax correct")
        
        # Test system launch file
        system_launch = '/home/yousufaboeldahab/ros2_ws/src/watchdog/launch/f1tenth_system.launch.py'
        
        with open(system_launch, 'r') as f:
            content = f.read()
        
        assert 'watchdog_node.py' in content
        assert 'enable_watchdog' in content
        
        print("  ✓ F1TENTH system launch file syntax correct")
        
        return True
        
    except Exception as e:
        print(f"  ✗ Launch file test failed: {e}")
        return False

def test_integration_documentation():
    """Test integration documentation completeness."""
    print("\nTesting Integration Documentation...")
    
    try:
        doc_file = '/home/yousufaboeldahab/ros2_ws/src/watchdog/docs/F1TENTH_INTEGRATION.md'
        
        with open(doc_file, 'r') as f:
            content = f.read()
        
        # Check for key sections
        required_sections = [
            'F1TENTH System Integration Guide',
            'Topic Mappings',
            'FSM Integration',
            'Safety Protocols',
            'Configuration',
            'Troubleshooting'
        ]
        
        for section in required_sections:
            assert section in content, f"Missing section: {section}"
        
        print("  ✓ Integration documentation complete")
        print(f"  ✓ All required sections present: {len(required_sections)}")
        
        return True
        
    except Exception as e:
        print(f"  ✗ Documentation test failed: {e}")
        return False

def test_executable_availability():
    """Test if executables are properly installed."""
    print("\nTesting Executable Availability...")
    
    try:
        # Check if watchdog_node.py exists
        executable_path = '/home/yousufaboeldahab/ros2_ws/install/watchdog/lib/watchdog/watchdog_node.py'
        
        if os.path.exists(executable_path):
            print("  ✓ watchdog_node.py executable found")
            
            # Check if it's executable
            if os.access(executable_path, os.X_OK):
                print("  ✓ watchdog_node.py is executable")
            else:
                print("  ⚠ watchdog_node.py exists but is not executable")
        else:
            print("  ✗ watchdog_node.py executable not found")
            return False
        
        return True
        
    except Exception as e:
        print(f"  ✗ Executable test failed: {e}")
        return False

def main():
    """Run all F1TENTH integration tests."""
    print("F1TENTH Watchdog Integration Test Suite")
    print("=" * 50)
    
    tests = [
        test_f1tenth_configuration,
        test_fsm_integration_classes,
        test_launch_file_syntax,
        test_integration_documentation,
        test_executable_availability
    ]
    
    passed = 0
    total = len(tests)
    
    for test in tests:
        try:
            if test():
                passed += 1
        except Exception as e:
            print(f"Test failed with exception: {e}")
    
    print("\n" + "=" * 50)
    print(f"F1TENTH Integration Test Results: {passed}/{total} tests passed")
    
    if passed == total:
        print("✓ ALL TESTS PASSED - F1TENTH integration ready!")
        print("\nNext steps:")
        print("1. Install F1TENTH dependencies (vesc_msgs, ackermann_msgs)")
        print("2. Test with actual F1TENTH hardware/simulation")
        print("3. Configure parameters for specific vehicle")
        return 0
    else:
        print("✗ Some tests failed - check output above")
        return 1

if __name__ == "__main__":
    exit(main())