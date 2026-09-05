#!/usr/bin/env python3
import sys
import yaml

def validate(file_path):
    try:
        with open(file_path, 'r') as f:
            data = yaml.safe_load(f)
        if not isinstance(data, dict):
            print(f"[FAIL] {file_path}: root must be a YAML dictionary")
            return False
        print(f"[PASS] {file_path} is valid YAML")
        return True
    except Exception as e:
        print(f"[ERROR] Failed to parse {file_path}: {e}")
        return False

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: validate_config.py <file.yaml>...")
        sys.exit(1)
    all_ok = True
    for path in sys.argv[1:]:
        if not validate(path):
            all_ok = False
    sys.exit(0 if all_ok else 1)
