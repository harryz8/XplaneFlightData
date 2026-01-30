from pathlib import Path
import subprocess
import sys
import json


def test_density_altitude():

    arguments = ["5000", "25", "150", "170"]

    expected_output = {
    "density_altitude_ft": 7388.72,
    "pressure_altitude_ft": 5000.00,
    "air_density_ratio": 0.80,
    "temperature_deviation_c": 19.91,
    "performance_loss_pct": 19.59,
    "eas_kts": 152.45,
    "tas_to_ias_ratio": 1.13,
    "pressure_ratio": 0.83
    }

    test_calculator("density_altitude_calculator", arguments, expected_output)

def test_calculator(filename, arguments, expected_output):
    print("Testing " + filename)
    script_dir = Path(__file__).parent
    calculator_path = script_dir / ("../" + filename)

    if not calculator_path.exists():
        print(filename + " not found")
        exit(1)

    result = subprocess.run(
                    [str(calculator_path)] + arguments,
                    capture_output=True,
                    text=True,
                    timeout=0.1
                )

    if result.returncode != 0:
        error_lines = result.stderr.strip().split('\n')
        # Get the actual error message (first line after "Error:")
        error_msg = "Unknown C++ error"
        for line in error_lines:
            if line.startswith("Error:"):
                error_msg = line.replace("Error:", "").strip()
                break
        print(error_msg)
        exit(1)

    output_data = json.loads(result.stdout)
    errors = compare_json(expected_output, output_data)

    if errors:
        print("❌ JSON mismatch:")
        for err in errors:
            print(f" - {err}")
        exit(1)
    else:
        print("✅ Output matches expected data")

def compare_json(expected, actual, tol=1e-2):
    errors = []

    # Missing or extra keys
    for key in expected:
        if key not in actual:
            errors.append(f"Missing key: {key}")
    for key in actual:
        if key not in expected:
            errors.append(f"Unexpected key: {key}")

    # Value comparison
    for key in expected:
        if key not in actual:
            continue

        exp_val = expected[key]
        act_val = actual[key]

        if isinstance(exp_val, (int, float)) and isinstance(act_val, (int, float)):
            diff = abs(exp_val - act_val)
            if diff > tol:
                errors.append(
                    f"{key}: expected {exp_val}, got {act_val} (diff {diff:.4f})"
                )
        else:
            if exp_val != act_val:
                errors.append(
                    f"{key}: expected {exp_val}, got {act_val}"
                )

    return errors

def main():
    test_density_altitude()

if __name__ == "__main__":
    main()