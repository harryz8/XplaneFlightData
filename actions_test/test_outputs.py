from pathlib import Path
import subprocess
import sys
import json


def test_density_altitude_calculator():

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

def test_flight_calculator():
    arguments = [
        "250",
        "245",
        "90",
        "95",
        "220",
        "0.65",
        "35000",
        "35000",
        "-500",
        "75000",
        "5",
        "120",
        "250",
        "0.82"
    ]

    expected_output = {
        "wind": {
        "speed_kts": 22.16,
        "direction_from": 15.53,
        "headwind": 4.05,
        "crosswind": 21.79,
        "gust_factor": 5.45
        },
        "envelope": {
        "stall_margin_pct": 82.98,
        "vmo_margin_pct": 12.00,
        "mmo_margin_pct": 20.73,
        "min_margin_pct": 12.00,
        "corner_speed_kts": 189.74,
        "load_factor": 1.00
        },
        "energy": {
        "specific_energy_ft": 37766.88,
        "energy_rate_fpm": -500.00,
        "trend": -1
        },
        "glide": {
        "max_range_nm": 69.12,
        "range_with_wind_nm": 65.39,
        "glide_ratio": 12.00,
        "best_glide_speed_kts": 75.00
        },
        "alternate_airports": {
        "combinations_5_choose_2": 10,
        "combinations_10_choose_3": 120,
        "note": "Recursive binomial calculation for flight planning"
        }
    }

    test_calculator("flight_calculator", arguments, expected_output)

def test_turn_calculator():
    arguments = ["250", "25", "90"]

    expected_output = {
        "radius_nm": 1.95,
        "radius_ft": 11867.19,
        "turn_rate_dps": 2.04,
        "lead_distance_nm": 1.95,
        "lead_distance_ft": 11867.19,
        "time_to_turn_sec": 44.18,
        "load_factor": 1.10,
        "standard_rate_bank": 34.48
        }
    
    test_calculator("turn_calculator", arguments, expected_output)

def test_vnav_calculator():
    arguments = ["35000", "10000", "100", "450", "-1500"]

    expected_output = {
        "altitude_to_lose_ft": 25000.00,
        "flight_path_angle_deg": -2.36,
        "required_vs_fpm": -1875.02,
        "tod_distance_nm": 78.51,
        "time_to_constraint_min": 13.33,
        "distance_per_1000ft": 4.00,
        "is_descent": True,
        "on_idle_path": True,
        "vs_for_3deg": -2388.30,
        "vs_for_5deg": -3986.99,
        "distance_at_current_vs_nm": 125.00
    }
    
    test_calculator("vnav_calculator", arguments, expected_output)

def test_wind_calculator():
    arguments = ["090", "085", "240", "60"]

    expected_output = {
    "headwind": 51.96,
    "crosswind": 30.00,
    "total_wind": 60.00,
    "wca": 0.00,
    "drift": 5.00
    }
    
    test_calculator("wind_calculator", arguments, expected_output)

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
                    timeout=2.0
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
        print("Json:")
        print(output_data)
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
    test_turn_calculator()
    test_density_altitude_calculator()
    test_wind_calculator()
    test_flight_calculator()
    test_vnav_calculator()

if __name__ == "__main__":
    main()