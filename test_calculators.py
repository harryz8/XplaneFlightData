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

    exit_code = 0
    if not test_calculator("density_altitude_calculator", arguments, expected_output):
        exit_code = 1
    if not test_calculator("density_altitude_calculator", arguments + ["1"], expected_return_code=3):
        exit_code = 1

    return exit_code == 0

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
            "direction_from": 195.53,
            "headwind": 4.05,
            "crosswind": 21.79,
            "gust_factor": 0.01
        },
        "envelope": {
            "stall_margin_pct": 82.98,
            "vmo_margin_pct": 12.00,
            "mmo_margin_pct": 20.73,
            "min_margin_pct": 12.00,
            "load_factor": 1.00,
            "corner_speed_kts": 170.00
        },
        "energy": {
            "specific_energy_ft": 37766.88,
            "energy_rate_kts": -4.94,
            "trend": -1
        },
        "glide": {
            "still_air_range_nm": 69.12,
            "wind_adjusted_range_nm": 68.00,
            "glide_ratio": 12.00,
            "best_glide_speed_kts": 78.00
        },
        "alternate_airports": {
            "combinations_5_choose_2": 10,
            "combinations_10_choose_3": 120,
            "note": "Iterative binomial calculation (JSF-compliant, no recursion)"
        }
    }

    return test_calculator("flight_calculator", arguments, expected_output)

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
    
    return test_calculator("turn_calculator", arguments, expected_output)

def test_vnav_calculator():
    arguments = ["35000", "10000", "100", "450", "-1500"]

    expected_output = {
        "altitude_to_lose_ft": 25000.00,
        "flight_path_angle_deg": -2.36,
        "required_vs_fpm": -1875.02,
        "tod_distance_nm": 78.51,
        "time_to_constraint_min": 16.67,
        "distance_per_1000ft": 4.00,
        "vs_for_3deg": 2388.30,
        "is_descent": True
    }
    
    return test_calculator("vnav_calculator", arguments, expected_output)

def test_wind_calculator():
    arguments = ["090", "085", "240", "60"]

    expected_output = {
        "headwind": 51.96,
        "crosswind": 30.00,
        "total_wind": 60.00,
        "wca": 0.00,
        "drift": 5.00
    }
    
    return test_calculator("wind_calculator", arguments, expected_output)

def test_calculator(filename, arguments, expected_output=None, expected_return_code=0):
    print(f"Testing {filename}")
    script_dir = Path(__file__).parent
    calculator_path = script_dir / filename

    if not calculator_path.exists():
        print(f"{filename} not found")
        return False

    result = subprocess.run(
        [str(calculator_path)] + arguments,
        capture_output=True,
        text=True,
        timeout=2.0
    )

    # ---- RETURN CODE CHECK ----
    if result.returncode != expected_return_code:
        print(
            f"❌ Return code mismatch: "
            f"expected {expected_return_code}, got {result.returncode}"
        )

        return False

    # ---- EXPECTED FAILURE CASE ----
    if expected_return_code != 0:
        # We expected an error and got it
        print(f"✅ Expected failure (return code {expected_return_code})")
        return True

    # ---- SUCCESS CASE (JSON VALIDATION) ----
    try:
        output_data = json.loads(result.stdout)
    except json.JSONDecodeError:
        print("❌ Output was not valid JSON")
        print(result.stdout)
        return False

    errors = compare_json(expected_output, output_data)

    if errors:
        print("❌ JSON mismatch:")
        for err in errors:
            print(f" - {err}")
        return False
    else:
        print("✅ Output matches expected data")
        return True

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

def run_test(test_fn):
    """Run a test function and return True if it passed, False otherwise."""
    result = test_fn()
    if not result:
        print(f"❌ {test_fn.__name__} FAILED\n")
    return result

def main():
    tests = [
        test_turn_calculator,
        test_vnav_calculator,
        test_density_altitude_calculator,
        test_wind_calculator,
        test_flight_calculator
    ]

    any_failures = False
    for test_fn in tests:
        if not run_test(test_fn):
            any_failures = True

    exit(1 if any_failures else 0)

if __name__ == "__main__":
    main()