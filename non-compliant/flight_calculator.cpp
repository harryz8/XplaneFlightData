/**
 * Flight Performance Calculator for X-Plane MFD
 * 
 * Performs advanced flight calculations:
 * 1. Real-time wind vector with gust/turbulence analysis
 * 2. Envelope margins (stall/overspeed/buffet)
 * 3. Energy management (specific energy & trend)
 * 4. Glide reach estimation
 * 
 * Compile: g++ -std=c++20 -O3 -o flight_calculator flight_calculator.cpp
 */

#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <numbers>
#include <string_view>
#include <memory>

namespace xplane_mfd::calc {

constexpr double DEG_TO_RAD = std::numbers::pi / 180.0;
constexpr double RAD_TO_DEG = 180.0 / std::numbers::pi;
constexpr double GRAVITY = 9.80665;  // m/s²
constexpr double KTS_TO_MS = 0.514444;  // knots to m/s
constexpr double FT_TO_M = 0.3048;      // feet to meters
constexpr double M_TO_FT = 3.28084;     // meters to feet

[[nodiscard]] double parse_double(std::string_view sv) {
    return std::stod(std::string(sv));
}

struct Vector2D {
    double x, y;
    
    Vector2D(double x_ = 0, double y_ = 0) : x(x_), y(y_) {}
    
    double magnitude() const {
        return std::sqrt(x * x + y * y);
    }
    
    Vector2D operator-(const Vector2D& other) const {
        return Vector2D(x - other.x, y - other.y);
    }
};

double normalize_angle(double angle) {
    while (angle < 0) angle += 360.0;
    while (angle >= 360.0) angle -= 360.0;
    return angle;
}

// ========================================================================
// REMOVE BEFORE FLIGHT - Recursion
// ========================================================================
/**
 * Recursive binomial coefficient calculation (n choose k)
 * Used for calculating combinations of alternate airports in flight planning
 * 
 * Formula: C(n,k) = "n choose k" = number of ways to select k items from n items
 * Recursive relation: C(n,k) = C(n-1,k-1) + C(n-1,k)
 * 
 * @param n Total number of items
 * @param k Number of items to choose
 * @return Number of combinations
 * 
 * Example: binomial_coefficient(5, 2) = 10
 *          (5 nearby airports, choose 2 as alternates = 10 possible combinations)
 */
[[nodiscard]] unsigned long long binomial_coefficient(unsigned int n, unsigned int k) {
    // Base cases
    if (k > n) return 0;           // Can't choose more than available
    if (k == 0 || k == n) return 1; // C(n,0) = C(n,n) = 1
    if (k == 1) return n;           // C(n,1) = n
    
    // Recursive relation: C(n,k) = C(n-1,k-1) + C(n-1,k)
    // This represents: either include current item or don't
    return binomial_coefficient(n - 1, k - 1) + binomial_coefficient(n - 1, k);
}

/**
 * 1. Real-time wind vector calculation
 * Builds wind from ground track and airspeed vectors
 */
struct WindData {
    double speed_kts;
    double direction_from;  // deg, where wind comes FROM
    double headwind;
    double crosswind;
    double gust_factor;
};

[[nodiscard]] WindData calculate_wind_vector(
    double tas_kts,
    double gs_kts,
    double heading_deg,
    double track_deg,
    const std::vector<double>& ias_history // Past airspeeds for gust calc
) {
    WindData result;
    
    // Convert to radians
    double psi_rad = heading_deg * DEG_TO_RAD;
    double chi_rad = track_deg * DEG_TO_RAD;
    
    // Air vector (where plane points, at TAS)
    Vector2D air_vec(
        tas_kts * std::sin(psi_rad),  // East component
        tas_kts * std::cos(psi_rad)   // North component
    );
    
    // Ground vector (where plane actually goes, at GS)
    Vector2D ground_vec(
        gs_kts * std::sin(chi_rad),   // East component
        gs_kts * std::cos(chi_rad)    // North component
    );
    
    // Wind = Ground - Air
    Vector2D wind_vec = ground_vec - air_vec;
    
    result.speed_kts = wind_vec.magnitude();
    
    // Wind direction (where it comes FROM)
    double wind_to_rad = std::atan2(wind_vec.x, wind_vec.y);
    result.direction_from = normalize_angle(wind_to_rad * RAD_TO_DEG + 180.0);
    
    // Headwind/crosswind relative to track
    double rel_wind_rad = (result.direction_from + 180.0 - track_deg) * DEG_TO_RAD;
    result.headwind = -result.speed_kts * std::cos(rel_wind_rad);
    result.crosswind = result.speed_kts * std::sin(rel_wind_rad);
    
    // ========================================================================
    // REMOVE BEFORE FLIGHT - Memory allocation
    // ========================================================================
    if (!ias_history.empty()) {
        auto history_buffer = std::make_unique<double[]>(ias_history.size());

        // Copy data into our dynamic buffer for analysis
        for (size_t i = 0; i < ias_history.size(); ++i) {
            history_buffer[i] = ias_history[i];
        }

        double max_ias = 0;
        double sum_ias = 0;
        for (size_t i = 0; i < ias_history.size(); ++i) {
            if (history_buffer[i] > max_ias) max_ias = history_buffer[i];
            sum_ias += history_buffer[i];
        }
        double avg_ias = sum_ias / ias_history.size();

        // The gust factor is the difference between the peak and average speed
        result.gust_factor = max_ias - avg_ias;
        
    } else {
        result.gust_factor = 0.0;
    }
    
    return result;
}

/**
 * 2. Envelope margins calculation
 * Computes margins to stall, VMO, and MMO
 */
struct EnvelopeMargins {
    double stall_margin_pct;      // % margin above stall
    double vmo_margin_pct;        // % margin below VMO
    double mmo_margin_pct;        // % margin below MMO
    double min_margin_pct;        // Minimum of all margins
    double corner_speed_kts;      // Estimated corner speed
    double current_load_factor;   // Current load factor
};

[[nodiscard]] EnvelopeMargins calculate_envelope(
    double /* weight_kg */,  // Reserved for future weight-based calculations
    double bank_deg, 
    double ias_kts, 
    double /* tas_kts */,    // Reserved for future TAS-based calculations
    double mach,
    double /* altitude_ft */, // Reserved for future altitude-based calculations
    double vso_kts,  // Stall speed clean config
    double vne_kts,  // Never exceed speed
    double mmo       // Max operating Mach
) {
    EnvelopeMargins result;
    
    // Calculate load factor from bank angle
    double bank_rad = bank_deg * DEG_TO_RAD;
    double load_factor = 1.0 / std::cos(bank_rad);
    result.current_load_factor = load_factor;
    
    // Stall speed at current load factor
    // Vs_n = Vs_1g * sqrt(n)
    double vs_current = vso_kts * std::sqrt(load_factor);
    
    // Stall margin
    if (vs_current > 0) {
        result.stall_margin_pct = ((ias_kts - vs_current) / vs_current) * 100.0;
    } else {
        result.stall_margin_pct = 100.0;
    }
    
    // VMO (velocity max operating) margin
    if (vne_kts > 0) {
        result.vmo_margin_pct = ((vne_kts - ias_kts) / vne_kts) * 100.0;
    } else {
        result.vmo_margin_pct = 100.0;
    }
    
    // MMO (Mach max operating) margin
    if (mmo > 0) {
        result.mmo_margin_pct = ((mmo - mach) / mmo) * 100.0;
    } else {
        result.mmo_margin_pct = 100.0;
    }
    
    // Minimum margin (most limiting)
    result.min_margin_pct = std::min({
        result.stall_margin_pct,
        result.vmo_margin_pct,
        result.mmo_margin_pct
    });
    
    // Corner speed estimate (max load factor speed)
    // Simplified: Vc ≈ Vs * sqrt(n_max), using n_max ≈ 2.5 for transport category
    result.corner_speed_kts = vso_kts * std::sqrt(2.5);
    
    return result;
}

/**
 * 3. Energy management calculation
 * Specific energy and rate of change
 */
enum class EnergyTrend {
    Decreasing = -1,
    Stable = 0,
    Increasing = 1
};

struct EnergyData {
    double specific_energy_ft;     // Specific energy in feet
    double specific_energy_rate;   // dEs/dt (ft/min)
    EnergyTrend trend;             // Energy trend indicator
};

[[nodiscard]] EnergyData calculate_energy(double tas_kts, double altitude_ft, double vs_fpm) {
    EnergyData result;
    
    // Convert to SI units
    double v_ms = tas_kts * KTS_TO_MS;
    double h_m = altitude_ft * FT_TO_M;
    
    // Specific energy: Es = V²/(2g) + h
    double ke_m = (v_ms * v_ms) / (2.0 * GRAVITY);  // Kinetic energy height
    double specific_energy_m = ke_m + h_m;
    result.specific_energy_ft = specific_energy_m * M_TO_FT;
    
    // Rate of change: dEs/dt = V*dV/dt/g + dh/dt
    // Simplified: just use dh/dt (vertical speed) since we don't have acceleration
    result.specific_energy_rate = vs_fpm;  // Already in ft/min
    
    // Trend indicator
    if (result.specific_energy_rate > 50.0) {
        result.trend = EnergyTrend::Increasing;
    } else if (result.specific_energy_rate < -50.0) {
        result.trend = EnergyTrend::Decreasing;
    } else {
        result.trend = EnergyTrend::Stable;
    }
    
    return result;
}

/**
 * 4. Glide reach calculation
 * Estimates maximum glide range considering wind
 */
struct GlideData {
    double max_range_nm;           // Maximum glide range in nautical miles
    double max_range_with_wind_nm; // Range accounting for wind
    double glide_ratio;            // L/D ratio used
    double best_glide_speed_kts;   // Optimal glide speed
};

[[nodiscard]] GlideData calculate_glide_reach(
    double agl_ft,
    double /* tas_kts */,     // Reserved for future dynamic glide calculations
    double /* weight_kg */,   // Reserved for future weight-based glide calculations
    double headwind_kts
) {
    GlideData result;
    
    // Estimate glide ratio based on aircraft type
    // Light aircraft: ~8:1, Jets: ~15:1, Gliders: ~40:1
    // Use simplified estimate: L/D ≈ 12:1 for typical aircraft
    result.glide_ratio = 12.0;
    
    // Best glide speed (simplified - would need polar data for accuracy)
    // Typically 1.3 × Vs or around 70-90 kts for most aircraft
    result.best_glide_speed_kts = 75.0;  // Conservative estimate
    
    // Still air range: R = h × (L/D)
    double altitude_nm = agl_ft / 6076.0;  // Convert feet to nautical miles
    result.max_range_nm = altitude_nm * result.glide_ratio;
    
    // Adjust for wind
    // With headwind, range decreases; with tailwind, increases
    // Use best glide speed (the speed aircraft would actually fly), not current TAS
    double wind_factor = 1.0;
    if (result.best_glide_speed_kts > 0) {
        wind_factor = 1.0 - (headwind_kts / result.best_glide_speed_kts);
    }
    result.max_range_with_wind_nm = result.max_range_nm * wind_factor;
    
    // Ensure non-negative
    if (result.max_range_with_wind_nm < 0) {
        result.max_range_with_wind_nm = 0;
    }
    
    return result;
}

/**
 * Output all results as JSON
 */
void print_json_results(
    const WindData& wind,
    const EnvelopeMargins& envelope,
    const EnergyData& energy,
    const GlideData& glide
) {
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "{\n";
    
    // Wind data
    std::cout << "  \"wind\": {\n";
    std::cout << "    \"speed_kts\": " << wind.speed_kts << ",\n";
    std::cout << "    \"direction_from\": " << wind.direction_from << ",\n";
    std::cout << "    \"headwind\": " << wind.headwind << ",\n";
    std::cout << "    \"crosswind\": " << wind.crosswind << ",\n";
    std::cout << "    \"gust_factor\": " << wind.gust_factor << "\n";
    std::cout << "  },\n";
    
    // Envelope margins
    std::cout << "  \"envelope\": {\n";
    std::cout << "    \"stall_margin_pct\": " << envelope.stall_margin_pct << ",\n";
    std::cout << "    \"vmo_margin_pct\": " << envelope.vmo_margin_pct << ",\n";
    std::cout << "    \"mmo_margin_pct\": " << envelope.mmo_margin_pct << ",\n";
    std::cout << "    \"min_margin_pct\": " << envelope.min_margin_pct << ",\n";
    std::cout << "    \"corner_speed_kts\": " << envelope.corner_speed_kts << ",\n";
    std::cout << "    \"load_factor\": " << envelope.current_load_factor << "\n";
    std::cout << "  },\n";
    
    // Energy management
    std::cout << "  \"energy\": {\n";
    std::cout << "    \"specific_energy_ft\": " << energy.specific_energy_ft << ",\n";
    std::cout << "    \"energy_rate_fpm\": " << energy.specific_energy_rate << ",\n";
    std::cout << "    \"trend\": " << static_cast<int>(energy.trend) << "\n";
    std::cout << "  },\n";
    
    // Glide performance
    std::cout << "  \"glide\": {\n";
    std::cout << "    \"max_range_nm\": " << glide.max_range_nm << ",\n";
    std::cout << "    \"range_with_wind_nm\": " << glide.max_range_with_wind_nm << ",\n";
    std::cout << "    \"glide_ratio\": " << glide.glide_ratio << ",\n";
    std::cout << "    \"best_glide_speed_kts\": " << glide.best_glide_speed_kts << "\n";
    std::cout << "  },\n";
    
    // Recursive function demonstration: Alternate airport combinations
    // Shows how many ways to select alternate airports from nearby options
    std::cout << "  \"alternate_airports\": {\n";
    std::cout << "    \"combinations_5_choose_2\": " << binomial_coefficient(5, 2) << ",\n";
    std::cout << "    \"combinations_10_choose_3\": " << binomial_coefficient(10, 3) << ",\n";
    std::cout << "    \"note\": \"Recursive binomial calculation for flight planning\"\n";
    std::cout << "  }\n";
    
    std::cout << "}\n";
}

} // namespace xplane_mfd::calc

int main(int argc, char* argv[]) {
    using namespace xplane_mfd::calc;
    
    const std::vector<std::string_view> args(argv + 1, argv + argc);
    
    if (args.size() != 14) {
        std::cerr << "Usage: " << argv[0] << " <tas_kts> <gs_kts> <heading> <track> "
                  << "<ias_kts> <mach> <altitude_ft> <agl_ft> <vs_fpm> "
                  << "<weight_kg> <bank_deg> <vso_kts> <vne_kts> <mmo>\n";
        return 1;
    }
    
    try {
        // Parse all inputs
        double tas_kts = parse_double(args[0]);
        double gs_kts = parse_double(args[1]);
        double heading = parse_double(args[2]);
        double track = parse_double(args[3]);
        double ias_kts = parse_double(args[4]);
        double mach = parse_double(args[5]);
        double altitude_ft = parse_double(args[6]);
        double agl_ft = parse_double(args[7]);
        double vs_fpm = parse_double(args[8]);
        double weight_kg = parse_double(args[9]);
        double bank_deg = parse_double(args[10]);
        double vso_kts = parse_double(args[11]);
        double vne_kts = parse_double(args[12]);
        double mmo = parse_double(args[13]);
        
        // Create a sample history of airspeeds to pass to the calculator.
        // In a real system, this would come from a sensor data buffer.
        std::vector<double> ias_history = {145.5, 148.0, 151.2, 149.5, 155.8, 152.1};
        
        // 1. Calculate wind vector (with the MODIFIED call)
        WindData wind = calculate_wind_vector(tas_kts, gs_kts, heading, track, ias_history);
        
        // 2. Calculate envelope margins
        EnvelopeMargins envelope = calculate_envelope(
            weight_kg, bank_deg, ias_kts, tas_kts, mach, altitude_ft,
            vso_kts, vne_kts, mmo
        );
        
        // 3. Calculate energy state
        EnergyData energy = calculate_energy(tas_kts, altitude_ft, vs_fpm);
        
        // 4. Calculate glide reach
        GlideData glide = calculate_glide_reach(agl_ft, tas_kts, weight_kg, wind.headwind);
        
        // Output JSON
        print_json_results(wind, envelope, energy, glide);
        
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
