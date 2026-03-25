#pragma once

// WGS84_Datum — thin wrapper around liteaero::nav::GeodeticPosition.
//
// All mathematical operations delegate to liteaero::nav::WGS84 free functions.
// This class preserves the existing liteaero-sim call-site API through Step 7.
// The static constants are aliases to liteaero::nav::WGS84 inline constexpr values.

#include <liteaero/nav/GeodeticPosition.hpp>
#include <liteaero/nav/WGS84.hpp>
#include <Eigen/Dense>
#include <cmath>

class WGS84_Datum {

public:

    // ── Defining parameters (aliases to liteaero::nav::WGS84 constants) ──────

    static constexpr double a     = liteaero::nav::WGS84::kA_m;
    static constexpr double finv  = liteaero::nav::WGS84::kFinv;
    static constexpr double GM    = liteaero::nav::WGS84::kGM;
    static constexpr double omega = liteaero::nav::WGS84::kOmega_rps;

    // Derived parameters — computed from the defining parameters.
    // e, E, b require sqrt; they are defined as inline const in the .cpp translation unit.
    static const double e2;  // = 2/finv - 1/finv²
    static const double e;   // = sqrt(e2)
    static const double E;   // = a * sqrt(1/finv * (2 - 1/finv))
    static const double b;   // = a * (1 - 1/finv)
    static const double f;   // = 1/finv

    // ── Construction ─────────────────────────────────────────────────────────

    WGS84_Datum() = default;
    ~WGS84_Datum() = default;

    /// Construct directly from a GeodeticPosition (no normalization applied).
    explicit WGS84_Datum(const liteaero::nav::GeodeticPosition& pos) : position_(pos) {}

    // ── Setters (normalize lat/lon on assignment) ─────────────────────────────

    void setLatitudeGeodetic_rad(double lat);
    void setLongitude_rad(double lon);
    void setHeight_WGS84_m(float h);

    void setECEF(const Eigen::Vector3d& ecef);
    void setQne(const Eigen::Quaterniond& q_ne);
    void setLLH(const Eigen::Vector3d& llh);
    void setCne(const Eigen::Matrix3d& Cne);

    // ── Getters ───────────────────────────────────────────────────────────────

    double latitudeGeodetic_rad() const { return position_.latitude_rad; }
    double longitude_rad()        const { return position_.longitude_rad; }
    float  height_WGS84_m()       const { return position_.altitude_m; }

    const liteaero::nav::GeodeticPosition& geodeticPosition() const { return position_; }

    // ── Derived quantities — delegate to liteaero::nav::WGS84 ────────────────

    double northRadius()                           const;
    double eastRadius()                            const;
    double meridionalRadius()                      const;
    double primeVerticalRadius()                   const;
    double skewRadius(double azimuth_rad)          const;
    double latitudeRate(double v_north_mps)        const;
    double longitudeRate(double v_east_mps)        const;
    double horizonRate(double v_north_mps, double v_east_mps) const;
    Eigen::Vector3d transportRate(double v_north_mps, double v_east_mps) const;
    double gravityMagnitude_mps2()                 const;
    Eigen::Vector3d omega_ie_n()                   const;

    Eigen::Vector3d  ECEF()  const;
    Eigen::Quaterniond qne() const;
    Eigen::Vector3d  LLH()   const;
    Eigen::Matrix3d  Cne()   const;

    void printJSON();

    // Static utilities
    static Eigen::Quaterniond NED2Nav(const Eigen::Quaterniond& q_ne);
    static Eigen::Quaterniond qne_fix(const Eigen::Quaterniond& q_ne);

private:

    liteaero::nav::GeodeticPosition position_;

    static void normalizeLatLon(double* lat, double* lon);
};
