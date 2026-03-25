#define _USE_MATH_DEFINES
#include "navigation/WGS84.hpp"
#include "math/math_util.hpp"
#include <cmath>
#include <cstdio>

// Derived parameters — cannot be constexpr because they require sqrt.
const double WGS84_Datum::e2 = 2.0/WGS84_Datum::finv - 1.0/(WGS84_Datum::finv*WGS84_Datum::finv);
const double WGS84_Datum::e  = std::sqrt(WGS84_Datum::e2);
const double WGS84_Datum::E  = WGS84_Datum::a * std::sqrt(1.0/WGS84_Datum::finv * (2.0 - 1.0/WGS84_Datum::finv));
const double WGS84_Datum::b  = WGS84_Datum::a * (1.0 - 1.0/WGS84_Datum::finv);
const double WGS84_Datum::f  = 1.0 / WGS84_Datum::finv;

// ---------------------------------------------------------------------------
// Private helper — normalize (lat, lon) to canonical range.
// ---------------------------------------------------------------------------

void WGS84_Datum::normalizeLatLon(double* lat, double* lon)
{
    *lat = MathUtil::wrapToPi(*lat);
    *lon = MathUtil::wrapToPi(*lon);

    if (std::fabs(*lat) > M_PI / 2.0) {
        *lat = (*lat > 0.0) ? (M_PI - *lat) : (-M_PI - *lat);
        *lon = MathUtil::wrapToPi(*lon + M_PI);
    }
}

// ---------------------------------------------------------------------------
// Setters
// ---------------------------------------------------------------------------

void WGS84_Datum::setLatitudeGeodetic_rad(double lat)
{
    position_.latitude_rad = lat;
    normalizeLatLon(&position_.latitude_rad, &position_.longitude_rad);
}

void WGS84_Datum::setLongitude_rad(double lon)
{
    position_.longitude_rad = lon;
    normalizeLatLon(&position_.latitude_rad, &position_.longitude_rad);
}

void WGS84_Datum::setHeight_WGS84_m(float h)
{
    if (h < -static_cast<float>(WGS84_Datum::b)) {
        h = -static_cast<float>(WGS84_Datum::b);
    }
    position_.altitude_m = h;
}

void WGS84_Datum::setECEF(const Eigen::Vector3d& ecef)
{
    liteaero::nav::GeodeticPosition p = liteaero::nav::WGS84::fromECEF(ecef);
    setLatitudeGeodetic_rad(p.latitude_rad);
    setLongitude_rad(p.longitude_rad);
    setHeight_WGS84_m(p.altitude_m);
}

void WGS84_Datum::setQne(const Eigen::Quaterniond& q_ne)
{
    liteaero::nav::GeodeticPosition p = liteaero::nav::WGS84::fromQne(q_ne);
    setLatitudeGeodetic_rad(p.latitude_rad);
    setLongitude_rad(p.longitude_rad);
}

void WGS84_Datum::setLLH(const Eigen::Vector3d& llh)
{
    setLatitudeGeodetic_rad(llh(0));
    setLongitude_rad(llh(1));
    setHeight_WGS84_m(static_cast<float>(llh(2)));
}

void WGS84_Datum::setCne(const Eigen::Matrix3d& Cne)
{
    liteaero::nav::GeodeticPosition p = liteaero::nav::WGS84::fromQne(Eigen::Quaterniond(Cne));
    setLatitudeGeodetic_rad(p.latitude_rad);
    setLongitude_rad(p.longitude_rad);
}

// ---------------------------------------------------------------------------
// Derived quantities — one-line delegates to liteaero::nav::WGS84
// ---------------------------------------------------------------------------

double WGS84_Datum::meridionalRadius()    const { return liteaero::nav::WGS84::meridionalRadius(position_.latitude_rad); }
double WGS84_Datum::primeVerticalRadius() const { return liteaero::nav::WGS84::primeVerticalRadius(position_.latitude_rad); }
double WGS84_Datum::northRadius()         const { return liteaero::nav::WGS84::northRadius(position_); }
double WGS84_Datum::eastRadius()          const { return liteaero::nav::WGS84::eastRadius(position_); }

double WGS84_Datum::skewRadius(double azimuth_rad) const
{
    const double N      = primeVerticalRadius();
    const double M      = meridionalRadius();
    const double sin_az = std::sin(azimuth_rad);
    const double cos_az = std::cos(azimuth_rad);
    return N * M / (N * cos_az * cos_az + M * sin_az * sin_az);
}

double WGS84_Datum::latitudeRate(double v_north_mps)   const { return liteaero::nav::WGS84::latitudeRate_rad_s(position_, static_cast<float>(v_north_mps)); }
double WGS84_Datum::longitudeRate(double v_east_mps)   const { return liteaero::nav::WGS84::longitudeRate_rad_s(position_, static_cast<float>(v_east_mps)); }

double WGS84_Datum::horizonRate(double v_north_mps, double v_east_mps) const
{
    const double az     = std::atan2(v_east_mps, v_north_mps);
    const double Rskew  = skewRadius(az) + position_.altitude_m;
    const double Vhoriz = std::sqrt(v_north_mps * v_north_mps + v_east_mps * v_east_mps);
    return Vhoriz / Rskew;
}

Eigen::Vector3d WGS84_Datum::transportRate(double v_north_mps, double v_east_mps) const
{
    return liteaero::nav::WGS84::transportRate(position_,
                                               static_cast<float>(v_north_mps),
                                               static_cast<float>(v_east_mps));
}

double WGS84_Datum::gravityMagnitude_mps2() const { return liteaero::nav::WGS84::gravity_mps2(position_); }

Eigen::Vector3d WGS84_Datum::omega_ie_n() const { return liteaero::nav::WGS84::omega_ie_n(position_); }

Eigen::Vector3d    WGS84_Datum::ECEF() const { return liteaero::nav::WGS84::toECEF(position_); }
Eigen::Quaterniond WGS84_Datum::qne()  const { return liteaero::nav::WGS84::qne(position_); }

Eigen::Vector3d WGS84_Datum::LLH() const
{
    return {position_.latitude_rad, position_.longitude_rad,
            static_cast<double>(position_.altitude_m)};
}

Eigen::Matrix3d WGS84_Datum::Cne() const
{
    return liteaero::nav::WGS84::qne(position_).toRotationMatrix();
}

void WGS84_Datum::printJSON()
{
    std::printf("{\n");
    std::printf("  \"WGS84_Datum\": {\n");
    std::printf("    \"latitude_rad\": %.12f,\n", position_.latitude_rad);
    std::printf("    \"longitude_rad\": %.12f,\n", position_.longitude_rad);
    std::printf("    \"height_m\": %.6f\n", static_cast<double>(position_.altitude_m));
    std::printf("  }\n");
    std::printf("}\n");
}

// ---------------------------------------------------------------------------
// Static utilities
// ---------------------------------------------------------------------------

Eigen::Quaterniond WGS84_Datum::NED2Nav(const Eigen::Quaterniond& q_ne)
{
    // Project ECEF Z axis to navigation frame and extract wander angle.
    Eigen::Vector3d z_nav = q_ne.toRotationMatrix() * Eigen::Vector3d::UnitZ();
    const double sin_wa = z_nav.cross(Eigen::Vector3d::UnitX()).dot(Eigen::Vector3d::UnitZ());
    const double cos_wa = z_nav.dot(Eigen::Vector3d::UnitX());
    const double wa     = std::atan2(sin_wa, cos_wa);
    return Eigen::Quaterniond(Eigen::AngleAxisd(wa, Eigen::Vector3d::UnitZ()));
}

Eigen::Quaterniond WGS84_Datum::qne_fix(const Eigen::Quaterniond& q_ne)
{
    Eigen::Quaterniond q = q_ne.normalized();
    q = NED2Nav(q) * q;
    q.normalize();
    if (q.w() < 0.0) { q.coeffs() = -q.coeffs(); }
    return q;
}
