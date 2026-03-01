#define _USE_MATH_DEFINES
#include "navigation/WGS84.hpp"
#include "math/math_util.hpp"
#include <gtest/gtest.h>
#include <cmath>

TEST(WGS84Test, Constants) {
    // Test that the WGS84 semi-major axis is correct
    EXPECT_DOUBLE_EQ(WGS84_Datum::a, 6378137.0);

    // Test that the WGS84 flattening factor is correct
    EXPECT_DOUBLE_EQ(WGS84_Datum::finv, 298.257223563);

    // Test that the WGS84 semi-minor axis is calculated correctly
    double expected_b = 6356752.314245;  // WGS84_Datum::a * (1.0 - 1.0/WGS84_Datum::finv);
    EXPECT_NEAR(WGS84_Datum::b, expected_b, 1e-6);

    double expected_e2 = 6.69437999014e-3; // 2.0/WGS84_Datum::finv - 1.0/(WGS84_Datum::finv * WGS84_Datum::finv);
    EXPECT_NEAR(WGS84_Datum::e2, expected_e2, 1e-9);
    EXPECT_NEAR(WGS84_Datum::e, sqrt(expected_e2), 1e-6);

    double expected_E = 521854.0084; // WGS84_Datum::a * sqrt(1.0/WGS84_Datum::finv * (2.0 - 1.0/WGS84_Datum::finv));
    EXPECT_NEAR(WGS84_Datum::E, expected_E, 1e-3);

    double expected_f = 1.0 / WGS84_Datum::finv;
    EXPECT_NEAR(WGS84_Datum::f, expected_f, 1e-9);

}

TEST(WGS84Test, getters) {

    WGS84_Datum datum;
    Eigen::Quaterniond quat;

    datum.setHeight_WGS84_m(100.0f);
    datum.setLatitudeGeodetic_rad(0.5);
    datum.setLongitude_rad(1.0);

    EXPECT_FLOAT_EQ(datum.height_WGS84_m(), 100.0f);
    EXPECT_DOUBLE_EQ(datum.latitudeGeodetic_rad(), 0.5);
    EXPECT_DOUBLE_EQ(datum.longitude_rad(), 1.0);

    Eigen::Vector3d llh = datum.LLH();
    EXPECT_DOUBLE_EQ(llh.x(), 0.5);
    EXPECT_DOUBLE_EQ(llh.y(), 1.0);
    EXPECT_DOUBLE_EQ(llh.z(), 100.0);

    // TODO: verify these values
    Eigen::Vector3d ecef = datum.ECEF();
    EXPECT_NEAR(ecef.x(), 3026633.4955051164, 1e-9);
    EXPECT_NEAR(ecef.y(), 4713702.3855989361, 1e-9);
    EXPECT_NEAR(ecef.z(), 3039758.8494056859, 1e-9);

    datum.setHeight_WGS84_m(0.0f);
    datum.setLatitudeGeodetic_rad(M_PI/4.0);
    datum.setLongitude_rad(0.0);

    // TODO: verify these values
    quat = datum.qne();
    EXPECT_NEAR(quat.w(), cos(M_PI/4.0+M_PI/8.0), 1e-9);
    EXPECT_NEAR(quat.x(), 0, 1e-9);
    EXPECT_NEAR(quat.y(), sin(M_PI/4.0+M_PI/8.0), 1e-9);
    EXPECT_NEAR(quat.z(), 0, 1e-9);

    datum.setHeight_WGS84_m(0.0f);
    datum.setLatitudeGeodetic_rad(0.0);
    datum.setLongitude_rad(0.0);

    // TODO: verify these values
    quat = datum.qne();
    EXPECT_NEAR(quat.w(), cos(M_PI/4.0), 1e-9);
    EXPECT_NEAR(quat.x(), 0.0, 1e-9);
    EXPECT_NEAR(quat.y(), sin(M_PI/4.0), 1e-9);
    EXPECT_NEAR(quat.z(), 0.0, 1e-9);

    Eigen::Matrix3d CNE = quat.toRotationMatrix();

    EXPECT_NEAR(CNE(0,0), 0.0, 1e-9);
    EXPECT_NEAR(CNE(1,0), 0.0, 1e-9);
    EXPECT_NEAR(CNE(2,0), -1.0, 1e-9);
    EXPECT_NEAR(CNE(0,1), 0.0, 1e-9);
    EXPECT_NEAR(CNE(1,1), 1.0, 1e-9);
    EXPECT_NEAR(CNE(2,1), 0.0, 1e-9);
    EXPECT_NEAR(CNE(0,2), 1.0, 1e-9);
    EXPECT_NEAR(CNE(1,2), 0.0, 1e-9);
    EXPECT_NEAR(CNE(2,2), 0.0, 1e-9);

    datum.setHeight_WGS84_m(0.0f);
    datum.setLatitudeGeodetic_rad(M_PI/4.0);
    datum.setLongitude_rad(0.0);

    // seems to use the [sin(phi/2); v*cos(phi/2)] quaternion definition
    quat = datum.qne();
    EXPECT_NEAR(quat.w(), cos(M_PI/4.0 + M_PI/8.0), 1e-9);
    EXPECT_NEAR(quat.x(), 0.0, 1e-9);
    EXPECT_NEAR(quat.y(), sin(M_PI/4.0 + M_PI/8.0), 1e-9);
    EXPECT_NEAR(quat.z(), 0.0, 1e-9);

    CNE = quat.toRotationMatrix();

    EXPECT_NEAR(CNE(0,0), -cos(M_PI/4.0), 1e-9);
    EXPECT_NEAR(CNE(1,0), 0.0, 1e-9);
    EXPECT_NEAR(CNE(2,0), -cos(M_PI/4.0), 1e-9);
    EXPECT_NEAR(CNE(0,1), 0.0, 1e-9);
    EXPECT_NEAR(CNE(1,1), 1.0, 1e-9);
    EXPECT_NEAR(CNE(2,1), 0.0, 1e-9);
    EXPECT_NEAR(CNE(0,2), cos(M_PI/4.0), 1e-9);
    EXPECT_NEAR(CNE(1,2), 0.0, 1e-9);
    EXPECT_NEAR(CNE(2,2), -cos(M_PI/4.0), 1e-9);

    datum.setHeight_WGS84_m(100.0f);
    datum.setLatitudeGeodetic_rad(0.5);
    datum.setLongitude_rad(1.0);

    // TODO: verify these values
    quat = datum.qne();
    EXPECT_NEAR(quat.w(), 0.44772816620802713, 1e-9);
    EXPECT_NEAR(quat.x(), -0.41233739484072318, 1e-9);
    EXPECT_NEAR(quat.y(), 0.75477853845884302, 1e-9);
    EXPECT_NEAR(quat.z(), -0.24459501197263966, 1e-9);

    // 0,0 case
    datum.setHeight_WGS84_m(0.0f);
    datum.setLatitudeGeodetic_rad(0.0);
    datum.setLongitude_rad(0.0);

    CNE = datum.Cne();
    EXPECT_NEAR(CNE(0,0), 0.0, 1e-9);
    EXPECT_NEAR(CNE(1,0), 0.0, 1e-9);
    EXPECT_NEAR(CNE(2,0), -1.0, 1e-9);
    EXPECT_NEAR(CNE(0,1), 0.0, 1e-9);
    EXPECT_NEAR(CNE(1,1), 1.0, 1e-9);
    EXPECT_NEAR(CNE(2,1), 0.0, 1e-9);
    EXPECT_NEAR(CNE(0,2), 1.0, 1e-9);
    EXPECT_NEAR(CNE(1,2), 0.0, 1e-9);
    EXPECT_NEAR(CNE(2,2), 0.0, 1e-9);

    // 45,0 case
    datum.setHeight_WGS84_m(0.0f);
    datum.setLatitudeGeodetic_rad(M_PI/4.0);
    datum.setLongitude_rad(0.0);

    CNE = datum.Cne();
    EXPECT_NEAR(CNE(0,0), -cos(M_PI/4.0), 1e-9);
    EXPECT_NEAR(CNE(1,0), 0.0, 1e-9);
    EXPECT_NEAR(CNE(2,0), -cos(M_PI/4.0), 1e-9);
    EXPECT_NEAR(CNE(0,1), 0.0, 1e-9);
    EXPECT_NEAR(CNE(1,1), 1.0, 1e-9);   
    EXPECT_NEAR(CNE(2,1), 0.0, 1e-9);
    EXPECT_NEAR(CNE(0,2), cos(M_PI/4.0), 1e-9);
    EXPECT_NEAR(CNE(1,2), 0.0, 1e-9);
    EXPECT_NEAR(CNE(2,2), -cos(M_PI/4.0), 1e-9);

    // 0,90 case
    datum.setHeight_WGS84_m(0.0f);
    datum.setLatitudeGeodetic_rad(0.0);
    datum.setLongitude_rad(M_PI/2.0);

    CNE = datum.Cne();
    EXPECT_NEAR(CNE(0,0), 0.0, 1e-9);
    EXPECT_NEAR(CNE(1,0), -1.0, 1e-9);
    EXPECT_NEAR(CNE(2,0), 0.0, 1e-9);
    EXPECT_NEAR(CNE(0,1), 0.0, 1e-9);
    EXPECT_NEAR(CNE(1,1), 0.0, 1e-9);
    EXPECT_NEAR(CNE(2,1), -1.0, 1e-9);
    EXPECT_NEAR(CNE(0,2), 1.0, 1e-9);
    EXPECT_NEAR(CNE(1,2), 0.0, 1e-9);
    EXPECT_NEAR(CNE(2,2), 0.0, 1e-9);

    // 0,180 case
    datum.setHeight_WGS84_m(0.0f);
    datum.setLatitudeGeodetic_rad(0.0);
    datum.setLongitude_rad(M_PI);
    CNE = datum.Cne();
    EXPECT_NEAR(CNE(0,0), 0.0, 1e-9);
    EXPECT_NEAR(CNE(1,0), 0.0, 1e-9);
    EXPECT_NEAR(CNE(2,0), 1.0, 1e-9);
    EXPECT_NEAR(CNE(0,1), 0.0, 1e-9);
    EXPECT_NEAR(CNE(1,1), -1.0, 1e-9);
    EXPECT_NEAR(CNE(2,1), 0.0, 1e-9);
    EXPECT_NEAR(CNE(0,2), 1.0, 1e-9);
    EXPECT_NEAR(CNE(1,2), 0.0, 1e-9);
    EXPECT_NEAR(CNE(2,2), 0.0, 1e-9);

    // 0,-90 case
    datum.setHeight_WGS84_m(0.0f);
    datum.setLatitudeGeodetic_rad(0.0);
    datum.setLongitude_rad(-M_PI/2.0);  
    CNE = datum.Cne();
    EXPECT_NEAR(CNE(0,0), 0.0, 1e-9);
    EXPECT_NEAR(CNE(1,0), 1.0, 1e-9);
    EXPECT_NEAR(CNE(2,0), 0.0, 1e-9);
    EXPECT_NEAR(CNE(0,1), 0.0, 1e-9);
    EXPECT_NEAR(CNE(1,1), 0.0, 1e-9);
    EXPECT_NEAR(CNE(2,1), 1.0, 1e-9);
    EXPECT_NEAR(CNE(0,2), 1.0, 1e-9);
    EXPECT_NEAR(CNE(1,2), 0.0, 1e-9);
    EXPECT_NEAR(CNE(2,2), 0.0, 1e-9);

    // 0,30 case
    datum.setHeight_WGS84_m(0.0f);
    datum.setLatitudeGeodetic_rad(0.0);
    datum.setLongitude_rad(M_PI/6.0);
    CNE = datum.Cne();
    EXPECT_NEAR(CNE(0,0), 0.0, 1e-9);
    EXPECT_NEAR(CNE(1,0), -sin(M_PI/6.0), 1e-9);
    EXPECT_NEAR(CNE(2,0), -cos(M_PI/6.0), 1e-9);
    EXPECT_NEAR(CNE(0,1), 0.0, 1e-9);
    EXPECT_NEAR(CNE(1,1), cos(M_PI/6.0), 1e-9);
    EXPECT_NEAR(CNE(2,1), -sin(M_PI/6.0), 1e-9);
    EXPECT_NEAR(CNE(0,2), 1.0, 1e-9);
    EXPECT_NEAR(CNE(1,2), 0.0, 1e-9);
    EXPECT_NEAR(CNE(2,2), 0.0, 1e-9);

    // 0,-30 case
    datum.setHeight_WGS84_m(0.0f);
    datum.setLatitudeGeodetic_rad(0.0);
    datum.setLongitude_rad(-M_PI/6.0);
    CNE = datum.Cne();
    EXPECT_NEAR(CNE(0,0), 0.0, 1e-9);
    EXPECT_NEAR(CNE(1,0), -sin(-M_PI/6.0), 1e-9);
    EXPECT_NEAR(CNE(2,0), -cos(-M_PI/6.0), 1e-9);
    EXPECT_NEAR(CNE(0,1), 0.0, 1e-9);
    EXPECT_NEAR(CNE(1,1), cos(-M_PI/6.0), 1e-9);
    EXPECT_NEAR(CNE(2,1), -sin(-M_PI/6.0), 1e-9);
    EXPECT_NEAR(CNE(0,2), 1.0, 1e-9);
    EXPECT_NEAR(CNE(1,2), 0.0, 1e-9);
    EXPECT_NEAR(CNE(2,2), 0.0, 1e-9);

    // 45,45 case
    datum.setHeight_WGS84_m(0.0f);
    datum.setLatitudeGeodetic_rad(M_PI/4.0);
    datum.setLongitude_rad(M_PI/4.0);
    CNE = datum.Cne();
    EXPECT_NEAR(CNE(0,0), -0.5, 1e-9);
    EXPECT_NEAR(CNE(1,0), -0.70710678118654757, 1e-9);
    EXPECT_NEAR(CNE(2,0), -0.5, 1e-9);
    EXPECT_NEAR(CNE(0,1), -0.5, 1e-9);
    EXPECT_NEAR(CNE(1,1), 0.70710678118654757, 1e-9);
    EXPECT_NEAR(CNE(2,1), -0.5, 1e-9);
    EXPECT_NEAR(CNE(0,2), 0.70710678118654757, 1e-9);
    EXPECT_NEAR(CNE(1,2), -0.0, 1e-9);
    EXPECT_NEAR(CNE(2,2), -0.70710678118654757, 1e-9);

}

TEST(WGS84Test, setters) {

    WGS84_Datum datum;

    Eigen::Vector3d ecef_set(3026633.4955051164, 4713702.3855989361, 3039758.8494056859);
    datum.setECEF(ecef_set);
    EXPECT_NEAR(datum.latitudeGeodetic_rad(), 0.5, 1e-9);
    EXPECT_NEAR(datum.longitude_rad(), 1.0, 1e-9);
    EXPECT_NEAR(datum.height_WGS84_m(), 100.0f, 1e-9);

    // 45,0 case
    Eigen::Quaterniond qne_set(0.0, sin(-0.5*(M_PI/2.0 + M_PI/4.0)), 0.0, cos(-0.5*(M_PI/2.0 + M_PI/4.0)));
    datum.setQne(qne_set);
    EXPECT_NEAR(datum.latitudeGeodetic_rad(), M_PI/4, 1e-9);
    EXPECT_NEAR(datum.longitude_rad(), 0.0, 1e-9);
    EXPECT_NEAR(datum.height_WGS84_m(), 100.0f, 1e-9);

    // 30,0 case
    qne_set = Eigen::Quaterniond(0.0, sin(-0.5*(M_PI/2.0 + M_PI/6.0)), 0.0, cos(-0.5*(M_PI/2.0 + M_PI/6.0)));
    datum.setQne(qne_set);
    EXPECT_NEAR(datum.latitudeGeodetic_rad(), M_PI/6, 1e-9);
    EXPECT_NEAR(datum.longitude_rad(), 0.0, 1e-9);
    EXPECT_NEAR(datum.height_WGS84_m(), 100.0f, 1e-9);

    Eigen::Vector3d llh_set(0.5, 1.0, 100.0);
    datum.setLLH(llh_set);
    EXPECT_NEAR(datum.latitudeGeodetic_rad(), 0.5, 1e-9);
    EXPECT_NEAR(datum.longitude_rad(), 1.0, 1e-9);
    EXPECT_NEAR(datum.height_WGS84_m(), 100.0f, 1e-9);

    // 0,0 case
    Eigen::Matrix3d Cne_set;
    Cne_set << 0, 0, 1,
               0, 1, 0,
              -1, 0, 0;

    datum.setCne(Cne_set);
    EXPECT_NEAR(datum.latitudeGeodetic_rad(), 0.0, 1e-9);
    EXPECT_NEAR(datum.longitude_rad(), 0.0, 1e-9);

    // 45,0 case
    Cne_set << -cos(M_PI/4.0), 0, cos(M_PI/4.0),
               0, 1, 0,
              -cos(M_PI/4.0), 0, -cos(M_PI/4.0);

    datum.setCne(Cne_set);
    EXPECT_NEAR(datum.latitudeGeodetic_rad(), M_PI/4.0, 1e-9);
    EXPECT_NEAR(datum.longitude_rad(), 0.0, 1e-9);

    // 0,90 case
    Cne_set << 0,  0, 1,
              -1,  0, 0,
               0, -1, 0;

    datum.setCne(Cne_set);
    EXPECT_NEAR(datum.latitudeGeodetic_rad(), 0.0, 1e-9);
    EXPECT_NEAR(datum.longitude_rad(), M_PI/2.0, 1e-9);

    // 0,180 case
    Cne_set << 0,  0, 1,
               0, -1, 0,
               1,  0, 0;

    datum.setCne(Cne_set);
    EXPECT_NEAR(datum.latitudeGeodetic_rad(), 0.0, 1e-9);
    EXPECT_NEAR(MathUtil::wrapToPi(datum.longitude_rad()-M_PI), 0.0, 1e-9);

    // 0,-90 case
    Cne_set << 0, 0, 1,
               1, 0, 0,
               0, 1, 0;

    datum.setCne(Cne_set);
    EXPECT_NEAR(datum.latitudeGeodetic_rad(), 0.0, 1e-9);
    EXPECT_NEAR(datum.longitude_rad(), -M_PI/2.0, 1e-9);

    // 0,30 case
    Cne_set << 0,  0, 1,
              -sin(M_PI/6.0),  cos(M_PI/6.0), 0,
              -cos(M_PI/6.0), -sin(M_PI/6.0), 0;

    datum.setCne(Cne_set);
    EXPECT_NEAR(datum.latitudeGeodetic_rad(), 0.0, 1e-9);
    EXPECT_NEAR(datum.longitude_rad(), M_PI/6.0, 1e-9);

    // 0,-30 case
    Cne_set << 0,  0, 1,
              -sin(-M_PI/6.0),  cos(-M_PI/6.0), 0,
              -cos(-M_PI/6.0), -sin(-M_PI/6.0), 0;

    datum.setCne(Cne_set);
    EXPECT_NEAR(datum.latitudeGeodetic_rad(), 0.0, 1e-9);
    EXPECT_NEAR(datum.longitude_rad(), -M_PI/6.0, 1e-9);

    // 45,90 case
    Cne_set <<  0, -cos(M_PI/4.0), cos(M_PI/4.0),
               -1, 0, 0,
                0, -cos(M_PI/4.0), -cos(M_PI/4.0);

    datum.setCne(Cne_set);
    EXPECT_NEAR(datum.latitudeGeodetic_rad(), M_PI/4.0, 1e-9);
    EXPECT_NEAR(datum.longitude_rad(), M_PI/2.0, 1e-9);

    // 30,90 case
    Cne_set <<  0, -sin(M_PI/6.0), cos(M_PI/6.0),
               -1, 0, 0,
                0, -cos(M_PI/6.0), -sin(M_PI/6.0);

    datum.setCne(Cne_set);
    EXPECT_NEAR(datum.latitudeGeodetic_rad(), M_PI/6.0, 1e-9);
    EXPECT_NEAR(datum.longitude_rad(), M_PI/2.0, 1e-9);

}

TEST(WGS84Test, getter_setter_round_trip) {

    WGS84_Datum datum;
    WGS84_Datum datum_set;

    Eigen::Vector3d ecef_set;
    Eigen::Quaterniond qne_set;
    Eigen::Vector3d llh_set;
    Eigen::Matrix3d Cne_set;

    Eigen::Vector3d ecef_get;
    Eigen::Quaterniond qne_get;
    Eigen::Vector3d llh_get;
    Eigen::Matrix3d Cne_get;

    ecef_set << 3026633.4955051164, 4713702.3855989361, 3039758.8494056859;
    datum.setECEF(ecef_set);
    ecef_get = datum.ECEF();
    EXPECT_NEAR(ecef_get.x(), ecef_set.x(), 1e-9);
    EXPECT_NEAR(ecef_get.y(), ecef_set.y(), 1e-9);
    EXPECT_NEAR(ecef_get.z(), ecef_set.z(), 1e-9);

    qne_set = Eigen::Vector4d(0.85030064529223293, -0.11861177641841195, -0.21711740038440563, 0.46452135963892849);
    qne_set = WGS84_Datum::qne_fix(qne_set);

    Cne_set = qne_set.toRotationMatrix();
    qne_get = Eigen::Quaterniond(Cne_set);
    qne_get.normalize();
    EXPECT_NEAR(qne_get.w(), qne_set.w(), 1e-9);
    EXPECT_NEAR(qne_get.x(), qne_set.x(), 1e-9);
    EXPECT_NEAR(qne_get.y(), qne_set.y(), 1e-9);
    EXPECT_NEAR(qne_get.z(), qne_set.z(), 1e-9);

    qne_set = Eigen::Vector4d(0.85030064529223293, -0.11861177641841195, -0.21711740038440563, 0.46452135963892849);
    qne_set = WGS84_Datum::qne_fix(qne_set);

    datum.setQne(qne_set);
    qne_get = datum.qne();
    qne_get.normalize();
    EXPECT_NEAR(qne_get.w(), qne_set.w(), 1e-9);
    EXPECT_NEAR(qne_get.x(), qne_set.x(), 1e-9);
    EXPECT_NEAR(qne_get.y(), qne_set.y(), 1e-9);
    EXPECT_NEAR(qne_get.z(), qne_set.z(), 1e-9);

    qne_set = Eigen::Quaterniond::UnitRandom();
    qne_set = WGS84_Datum::qne_fix(qne_set);

    datum.setQne(qne_set);
    qne_get = datum.qne();
    qne_get.normalize();
    EXPECT_NEAR(qne_get.w(), qne_set.w(), 1e-9);
    EXPECT_NEAR(qne_get.x(), qne_set.x(), 1e-9);
    EXPECT_NEAR(qne_get.y(), qne_set.y(), 1e-9);
    EXPECT_NEAR(qne_get.z(), qne_set.z(), 1e-9);


    llh_set << 0.5, 1.0, 100.0;
    datum.setLLH(llh_set);
    llh_get = datum.LLH();
    EXPECT_NEAR(llh_get.x(), llh_set.x(), 1e-9);
    EXPECT_NEAR(llh_get.y(), llh_set.y(), 1e-9);
    EXPECT_NEAR(llh_get.z(), llh_set.z(), 1e-9);

    // 0,0 case
    Cne_set << 0, 0, 1,
               0, 1, 0,
              -1, 0, 0;
    datum.setCne(Cne_set);
    Cne_get = datum.Cne();
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            EXPECT_NEAR(Cne_get(i,j), Cne_set(i,j), 1e-9);
        }
    }

    // 45,0 case
    Cne_set <<  0, -sin(M_PI/4.0), cos(M_PI/4.0),
               -1, 0, 0,
                0, -cos(M_PI/4.0), -sin(M_PI/4.0);
    datum.setCne(Cne_set);
    Cne_get = datum.Cne();
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            EXPECT_NEAR(Cne_get(i,j), Cne_set(i,j), 1e-9);
        }
    }

    // 30,0 case
    Cne_set <<  0, -sin(M_PI/6.0), cos(M_PI/6.0),
               -1, 0, 0,
                0, -cos(M_PI/6.0), -sin(M_PI/6.0);
    datum.setCne(Cne_set);
    Cne_get = datum.Cne();
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            EXPECT_NEAR(Cne_get(i,j), Cne_set(i,j), 1e-9);
        }
    }

    datum.setHeight_WGS84_m(100.0f);
    datum.setLatitudeGeodetic_rad(0.5);
    datum.setLongitude_rad(1.0);

    ecef_get = datum.ECEF();
    datum_set.setECEF(ecef_get);
    EXPECT_NEAR(datum_set.height_WGS84_m(), 100.0f, 1e-9);
    EXPECT_NEAR(datum_set.latitudeGeodetic_rad(), 0.5, 1e-9);
    EXPECT_NEAR(datum_set.longitude_rad(), 1.0, 1e-9);

    qne_get = datum.qne();
    datum_set.setQne(qne_get);
    EXPECT_NEAR(datum_set.height_WGS84_m(), 100.0f, 1e-9);
    EXPECT_NEAR(datum_set.latitudeGeodetic_rad(), 0.5, 1e-9);
    EXPECT_NEAR(datum_set.longitude_rad(), 1.0, 1e-9);

    llh_get = datum.LLH();
    datum_set.setLLH(llh_get);
    EXPECT_NEAR(datum_set.height_WGS84_m(), 100.0f, 1e-9);
    EXPECT_NEAR(datum_set.latitudeGeodetic_rad(), 0.5, 1e-9);
    EXPECT_NEAR(datum_set.longitude_rad(), 1.0, 1e-9);

    Cne_get = datum.Cne();
    datum_set.setCne(Cne_get);
    EXPECT_NEAR(datum_set.height_WGS84_m(), 100.0f, 1e-9);
    EXPECT_NEAR(datum_set.latitudeGeodetic_rad(), 0.5, 1e-9);
    EXPECT_NEAR(datum_set.longitude_rad(), 1.0, 1e-9);


    Cne_get = datum.Cne();
    qne_get = Eigen::Quaterniond(Cne_get);
    Cne_set = qne_get.toRotationMatrix();
    datum_set.setCne(Cne_set);
    EXPECT_NEAR(datum_set.height_WGS84_m(), 100.0f, 1e-9);
    EXPECT_NEAR(datum_set.latitudeGeodetic_rad(), 0.5, 1e-9);
    EXPECT_NEAR(datum_set.longitude_rad(), 1.0, 1e-9);

    Cne_get = datum.Cne();
    qne_get = Eigen::Quaterniond(Cne_get);
    Cne_set = qne_get.toRotationMatrix();
    datum_set.setCne(Cne_set);
    EXPECT_NEAR(datum_set.height_WGS84_m(), 100.0f, 1e-9);
    EXPECT_NEAR(datum_set.latitudeGeodetic_rad(), 0.5, 1e-9);
    EXPECT_NEAR(datum_set.longitude_rad(), 1.0, 1e-9);

    // random rotation matrix test
    qne_set = Eigen::Quaterniond::UnitRandom();
    qne_set = WGS84_Datum::qne_fix(qne_set);

    Cne_set = qne_set.toRotationMatrix();
    datum.setCne(Cne_set);
    Cne_get = datum.Cne();
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            EXPECT_NEAR(Cne_get(i,j), Cne_set(i,j), 1e-9);
        }
    }

}

// test earth curvatures at various datum points
TEST(WGS84Test, curvatures) {

    // test at equator and prime meridian
    WGS84_Datum datum;
    datum.setLatitudeGeodetic_rad(0.0);
    datum.setLongitude_rad(0.0);
    double Re_N = datum.meridionalRadius();
    EXPECT_NEAR(Re_N, 6335439.327292124, 1e-6);
    Re_N = datum.northRadius();
    EXPECT_NEAR(Re_N, 6335439.327292124, 1e-6);
    double Re_E = datum.primeVerticalRadius();
    EXPECT_NEAR(Re_E, 6378137.0, 1e-6);
    Re_E = datum.eastRadius();
    EXPECT_NEAR(Re_E, 6378137.0, 1e-6);

    // test at 45 deg latitude
    datum.setLatitudeGeodetic_rad(M_PI/4.0);
    datum.setLongitude_rad(0.0);
    Re_N = datum.meridionalRadius();
    EXPECT_NEAR(Re_N, 6367381.8156195479, 1e-6);
    Re_N = datum.northRadius();
    EXPECT_NEAR(Re_N, 6367381.8156195479, 1e-6);
    Re_E = datum.primeVerticalRadius();
    EXPECT_NEAR(Re_E, 6388838.290121146, 1e-6);
    Re_E = datum.eastRadius(); // 4517590.8788489318 ??
    EXPECT_NEAR(Re_E, 6388838.290121146*cos(45.0 * M_PI / 180.0), 1e-6);

    // test at -30 deg latitude
    datum.setLatitudeGeodetic_rad(-30.0 * M_PI / 180.0);
    datum.setLongitude_rad(0.0);
    Re_N = datum.meridionalRadius();
    EXPECT_NEAR(Re_N, 6351377.1037155138, 1e-6);
    Re_N = datum.northRadius();
    EXPECT_NEAR(Re_N, 6351377.1037155138, 1e-6);
    Re_E = datum.primeVerticalRadius();
    EXPECT_NEAR(Re_E, 6383480.9176901085, 1e-6);
    Re_E = datum.eastRadius(); // ??
    EXPECT_NEAR(Re_E, 6383480.9176901085*cos(-30.0 * M_PI / 180.0), 1e-6);

    // test at -90 deg latitude
    datum.setLatitudeGeodetic_rad(-M_PI/2.0);
    datum.setLongitude_rad(0.0);
    Re_N = datum.meridionalRadius();
    EXPECT_NEAR(Re_N, 6399593.625758194, 1e-6);
    Re_N = datum.northRadius();
    EXPECT_NEAR(Re_N, 6399593.625758194, 1e-6);
    Re_E = datum.primeVerticalRadius();
    EXPECT_NEAR(Re_E, 6399593.6257584933, 1e-6);
    Re_E = datum.eastRadius();
    EXPECT_NEAR(Re_E, 0.0, 1e-6);

    // test at 90 deg latitude
    datum.setLatitudeGeodetic_rad(M_PI/2.0);
    datum.setLongitude_rad(0.0);
    Re_N = datum.meridionalRadius();
    EXPECT_NEAR(Re_N, 6399593.625758194, 1e-6);
    Re_N = datum.northRadius();
    EXPECT_NEAR(Re_N, 6399593.625758194, 1e-6);
    Re_E = datum.primeVerticalRadius();
    EXPECT_NEAR(Re_E, 6399593.6257584933, 1e-6);
    Re_E = datum.eastRadius();
    EXPECT_NEAR(Re_E, 0.0, 1e-6);

    // test at 0 deg latitude and 45 deg longitude
    datum.setLatitudeGeodetic_rad(0.0);
    datum.setLongitude_rad(M_PI/4.0);
    Re_N = datum.meridionalRadius();
    EXPECT_NEAR(Re_N, 6335439.327292124, 1e-6);
    Re_N = datum.northRadius();
    EXPECT_NEAR(Re_N, 6335439.327292124, 1e-6);
    Re_E = datum.primeVerticalRadius();
    EXPECT_NEAR(Re_E, 6378137.0, 1e-6);
    Re_E = datum.eastRadius();
    EXPECT_NEAR(Re_E, 6378137.0, 1e-6);

    // test at 30 deg latitude and -60 deg longitude
    datum.setLatitudeGeodetic_rad(30.0 * M_PI / 180.0);
    datum.setLongitude_rad(-60.0 * M_PI / 180.0);
    Re_N = datum.meridionalRadius();
    EXPECT_NEAR(Re_N, 6351377.1037155138, 1e-6);
    Re_N = datum.northRadius();
    EXPECT_NEAR(Re_N, 6351377.1037155138, 1e-6);
    Re_E = datum.primeVerticalRadius();
    EXPECT_NEAR(Re_E, 6383480.9176901085, 1e-6);
    Re_E = datum.eastRadius();
    EXPECT_NEAR(Re_E, 6383480.9176901085*cos(30.0 * M_PI / 180.0), 1e-6);

    // test skewRadius at the equator
    datum.setLatitudeGeodetic_rad(0.0);
    datum.setLongitude_rad(0.0);
    double Re_S = datum.skewRadius(90.0*M_PI/180.0);
    EXPECT_NEAR(Re_S, 6378137.0, 1e-6);
    Re_S = datum.skewRadius(0.0*M_PI/180.0);
    EXPECT_NEAR(Re_S, 6335439.327292124, 1e-6);
    Re_S = datum.skewRadius(45.0*M_PI/180.0);
    EXPECT_NEAR(Re_S, 6356716.4650461227, 1e-6);
    Re_S = datum.skewRadius(135.0*M_PI/180.0);
    EXPECT_NEAR(Re_S, 6356716.4650461227, 1e-6);
    Re_S = datum.skewRadius(-45.0*M_PI/180.0);
    EXPECT_NEAR(Re_S, 6356716.4650461227, 1e-6);
    Re_S = datum.skewRadius(-135.0*M_PI/180.0);
    EXPECT_NEAR(Re_S, 6356716.4650461227, 1e-6);
    Re_S = datum.skewRadius(180.0*M_PI/180.0);
    EXPECT_NEAR(Re_S, 6335439.327292124, 1e-6);
    
}

// test cases for transport rate
TEST(WGS84Test, transportRates) {

    WGS84_Datum datum;

    // test at equator and prime meridian
    datum.setLatitudeGeodetic_rad(0.0);
    datum.setLongitude_rad(0.0);
    datum.setHeight_WGS84_m(0.0f);
    double Vn = 1000.0; // m/s
    double Ve = 0.0;    // m/s
    double lat_rate = datum.latitudeRate(Vn);
    double lon_rate = datum.longitudeRate(Ve);
    double horiz_rate = datum.horizonRate(Vn, Ve);
    EXPECT_NEAR(lat_rate, Vn / datum.meridionalRadius(), 1e-12);
    EXPECT_NEAR(lon_rate, Ve / (datum.primeVerticalRadius() * cos(0.0)), 1e-12);
    EXPECT_NEAR(horiz_rate, sqrt(lat_rate*lat_rate + lon_rate*lon_rate), 1e-12);

    // test at 45 deg latitude, north velocity
    datum.setLatitudeGeodetic_rad(M_PI/4.0);
    datum.setLongitude_rad(0.0);
    datum.setHeight_WGS84_m(0.0f);
    Vn = 1000.0; // m/s
    Ve = 0.0;    // m/s
    lat_rate = datum.latitudeRate(Vn);
    lon_rate = datum.longitudeRate(Ve);
    horiz_rate = datum.horizonRate(Vn, Ve);
    EXPECT_NEAR(lat_rate, Vn / datum.northRadius(), 1e-12);
    EXPECT_NEAR(lon_rate, 0.0, 1e-12);
    EXPECT_NEAR(horiz_rate, Vn / datum.northRadius(), 1e-12);

    // test at 45 deg latitude, east velocity
    datum.setLatitudeGeodetic_rad(M_PI/4.0);
    datum.setLongitude_rad(0.0);
    datum.setHeight_WGS84_m(0.0f);
    Vn = 0.0;    // m/s
    Ve = 1000.0; // m/s
    lat_rate = datum.latitudeRate(Vn);
    lon_rate = datum.longitudeRate(Ve);
    horiz_rate = datum.horizonRate(Vn, Ve);
    EXPECT_NEAR(lat_rate, 0.0, 1e-12);
    EXPECT_NEAR(lon_rate, Ve/datum.eastRadius(), 1e-12);
    EXPECT_NEAR(horiz_rate, Ve/datum.primeVerticalRadius(), 1e-12);

    // test at 45 deg latitude, skewed azimuth
    datum.setLatitudeGeodetic_rad(M_PI/4.0);
    datum.setLongitude_rad(0.0);
    datum.setHeight_WGS84_m(0.0f);
    Vn = 1000.0; // m/s
    Ve = 500.0;    // m/s
    lat_rate = datum.latitudeRate(Vn);
    lon_rate = datum.longitudeRate(Ve);
    horiz_rate = datum.horizonRate(Vn, Ve);
    EXPECT_NEAR(lat_rate, Vn / datum.northRadius(), 1e-12);
    EXPECT_NEAR(lon_rate, Ve / datum.eastRadius(), 1e-12);
    EXPECT_NEAR(horiz_rate, sqrt(Vn*Vn + Ve*Ve)/datum.skewRadius(atan2(Ve,Vn)), 1e-12);

    // test transport rate vector calculation
    datum.setLatitudeGeodetic_rad(M_PI/4.0);
    datum.setLongitude_rad(0.0);
    datum.setHeight_WGS84_m(0.0f);
    Vn = 1000.0; // m/s
    Ve = 500.0;    // m/s
    Eigen::Vector3d omega_en_n = datum.transportRate(Vn, Ve);
    EXPECT_NEAR(omega_en_n.x(), -datum.horizonRate(Vn,Ve)*Ve/hypot(Vn,Ve), 1e-12);
    EXPECT_NEAR(omega_en_n.y(), -datum.horizonRate(Vn,Ve)*Vn/hypot(Vn,Ve), 1e-12);
    EXPECT_NEAR(omega_en_n.z(), -datum.longitudeRate(Ve), 1e-12);


    // test at 45 deg latitude, skewed azimuth, height 50000 m
    double h = 50000.0; // m
    double lat_rad = M_PI/4.0;
    datum.setLatitudeGeodetic_rad(lat_rad);
    datum.setLongitude_rad(0.0);
    datum.setHeight_WGS84_m(h);
    Vn = 1000.0; // m/s
    Ve = 500.0;    // m/s
    lat_rate = datum.latitudeRate(Vn);
    lon_rate = datum.longitudeRate(Ve);
    horiz_rate = datum.horizonRate(Vn, Ve);
    EXPECT_NEAR(lat_rate, Vn / (datum.northRadius() + h), 1e-12);
    EXPECT_NEAR(lon_rate, Ve / (datum.eastRadius() + cos(lat_rad)*h), 1e-12);
    EXPECT_NEAR(horiz_rate, sqrt(Vn*Vn + Ve*Ve)/(datum.skewRadius(atan2(Ve,Vn)) + h), 1e-12);

    // test transport rate vector calculation, height 50000 m
    h = 50000.0; // m
    lat_rad = M_PI/4.0;
    datum.setLatitudeGeodetic_rad(lat_rad);
    datum.setLongitude_rad(0.0);
    datum.setHeight_WGS84_m(h);
    Vn = 1000.0; // m/s
    Ve = 500.0;    // m/s
    omega_en_n = datum.transportRate(Vn, Ve);
    EXPECT_NEAR(omega_en_n.x(), -datum.horizonRate(Vn,Ve)*Ve/hypot(Vn,Ve), 1e-12);
    EXPECT_NEAR(omega_en_n.y(), -datum.horizonRate(Vn,Ve)*Vn/hypot(Vn,Ve), 1e-12);
    EXPECT_NEAR(omega_en_n.z(), -datum.longitudeRate(Ve), 1e-12);

}

// test cases for earth rate
TEST(WGS84Test, earthRates) {

    WGS84_Datum datum;

    // test at equator and prime meridian
    datum.setLatitudeGeodetic_rad(0.0);
    datum.setLongitude_rad(0.0);
    Eigen::Vector3d omega_ie_n = datum.omega_ie_n();
    EXPECT_NEAR(omega_ie_n.x(), WGS84_Datum::omega, 1e-9);
    EXPECT_NEAR(omega_ie_n.y(), 0.0, 1e-9);
    EXPECT_NEAR(omega_ie_n.z(), 0.0, 1e-12);

    // test at 45 deg latitude
    datum.setLatitudeGeodetic_rad(M_PI/4.0);
    datum.setLongitude_rad(0.0);
    omega_ie_n = datum.omega_ie_n();
    EXPECT_NEAR(omega_ie_n.x(), WGS84_Datum::omega * cos(M_PI/4.0), 1e-12);
    EXPECT_NEAR(omega_ie_n.y(), 0.0, 1e-9);
    EXPECT_NEAR(omega_ie_n.z(), -WGS84_Datum::omega * sin(M_PI/4.0), 1e-12);

    // test at -30 deg latitude
    datum.setLatitudeGeodetic_rad(-30.0 * M_PI / 180.0);
    datum.setLongitude_rad(0.0);
    omega_ie_n = datum.omega_ie_n();
    EXPECT_NEAR(omega_ie_n.x(), WGS84_Datum::omega * cos(-30.0 * M_PI / 180.0), 1e-12);
    EXPECT_NEAR(omega_ie_n.y(), 0.0, 1e-9);
    EXPECT_NEAR(omega_ie_n.z(), -WGS84_Datum::omega * sin(-30.0 * M_PI / 180.0), 1e-12);

}


// TODO: check these values against a reference source
TEST(WGS84Test, gravity) {

    WGS84_Datum datum;

    // test at equator and prime meridian
    datum.setLatitudeGeodetic_rad(0.0);
    datum.setLongitude_rad(0.0);
    double g = datum.gravityMagnitude_mps2();
    EXPECT_NEAR(g, 9.7803253359, 1e-9);

    // test at 45 deg latitude
    datum.setLatitudeGeodetic_rad(M_PI/4.0);
    datum.setLongitude_rad(0.0);
    g = datum.gravityMagnitude_mps2();
    EXPECT_NEAR(g, 9.8061977693732381, 1e-9);

    // test at 30 deg latitude
    datum.setLatitudeGeodetic_rad(30.0 * M_PI / 180.0);
    datum.setLongitude_rad(0.0);
    g = datum.gravityMagnitude_mps2();
    EXPECT_NEAR(g, 9.793247269215307, 1e-9);

    // test at -30 deg latitude
    datum.setLatitudeGeodetic_rad(-30.0 * M_PI / 180.0);
    datum.setLongitude_rad(0.0);
    g = datum.gravityMagnitude_mps2();
    EXPECT_NEAR(g, 9.793247269215307, 1e-9);

    // test at -90 deg latitude
    datum.setLatitudeGeodetic_rad(-M_PI/2.0);
    datum.setLongitude_rad(0.0);
    g = datum.gravityMagnitude_mps2();
    EXPECT_NEAR(g, 9.8321849378590152, 1e-9);

    // test at 90 deg latitude
    datum.setLatitudeGeodetic_rad(M_PI/2.0);
    datum.setLongitude_rad(0.0);
    g = datum.gravityMagnitude_mps2();
    EXPECT_NEAR(g, 9.8321849378590152, 1e-9);

    // test with altitude = 50000 m
    datum.setHeight_WGS84_m(50000.0f);
    datum.setLatitudeGeodetic_rad(45.0 * M_PI / 180.0);
    datum.setLongitude_rad(0.0);
    g = datum.gravityMagnitude_mps2();
    EXPECT_NEAR(g, 9.6519977693732386, 1e-9);

    // test with altitude = -50000 m
    datum.setHeight_WGS84_m(-50000.0f);
    datum.setLatitudeGeodetic_rad(45.0 * M_PI / 180.0);
    datum.setLongitude_rad(0.0);
    g = datum.gravityMagnitude_mps2();
    EXPECT_NEAR(g, 9.9603977693732376, 1e-9);

    // test with longitude = 90 deg
    datum.setHeight_WGS84_m(0.0f);
    datum.setLatitudeGeodetic_rad(45.0 * M_PI / 180.0);
    datum.setLongitude_rad(M_PI/2.0);
    g = datum.gravityMagnitude_mps2();
    EXPECT_NEAR(g, 9.8061977693732381, 1e-9);

}
