#pragma once

#include <Eigen/Dense>
#include <cstdint>

namespace liteaerosim {

/// Lightweight Eigen type aliases and bounded-vector utilities used across all
/// subsystems. Keep this header dependency-free (no project headers, no STL
/// beyond cstdint).

// ---------------------------------------------------------------------------
// Fixed-size matrix aliases
// ---------------------------------------------------------------------------

using Vec3  = Eigen::Vector3f;
using Vec2c = Eigen::Vector2cf;

using Mat11 = Eigen::Matrix<float, 1, 1>;
using Mat12 = Eigen::Matrix<float, 1, 2>;
using Mat21 = Eigen::Matrix<float, 2, 1>;
using Mat22 = Eigen::Matrix<float, 2, 2>;

// ---------------------------------------------------------------------------
// Bounded dynamic matrix aliases
//
// Maximum row/column dimension for dynamic matrices used in filter state-space
// representations. Raising this increases stack usage; above 8 risks
// numerical ill-conditioning for floating-point filter design.
// ---------------------------------------------------------------------------

static constexpr int kFilterMaxStates = 8;

using FiltVectorXf = Eigen::Matrix<float, Eigen::Dynamic, 1, 0, kFilterMaxStates + 1, 1>;
using MatNN        = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, 0, kFilterMaxStates, kFilterMaxStates>;
using MatN1        = Eigen::Matrix<float, Eigen::Dynamic, 1, 0, kFilterMaxStates, 1>;
using Mat1N        = Eigen::Matrix<float, 1, Eigen::Dynamic, Eigen::RowMajor, 1, kFilterMaxStates>;

// ---------------------------------------------------------------------------
// Bounded-vector utilities
// ---------------------------------------------------------------------------

/// Resize by zero-padding or truncating from the left.
FiltVectorXf left_resize(const FiltVectorXf& in, int len);

/// Resize by zero-padding or truncating from the right.
FiltVectorXf right_resize(const FiltVectorXf& in, int len);

/// Shift all elements right by one and insert u at position 0.
void roll_buffer(FiltVectorXf& buff, float u);

/// Shift all elements right by one and insert u at position 0 (Vec3 overload).
void roll_buffer(Vec3& buff, float u);

}  // namespace liteaerosim
