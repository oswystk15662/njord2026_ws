#pragma once

namespace um982_driver {

// Enum定義（Pythonコード準拠）
enum class SolutionStatus {
    SOL_COMPUTED = 0,
    INSUFFICIENT_OBS = 1,
    NO_CONVERGENCE = 2,
    COV_TRACE = 4
};

enum class PositionType {
    NONE = 0,
    FIXED = 1,
    INT = 2,
    FLOAT = 3
    // ... 他のタイプ
};
}