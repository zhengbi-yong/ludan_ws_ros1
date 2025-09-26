#pragma once

// Provide a project-wide namespace alias so code can use the robot specific
// naming (`ludan`) while continuing to reuse the legacy implementation that
// still lives in the `legged` namespace.
//
// Including this header in translation units guarantees the alias is available
// everywhere without having to refactor every single source file at once.
namespace legged
{
// Intentionally empty – this forward declaration ensures the namespace exists
// before we create an alias for it below. The actual definitions remain in the
// original headers spread across the project.
}  // namespace legged

namespace ludan = legged;

