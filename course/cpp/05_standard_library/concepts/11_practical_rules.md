# 5.10 Practical Rules

- Default to `std::vector` unless another access pattern is clearly better.
- Call `reserve` when the approximate size is known.
- Use `std::array` for fixed dimensions.
- Use `std::chrono` for durations and deadlines.
- Use `std::string_view` only for non-owning parameters.
- Prefer algorithms for search, transform, reduction, and filtering.
