# 5.7 `std::string_view`

`std::string_view` is a non-owning view into character data. It avoids copies but does not extend lifetime.

```cpp
void parse_key(std::string_view key);
```

Do not store a `string_view` to a temporary string. Store `std::string` when ownership is needed.
