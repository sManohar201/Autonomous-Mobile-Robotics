# Q2 - Structured Binding Copies

Why can this loop accidentally copy more than intended?

```cpp
for (auto [id, scan] : scan_map) {
    process(scan);
}
```

How should it be written if `scan` is large and read-only?

<details>
<summary>Answer</summary>

`auto [id, scan]` copies each key/value pair member into local variables. If `scan` owns a large vector, every iteration performs an expensive copy.

Use a const reference binding:

```cpp
for (const auto& [id, scan] : scan_map) {
    process(scan);
}
```

</details>
