# Q9 - Fold Expressions

What does this return?

```cpp
template <typename... Checks>
bool all(Checks... checks) {
    return (checks && ...);
}
```

<details>
<summary>Answer</summary>

It returns the logical AND of every argument in the parameter pack. For example, `all(true, true, false)` returns `false`.

For an empty pack, this unary fold over `&&` has identity `true`.

</details>
