# 1.1 The Compilation Pipeline

When you run `g++ main.cpp -o program`, four things happen in sequence.

### Step 1 — Preprocessor

The preprocessor runs before compilation. It handles all lines starting with `#`. It does text substitution — it knows nothing about C++ syntax.

```cpp
#include <iostream>      // copy-paste the entire iostream header here
#define PI 3.14159       // replace every occurrence of PI with 3.14159

int main() {
    double area = PI * 5 * 5;   // becomes: double area = 3.14159 * 5 * 5;
}
```

See the preprocessed output:
```bash
g++ -E main.cpp -o main.i
```

### Step 2 — Compiler

Translates preprocessed C++ into object code (`.o` files). Each `.cpp` compiles independently. The compiler checks syntax and types but does **not** resolve references between files.

```bash
g++ -c main.cpp -o main.o    # compile only, do not link
```

If `main.cpp` calls `void foo()` defined in `utils.cpp`, the compiler accepts it — it leaves a placeholder for the linker.

### Step 3 — Linker

Combines `.o` files and libraries into an executable. Resolves all placeholders.

```bash
g++ main.o utils.o -o program
```

**"undefined reference" errors come from here.** The compiler compiled fine. The linker cannot find the definition.

### Step 4 — Executable

The OS loads the executable, maps it into memory, and jumps to `main()`.

### Summary

```
main.cpp  →  [Preprocessor]  →  main.i  →  [Compiler]  →  main.o  ─┐
utils.cpp →  [Preprocessor]  →  utils.i →  [Compiler]  →  utils.o ─┤→ [Linker] → program
```
