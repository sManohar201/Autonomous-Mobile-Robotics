# 1.3 Include Guards and `#pragma once`

Without a guard, including `b.hpp` through two paths causes its contents to appear twice — redefinition error.

### Include guards (portable)
```cpp
#ifndef MYPROJECT_B_HPP
#define MYPROJECT_B_HPP
// ... contents ...
#endif
```

### `#pragma once` (simpler, universally supported)
```cpp
#pragma once
// ... contents ...
```

Use `#pragma once` for new code. All major robotics codebases use it.
