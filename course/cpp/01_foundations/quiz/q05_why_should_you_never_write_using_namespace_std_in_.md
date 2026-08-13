# **Q5.** Why should you never write `using namespace std;` in a header file?

<details>
<summary>Answer</summary>

`#include` pastes the header into every file that includes it. The `using namespace` then silently applies to all those translation units, potentially pulling in names that clash with names in those files or other headers. This can cause ambiguous overloads or silent changes in which function gets called.

</details>
