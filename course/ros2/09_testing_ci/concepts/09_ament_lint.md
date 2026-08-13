# 8. ament_lint

`ament_lint_auto` runs several linting tools automatically:

```bash
ament_cpplint --filter=-whitespace/newline src/
ament_cppcheck src/
ament_uncrustify src/
```

Add to `CMakeLists.txt`:
```cmake
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
```

And to `package.xml`:
```xml
<test_depend>ament_lint_auto</test_depend>
<test_depend>ament_lint_common</test_depend>
```
