#include <cassert>
#include <cctype>
#include <iostream>
#include <string>
#include <string_view>

// Note on string_view lifetime: a string_view is a non-owning view. Returning
// string_view to a temporary local std::string is dangling — the string is
// destroyed when the function returns, leaving the view pointing at freed memory.
// string_view is safe to return only when the pointed-to data outlives the caller.

bool starts_with(std::string_view text, std::string_view prefix) {
    return text.size() >= prefix.size() &&
           text.substr(0, prefix.size()) == prefix;
}

std::string_view trim(std::string_view text) {
    while (!text.empty() && std::isspace(static_cast<unsigned char>(text.front())))
        text.remove_prefix(1);
    while (!text.empty() && std::isspace(static_cast<unsigned char>(text.back())))
        text.remove_suffix(1);
    return text;
}

int main() {
    assert(starts_with("/imu/data", "/imu"));
    assert(!starts_with("/gps/fix", "/imu"));
    assert(trim("  base_link \n") == "base_link");
    std::cout << "ex04_string_view passed\n";
}

