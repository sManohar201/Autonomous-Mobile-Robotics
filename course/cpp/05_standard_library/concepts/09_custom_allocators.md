# 5.8 Custom Allocators

Allocators control how containers acquire memory. Most robotics code should not write custom allocators early. First use `reserve`, fixed-capacity containers, and clear ownership boundaries. Allocators matter in real-time systems when dynamic allocation must be bounded or eliminated.
