# Per-sector MiDaS floor-drop signal, replacing the JSON String on /vision/floor.
# center is the primary forward-facing signal used by cliff_guard/creep_test/cliff_scan;
# left/right support turning-away decisions. "sharp" fields gate out smooth-floor gradients
# from genuine sharp edges — see docs/navigation-architecture.md G9 (never treat a low reading
# up close as "clear"; MiDaS has a confirmed blind spot right at a real edge).
std_msgs/Header header

float32 left
float32 left_sharp
float32 center
float32 center_sharp
float32 right
float32 right_sharp
