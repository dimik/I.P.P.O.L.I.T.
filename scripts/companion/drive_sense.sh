#!/bin/bash
# prints: "<center_drop> <front_clearance_m> <left_clear_m> <right_clear_m>"  (NA on failure)
source /opt/ros/jazzy/setup.bash; export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
C=$(timeout 3 ros2 topic echo /vision/floor --field data 2>/dev/null | grep -m1 '^{' \
    | python3 -c 'import sys,json;print(round(json.loads(sys.stdin.readline())["sectors"]["center"][0],3))' 2>/dev/null)
read F L R < <(timeout 3 ros2 topic echo --once /scan 2>/dev/null | python3 -c '
import sys,re,math
t=sys.stdin.read()
def g(k):
 m=re.search(k+r": (-?[0-9.eE+-]+)",t); return float(m.group(1)) if m else None
amin=g("angle_min"); ainc=g("angle_increment")
seg=t.split("ranges:")[1].split("intensities:")[0] if "ranges:" in t else ""
rng=[math.inf if "inf" in x else float(x) for x in re.findall(r"-?\d+\.\d+(?:e[+-]?\d+)?|\.inf|inf",seg)]
sec={"F":0,"L":60,"R":-60}; mn={k:99 for k in sec}
for i,r in enumerate(rng):
 if not math.isfinite(r) or r<=0.05: continue
 a=(math.degrees(amin+i*ainc)+180)%360-180
 for n,c in sec.items():
  if abs(((a-c+180)%360)-180)<=25 and r<mn[n]: mn[n]=r
print(round(mn["F"],2),round(mn["L"],2),round(mn["R"],2))' 2>/dev/null)
echo "${C:-NA} ${F:-NA} ${L:-NA} ${R:-NA}"
