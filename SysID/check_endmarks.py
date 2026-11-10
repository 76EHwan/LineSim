import sys
sys.path.insert(0, 'tools')
import json
import course

j = json.load(open('tracks/practice_board.json'))
ps, end, total = course.pieces(j['segments'])
# Find the closing straight and first straight
print("Segment 0:", ps[0])
print("Segment 1:", ps[1])
print("Segment -1:", ps[-1])
print("Total length:", total)

b_goal, a_goal = course.straight_run(ps, j['goal_s'], total)
b_start, a_start = course.straight_run(ps, j['start_s'], total)
print(f"Goal  (s={j['goal_s']:.4f}): before={b_goal:.3f}m, after={a_goal:.3f}m")
print(f"Start (s={j['start_s']:.4f}): before={b_start:.3f}m, after={a_start:.3f}m")
