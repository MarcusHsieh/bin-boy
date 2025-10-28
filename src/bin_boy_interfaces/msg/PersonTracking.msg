# Single person tracking data

# Unique persistent ID for this person
uint32 id

# Detection confidence (0.0 - 1.0)
float32 confidence

# Bounding box (x1, y1, x2, y2)
float32 x1
float32 y1
float32 x2
float32 y2

# Color histogram similarity to target (0.0 - 1.0)
# -1.0 if not computed
float32 color_similarity

# Combined tracking score (0.0 - 1.0)
# Weighted combination of confidence and color similarity
# -1.0 if not computed
float32 match_score

# Is this the current tracking target?
bool is_target

# Estimated distance to person (meters)
# -1.0 if cannot estimate
float32 distance

# Estimated angle to person (radians)
# Positive = right, negative = left
# 0.0 if cannot estimate
float32 angle
