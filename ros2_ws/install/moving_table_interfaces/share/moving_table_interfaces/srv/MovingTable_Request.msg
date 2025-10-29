# Request: Specify which table and the desired movement parameters

string table_id      # Add this field: Target table ("table1" or "table2")
float32 distance_mm  # Target linear position (if applicable for operation_type)
float32 angle_deg    # Target angular position (if applicable for operation_type)
int32 linear_speed   # Speed for linear movement
int32 rotate_speed   # Speed for rotational movement
int32 operation_type # Defines which specific action to perform (e.g., move linear, move rotate)

