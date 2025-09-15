# Frontier.msg - Custom message for Frontier structure

# Normal vector components
float32 normal_x
float32 normal_y
float32 normal_z

# Position coordinates
float32 x
float32 y
float32 z

# Free space flag
bool free

# Start duration
duration start

# Cluster number
int32 cluster_number
int32 max_cluster_count


# Cluster centroid coordinates
float32 cluster_centroid_x
float32 cluster_centroid_y
float32 cluster_centroid_z