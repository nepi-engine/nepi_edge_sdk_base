# This message details the ND node's data saving settings.

# The save_next field indicates that the next acquired data set will be saved to disk, whether or not data
# saving is enabled (via save_continuous field)
bool save_next

# The save_continuous field provides the on/off mechanism for traditional data saving
bool save_continuous

# The save_raw field works in conjunction with the save_continuous field to capture "raw" data alongside
# the standard data product(s). The meanings of "raw" and "standard" here are dependent on the specific
# nd_node
bool save_raw