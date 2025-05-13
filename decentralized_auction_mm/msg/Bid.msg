# Bid.msg - Representation of a bid in the auction
std_msgs/Header header

uint32 robot_id            # ID of the bidding robot
uint32 task_id             # ID of the task being bid on
float64 bid_value          # Value of the bid
float64 utility            # Utility (bid_value - current_price)