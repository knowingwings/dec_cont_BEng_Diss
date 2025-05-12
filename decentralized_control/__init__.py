"""
Decentralized Control Package for dual mobile manipulators.
"""

try:
    # Import ROS2 generated messages
    from dec_control.msg._bid import Bid  # noqa: F401
    from dec_control.msg._capability_update import CapabilityUpdate  # noqa: F401
    from dec_control.msg._collaboration_request import CollaborationRequest  # noqa: F401
    from dec_control.msg._collaboration_response import CollaborationResponse  # noqa: F401
    from dec_control.msg._heartbeat import Heartbeat  # noqa: F401
    from dec_control.msg._network_topology import NetworkTopology  # noqa: F401
    from dec_control.msg._recovery_status import RecoveryStatus  # noqa: F401
    from dec_control.msg._robot_state import RobotState  # noqa: F401
    from dec_control.msg._synchronization_signal import SynchronizationSignal  # noqa: F401
    from dec_control.msg._task import Task  # noqa: F401
    from dec_control.msg._task_assignment import TaskAssignment  # noqa: F401
    from dec_control.msg._task_list import TaskList  # noqa: F401
    from dec_control.msg._task_priority import TaskPriority  # noqa: F401

    # Core node imports
    from .auction.auction_node import AuctionNode  # noqa: F401
    from .consensus.consensus_node import ConsensusNode  # noqa: F401
    from .task_manager.task_manager_node import TaskManagerNode  # noqa: F401
    from .recovery.recovery_node import RecoveryNode  # noqa: F401
    from .execution.execution_controller import ExecutionController  # noqa: F401

except ImportError as e:
    import warnings
    warnings.warn(f'Error importing ROS modules: {str(e)}')

__version__ = '0.1.0'