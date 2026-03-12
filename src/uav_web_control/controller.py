import logging
import json
from pathlib import Path

log = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)

CMD_LOG = Path(__file__).parent / 'last_command.json'


def fly_to(uav_id, position):
    """Placeholder function to send a fly command to a UAV.

    In a real system this would publish a MAVLink/ROS message or call
    into PX4/mission APIs. For now this writes a JSON file and logs the command.
    """
    cmd = {'uav_id': uav_id, 'target': position}
    log.info('Fly command: %s', cmd)
    with open(CMD_LOG, 'w') as f:
        json.dump(cmd, f)
    return True
