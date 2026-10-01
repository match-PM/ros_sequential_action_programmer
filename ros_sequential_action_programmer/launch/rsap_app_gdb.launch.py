"""Launch RSAP under GDB and retain a native backtrace after a crash."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    gdb_prefix = (
        'gdb -q -batch '
        '-ex "set pagination off" '
        '-ex "set print thread-events off" '
        '-ex "set logging file /tmp/rsap-gdb-backtrace.txt" '
        '-ex "set logging overwrite on" '
        '-ex "set logging enabled on" '
        '-ex run '
        '-ex "thread apply all bt full" '
        # ROS 2 Python entry points are scripts, not ELF executables.  Make the
        # interpreter GDB's target and let launch append the entry-point path.
        '--args python3'
    )

    return LaunchDescription([
        Node(
            package='ros_sequential_action_programmer',
            executable='ros_sequential_action_programmer',
            name='RSAP_App',
            emulate_tty=True,
            prefix=gdb_prefix,
            additional_env={
                # Turn silent allocator corruption into an earlier, more useful abort.
                'PYTHONMALLOC': 'debug',
                'MALLOC_CHECK_': '3',
            },
        ),
    ])
