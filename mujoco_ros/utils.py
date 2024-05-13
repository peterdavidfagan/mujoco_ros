from typing import List, Optional

import sys
import traceback
import yaml

from tempfile import NamedTemporaryFile


def create_params_file_from_dict(params, node_name):
    with NamedTemporaryFile(mode="w", prefix="launch_params_", delete=False) as h:
        param_file_path = h.name
        param_dict = {node_name: {"ros__parameters": params}}
        yaml.dump(param_dict, h, default_flow_style=False)
        return param_file_path


def get_launch_params_filepaths(cli_args: Optional[List[str]] = None) -> List[str]:
    """
    A utility that returns the path value after the --params-file arguments.
    """
    if cli_args is None:
        cli_args = sys.argv

    try:
        indexes = [i for i, v in enumerate(cli_args) if v == "--params-file"]
        return [cli_args[i + 1] for i in indexes]
    except IndexError:
        return [
            "Failed to parse params file paths from command line arguments. Check that --params-file command line argument is specified."
        ]
