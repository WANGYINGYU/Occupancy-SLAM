"""Flat YAML defaults for pipeline CLIs; explicit command-line options win."""
import argparse
import os
from pathlib import Path
import shutil
import yaml


class UniqueLoader(yaml.SafeLoader): pass

def unique_mapping(loader,node):
    result={}
    for key_node,value_node in node.value:
        key=loader.construct_object(key_node)
        if not isinstance(key,str) or key in result:raise ValueError(f'Invalid/duplicate configuration key: {key!r}')
        result[key]=loader.construct_object(value_node)
    return result
UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,unique_mapping)

def parse_config(parser, argv=None, path_keys=()):
    caller = Path(os.environ.get('OCCUPANCY_INPUT_CALLER_DIR', Path.cwd()))
    probe = argparse.ArgumentParser(add_help=False)
    probe.add_argument('--config', type=Path)
    config_path = probe.parse_known_args(argv)[0].config
    defaults = {}
    if config_path:
        config_path = (caller/config_path.expanduser()).resolve()
        loaded = yaml.load(config_path.read_text(),Loader=UniqueLoader)
        if not isinstance(loaded, dict):
            raise ValueError('Input config must be a YAML mapping')
        actions = {a.dest:a for a in parser._actions if a.dest not in ('help','config')}
        unknown = loaded.keys()-actions.keys()
        if unknown:
            raise ValueError(f'Unknown input config keys: {sorted(unknown)}')
        for key,value in loaded.items():
            if value is None:
                continue
            action = actions[key]
            cast = action.type or str
            if isinstance(value,(dict,list,bool)) or (cast is int and not isinstance(value,int)):
                raise ValueError(f'Invalid config value for {key}: {value!r}')
            try: value = cast(value)
            except (ValueError,TypeError) as exc: raise ValueError(f'Invalid config value for {key}') from exc
            if action.choices and value not in action.choices:
                raise ValueError(f'Invalid {key}: choose from {action.choices}')
            if key in path_keys:
                value = (config_path.parent/Path(value).expanduser()).resolve()
            defaults[key] = value
            action.required = False
        parser.set_defaults(**defaults)
    args = parser.parse_args(argv)
    args.config = config_path
    for key in path_keys:
        value = getattr(args,key)
        if value is not None:
            setattr(args,key,(caller/Path(value).expanduser()).resolve())
    return args


def copy_input_config(args, output):
    if args.config:
        shutil.copyfile(args.config,output/'input_config.yaml')
