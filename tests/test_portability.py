#!/usr/bin/env python3
"""Verify ordinary workspaces, path quoting and isolation without host-specific paths."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
from workspace import workspace_path, current_run
from docker_runner import prepare_arguments


class PortableWorkspaceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(dir=os.environ['OCCUPANCY_RUN_DIR'])
        self.root = Path(self.temp.name)
        self.addCleanup(self.temp.cleanup)

    def launch(self, workspace, name='probe', env=None):
        code = "import os,json,pathlib; pathlib.Path('probe.json').write_text(json.dumps(dict(cwd=os.getcwd(),run=os.environ['OCCUPANCY_RUN_DIR'],workspace=os.environ['OCCUPANCY_WORKSPACE'],tmp=os.environ['TMPDIR'],ros=os.environ['ROS_LOG_DIR'])))"
        return subprocess.run([sys.executable, str(ROOT/'tools/run.py'), '--workspace', str(workspace), '--name', name, '--', sys.executable, '-c', code],
                              env=env, capture_output=True, text=True)

    def test_default_uses_current_user(self):
        with patch.dict(os.environ, {}, clear=True), patch('pathlib.Path.home', return_value=self.root/'other user'):
            self.assertEqual(workspace_path(), self.root/'other user/occupancy-slam-workspace')

    def test_non_mount_directory_with_spaces(self):
        workspace = self.root/'ordinary directory'/'output space'
        result = self.launch(workspace)
        self.assertEqual(result.returncode, 0, result.stderr)
        run = workspace/'runs/probe'
        receipt = json.loads((run/'probe.json').read_text())
        self.assertEqual(Path(receipt['cwd']), run)
        self.assertEqual(Path(receipt['workspace']), workspace)
        for key in ('run','tmp','ros'):
            self.assertTrue(Path(receipt[key]).is_relative_to(workspace))
        self.assertFalse(workspace.is_mount())
        again = self.launch(workspace)
        self.assertNotEqual(again.returncode, 0)
        self.assertEqual(json.loads((run/'probe.json').read_text()), receipt)

    def test_explicit_workspace_overrides_environment(self):
        ignored = self.root/'not used'
        selected = self.root/'selected'
        result = self.launch(selected, env=dict(os.environ, OCCUPANCY_WORKSPACE=str(ignored)))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse(ignored.exists())
        self.assertTrue((selected/'runs/probe/probe.json').is_file())

    def test_run_symlink_cannot_escape_workspace(self):
        workspace = self.root/'workspace'; workspace.mkdir()
        outside = self.root/'outside'; outside.mkdir()
        (workspace/'runs').symlink_to(outside, target_is_directory=True)
        result = self.launch(workspace)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('outside the workspace', result.stderr)
        self.assertEqual(list(outside.iterdir()), [])

    def test_current_run_must_match_workspace(self):
        with patch.dict(os.environ, OCCUPANCY_WORKSPACE=str(self.root/'workspace'), OCCUPANCY_RUN_DIR=str(self.root/'different')):
            with self.assertRaises(ValueError): current_run()

    def test_docker_input_and_output_paths(self):
        workspace = self.root/'work space'; run = workspace/'runs/probe'
        caller = self.root/'source tree'; caller.mkdir()
        dataset = caller/'input data'; dataset.mkdir()
        args, inputs = prepare_arguments(['--dataset','input data','--output','pipeline'],caller,workspace,run)
        self.assertEqual(args,['--dataset',str(dataset),'--output','pipeline'])
        self.assertEqual(inputs,[dataset])
        with self.assertRaises(ValueError):
            prepare_arguments(['--output',str(self.root/'outside')],caller,workspace,run)

    def test_explicit_ros_setup_is_not_silently_ignored(self):
        setup = self.root/'missing ROS/setup.bash'
        result = subprocess.run(['bash','-c','source "$1"','test',str(ROOT/'tools/env.sh')],
                                env=dict(os.environ, OCCUPANCY_RUNTIME_SETUP=str(setup)), capture_output=True,text=True)
        self.assertNotEqual(result.returncode,0)
        self.assertIn(str(setup),result.stderr)


if __name__ == '__main__':
    unittest.main()
