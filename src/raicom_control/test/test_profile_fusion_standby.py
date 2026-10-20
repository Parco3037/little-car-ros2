"""Behavioral and structural checks for the bounded STANDBY profiler."""
import ast
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
PROFILE = ROOT/'deploy/profile_fusion_standby.py'
RUNTIME = ROOT/'deploy/continuous_fusion_runtime.py'
sys.path.insert(0, str(ROOT/'deploy'))
import fusion_runtime_contract as contract
import profile_fusion_standby as profile


class StandbyProfilerSafetyTests(unittest.TestCase):
    def write_readers(self, directory):
        directory.mkdir()
        for name in ('verify_stationary_localization.py', 'verify_scan_map_alignment.py'):
            (directory/name).write_text(
                "def main():\n    return 'subscription-only fixture'\n",
                encoding='utf-8')

    def argv(self, root, readers, *extra):
        return [
            '--output', str(root/'profile.json'), '--seconds', '70',
            '--helper-delay', '35', '--readers', str(readers),
            '--directory', str(root/'runtime'), '--session', 'profile-fixture-abcdef',
            '--parent-pid', '100', '--parent-start', 'parent-start',
            '--old-pid', '200', '--old-start', 'old-start',
            '--main-cpu', '2', '--child-cpu', '3', *extra]

    def parse(self, argv):
        with patch.dict(os.environ, {'FASTDDS_BUILTIN_TRANSPORTS': 'UDPv4'}):
            return profile.parse_arguments(argv)

    def test_profiler_parser_forces_single_standby_flag(self):
        with tempfile.TemporaryDirectory() as raw:
            root = Path(raw)
            readers = root/'readers'
            self.write_readers(readers)
            args, runtime_args, runtime_argv, paths, hashes = self.parse(
                self.argv(root, readers))
            self.assertEqual(runtime_argv.count('--standby-only'), 1)
            self.assertEqual(runtime_argv[-1], '--standby-only')
            self.assertFalse(runtime_args.reset_origin)
            self.assertFalse(runtime_args.standby_only)
            self.assertEqual(args.helper_cpu, 1)
            self.assertEqual({p.name for p in paths}, set(hashes))
            self.assertFalse(args.output.exists())

    def test_profiler_rejects_caller_standby_but_allows_explicit_missing_old(self):
        with tempfile.TemporaryDirectory() as raw:
            root = Path(raw)
            readers = root/'readers'
            self.write_readers(readers)
            with self.assertRaises(SystemExit):
                self.parse(self.argv(root, readers, '--standby-only'))
            _, runtime_args, runtime_argv, _, _ = self.parse(
                self.argv(root, readers, '--reset-origin'))
            self.assertTrue(runtime_args.reset_origin)
            self.assertIn('--reset-origin', runtime_argv)
            self.assertIn('--standby-only', runtime_argv)

    def test_profiler_rejects_default_broken_shm_transport(self):
        with tempfile.TemporaryDirectory() as raw:
            root = Path(raw)
            readers = root/'readers'
            self.write_readers(readers)
            with patch.dict(os.environ, {}, clear=True), self.assertRaises(SystemExit):
                profile.parse_arguments(self.argv(root, readers))

    def test_actual_runtime_contract_rejects_activation_file(self):
        with tempfile.TemporaryDirectory() as raw:
            directory = Path(raw)
            contract.enforce_standby_contract(
                standby_only=True, directory=directory,
                observed_old_start='old-start', expected_old_start='old-start')
            (directory/'activate.json').write_text('{}', encoding='utf-8')
            with self.assertRaisesRegex(RuntimeError, 'Activation token forbidden'):
                contract.enforce_standby_contract(
                    standby_only=True, directory=directory,
                    observed_old_start='old-start', expected_old_start='old-start')
            self.assertFalse(contract.activation_requested(
                standby_only=True, mode='STANDBY', ready_elapsed=999,
                token_path=directory/'activate.json'))

    def test_actual_runtime_contract_rejects_old_process_change(self):
        with tempfile.TemporaryDirectory() as raw:
            with self.assertRaisesRegex(RuntimeError, 'old odometry process changed'):
                contract.enforce_standby_contract(
                    standby_only=True, directory=Path(raw),
                    observed_old_start=None, expected_old_start='old-start')
            contract.enforce_standby_contract(
                standby_only=True, directory=Path(raw),
                observed_old_start=None, expected_old_start=None)

    def test_runtime_contract_requires_distinct_nonnegative_cpu_pair(self):
        base = [
            '--directory', 'runtime', '--session', 'profile-fixture-abcdef',
            '--parent-pid', '100', '--parent-start', 'parent-start',
            '--old-pid', '200', '--old-start', 'old-start',
        ]
        parser = contract.build_argument_parser()
        for extra in ([], ['--main-cpu', '2', '--child-cpu', '2'],
                      ['--main-cpu', '-1', '--child-cpu', '3']):
            with self.subTest(extra=extra), self.assertRaises(SystemExit):
                contract.validate_arguments(parser.parse_args([*base, *extra]), parser)
        args = contract.validate_arguments(
            parser.parse_args([*base, '--main-cpu', '2', '--child-cpu', '3']), parser)
        self.assertEqual((args.main_cpu, args.child_cpu), (2, 3))
        self.assertEqual(contract.derive_affinity_roles(4, 2, 3), (0, 1))
        with self.assertRaisesRegex(ValueError, 'Four CPUs'):
            contract.derive_affinity_roles(3, 1, 2)

    def test_activation_predicate_requires_every_non_standby_gate(self):
        with tempfile.TemporaryDirectory() as raw:
            token = Path(raw)/'activate.json'
            token.write_text('{}', encoding='utf-8')
            self.assertTrue(contract.activation_requested(
                standby_only=False, mode='STANDBY', ready_elapsed=16,
                token_path=token))
            cases = [
                dict(standby_only=True, mode='STANDBY', ready_elapsed=16),
                dict(standby_only=False, mode='ACTIVE', ready_elapsed=16),
                dict(standby_only=False, mode='STANDBY', ready_elapsed=15),
            ]
            for case in cases:
                with self.subTest(case=case):
                    self.assertFalse(contract.activation_requested(token_path=token, **case))
            token.unlink()
            self.assertFalse(contract.activation_requested(
                standby_only=False, mode='STANDBY', ready_elapsed=16,
                token_path=token))

    def test_initialization_timeout_never_confuses_paused_with_waiting(self):
        self.assertFalse(contract.initialization_timed_out(
            ready_elapsed=5.0, gate_state='WAITING'))
        self.assertTrue(contract.initialization_timed_out(
            ready_elapsed=5.001, gate_state='WAITING'))
        for state in ('VALID', 'PAUSED', 'LOST'):
            self.assertFalse(contract.initialization_timed_out(
                ready_elapsed=999.0, gate_state=state))
        runtime = RUNTIME.read_text(encoding='utf-8')
        self.assertNotIn('not gate.established', runtime)
        self.assertIn('initialization_timed_out(', runtime)

    def test_profiler_has_no_motion_or_activation_api(self):
        tree = ast.parse(PROFILE.read_text(encoding='utf-8'))
        calls = {
            node.func.attr for node in ast.walk(tree)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
        }
        self.assertFalse(calls & {
            'create_publisher', 'create_client', 'create_service',
            'send_goal_async', 'call_async'
        })
        constants = {
            node.value for node in ast.walk(tree)
            if isinstance(node, ast.Constant) and isinstance(node.value, str)
        }
        self.assertEqual(
            sum(node.value == 'activate.json' for node in ast.walk(tree)
                if isinstance(node, ast.Constant)), 1)
        self.assertIn(
            "activation_file_exists=(runtime_args.directory/'activate.json').exists()",
            PROFILE.read_text(encoding='utf-8'))

    def test_helpers_are_statically_restricted_and_robustly_reaped(self):
        source = PROFILE.read_text(encoding='utf-8')
        self.assertIn('FORBIDDEN_READER_CALLS', source)
        self.assertIn('runtime.stop_child(child)', source)
        self.assertIn("command = ['/usr/bin/taskset', '-c', str(args.helper_cpu)", source)
        self.assertNotIn('child.wait(timeout=20)', source)

    def test_runtime_standby_only_blocks_activation_and_old_process_change(self):
        tree = ast.parse(RUNTIME.read_text(encoding='utf-8'))
        source = ast.unparse(tree)
        publishers = [
            node for node in ast.walk(tree)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
            and node.func.attr == 'create_publisher' and len(node.args) > 1
            and isinstance(node.args[1], ast.Constant) and node.args[1].value in ('/odom', '/tf')
        ]
        self.assertEqual(len(publishers), 2)
        self.assertIn('activation_requested', source)
        self.assertIn('enforce_standby_contract', source)

    def test_profiler_observes_process_results_without_reintroducing_shared_tracker(self):
        source = PROFILE.read_text(encoding='utf-8')
        self.assertIn('original = runtime.observe_match_result', source)
        self.assertIn('runtime.observe_match_result = measured', source)
        self.assertIn("wall_ms=response['worker_wall_ms']", source)
        self.assertIn('ipc_supervisor_overhead_ms=max(', source)
        self.assertNotIn('runtime.ScanTracker.update', source)

    def test_runtime_applies_and_verifies_split_affinity_before_child_use(self):
        source = RUNTIME.read_text(encoding='utf-8')
        self.assertLess(source.index('os.sched_setaffinity(0, {receiver_cpu})'),
                        source.index('    import rclpy'))
        self.assertLess(source.index("multiprocessing.get_context('fork')"),
                        source.index('    import rclpy'))
        self.assertLess(source.index('matcher.start()'), source.index('    import rclpy'))
        self.assertGreater(source.index('os.sched_setaffinity(0, {diagnostic_cpu})'),
                           source.index('subs = [node.create_subscription'))
        self.assertIn("state['receiver_affinity'] = sorted(os.sched_getaffinity(0))", source)
        self.assertIn("command = ['/usr/bin/taskset', '-c', str(args.child_cpu)", source)
        self.assertIn("child_affinity = sorted(os.sched_getaffinity(child.pid))", source)
        self.assertIn("log('child_affinity', pid=child.pid, affinity=child_affinity,", source)
        self.assertIn('matcher_scheduler = os.sched_getscheduler(matcher.pid)', source)
        self.assertIn("log('matcher_process', pid=matcher.pid, affinity=matcher_affinity,", source)
        self.assertNotIn('os.sched_setscheduler(0, os.SCHED_RR', source)
        worker = (ROOT/'deploy/scan_match_worker.py').read_text(encoding='utf-8')
        self.assertIn('os.sched_setaffinity(0, {cpu})', worker)
        self.assertIn('os.sched_setscheduler(0, os.SCHED_RR, os.sched_param(1))', worker)
        self.assertIn('os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))', worker)


if __name__ == '__main__':
    unittest.main()
