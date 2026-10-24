"""Offline installer regression tests. No sudo/network/builds or ROS nodes."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

SCRIPTS = Path(__file__).resolve().parent


class SetupTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="thermoswarm setup ")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        (self.root / "scripts").mkdir()
        for name in ("setup.sh", "setup-versions.env", "setup_verify.py"):
            shutil.copy2(SCRIPTS / name, self.root / "scripts" / name)
        (self.root / "config").mkdir()
        (self.root / "config/fleet.yaml").write_text("fleet: initial\n")
        (self.root / "ros2_ws/src/thermoswarm_control").mkdir(parents=True)
        self.log = self.root / "commands"
        self.env = {**os.environ, "TEST_ROOT": str(self.root), "TEST_LOG": str(self.log),
                    "PYTHONDONTWRITEBYTECODE": "1"}
        # Isolate functions without changing production's paths/platform checks.
        self.prelude = '''
source "$TEST_ROOT/scripts/setup.sh"
sudo() { echo "UNEXPECTED sudo $*" >> "$TEST_LOG"; return 99; }
pgrep() { return 1; }
'''

    def bash(self, body, ok=True):
        result = subprocess.run(["bash", "-c", self.prelude + body], env=self.env,
                                text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        if ok:
            self.assertEqual(result.returncode, 0, result.stdout)
        else:
            self.assertNotEqual(result.returncode, 0, result.stdout)
        return result.stdout

    def init_repo(self, name="PX4-Autopilot"):
        repo = self.root / name
        subprocess.run(["git", "init", "-q", str(repo)], check=True)
        (repo / "source.txt").write_text("keep me\n")
        subprocess.run(["git", "-C", str(repo), "add", "source.txt"], check=True)
        subprocess.run(["git", "-C", str(repo), "-c", "user.name=Test", "-c",
                        "user.email=test@example.invalid", "commit", "-qm", "fixture"], check=True)
        ref = subprocess.check_output(["git", "-C", str(repo), "rev-parse", "HEAD"], text=True).strip()
        return repo, ref

    def test_help_and_bad_options(self):
        for args, expected in ((["--help"], 0), (["--unknown"], 1),
                               (["--jobs", "0"], 1), (["--plan", "--install"], 1)):
            result = subprocess.run(["bash", str(self.root / "scripts/setup.sh"), *args],
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, expected, result.stdout + result.stderr)
        self.assertFalse((self.root / ".runtime").exists())

    def test_default_plan_and_repeat_are_read_only(self):
        body = 'MODE=plan; main; main'
        output = self.bash(body)
        self.assertIn("CREATE source", output)
        self.assertFalse((self.root / ".runtime").exists())
        self.assertFalse(self.log.exists())
        self.assertFalse((self.root / "PX4-Autopilot").exists())

    def test_missing_dependencies_fail_before_any_mutation(self):
        output = self.bash('''
installed() { return 1; }
agent_command() { return 1; }
MODE=install; YES=1; main
''', ok=False)
        self.assertIn("Missing dependencies", output)
        self.assertFalse((self.root / ".runtime").exists())
        self.assertFalse(self.log.exists())

    def test_noninteractive_install_requires_explicit_local_consent(self):
        output = self.bash('''
installed() { return 0; }
agent_command() { return 1; }
MODE=install; main
''', ok=False)
        self.assertIn("No interactive terminal", output)
        self.assertFalse((self.root / ".runtime").exists())

    def test_check_reports_missing_stack_and_creates_nothing(self):
        output = self.bash('''
agent_ok() { return 1; }
ros_check() { echo 'FAIL mocked missing ROS'; return 1; }
installed() { return 1; }
MODE=check; main
''', ok=False)
        for message in ("MISSING source", "FAIL PX4 SITL binary", "FAIL Agent", "FAIL .venv-ros", "missing build prerequisites"):
            self.assertIn(message, output)
        self.assertFalse((self.root / ".runtime").exists())
        self.assertFalse(self.log.exists())

    def test_wrong_revision_preserves_checkout(self):
        repo, ref = self.init_repo()
        output = self.bash('repo_ok "$PX4" deadbeef', ok=False)
        self.assertIn("FAIL revision", output)
        self.assertEqual(subprocess.check_output(["git", "-C", str(repo), "rev-parse", "HEAD"], text=True).strip(), ref)
        self.assertEqual((repo / "source.txt").read_text(), "keep me\n")

    def test_local_edits_and_untracked_files_preserved(self):
        repo, ref = self.init_repo()
        (repo / "source.txt").write_text("user change\n")
        (repo / "user.cfg").write_text("local parameters\n")
        self.bash(f'repo_ok "$PX4" {ref}', ok=False)
        self.assertEqual((repo / "source.txt").read_text(), "user change\n")
        self.assertEqual((repo / "user.cfg").read_text(), "local parameters\n")

    def test_existing_repo_rerun_never_fetches_or_switches(self):
        _, ref = self.init_repo()
        self.bash(f'clone_pinned "$PX4" invalid://must-not-fetch {ref}\nclone_pinned "$PX4" invalid://must-not-fetch {ref}')

    def test_non_repository_nested_in_project_is_rejected(self):
        self.init_repo("outer")
        nested = self.root / "outer/nested"
        nested.mkdir()
        self.bash('repo_ok "$ROOT/outer/nested" deadbeef', ok=False)

    def test_apt_upgrade_and_removal_abort_before_sudo(self):
        for line in ("Inst existing [1.0] (2.0 Ubuntu)", "Remv existing [1.0]"):
            output = self.bash(f"apt-get() {{ echo '{line}'; }}\napt_transaction missing", ok=False)
            self.assertIn("would change/remove", output)
            self.assertFalse(self.log.exists())

    def test_apt_missing_package_is_simulated_then_approved_transaction(self):
        self.bash('''
apt-get() { echo "Inst missing (1.0 Ubuntu)"; }
sudo() { printf '%s\n' "$*" >> "$TEST_LOG"; }
apt_transaction missing
''')
        command = self.log.read_text()
        self.assertIn("apt-get --no-remove --no-upgrade install missing", command)
        self.assertNotIn("-y", command)

    def test_installed_system_dependencies_skip_all_apt_operations(self):
        self.bash('MISSING=(); system_setup')
        self.assertFalse(self.log.exists())

    def test_existing_px4_binary_and_parameters_untouched(self):
        binary = self.root / "PX4-Autopilot/build/px4_sitl_default/bin/px4"
        binary.parent.mkdir(parents=True)
        binary.write_text("#!/bin/sh\nexit 99\n")
        binary.chmod(0o755)
        params = binary.parent.parent / "parameters.bson"
        params.write_bytes(b"local flight parameters")
        self.bash('make() { exit 99; }; build_px4; build_px4')
        self.assertEqual(params.read_bytes(), b"local flight parameters")
        self.assertFalse((self.root / ".venv-px4").exists())

    def test_partial_px4_cache_is_not_overwritten(self):
        cache = self.root / "PX4-Autopilot/build/px4_sitl_default/CMakeCache.txt"
        cache.parent.mkdir(parents=True)
        cache.write_text("user cache\n")
        output = self.bash("build_px4", ok=False)
        self.assertIn("Partial PX4 build cache", output)
        self.assertEqual(cache.read_text(), "user cache\n")

    def test_px4_rootfs_without_cache_is_preserved(self):
        params = self.root / "PX4-Autopilot/build/px4_sitl_default/rootfs/parameters.bson"
        params.parent.mkdir(parents=True)
        params.write_bytes(b"user parameters without a build cache")
        self.bash("build_px4", ok=False)
        self.assertEqual(params.read_bytes(), b"user parameters without a build cache")

    def test_new_local_clone_fetches_pinned_release_tag(self):
        repo, ref = self.init_repo("fixture-origin")
        subprocess.run(["git", "-C", str(repo), "tag", "v1.17.0"], check=True)
        self.bash(f'clone_pinned "$PX4" "$ROOT/fixture-origin" {ref} v1.17.0')
        result = subprocess.check_output(["git", "-C", str(self.root / "PX4-Autopilot"),
                                          "describe", "--tags", "--exact-match"], text=True)
        self.assertEqual(result.strip(), "v1.17.0")

    def test_moved_release_tag_fails_before_checkout(self):
        repo, ref = self.init_repo("fixture-origin")
        (repo / "source.txt").write_text("moved tag\n")
        subprocess.run(["git", "-C", str(repo), "-c", "user.name=Test", "-c",
                        "user.email=test@example.invalid", "commit", "-am", "second"],
                       check=True, capture_output=True)
        subprocess.run(["git", "-C", str(repo), "tag", "v1.17.0"], check=True)
        output = self.bash(f'clone_pinned "$PX4" "$ROOT/fixture-origin" {ref} v1.17.0', ok=False)
        self.assertIn("no longer matches pinned", output)
        self.assertFalse((self.root / "PX4-Autopilot/source.txt").exists())

    def test_incompatible_agent_never_replaced(self):
        output = self.bash('''
agent_ok() { return 1; }
agent_command() { echo /usr/local/bin/MicroXRCEAgent; }
build_agent
''', ok=False)
        self.assertIn("preserved", output)
        self.assertFalse((self.root / ".runtime").exists())

    def test_unmanaged_px4_venv_is_preserved(self):
        req = self.root / "PX4-Autopilot/Tools/setup/requirements.txt"
        req.parent.mkdir(parents=True)
        req.write_text("empy\n")
        (self.root / "scripts/requirements-px4-constraints.txt").write_text("empy==3.3.4\n")
        venv = self.root / ".venv-px4"
        venv.mkdir()
        marker = venv / "user-file"
        marker.write_text("keep")
        output = self.bash("build_px4", ok=False)
        self.assertIn("Unmanaged/partial .venv-px4", output)
        self.assertEqual(marker.read_text(), "keep")

    def test_active_px4_blocks_install_without_stopping_process(self):
        output = self.bash('''
installed() { return 0; }
pgrep() { return 0; }
MODE=install; YES=1; main
''', ok=False)
        self.assertIn("PX4 is running", output)
        self.assertFalse((self.root / ".runtime").exists())

    def test_cmake_component_rerun_skips_build(self):
        self.bash('''
mkdir -p "$STATE/agent" "$PREFIX/lib"
cmake() { echo "$*" >> "$TEST_LOG"; }
cmake_install fixture "$ROOT" abc -DBUILD_TESTING=OFF
cmake_install fixture "$ROOT" abc -DBUILD_TESTING=OFF
''')
        self.assertEqual(len(self.log.read_text().splitlines()), 3)

    def test_workspace_repeat_skips_and_changed_config_rebuilds(self):
        output = self.bash('''
mkdir -p "$STATE"
colcon_build() { echo build >> "$TEST_LOG"; }
ros_check() { return 0; }
build_workspace
build_workspace
echo 'fleet: changed' > "$ROOT/config/fleet.yaml"
build_workspace
''')
        self.assertEqual(self.log.read_text().splitlines(), ["build", "build"])
        self.assertIn("KEEP ROS workspace", output)

    def test_failed_build_cannot_write_success_marker(self):
        self.bash('''
mkdir -p "$STATE"
colcon_build() { return 42; }
ros_check() { return 0; }
build_workspace
''', ok=False)
        self.assertFalse((self.root / ".runtime/setup/workspace.sha256").exists())

    def test_failed_installed_verification_cannot_write_success_marker(self):
        self.bash('''
mkdir -p "$STATE"
colcon_build() { return 0; }
ros_check() { return 1; }
build_workspace
''', ok=False)
        self.assertFalse((self.root / ".runtime/setup/workspace.sha256").exists())


if __name__ == "__main__":
    unittest.main(verbosity=2)
