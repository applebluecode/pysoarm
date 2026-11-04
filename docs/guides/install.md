# Install

## From PyPI

```bash
pip install pysoarm                 # reference backend (pyserial only)
pip install "pysoarm[rustypot]"     # + fast Rust backend
pip install "pysoarm[kinematics]"   # + FK/IK (placo)
```

## From source (development)

```bash
git clone https://github.com/applebluecode/pysoarm && cd pysoarm
make install-dev     # uv sync --locked + pre-commit hooks
make check           # lint + type + test
```

Development dependencies are declared as PEP 735 groups in `pyproject.toml` and pinned by
the committed `uv.lock`, resolved against `pypi.org` (see `[[tool.uv.index]]`). Without uv:

```bash
python -m venv .venv && . .venv/bin/activate
pip install --group dev -e .
```

Only users of the *library* need `pip install pysoarm`; nothing in the published metadata
depends on the dev groups.

## Device access

On Linux, add a udev rule so the arms keep stable names across replugs:

```bash
scripts/install_udev_rules.sh /dev/ttyACM0 /dev/ttyACM1
# → /dev/so101_follower, /dev/so101_leader
```

Alternatively make sure your user can open the tty (`dialout` group, or a 0666 rule).

Also recommended, because it periodically probes serial devices and can make the port
momentarily busy:

```bash
sudo systemctl disable --now ModemManager
```

## Verify

```bash
pysoarm --version
pysoarm doctor --port /dev/so101_follower
pysoarm scan   --port /dev/so101_follower   # ids 1..6, model 777 = STS3215
```
