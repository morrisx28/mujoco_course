"""Pineapple V0 velocity task registration.

Registration builds the environment config at import time, and mjlab imports every task when
``train``, ``play`` or ``list-envs`` starts. An unfinished env_cfgs.py would therefore break
every task, so the registration below stays commented out until your six functions are done.
Then uncomment it and check with ``uv run list-envs``.
"""

# from mjlab.tasks.registry import register_mjlab_task
# from mjlab.tasks.velocity.rl import VelocityOnPolicyRunner
#
# from .env_cfgs import pineapple_v0_flat_env_cfg
# from .rl_cfg import pineapple_v0_ppo_runner_cfg
#
# register_mjlab_task(
#   task_id="Mjlab-Velocity-Flat-Pineapple-V0",
#   env_cfg=pineapple_v0_flat_env_cfg(),
#   play_env_cfg=pineapple_v0_flat_env_cfg(play=True),
#   rl_cfg=pineapple_v0_ppo_runner_cfg(),
#   runner_cls=VelocityOnPolicyRunner,
# )
