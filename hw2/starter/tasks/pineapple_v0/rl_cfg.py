"""PPO settings for the Pineapple V0 velocity task (given; you may tune them)."""

from mjlab.rl import RslRlModelCfg, RslRlOnPolicyRunnerCfg, RslRlPpoAlgorithmCfg


def pineapple_v0_ppo_runner_cfg() -> RslRlOnPolicyRunnerCfg:
  """Same PPO as mjlab's Go1 velocity task, with a smaller network and fewer iterations."""
  return RslRlOnPolicyRunnerCfg(
    actor=RslRlModelCfg(
      hidden_dims=(256, 128, 64),
      activation="elu",
      obs_normalization=True,
      distribution_cfg={"class_name": "GaussianDistribution", "init_std": 1.0, "std_type": "scalar"},
    ),
    critic=RslRlModelCfg(hidden_dims=(256, 128, 64), activation="elu", obs_normalization=True),
    algorithm=RslRlPpoAlgorithmCfg(
      value_loss_coef=1.0,
      use_clipped_value_loss=True,
      clip_param=0.2,
      entropy_coef=0.01,
      num_learning_epochs=5,
      num_mini_batches=4,
      learning_rate=1.0e-3,
      schedule="adaptive",
      gamma=0.99,
      lam=0.95,
      desired_kl=0.01,
      max_grad_norm=1.0,
    ),
    experiment_name="pineapple_v0_velocity",
    logger="tensorboard",  # no account needed; use --agent.logger wandb if you prefer W&B
    save_interval=100,
    num_steps_per_env=24,
    max_iterations=3_000,
  )
