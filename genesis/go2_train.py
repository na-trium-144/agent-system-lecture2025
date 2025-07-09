import argparse
import os
import pickle
import shutil
import math
import yaml
import torch

from go2_env import Go2Env
from rsl_rl.runners import OnPolicyRunner

import genesis as gs


def get_train_cfg(exp_name, max_iterations):

    train_cfg_dict = {
        "algorithm": {
            "clip_param": 0.2,
            "desired_kl": 0.01,
            "entropy_coef": 0.01,
            "gamma": 0.99,
            "lam": 0.95,
            "learning_rate": 0.001,
            "max_grad_norm": 1.0,
            "num_learning_epochs": 5,
            "num_mini_batches": 4,
            "schedule": "adaptive",
            "use_clipped_value_loss": True,
            "value_loss_coef": 1.0,
        },
        "init_member_classes": {},
        "policy": {
            "activation": "elu",
            "actor_hidden_dims": [512, 256, 128],
            "critic_hidden_dims": [512, 256, 128],
            "init_noise_std": 1.0,
        },
        "runner": {
            "algorithm_class_name": "PPO",
            "checkpoint": -1,
            "experiment_name": exp_name,
            "load_run": -1,
            "log_interval": 1,
            "max_iterations": max_iterations,
            "num_steps_per_env": 24,
            "policy_class_name": "ActorCritic",
            "record_interval": -1,
            "resume": False,
            "resume_path": None,
            "run_name": "",
            "runner_class_name": "runner_class_name",
            "save_interval": 10,
        },
        "runner_class_name": "OnPolicyRunner",
        "seed": 1,
    }

    return train_cfg_dict


def get_cfgs(exp_name: str):
    default_joint_angles = {}
    dof_names = []
    pd_default_joint_angles = {}
    pd_dof_names = []
    for i in range(4):
        default_joint_angles[f"leg{i}_joint_1"] = -math.pi / 6
        default_joint_angles[f"leg{i}_joint_2"] = math.pi / 3
        dof_names += [f"leg{i}_joint_1", f"leg{i}_joint_2"]
    if exp_name == "walking_only":
        pd_default_joint_angles[f"longleg_joint_0"] = math.pi / 2
        pd_dof_names += [f"longleg_joint_0"]
        for i in range(1, 12):
            pd_default_joint_angles[f"longleg_joint_{i}"] = -math.pi if i % 2 == 1 else math.pi
            pd_dof_names += [f"longleg_joint_{i}"]
    else:
        default_joint_angles[f"longleg_joint_0"] = math.pi / 2
        dof_names += [f"longleg_joint_0"]
        for i in range(1, 12):
            default_joint_angles[f"longleg_joint_{i}"] = -math.pi if i % 2 == 1 else math.pi
            dof_names += [f"longleg_joint_{i}"]
    env_cfg = {
        "num_actions": len(dof_names),
        # joint/link names
        "default_joint_angles": default_joint_angles,
        "dof_names": dof_names,

        "jump": "jump" in exp_name,
        "walking_only": exp_name == "walking_only",
        "obj": exp_name == "walking_only" and False,
        "pd_default_joint_angles": pd_default_joint_angles,
        "pd_dof_names": pd_dof_names,
        # PD
        "kp": 20.0,
        "kd": 0.5,
        # termination
        "termination_if_roll_greater_than": 180 if "jump" in exp_name else 10,  # degree
        "termination_if_pitch_greater_than": 180 if "jump" in exp_name else 10,
        # base pose
        "base_init_pos": [0.0, 0.0, 0.45],
        "base_init_quat": [1.0, 0.0, 0.0, 0.0],
        "random_move_z": [0, 0.3] if exp_name == "walking_only" else None,
        "episode_length_s": 5.0 if "jump" in exp_name else 40.0,
        "resampling_time_s": 4.0,
        "action_scale": 0.25,
        "simulate_action_latency": True,
        "clip_actions": 100.0,
    }
    obs_cfg = {
        "num_obs": 9 + 4 * len(dof_names),
        "obs_scales": {
            "lin_vel": 2.0,
            "ang_vel": 0.25,
            "dof_pos": 1.0,
            "dof_vel": 0.05,
            "dof_force": 0.1,
        },
    }
    if exp_name == "jump":
        reward_cfg = {
            "tracking_sigma": 0.1,
            "base_height_target": 0.3,
            "feet_height_target": 0.075,
            "reward_scales": {
                "tracking_jump_pd": 0.0025,
                "tracking_jump_z": 2.0,
                # "tracking_jump_action": 10.0,
                "tracking_jump_action_ang": 10.0,
                "tracking_jump_action_ang2": 3.0,
                # "tracking_jump_vel": 2.0,
                "tracking_jump_traj": 3.0,
                "action_rate": -0.002,
                # "similar_to_default": -0.1,
            },
        }
    elif exp_name == "jump_pd":
        reward_cfg = {
            "tracking_sigma": 0.1,
            "base_height_target": 0.3,
            "feet_height_target": 0.075,
            "reward_scales": {
                "tracking_jump_pd": 1.0,
            },
        }
    elif exp_name == "walking_only":
        reward_cfg = {
            "tracking_sigma": 0.25,
            "base_height_target": 0.3,
            "feet_height_target": 0.075,
            "reward_scales": {
                "tracking_lin_vel": 1.0,
                "tracking_ang_vel": 3.0,
                "lin_vel_z": -1.0,
                "base_height": -50.0,
                "action_rate": -0.005,
                "similar_to_default": -0.1,
                # "obj_moving": -0.3,
            },
        }
    else:
        reward_cfg = {
            "tracking_sigma": 0.25,
            "base_height_target": 0.3,
            "feet_height_target": 0.075,
            "reward_scales": {
                "tracking_lin_vel": 1.0,
                "tracking_ang_vel": 3.0,
                "lin_vel_z": -1.0,
                "base_height": -50.0,
                "action_rate": -0.002,
                "similar_to_default": -0.05,
                "similar_to_default_long": -5,
            },
        }
    command_cfg = {
        "num_commands": 3,
        "lin_vel_x_range": [0, 2],
        "lin_vel_y_range": [0, 0],
        "ang_vel_range": [-1, 1],
    }

    return env_cfg, obs_cfg, reward_cfg, command_cfg


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--exp_name", type=str, default="go2-walking")
    parser.add_argument("-l", "--log_dir", type=str, default="logs")
    parser.add_argument("-B", "--num_envs", type=int, default=4096)
    parser.add_argument("-p", "--param_name", type=str, default="test")
    parser.add_argument("-s", "--substeps", type=int, default=2)
    parser.add_argument("--max_iterations", type=int, default=300)
    args = parser.parse_args()

    gs.init(logging_level="warning")

    log_dir = f"{args.log_dir}/{args.exp_name}/{args.param_name}"
    env_cfg, obs_cfg, reward_cfg, command_cfg = get_cfgs(args.exp_name)
    train_cfg = get_train_cfg(args.exp_name, args.max_iterations)

    env_cfg["substeps"] = args.substeps

    if os.path.exists(log_dir):
        if input(f"{log_dir} already exists!!! are you sure? [y/N]") != "y":
            raise Exception(f"{log_dir} already exists!!!")
        shutil.rmtree(log_dir)
    os.makedirs(log_dir, exist_ok=True)

    env = Go2Env(
        num_envs=args.num_envs, env_cfg=env_cfg, obs_cfg=obs_cfg, reward_cfg=reward_cfg, command_cfg=command_cfg
    )

    runner = OnPolicyRunner(env, train_cfg, log_dir, device="cuda:0")

    pickle.dump(
        [env_cfg, obs_cfg, reward_cfg, command_cfg, train_cfg],
        open(f"{log_dir}/cfgs.pkl", "wb"),
    )

    ##### dump_cfgs_to_yaml

    all_cfgs = {
        "env_cfg": env_cfg,
        "obs_cfg": obs_cfg,
        "reward_cfg": reward_cfg,
        "command_cfg": command_cfg,
        "train_cfg": train_cfg,
    }

    def to_serializable(obj):
        if isinstance(obj, torch.Tensor):
            return obj.tolist()
        if isinstance(obj, dict):
            return {k: to_serializable(v) for k, v in obj.items()}
        if isinstance(obj, (list, tuple)):
            return [to_serializable(v) for v in obj]
        return obj

    all_cfgs = to_serializable(all_cfgs)

    with open(f"{log_dir}/cfgs.yaml", "w") as f:
        yaml.safe_dump(all_cfgs, f, sort_keys=False)

    #####

    runner.learn(num_learning_iterations=args.max_iterations, init_at_random_ep_len=True)

    ##### dump_training_data をここにコピペするとなぜかPYOPENGL_PLATFORMを指定しているにも関わらず真っ暗になる

if __name__ == "__main__":
    main()

"""
# training
python examples/locomotion/go2_train.py
"""
