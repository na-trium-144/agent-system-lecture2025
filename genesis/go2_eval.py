import argparse
import os
import pickle
import math
import torch
from go2_env import Go2Env
from rsl_rl.runners import OnPolicyRunner

import genesis as gs


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--exp_name", type=str, default="go2-walking")
    parser.add_argument("-l", "--log_dir", type=str, default="logs")
    parser.add_argument("-p", "--param_name", type=str, default="test")
    parser.add_argument("-c", "--ckpt", type=int, default=100)
    args = parser.parse_args()

    gs.init()

    log_dir = f"{args.log_dir}/{args.exp_name}/{args.param_name}"
    env_cfg, obs_cfg, reward_cfg, command_cfg, train_cfg = pickle.load(open(f"{log_dir}/cfgs.pkl", "rb"))
    reward_cfg["reward_scales"] = {}

    env = Go2Env(
        num_envs=1,
        env_cfg=env_cfg,
        obs_cfg=obs_cfg,
        reward_cfg=reward_cfg,
        command_cfg=command_cfg,
        show_viewer=True,
    )

    runner = OnPolicyRunner(env, train_cfg, log_dir, device="cuda:0")
    resume_path = os.path.join(log_dir, f"model_{args.ckpt}.pt")
    runner.load(resume_path)
    policy = runner.get_inference_policy(device="cuda:0")

    obs, _ = env.reset()
    with torch.no_grad():
        while True:
            actions = policy(obs)
            # print(actions)
            obs, _, rews, dones, infos = env.step(actions)
            print("command: ", env.commands)
            # print("vel:", env.base_lin_vel[:, :])
            # print("vel_rew_target:", torch.tanh(100 * env.base_lin_vel[:, 2]) * torch.sqrt(torch.sum(torch.square(env.base_lin_vel[:, :]), dim=1)))
            # print("rew_base_height:", env._reward_base_height())
            # print(env.dof_pos[0, 9:19])
            # print("rew_jump_action_ang:", env._reward_tracking_jump_action_ang())
            # print(env.dof_pos[:, 8] - env.default_dof_pos[8], env.base_euler[:, 1] / 180 * math.pi)
            # print(env.robot.get_dofs_position(env.pd_motor_dofs)[:, 0] - env.pd_default_dof_pos[0], env.base_euler[:, 1] / 180 * math.pi)


if __name__ == "__main__":
    main()

"""
# evaluation
python examples/locomotion/go2_eval.py -e go2-walking -v --ckpt 100
"""
