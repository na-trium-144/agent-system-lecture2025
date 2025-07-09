import torch
import math
import genesis as gs
import random
from genesis.utils.geom import quat_to_xyz, transform_by_quat, inv_quat, transform_quat_by_quat
from os.path import join, dirname

def gs_rand_float(lower, upper, shape, device):
    return (upper - lower) * torch.rand(size=shape, device=device) + lower


class Go2Env:
    def __init__(self, num_envs, env_cfg, obs_cfg, reward_cfg, command_cfg, show_viewer=False, device="cuda"):
        self.device = torch.device(device)

        self.num_envs = num_envs
        self.num_obs = obs_cfg["num_obs"]
        self.num_privileged_obs = None
        self.num_actions = env_cfg["num_actions"]
        self.num_commands = command_cfg["num_commands"]

        self.simulate_action_latency = True  # there is a 1 step latency on real robot
        self.dt = 0.02  # control frequency on real robot is 50hz
        self.max_episode_length = math.ceil(env_cfg["episode_length_s"] / self.dt)

        self.env_cfg = env_cfg
        self.obs_cfg = obs_cfg
        self.reward_cfg = reward_cfg
        self.command_cfg = command_cfg

        self.obs_scales = obs_cfg["obs_scales"]
        self.reward_scales = reward_cfg["reward_scales"]

        # create scene
        self.scene = gs.Scene(
            sim_options=gs.options.SimOptions(dt=self.dt, substeps=env_cfg["substeps"]),
            viewer_options=gs.options.ViewerOptions(
                max_FPS=int(0.5 / self.dt),
                camera_pos=(1.0, 4.0, 0.3),
                camera_lookat=(1.0, 0.0, 0.3),
                camera_fov=40,
            ),
            vis_options=gs.options.VisOptions(n_rendered_envs=1),
            rigid_options=gs.options.RigidOptions(
                dt=self.dt,
                constraint_solver=gs.constraint_solver.Newton,
                enable_collision=True,
                enable_joint_limit=True,
            ),
            show_viewer=show_viewer,
        )

        # add ground
        self.ground = self.scene.add_entity(gs.morphs.URDF(file="urdf/plane/plane.urdf", fixed=True))

        # add robot
        self.base_init_pos = torch.tensor(self.env_cfg["base_init_pos"], device=self.device)
        self.base_init_quat = torch.tensor(self.env_cfg["base_init_quat"], device=self.device)
        self.inv_base_init_quat = inv_quat(self.base_init_quat)
        self.robot = self.scene.add_entity(
            gs.morphs.URDF(
                file=join(dirname(dirname(__file__)), "framy.urdf"),
                pos=self.base_init_pos.cpu().numpy(),
                quat=self.base_init_quat.cpu().numpy(),
            ),
        )

        if self.env_cfg.get("jump"):
            self.wall = self.scene.add_entity(gs.morphs.Box(
                pos=(2.3 + 1, 0, 1.5 - 1),
                size=(2, 2, 2),
                # fixed=True,
            ))
            self.box_pos = torch.tensor([[2.1 + random.random() * 0.4 + 1, 0, 1.3 + random.random() * 0.4 - 1]], device=self.device)
        elif self.env_cfg.get("obj", True):
            a = random.random() * 2 * math.pi
            r = random.random() * 1 + 0.7
            self.box_pos = torch.tensor([[r * math.cos(a), r * math.sin(a), 0.02]], device=self.device)
            self.obj = self.scene.add_entity(gs.morphs.Box(
                pos=tuple(self.box_pos[0, :].detach().cpu()),
                size=(1, 1, 0.04),
                # fixed=True,
            ))

        # build
        self.scene.build(n_envs=num_envs)

        # names to indices
        self.motor_dofs = [self.robot.get_joint(name).dof_idx_local for name in self.env_cfg["dof_names"]]
        self.pd_motor_dofs = [self.robot.get_joint(name).dof_idx_local for name in self.env_cfg.get("pd_dof_names", [])]

        # PD control parameters
        # self.robot.set_dofs_kp([self.env_cfg["kp"]] * self.num_actions, self.motor_dofs)
        # self.robot.set_dofs_kv([self.env_cfg["kd"]] * self.num_actions, self.motor_dofs)
        self.output_factor = 1

        # prepare reward functions and multiply reward scales by dt
        self.reward_functions, self.episode_sums = dict(), dict()
        for name in self.reward_scales.keys():
            self.reward_scales[name] *= self.dt
            self.reward_functions[name] = getattr(self, "_reward_" + name)
            self.episode_sums[name] = torch.zeros((self.num_envs,), device=self.device, dtype=gs.tc_float)

        # initialize buffers
        self.base_lin_vel = torch.zeros((self.num_envs, 3), device=self.device, dtype=gs.tc_float)
        self.base_ang_vel = torch.zeros((self.num_envs, 3), device=self.device, dtype=gs.tc_float)
        self.projected_gravity = torch.zeros((self.num_envs, 3), device=self.device, dtype=gs.tc_float)
        self.global_gravity = torch.tensor([0.0, 0.0, -1.0], device=self.device, dtype=gs.tc_float).repeat(
            self.num_envs, 1
        )
        self.obs_buf = torch.zeros((self.num_envs, self.num_obs), device=self.device, dtype=gs.tc_float)
        self.rew_buf = torch.zeros((self.num_envs,), device=self.device, dtype=gs.tc_float)
        self.reset_buf = torch.ones((self.num_envs,), device=self.device, dtype=gs.tc_int)
        self.episode_length_buf = torch.zeros((self.num_envs,), device=self.device, dtype=gs.tc_int)
        self.commands = torch.zeros((self.num_envs, self.num_commands), device=self.device, dtype=gs.tc_float)
        self.commands_scale = torch.tensor(
            [self.obs_scales["lin_vel"], self.obs_scales["lin_vel"], self.obs_scales["ang_vel"]],
            device=self.device,
            dtype=gs.tc_float,
        )
        self.actions = torch.zeros((self.num_envs, self.num_actions), device=self.device, dtype=gs.tc_float)
        self.last_actions = torch.zeros_like(self.actions)
        self.dof_pos = torch.zeros_like(self.actions)
        self.dof_vel = torch.zeros_like(self.actions)
        self.dof_force = torch.zeros_like(self.actions)
        self.last_dof_vel = torch.zeros_like(self.actions)
        self.base_pos = torch.zeros((self.num_envs, 3), device=self.device, dtype=gs.tc_float)
        self.base_quat = torch.zeros((self.num_envs, 4), device=self.device, dtype=gs.tc_float)
        self.base_euler = torch.zeros((self.num_envs, 3), device=self.device, dtype=gs.tc_float)
        self.default_dof_pos = torch.tensor(
            [self.env_cfg["default_joint_angles"][name] for name in self.env_cfg["dof_names"]],
            device=self.device,
            dtype=gs.tc_float,
        )
        self.pd_default_dof_pos = torch.tensor(
            [self.env_cfg["pd_default_joint_angles"][name] for name in self.env_cfg.get("pd_dof_names", [])],
            device=self.device,
            dtype=gs.tc_float,
        )
        self.extras = dict()  # extra information for logging

    def _resample_commands(self, envs_idx):
        # 回転のコマンドが大きい場合は並進のコマンドは小さく
        self.commands[envs_idx, 2] = gs_rand_float(*self.command_cfg["ang_vel_range"], (len(envs_idx),), self.device)
        self.commands[envs_idx, 0] = gs_rand_float(*self.command_cfg["lin_vel_x_range"], (len(envs_idx),), self.device) / (1 + 10 * self.commands[envs_idx, 2].abs())
        self.commands[envs_idx, 1] = gs_rand_float(*self.command_cfg["lin_vel_y_range"], (len(envs_idx),), self.device) / (1 + 10 * self.commands[envs_idx, 2].abs())

    def step(self, actions):
        self.actions = torch.clip(actions, -self.env_cfg["clip_actions"], self.env_cfg["clip_actions"])
        exec_actions = self.last_actions if self.simulate_action_latency else self.actions
        # target_dof_pos = exec_actions * self.env_cfg["action_scale"] + self.default_dof_pos
        # self.robot.control_dofs_position(target_dof_pos, self.motor_dofs)
        self.robot.control_dofs_force(exec_actions * self.output_factor, self.motor_dofs)
        # p = 70
        # d = 10
        # u = p * (0 - self.dof_pos[:, :]) + d * (0 - self.dof_vel[:, :])
        # self.robot.control_dofs_force(u, self.motor_dofs)
        if random.random() < 0.001 and self.env_cfg["random_move_z"]:
            random_pos = self.robot.get_pos()[...]
            random_pos[:, 2] += gs_rand_float(*self.env_cfg["random_move_z"], (len(random_pos),), self.device)
            self.robot.set_pos(random_pos, zero_velocity=False)
        if self.env_cfg.get("walking_only"):
            self.robot.control_dofs_position(self.pd_default_dof_pos.repeat(len(self.robot.get_pos()), 1), self.pd_motor_dofs)
        if self.env_cfg.get("jump"):
            self.wall.set_pos(self.box_pos.repeat(len(self.wall.get_pos()), 1), zero_velocity=True)
            self.wall.set_quat(torch.tensor([1, 0, 0, 0], device=self.device).repeat(len(self.wall.get_pos()), 1), zero_velocity=True)

        self.scene.step()

        if self.env_cfg.get("jump"):
            self.wall.set_pos(self.box_pos.repeat(len(self.wall.get_pos()), 1), zero_velocity=True)
            self.wall.set_quat(torch.tensor([1, 0, 0, 0], device=self.device).repeat(len(self.wall.get_pos()), 1), zero_velocity=True)

        # update buffers
        self.episode_length_buf += 1
        self.base_pos[:] = self.robot.get_pos()
        self.base_quat[:] = self.robot.get_quat()
        self.base_euler = quat_to_xyz(
            transform_quat_by_quat(torch.ones_like(self.base_quat) * self.inv_base_init_quat, self.base_quat)
        )
        inv_base_quat = inv_quat(self.base_quat)
        self.base_lin_vel[:] = transform_by_quat(self.robot.get_vel(), inv_base_quat)
        self.base_ang_vel[:] = transform_by_quat(self.robot.get_ang(), inv_base_quat)
        self.projected_gravity = transform_by_quat(self.global_gravity, inv_base_quat)
        self.dof_pos[:] = self.robot.get_dofs_position(self.motor_dofs)
        self.dof_vel[:] = self.robot.get_dofs_velocity(self.motor_dofs)
        self.dof_force[:] = self.robot.get_dofs_force(self.motor_dofs)

        # resample commands
        envs_idx = (
            (self.episode_length_buf % int(self.env_cfg["resampling_time_s"] / self.dt) == 0)
            .nonzero(as_tuple=False)
            .flatten()
        )
        self._resample_commands(envs_idx)

        # check termination and reset
        self.reset_buf = self.episode_length_buf > self.max_episode_length
        self.reset_buf |= torch.abs(self.base_euler[:, 1]) > self.env_cfg["termination_if_pitch_greater_than"]
        self.reset_buf |= torch.abs(self.base_euler[:, 0]) > self.env_cfg["termination_if_roll_greater_than"]
        if self.env_cfg.get("jump"):
            self.reset_buf |= self.base_pos[:, 2] < 0.3
        self.reset_buf |= torch.isnan(self.base_pos).any(dim=1)
        self.reset_buf |= (torch.abs(self.base_pos) > 100).any(dim=1)
        self.reset_buf |= torch.isnan(self.dof_pos).any(dim=1)
        self.reset_buf |= (torch.abs(self.dof_pos) > 4 * math.pi).any(dim=1)
        self.reset_buf |= torch.isnan(self.dof_vel).any(dim=1)
        self.reset_buf |= (torch.abs(self.dof_vel) > 4 * math.pi / 0.001).any(dim=1)

        time_out_idx = (self.episode_length_buf > self.max_episode_length).nonzero(as_tuple=False).flatten()
        self.extras["time_outs"] = torch.zeros_like(self.reset_buf, device=self.device, dtype=gs.tc_float)
        self.extras["time_outs"][time_out_idx] = 1.0

        self.reset_idx(self.reset_buf.nonzero(as_tuple=False).flatten())

        # compute reward
        self.rew_buf[:] = 0.0
        for name, reward_func in self.reward_functions.items():
            rew = reward_func() * self.reward_scales[name]
            self.rew_buf += rew
            self.episode_sums[name] += rew

        # compute observations
        self.obs_buf = torch.cat(
            [
                self.base_ang_vel * self.obs_scales["ang_vel"],  # 3
                self.projected_gravity,  # 3
                self.commands * self.commands_scale,  # 3
                (self.dof_pos - self.default_dof_pos) * self.obs_scales["dof_pos"],  # 12
                self.dof_vel * self.obs_scales["dof_vel"],  # 12
                self.dof_force * self.obs_scales["dof_force"],  # 12
                self.actions,  # 12
            ],
            axis=-1,
        )
        if self.obs_buf.isnan().any():
            raise Exception(f"nan in obs, at col {torch.nonzero(self.obs_buf.isnan().any(dim=0), as_tuple=True)}")
        if self.rew_buf.isnan().any():
            for name, reward_func in self.reward_functions.items():
                rew = reward_func() * self.reward_scales[name]
                if rew.isnan().any():
                    raise Exception(f"nan in rew {name}")
        if self.reset_buf.isnan().any():
            raise Exception(f"nan in reset")
        if self.actions.isnan().any():
            raise Exception(f"nan in actions")

        self.last_actions[:] = self.actions[:]
        self.last_dof_vel[:] = self.dof_vel[:]

        return self.obs_buf, None, self.rew_buf, self.reset_buf, self.extras

    def get_observations(self):
        return self.obs_buf

    def get_privileged_observations(self):
        return None

    def reset_idx(self, envs_idx):
        if len(envs_idx) == 0:
            return

        # reset dofs
        self.dof_pos[envs_idx] = self.default_dof_pos
        self.dof_vel[envs_idx] = 0.0
        if self.env_cfg.get("jump"):
            self.dof_pos[envs_idx, :8] += gs_rand_float(-0.2, 0.2, (len(envs_idx), 8), self.device)
            self.dof_vel[envs_idx, :8] += gs_rand_float(-0.2, 0.2, (len(envs_idx), 8), self.device)
        self.dof_force[envs_idx] = 0.0
        self.robot.set_dofs_position(
            position=self.dof_pos[envs_idx],
            dofs_idx_local=self.motor_dofs,
            zero_velocity=True,
            envs_idx=envs_idx,
        )
        if len(self.pd_motor_dofs):
            self.robot.set_dofs_position(
                position=self.pd_default_dof_pos.repeat(len(envs_idx), 1),
                dofs_idx_local=self.pd_motor_dofs,
                zero_velocity=True,
                envs_idx=envs_idx,
            )

        # reset base
        self.base_pos[envs_idx] = self.base_init_pos
        self.base_quat[envs_idx] = self.base_init_quat.reshape(1, -1)
        self.base_euler[envs_idx] = 0
        self.robot.set_pos(self.base_pos[envs_idx], zero_velocity=False, envs_idx=envs_idx)
        self.robot.set_quat(self.base_quat[envs_idx], zero_velocity=False, envs_idx=envs_idx)
        self.base_lin_vel[envs_idx] = 0
        self.base_ang_vel[envs_idx] = 0
        self.robot.zero_all_dofs_velocity(envs_idx)
        self.projected_gravity[envs_idx] = 0

        if self.env_cfg.get("jump"):
            self.box_pos = torch.tensor([[2.1 + random.random() * 0.4 + 1, 0, 1.3 + random.random() * 0.4 - 1]], device=self.device)
        elif self.env_cfg.get("obj", True):
            a = random.random() * 2 * math.pi
            r = random.random() * 1 + 0.7
            self.box_pos = torch.tensor([[r * math.cos(a), r * math.sin(a), 0.02]], device=self.device)
            self.obj.set_pos(self.box_pos.repeat(len(self.obj.get_pos()), 1), zero_velocity=True)
            self.obj.set_quat(torch.tensor([1, 0, 0, 0], device=self.device).repeat(len(self.obj.get_pos()), 1), zero_velocity=True)

        # reset buffers
        self.last_actions[envs_idx] = 0.0
        self.last_dof_vel[envs_idx] = 0.0
        self.episode_length_buf[envs_idx] = 0
        self.reset_buf[envs_idx] = True

        # randomization
        self.randomize_friction()
        self.randomize_pd_gains()
        self.randomize_armature()

        # fill extras
        self.extras["episode"] = {}
        for key in self.episode_sums.keys():
            self.extras["episode"]["rew_" + key] = (
                torch.mean(self.episode_sums[key][envs_idx]).item() / self.env_cfg["episode_length_s"]
            )
            self.episode_sums[key][envs_idx] = 0.0

        self._resample_commands(envs_idx)

    def reset(self):
        self.reset_buf[:] = True
        self.reset_idx(torch.arange(self.num_envs, device=self.device))
        return self.obs_buf, None

    # ------------ reward functions----------------
    def _reward_tracking_lin_vel(self):
        # Tracking of linear velocity commands (xy axes)
        lin_vel_error = torch.sum(torch.square(self.commands[:, :2] - self.base_lin_vel[:, :2]), dim=1)
        return torch.exp(-lin_vel_error / self.reward_cfg["tracking_sigma"])

    def _reward_tracking_ang_vel(self):
        # Tracking of angular velocity commands (yaw)
        ang_vel_error = torch.square(self.commands[:, 2] - self.base_ang_vel[:, 2])
        return torch.exp(-ang_vel_error / self.reward_cfg["tracking_sigma"])

    def _reward_tracking_jump_z(self):
        lin_vel_error = torch.square(1 - self.base_lin_vel[:, 2]) * (self.base_pos[:, 2] < 1)
        return torch.exp(-lin_vel_error / self.reward_cfg["tracking_sigma"])

    def _reward_tracking_jump_action(self):
        long_dofs = self.dof_pos[:, 8:] - self.default_dof_pos[8:]
        if long_dofs.isnan().any():
            print(f"nan in long_dofs")
            long_dofs = torch.nan_to_num(long_dofs)
        long_actions = self.dof_vel[:, 8:]
        if long_actions.isnan().any():
            print(f"nan in long_actions")
            long_actions = torch.nan_to_num(long_actions)
        angle = torch.zeros((len(long_dofs),), device=self.device)
        action_jump = torch.zeros((len(long_dofs),), device=self.device)
        for i in range(12):
            angle += long_dofs[:, i]
            if angle.isnan().any():
                print(f"nan in angle")
                angle = torch.nan_to_num(angle)
            # iが偶数: angle負が下方向 * action負が下方向
            # iが奇数: angle正が下方向 * action正が下方向
            action_jump += torch.tanh(math.pi / 2 - angle * (-1 if i % 2 == 0 else 1)) * long_actions[:, i] * (-1 if i % 2 == 0 else 1)
            if action_jump.isnan().any():
                r = torch.nonzero(action_jump.isnan())
                print(f"nan in action_jump {r}, angle={angle[r]}, long_actions={long_actions[r, i]}")
                action_jump = torch.nan_to_num(action_jump)
        ret = action_jump * (self.base_pos[:, 2] < 1)
        if ret.isnan().any():
            r = torch.nonzero(ret.isnan())
            print(f"nan in return value of action_jump {r}, {ret[r]}, action_jump={action_jump[r]}, base_pos={self.base_pos[r, 2]}")
            ret = torch.nan_to_num(ret)
        return ret

    def _reward_tracking_jump_action_ang(self):
        long_dofs = self.dof_pos[:, 9:19]
        long_dofs_normalized = long_dofs * torch.tensor([1, -1, 1, -1, 1, -1, 1, -1, 1, -1], device=self.device)
        return torch.exp(-torch.var(long_dofs_normalized, dim=1)) - 1

    def _reward_tracking_jump_action_ang2(self):
        angle = self.dof_pos[:, 8] - self.default_dof_pos[8] + (self.base_euler[:, 1] / 180 * math.pi)
        return (torch.exp(-torch.abs(angle) / 0.1) - 1) * (self.base_pos[:, 2] < 1)

    def _reward_tracking_jump_pd(self):
        p = 70
        d = 10
        u = p * (0 - self.dof_pos[:, 8:]) + d * (0 - self.dof_vel[:, 8:])
        return -torch.sum(torch.abs(self.actions[:, 8:] * self.output_factor - u), dim=1) * (self.base_pos[:, 2] < 0.7) + 3200

    def _reward_tracking_jump_vel(self):
        lin_vel_error = torch.square(0.5 - torch.tanh(100 * self.base_lin_vel[:, 2]) * torch.sqrt(torch.sum(torch.square(self.base_lin_vel[:, :]), dim=1)))
        return torch.exp(-lin_vel_error / self.reward_cfg["tracking_sigma"]) - 1

    def _reward_tracking_jump_traj(self):
        wall_origin = self.wall.get_pos().detach()
        wall_origin[:, 0] -= 1
        wall_origin[:, 2] -= 1
        radius_error = torch.square(2.0 - torch.sqrt(torch.sum(torch.square(self.base_pos[:, :] - wall_origin), dim=1)))
        return torch.exp(-radius_error / 0.2) * (self.base_pos[:, 2] > 1) # - 1

    def _reward_lin_vel_z(self):
        # Penalize z axis base linear velocity
        return torch.square(self.base_lin_vel[:, 2])

    def _reward_action_rate(self):
        # Penalize changes in actions
        return torch.sum(torch.square(self.last_actions - self.actions), dim=1)

    def _reward_obj_moving(self):
        return torch.sum(torch.square(self.obj.get_pos() - self.box_pos), dim=1)

    def _reward_similar_to_default(self):
        # Penalize joint poses far away from default pose
        return torch.sum(torch.abs(self.dof_pos[:, :8] - self.default_dof_pos[:8]), dim=1)

    def _reward_similar_to_default_long(self):
        # Penalize joint poses far away from default pose
        return torch.sum(torch.abs(self.dof_pos[:, 8:] - self.default_dof_pos[8:]), dim=1)

    def _reward_base_height(self):
        # Penalize base height lower than target
        return torch.square(self.base_pos[:, 2] - self.reward_cfg["base_height_target"]) * (self.base_pos[:, 2] < self.reward_cfg["base_height_target"])

    # ------------ randomization ----------------
    def randomize_link_properties(self):
        # scale mass of links
        mass_scale = 0.9 + 0.2 * torch.rand((self.robot.n_links,), device=self.device)  # 0.9〜1.1
        self.robot.set_links_inertial_mass(mass_scale)

    def randomize_com_shift(self):
        # shift COM positions
        num_links = self.robot.n_links
        link_indices = list(range(num_links))
        com_shift = 0.01 * torch.randn((self.num_envs, num_links, 3), device=self.device)  # +-0.01m=+-10mm
        self.robot.set_COM_shift(com_shift, link_indices)

    def randomize_friction(self):
        # frictions between the ground and robots
        # friction = 0.5 + torch.rand(1).item() # 0.5~1.5
        friction = 0.2 + 1.6*torch.rand(1).item() # 0.2~1.8

        self.robot.set_friction(friction)
        self.ground.set_friction(friction)

    def randomize_pd_gains(self):
        # pd gains of the joint control
        # num_dofs = self.robot.n_dofs
        # kp_min, kp_max = 5.0, 40.0
        # kv_min, kv_max = 0.2, 1.5
        # kp = torch.rand(num_dofs, device=self.device) * (kp_max - kp_min) + kp_min
        # kv = torch.rand(num_dofs, device=self.device) * (kv_max - kv_min) + kv_min
        # self.robot.set_dofs_kp(kp)
        # self.robot.set_dofs_kv(kv)
        self.output_factor = random.random() * 2 + 0.1
        # self.output_factor = math.exp(random.random() * 5 - 3)

    def randomize_armature(self):
        # joint's rotor inertia
        armature_min, armature_max = 0.01, 0.15
        armature = torch.rand(self.robot.n_dofs, device=self.device) * (armature_max - armature_min) + armature_min
        self.robot.set_dofs_armature(armature)
