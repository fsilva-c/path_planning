import rospy
from uav.uav import UAV
from RL_path_planner.fspp_env_v0 import FSPPEnv
from stable_baselines3 import DQN
from stable_baselines3.common.vec_env.dummy_vec_env import DummyVecEnv
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import EvalCallback

uav = UAV(uav_id=1)

def start():
    rospy.init_node('rl_mission', anonymous=True)

    rospy.loginfo('Iniciando os testes...')
    rospy.sleep(5.0)

    rospy.loginfo('Aguardando carregamento dos módulos do controlador de voo...')
    # uav.movements.takeoff()
    while rospy.get_time() <= 25.0 or uav.uav_info.get_active_tracker() == 'NullTracker':
        rospy.sleep(0.1)

    rospy.loginfo('Iniciando os testes...')
    env = DummyVecEnv([lambda: Monitor(FSPPEnv())])
    model = DQN(
        'MultiInputPolicy',
        env,
        learning_rate=0.00025,
        verbose=1,
        batch_size=32,
        train_freq=4,
        target_update_interval=10000,
        learning_starts=10000,
        buffer_size=500000,
        max_grad_norm=10,
        exploration_fraction=0.1,
        exploration_final_eps=0.01,
        device='cuda',
        tensorboard_log='./tb_logs/',
    )

    callbacks = []
    eval_callback = EvalCallback(
        env,
        callback_on_new_best=None,
        n_eval_episodes=5,
        best_model_save_path=".",
        log_path=".",
        eval_freq=10000,
    )
    callbacks.append(eval_callback)

    kwargs = {}
    kwargs["callback"] = callbacks

    model.learn(
        total_timesteps=5e5,
        tb_log_name="dqn_drone_run_" + str(rospy.get_time()),
        **kwargs
    )
    model.save(f'DQN_training_model_docs')

start()
