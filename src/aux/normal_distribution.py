import rospy
import numpy as np
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState

import matplotlib.pyplot as plt
import scipy.stats as stats

SCENARIOS = {
    1: {'minz': 0.0, 'maxz': 6.0},
    2: {'minz': 0.0, 'maxz': 4.0},
    3: {'minz': 0.0, 'maxz': 2.5},
}

N_TREES = 500

def distribution(scenario, mean, sigma):
    minz = SCENARIOS.get(scenario).get('minz')
    maxz = SCENARIOS.get(scenario).get('maxz')
    heights = np.random.normal(mean, sigma, N_TREES)
    heights = np.clip(heights, minz, maxz)
    return heights

def apply_distribution(scenario, sigma):
    rospy.init_node('move_gazebo_model_z')

    get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    minz = SCENARIOS.get(scenario).get('minz')
    maxz = SCENARIOS.get(scenario).get('maxz')
    mean = (maxz + minz) / 2
    dist = distribution(scenario, mean, sigma)

    for tree in range(0, N_TREES):
        model_name = f'tree_simple{tree}'
        new_z = -dist[tree]

        model_pose = get_model_state(model_name, '').pose.position
        model_state_msg = ModelState()
        model_state_msg.model_name = model_name
        model_state_msg.pose.position.x = model_pose.x
        model_state_msg.pose.position.y = model_pose.y
        model_state_msg.pose.position.z = new_z

        try:
            set_model_state(model_state_msg)
        except rospy.ServiceException as e:
            rospy.logerr("Erro ao definir a posição Z do modelo: %s", str(e))
        rospy.sleep(0.1)

    # save plot...
    plt.hist(dist, bins=50, density=True, alpha=0.7, color='blue')
    x = np.linspace(minz, maxz, 100)
    pdf = stats.norm.pdf(x, mean, sigma)
    plt.plot(x, pdf, 'r-', label='Frequência')

    mean_height = np.mean(dist)
    plt.axvline(mean_height, color='b', linestyle='dashed', linewidth=2, label=f'Média da Altura: {mean_height:.2f}')

    plt.title(f'Distribuição da Altura das Árvores - Cenário {scenario}')
    plt.xlabel('Altura')
    plt.ylabel('Frequência')
    plt.legend()
    plt.savefig(f'images/scenario_{scenario}.png')
    plt.clf()

if __name__ == '__main__':
    # apply_distribution(1, 1.0)
    # apply_distribution(2, 1.0)
    apply_distribution(3, 0.6)
