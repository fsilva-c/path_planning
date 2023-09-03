import numpy as np
import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import scipy.stats as stats

SCENARIOS = {
    1: {'minz': 0.0, 'maxz': 6.0},
    2: {'minz': 0.0, 'maxz': 4.0},
    3: {'minz': 0.0, 'maxz': 2.5},
}
N_TREES = 500
tree = ET.parse('../../worlds/forest.world')
root = tree.getroot()

def distribution(scenario, mean, sigma):
    minz = SCENARIOS.get(scenario).get('minz')
    maxz = SCENARIOS.get(scenario).get('maxz')
    heights = np.random.normal(mean, sigma, N_TREES)
    heights = np.clip(heights, minz, maxz)
    return heights

def apply_distribution(scenario, sigma):
    minz = SCENARIOS.get(scenario).get('minz')
    maxz = SCENARIOS.get(scenario).get('maxz')
    mean = (maxz + minz) / 2
    dist = distribution(scenario, mean, sigma)

    for i, model in enumerate(root.findall('.//model')):
        model_name = model.get('name')
        if model_name.startswith('tree'):
            pose_element = model.find('pose')
            if pose_element is not None:
                pose_value = pose_element.text.split()
                pose_value[2] = str(-dist[i])
                pose_element.text = ' '.join(pose_value)

    tree.write(f'../../worlds/tree_scenario_{scenario}.world')

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
