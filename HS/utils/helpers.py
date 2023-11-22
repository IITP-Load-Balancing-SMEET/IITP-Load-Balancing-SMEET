import yaml
import argparse


def arg_parse():
    parser = argparse.ArgumentParser(description='Generative Adversarial User Model.')
    parser.add_argument('--config_path', type=str, default="config.yaml",
                        help='Path of the configurations yaml file.')

    args = parser.parse_args()
    return args


def parse_config_yaml(file_path):
    """
    Input:
        file_path (str): path of the config yaml file
    Return:
        config_dict (dict): dictionary containing the information in the yaml file
    Given a path for the config yaml file as str, parses the parameters and returns the dictionary.
    """
    with open(file_path) as config_yaml:
        config_dict = yaml.load(config_yaml, Loader=yaml.SafeLoader)

    return config_dict