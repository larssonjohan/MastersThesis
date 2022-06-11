import yaml

def loadYaml(file):
    """
        Loads YAML
    """
    with open(file, 'r') as f:
        return yaml.safe_load(f)