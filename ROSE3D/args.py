import argparse

def fetchArguments():
    parser = argparse.ArgumentParser(description="Run ROSE on pointcloud stored as PCD file")
    
    # Requirements
    parser.add_argument('-f', '--input_file', help='File to declutter using ROSE', required=True)
    parser.add_argument('-t', '--threshold', type=float, help='Threshold for ground removal', required=True)

    # Additionals
    parser.add_argument('-s', '--json_schema_file', help='JSON schema file', required=False)
    parser.add_argument('-c', '--json_config_file', help='JSON configuration file', required=False)

    args = parser.parse_args()
    
    return args.input_file, args.threshold, args.json_schema_file, args.json_config_file