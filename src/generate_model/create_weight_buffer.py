#!/usr/bin/env python3
import argparse
import json

from keras.models import load_model

def create_weight_buffer(model):
    buf = b''
    model_arch = json.loads(model.to_json())

    for ind, desc in enumerate(model_arch["config"]):
        weights = model.layers[ind].get_weights()
        buf += weights[0].tobytes() + weights[1].tobytes()

    return buf

def main():
    # parse arguments
    arg_parser = argparse.ArgumentParser(description='')
    arg_parser.add_argument('MODEL_FILE', help='')
    script_args = arg_parser.parse_args()

    path_model = script_args.MODEL_FILE
    model = load_model(path_model)

if __name__ == '__main__':
    main()
