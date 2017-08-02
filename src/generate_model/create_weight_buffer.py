#!/usr/bin/env python3
import argparse
import json

from keras.models import load_model

def create_weight_buffer(model):
    buf = b''
    model_arch = json.loads(model.to_json())
    layer_arch = model_arch['config']['layers']

    for ind, desc in enumerate(layer_arch):
        if layer_arch[ind]['class_name'] == 'Dense':
            weights = model.layers[ind].get_weights()
            # print(weights)
            buf += weights[0].tobytes() + weights[1].tobytes()

    return buf

def main():
    # parse arguments
    arg_parser = argparse.ArgumentParser(description='')
    arg_parser.add_argument('MODEL_FILE', help='')
    arg_parser.add_argument('WEIGHT_FILE', help='')
    script_args = arg_parser.parse_args()

    path_model = script_args.MODEL_FILE
    model = load_model(path_model)

    with open(script_args.WEIGHT_FILE, 'wb+') as file_weight:
        file_weight.write(create_weight_buffer(model))

if __name__ == '__main__':
    main()
